#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "pc_msgs/msg/o3_d_point_cloud.hpp"
#include "pc_msgs/msg/clustered_point_cloud.hpp"

// 引入 Open3D 核心头文件
#include "open3d/Open3D.h"

using std::placeholders::_1;

class ParallelDbscanNode : public rclcpp::Node {
public:
    ParallelDbscanNode() : Node("parallel_dbscan_node") {
        // 1. 声明并读取参数
        this->declare_parameter<std::string>("subscribe_topic", "/gs/non_ground_pointcloud");
        this->declare_parameter<std::string>("publish_topic", "/clustering/clustered_pointcloud");

        this->declare_parameter<double>("r_weight", 3.0);
        this->declare_parameter<double>("theta_weight", 650.0);
        this->declare_parameter<double>("phi_weight", 600.0);

        this->declare_parameter<double>("cluster_eps", 18.0);
        this->declare_parameter<int>("cluster_min_samples", 5);

        auto sub_topic = this->get_parameter("subscribe_topic").as_string();
        auto pub_topic = this->get_parameter("publish_topic").as_string();

        r_weight_ = this->get_parameter("r_weight").as_double();
        theta_weight_ = this->get_parameter("theta_weight").as_double();
        phi_weight_ = this->get_parameter("phi_weight").as_double();
        cluster_eps_ = this->get_parameter("cluster_eps").as_double();
        cluster_min_samples_ = this->get_parameter("cluster_min_samples").as_int();

        // 2. 创建订阅者与发布者
        subscription_ = this->create_subscription<pc_msgs::msg::O3DPointCloud>(
            sub_topic, 10, std::bind(&ParallelDbscanNode::pointcloud_callback, this, _1)
        );

        publisher_ = this->create_publisher<pc_msgs::msg::ClusteredPointCloud>(pub_topic, 10);

        RCLCPP_INFO(this->get_logger(), "[*] Clustering node has been brought up. Listening on: %s", sub_topic.c_str());
    }

private:
    void pointcloud_callback(const pc_msgs::msg::O3DPointCloud::SharedPtr msg) const {
        // 1. 检查输入
        if (msg->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "[?] Received no points message. Scaped clustering.");
            return;
        }

        size_t num_points = msg->points.size() / 3;

        // 2 & 3. 特征工程：将笛卡尔坐标转换为加权极坐标空间
        // 我们创建一个 "虚拟点云"，将其 X, Y, Z 替换为加权后的 r, theta, phi 特征
        auto feature_pcd = std::make_shared<open3d::geometry::PointCloud>();
        feature_pcd->points_.reserve(num_points);

        for (size_t i = 0; i < num_points; ++i) {
            double x = msg->points[i * 3 + 0];
            double y = msg->points[i * 3 + 1];
            double z = msg->points[i * 3 + 2];

            double r = std::hypot(x, y, z); // std::hypot 计算 sqrt(x^2 + y^2 + z^2) 且防止溢出
            double theta = std::atan2(y, x);
            double r_safe = std::max(r, 1e-6);
            double phi = std::asin(z / r_safe);

            // 存入虚拟点云
            feature_pcd->points_.emplace_back(
                r * r_weight_,
                theta * theta_weight_,
                phi * phi_weight_
            );
        }

        // 4. 执行 DBSCAN 聚类
        // C++ API: ClusterDBSCAN(eps, min_points, print_progress)
        std::vector<int> labels;
        try {
            labels = feature_pcd->ClusterDBSCAN(cluster_eps_, cluster_min_samples_, false);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "[!] Failed to clustering: %s", e.what());
            return;
        }

        // 获取最大类标签
        int max_label = -1;
        if (!labels.empty()) {
            max_label = *std::max_element(labels.begin(), labels.end());
        }

        // 5. 构建并发布带有聚类标签的自定义消息
        auto out_msg = pc_msgs::msg::ClusteredPointCloud();
        out_msg.header = msg->header;

        // 直接复用输入消息中已展平的点数据，避免重新内存分配拷贝
        out_msg.points = msg->points;

        // 分配标签并设值
        out_msg.labels.assign(labels.begin(), labels.end());
        out_msg.max_label = max_label;

        publisher_->publish(out_msg);

        int num_clusters = max_label >= 0 ? max_label + 1 : 0;
        RCLCPP_INFO(this->get_logger(), "[*] Clustering finished: Input points %zu, Recover %d clusters.", num_points, num_clusters);
    }

    double r_weight_;
    double theta_weight_;
    double phi_weight_;
    double cluster_eps_;
    int cluster_min_samples_;

    rclcpp::Subscription<pc_msgs::msg::O3DPointCloud>::SharedPtr subscription_;
    rclcpp::Publisher<pc_msgs::msg::ClusteredPointCloud>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParallelDbscanNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
