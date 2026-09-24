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

        this->declare_parameter<double>("r_ref", 2000.0);
        this->declare_parameter<double>("r_weight", 3.5);
        this->declare_parameter<double>("theta_weight", 1.0);
        this->declare_parameter<double>("phi_weight", 1.0);

        this->declare_parameter<double>("cluster_eps", 8.0);
        this->declare_parameter<int>("cluster_min_samples", 5);
        this->declare_parameter<int>("cluster_max_samples", 3000);
        this->declare_parameter<double>("max_cluster_extent", 150.0);
        this->declare_parameter<bool>("enable_range_balancing", true);

        auto sub_topic = this->get_parameter("subscribe_topic").as_string();
        auto pub_topic = this->get_parameter("publish_topic").as_string();

        r_ref_ = this->get_parameter("r_ref").as_double();
        r_weight_ = this->get_parameter("r_weight").as_double();
        theta_weight_ = this->get_parameter("theta_weight").as_double();
        phi_weight_ = this->get_parameter("phi_weight").as_double();
        cluster_eps_ = this->get_parameter("cluster_eps").as_double();
        cluster_min_samples_ = this->get_parameter("cluster_min_samples").as_int();
        cluster_max_samples_ = this->get_parameter("cluster_max_samples").as_int();
        max_cluster_extent_ = this->get_parameter("max_cluster_extent").as_double();
        enable_range_balancing_ = this->get_parameter("enable_range_balancing").as_bool();

        // 2. 创建订阅者与发布者
        subscription_ = this->create_subscription<pc_msgs::msg::O3DPointCloud>(
            sub_topic, 10, std::bind(&ParallelDbscanNode::pointcloud_callback, this, _1)
        );

        publisher_ = this->create_publisher<pc_msgs::msg::ClusteredPointCloud>(pub_topic, 10);

        RCLCPP_INFO(this->get_logger(), "[*] Clustering node brought up. Max points cap: %d, Max extent: %.1fm",
                    cluster_max_samples_, max_cluster_extent_);
    }

private:
    void pointcloud_callback(const pc_msgs::msg::O3DPointCloud::SharedPtr msg) const {
        // 1. 检查输入
        if (msg->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "[?] Received no points message. Scaped clustering.");
            return;
        }

        size_t num_points = msg->points.size() / 3;

        // 2 & 3. 特征工程：极坐标特征物理量纲对齐与距离密度平衡
        auto feature_pcd = std::make_shared<open3d::geometry::PointCloud>();
        feature_pcd->points_.reserve(num_points);

        for (size_t i = 0; i < num_points; ++i) {
            double x = msg->points[i * 3 + 0];
            double y = msg->points[i * 3 + 1];
            double z = msg->points[i * 3 + 2];

            double r = std::hypot(x, y, z);
            double theta = std::atan2(y, x);
            double r_safe = std::max(r, 1e-6);
            double phi = std::asin(z / r_safe);

            // 距离密度平衡缩放：在远端平衡点云稀疏度，在近端拉开特征间距
            double effective_r = enable_range_balancing_ ? std::sqrt(r_safe * r_ref_) : r_safe;

            double r_meter = r;
            double theta_meter = effective_r * theta;
            double phi_meter = effective_r * phi;

            // 存入虚拟点云
            feature_pcd->points_.emplace_back(
                r_meter * r_weight_,
                theta_meter * theta_weight_,
                phi_meter * phi_weight_
            );
        }

        // 4. 执行 DBSCAN 聚类
        std::vector<int> labels;
        try {
            labels = feature_pcd->ClusterDBSCAN(cluster_eps_, cluster_min_samples_, false);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "[!] Failed to clustering: %s", e.what());
            return;
        }

        // 4.5 过滤超标巨型团 (基于最大点数 cluster_max_samples_ 与最大外接盒尺寸 max_cluster_extent_)
        if (!labels.empty() && (cluster_max_samples_ > 0 || max_cluster_extent_ > 0.0)) {
            std::vector<size_t> cluster_counts;
            std::vector<double> min_x, max_x, min_y, max_y, min_z, max_z;

            int current_max = *std::max_element(labels.begin(), labels.end());
            if (current_max >= 0) {
                size_t num_labels = current_max + 1;
                cluster_counts.resize(num_labels, 0);
                min_x.resize(num_labels, 1e9); max_x.resize(num_labels, -1e9);
                min_y.resize(num_labels, 1e9); max_y.resize(num_labels, -1e9);
                min_z.resize(num_labels, 1e9); max_z.resize(num_labels, -1e9);

                for (size_t i = 0; i < num_points; ++i) {
                    int label = labels[i];
                    if (label < 0) continue;

                    cluster_counts[label]++;

                    double x = msg->points[i * 3 + 0];
                    double y = msg->points[i * 3 + 1];
                    double z = msg->points[i * 3 + 2];

                    min_x[label] = std::min(min_x[label], x);
                    max_x[label] = std::max(max_x[label], x);
                    min_y[label] = std::min(min_y[label], y);
                    max_y[label] = std::max(max_y[label], y);
                    min_z[label] = std::min(min_z[label], z);
                    max_z[label] = std::max(max_z[label], z);
                }

                // 标记超标类别
                std::vector<bool> is_invalid(num_labels, false);
                for (size_t l = 0; l < num_labels; ++l) {
                    if (cluster_counts[l] == 0) continue;

                    if (cluster_max_samples_ > 0 && cluster_counts[l] > static_cast<size_t>(cluster_max_samples_)) {
                        is_invalid[l] = true;
                        continue;
                    }

                    if (max_cluster_extent_ > 0.0) {
                        double dx = max_x[l] - min_x[l];
                        double dy = max_y[l] - min_y[l];
                        double dz = max_z[l] - min_z[l];
                        double max_extent = std::max({dx, dy, dz});
                        if (max_extent > max_cluster_extent_) {
                            is_invalid[l] = true;
                        }
                    }
                }

                // 将超标团的点重置为 -1 噪声
                for (size_t i = 0; i < labels.size(); ++i) {
                    int l = labels[i];
                    if (l >= 0 && is_invalid[l]) {
                        labels[i] = -1;
                    }
                }
            }
        }

        // 获取最大类标签
        int max_label = -1;
        if (!labels.empty()) {
            max_label = *std::max_element(labels.begin(), labels.end());
        }

        // 5. 构建并发布带有聚类标签的自定义消息
        auto out_msg = pc_msgs::msg::ClusteredPointCloud();
        out_msg.header = msg->header;
        out_msg.points = msg->points;
        out_msg.labels.assign(labels.begin(), labels.end());
        out_msg.max_label = max_label;

        publisher_->publish(out_msg);

        int num_clusters = max_label >= 0 ? max_label + 1 : 0;
        RCLCPP_INFO(this->get_logger(), "[*] Clustering finished: Input points %zu, Recover %d valid clusters.", num_points, num_clusters);
    }

    double r_ref_;
    double r_weight_;
    double theta_weight_;
    double phi_weight_;
    double cluster_eps_;
    int cluster_min_samples_;
    int cluster_max_samples_;
    double max_cluster_extent_;
    bool enable_range_balancing_;

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
