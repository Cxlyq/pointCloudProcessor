//
// Created by cx on 6/22/26.
//
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "pc_msgs/msg/o3_d_point_cloud.hpp"

// 引入 Open3D 核心几何体和点云处理头文件
#include "open3d/Open3D.h"

using std::placeholders::_1;

class RansacNode : public rclcpp::Node {
public:
    RansacNode() : Node("ransac_node") {
        // 1. 声明并读取参数
        this->declare_parameter<std::string>("subscribe_topic", "/io/raw_pointcloud");
        this->declare_parameter<std::string>("publish_ground_topic", "/gs/ground_pointcloud");
        this->declare_parameter<std::string>("publish_non_ground_topic", "/gs/non_ground_pointcloud");

        this->declare_parameter<double>("distance_threshold", 13.0);
        this->declare_parameter<int>("ransac_n", 3);
        this->declare_parameter<int>("num_iterations", 100);

        auto sub_topic = this->get_parameter("subscribe_topic").as_string();
        auto pub_g_topic = this->get_parameter("publish_ground_topic").as_string();
        auto pub_ng_topic = this->get_parameter("publish_non_ground_topic").as_string();

        dist_thresh_ = this->get_parameter("distance_threshold").as_double();
        ransac_n_ = this->get_parameter("ransac_n").as_int();
        num_iters_ = this->get_parameter("num_iterations").as_int();

        // 2. 创建订阅者与发布者
        subscription_ = this->create_subscription<pc_msgs::msg::O3DPointCloud>(
            sub_topic, 10, std::bind(&RansacNode::pointcloud_callback, this, _1)
        );

        ground_pub_ = this->create_publisher<pc_msgs::msg::O3DPointCloud>(pub_g_topic, 10);
        non_ground_pub_ = this->create_publisher<pc_msgs::msg::O3DPointCloud>(pub_ng_topic, 10);

        RCLCPP_INFO(this->get_logger(), "[*] RANSAC node has been brought up. Listening on: %s...", sub_topic.c_str());
    }

private:
    void pointcloud_callback(const pc_msgs::msg::O3DPointCloud::SharedPtr msg) const {
        // 1. 检查输入是否为空
        if (msg->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "[?] Scaped empty point cloud message.");
            return;
        }

        // 2. 反序列化：ROS 2 消息 (1D std::vector<float>) 转 Open3D 点云 (Eigen::Vector3d)
        auto pcd = std::make_shared<open3d::geometry::PointCloud>();

        size_t num_points = msg->points.size() / 3;
        pcd->points_.reserve(num_points);

        for (size_t i = 0; i < num_points; ++i) {
            // 注意类型转换：float -> double
            pcd->points_.emplace_back(
                static_cast<double>(msg->points[i * 3 + 0]),
                static_cast<double>(msg->points[i * 3 + 1]),
                static_cast<double>(msg->points[i * 3 + 2])
            );
        }

        // 如果需要处理颜色，同理进行反序列化
        if (!msg->colors.empty() && msg->colors.size() == msg->points.size()) {
            pcd->colors_.reserve(num_points);
            for (size_t i = 0; i < num_points; ++i) {
                pcd->colors_.emplace_back(
                    static_cast<double>(msg->colors[i * 3 + 0]),
                    static_cast<double>(msg->colors[i * 3 + 1]),
                    static_cast<double>(msg->colors[i * 3 + 2])
                );
            }
        }

        // 3. RANSAC 算法逻辑
        std::shared_ptr<open3d::geometry::PointCloud> ground_pcd;
        std::shared_ptr<open3d::geometry::PointCloud> non_ground_pcd;

        try {
            // C++ API: SegmentPlane(distance_threshold, ransac_n, num_iterations)
            // 返回值是 std::tuple<Eigen::Vector4d, std::vector<size_t>>
            // 第一项是平面参数 (a, b, c, d)，第二项是内点（地面点）的索引列表
            Eigen::Vector4d plane_model;
            std::vector<size_t> inliers;

            std::tie(plane_model, inliers) = pcd->SegmentPlane(dist_thresh_, ransac_n_, num_iters_);

            // 提取非地面点和地面点
            // C++ API SelectByIndex(indices, invert=false)
            ground_pcd = pcd->SelectByIndex(inliers);
            non_ground_pcd = pcd->SelectByIndex(inliers, true); // invert = true
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "[?] RANSAC run failed: %s. All points will be regarded as non-ground points.", e.what());
            non_ground_pcd = pcd;
            ground_pcd = std::make_shared<open3d::geometry::PointCloud>();
        }

        if (non_ground_pcd->points_.empty()) {
            RCLCPP_WARN(this->get_logger(), "[?] Empty non-ground point cloud. Scaped publish.");
            return;
        }

        // 4. 序列化：Open3D 点云转 ROS 2 消息并发布

        // 发布地面点云
        if (!ground_pcd->points_.empty()) {
            auto ground_msg = pc_msgs::msg::O3DPointCloud();
            ground_msg.header = msg->header; // 关键：完全继承原始时间戳和 frame_id

            ground_msg.points.reserve(ground_pcd->points_.size() * 3);
            for (const auto& point : ground_pcd->points_) {
                ground_msg.points.push_back(static_cast<float>(point.x()));
                ground_msg.points.push_back(static_cast<float>(point.y()));
                ground_msg.points.push_back(static_cast<float>(point.z()));
            }
            ground_pub_->publish(ground_msg);
        }

        // 发布非地面点云
        auto non_ground_msg = pc_msgs::msg::O3DPointCloud();
        non_ground_msg.header = msg->header; // 关键：完全继承原始时间戳和 frame_id

        non_ground_msg.points.reserve(non_ground_pcd->points_.size() * 3);
        for (const auto& point : non_ground_pcd->points_) {
            non_ground_msg.points.push_back(static_cast<float>(point.x()));
            non_ground_msg.points.push_back(static_cast<float>(point.y()));
            non_ground_msg.points.push_back(static_cast<float>(point.z()));
        }
        non_ground_pub_->publish(non_ground_msg);

        RCLCPP_INFO(this->get_logger(), "[*] Segmentation successful with Ground points %zu, non ground points %zu",
                    ground_pcd->points_.size(), non_ground_pcd->points_.size());
    }

    // 算法参数
    double dist_thresh_;
    int ransac_n_;
    int num_iters_;

    // ROS 对象
    rclcpp::Subscription<pc_msgs::msg::O3DPointCloud>::SharedPtr subscription_;
    rclcpp::Publisher<pc_msgs::msg::O3DPointCloud>::SharedPtr ground_pub_;
    rclcpp::Publisher<pc_msgs::msg::O3DPointCloud>::SharedPtr non_ground_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RansacNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}