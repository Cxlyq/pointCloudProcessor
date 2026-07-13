//
// Created by cx on 7/6/26.
//
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "pc_msgs/msg/o3_d_point_cloud.hpp"

// 引入 Open3D 核心几何体和点云处理头文件
#include "open3d/Open3D.h"

using std::placeholders::_1;

class GMZNode : public rclcpp::Node {
public:
    GMZNode() : Node("gmz_node") {
        // 1. 声明并读取参数
        this->declare_parameter<std::string>("subscribe_topic", "/io/raw_pointcloud");
        this->declare_parameter<std::string>("publish_ground_topic", "/gs/ground_pointcloud");
        this->declare_parameter<std::string>("publish_non_ground_topic", "/gs/non_ground_pointcloud");

        this->declare_parameter<double>("grid_size", 5.0);
        this->declare_parameter<double>("height_threshold", 0.3);

        auto sub_topic = this->get_parameter("subscribe_topic").as_string();
        auto pub_g_topic = this->get_parameter("publish_ground_topic").as_string();
        auto pub_ng_topic = this->get_parameter("publish_non_ground_topic").as_string();

        grid_size_ = this->get_parameter("grid_size").as_double();
        height_threshold_ = this->get_parameter("height_threshold").as_double();

        // 2. 创建订阅者与发布者
        subscription_ = this->create_subscription<pc_msgs::msg::O3DPointCloud>(
            sub_topic, 10, std::bind(&GMZNode::pointcloud_callback, this, _1)
        );

        ground_pub_ = this->create_publisher<pc_msgs::msg::O3DPointCloud>(pub_g_topic, 10);
        non_ground_pub_ = this->create_publisher<pc_msgs::msg::O3DPointCloud>(pub_ng_topic, 10);

        RCLCPP_INFO(this->get_logger(), "[*] GMZ (Grid Minimum Z) node brought up. Listening on: %s...", sub_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "[*] Parameters - grid_size: %.2f m, height_threshold: %.2f m", grid_size_, height_threshold_);
    }

private:
    void pointcloud_callback(const pc_msgs::msg::O3DPointCloud::SharedPtr msg) const {
        auto start_time = std::chrono::high_resolution_clock::now();

        // 1. 检查输入是否为空
        if (msg->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "[?] Scaped empty point cloud message.");
            return;
        }

        // 2. 反序列化：ROS 2 消息转 Open3D 点云
        auto pcd = std::make_shared<open3d::geometry::PointCloud>();
        size_t num_points = msg->points.size() / 3;
        pcd->points_.reserve(num_points);

        for (size_t i = 0; i < num_points; ++i) {
            pcd->points_.emplace_back(
                static_cast<double>(msg->points[i * 3 + 0]),
                static_cast<double>(msg->points[i * 3 + 1]),
                static_cast<double>(msg->points[i * 3 + 2])
            );
        }

        bool has_colors = false;
        if (!msg->colors.empty() && msg->colors.size() == msg->points.size()) {
            has_colors = true;
            pcd->colors_.reserve(num_points);
            for (size_t i = 0; i < num_points; ++i) {
                pcd->colors_.emplace_back(
                    static_cast<double>(msg->colors[i * 3 + 0]),
                    static_cast<double>(msg->colors[i * 3 + 1]),
                    static_cast<double>(msg->colors[i * 3 + 2])
                );
            }
        }

        // 3. GMZ 算法核心逻辑 (O(N) 复杂度)
        double min_x = std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();

        // 3.1 寻找 X 和 Y 的全局最小值，作为网格划分的原点
        for (const auto& pt : pcd->points_) {
            min_x = std::min(min_x, pt.x());
            min_y = std::min(min_y, pt.y());
        }

        // 3.2 第一遍遍历：找到每个网格的最低点索引 (用于提取最稀疏待重建地面)
        // 使用 uint64_t 存储 2D 网格的联合哈希键
        std::unordered_map<uint64_t, size_t> grid_min_pt_idx_map;

        for (size_t i = 0; i < pcd->points_.size(); ++i) {
            const auto& pt = pcd->points_[i];
            // 由于减去了最小值，坐标必为非负，转为 uint32_t 是安全的
            uint32_t grid_x = static_cast<uint32_t>(std::floor((pt.x() - min_x) / grid_size_));
            uint32_t grid_y = static_cast<uint32_t>(std::floor((pt.y() - min_y) / grid_size_));

            // 将 x 和 y 拼接成一个 64 位唯一的 key
            uint64_t key = (static_cast<uint64_t>(grid_x) << 32) | grid_y;

            auto it = grid_min_pt_idx_map.find(key);
            if (it == grid_min_pt_idx_map.end()) {
                grid_min_pt_idx_map[key] = i; // 记录最低点的索引
            } else {
                if (pt.z() < pcd->points_[it->second].z()) {
                    it->second = i; // 更新为更低点的索引
                }
            }
        }

        // 提取待重建地面点 (每个网格唯一的最低点)
        auto sparse_ground_pcd = std::make_shared<open3d::geometry::PointCloud>();
        sparse_ground_pcd->points_.reserve(grid_min_pt_idx_map.size());
        if (has_colors) sparse_ground_pcd->colors_.reserve(grid_min_pt_idx_map.size());

        for (const auto& pair : grid_min_pt_idx_map) {
            size_t idx = pair.second;
            sparse_ground_pcd->points_.push_back(pcd->points_[idx]);
            if (has_colors) sparse_ground_pcd->colors_.push_back(pcd->colors_[idx]);
        }

        // 3.3 第二遍遍历：根据最低点 + 阈值剔除所有附着在地面的点，只保留悬浮的非地面点用于聚类
        auto non_ground_pcd = std::make_shared<open3d::geometry::PointCloud>();

        // 预分配内存，提升速度
        non_ground_pcd->points_.reserve(num_points);
        if (has_colors) {
            non_ground_pcd->colors_.reserve(num_points);
        }

        for (size_t i = 0; i < pcd->points_.size(); ++i) {
            const auto& pt = pcd->points_[i];

            uint32_t grid_x = static_cast<uint32_t>(std::floor((pt.x() - min_x) / grid_size_));
            uint32_t grid_y = static_cast<uint32_t>(std::floor((pt.y() - min_y) / grid_size_));
            uint64_t key = (static_cast<uint64_t>(grid_x) << 32) | grid_y;

            double min_z = pcd->points_[grid_min_pt_idx_map[key]].z();

            // 判断是否在厚度阈值范围内，如果不属于地面厚度区间，则是纯粹的非地面点
            if (pt.z() > (min_z + height_threshold_)) {
                non_ground_pcd->points_.push_back(pt);
                if (has_colors) non_ground_pcd->colors_.push_back(pcd->colors_[i]);
            }
        }

        if (non_ground_pcd->points_.empty()) {
            RCLCPP_WARN(this->get_logger(), "[?] Empty non-ground point cloud. Scaped publish.");
            return;
        }

        // 4. 序列化：Open3D 点云转 ROS 2 消息并发布
        // 发布稀疏地面点云 (加速后续泊松重建)
        if (!sparse_ground_pcd->points_.empty()) {
            auto ground_msg = pc_msgs::msg::O3DPointCloud();
            ground_msg.header = msg->header;

            ground_msg.points.reserve(sparse_ground_pcd->points_.size() * 3);
            for (const auto& point : sparse_ground_pcd->points_) {
                ground_msg.points.push_back(static_cast<float>(point.x()));
                ground_msg.points.push_back(static_cast<float>(point.y()));
                ground_msg.points.push_back(static_cast<float>(point.z()));
            }

            if (has_colors) {
                ground_msg.colors.reserve(sparse_ground_pcd->colors_.size() * 3);
                for (const auto& color : sparse_ground_pcd->colors_) {
                    ground_msg.colors.push_back(static_cast<float>(color.x()));
                    ground_msg.colors.push_back(static_cast<float>(color.y()));
                    ground_msg.colors.push_back(static_cast<float>(color.z()));
                }
            }
            ground_pub_->publish(ground_msg);
        }

        // 发布非地面点云
        auto non_ground_msg = pc_msgs::msg::O3DPointCloud();
        non_ground_msg.header = msg->header;

        non_ground_msg.points.reserve(non_ground_pcd->points_.size() * 3);
        for (const auto& point : non_ground_pcd->points_) {
            non_ground_msg.points.push_back(static_cast<float>(point.x()));
            non_ground_msg.points.push_back(static_cast<float>(point.y()));
            non_ground_msg.points.push_back(static_cast<float>(point.z()));
        }

        if (has_colors) {
            non_ground_msg.colors.reserve(non_ground_pcd->colors_.size() * 3);
            for (const auto& color : non_ground_pcd->colors_) {
                non_ground_msg.colors.push_back(static_cast<float>(color.x()));
                non_ground_msg.colors.push_back(static_cast<float>(color.y()));
                non_ground_msg.colors.push_back(static_cast<float>(color.z()));
            }
        }
        non_ground_pub_->publish(non_ground_msg);

        // 耗时统计
        auto end_time = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        RCLCPP_INFO(this->get_logger(), "[*] GMZ Segmented! Ground: %zu, Non-Ground: %zu, Waiting for reconstruction: %zu. Time: %.2f ms",
                    pcd->points_.size()-non_ground_pcd->points_.size(), non_ground_pcd->points_.size(), sparse_ground_pcd->points_.size(), elapsed_ms);
    }

    // 算法参数
    double grid_size_;
    double height_threshold_;

    // ROS 对象
    rclcpp::Subscription<pc_msgs::msg::O3DPointCloud>::SharedPtr subscription_;
    rclcpp::Publisher<pc_msgs::msg::O3DPointCloud>::SharedPtr ground_pub_;
    rclcpp::Publisher<pc_msgs::msg::O3DPointCloud>::SharedPtr non_ground_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GMZNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}