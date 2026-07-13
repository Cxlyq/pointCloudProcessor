//
// Created by cx on 6/22/26.
//
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "pc_msgs/msg/o3_d_point_cloud.hpp"
#include "rclcpp/rclcpp.hpp"

// 引入 Open3D 核心头文件和 IO 模块
#include "open3d/Open3D.h"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

// 定义用于缓存数据的结构体
struct FrameData {
    std::string file_name;
    std::vector<float> points;
    std::vector<float> colors;
    size_t num_points;
};

class IOHandlerNode : public rclcpp::Node {
public:
    IOHandlerNode() : Node("sim_lidar_data_flow_node"), current_idx_(0) {
        // 1. 声明并读取参数
        this->declare_parameter<std::string>("dataset_dir", "");
        this->declare_parameter<double>("publish_rate", 10.0);
        this->declare_parameter<std::string>("publish_topic", "/io/raw_pointcloud");
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<bool>("is_loop", false);

        dataset_dir_ = this->get_parameter("dataset_dir").as_string();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        publish_topic_ = this->get_parameter("publish_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        is_loop_ = this->get_parameter("is_loop").as_bool();

        // 2. 搜索点云文件 (等价于 Python 的 glob)
        if (dataset_dir_.empty() || !fs::exists(dataset_dir_)) {
            RCLCPP_ERROR(this->get_logger(), "[!] Invalid dataset_dir: %s", dataset_dir_.c_str());
            return;
        }

        for (const auto& entry : fs::directory_iterator(dataset_dir_)) {
            if (entry.path().extension() == ".ply") {
                ply_files_.push_back(entry.path().string());
            }
        }

        // 保证按字母顺序排序，确保时序正确
        std::sort(ply_files_.begin(), ply_files_.end());

        if (ply_files_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "[!] Cannot find any .ply file in %s", dataset_dir_.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "[*] Found %zu .ply frames in %s", ply_files_.size(), dataset_dir_.c_str());

        // 3. 预加载所有点云数据到内存中
        preload_pointclouds();

        if (frames_data_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "[!] Cannot find any valid point cloud data!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "[*] Preload successfully! Loaded %zu frames valid data.", frames_data_.size());

        // 4. 创建发布者
        publisher_ = this->create_publisher<pc_msgs::msg::O3DPointCloud>(publish_topic_, 10);

        // 5. 创建定时器，模拟雷达实时数据流
        auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
            std::bind(&IOHandlerNode::timer_callback, this)
        );
        RCLCPP_INFO(this->get_logger(), "[*] Start publishing Lidar data at frequency %.1f Hz...", publish_rate_);
    }

private:
    void preload_pointclouds() {
        for (size_t i = 0; i < ply_files_.size(); ++i) {
            const auto& file_path = ply_files_[i];
            try {
                auto pcd = open3d::io::CreatePointCloudFromFile(file_path);

                if (pcd->points_.empty()) {
                    RCLCPP_WARN(this->get_logger(), "[?] Scape empty ply file: %s", fs::path(file_path).filename().c_str());
                    continue;
                }

                FrameData frame;
                frame.file_name = fs::path(file_path).filename().string();
                frame.num_points = pcd->points_.size();

                // 预先分配内存，提高展平效率
                frame.points.reserve(frame.num_points * 3);
                for (const auto& point : pcd->points_) {
                    // Open3D C++ 内部使用 double，转为 float
                    frame.points.push_back(static_cast<float>(point.x()));
                    frame.points.push_back(static_cast<float>(point.y()));
                    frame.points.push_back(static_cast<float>(point.z()));
                }

                if (pcd->HasColors()) {
                    frame.colors.reserve(frame.num_points * 3);
                    for (const auto& color : pcd->colors_) {
                        frame.colors.push_back(static_cast<float>(color.x()));
                        frame.colors.push_back(static_cast<float>(color.y()));
                        frame.colors.push_back(static_cast<float>(color.z()));
                    }
                }

                frames_data_.push_back(std::move(frame));

                // 打印加载进度
                if ((i + 1) % 50 == 0 || (i + 1) == ply_files_.size()) {
                    RCLCPP_INFO(this->get_logger(), "[*] 预加载进度: %zu/%zu ...", i + 1, ply_files_.size());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "[!] An error occurred while preloading %s: %s", file_path.c_str(), e.what());
            }
        }
    }

    void timer_callback() {
        if (current_idx_ >= frames_data_.size()) {
            RCLCPP_INFO(this->get_logger(), "[*] All ply files have been published.");
            timer_->cancel();
            return;
        }

        const auto& frame = frames_data_[current_idx_];

        auto msg = pc_msgs::msg::O3DPointCloud();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = frame_id_;

        msg.points = frame.points;
        if (!frame.colors.empty()) {
            msg.colors = frame.colors;
        }

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "[*] %s has been published | Point: %zu", frame.file_name.c_str(), frame.num_points);

        current_idx_++;
        if (is_loop_)
        {
            if (current_idx_ >= frames_data_.size())
            {
                current_idx_ = 0;
            }
        }
    }

    std::string dataset_dir_;
    double publish_rate_;
    std::string publish_topic_;
    std::string frame_id_;
    bool is_loop_;

    std::vector<std::string> ply_files_;
    std::vector<FrameData> frames_data_;
    size_t current_idx_;

    rclcpp::Publisher<pc_msgs::msg::O3DPointCloud>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IOHandlerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}