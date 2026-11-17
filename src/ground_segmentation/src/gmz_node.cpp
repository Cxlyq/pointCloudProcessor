
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
        this->declare_parameter<bool>("enable_outlier_suppression", true);
        this->declare_parameter<double>("outlier_z_threshold", 3.0);
        this->declare_parameter<bool>("enable_slope_interpolation", true);

        this->declare_parameter<std::string>("data_saved_dir", "/home");
        this->declare_parameter<bool>("enable_sparse_ground_points_saving", false);
        this->declare_parameter<bool>("enable_ground_points_saving", false);
        this->declare_parameter<bool>("enable_non_ground_points_saving", false);

        auto sub_topic = this->get_parameter("subscribe_topic").as_string();
        auto pub_g_topic = this->get_parameter("publish_ground_topic").as_string();
        auto pub_ng_topic = this->get_parameter("publish_non_ground_topic").as_string();

        grid_size_ = this->get_parameter("grid_size").as_double();
        height_threshold_ = this->get_parameter("height_threshold").as_double();
        enable_outlier_suppression_ = this->get_parameter("enable_outlier_suppression").as_bool();
        outlier_z_threshold_ = this->get_parameter("outlier_z_threshold").as_double();
        enable_slope_interpolation_ = this->get_parameter("enable_slope_interpolation").as_bool();
        data_saved_dir_ = this->get_parameter("data_saved_dir").as_string();
        enable_sparse_ground_points_saving_ = this->get_parameter("enable_sparse_ground_points_saving").as_bool();
        enable_ground_points_saving_ = this->get_parameter("enable_ground_points_saving").as_bool();
        enable_non_ground_points_saving_ = this->get_parameter("enable_non_ground_points_saving").as_bool();

        // 2. 创建订阅者与发布者
        auto qos = rclcpp::QoS(rclcpp::KeepLast(5))
            .reliable()
            .durability_volatile();
        subscription_ = this->create_subscription<pc_msgs::msg::O3DPointCloud>(
            sub_topic, qos, std::bind(&GMZNode::pointcloud_callback, this, _1)
        );

        ground_pub_ = this->create_publisher<pc_msgs::msg::O3DPointCloud>(pub_g_topic, 10);
        non_ground_pub_ = this->create_publisher<pc_msgs::msg::O3DPointCloud>(pub_ng_topic, 10);

        RCLCPP_INFO(this->get_logger(), "[*] GMZ (Grid Minimum Z) node brought up. Listening on: %s...", sub_topic.c_str());
        RCLCPP_INFO(this->get_logger(),
            "[*] Parameters - grid_size: %.2f m, height_threshold: %.2f m, outlier_suppression: %s (thr=%.2f m), slope_interp: %s",
            grid_size_, height_threshold_,
            enable_outlier_suppression_ ? "true" : "false", outlier_z_threshold_,
            enable_slope_interpolation_ ? "true" : "false");
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

        // 3.2a 邻域极低异常点抑制 & 提取待重建稀疏地面点
        std::unordered_map<uint64_t, double> clean_z_map;
        clean_z_map.reserve(grid_min_pt_idx_map.size());

        auto sparse_ground_pcd = std::make_shared<open3d::geometry::PointCloud>();
        sparse_ground_pcd->points_.reserve(grid_min_pt_idx_map.size());
        if (has_colors) sparse_ground_pcd->colors_.reserve(grid_min_pt_idx_map.size());

        for (const auto& pair : grid_min_pt_idx_map) {
            const uint64_t key = pair.first;
            const size_t idx = pair.second;
            const auto& raw_pt = pcd->points_[idx];
            double clean_z = raw_pt.z();

            if (enable_outlier_suppression_) {
                const int64_t gx = static_cast<int64_t>(key >> 32);
                const int64_t gy = static_cast<int64_t>(key & 0xFFFFFFFFULL);
                double nz[8];
                int n_cnt = 0;

                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        if (dx == 0 && dy == 0) continue;
                        const int64_t nx = gx + dx;
                        const int64_t ny = gy + dy;
                        if (nx < 0 || ny < 0) continue;
                        const uint64_t nkey = (static_cast<uint64_t>(nx) << 32) | static_cast<uint64_t>(ny);
                        auto nit = grid_min_pt_idx_map.find(nkey);
                        if (nit != grid_min_pt_idx_map.end()) {
                            nz[n_cnt++] = pcd->points_[nit->second].z();
                        }
                    }
                }

                if (n_cnt >= 3) {
                    std::sort(nz, nz + n_cnt);
                    const double med_z = (n_cnt % 2 == 1)
                        ? nz[n_cnt / 2]
                        : 0.5 * (nz[n_cnt / 2 - 1] + nz[n_cnt / 2]);
                    if (clean_z < med_z - outlier_z_threshold_) {
                        clean_z = med_z - outlier_z_threshold_;
                    }
                }
            }

            clean_z_map[key] = clean_z;
            sparse_ground_pcd->points_.emplace_back(raw_pt.x(), raw_pt.y(), clean_z);
            if (has_colors) sparse_ground_pcd->colors_.push_back(pcd->colors_[idx]);
        }

        // 3.2b 预计算每个网格的 3x3 局部坡面模型
        struct CellSlopeModel {
            Eigen::Vector3d own_pt;
            Eigen::Vector2d grad{0.0, 0.0};
            double max_neighbor_z{0.0};
            Eigen::Vector3d neighbors[9];
            int neighbor_count{0};
        };

        std::unordered_map<uint64_t, CellSlopeModel> cell_model_map;
        cell_model_map.reserve(grid_min_pt_idx_map.size());

        for (const auto& pair : grid_min_pt_idx_map) {
            const uint64_t key = pair.first;
            const size_t idx = pair.second;
            const auto& own_raw_pt = pcd->points_[idx];

            CellSlopeModel model;
            model.own_pt = Eigen::Vector3d(own_raw_pt.x(), own_raw_pt.y(), clean_z_map[key]);
            model.max_neighbor_z = model.own_pt.z();

            if (enable_slope_interpolation_) {
                const int64_t gx = static_cast<int64_t>(key >> 32);
                const int64_t gy = static_cast<int64_t>(key & 0xFFFFFFFFULL);

                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        const int64_t nx = gx + dx;
                        const int64_t ny = gy + dy;
                        if (nx < 0 || ny < 0) continue;
                        const uint64_t nkey = (static_cast<uint64_t>(nx) << 32) | static_cast<uint64_t>(ny);
                        auto nit = grid_min_pt_idx_map.find(nkey);
                        if (nit != grid_min_pt_idx_map.end()) {
                            const auto& n_raw_pt = pcd->points_[nit->second];
                            const Eigen::Vector3d npt(n_raw_pt.x(), n_raw_pt.y(), clean_z_map[nkey]);
                            model.neighbors[model.neighbor_count++] = npt;
                            model.max_neighbor_z = std::max(model.max_neighbor_z, npt.z());
                        }
                    }
                }

                if (model.neighbor_count >= 3) {
                    double sxx = 1e-3, sxy = 0.0, syy = 1e-3;
                    double sxz = 0.0, syz = 0.0;
                    for (int k = 0; k < model.neighbor_count; ++k) {
                        const double dx = model.neighbors[k].x() - model.own_pt.x();
                        const double dy = model.neighbors[k].y() - model.own_pt.y();
                        const double dz = model.neighbors[k].z() - model.own_pt.z();
                        sxx += dx * dx;
                        sxy += dx * dy;
                        syy += dy * dy;
                        sxz += dx * dz;
                        syz += dy * dz;
                    }
                    const double det = sxx * syy - sxy * sxy;
                    if (std::abs(det) > 1e-9) {
                        model.grad.x() = (syy * sxz - sxy * syz) / det;
                        model.grad.y() = (sxx * syz - sxy * sxz) / det;
                    }
                }
            }

            cell_model_map.emplace(key, model);
        }

        // 3.3 第二遍遍历：基于连续坡面估计高度切分非地面点，同时滤除深层地下噪点
        auto non_ground_pcd = std::make_shared<open3d::geometry::PointCloud>();
        auto ground_pcd = std::make_shared<open3d::geometry::PointCloud>();

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

            const auto& model = cell_model_map.at(key);
            const double own_z = model.own_pt.z();
            double ref_ground_z = own_z;

            if (enable_slope_interpolation_) {
                const double plane_raw = own_z
                    + model.grad.x() * (pt.x() - model.own_pt.x())
                    + model.grad.y() * (pt.y() - model.own_pt.y());
                const double plane_z = std::clamp(plane_raw, own_z, model.max_neighbor_z);

                double idw_z = own_z;
                if (model.neighbor_count >= 2) {
                    double w_sum = 0.0;
                    double wz_sum = 0.0;
                    for (int k = 0; k < model.neighbor_count; ++k) {
                        const double dx = model.neighbors[k].x() - pt.x();
                        const double dy = model.neighbors[k].y() - pt.y();
                        const double w = 1.0 / (dx * dx + dy * dy + 1.0);
                        w_sum += w;
                        wz_sum += w * model.neighbors[k].z();
                    }
                    idw_z = wz_sum / w_sum;
                }

                ref_ground_z = std::max({own_z, idw_z, plane_z});
            }

            // 高度阈值切分：高于坡面估计高程 + 阈值为非地面点；低于最低点 - 阈值的深层地下噪点丢弃
            if (pt.z() > (ref_ground_z + height_threshold_)) {
                non_ground_pcd->points_.push_back(pt);
                if (has_colors) non_ground_pcd->colors_.push_back(pcd->colors_[i]);
            } else if (pt.z() >= (own_z - height_threshold_)) {
                ground_pcd->points_.push_back(pt);
                if (has_colors) ground_pcd->colors_.push_back(pcd->colors_[i]);
            }
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

        // 发布非地面点云（若为空则仅发布地面点云并告警）
        if (non_ground_pcd->points_.empty()) {
            RCLCPP_WARN(this->get_logger(), "[?] Empty non-ground point cloud. Ground published, skipped non-ground publish.");
        } else {
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
        }

        // 耗时统计
        auto end_time = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        static size_t gs_frame_seq = 0;
        ++gs_frame_seq;
        RCLCPP_INFO(this->get_logger(),
            "[STAGE-DONE] ground_segmentation: Frame %zu processed | Ground: %zu, Non-Ground: %zu | Time: %.2f ms",
            gs_frame_seq,
            ground_pcd->points_.size(),
            non_ground_pcd->points_.size(),
            elapsed_ms);


        // 本地文件保存 (.ply)
        if (!data_saved_dir_.empty()) {
            // 利用 ROS 消息时间戳作为文件名唯一标识
            std::string timestamp = std::to_string(msg->header.stamp.sec) + "_" +
                                    std::to_string(msg->header.stamp.nanosec);

            // 保存重建地面点
            if (enable_sparse_ground_points_saving_ && sparse_ground_pcd && !sparse_ground_pcd->points_.empty()) {
                std::string file_path = data_saved_dir_ + "sparse_gpcd_" + timestamp + ".ply";
                open3d::io::WritePointCloud(file_path, *sparse_ground_pcd);
            }

            // 保存全量地面点
            if (enable_ground_points_saving_ && ground_pcd && !ground_pcd->points_.empty()) {
                std::string file_path = data_saved_dir_ + "full_gpcd_" + timestamp + ".ply";
                open3d::io::WritePointCloud(file_path, *ground_pcd);
            }

            // 保存无地面点
            if (enable_non_ground_points_saving_ && non_ground_pcd && !non_ground_pcd->points_.empty()) {
                std::string file_path = data_saved_dir_ + "nongpcd_" + timestamp + ".ply";
                open3d::io::WritePointCloud(file_path, *non_ground_pcd);
            }
        }
    }

    // 算法参数
    double grid_size_;
    double height_threshold_;
    bool enable_outlier_suppression_;
    double outlier_z_threshold_;
    bool enable_slope_interpolation_;
    bool enable_ground_points_saving_;
    bool enable_non_ground_points_saving_;
    bool enable_sparse_ground_points_saving_;
    std::string data_saved_dir_;

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