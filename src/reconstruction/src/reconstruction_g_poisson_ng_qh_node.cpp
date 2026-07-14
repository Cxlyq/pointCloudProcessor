//
// Created by cx on 7/7/26.
//
//
// Created for hybrid reconstruction (Poisson for ground, ConvexHull/OBB for clusters).
//
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <tuple>
#include <algorithm>
#include <stdexcept>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
// 引入 message_filters 实现时间同步
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include "pc_msgs/msg/o3_d_point_cloud.hpp"
#include "pc_msgs/msg/clustered_point_cloud.hpp"
#include "pc_msgs/msg/o3_d_mesh.hpp"

#include "open3d/Open3D.h"
#include <Eigen/Core>

using std::placeholders::_1;
using std::placeholders::_2;

class ReconstructionPoissonQHNode : public rclcpp::Node {
public:
    // 定义同步策略：近似时间同步
    typedef message_filters::sync_policies::ApproximateTime<
        pc_msgs::msg::O3DPointCloud,
        pc_msgs::msg::ClusteredPointCloud> SyncPolicy;

    ReconstructionPoissonQHNode() : Node("reconstruction_poisson_node") {
        // 1. 声明并读取参数
        this->declare_parameter<std::string>("sub_ground_topic", "/gs/ground_pointcloud");
        this->declare_parameter<std::string>("sub_cluster_topic", "/clustering/clustered_pointcloud");
        this->declare_parameter<std::string>("pub_mesh_topic", "/reconstruction/white_mesh");

        // 聚类重建参数
        this->declare_parameter<int>("min_cluster_size", 15);

        // 泊松重建(地面)特有参数
        this->declare_parameter<int>("poisson_depth", 8);
        this->declare_parameter<double>("normal_radius", 20.0);
        this->declare_parameter<int>("normal_max_nn", 30);
        this->declare_parameter<double>("density_quantile", 0.06);

        auto sub_g_topic = this->get_parameter("sub_ground_topic").as_string();
        auto sub_c_topic = this->get_parameter("sub_cluster_topic").as_string();
        auto pub_m_topic = this->get_parameter("pub_mesh_topic").as_string();

        min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
        poisson_depth_ = this->get_parameter("poisson_depth").as_int();
        normal_radius_ = this->get_parameter("normal_radius").as_double();
        normal_max_nn_ = this->get_parameter("normal_max_nn").as_int();
        density_quantile_ = this->get_parameter("density_quantile").as_double();

        // 2. 创建消息同步订阅器
        ground_sub_.subscribe(this, sub_g_topic);
        cluster_sub_.subscribe(this, sub_c_topic);

        // 队列大小设为 10
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), ground_sub_, cluster_sub_);
        sync_->registerCallback(std::bind(&ReconstructionPoissonQHNode::sync_callback, this, _1, _2));

        // 3. 创建发布者
        publisher_ = this->create_publisher<pc_msgs::msg::O3DMesh>(pub_m_topic, 10);

        RCLCPP_INFO(this->get_logger(), "[*] Hybrid Reconstruction node brought up. Listening on: 1. %s  2. %s...",
                    sub_g_topic.c_str(), sub_c_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "    - Poisson Depth: %d, Density Quantile: %.2f", poisson_depth_, density_quantile_);
    }

private:
    void sync_callback(
        const pc_msgs::msg::O3DPointCloud::ConstSharedPtr& ground_msg,
        const pc_msgs::msg::ClusteredPointCloud::ConstSharedPtr& cluster_msg) const
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        size_t ground_points_count = ground_msg->points.size() / 3;
        size_t cluster_points_count = cluster_msg->points.size() / 3;

        // 最终要拼接在一起的总 Mesh
        auto combined_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
        int valid_hull_count = 0;
        bool ground_reconstructed = false;

        // =========================================================
        // 阶段 1: 地面点泊松重建 (Poisson Surface Reconstruction)
        // =========================================================
        if (ground_points_count > 0) {
            auto ground_pcd = std::make_shared<open3d::geometry::PointCloud>();
            ground_pcd->points_.reserve(ground_points_count);
            for (size_t i = 0; i < ground_points_count; ++i) {
                ground_pcd->points_.emplace_back(
                    static_cast<double>(ground_msg->points[i * 3 + 0]),
                    static_cast<double>(ground_msg->points[i * 3 + 1]),
                    static_cast<double>(ground_msg->points[i * 3 + 2])
                );
            }


            try {
                // 1.1 计算法线
                ground_pcd->EstimateNormals(
                    open3d::geometry::KDTreeSearchParamHybrid(normal_radius_, normal_max_nn_)
                );

                // 1.2 法线朝向对齐到 Z 轴正方向
                ground_pcd->OrientNormalsToAlignWithDirection(Eigen::Vector3d(0.0, 0.0, 1.0));

                // 1.3 泊松重建 (返回网格和密度数组)
                auto poisson_result = open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(*ground_pcd,
                    poisson_depth_,
                    0,     // width (设为0由算法自动推断)
                    1.1,   // scale (默认包围盒缩放比例)
                    true,  // linear_fit (开启线性拟合加速)
                    -1     // n_threads (使用所有可用 CPU 核心)
                );
                auto terrain_mesh = std::get<0>(poisson_result);
                auto densities = std::get<1>(poisson_result);

                // 1.4 密度过滤，切除多余裙边和“橄榄球底座”
                if (!densities.empty() && density_quantile_ > 0.0) {
                    std::vector<double> sorted_densities = densities;
                    std::sort(sorted_densities.begin(), sorted_densities.end());

                    size_t q_idx = static_cast<size_t>(std::floor(sorted_densities.size() * density_quantile_));
                    if (q_idx >= sorted_densities.size()) q_idx = sorted_densities.size() - 1;

                    double density_threshold = sorted_densities[q_idx];

                    std::vector<bool> vertices_to_remove(densities.size());
                    for (size_t i = 0; i < densities.size(); ++i) {
                        vertices_to_remove[i] = (densities[i] < density_threshold);
                    }
                    terrain_mesh->RemoveVerticesByMask(vertices_to_remove);
                }

                // 1.5 包围盒裁剪 (死死限制在原点云范围内)
                auto bbox = ground_pcd->GetAxisAlignedBoundingBox();
                terrain_mesh = terrain_mesh->Crop(bbox);

                // 1.6 刷新法线并赋色
                terrain_mesh->ComputeVertexNormals();
                terrain_mesh->PaintUniformColor(Eigen::Vector3d(0.9, 0.9, 0.9));

                *combined_mesh += *terrain_mesh;
                ground_reconstructed = true;
                valid_hull_count++;
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "[!] Ground Poisson Reconstruction failed: %s", e.what());
            }
        }

        // =========================================================
        // 阶段 2: 非地面聚类块 凸包/OBB 重建
        // =========================================================
        if (cluster_points_count > 0) {
            auto cluster_pcd_full = std::make_shared<open3d::geometry::PointCloud>();
            cluster_pcd_full->points_.reserve(cluster_points_count);

            for (size_t i = 0; i < cluster_points_count; ++i) {
                cluster_pcd_full->points_.emplace_back(
                    static_cast<double>(cluster_msg->points[i * 3 + 0]),
                    static_cast<double>(cluster_msg->points[i * 3 + 1]),
                    static_cast<double>(cluster_msg->points[i * 3 + 2])
                );
            }

            int max_label = cluster_msg->max_label;

            // 遍历所有非地面标签
            for (int i = 0; i <= max_label; ++i) {
                std::vector<size_t> cluster_indices;
                for (size_t j = 0; j < cluster_msg->labels.size(); ++j) {
                    if (cluster_msg->labels[j] == i) {
                        cluster_indices.push_back(j);
                    }
                }

                // 忽略小碎块
                if (cluster_indices.size() < static_cast<size_t>(min_cluster_size_)) {
                    continue;
                }

                auto cluster_pcd = cluster_pcd_full->SelectByIndex(cluster_indices);

                try {
                    // 2.1 尝试使用 3D 凸包
                    auto result = cluster_pcd->ComputeConvexHull();
                    auto hull_mesh = std::get<0>(result);

                    hull_mesh->ComputeVertexNormals();
                    hull_mesh->PaintUniformColor(Eigen::Vector3d(0.9, 0.9, 0.9));

                    *combined_mesh += *hull_mesh;
                    valid_hull_count++;
                } catch (const std::exception& e) {
                    // 2.2 降级策略：凸包失败（如纯平面），使用 OBB
                    try {
                        auto obb = cluster_pcd->GetOrientedBoundingBox();
                        auto obb_mesh = open3d::geometry::TriangleMesh::CreateFromOrientedBoundingBox(obb);

                        obb_mesh->ComputeVertexNormals();
                        obb_mesh->PaintUniformColor(Eigen::Vector3d(0.9, 0.9, 0.9));

                        *combined_mesh += *obb_mesh;
                        valid_hull_count++;
                    } catch (...) {
                        // 彻底失败，忽略
                    }
                }
            }
        }

        // =========================================================
        // 阶段 3: 序列化为自定义 ROS 2 消息并发布
        // =========================================================
        if (combined_mesh->vertices_.empty()) {
            RCLCPP_WARN(this->get_logger(), "[?] No valid mesh generated. Scaped publish.");
            return;
        }

        auto out_msg = pc_msgs::msg::O3DMesh();
        out_msg.header = ground_msg->header; // 继承地面点的时间戳

        // 预分配内存
        out_msg.vertices.reserve(combined_mesh->vertices_.size() * 3);
        for (const auto& v : combined_mesh->vertices_) {
            out_msg.vertices.push_back(static_cast<float>(v.x()));
            out_msg.vertices.push_back(static_cast<float>(v.y()));
            out_msg.vertices.push_back(static_cast<float>(v.z()));
        }

        out_msg.triangles.reserve(combined_mesh->triangles_.size() * 3);
        for (const auto& t : combined_mesh->triangles_) {
            out_msg.triangles.push_back(t.x());
            out_msg.triangles.push_back(t.y());
            out_msg.triangles.push_back(t.z());
        }

        if (combined_mesh->HasVertexNormals()) {
            out_msg.vertex_normals.reserve(combined_mesh->vertex_normals_.size() * 3);
            for (const auto& n : combined_mesh->vertex_normals_) {
                out_msg.vertex_normals.push_back(static_cast<float>(n.x()));
                out_msg.vertex_normals.push_back(static_cast<float>(n.y()));
                out_msg.vertex_normals.push_back(static_cast<float>(n.z()));
            }
        }

        if (combined_mesh->HasVertexColors()) {
            out_msg.vertex_colors.reserve(combined_mesh->vertex_colors_.size() * 3);
            for (const auto& c : combined_mesh->vertex_colors_) {
                out_msg.vertex_colors.push_back(static_cast<float>(c.x()));
                out_msg.vertex_colors.push_back(static_cast<float>(c.y()));
                out_msg.vertex_colors.push_back(static_cast<float>(c.z()));
            }
        }

        // 发布
        publisher_->publish(out_msg);

        auto end_time = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        RCLCPP_INFO(this->get_logger(), "[*] Hybrid Reconstruction done! Terrain: %s | Objects: %d | Time: %.2f ms",
                    ground_reconstructed ? "Yes" : "Failed",
                    ground_reconstructed ? valid_hull_count - 1 : valid_hull_count,
                    elapsed_ms);
    }

    // 参数
    int min_cluster_size_;
    int poisson_depth_;
    double normal_radius_;
    int normal_max_nn_;
    double density_quantile_;

    // ROS 2 对象
    message_filters::Subscriber<pc_msgs::msg::O3DPointCloud> ground_sub_;
    message_filters::Subscriber<pc_msgs::msg::ClusteredPointCloud> cluster_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Publisher<pc_msgs::msg::O3DMesh>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReconstructionPoissonQHNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}