//
// reconstruction_g_poisson_ng_obb_pt_node.cpp
// Created by cx on 9/15/26.
//
// 在 reconstruction_g_poisson_ng_obb_node 的基础上，增加高压塔白模插入功能：
//   - 订阅载机 GPS 状态 topic（来自 io_handlers / sim_plane_data_flow_node）
//   - 每帧重建完成后，调用 QueryNearbyTowers Service 获取附近高压塔 GPS 列表
//   - 将 GPS 坐标（WGS84 经纬高）转换为以载机为原点的局部坐标系 ENU 坐标
//   - 将预加载的高压塔 .ply 模型（已在节点启动时载入内存并旋转好）平移到对应位置后合入总白模
//

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "open3d/Open3D.h"
#include "pc_msgs/msg/o3_d_mesh.hpp"
#include "pc_msgs/msg/o3_d_point_cloud.hpp"
#include "pc_msgs/msg/clustered_point_cloud.hpp"
#include "pc_msgs/msg/plane_state.hpp"
#include "pc_msgs/srv/query_nearby_towers.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
namespace fs = std::filesystem;

// ─────────────────────────────────────────────────────────────────────────────
// 地理工具：WGS84 GPS → ENU 局部坐标（以参考点为原点，单位：米）
// 参考：Zhu et al. "Conversion of Geodetic coordinates to the Local Tangent Plane"
// ─────────────────────────────────────────────────────────────────────────────
namespace geo {

static constexpr double kA  = 6378137.0;          // WGS84 长半轴 (m)
static constexpr double kF  = 1.0 / 298.257223563; // WGS84 扁率
static constexpr double kE2 = 2.0 * kF - kF * kF; // 第一偏心率平方

static inline double deg2rad(double d) { return d * M_PI / 180.0; }

// 以参考点 (ref_lat, ref_lon, ref_alt) 为 ENU 原点，
// 将目标点 (tgt_lat, tgt_lon, tgt_alt) 转换为 ENU 坐标 (east, north, up)，单位 m
static Eigen::Vector3d gps_to_enu(
    double ref_lat_deg, double ref_lon_deg, double ref_alt_m,
    double tgt_lat_deg, double tgt_lon_deg, double tgt_alt_m)
{
    auto to_ecef = [](double lat_d, double lon_d, double alt) -> Eigen::Vector3d {
        const double lat = deg2rad(lat_d);
        const double lon = deg2rad(lon_d);
        const double N = kA / std::sqrt(1.0 - kE2 * std::sin(lat) * std::sin(lat));
        return {
            (N + alt) * std::cos(lat) * std::cos(lon),
            (N + alt) * std::cos(lat) * std::sin(lon),
            (N * (1.0 - kE2) + alt) * std::sin(lat)
        };
    };

    const Eigen::Vector3d ref_ecef = to_ecef(ref_lat_deg, ref_lon_deg, ref_alt_m);
    const Eigen::Vector3d tgt_ecef = to_ecef(tgt_lat_deg, tgt_lon_deg, tgt_alt_m);
    const Eigen::Vector3d delta = tgt_ecef - ref_ecef;

    const double lat = deg2rad(ref_lat_deg);
    const double lon = deg2rad(ref_lon_deg);

    // ENU 旋转矩阵（ECEF → ENU）
    Eigen::Matrix3d R;
    R << -std::sin(lon),              std::cos(lon),             0.0,
         -std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat),
          std::cos(lat) * std::cos(lon),  std::cos(lat) * std::sin(lon), std::sin(lat);

    return R * delta; // (east, north, up)
}

} // namespace geo

// ─────────────────────────────────────────────────────────────────────────────
// 节点类
// ─────────────────────────────────────────────────────────────────────────────
class ReconstructionPoissonOBBPTNode : public rclcpp::Node {
public:
    // 近似时间同步：地面点云 + 聚类点云 + 载机状态（三路同步）
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        pc_msgs::msg::O3DPointCloud,
        pc_msgs::msg::ClusteredPointCloud,
        pc_msgs::msg::PlaneState>;

    ReconstructionPoissonOBBPTNode() : Node("reconstruction_g_poisson_ng_obb_pt_node") {

        // 1. 声明并读取所有参数
        // 继承自 obb 节点的参数
        this->declare_parameter<std::string>("sub_ground_topic", "/gs/ground_pointcloud");
        this->declare_parameter<std::string>("sub_cluster_topic", "/clustering/clustered_pointcloud");
        this->declare_parameter<std::string>("pub_mesh_topic", "/reconstruction/white_mesh");
        this->declare_parameter<int>("min_cluster_size", 15);
        this->declare_parameter<int>("poisson_depth", 8);
        this->declare_parameter<double>("normal_radius", 20.0);
        this->declare_parameter<int>("normal_max_nn", 30);
        this->declare_parameter<double>("density_quantile", 0.06);
        this->declare_parameter<std::string>("data_saved_dir", "");
        this->declare_parameter<bool>("enable_terrain_mesh_saving", false);
        this->declare_parameter<bool>("enable_architecture_mesh_saving", false);
        this->declare_parameter<bool>("enable_combined_mesh_saving", false);
        // 载机状态 topic（来自 sim_plane_data_flow_node）
        this->declare_parameter<std::string>("sub_plane_state_topic", "/io/plane/state");
        // QueryNearbyTowers Service 名称（来自 io_handlers）
        this->declare_parameter<std::string>("tower_query_service", "/io/query_nearby_towers");
        // Service 查询半径（km）
        this->declare_parameter<double>("tower_query_radius_km", 5.0);
        // 三路同步器容错时间（秒），默认 0.05 s（50 ms）
        // 上游三路消息使用相同时间戳，此值仅作安全冗余；设置过大会引入帧错位
        this->declare_parameter<double>("sync_slop_sec", 0.05);
        // 高压塔 .ply 模型路径
        this->declare_parameter<std::string>("tower_model_path", "");
        // 高压塔模型旋转矩阵（角度制，行优先存储的 3x3 展开为 9 个 double）
        // 默认值为单位矩阵（不旋转）
        this->declare_parameter<std::vector<double>>(
            "tower_model_rotation_deg",
            std::vector<double>{1.0, 0.0, 0.0,
                                0.0, 1.0, 0.0,
                                0.0, 0.0, 1.0});
        // 高压塔模型缩放比例（默认 1.0，不缩放）
        this->declare_parameter<double>("tower_model_scale", 1.0);
        // 高压塔模型颜色（RGB，0~1）
        this->declare_parameter<std::vector<double>>(
            "tower_model_color",
            std::vector<double>{0.8, 0.5, 0.1});

        // 读取参数
        auto sub_g_topic  = this->get_parameter("sub_ground_topic").as_string();
        auto sub_c_topic  = this->get_parameter("sub_cluster_topic").as_string();
        auto pub_m_topic  = this->get_parameter("pub_mesh_topic").as_string();

        min_cluster_size_  = this->get_parameter("min_cluster_size").as_int();
        poisson_depth_     = this->get_parameter("poisson_depth").as_int();
        normal_radius_     = this->get_parameter("normal_radius").as_double();
        normal_max_nn_     = this->get_parameter("normal_max_nn").as_int();
        density_quantile_  = this->get_parameter("density_quantile").as_double();

        data_saved_dir_                  = this->get_parameter("data_saved_dir").as_string();
        enable_terrain_mesh_saving_      = this->get_parameter("enable_terrain_mesh_saving").as_bool();
        enable_architecture_mesh_saving_ = this->get_parameter("enable_architecture_mesh_saving").as_bool();
        enable_combined_mesh_saving_     = this->get_parameter("enable_combined_mesh_saving").as_bool();

        sub_plane_state_topic_ = this->get_parameter("sub_plane_state_topic").as_string();
        tower_query_service_   = this->get_parameter("tower_query_service").as_string();
        tower_query_radius_km_ = this->get_parameter("tower_query_radius_km").as_double();
        sync_slop_sec_         = this->get_parameter("sync_slop_sec").as_double();
        tower_model_path_      = this->get_parameter("tower_model_path").as_string();
        tower_model_scale_     = this->get_parameter("tower_model_scale").as_double();

        auto rot_flat = this->get_parameter("tower_model_rotation_deg").as_double_array();
        auto color_v  = this->get_parameter("tower_model_color").as_double_array();

        // 2. 解析旋转矩阵（角度制 → 弧度 → Eigen::Matrix3d）
        // 约定：参数为行优先的 3x3 矩阵，每个元素单位为"度"（角度制欧拉/分量）
        // 这里将其视为直接的旋转矩阵元素（角度制旋转矩阵与弧度制旋转矩阵的值完全相同；
        // 若参数意图是"直接填旋转矩阵数值"，则无需度→弧转换；
        // 若参数意图是 ZYX 欧拉角（度），则按下方构造）
        // 为明确语义，此处采用"直接旋转矩阵数值"方式（angle_deg 其实就是矩阵元素，不需要 deg2rad）
        tower_rotation_ = Eigen::Matrix3d::Identity();
        if (rot_flat.size() == 9) {
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    tower_rotation_(r, c) = rot_flat[static_cast<size_t>(r * 3 + c)];
        } else {
            RCLCPP_WARN(this->get_logger(),
                "[?] tower_model_rotation_deg should have exactly 9 elements; using identity.");
        }

        // 高压塔颜色
        if (color_v.size() == 3) {
            tower_color_ = Eigen::Vector3d(color_v[0], color_v[1], color_v[2]);
        } else {
            tower_color_ = Eigen::Vector3d(0.8, 0.5, 0.1);
        }

        // 3. 保存目录初始化
        bool need_saving = enable_terrain_mesh_saving_ ||
                           enable_architecture_mesh_saving_ ||
                           enable_combined_mesh_saving_;
        if (need_saving && !data_saved_dir_.empty()) {
            if (data_saved_dir_.back() != '/' && data_saved_dir_.back() != '\\')
                data_saved_dir_ += "/";
            if (!fs::exists(data_saved_dir_)) {
                fs::create_directories(data_saved_dir_);
                RCLCPP_INFO(this->get_logger(),
                    "[+] Created mesh saving directory: %s", data_saved_dir_.c_str());
            }
        }

        // 4. 预加载高压塔模型到内存
        load_tower_model();

        // 5. 创建三路时间同步订阅者
        // 上游三路消息（地面点云 / 聚类点云 / 载机状态）共享同一时间戳，
        // 使用 ApproximateTime + 较小 slop 保证帧级对应，防止跨帧错配
        ground_sub_.subscribe(this, sub_g_topic);
        cluster_sub_.subscribe(this, sub_c_topic);
        plane_state_sub_.subscribe(this, sub_plane_state_topic_);

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), ground_sub_, cluster_sub_, plane_state_sub_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_sec_));
        sync_->registerCallback(
            std::bind(&ReconstructionPoissonOBBPTNode::sync_callback, this, _1, _2, _3));

        // ── 6. 创建 Service 客户端（使用独立回调组 + 独立 Executor）────────
        // 原因：sync_callback 本身已在主 Executor 中执行；若在其内部调用
        // rclcpp::spin_until_future_complete(this->get_node_base_interface())
        // 会触发 "already added to an executor" 运行时崩溃。
        // 解决方案：为 Service Client 创建一个不自动加入主 Executor 的回调组，
        // 并绑定到专用 SingleThreadedExecutor，由 srv_executor_ 单独驱动。
        srv_cbg_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);  // false = 不自动加入主 Executor
        tower_client_ = this->create_client<pc_msgs::srv::QueryNearbyTowers>(
            tower_query_service_,
            rmw_qos_profile_services_default,
            srv_cbg_);
        srv_executor_.add_callback_group(srv_cbg_, this->get_node_base_interface());

        // 7. 创建发布者
        publisher_ = this->create_publisher<pc_msgs::msg::O3DMesh>(pub_m_topic, 10);

        RCLCPP_INFO(this->get_logger(),
            "[*] ReconstructionPoissonOBBPT node ready.\n"
            "    Ground topic : %s\n"
            "    Cluster topic: %s\n"
            "    Plane state  : %s\n"
            "    Tower service: %s (radius=%.1f km)\n"
            "    Sync slop    : %.3f s",
            sub_g_topic.c_str(), sub_c_topic.c_str(),
            sub_plane_state_topic_.c_str(),
            tower_query_service_.c_str(), tower_query_radius_km_,
            sync_slop_sec_);
    }

private:

    // 预加载高压塔模型
    void load_tower_model() {
        if (tower_model_path_.empty()) {
            RCLCPP_WARN(this->get_logger(),
                "[?] tower_model_path is empty; power tower insertion will be skipped.");
            return;
        }
        if (!fs::exists(tower_model_path_)) {
            RCLCPP_ERROR(this->get_logger(),
                "[!] tower_model_path not found: %s", tower_model_path_.c_str());
            return;
        }

        try {
            // 读取 .ply（作为点云或三角网格均可；这里统一读为 TriangleMesh）
            tower_mesh_ = std::make_shared<open3d::geometry::TriangleMesh>();
            bool ok = open3d::io::ReadTriangleMesh(tower_model_path_, *tower_mesh_);
            if (!ok || tower_mesh_->vertices_.empty()) {
                // 退回尝试以点云方式读取再凸包化
                RCLCPP_WARN(this->get_logger(),
                    "[?] ReadTriangleMesh failed or empty; trying point cloud + ConvexHull...");
                auto pcd = open3d::io::CreatePointCloudFromFile(tower_model_path_);
                if (!pcd || pcd->points_.empty()) {
                    RCLCPP_ERROR(this->get_logger(),
                        "[!] Cannot read tower model as TriangleMesh or PointCloud: %s",
                        tower_model_path_.c_str());
                    tower_mesh_ = nullptr;
                    return;
                }
                auto [hull, _] = pcd->ComputeConvexHull();
                tower_mesh_ = hull;
            }

            // 缩放
            if (std::abs(tower_model_scale_ - 1.0) > 1e-9) {
                tower_mesh_->Scale(tower_model_scale_, tower_mesh_->GetCenter());
            }

            // 将模型平移到原点（后续在插入时再平移到目标位置）
            const Eigen::Vector3d center = tower_mesh_->GetCenter();
            tower_mesh_->Translate(-center);

            // 旋转（施加用户配置的旋转矩阵）
            tower_mesh_->Rotate(tower_rotation_, Eigen::Vector3d::Zero());

            // 赋色
            tower_mesh_->PaintUniformColor(tower_color_);
            tower_mesh_->ComputeVertexNormals();

            tower_model_loaded_ = true;
            RCLCPP_INFO(this->get_logger(),
                "[*] Tower model loaded: %s (%zu vertices, %zu triangles)",
                tower_model_path_.c_str(),
                tower_mesh_->vertices_.size(),
                tower_mesh_->triangles_.size());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                "[!] Exception while loading tower model: %s", e.what());
            tower_mesh_ = nullptr;
        }
    }

    // 主同步回调：重建 + 高压塔插入（三路严格同帧）
    void sync_callback(
        const pc_msgs::msg::O3DPointCloud::ConstSharedPtr& ground_msg,
        const pc_msgs::msg::ClusteredPointCloud::ConstSharedPtr& cluster_msg,
        const pc_msgs::msg::PlaneState::ConstSharedPtr& plane_state_msg)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        const size_t ground_points_count  = ground_msg->points.size() / 3;
        const size_t cluster_points_count = cluster_msg->points.size() / 3;

        auto combined_mesh    = std::make_shared<open3d::geometry::TriangleMesh>();
        auto architecture_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
        std::shared_ptr<open3d::geometry::TriangleMesh> final_terrain_mesh = nullptr;

        int  valid_hull_count    = 0;
        bool ground_reconstructed = false;

        // ─── 阶段 1：地面 Poisson 重建 ────────────────────────────────────
        if (ground_points_count > 0) {
            auto ground_pcd = std::make_shared<open3d::geometry::PointCloud>();
            ground_pcd->points_.reserve(ground_points_count);
            for (size_t i = 0; i < ground_points_count; ++i) {
                ground_pcd->points_.emplace_back(
                    static_cast<double>(ground_msg->points[i * 3 + 0]),
                    static_cast<double>(ground_msg->points[i * 3 + 1]),
                    static_cast<double>(ground_msg->points[i * 3 + 2]));
            }

            try {
                ground_pcd->EstimateNormals(
                    open3d::geometry::KDTreeSearchParamHybrid(normal_radius_, normal_max_nn_));
                ground_pcd->OrientNormalsToAlignWithDirection(Eigen::Vector3d(0.0, 0.0, 1.0));

                auto poisson_result = open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(
                    *ground_pcd, poisson_depth_, 0, 1.1, true, -1);
                auto terrain_mesh = std::get<0>(poisson_result);
                auto densities    = std::get<1>(poisson_result);

                if (!densities.empty() && density_quantile_ > 0.0) {
                    std::vector<double> sorted_d = densities;
                    std::sort(sorted_d.begin(), sorted_d.end());
                    size_t q_idx = static_cast<size_t>(
                        std::floor(sorted_d.size() * density_quantile_));
                    if (q_idx >= sorted_d.size()) q_idx = sorted_d.size() - 1;
                    const double thr = sorted_d[q_idx];
                    std::vector<bool> to_remove(densities.size());
                    for (size_t i = 0; i < densities.size(); ++i)
                        to_remove[i] = (densities[i] < thr);
                    terrain_mesh->RemoveVerticesByMask(to_remove);
                }

                auto bbox = ground_pcd->GetAxisAlignedBoundingBox();
                terrain_mesh = terrain_mesh->Crop(bbox);
                terrain_mesh->ComputeVertexNormals();
                terrain_mesh->PaintUniformColor(Eigen::Vector3d(0.9, 0.9, 0.9));

                *combined_mesh += *terrain_mesh;
                final_terrain_mesh = terrain_mesh;
                ground_reconstructed = true;
                ++valid_hull_count;
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(),
                    "[!] Ground Poisson Reconstruction failed: %s", e.what());
            }
        }

        // ─── 阶段 2：非地面聚类 OBB 重建 ──────────────────────────────────
        if (cluster_points_count > 0) {
            auto cluster_pcd_full = std::make_shared<open3d::geometry::PointCloud>();
            cluster_pcd_full->points_.reserve(cluster_points_count);
            for (size_t i = 0; i < cluster_points_count; ++i) {
                cluster_pcd_full->points_.emplace_back(
                    static_cast<double>(cluster_msg->points[i * 3 + 0]),
                    static_cast<double>(cluster_msg->points[i * 3 + 1]),
                    static_cast<double>(cluster_msg->points[i * 3 + 2]));
            }

            const int max_label = cluster_msg->max_label;
            for (int i = 0; i <= max_label; ++i) {
                std::vector<size_t> indices;
                for (size_t j = 0; j < cluster_msg->labels.size(); ++j) {
                    if (cluster_msg->labels[j] == i) indices.push_back(j);
                }
                if (indices.size() < static_cast<size_t>(min_cluster_size_)) continue;

                auto cluster_pcd = cluster_pcd_full->SelectByIndex(indices);
                try {
                    auto obb      = cluster_pcd->GetOrientedBoundingBox();
                    auto obb_mesh = open3d::geometry::TriangleMesh::CreateFromOrientedBoundingBox(obb);
                    obb_mesh->ComputeVertexNormals();
                    obb_mesh->PaintUniformColor(Eigen::Vector3d(0.9, 0.9, 0.9));
                    *architecture_mesh += *obb_mesh;
                    ++valid_hull_count;
                } catch (...) {}
            }
            if (!architecture_mesh->vertices_.empty()) {
                *combined_mesh += *architecture_mesh;
            }
        }

        // ─── 阶段 3：高压塔白模插入（使用与本帧同步的载机状态）──────────
        insert_power_towers(combined_mesh, plane_state_msg);

        // ─── 阶段 4：本地文件保存 ─────────────────────────────────────────
        if (!data_saved_dir_.empty()) {
            const std::string ts = std::to_string(ground_msg->header.stamp.sec) + "_" +
                                   std::to_string(ground_msg->header.stamp.nanosec);
            if (enable_terrain_mesh_saving_ && final_terrain_mesh &&
                !final_terrain_mesh->vertices_.empty()) {
                open3d::io::WriteTriangleMesh(
                    data_saved_dir_ + "terrain_" + ts + ".ply", *final_terrain_mesh);
            }
            if (enable_architecture_mesh_saving_ && !architecture_mesh->vertices_.empty()) {
                open3d::io::WriteTriangleMesh(
                    data_saved_dir_ + "architecture_" + ts + ".ply", *architecture_mesh);
            }
            if (enable_combined_mesh_saving_ && !combined_mesh->vertices_.empty()) {
                open3d::io::WriteTriangleMesh(
                    data_saved_dir_ + "combined_" + ts + ".ply", *combined_mesh);
            }
        }

        // ─── 阶段 5：序列化并发布 ──────────────────────────────────────────
        if (combined_mesh->vertices_.empty()) {
            RCLCPP_WARN(this->get_logger(), "[?] No valid mesh generated. Skipped publish.");
            return;
        }

        auto out_msg = pc_msgs::msg::O3DMesh();
        out_msg.header = ground_msg->header;

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

        publisher_->publish(out_msg);

        auto elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - start_time).count();

        RCLCPP_INFO(this->get_logger(),
            "[*] PT Reconstruction done! Terrain: %s | Objects: %d | Time: %.2f ms",
            ground_reconstructed ? "Yes" : "Failed",
            ground_reconstructed ? valid_hull_count - 1 : valid_hull_count,
            elapsed_ms);
    }

    // ── 高压塔插入：查询 Service → GPS→ENU → 插入模型 ────────────────────
    void insert_power_towers(
        std::shared_ptr<open3d::geometry::TriangleMesh>& combined_mesh,
        const pc_msgs::msg::PlaneState::ConstSharedPtr& plane_state)
    {
        if (!tower_model_loaded_ || !tower_mesh_) {
            return; // 模型未加载，跳过
        }

        if (!plane_state) {
            RCLCPP_DEBUG(this->get_logger(),
                "[~] Plane state is null in this sync frame; skipping tower insertion.");
            return;
        }

        // Service 未就绪则跳过
        if (!tower_client_->service_is_ready()) {
            RCLCPP_DEBUG(this->get_logger(),
                "[~] Tower query service not ready; skipping tower insertion.");
            return;
        }

        // 构造请求
        auto request = std::make_shared<pc_msgs::srv::QueryNearbyTowers::Request>();
        request->latitude  = plane_state->latitude;
        request->longitude = plane_state->longitude;
        request->radius_km = tower_query_radius_km_;

        // 同步调用：用专属 Executor 驱动，避免与主 Executor 冲突
        auto future = tower_client_->async_send_request(request);
        const auto wait_result = srv_executor_.spin_until_future_complete(
            future, std::chrono::milliseconds(500));

        if (wait_result != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(),
                "[?] Tower query service call timed out or failed; skipping insertion.");
            return;
        }

        auto response = future.get();
        if (!response->success || response->latitudes.empty()) {
            RCLCPP_DEBUG(this->get_logger(),
                "[~] No nearby towers found (%s).", response->message.c_str());
            return;
        }

        const size_t n_towers = response->latitudes.size();
        RCLCPP_INFO(this->get_logger(),
            "[*] Inserting %zu power tower model(s) into the scene.", n_towers);

        for (size_t i = 0; i < n_towers; ++i) {
            // GPS → ENU（以本帧载机位置为原点）
            // ENU 坐标系：East=X, North=Y, Up=Z
            const Eigen::Vector3d enu = geo::gps_to_enu(
                plane_state->latitude,  plane_state->longitude,  plane_state->altitude,
                response->latitudes[i], response->longitudes[i], response->altitudes[i]);

            // 复制模板网格并平移到目标位置
            auto tower_instance = std::make_shared<open3d::geometry::TriangleMesh>(*tower_mesh_);
            tower_instance->Translate(enu);

            *combined_mesh += *tower_instance;
        }
    }

    // ── 参数（继承自 obb 节点）────────────────────────────────────────────
    int    min_cluster_size_;
    int    poisson_depth_;
    double normal_radius_;
    int    normal_max_nn_;
    double density_quantile_;

    std::string data_saved_dir_;
    bool enable_terrain_mesh_saving_;
    bool enable_architecture_mesh_saving_;
    bool enable_combined_mesh_saving_;

    // ── 新增参数 ──────────────────────────────────────────────────────────
    std::string sub_plane_state_topic_;
    std::string tower_query_service_;
    double      tower_query_radius_km_;
    double      sync_slop_sec_;
    std::string tower_model_path_;
    double      tower_model_scale_;
    Eigen::Matrix3d tower_rotation_;
    Eigen::Vector3d tower_color_;

    // ── 高压塔模型（预加载，旋转 + 缩放已处理，中心已归零）────────────────
    std::shared_ptr<open3d::geometry::TriangleMesh> tower_mesh_{nullptr};
    bool tower_model_loaded_{false};

    // ── ROS 2 对象 ────────────────────────────────────────────────────────
    message_filters::Subscriber<pc_msgs::msg::O3DPointCloud>       ground_sub_;
    message_filters::Subscriber<pc_msgs::msg::ClusteredPointCloud> cluster_sub_;
    message_filters::Subscriber<pc_msgs::msg::PlaneState>          plane_state_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>>     sync_;

    // Service Client 专属回调组与 Executor（与主 Executor 完全隔离）
    rclcpp::CallbackGroup::SharedPtr                               srv_cbg_;
    rclcpp::executors::SingleThreadedExecutor                      srv_executor_;

    rclcpp::Client<pc_msgs::srv::QueryNearbyTowers>::SharedPtr tower_client_;
    rclcpp::Publisher<pc_msgs::msg::O3DMesh>::SharedPtr        publisher_;
};

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReconstructionPoissonOBBPTNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
