// =============================================================================
// flow_alignment_node : 白膜帧间配准 ROS 2 节点（正式嵌入版）
// -----------------------------------------------------------------------------
// 在原有 ROS 2 流水线（分割→聚类→白膜重建→可视化）中插入一格：
//
//   /reconstruction/white_mesh (pc_msgs/msg/O3DMesh)
//         │  订阅：每帧重建白膜
//         ▼
//   flow_alignment_node
//         │  内部：mesh顶点转点云 → 滑动窗口世界系 map
//         │       → Trimmed Point-to-Plane ICP（8r→4r→2r，alpha截断）
//         ▼
//   /flow/aligned_mesh        (pc_msgs/msg/O3DMesh)           对齐后白膜（世界系）
//   /flow/aligned_pointcloud  (pc_msgs/msg/O3DPointCloud)      窗口 map 采样点云（可选）
//   /flow/registration_stats  (std_msgs/Float64MultiArray)     配准统计
//
// 算法与 flow_test/demo_flow.cpp 完全一致（滑动窗口4帧、voxel 3.0、
// alpha 0.7、法向夹角60°、多尺度8r/4r/2r、每级40次迭代、收敛阈值
// trans<1e-4 & rot<1e-3deg）。不修改任何已有节点/包；本包独立新增。
// =============================================================================
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cfloat>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "pc_msgs/msg/o3_d_mesh.hpp"
#include "pc_msgs/msg/o3_d_point_cloud.hpp"
#include "open3d/Open3D.h"
#include <Eigen/Core>

using namespace open3d;

namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// 与 demo_flow.cpp 一致的配准核心（原样移植）
// ---------------------------------------------------------------------------
struct IcpResult {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    double fitness = 0.0;      // 有效对应 / 源点数
    double inlier_rmse = 0.0;  // 保留集 RMS
    double overlap = 0.0;      // 截断后保留比例
};

static std::shared_ptr<geometry::PointCloud> TransformPcd(
        const std::shared_ptr<geometry::PointCloud>& src, const Eigen::Matrix4d& T) {
    auto out = std::make_shared<geometry::PointCloud>(*src);
    out->Transform(T);
    return out;
}

static IcpResult TrimmedPointToPlaneICP(
        const std::shared_ptr<geometry::PointCloud>& src,
        const std::shared_ptr<geometry::PointCloud>& tgt,
        const Eigen::Matrix4d& T0,
        double r, double alpha, int max_iter_per_level, double max_normal_angle_deg) {
    IcpResult res;
    Eigen::Matrix4d T = T0;

    geometry::KDTreeFlann kd(*tgt);
    pipelines::registration::TransformationEstimationPointToPlane est;
    double cos_max = std::cos(max_normal_angle_deg * M_PI / 180.0);

    std::vector<double> levels = {8.0 * r, 4.0 * r, 2.0 * r};
    size_t n = src->points_.size();

    for (double max_dist : levels) {
        for (int it = 0; it < max_iter_per_level; ++it) {
            auto src_t = TransformPcd(src, T);

            std::vector<int> src_idx, tgt_idx;
            std::vector<double> dist;
            std::vector<int> knn_idx(1);
            std::vector<double> knn_d2(1);
            for (size_t i = 0; i < n; ++i) {
                if (kd.SearchKNN(src_t->points_[i], 1, knn_idx, knn_d2) == 0) continue;
                int j = knn_idx[0];
                if (knn_d2[0] > max_dist * max_dist) continue;              // 距离阈值
                if (tgt->HasNormals()) {
                    double nd = src_t->normals_[i].dot(tgt->normals_[j]);
                    if (nd < cos_max) continue;                              // 法向夹角过滤
                }
                src_idx.push_back(static_cast<int>(i));
                tgt_idx.push_back(j);
                dist.push_back(std::sqrt(knn_d2[0]));
            }
            if (src_idx.empty()) break;

            // 残差排序，截断保留前 alpha
            std::vector<size_t> order(src_idx.size());
            for (size_t k = 0; k < order.size(); ++k) order[k] = k;
            std::sort(order.begin(), order.end(),
                      [&](size_t a, size_t b) { return dist[a] < dist[b]; });

            size_t keep = static_cast<size_t>(std::floor(order.size() * alpha));
            if (keep < 10) keep = std::min(order.size(), size_t(10));

            pipelines::registration::CorrespondenceSet corr;
            corr.reserve(keep);
            for (size_t k = 0; k < keep; ++k) corr.emplace_back(src_idx[order[k]], tgt_idx[order[k]]);

            Eigen::Matrix4d delta = est.ComputeTransformation(*src_t, *tgt, corr);
            T = delta * T;

            double trans_norm = (delta.block<3, 1>(0, 3)).norm();
            Eigen::Matrix3d R = delta.block<3, 3>(0, 0);
            double rot_deg = std::acos(std::max(-1.0, std::min(1.0, (R.trace() - 1.0) / 2.0)))
                             * 180.0 / M_PI;
            if (trans_norm < 1e-4 && rot_deg < 1e-3) break;  // 收敛
        }
    }

    // 最终统计（与 demo_flow 一致）
    auto src_t = TransformPcd(src, T);
    std::vector<int> knn_idx(1);
    std::vector<double> knn_d2(1);
    std::vector<double> final_dist;
    for (size_t i = 0; i < n; ++i) {
        if (kd.SearchKNN(src_t->points_[i], 1, knn_idx, knn_d2) == 0) continue;
        if (knn_d2[0] > (2.0 * r) * (2.0 * r)) continue;
        final_dist.push_back(std::sqrt(knn_d2[0]));
    }
    if (!final_dist.empty()) {
        std::sort(final_dist.begin(), final_dist.end());
        size_t keep = static_cast<size_t>(std::floor(final_dist.size() * alpha));
        if (keep == 0) keep = 1;
        double sum = 0.0;
        for (size_t i = 0; i < keep; ++i) sum += final_dist[i] * final_dist[i];
        res.inlier_rmse = std::sqrt(sum / keep);
        res.overlap = static_cast<double>(final_dist.size()) / static_cast<double>(n);
    }
    res.fitness = static_cast<double>(final_dist.size()) / static_cast<double>(n);
    res.T = T;
    return res;
}

// ---------------------------------------------------------------------------
// 窗口合并 + 体素降采样 + 分辨率估计（与 demo_flow 一致）
// ---------------------------------------------------------------------------
static std::shared_ptr<geometry::PointCloud> MergeWindow(
        const std::vector<std::shared_ptr<geometry::PointCloud>>& frames, double voxel) {
    auto m = std::make_shared<geometry::PointCloud>();
    for (auto& f : frames) {
        m->points_.insert(m->points_.end(), f->points_.begin(), f->points_.end());
        m->normals_.insert(m->normals_.end(), f->normals_.begin(), f->normals_.end());
        m->colors_.insert(m->colors_.end(), f->colors_.begin(), f->colors_.end());
    }
    if (voxel > 0.0) return m->VoxelDownSample(voxel);
    return m;
}

static double EstimateResolution(const std::shared_ptr<geometry::PointCloud>& a,
                                 const std::shared_ptr<geometry::PointCloud>& b) {
    auto nn_median = [](const std::shared_ptr<geometry::PointCloud>& p) {
        geometry::KDTreeFlann kd(*p);
        std::vector<double> d;
        size_t step = p->points_.size() / 2000 + 1;
        std::vector<int> idx(1);
        std::vector<double> d2(1);
        for (size_t i = 0; i < p->points_.size(); i += step) {
            if (kd.SearchKNN(p->points_[i], 2, idx, d2) == 2) d.push_back(std::sqrt(d2[1]));
        }
        if (d.empty()) return 1.0;
        std::sort(d.begin(), d.end());
        return d[d.size() / 2];
    };
    return 0.5 * (nn_median(a) + nn_median(b));
}

// ---------------------------------------------------------------------------
// mesh → 点云（顶点 + 法线 + 颜色，白模渲染用）
// ---------------------------------------------------------------------------
static std::shared_ptr<geometry::PointCloud> MeshToPoints(
        const std::shared_ptr<geometry::TriangleMesh>& mesh) {
    auto pcd = std::make_shared<geometry::PointCloud>();
    pcd->points_ = mesh->vertices_;
    pcd->normals_ = mesh->HasVertexNormals() ? mesh->vertex_normals_
                                             : std::vector<Eigen::Vector3d>(mesh->vertices_.size(),
                                                                            Eigen::Vector3d::Zero());
    pcd->colors_ = mesh->HasVertexColors() ? mesh->vertex_colors_
                                           : std::vector<Eigen::Vector3d>(mesh->vertices_.size(),
                                                                          Eigen::Vector3d(0.9, 0.9, 0.9));
    return pcd;
}

// ---------------------------------------------------------------------------
// 节点
// ---------------------------------------------------------------------------
class FlowAlignmentNode : public rclcpp::Node {
public:
    FlowAlignmentNode() : Node("flow_alignment_node") {
        // ---- 参数（与 flow_test/flow_config.yaml 固化值一致）----
        this->declare_parameter<std::string>("sub_mesh_topic", "/reconstruction/white_mesh");
        this->declare_parameter<std::string>("pub_aligned_mesh_topic", "/flow/aligned_mesh");
        this->declare_parameter<std::string>("pub_map_pc_topic", "/flow/aligned_pointcloud");
        this->declare_parameter<std::string>("pub_stats_topic", "/flow/registration_stats");

        this->declare_parameter<int>("window_size", 4);
        this->declare_parameter<double>("icp_voxel_downsample", 3.0);
        this->declare_parameter<double>("alpha", 0.7);
        this->declare_parameter<int>("icp_max_iter_per_level", 40);
        this->declare_parameter<double>("icp_normal_angle_deg", 60.0);
        this->declare_parameter<bool>("use_gps_init", false);
        this->declare_parameter<double>("align_scale", 1.0);
        this->declare_parameter<double>("normal_radius", 20.0);
        this->declare_parameter<int>("normal_max_nn", 15);

        // ---- 保存（CloudCompare 查看）----
        this->declare_parameter<std::string>("data_saved_dir", "");
        this->declare_parameter<bool>("enable_aligned_saving", false);
        this->declare_parameter<bool>("enable_map_saving", false);

        sub_mesh_topic_         = this->get_parameter("sub_mesh_topic").as_string();
        pub_aligned_mesh_topic_ = this->get_parameter("pub_aligned_mesh_topic").as_string();
        pub_map_pc_topic_       = this->get_parameter("pub_map_pc_topic").as_string();
        pub_stats_topic_        = this->get_parameter("pub_stats_topic").as_string();

        window_size_        = this->get_parameter("window_size").as_int();
        icp_voxel_          = this->get_parameter("icp_voxel_downsample").as_double();
        alpha_              = this->get_parameter("alpha").as_double();
        icp_max_iter_       = this->get_parameter("icp_max_iter_per_level").as_int();
        icp_normal_angle_   = this->get_parameter("icp_normal_angle_deg").as_double();
        use_gps_init_       = this->get_parameter("use_gps_init").as_bool();
        align_scale_        = this->get_parameter("align_scale").as_double();
        normal_radius_      = this->get_parameter("normal_radius").as_double();
        normal_max_nn_      = this->get_parameter("normal_max_nn").as_int();

        data_saved_dir_ = this->get_parameter("data_saved_dir").as_string();
        enable_aligned_saving_ = this->get_parameter("enable_aligned_saving").as_bool();
        enable_map_saving_     = this->get_parameter("enable_map_saving").as_bool();
        if ((enable_aligned_saving_ || enable_map_saving_) && !data_saved_dir_.empty()) {
            if (data_saved_dir_.back() != '/' && data_saved_dir_.back() != '\\') data_saved_dir_ += "/";
            if (!fs::exists(data_saved_dir_)) fs::create_directories(data_saved_dir_);
            RCLCPP_INFO(this->get_logger(), "[+] created save dir: %s", data_saved_dir_.c_str());
        }

        // ---- 发布者 / 订阅者 ----
        pub_aligned_mesh_ = this->create_publisher<pc_msgs::msg::O3DMesh>(pub_aligned_mesh_topic_, 10);
        pub_stats_       = this->create_publisher<std_msgs::msg::Float64MultiArray>(pub_stats_topic_, 10);
        if (!pub_map_pc_topic_.empty()) {
            pub_map_pc_ = this->create_publisher<pc_msgs::msg::O3DPointCloud>(pub_map_pc_topic_, 10);
        }

        auto qos = rclcpp::QoS(rclcpp::KeepLast(5))
            .reliable()
            .durability_volatile();
        sub_mesh_ = this->create_subscription<pc_msgs::msg::O3DMesh>(
            sub_mesh_topic_, qos,
            std::bind(&FlowAlignmentNode::mesh_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "[*] FlowAlignmentNode up. sub=%s pub_mesh=%s pub_pc=%s pub_stats=%s",
                    sub_mesh_topic_.c_str(), pub_aligned_mesh_topic_.c_str(),
                    pub_map_pc_topic_.c_str(), pub_stats_topic_.c_str());
        RCLCPP_INFO(this->get_logger(),
                    "    window=%d voxel=%.1f alpha=%.2f iter=%d angle=%.1f deg gps=%s",
                    window_size_, icp_voxel_, alpha_, icp_max_iter_, icp_normal_angle_,
                    use_gps_init_ ? "on" : "off");
    }

private:
    void mesh_callback(const pc_msgs::msg::O3DMesh::ConstSharedPtr& msg) {
        auto t_start = std::chrono::high_resolution_clock::now();

        // 1) 消息 → Open3D mesh
        auto mesh = std::make_shared<geometry::TriangleMesh>();
        size_t nv = msg->vertices.size() / 3;
        mesh->vertices_.reserve(nv);
        for (size_t i = 0; i < nv; ++i) {
            mesh->vertices_.emplace_back(
                static_cast<double>(msg->vertices[i * 3 + 0]),
                static_cast<double>(msg->vertices[i * 3 + 1]),
                static_cast<double>(msg->vertices[i * 3 + 2]));
        }
        size_t nt = msg->triangles.size() / 3;
        mesh->triangles_.reserve(nt);
        for (size_t i = 0; i < nt; ++i) {
            mesh->triangles_.emplace_back(
                static_cast<int>(msg->triangles[i * 3 + 0]),
                static_cast<int>(msg->triangles[i * 3 + 1]),
                static_cast<int>(msg->triangles[i * 3 + 2]));
        }
        if (!msg->vertex_normals.empty() && msg->vertex_normals.size() >= nv * 3) {
            mesh->vertex_normals_.reserve(nv);
            for (size_t i = 0; i < nv; ++i) {
                mesh->vertex_normals_.emplace_back(
                    static_cast<double>(msg->vertex_normals[i * 3 + 0]),
                    static_cast<double>(msg->vertex_normals[i * 3 + 1]),
                    static_cast<double>(msg->vertex_normals[i * 3 + 2]));
            }
        } else {
            // 兜底：法线估计 + 朝向 +z（与 demo 一致）
            auto tmp = std::make_shared<geometry::PointCloud>();
            tmp->points_ = mesh->vertices_;
            tmp->EstimateNormals(geometry::KDTreeSearchParamHybrid(normal_radius_, normal_max_nn_));
            tmp->OrientNormalsToAlignWithDirection(Eigen::Vector3d(0.0, 0.0, 1.0));
            mesh->vertex_normals_ = tmp->normals_;
        }
        if (!msg->vertex_colors.empty() && msg->vertex_colors.size() >= nv * 3) {
            mesh->vertex_colors_.reserve(nv);
            for (size_t i = 0; i < nv; ++i) {
                mesh->vertex_colors_.emplace_back(
                    static_cast<double>(msg->vertex_colors[i * 3 + 0]),
                    static_cast<double>(msg->vertex_colors[i * 3 + 1]),
                    static_cast<double>(msg->vertex_colors[i * 3 + 2]));
            }
        } else {
            mesh->PaintUniformColor(Eigen::Vector3d(0.9, 0.9, 0.9));  // 白模灰白
        }
        if (mesh->vertices_.empty()) {
            RCLCPP_WARN(this->get_logger(), "[?] empty mesh, skip");
            return;
        }

        // 2) 当前帧源点云
        auto src = MeshToPoints(mesh);

        // 3) 配准到滑动窗口世界系 map
        IcpResult res;
        std::shared_ptr<geometry::PointCloud> src_w;
        std::shared_ptr<geometry::PointCloud> map;
        if (window_.empty()) {
            // 首帧：锚定世界系
            res.T = Eigen::Matrix4d::Identity();
            src_w = std::make_shared<geometry::PointCloud>(*src);
            RCLCPP_INFO(this->get_logger(), "  [reg] frame=%zu anchor (identity)", frame_seq_);
        } else {
            map = MergeWindow(window_, icp_voxel_);
            Eigen::Matrix4d T0 = Eigen::Matrix4d::Identity();
            if (use_gps_init_) {
                // white_mesh 消息不含 GPS；真机若需 GPS 初值，需另行注入。
                // 此处退化 Identity 并提示一次。
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "[i] use_gps_init=true but mesh msg has no GPS; fallback identity");
            }
            double r = EstimateResolution(src, map);
            auto t0 = std::chrono::high_resolution_clock::now();
            res = TrimmedPointToPlaneICP(src, map, T0, r, alpha_, icp_max_iter_, icp_normal_angle_);
            auto t1 = std::chrono::high_resolution_clock::now();
            double icp_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            src_w = TransformPcd(src, res.T);
            Eigen::Matrix3d R = res.T.block<3, 3>(0, 0);
            double rot_deg = std::acos(std::max(-1.0, std::min(1.0, (R.trace() - 1.0) / 2.0)))
                             * 180.0 / M_PI;
            RCLCPP_INFO(this->get_logger(),
                        "  [reg] frame=%zu fitness=%.4f overlap=%.4f rmse=%.4f T=(%.3f,%.3f,%.3f) "
                        "rot=%.3fdeg time=%.1fms map_pts=%zu",
                        frame_seq_, res.fitness, res.overlap, res.inlier_rmse,
                        res.T(0, 3), res.T(1, 3), res.T(2, 3), rot_deg, icp_ms,
                        map->points_.size());
        }

        // 4) 滑动窗口更新：新帧入、最旧帧出
        window_.push_back(src_w);
        if (static_cast<int>(window_.size()) > window_size_) window_.erase(window_.begin());
        auto map_out = MergeWindow(window_, icp_voxel_);

        // 5) 发布对齐后的白膜 mesh（世界系）
        auto aligned = std::make_shared<geometry::TriangleMesh>(*mesh);
        aligned->Transform(res.T);
        // 法线旋转（刚性，保持与几何一致；不重算避免朝向翻转）
        Eigen::Matrix3d R3 = res.T.block<3, 3>(0, 0);
        for (auto& n : aligned->vertex_normals_) n = R3 * n;
        publish_aligned_mesh(aligned, msg->header);

        // 6) 发布窗口 map 点云（可选）
        if (pub_map_pc_) publish_map_pc(map_out, msg->header);

        // 7) 发布配准统计
        publish_stats(res, src->points_.size(), map_out->points_.size());

        // 8) 可选保存 PLY（CloudCompare 查看）
        if ((enable_aligned_saving_ || enable_map_saving_) && !data_saved_dir_.empty()) {
            std::string ts = std::to_string(msg->header.stamp.sec) + "_" +
                             std::to_string(msg->header.stamp.nanosec);
            if (enable_aligned_saving_) {
                std::string p = data_saved_dir_ + "aligned_" + ts + ".ply";
                open3d::io::WriteTriangleMesh(p, *aligned);
                RCLCPP_INFO(this->get_logger(), "  [save] %s", p.c_str());
            }
            if (enable_map_saving_) {
                std::string p = data_saved_dir_ + "map_" + ts + ".ply";
                open3d::io::WritePointCloud(p, *map_out);
                RCLCPP_INFO(this->get_logger(), "  [save] %s", p.c_str());
            }
        }

        double total_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::high_resolution_clock::now() - t_start).count();
        RCLCPP_INFO(this->get_logger(),
            "[STAGE-DONE] flow_alignment: Frame %zu processed | Aligned vertices: %zu | Time: %.2f ms",
            frame_seq_, aligned ? aligned->vertices_.size() : 0, total_ms);
        ++frame_seq_;
    }

    void publish_aligned_mesh(const std::shared_ptr<geometry::TriangleMesh>& m,
                              const std_msgs::msg::Header& hdr) {
        pc_msgs::msg::O3DMesh out;
        out.header = hdr;
        out.vertices.reserve(m->vertices_.size() * 3);
        for (auto& v : m->vertices_) {
            out.vertices.push_back(static_cast<float>(v.x()));
            out.vertices.push_back(static_cast<float>(v.y()));
            out.vertices.push_back(static_cast<float>(v.z()));
        }
        out.triangles.reserve(m->triangles_.size() * 3);
        for (auto& t : m->triangles_) {
            out.triangles.push_back(t.x());
            out.triangles.push_back(t.y());
            out.triangles.push_back(t.z());
        }
        if (m->HasVertexNormals()) {
            out.vertex_normals.reserve(m->vertex_normals_.size() * 3);
            for (auto& n : m->vertex_normals_) {
                out.vertex_normals.push_back(static_cast<float>(n.x()));
                out.vertex_normals.push_back(static_cast<float>(n.y()));
                out.vertex_normals.push_back(static_cast<float>(n.z()));
            }
        }
        if (m->HasVertexColors()) {
            out.vertex_colors.reserve(m->vertex_colors_.size() * 3);
            for (auto& c : m->vertex_colors_) {
                out.vertex_colors.push_back(static_cast<float>(c.x()));
                out.vertex_colors.push_back(static_cast<float>(c.y()));
                out.vertex_colors.push_back(static_cast<float>(c.z()));
            }
        }
        pub_aligned_mesh_->publish(out);
    }

    void publish_map_pc(const std::shared_ptr<geometry::PointCloud>& p,
                        const std_msgs::msg::Header& hdr) {
        pc_msgs::msg::O3DPointCloud out;
        out.header = hdr;
        out.points.reserve(p->points_.size() * 3);
        for (auto& pt : p->points_) {
            out.points.push_back(static_cast<float>(pt.x()));
            out.points.push_back(static_cast<float>(pt.y()));
            out.points.push_back(static_cast<float>(pt.z()));
        }
        pub_map_pc_->publish(out);
    }

    void publish_stats(const IcpResult& res, size_t src_pts, size_t map_pts) {
        std_msgs::msg::Float64MultiArray out;
        // [0]fitness [1]overlap [2]inlier_rmse [3..5]tx ty tz [6]rot_deg [7]src_pts [8]map_pts [9]seq
        Eigen::Matrix3d R = res.T.block<3, 3>(0, 0);
        double rot_deg = std::acos(std::max(-1.0, std::min(1.0, (R.trace() - 1.0) / 2.0)))
                         * 180.0 / M_PI;
        out.data = {res.fitness, res.overlap, res.inlier_rmse,
                    res.T(0, 3), res.T(1, 3), res.T(2, 3),
                    rot_deg, static_cast<double>(src_pts), static_cast<double>(map_pts),
                    static_cast<double>(frame_seq_)};
        pub_stats_->publish(out);
    }

    // ---- 参数 ----
    std::string sub_mesh_topic_, pub_aligned_mesh_topic_, pub_map_pc_topic_, pub_stats_topic_;
    int window_size_;
    double icp_voxel_;
    double alpha_;
    int icp_max_iter_;
    double icp_normal_angle_;
    bool use_gps_init_;
    double align_scale_;
    double normal_radius_;
    int normal_max_nn_;
    std::string data_saved_dir_;
    bool enable_aligned_saving_, enable_map_saving_;

    // ---- 状态 ----
    std::vector<std::shared_ptr<geometry::PointCloud>> window_;  // 世界系滑动窗口帧
    size_t frame_seq_ = 0;

    // ---- ROS ----
    rclcpp::Subscription<pc_msgs::msg::O3DMesh>::SharedPtr sub_mesh_;
    rclcpp::Publisher<pc_msgs::msg::O3DMesh>::SharedPtr pub_aligned_mesh_;
    rclcpp::Publisher<pc_msgs::msg::O3DPointCloud>::SharedPtr pub_map_pc_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_stats_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FlowAlignmentNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
