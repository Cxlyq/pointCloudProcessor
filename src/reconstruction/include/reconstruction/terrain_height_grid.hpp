#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include "open3d/Open3D.h"

namespace reconstruction {

/**
 * @brief 2D 均匀网格空间索引，用于从泊松重建地形三角网格快速查询任意 (x, y) 处的地面高程。
 *
 * 查询策略：
 * 1. 先检查 (x, y) 所属网格单元中注册的三角面片，做 2D 重心坐标包含测试与高程插值；
 * 2. 若未落入任何三角面片且 max_fallback_dist > 0，在半径容差覆盖的网格单元内检索
 *    最近 4 个地形顶点做反距离加权 (IDW) 高程插值；
 * 3. 超出整体包围盒容差范围时返回 std::nullopt。
 */
class TerrainHeightGrid {
public:
    TerrainHeightGrid() = default;

    void build(
        const std::shared_ptr<open3d::geometry::TriangleMesh>& terrain_mesh,
        int grid_nx = 128,
        int grid_ny = 128)
    {
        valid_ = false;
        tris_.clear();
        vertices_.clear();
        cell_triangles_.clear();
        cell_vertices_.clear();

        if (!terrain_mesh || terrain_mesh->vertices_.empty() || terrain_mesh->triangles_.empty()) {
            return;
        }

        grid_nx_ = std::max(grid_nx, 1);
        grid_ny_ = std::max(grid_ny, 1);
        vertices_ = terrain_mesh->vertices_;

        min_x_ = std::numeric_limits<double>::infinity();
        max_x_ = -std::numeric_limits<double>::infinity();
        min_y_ = std::numeric_limits<double>::infinity();
        max_y_ = -std::numeric_limits<double>::infinity();

        for (const auto& v : vertices_) {
            min_x_ = std::min(min_x_, v.x());
            max_x_ = std::max(max_x_, v.x());
            min_y_ = std::min(min_y_, v.y());
            max_y_ = std::max(max_y_, v.y());
        }

        const double span_x = std::max(max_x_ - min_x_, 1.0);
        const double span_y = std::max(max_y_ - min_y_, 1.0);
        cell_dx_ = span_x / static_cast<double>(grid_nx_);
        cell_dy_ = span_y / static_cast<double>(grid_ny_);

        const size_t num_cells = static_cast<size_t>(grid_nx_) * static_cast<size_t>(grid_ny_);
        cell_triangles_.resize(num_cells);
        cell_vertices_.resize(num_cells);

        // 注册三角面片到其 2D AABB 覆盖的所有网格单元
        tris_.reserve(terrain_mesh->triangles_.size());
        for (const auto& tri : terrain_mesh->triangles_) {
            const Eigen::Vector3d& A = vertices_[static_cast<size_t>(tri.x())];
            const Eigen::Vector3d& B = vertices_[static_cast<size_t>(tri.y())];
            const Eigen::Vector3d& C = vertices_[static_cast<size_t>(tri.z())];

            const double denom = (B.y() - C.y()) * (A.x() - C.x()) + (C.x() - B.x()) * (A.y() - C.y());
            if (std::abs(denom) <= 1e-9) {
                continue;
            }

            TriCache tc;
            tc.A = A;
            tc.B = B;
            tc.C = C;
            tc.min_x = std::min({A.x(), B.x(), C.x()});
            tc.max_x = std::max({A.x(), B.x(), C.x()});
            tc.min_y = std::min({A.y(), B.y(), C.y()});
            tc.max_y = std::max({A.y(), B.y(), C.y()});
            tc.denom = denom;

            const size_t tri_idx = tris_.size();
            tris_.push_back(tc);

            const int gx_min = clamp_x(tc.min_x - 1e-3);
            const int gx_max = clamp_x(tc.max_x + 1e-3);
            const int gy_min = clamp_y(tc.min_y - 1e-3);
            const int gy_max = clamp_y(tc.max_y + 1e-3);

            for (int cy = gy_min; cy <= gy_max; ++cy) {
                for (int cx = gx_min; cx <= gx_max; ++cx) {
                    cell_triangles_[static_cast<size_t>(cy) * grid_nx_ + cx].push_back(tri_idx);
                }
            }
        }

        // 注册顶点用于容差范围内的快速 IDW 回退
        for (size_t i = 0; i < vertices_.size(); ++i) {
            const int cx = clamp_x(vertices_[i].x());
            const int cy = clamp_y(vertices_[i].y());
            cell_vertices_[static_cast<size_t>(cy) * grid_nx_ + cx].push_back(i);
        }

        valid_ = !tris_.empty();
    }

    bool valid() const { return valid_; }

    std::optional<double> get_height(double target_x, double target_y, double max_fallback_dist = 25.0) const {
        if (!valid_) {
            return std::nullopt;
        }

        // 1. 整体 2D AABB + 容差粗筛
        if (target_x < min_x_ - max_fallback_dist || target_x > max_x_ + max_fallback_dist ||
            target_y < min_y_ - max_fallback_dist || target_y > max_y_ + max_fallback_dist) {
            return std::nullopt;
        }

        // 2. 精确三角形包含判定与重心坐标插值
        if (target_x >= min_x_ - 1e-3 && target_x <= max_x_ + 1e-3 &&
            target_y >= min_y_ - 1e-3 && target_y <= max_y_ + 1e-3) {
            const int cx = clamp_x(target_x);
            const int cy = clamp_y(target_y);
            const auto& tri_list = cell_triangles_[static_cast<size_t>(cy) * grid_nx_ + cx];

            bool hit = false;
            double best_z = -std::numeric_limits<double>::infinity();

            for (size_t tri_idx : tri_list) {
                const auto& tc = tris_[tri_idx];
                if (target_x < tc.min_x - 1e-3 || target_x > tc.max_x + 1e-3 ||
                    target_y < tc.min_y - 1e-3 || target_y > tc.max_y + 1e-3) {
                    continue;
                }

                const double wA = ((tc.B.y() - tc.C.y()) * (target_x - tc.C.x())
                                 + (tc.C.x() - tc.B.x()) * (target_y - tc.C.y())) / tc.denom;
                const double wB = ((tc.C.y() - tc.A.y()) * (target_x - tc.C.x())
                                 + (tc.A.x() - tc.C.x()) * (target_y - tc.C.y())) / tc.denom;
                const double wC = 1.0 - wA - wB;

                if (wA >= -1e-4 && wB >= -1e-4 && wC >= -1e-4) {
                    const double z = wA * tc.A.z() + wB * tc.B.z() + wC * tc.C.z();
                    if (!hit || z > best_z) {
                        best_z = z;
                        hit = true;
                    }
                }
            }

            if (hit) {
                return best_z;
            }
        }

        // 3. 未命中三角面片时，在 max_fallback_dist 容差内做最近 4 顶点 IDW 插值
        if (max_fallback_dist <= 0.0) {
            return std::nullopt;
        }

        const double max_dist_sq = max_fallback_dist * max_fallback_dist;
        const int gx_min = clamp_x(target_x - max_fallback_dist);
        const int gx_max = clamp_x(target_x + max_fallback_dist);
        const int gy_min = clamp_y(target_y - max_fallback_dist);
        const int gy_max = clamp_y(target_y + max_fallback_dist);

        struct Neighbor {
            double dist_sq;
            double z;
        };
        Neighbor nearest[4];
        int nearest_count = 0;

        for (int cy = gy_min; cy <= gy_max; ++cy) {
            for (int cx = gx_min; cx <= gx_max; ++cx) {
                const auto& v_list = cell_vertices_[static_cast<size_t>(cy) * grid_nx_ + cx];
                for (size_t v_idx : v_list) {
                    const auto& v = vertices_[v_idx];
                    const double dx = v.x() - target_x;
                    const double dy = v.y() - target_y;
                    const double d2 = dx * dx + dy * dy;
                    if (d2 > max_dist_sq) {
                        continue;
                    }

                    if (nearest_count < 4) {
                        nearest[nearest_count++] = {d2, v.z()};
                        if (nearest_count == 4) {
                            std::sort(nearest, nearest + 4, [](const Neighbor& a, const Neighbor& b) {
                                return a.dist_sq < b.dist_sq;
                            });
                        }
                    } else if (d2 < nearest[3].dist_sq) {
                        nearest[3] = {d2, v.z()};
                        std::sort(nearest, nearest + 4, [](const Neighbor& a, const Neighbor& b) {
                            return a.dist_sq < b.dist_sq;
                        });
                    }
                }
            }
        }

        if (nearest_count == 0) {
            return std::nullopt;
        }

        if (nearest_count < 4) {
            std::sort(nearest, nearest + nearest_count, [](const Neighbor& a, const Neighbor& b) {
                return a.dist_sq < b.dist_sq;
            });
        }

        if (nearest[0].dist_sq < 1e-4) {
            return nearest[0].z;
        }

        double weight_sum = 0.0;
        double z_weighted = 0.0;
        for (int i = 0; i < nearest_count; ++i) {
            const double w = 1.0 / (nearest[i].dist_sq + 1e-6);
            weight_sum += w;
            z_weighted += w * nearest[i].z;
        }
        return z_weighted / weight_sum;
    }

private:
    struct TriCache {
        Eigen::Vector3d A;
        Eigen::Vector3d B;
        Eigen::Vector3d C;
        double min_x{0.0};
        double max_x{0.0};
        double min_y{0.0};
        double max_y{0.0};
        double denom{0.0};
    };

    int clamp_x(double x) const {
        const int gx = static_cast<int>(std::floor((x - min_x_) / cell_dx_));
        return std::clamp(gx, 0, grid_nx_ - 1);
    }

    int clamp_y(double y) const {
        const int gy = static_cast<int>(std::floor((y - min_y_) / cell_dy_));
        return std::clamp(gy, 0, grid_ny_ - 1);
    }

    bool valid_{false};
    int grid_nx_{128};
    int grid_ny_{128};
    double min_x_{0.0};
    double max_x_{0.0};
    double min_y_{0.0};
    double max_y_{0.0};
    double cell_dx_{1.0};
    double cell_dy_{1.0};

    std::vector<TriCache> tris_;
    std::vector<Eigen::Vector3d> vertices_;
    std::vector<std::vector<size_t>> cell_triangles_;
    std::vector<std::vector<size_t>> cell_vertices_;
};

/**
 * @brief 使用地形高程网格对单个聚类簇内的地面以下点进行过滤，保留高于地面净空阈值的点索引。
 */
inline std::vector<size_t> filter_cluster_above_terrain(
    const open3d::geometry::PointCloud& pcd,
    const std::vector<size_t>& cluster_indices,
    const TerrainHeightGrid& terrain_grid,
    double clearance,
    double fallback_dist = 10.0)
{
    if (!terrain_grid.valid()) {
        return cluster_indices;
    }
    std::vector<size_t> filtered;
    filtered.reserve(cluster_indices.size());
    for (size_t idx : cluster_indices) {
        const auto& pt = pcd.points_[idx];
        auto gz = terrain_grid.get_height(pt.x(), pt.y(), fallback_dist);
        if (!gz.has_value() || pt.z() >= *gz + clearance) {
            filtered.push_back(idx);
        }
    }
    return filtered;
}

/**
 * @brief 对 OBB 网格顶点与三角面中心执行地面防穿模抬升约束；若退化扁平（高差 < min_height）返回 false。
 */
inline bool clamp_obb_mesh_above_terrain(
    open3d::geometry::TriangleMesh& obb_mesh,
    const TerrainHeightGrid& terrain_grid,
    double clearance,
    double fallback_dist = 10.0,
    double min_height = 0.5)
{
    if (obb_mesh.vertices_.empty()) {
        return false;
    }
    if (!terrain_grid.valid()) {
        return true;
    }

    // 1. 逐顶点防穿模抬升
    for (auto& v : obb_mesh.vertices_) {
        auto gz = terrain_grid.get_height(v.x(), v.y(), fallback_dist);
        if (gz.has_value() && v.z() < *gz + clearance) {
            v.z() = *gz + clearance;
        }
    }

    // 2. 逐三角面中心防穿模抬升（防止底面跨过凸起山脊时中部穿模）
    for (const auto& tri : obb_mesh.triangles_) {
        auto& v0 = obb_mesh.vertices_[static_cast<size_t>(tri.x())];
        auto& v1 = obb_mesh.vertices_[static_cast<size_t>(tri.y())];
        auto& v2 = obb_mesh.vertices_[static_cast<size_t>(tri.z())];
        const Eigen::Vector3d center = (v0 + v1 + v2) / 3.0;
        auto gz_c = terrain_grid.get_height(center.x(), center.y(), fallback_dist);
        if (gz_c.has_value() && center.z() < *gz_c + clearance) {
            const double delta = (*gz_c + clearance) - center.z();
            v0.z() += delta;
            v1.z() += delta;
            v2.z() += delta;
        }
    }

    // 3. 检查修正后高度跨度，丢弃被地面压扁的退化包围盒
    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    for (const auto& v : obb_mesh.vertices_) {
        min_z = std::min(min_z, v.z());
        max_z = std::max(max_z, v.z());
    }
    return (max_z - min_z) >= min_height;
}

}  // namespace reconstruction
