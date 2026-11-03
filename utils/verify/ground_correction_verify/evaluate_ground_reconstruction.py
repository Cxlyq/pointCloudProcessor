#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地面重建效果评估脚本 (evaluate_ground_reconstruction.py)

功能：
    输入原始地面（真值）与重建地面（算法输出），对比且仅对比点云扫描覆盖到的扇面区域，
    输出多项核心定量评价指标：
    1. 最大误差点（最高偏高点与最大绝对误差点及三维坐标）
    2. 最小误差点（最严重偏低点与最小绝对误差点及三维坐标）
    3. 平均误差（平均偏差 Bias、平均绝对误差 MAE、均方根误差 RMSE、误差标准差 Std）
    4. 精度分位数：在多少误差范围内地面重建能达到 85%（及 50%、90%、95%）的精准度
    5. 输出带颜色的重建地面 .ply 模型：
       - 红色代表偏高 (重建 > 原始)
       - 蓝色代表偏低 (重建 < 原始)
       - 白色代表误差接近 0
       - 颜色深浅直接反映误差大小

使用环境：
    pointcloud_607 (/home/asahichinn/miniconda3/envs/pointcloud_607)
"""

import os
import sys
import json
import argparse
import numpy as np
import open3d as o3d


def load_model(file_path):
    """
    自适应加载 PLY 模型，支持 TriangleMesh 网格和 PointCloud 点云。
    返回: (model, is_mesh, points_or_vertices, triangles_or_none)
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"找不到输入文件: {file_path}")

    # 尝试作为三角网格加载
    mesh = o3d.io.read_triangle_mesh(file_path)
    if len(mesh.triangles) > 0 and len(mesh.vertices) > 0:
        vertices = np.asarray(mesh.vertices)
        triangles = np.asarray(mesh.triangles)
        return mesh, True, vertices, triangles

    # 尝试作为点云加载
    pcd = o3d.io.read_point_cloud(file_path)
    if len(pcd.points) > 0:
        points = np.asarray(pcd.points)
        return pcd, False, points, None

    raise ValueError(f"文件 '{file_path}' 既不是有效的三角网格，也不是点云！")


def detect_radar_station(orig_mesh_or_pcd, pointcloud_file=None, custom_radar_pos=None):
    """
    确定机载雷达的站位坐标与前向/侧向几何矢量
    1. 若用户指定 custom_radar_pos，则优先使用
    2. 若提供 pointcloud_file，可从点云近点/几何包围盒辅助推断
    3. 默认：定位在原始地面矩形长边的中心点上
    """
    if custom_radar_pos is not None:
        return np.array(custom_radar_pos, dtype=np.float32)

    bbox = orig_mesh_or_pcd.get_axis_aligned_bounding_box()
    min_b = bbox.get_min_bound()
    max_b = bbox.get_max_bound()

    span_x = max_b[0] - min_b[0]
    span_y = max_b[1] - min_b[1]

    if span_x >= span_y:
        # X 轴为长边 (约 6km)，Y 轴为短边 (约 4km)
        # 长边中心点位于 X=(min_x+max_x)/2, Y=min_y
        radar_x = (min_b[0] + max_b[0]) / 2.0
        radar_y = min_b[1]
    else:
        # Y 轴为长边，X 轴为短边
        radar_x = min_b[0]
        radar_y = (min_b[1] + max_b[1]) / 2.0

    return np.array([radar_x, radar_y], dtype=np.float32)


def is_inside_scan_sector(xy_coords, radar_xy, forward_vec, right_vec,
                          azimuth_fov=120.0, max_range=None):
    """
    判断二维平面坐标 (X, Y) 是否落在雷达扫描覆盖的 120° 扇面内
    """
    diff = xy_coords - radar_xy[None, :]
    dist_forward = diff @ forward_vec
    dist_right = diff @ right_vec

    # 判定必须位于前向半平面
    forward_mask = dist_forward >= 0.0

    # 计算方位角 (度)
    azimuth = np.degrees(np.arctan2(dist_right, np.maximum(dist_forward, 1e-6)))
    half_fov = azimuth_fov / 2.0
    fov_mask = np.abs(azimuth) <= half_fov

    mask = forward_mask & fov_mask

    if max_range is not None and max_range > 0:
        dist_total = np.sqrt(dist_forward ** 2 + dist_right ** 2)
        mask = mask & (dist_total <= max_range)

    return mask


def query_ground_truth_heights(orig_mesh, eval_points_xy, z_high, z_low=-1000.0):
    """
    利用 Open3D RaycastingScene 向下垂直发射射线，查询原始地面网格在 (x, y) 处的绝对高程。
    """
    t_mesh = o3d.t.geometry.TriangleMesh.from_legacy(orig_mesh)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(t_mesh)

    n_pts = len(eval_points_xy)
    rays = np.zeros((n_pts, 6), dtype=np.float32)
    rays[:, 0] = eval_points_xy[:, 0]
    rays[:, 1] = eval_points_xy[:, 1]
    rays[:, 2] = z_high
    rays[:, 5] = -1.0  # 垂直向下

    ans = scene.cast_rays(o3d.core.Tensor(rays, dtype=o3d.core.Dtype.Float32))
    t_hit = ans['t_hit'].numpy()
    valid = np.isfinite(t_hit)

    z_truth = np.full(n_pts, np.nan, dtype=np.float32)
    z_truth[valid] = z_high - t_hit[valid]
    return z_truth, valid


def generate_error_colormap(errors, in_sector_mask, max_scale=None):
    """
    生成高对比度发散色彩映射 (Diverging Colormap):
    - 误差 = 0 : 纯白 / 米白色 (1.0, 1.0, 1.0)
    - 误差 > 0 : 红色 (偏高)，误差越大颜色越深/越纯 (红渐变到深红)
    - 误差 < 0 : 蓝色 (偏低)，误差越大颜色越深/越纯 (蓝渐变到深蓝)
    - 扇面外点 : 柔和浅灰色 (0.75, 0.75, 0.75)，用于对比背景
    """
    n_pts = len(errors)
    colors = np.full((n_pts, 3), 0.75, dtype=np.float64)  # 默认灰色

    valid_sector_errors = errors[in_sector_mask]
    if len(valid_sector_errors) == 0:
        return colors, 1.0

    if max_scale is None:
        # 取 98 分位绝对误差作为色标饱和上限，避免单个孤立极端点破坏整体色彩辨识度
        p98 = float(np.percentile(np.abs(valid_sector_errors), 98.0))
        max_scale = max(p98, 0.5)

    norm = np.clip(errors / max_scale, -1.0, 1.0)

    # 扇面内部着色
    sec_indices = np.where(in_sector_mask)[0]
    for idx in sec_indices:
        v = norm[idx]
        if np.isnan(v):
            colors[idx] = [0.6, 0.6, 0.6]
            continue
        if v >= 0:
            # 偏高：红色渐变 (v=0 为白 [1,1,1]，v=1 为纯红 [1,0,0])
            r = 1.0
            g = max(0.0, 1.0 - v)
            b = max(0.0, 1.0 - v)
            colors[idx] = [r, g, b]
        else:
            # 偏低：蓝色渐变 (v=0 为白 [1,1,1]，v=-1 为纯蓝 [0,0,1])
            absv = -v
            r = max(0.0, 1.0 - absv)
            g = max(0.0, 1.0 - absv)
            b = 1.0
            colors[idx] = [r, g, b]

    return colors, max_scale


def evaluate_reconstruction(orig_path, recon_path,
                            pointcloud_path=None,
                            output_ply_path=None,
                            azimuth_fov=120.0,
                            radar_pos_override=None,
                            max_scale=None,
                            crop_sector=False):
    """
    主评估执行函数
    """
    print("=" * 78)
    print("地面三维重建精度评估器 (Ground Reconstruction Evaluation Tool)")
    print("=" * 78)
    print(f"原始地面模型 (真值) : {os.path.abspath(orig_path)}")
    print(f"重建地面模型 (待评) : {os.path.abspath(recon_path)}")
    if pointcloud_path:
        print(f"雷达模拟点云 (参考) : {os.path.abspath(pointcloud_path)}")
    print("-" * 78)

    # 1. 加载模型
    orig_model, orig_is_mesh, orig_pts, orig_tris = load_model(orig_path)
    recon_model, recon_is_mesh, recon_pts, recon_tris = load_model(recon_path)

    if not orig_is_mesh:
        raise ValueError("原始地面模型必须是带有三角网格的 .ply 文件，以提供连续的基准几何表面！")

    orig_bbox = orig_model.get_axis_aligned_bounding_box()
    orig_min_b = orig_bbox.get_min_bound()
    orig_max_b = orig_bbox.get_max_bound()

    span_x = orig_max_b[0] - orig_min_b[0]
    span_y = orig_max_b[1] - orig_min_b[1]

    # 2. 定位雷达站位与扇面几何
    radar_xy = detect_radar_station(orig_model, pointcloud_path, radar_pos_override)
    if span_x >= span_y:
        forward_vec = np.array([0.0, 1.0], dtype=np.float32)
        right_vec = np.array([1.0, 0.0], dtype=np.float32)
        max_depth = span_y
    else:
        forward_vec = np.array([1.0, 0.0], dtype=np.float32)
        right_vec = np.array([0.0, -1.0], dtype=np.float32)
        max_depth = span_x

    print(f"雷达基准站位 : 【长边中心点】 X={radar_xy[0]:.1f}m, Y={radar_xy[1]:.1f}m")
    print(f"对地覆盖视场 : 水平方位角 {azimuth_fov:.1f}° (扇面范围 [{-azimuth_fov/2:.1f}°, {azimuth_fov/2:.1f}°]), 纵深 {max_depth:.1f}m")
    print(f"待评节点规模 : 重建模型包含 {len(recon_pts)} 个评估点" + (f", {len(recon_tris)} 个三角面" if recon_is_mesh else ""))

    # 3. 筛选落在雷达扇面内的评估点
    recon_xy = recon_pts[:, :2]
    in_sector_mask = is_inside_scan_sector(
        recon_xy, radar_xy, forward_vec, right_vec,
        azimuth_fov=azimuth_fov, max_range=max_depth * 1.05
    )

    n_total = len(recon_pts)
    n_sector = int(np.sum(in_sector_mask))
    print(f"扇面掩膜过滤 : 扇面内有效评估点数 {n_sector} / {n_total} ({n_sector/n_total*100:.1f}%)")

    if n_sector == 0:
        raise RuntimeError("扇面内未发现任何重建点，请检查两模型的坐标系是否对齐！")

    # 4. 高精度射线查询：获取每个评估点在原始地面的真值高度
    z_high = float(orig_max_b[2] + 200.0)
    z_truth, valid_hit = query_ground_truth_heights(orig_model, recon_xy, z_high)

    # 综合有效掩码：既在扇面内，又能成功垂直投影到原始地面
    eval_mask = in_sector_mask & valid_hit
    n_eval = int(np.sum(eval_mask))

    if n_eval == 0:
        raise RuntimeError("扇面内的评估点均未能垂直投影到原始地面！")

    # 5. 计算垂直高程误差 (误差 = 重建高程 - 原始真值高程)
    z_recon = recon_pts[:, 2]
    errors = np.full(n_total, np.nan, dtype=np.float32)
    errors[eval_mask] = z_recon[eval_mask] - z_truth[eval_mask]

    sector_errors = errors[eval_mask]
    abs_errors = np.abs(sector_errors)
    eval_indices = np.where(eval_mask)[0]

    # 6. 计算各项统计指标
    # (1) 偏高最严重误差点 (Max positive error)
    max_pos_idx_local = int(np.argmax(sector_errors))
    max_pos_idx = eval_indices[max_pos_idx_local]
    max_pos_error = float(sector_errors[max_pos_idx_local])
    max_pos_pt = recon_pts[max_pos_idx]

    # (2) 偏低最严重误差点 (Min error / Max negative undershoot)
    min_neg_idx_local = int(np.argmin(sector_errors))
    min_neg_idx = eval_indices[min_neg_idx_local]
    min_neg_error = float(sector_errors[min_neg_idx_local])
    min_neg_pt = recon_pts[min_neg_idx]

    # (3) 绝对误差最大点 (Worst-case point)
    max_abs_idx_local = int(np.argmax(abs_errors))
    max_abs_idx = eval_indices[max_abs_idx_local]
    max_abs_error = float(abs_errors[max_abs_idx_local])
    max_abs_pt = recon_pts[max_abs_idx]

    # (4) 绝对误差最小点 (Best-fit point)
    min_abs_idx_local = int(np.argmin(abs_errors))
    min_abs_idx = eval_indices[min_abs_idx_local]
    min_abs_error = float(abs_errors[min_abs_idx_local])
    min_abs_pt = recon_pts[min_abs_idx]

    # (5) 均值指标
    mean_error = float(np.mean(sector_errors))     # 平均偏差 Bias
    mae = float(np.mean(abs_errors))              # 平均绝对误差
    rmse = float(np.sqrt(np.mean(sector_errors ** 2))) # 均方根误差
    std_error = float(np.std(sector_errors))      # 误差标准差

    # (6) 分位数精度指标 (核心指标: 85% 精准度阈值)
    p50 = float(np.percentile(abs_errors, 50.0))
    p85 = float(np.percentile(abs_errors, 85.0))
    p90 = float(np.percentile(abs_errors, 90.0))
    p95 = float(np.percentile(abs_errors, 95.0))
    p99 = float(np.percentile(abs_errors, 99.0))

    # 误差区间分布统计
    bins = [0.1, 0.25, 0.5, 1.0, 2.0, 5.0]
    dist_stats = {}
    for b in bins:
        ratio = float(np.sum(abs_errors <= b) / n_eval * 100.0)
        dist_stats[f"<= {b:.2f}m"] = f"{ratio:.2f}%"

    # 7. 打印评估报告
    print("=" * 78)
    print("                    【地面重建扇面精度评估结果】")
    print("=" * 78)
    print(f"评估有效点数       : {n_eval} 点 (完全覆盖扇面区域)")
    print("-" * 78)
    print("【1. 极限误差点分析】")
    print(f"  ● 最大偏高误差点 (Recon > Ground) : +{max_pos_error:.3f} 米")
    print(f"    坐标: X={max_pos_pt[0]:.2f}m, Y={max_pos_pt[1]:.2f}m, Z_recon={max_pos_pt[2]:.2f}m (真实Z={z_truth[max_pos_idx]:.2f}m)")
    print(f"  ● 最大偏低误差点 (Recon < Ground) : {min_neg_error:.3f} 米")
    print(f"    坐标: X={min_neg_pt[0]:.2f}m, Y={min_neg_pt[1]:.2f}m, Z_recon={min_neg_pt[2]:.2f}m (真实Z={z_truth[min_neg_idx]:.2f}m)")
    print(f"  ● 最大绝对误差点 (峰值偏差)        : {max_abs_error:.3f} 米 (坐标: [{max_abs_pt[0]:.1f}, {max_abs_pt[1]:.1f}, {max_abs_pt[2]:.1f}])")
    print(f"  ● 最小绝对误差点 (最精确吻合点)    : {min_abs_error:.4f} 米 (坐标: [{min_abs_pt[0]:.1f}, {min_abs_pt[1]:.1f}, {min_abs_pt[2]:.1f}])")
    print("-" * 78)
    print("【2. 平均误差统计】")
    print(f"  ● 平均系统偏差 (Bias / Mean Error) : {mean_error:+.3f} 米 (" + ("整体偏高" if mean_error > 0 else "整体偏低") + ")")
    print(f"  ● 平均绝对误差 (MAE)              : {mae:.3f} 米")
    print(f"  ● 均方根误差 (RMSE)               : {rmse:.3f} 米")
    print(f"  ● 误差标准差 (Std Dev)            : {std_error:.3f} 米")
    print("-" * 78)
    print("【3. 精准度分位数阈值 (关键考核指标)】")
    print(f"  ★ 85% 精准度误差范围 (P85)       : ± {p85:.3f} 米")
    print(f"     => 【结论】：在 ±{p85:.3f} 米误差范围内，地面重建精度达到 85.0%！")
    print(f"  ● 50% 精准度误差范围 (中位数 P50)  : ± {p50:.3f} 米")
    print(f"  ● 90% 精准度误差范围 (P90)        : ± {p90:.3f} 米")
    print(f"  ● 95% 精准度误差范围 (P95)        : ± {p95:.3f} 米")
    print(f"  ● 99% 精准度误差范围 (P99)        : ± {p99:.3f} 米")
    print("-" * 78)
    print("【4. 误差区间分布累计占比】")
    for b_str, pct_str in dist_stats.items():
        print(f"  ● 绝对误差 {b_str:<10} : {pct_str:>8}")
    print("=" * 78)

    # 8. 生成带颜色的重建地面 .ply 模型
    colors, actual_scale = generate_error_colormap(errors, eval_mask, max_scale=max_scale)

    if output_ply_path is None:
        recon_dir = os.path.dirname(recon_path)
        recon_base = os.path.splitext(os.path.basename(recon_path))[0]
        output_ply_path = os.path.join(recon_dir, f"{recon_base}_error_colored.ply")

    os.makedirs(os.path.dirname(os.path.abspath(output_ply_path)), exist_ok=True)

    if recon_is_mesh:
        recon_model.vertex_colors = o3d.utility.Vector3dVector(colors)
        o3d.io.write_triangle_mesh(output_ply_path, recon_model, write_vertex_colors=True)
    else:
        recon_model.colors = o3d.utility.Vector3dVector(colors)
        o3d.io.write_point_cloud(output_ply_path, recon_model)

    print(f"带色差重建模型已保存 : {os.path.abspath(output_ply_path)}")
    print(f"色阶说明: 【红色】=偏高, 【蓝色】=偏低, 【白色】=无误差, 饱和上限=±{actual_scale:.2f}m")

    # 可选导出扇面裁剪模型
    if crop_sector:
        crop_path = os.path.splitext(output_ply_path)[0] + "_sector_cropped.ply"
        if recon_is_mesh:
            # 提取扇面内顶点与相应面
            sector_triangles = []
            for tri in recon_tris:
                if eval_mask[tri[0]] and eval_mask[tri[1]] and eval_mask[tri[2]]:
                    sector_triangles.append(tri)
            if len(sector_triangles) > 0:
                crop_mesh = o3d.geometry.TriangleMesh()
                crop_mesh.vertices = recon_model.vertices
                crop_mesh.vertex_colors = recon_model.vertex_colors
                crop_mesh.triangles = o3d.utility.Vector3iVector(np.array(sector_triangles, dtype=np.int32))
                crop_mesh.remove_unreferenced_vertices()
                o3d.io.write_triangle_mesh(crop_path, crop_mesh, write_vertex_colors=True)
                print(f"扇面裁剪模型已保存   : {os.path.abspath(crop_path)}")
        else:
            sector_pcd = o3d.geometry.PointCloud()
            sector_pcd.points = o3d.utility.Vector3dVector(recon_pts[eval_mask])
            sector_pcd.colors = o3d.utility.Vector3dVector(colors[eval_mask])
            o3d.io.write_point_cloud(crop_path, sector_pcd)
            print(f"扇面裁剪点云已保存   : {os.path.abspath(crop_path)}")

    results = {
        "n_evaluated_points": n_eval,
        "max_positive_error_m": max_pos_error,
        "max_positive_point": max_pos_pt.tolist(),
        "min_negative_error_m": min_neg_error,
        "min_negative_point": min_neg_pt.tolist(),
        "max_absolute_error_m": max_abs_error,
        "min_absolute_error_m": min_abs_error,
        "mean_error_bias_m": mean_error,
        "mae_m": mae,
        "rmse_m": rmse,
        "std_m": std_error,
        "p50_precision_bound_m": p50,
        "p85_precision_bound_m": p85,
        "p90_precision_bound_m": p90,
        "p95_precision_bound_m": p95,
        "p99_precision_bound_m": p99,
        "colored_ply_path": os.path.abspath(output_ply_path)
    }
    return results


def main():
    parser = argparse.ArgumentParser(
        description="地面重建精度评估脚本：对比且仅对比点云覆盖扇面内的重建误差，输出各项误差指标并导出着色 PLY 模型。"
    )
    parser.add_argument(
        "-g", "--original",
        type=str,
        required=True,
        help="原始地面模型路径 (.ply 三角网格真值)"
    )
    parser.add_argument(
        "-r", "--reconstructed",
        type=str,
        required=True,
        help="重建地面模型路径 (.ply 网格或点云)"
    )
    parser.add_argument(
        "-p", "--pointcloud",
        type=str,
        default=None,
        help="可选：用于生成该重建的雷达模拟点云文件 (.ply)"
    )
    parser.add_argument(
        "-o", "--output",
        type=str,
        default=None,
        help="着色后的重建模型保存路径 (.ply，默认保存在重建模型同目录下)"
    )
    parser.add_argument(
        "--azimuth_fov",
        type=float,
        default=120.0,
        help="雷达水平扫描角 (单位: 度，默认 120.0°)"
    )
    parser.add_argument(
        "--max_scale",
        type=float,
        default=None,
        help="色彩饱和误差阈值 (单位: 米，默认自适应 98% 误差分位数)"
    )
    parser.add_argument(
        "--crop_sector",
        action="store_true",
        help="是否额外输出一份仅包含扇面内的裁剪 .ply 模型"
    )
    parser.add_argument(
        "--save_json",
        type=str,
        default=None,
        help="可选：将量化指标保存为 JSON 文件"
    )

    args = parser.parse_args()

    results = evaluate_reconstruction(
        orig_path=args.original,
        recon_path=args.reconstructed,
        pointcloud_path=args.pointcloud,
        output_ply_path=args.output,
        azimuth_fov=args.azimuth_fov,
        max_scale=args.max_scale,
        crop_sector=args.crop_sector
    )

    if args.save_json:
        with open(args.save_json, "w", encoding="utf-8") as f:
            json.dump(results, f, indent=2, ensure_ascii=False)
        print(f"评估结果数据已保存至 : {os.path.abspath(args.save_json)}")


if __name__ == "__main__":
    main()
