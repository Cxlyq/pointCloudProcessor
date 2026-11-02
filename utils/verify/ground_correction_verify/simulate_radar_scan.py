#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
雷达地面扫描模拟脚本 (simulate_radar_scan.py)

功能：
    输入 .ply 地面白模文件夹（或单个文件），模拟机载雷达对每个白模进行对地扫描。
    1. 雷达位置：位于地面矩形长边的中心点上，离地高度约 300~400 米，朝向矩形内部扫描。
    2. 扫描模式：
       - 水平扫描角 120 度 (方位角 [-60°, +60°])
       - 俯仰扫描角从正下方 (-90°) 到正前方/正上方 (+90°)
       - 【动态步长扫描】：近处采用大俯仰步长防止点云过密，远处采用小步长保证地表细节
    3. 目标点数：每个地形采集的点云点数大约为 15,000 个。
    4. 输出：输出每个地面扫描后的 .ply 地面点云。

使用环境：
    pointcloud_607 (/home/asahichinn/miniconda3/envs/pointcloud_607)
"""

import os
import sys
import glob
import argparse
import numpy as np
import open3d as o3d


def find_ground_elevation_at(scene, x, y, z_high, z_low=-1000.0):
    """
    通过从高空向下发射垂直射线获取地面在 (x, y) 处的真实表面高程 Z_ground
    """
    ray = np.array([[x, y, z_high, 0.0, 0.0, -1.0]], dtype=np.float32)
    ans = scene.cast_rays(o3d.core.Tensor(ray, dtype=o3d.core.Dtype.Float32))
    t_hit = ans['t_hit'].numpy()[0]
    if np.isfinite(t_hit):
        return float(z_high - t_hit)
    return float(z_low)


def generate_pitch_angles(n_pitch_total, radar_h, terrain_length,
                          pitch_min=-90.0, pitch_max=90.0, mode="dynamic", power=1.0):
    """
    生成俯仰扫描角序列：
    - uniform 模式：线性等角度步长 [-90°, +90°]
    - dynamic 模式：根据地表几何距离进行非线性动态步长采样。
      近处 (正下方 Nadir -90° 附近) 步长较大，防止点云过密；
      远处 (切向浅角附近) 步长极小，密集采样保证远处细节；
      空域部分 (> 0° 至 +90°) 平滑过渡，覆盖完整俯仰视场。
    """
    if mode == "uniform":
        return np.linspace(pitch_min, pitch_max, n_pitch_total, dtype=np.float32)

    # 地面最远点对应的理论最大下视角 (例如 H=350m, L=6000m 时约为 -3.34°)
    phi_ground_far = -np.degrees(np.arctan2(radar_h, max(terrain_length, 1.0)))
    phi_ground_far = min(phi_ground_far, -0.5)

    # 俯仰角度点数分配：大部分分配给地面有效交切区，少部分分配给地平线及以上天际/空域
    n_ground = max(10, int(round(n_pitch_total * 0.88)))
    n_sky = max(2, n_pitch_total - n_ground)

    # 沿地表纵深距离进行幂律/等间距连续映射
    # s in [0, 1] 映射到地表距离 x in [0, terrain_length]
    s = np.linspace(0.0, 1.0, n_ground, dtype=np.float32)
    xs = (s ** power) * terrain_length

    # 俯仰角 phi = -arctan(H / x)，当 x->0 时 phi->-90°，当 x->L 时 phi->phi_ground_far
    pitches_ground = -np.degrees(np.arctan2(radar_h, np.maximum(xs, 1e-4)))
    pitches_ground[0] = pitch_min

    # 从地平线往上的空域/正前方至正上方扫描角 (phi_ground_far 到 pitch_max)
    pitches_sky = np.linspace(phi_ground_far, pitch_max, n_sky + 1, dtype=np.float32)[1:]

    pitches = np.concatenate([pitches_ground, pitches_sky])
    return pitches


def generate_scan_rays(radar_pos, forward_vec, right_vec, up_vec,
                       n_azimuth, n_pitch, radar_h, terrain_length,
                       az_fov=120.0, pitch_min=-90.0, pitch_max=90.0,
                       pitch_mode="dynamic", pitch_power=1.0):
    """
    根据雷达安装站位、水平视场角及动态/均匀俯仰采样生成 3D 扫描光线集
    """
    azs = np.radians(np.linspace(-az_fov / 2.0, az_fov / 2.0, n_azimuth, dtype=np.float32))
    pitch_degs = generate_pitch_angles(
        n_pitch_total=n_pitch,
        radar_h=radar_h,
        terrain_length=terrain_length,
        pitch_min=pitch_min,
        pitch_max=pitch_max,
        mode=pitch_mode,
        power=pitch_power
    )
    pitches = np.radians(pitch_degs)

    AZ, PITCH = np.meshgrid(azs, pitches)
    az_flat = AZ.ravel()
    pitch_flat = PITCH.ravel()

    # 3D 方向向量合成: cos(pitch)*cos(az)*forward + cos(pitch)*sin(az)*right + sin(pitch)*up
    cos_p = np.cos(pitch_flat)
    sin_p = np.sin(pitch_flat)
    cos_a = np.cos(az_flat)
    sin_a = np.sin(az_flat)

    dirs = (cos_p[:, None] * cos_a[:, None] * forward_vec[None, :] +
            cos_p[:, None] * sin_a[:, None] * right_vec[None, :] +
            sin_p[:, None] * up_vec[None, :]).astype(np.float32)

    # 射线归一化
    norms = np.linalg.norm(dirs, axis=1, keepdims=True)
    dirs = dirs / np.maximum(norms, 1e-8)

    origins = np.tile(radar_pos, (len(dirs), 1)).astype(np.float32)
    rays = np.concatenate([origins, dirs], axis=-1)
    return origins, dirs, rays, pitch_degs


def cast_and_collect_hits(scene, origins, dirs, rays, add_noise=0.0):
    """
    执行射线求交，提取有效击中地面的点云及测距、法向量
    """
    ans = scene.cast_rays(o3d.core.Tensor(rays, dtype=o3d.core.Dtype.Float32))
    t_hit = ans['t_hit'].numpy()
    valid = np.isfinite(t_hit)

    hit_origins = origins[valid]
    hit_dirs = dirs[valid]
    hit_ranges = t_hit[valid]
    hit_normals = ans['primitive_normals'].numpy()[valid]

    # 添加可选测距高斯噪声
    if add_noise > 0.0:
        rng = np.random.normal(0.0, add_noise, size=hit_ranges.shape).astype(np.float32)
        hit_ranges = np.maximum(hit_ranges + rng, 0.1)

    hit_points = hit_origins + hit_dirs * hit_ranges[:, None]
    return hit_points, hit_ranges, hit_normals


def simulate_radar_on_mesh(mesh_path,
                           radar_height=350.0,
                           target_points=15000,
                           azimuth_fov=120.0,
                           pitch_min=-90.0,
                           pitch_max=90.0,
                           pitch_mode="dynamic",
                           pitch_power=1.0,
                           add_noise=0.0,
                           exact_target=False):
    """
    对单个 .ply 地面白模执行雷达扫描模拟
    """
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    if len(mesh.triangles) == 0:
        raise ValueError(f"文件 '{mesh_path}' 不是有效的三角网格或三角形数量为0！")

    t_mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(t_mesh)

    # 计算包围盒与长边中心点
    bbox = mesh.get_axis_aligned_bounding_box()
    min_b = bbox.get_min_bound()
    max_b = bbox.get_max_bound()

    span_x = max_b[0] - min_b[0]
    span_y = max_b[1] - min_b[1]

    # 将雷达架设在矩形长边的中心点上，朝向地形内部扫描
    if span_x >= span_y:
        # X 轴为长边 (约 6km)，Y 轴为短边 (约 4km)
        # 长边位于 Y = min_b[1] 处，长边中心点为 X = (min_b[0] + max_b[0]) / 2
        start_x = (min_b[0] + max_b[0]) / 2.0
        start_y = min_b[1]
        forward_vec = np.array([0.0, 1.0, 0.0], dtype=np.float32)   # 正前方朝向 +Y (横贯 4km 地形)
        right_vec = np.array([1.0, 0.0, 0.0], dtype=np.float32)     # 右侧朝向 +X
        terrain_depth = span_y
    else:
        # Y 轴为长边 (约 6km)，X 轴为短边 (约 4km)
        # 长边位于 X = min_b[0] 处，长边中心点为 Y = (min_b[1] + max_b[1]) / 2
        start_x = min_b[0]
        start_y = (min_b[1] + max_b[1]) / 2.0
        forward_vec = np.array([1.0, 0.0, 0.0], dtype=np.float32)   # 正前方朝向 +X
        right_vec = np.array([0.0, -1.0, 0.0], dtype=np.float32)
        terrain_depth = span_x

    terrain_length = terrain_depth

    up_vec = np.array([0.0, 0.0, 1.0], dtype=np.float32)

    # 确定雷达安装位置 (向网格内微偏 5m 探测地表真实高程，防止边界浮点缝隙)
    test_x = start_x + forward_vec[0] * 5.0
    test_y = start_y + forward_vec[1] * 5.0
    ground_z = find_ground_elevation_at(scene, test_x, test_y, max_b[2] + 200.0, min_b[2])
    radar_z = ground_z + radar_height
    radar_pos = np.array([start_x, start_y, radar_z], dtype=np.float32)

    # 自适应标定光线角分辨率，使采集点数稳定在目标点数 (~15,000 点)
    if pitch_mode == "dynamic":
        n_az = 130
        n_pitch = 150
    else:
        n_az = 175
        n_pitch = 175

    origins, dirs, rays, pitch_degs = generate_scan_rays(
        radar_pos, forward_vec, right_vec, up_vec,
        n_az, n_pitch, radar_h=radar_height, terrain_length=terrain_length,
        az_fov=azimuth_fov, pitch_min=pitch_min, pitch_max=pitch_max,
        pitch_mode=pitch_mode, pitch_power=pitch_power
    )
    hit_pts, hit_rng, hit_norms = cast_and_collect_hits(scene, origins, dirs, rays, add_noise=0.0)
    current_hits = len(hit_pts)

    if current_hits > 0 and abs(current_hits - target_points) > 400:
        scale = np.sqrt(target_points / float(current_hits))
        n_az = max(10, int(round(n_az * scale)))
        n_pitch = max(10, int(round(n_pitch * scale)))
        origins, dirs, rays, pitch_degs = generate_scan_rays(
            radar_pos, forward_vec, right_vec, up_vec,
            n_az, n_pitch, radar_h=radar_height, terrain_length=terrain_length,
            az_fov=azimuth_fov, pitch_min=pitch_min, pitch_max=pitch_max,
            pitch_mode=pitch_mode, pitch_power=pitch_power
        )
        hit_pts, hit_rng, hit_norms = cast_and_collect_hits(scene, origins, dirs, rays, add_noise=add_noise)

    # 若需要精确截断至目标点数
    if exact_target and len(hit_pts) > target_points:
        indices = np.random.choice(len(hit_pts), size=target_points, replace=False)
        indices.sort()
        hit_pts = hit_pts[indices]
        hit_rng = hit_rng[indices]
        hit_norms = hit_norms[indices]

    # 构建输出点云及色彩
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(hit_pts)

    # 渐变颜色渲染（基于高程色彩）
    z_coords = hit_pts[:, 2]
    z_min, z_max = z_coords.min(), z_coords.max()
    norm_z = (z_coords - z_min) / (z_max - z_min + 1e-6)

    colors = np.zeros((len(hit_pts), 3), dtype=np.float64)
    colors[:, 0] = np.clip(1.5 * norm_z - 0.2, 0.0, 1.0)
    colors[:, 1] = np.clip(1.0 - np.abs(norm_z - 0.5) * 1.5, 0.0, 1.0)
    colors[:, 2] = np.clip(1.2 - 1.5 * norm_z, 0.0, 1.0)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # 统计纵深分布
    rel_x = (hit_pts - radar_pos[None, :]) @ forward_vec
    near_count = int(np.sum(rel_x < 1000.0))
    mid_count = int(np.sum((rel_x >= 1000.0) & (rel_x < 3000.0)))
    far_count = int(np.sum(rel_x >= 3000.0))

    # 统计步长变化
    near_step = float(pitch_degs[1] - pitch_degs[0])
    far_ground_idx = int(np.argmin(np.abs(pitch_degs - (-5.0))))
    far_step = float(pitch_degs[far_ground_idx] - pitch_degs[far_ground_idx - 1])

    info = {
        "mesh_path": mesh_path,
        "radar_pos": radar_pos,
        "radar_height": radar_height,
        "ground_z": ground_z,
        "pitch_mode": pitch_mode,
        "n_azimuth": n_az,
        "n_pitch": n_pitch,
        "point_count": len(hit_pts),
        "near_step_deg": near_step,
        "far_step_deg": far_step,
        "near_count": near_count,
        "mid_count": mid_count,
        "far_count": far_count,
        "z_span": (float(z_min), float(z_max)),
        "max_range": float(hit_rng.max()) if len(hit_rng) > 0 else 0.0
    }
    return pcd, info


def simulate_radar_scans_batch(input_path, output_dir,
                               radar_height=350.0,
                               random_height=False,
                               target_points=15000,
                               azimuth_fov=120.0,
                               pitch_min=-90.0,
                               pitch_max=90.0,
                               pitch_mode="dynamic",
                               pitch_power=1.0,
                               add_noise=0.0):
    """
    批量对文件夹中的 .ply 白模执行雷达对地扫描
    """
    os.makedirs(output_dir, exist_ok=True)

    if os.path.isfile(input_path):
        ply_files = [input_path]
    elif os.path.isdir(input_path):
        ply_files = sorted(glob.glob(os.path.join(input_path, "*.ply")))
    else:
        raise FileNotFoundError(f"找不到输入的路径: {input_path}")

    if not ply_files:
        print(f"[警告] 目录 '{input_path}' 中未找到任何 .ply 文件！")
        return []

    print("=" * 75)
    print("雷达机载对地扫描模拟器 (Airborne Radar Ground Scan Simulator)")
    print("=" * 75)
    print(f"待处理白模数量 : {len(ply_files)} 个")
    print(f"水平扫描视场角 : {azimuth_fov:.1f}° (方位角 [-{azimuth_fov/2:.1f}°, +{azimuth_fov/2:.1f}°])")
    print(f"俯仰扫描角范围 : [{pitch_min:.1f}°, {pitch_max:.1f}°] (正下方 -90° 至 正前方/上方 +90°)")
    print(f"俯仰步长模式   : 【{pitch_mode.upper()}】" +
          (f" (动态步长：近处大步长防密集，远处小步长保细节, 幂参数={pitch_power})" if pitch_mode == 'dynamic' else " (线性等角度)"))
    print(f"基准对地高度   : {radar_height:.1f} 米" + (" (各模型将在 300~400m 间随机采样)" if random_height else ""))
    print(f"目标扫描点数   : 约 {target_points} 个点/地形")
    print(f"点云输出目录   : {os.path.abspath(output_dir)}")
    print("-" * 75)

    output_files = []

    for idx, ply_file in enumerate(ply_files):
        base_name = os.path.splitext(os.path.basename(ply_file))[0]
        out_ply_name = f"scanned_pcd_{base_name}.ply"
        out_ply_path = os.path.join(output_dir, out_ply_name)

        current_h = float(np.random.uniform(300.0, 400.0)) if random_height else radar_height

        pcd, info = simulate_radar_on_mesh(
            ply_file,
            radar_height=current_h,
            target_points=target_points,
            azimuth_fov=azimuth_fov,
            pitch_min=pitch_min,
            pitch_max=pitch_max,
            pitch_mode=pitch_mode,
            pitch_power=pitch_power,
            add_noise=add_noise
        )

        success = o3d.io.write_point_cloud(out_ply_path, pcd, write_ascii=False)
        if not success:
            raise IOError(f"无法保存点云文件到: {out_ply_path}")

        r_pos = info['radar_pos']
        total_pts = info['point_count']
        print(f"[{idx+1}/{len(ply_files)}] 扫描完成: {out_ply_name}")
        print(f"       原始白模: {os.path.basename(ply_file)}")
        print(f"       雷达站位: 【长边中心点】 X={r_pos[0]:.1f}m, Y={r_pos[1]:.1f}m, Z={r_pos[2]:.1f}m (对地高度: {info['radar_height']:.1f}m)")
        print(f"       有效点数: {total_pts} 点 (方位 {info['n_azimuth']} 线 x 俯仰 {info['n_pitch']} 线)")
        if info['pitch_mode'] == 'dynamic':
            print(f"       动态俯仰步长: 近处步长={info['near_step_deg']:.2f}° (防过密), 远处步长={info['far_step_deg']:.3f}° (保细节)")
        print(f"       纵深分布: 近处(<1km) {info['near_count']}点 ({info['near_count']/total_pts*100:.1f}%), "
              f"中距(1~3km) {info['mid_count']}点 ({info['mid_count']/total_pts*100:.1f}%), "
              f"远距(>3km) {info['far_count']}点 ({info['far_count']/total_pts*100:.1f}%)")
        print(f"       最大测程: {info['max_range']:.1f}m, 高程分布: [{info['z_span'][0]:.1f}m ~ {info['z_span'][1]:.1f}m]")
        output_files.append(out_ply_path)

    print("=" * 75)
    print(f"全部 {len(ply_files)} 个地形扫描模拟完成！点云均已保存在: {os.path.abspath(output_dir)}")
    print("=" * 75)
    return output_files


def main():
    parser = argparse.ArgumentParser(
        description="机载雷达地面扫描模拟脚本：支持动态步长俯仰角扫描（近处大步长防过密，远处小步长保细节）。"
    )
    parser.add_argument(
        "-i", "--input",
        type=str,
        default=None,
        help="输入 .ply 地面白模文件或文件夹路径"
    )
    parser.add_argument(
        "-o", "--output_dir",
        type=str,
        default=os.path.join(os.path.dirname(__file__), "output_scanned_pointclouds"),
        help="扫描后点云保存目录 (默认: 当前脚本目录下的 output_scanned_pointclouds)"
    )
    parser.add_argument(
        "-H", "--radar_height",
        type=float,
        default=350.0,
        help="雷达离地高度 (单位: 米，默认 350.0m，通常在 300~400m 之间)"
    )
    parser.add_argument(
        "--random_height",
        action="store_true",
        help="是否在 300~400 米高度范围内为每个地形随机抽取飞行高度"
    )
    parser.add_argument(
        "--target_points",
        type=int,
        default=15000,
        help="每个地形的目标总扫描点云点数 (默认 15000)"
    )
    parser.add_argument(
        "--pitch_mode",
        type=str,
        choices=["dynamic", "uniform"],
        default="dynamic",
        help="俯仰角扫描步长模式：'dynamic' 为动态步长（近疏远密，默认推荐），'uniform' 为均匀等角度步长"
    )
    parser.add_argument(
        "--pitch_power",
        type=float,
        default=1.0,
        help="动态步长地表距离幂律指数 (默认 1.0 为均匀地表间隔，<1.0 偏向近处，>1.0 更偏向远处)"
    )
    parser.add_argument(
        "--azimuth_fov",
        type=float,
        default=120.0,
        help="水平扫描角 (单位: 度，默认 120.0°，即 [-60°, +60°])"
    )
    parser.add_argument(
        "--pitch_min",
        type=float,
        default=-90.0,
        help="俯仰起始扫描角 (单位: 度，默认 -90.0°，即正下方)"
    )
    parser.add_argument(
        "--pitch_max",
        type=float,
        default=90.0,
        help="俯仰结束扫描角 (单位: 度，默认 +90.0°，即正前方/正上方)"
    )
    parser.add_argument(
        "--noise",
        type=float,
        default=0.0,
        help="可选添加测距高斯噪声标准差 (单位: 米，默认 0.0)"
    )

    args = parser.parse_args()

    input_path = args.input
    if input_path is None:
        default_candidate = os.path.join(os.path.dirname(__file__), "output_ground_meshes")
        if os.path.exists(default_candidate):
            input_path = default_candidate
        else:
            try:
                input_path = input("请输入包含 .ply 地面白模的文件夹路径: ").strip()
            except (EOFError, KeyboardInterrupt):
                sys.exit(1)

    if not input_path or not os.path.exists(input_path):
        print(f"[错误] 输入路径不存在: {input_path}")
        sys.exit(1)

    simulate_radar_scans_batch(
        input_path=input_path,
        output_dir=args.output_dir,
        radar_height=args.radar_height,
        random_height=args.random_height,
        target_points=args.target_points,
        azimuth_fov=args.azimuth_fov,
        pitch_min=args.pitch_min,
        pitch_max=args.pitch_max,
        pitch_mode=args.pitch_mode,
        pitch_power=args.pitch_power,
        add_noise=args.noise
    )


if __name__ == "__main__":
    main()
