#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无人机雷达地面扫描与场景数据模拟脚本 (simulate_radar_scanning.py)

功能：
  输入 .ply 地面场景白模文件夹，对每个白模地面场景（X轴 6km，Y轴 8km）：
  1. 模拟无人机位于 (3km, 0) 处（作为 GPS 坐标 (0, 0) 基准点），在离地约 300-400m 高度上，
     笔直平行于 Y 轴向前方飞行，飞行速度为 30 m/s。
  2. 模拟机载雷达扫描地面：
     - 每两秒扫描一帧，共连续扫描 20 帧；
     - 水平扫描视场角 (Azimuth FOV) 120 度；
     - 俯仰扫描角从正下方 (-90度) 到正前方扫描；
     - 采用动态步长 (Dynamic Step Size) 进行俯仰扫描，克服远距离几何发散造成的点云稀疏，保持地面采样均匀；
     - 每帧扫描总点云数约为 15,000 点；
  3. 对于每个地面场景输出：
     - 一个包含每帧扫描后 .ply 地面点云的文件夹 (例如 pointclouds/frame_00_DensePointCloud.ply)；
     - 一个 extend_data.json（记录每帧无人机 GPS 经纬高与航向角，格式对齐 dataset/Wild/04/extend_data.json）；
     - 一个 powerLineGPS.json（模拟场景内的多条高压线与高压塔 GPS 信息，格式对齐 dataset/Wild/04/powerLineGPS.json）。

运行环境：
  推荐使用 conda 环境: /home/asahichinn/miniconda3/envs/pointcloud_607
"""

import os
import sys
import glob
import math
import json
import time
import argparse
import numpy as np

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False


# ==============================================================================
# 1. WGS-84 地理坐标系投影转换 (WGS-84 Geodetic Projection)
# ==============================================================================

class GPSConverter:
    """
    负责场景局部米制坐标 (以 (3000m, 0m) 为 GPS (0, 0) 基准) 与 WGS-84 经纬度之间的双向转换
    """
    def __init__(self, base_lat=0.0, base_lon=0.0):
        self.base_lat = base_lat
        self.base_lon = base_lon
        
        # WGS-84 椭球体常数
        self.a = 6378137.0  # 半长轴 (m)
        self.f = 1.0 / 298.257223563  # 扁率
        self.e2 = 2.0 * self.f - self.f * self.f  # 第一偏心率平方
        
        lat0_rad = math.radians(base_lat)
        sin_lat = math.sin(lat0_rad)
        
        # 纬度 1 度对应的米数 (子午圈曲率半径 M)
        self.m_per_deg_lat = (
            math.pi * self.a * (1.0 - self.e2) / (180.0 * ((1.0 - self.e2 * sin_lat ** 2) ** 1.5))
        )
        # 经度 1 度对应的米数 (卯酉圈曲率半径 N * cos(lat))
        self.m_per_deg_lon = (
            math.pi * self.a * math.cos(lat0_rad) / (180.0 * math.sqrt(1.0 - self.e2 * sin_lat ** 2))
        )

    def xy_to_gps(self, x_scene, y_scene, z_scene=0.0):
        """
        场景局部坐标 (X, Y) 转换为 WGS-84 (Lat, Lon, Alt)。
        以场景 (3000m, 0m) 处作为 GPS 基准原点 (base_lat, base_lon)。
        """
        dx = x_scene - 3000.0  # 向东相对米数
        dy = y_scene - 0.0     # 向北相对米数
        
        lat = self.base_lat + dy / self.m_per_deg_lat
        lon = self.base_lon + dx / self.m_per_deg_lon
        alt = float(z_scene)
        return lat, lon, alt


# ==============================================================================
# 2. 地形高程查询与表面采样器
# ==============================================================================

class TerrainElevationSampler:
    """
    从读取的白模网格中构建快速高程查询器 (基于 KD 树或网格就近/插值)
    """
    def __init__(self, vertices, triangles):
        self.vertices = vertices
        self.triangles = triangles
        
        # 预先构建XY平面网格或者利用KD-Tree进行快速高程插值
        if HAS_OPEN3D:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(vertices[:, :3])
            self.kdtree = o3d.geometry.KDTreeFlann(pcd)
        else:
            self.kdtree = None

    def get_elevation_at(self, x, y):
        """根据 (x, y) 获取地形表面的高程 Z"""
        if self.kdtree is not None:
            # 查找最近的3个网格顶点并按反距离权重插值
            _, idxs, dists2 = self.kdtree.search_knn_vector_3d([x, y, 0.0], 3)
            idxs = np.asarray(idxs)
            weights = 1.0 / (np.sqrt(np.asarray(dists2)) + 1e-6)
            weights /= np.sum(weights)
            z = float(np.sum(self.vertices[idxs, 2] * weights))
            return z
        else:
            # 简易就近查找
            d2 = (self.vertices[:, 0] - x) ** 2 + (self.vertices[:, 1] - y) ** 2
            nearest_idx = np.argmin(d2)
            return float(self.vertices[nearest_idx, 2])


# ==============================================================================
# 3. 雷达动态步长扫描与光线投射模拟
# ==============================================================================

def generate_dynamic_radar_rays(drone_pos, fov_horizontal_deg=120.0,
                                num_azimuth=150, num_pitch=100,
                                max_ground_range=4500.0, power_factor=1.12):
    """
    生成带动态步长的雷达扫描光线扇区。
    
    动态步长原理：
      当俯仰扫描从正下方 (-90度) 逐渐抬高朝向前方时，光线与地面的交点距离 y_ground 随俯仰角
      以 tan 函数关系急剧增加 (y = H / tan(psi))。若采用等角度扫描，远方地面点距会以 1/sin^2(psi)
      呈平方量级严重发散，导致远距离极度稀疏。
      因此，本算法在水平向地面距离空间进行均匀/受控采样，并反推每条光线所需的俯仰角 psi = arctan(H / y)，
      从而在远离无人机时自适应缩减俯仰角步长 (动态加密角分辨率)，保证地面点云在远距离依然保持连续且均匀的密度。
    
    :param drone_pos: (X_uav, Y_uav, Z_uav)
    :param fov_horizontal_deg: 水平扫描角范围 (默认 120 度: -60° 到 +60°)
    :param num_azimuth: 水平角采样线数 (默认 150)
    :param num_pitch: 俯仰动态采样步数 (默认 100，总光线数 = 150 * 100 = 15000)
    :param max_ground_range: 最远地面扫描前向探测距离 (米，默认 4500m)
    :return: rays 数组，shape (N, 6)，每行 [ox, oy, oz, dx, dy, dz]
    """
    x_uav, y_uav, z_uav = drone_pos
    H = max(100.0, z_uav)  # 无人机相对参考高度
    
    # 1. 水平方位角 (Azimuth) 均匀分布在 [-60°, +60°]
    half_fov_rad = math.radians(fov_horizontal_deg / 2.0)
    azimuths = np.linspace(-half_fov_rad, half_fov_rad, num_azimuth, dtype=np.float32)
    
    # 2. 俯仰动态步长：从正下方 (y=0m) 到前方 (y=max_ground_range)
    # 使用非线性归一化参数 s in [0, 1]，使近处到远处过渡平滑自然
    s = np.linspace(0.0, 1.0, num_pitch, dtype=np.float32)
    y_ground_samples = max_ground_range * (s ** power_factor)
    
    # 由几何关系反求俯仰视线角 (负俯仰角，相对水平面向下为负)
    # 当 y=0 时，psi = 90° (指向正下方，俯仰角 -90°)
    # 当 y=max 时，psi 趋近小俯角 (指向前方远端)
    depression_angles = np.arctan2(H, np.maximum(y_ground_samples, 0.05))
    
    # 构建所有光线组合: num_pitch x num_azimuth = 15,000 条光线
    cos_dep = np.cos(depression_angles)  # shape (num_pitch,)
    sin_dep = np.sin(depression_angles)  # shape (num_pitch,)
    
    sin_az = np.sin(azimuths)            # shape (num_azimuth,)
    cos_az = np.cos(azimuths)            # shape (num_azimuth,)
    
    # 笛卡尔坐标方向向量:
    # dx = cos(dep) * sin(az)
    # dy = cos(dep) * cos(az)  (由于无人机沿 +Y 飞行，0度方位角朝向 +Y)
    # dz = -sin(dep)           (朝向地面下方为负)
    dx = np.outer(cos_dep, sin_az).flatten()
    dy = np.outer(cos_dep, cos_az).flatten()
    dz = np.outer(-sin_dep, np.ones_like(azimuths)).flatten()
    
    total_rays = len(dx)
    rays = np.zeros((total_rays, 6), dtype=np.float32)
    rays[:, 0] = x_uav
    rays[:, 1] = y_uav
    rays[:, 2] = z_uav
    rays[:, 3] = dx
    rays[:, 4] = dy
    rays[:, 5] = dz
    
    return rays


def simulate_radar_frame_points(raycast_scene, drone_pos, frame_idx,
                                fov_horizontal=120.0,
                                target_points=15000,
                                max_range=4500.0,
                                coord_mode="local",
                                flight_alt=350.0):
    """
    执行单帧雷达扫描与光线碰撞，生成雷达地面点云
    :param raycast_scene: Open3D RaycastingScene
    :param drone_pos: (X_uav, Y_uav, Z_uav)
    :param coord_mode: 'local' (无人机本体局部坐标系) 或 'scene' (场景绝对坐标系)
    :param flight_alt: 雷达在局部点云坐标系中的高程 Z，雷达所在局部坐标严格为 (0, 0, flight_alt)
    :return: points (N, 3), powers (N,)
    """
    # 动态调配光线阵列总数接近 15000 (例如 150 x 100 = 15000)
    num_azimuth = 150
    num_pitch = target_points // num_azimuth
    
    rays = generate_dynamic_radar_rays(
        drone_pos,
        fov_horizontal_deg=fov_horizontal,
        num_azimuth=num_azimuth,
        num_pitch=num_pitch,
        max_ground_range=max_range
    )
    
    # 利用 Open3D 高速 BVH 求交
    rays_tensor = o3d.core.Tensor(rays, dtype=o3d.core.Dtype.Float32)
    ans = raycast_scene.cast_rays(rays_tensor)
    t_hits = ans['t_hit'].numpy()
    
    # 过滤未碰撞光线 (超出地形网格边界)
    valid_mask = np.isfinite(t_hits) & (t_hits > 0.1) & (t_hits < (max_range * 1.5))
    valid_t = t_hits[valid_mask]
    valid_rays = rays[valid_mask]
    
    # 计算交点世界坐标: P_hit = Origin + t * Dir
    hit_x = valid_rays[:, 0] + valid_t * valid_rays[:, 3]
    hit_y = valid_rays[:, 1] + valid_t * valid_rays[:, 4]
    hit_z = valid_rays[:, 2] + valid_t * valid_rays[:, 5]
    
    # 约束交点在场景 6km x 8km 范围之内
    in_bounds = (
        (hit_x >= 0.0) & (hit_x <= 6000.0) &
        (hit_y >= 0.0) & (hit_y <= 8000.0)
    )
    hit_x = hit_x[in_bounds]
    hit_y = hit_y[in_bounds]
    hit_z = hit_z[in_bounds]
    valid_t = valid_t[in_bounds]
    
    # 模拟雷达回波强度 (Power / dB):
    # 雷达气象/散射公式 P_r 随距离衰减，并附加高斯纹理噪声
    # Wild/04 数据集中的 power 值范围集中在 [65, 95] dB
    dist_norm = np.clip(valid_t / 1000.0, 0.3, 5.0)
    base_power = 92.0 - 15.0 * np.log10(dist_norm)
    noise = np.random.normal(0.0, 2.5, size=len(hit_x))
    powers = np.clip(base_power + noise, 55.0, 98.0).astype(np.float32)
    
    if coord_mode == "local":
        # 转换到机体局部坐标系：
        # 雷达所在局部坐标严格为 (0, 0, flight_alt)
        # x 为横向左右偏差 (雷达位置 x=0)
        # y 为沿航向正前方距离 (雷达位置 y=0)
        # z 为高程，地面点高程 lz = hit_z - drone_pos[2] + flight_alt (雷达位置处 z=flight_alt)
        lx = hit_x - drone_pos[0]
        ly = hit_y - drone_pos[1]
        lz = hit_z - drone_pos[2] + flight_alt
        pts = np.stack([lx, ly, lz], axis=-1).astype(np.float32)
    else:
        pts = np.stack([hit_x, hit_y, hit_z], axis=-1).astype(np.float32)
        
    return pts, powers


def save_pointcloud_ply(filepath, points, powers=None, write_ascii=True):
    """
    保存点云为 .ply 文件 (支持 x, y, z, power 属性，完美兼容 Wild/04 格式)
    """
    os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
    num_pts = len(points)
    
    if write_ascii:
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write("comment Simulated UAV radar ground point cloud\n")
            f.write(f"element vertex {num_pts}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            if powers is not None:
                f.write("property float power\n")
            f.write("end_header\n")
            if powers is not None:
                for (x, y, z), p in zip(points, powers):
                    f.write(f"{x:.6f} {y:.6f} {z:.6f} {p:.6f}\n")
            else:
                for x, y, z in points:
                    f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")
    else:
        # 二进制写入
        with open(filepath, 'wb') as f:
            header = (
                f"ply\nformat binary_little_endian 1.0\ncomment Simulated UAV radar point cloud\n"
                f"element vertex {num_pts}\n"
                f"property float x\nproperty float y\nproperty float z\n"
            )
            if powers is not None:
                header += "property float power\n"
            header += "end_header\n"
            f.write(header.encode('ascii'))
            
            if powers is not None:
                data = np.column_stack([points, powers]).astype('<f4')
            else:
                data = points.astype('<f4')
            f.write(data.tobytes())


# ==============================================================================
# 4. 高压线与高压铁塔模拟生成 (Power Line & Pylon Simulation)
# ==============================================================================

def simulate_power_lines(terrain_sampler, gps_converter, num_lines=3, span_distance=350.0):
    """
    在 6km * 8km 场景内模拟多条合法的高压线与高压塔。
    
    合法规则设计：
      1. 高压线为连续直线走向，跨越场景内部不同地理区域；
      2. 避免与飞行路径完全重合，包含横跨航道、斜跨或平行伴行的高压廊道；
      3. 高压铁塔沿高压线按固定工程间距 (默认 350m) 连续布设；
      4. 高压塔的局部高程基于地面白模对应位置的真实高程；
      5. 输出格式对齐 dataset/Wild/04/powerLineGPS.json。
    """
    power_lines_data = {}
    
    # 预设几个典型的合理高压走廊模板 (端点在 [0, 6000] x [0, 8000] 内)
    corridor_templates = [
        # HVL_1: 从西南斜向横跨至东北 (横跨无人机航迹)
        {"start": (800.0, 1200.0), "end": (5200.0, 3200.0)},
        # HVL_2: 位于无人机西侧平行伴行偏转的走廊
        {"start": (1800.0, 400.0), "end": (2200.0, 7600.0)},
        # HVL_3: 场景中北部东西向横断走廊
        {"start": (600.0, 4800.0), "end": (5400.0, 5600.0)},
        # HVL_4: 场景东侧走廊
        {"start": (4200.0, 800.0), "end": (4500.0, 7200.0)},
    ]
    
    for line_idx in range(1, num_lines + 1):
        line_key = f"HVL_{line_idx}"
        tpl = corridor_templates[(line_idx - 1) % len(corridor_templates)]
        
        # 引入适度随机偏移，使不同场景的高压线有所区别
        p_start = np.array(tpl["start"]) + np.random.uniform(-150.0, 150.0, size=2)
        p_end = np.array(tpl["end"]) + np.random.uniform(-150.0, 150.0, size=2)
        
        # 边界保护
        p_start = np.clip(p_start, [200.0, 200.0], [5800.0, 7800.0])
        p_end = np.clip(p_end, [200.0, 200.0], [5800.0, 7800.0])
        
        # 计算走廊长度与铁塔数量
        delta = p_end - p_start
        total_length = float(np.linalg.norm(delta))
        num_towers = int(total_length // span_distance) + 1
        
        towers = []
        unit_vec = delta / total_length
        
        for t_idx in range(1, num_towers + 1):
            dist = (t_idx - 1) * span_distance
            if dist > total_length:
                dist = total_length
                
            curr_pos = p_start + unit_vec * dist
            x_tower, y_tower = float(curr_pos[0]), float(curr_pos[1])
            
            # 获取该塔位所在地面高程
            z_ground = terrain_sampler.get_elevation_at(x_tower, y_tower)
            
            # 计算以 (3000, 0) 为 GPS (0, 0) 原点的 WGS-84 经纬度
            lat, lon, _ = gps_converter.xy_to_gps(x_tower, y_tower, z_ground)
            
            towers.append({
                "point_index": t_idx,
                "Latitude": round(lat, 8),
                "Longitude": round(lon, 8),
                "Altitude": round(z_ground, 2)
            })
            
        power_lines_data[line_key] = towers
        
    return power_lines_data


# ==============================================================================
# 5. 单个场景完整雷达扫描流程
# ==============================================================================

def process_single_terrain_scene(ply_path, output_base_dir, gps_converter,
                                 speed=30.0, h_agl=350.0, total_frames=20,
                                 interval_sec=2.0, points_per_frame=15000,
                                 fov_horizontal=120.0, coord_mode="local",
                                 radar_z_mode="agl",
                                 num_power_lines=3, span_distance=350.0,
                                 save_ascii_ply=True):
    """
    处理单个地面场景白模，生成 20 帧雷达点云、extend_data.json 及 powerLineGPS.json
    """
    scene_stem = os.path.splitext(os.path.basename(ply_path))[0]
    scene_out_dir = os.path.join(output_base_dir, scene_stem)
    pointclouds_dir = os.path.join(scene_out_dir, "pointclouds")
    os.makedirs(pointclouds_dir, exist_ok=True)
    
    print("\n" + "=" * 75)
    print(f"[*] 正在处理场景: {scene_stem}")
    print(f"    白模路径: {ply_path}")
    print(f"    输出目录: {scene_out_dir}")
    print("=" * 75)
    
    # 1. 加载地形网格
    mesh = o3d.io.read_triangle_mesh(ply_path)
    if not mesh.has_vertices():
        raise ValueError(f"读取网格失败或无顶点数据: {ply_path}")
    
    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)
    print(f"[*] 成功加载地面白模: {len(vertices)} 顶点, {len(triangles)} 三角面")
    
    # 2. 构建 RaycastingScene 与高程查询器
    t_mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    raycast_scene = o3d.t.geometry.RaycastingScene()
    raycast_scene.add_triangles(t_mesh)
    
    terrain_sampler = TerrainElevationSampler(vertices, triangles)
    
    # 3. 规划无人机航线
    # 起点 (3000, 0)，以 30m/s 笔直平行于 Y 轴向前方飞行
    # 确定航迹下方的基准地面高度
    start_ground_z = terrain_sampler.get_elevation_at(3000.0, 0.0)
    # 无人机巡航高度 (绝对高程) = 起始地面高度 + AGL高度 (300-400m，默认350m)
    flight_cruise_alt = start_ground_z + h_agl
    
    # 雷达在局部点云坐标系中的 Z 坐标基准值
    target_radar_z = h_agl if radar_z_mode == "agl" else flight_cruise_alt

    print(f"[*] 无人机飞行参数:")
    print(f"    - 起点位置: (X=3000m, Y=0m), 起点地面高度: {start_ground_z:.1f} m")
    print(f"    - 飞行高度: 离地 {h_agl:.1f} m (绝对海拔高度 {flight_cruise_alt:.1f} m)")
    print(f"    - 雷达局部坐标位置: (0.0, 0.0, {target_radar_z:.1f})")
    print(f"    - 飞行速度: {speed:.1f} m/s, 扫描周期: {interval_sec:.1f} s/帧, 共 {total_frames} 帧")
    print(f"    - 航线总长: {speed * interval_sec * (total_frames - 1):.1f} m (Y: 0m -> {speed * interval_sec * (total_frames - 1):.1f}m)")
    
    extend_data = {}
    
    # 4. 执行每两秒一帧的雷达扫描模拟
    print("\n[*] 开始进行雷达扫描模拟 (动态步长俯仰，120度水平FOV)...")
    for k in range(total_frames):
        t_sec = k * interval_sec
        y_uav = speed * t_sec
        x_uav = 3000.0
        z_uav = flight_cruise_alt
        drone_pos = (x_uav, y_uav, z_uav)
        
        # 计算该帧无人机 GPS 坐标 (以 (3000, 0) 为 (0, 0) 原点)
        lat_uav, lon_uav, alt_uav = gps_converter.xy_to_gps(x_uav, y_uav, z_uav)
        # 航向角: 沿 +Y 轴平行笔直飞行，航向角为 0.0 度 (正北)
        course_uav = 0.0
        
        # 确定本帧点云中雷达所在的局部坐标高程 (0, 0, [飞行高度])
        current_flight_alt = h_agl if radar_z_mode == "agl" else alt_uav

        # 模拟雷达扫描获取地面点云
        pts, powers = simulate_radar_frame_points(
            raycast_scene=raycast_scene,
            drone_pos=drone_pos,
            frame_idx=k,
            fov_horizontal=fov_horizontal,
            target_points=points_per_frame,
            max_range=4500.0,
            coord_mode=coord_mode,
            flight_alt=current_flight_alt
        )
        
        # 构造帧文件名与保存点云
        frame_key = f"frame_{k:02d}_DensePointCloud"
        ply_filename = f"{frame_key}.ply"
        ply_save_path = os.path.join(pointclouds_dir, ply_filename)
        save_pointcloud_ply(ply_save_path, pts, powers, write_ascii=save_ascii_ply)
        
        # 记录 extend_data.json
        extend_data[frame_key] = {
            "ply_filename": ply_filename,
            "gps_info": {
                "Latitude": round(lat_uav, 8),
                "Longitude": round(lon_uav, 8),
                "Altitude": round(alt_uav, 3),
                "Course": course_uav
            }
        }
        
        print(f"  [Frame {k+1:02d}/{total_frames:02d}] Y={y_uav:6.1f}m | GPS: ({lat_uav:.6f}, {lon_uav:.6f}, Alt={alt_uav:.1f}m) | 点云点数: {len(pts)}")
    
    # 保存 extend_data.json
    extend_json_path = os.path.join(scene_out_dir, "extend_data.json")
    with open(extend_json_path, 'w', encoding='utf-8') as f:
        json.dump(extend_data, f, indent=2, ensure_ascii=False)
    print(f"\n[+] 成功写入: {extend_json_path}")
    
    # 5. 模拟生成高压线与高压塔 GPS 数据
    print("[*] 正在模拟场景内高压线走向及铁塔位置...")
    power_lines_data = simulate_power_lines(
        terrain_sampler=terrain_sampler,
        gps_converter=gps_converter,
        num_lines=num_power_lines,
        span_distance=span_distance
    )
    
    power_json_path = os.path.join(scene_out_dir, "powerLineGPS.json")
    with open(power_json_path, 'w', encoding='utf-8') as f:
        json.dump(power_lines_data, f, indent=2, ensure_ascii=False)
    print(f"[+] 成功写入: {power_json_path} (包含 {len(power_lines_data)} 条高压线)")
    for line_k, tower_list in power_lines_data.items():
        print(f"    - {line_k}: 共 {len(tower_list)} 座高压塔, 首塔 GPS: ({tower_list[0]['Latitude']:.6f}, {tower_list[0]['Longitude']:.6f}), 末塔 GPS: ({tower_list[-1]['Latitude']:.6f}, {tower_list[-1]['Longitude']:.6f})")

    print(f"\n[OK] 场景 {scene_stem} 仿真数据已全部成功生成！")
    return scene_out_dir


# ==============================================================================
# 6. 主程序命令行入口
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="无人机雷达地面扫描与高压线GPS仿真脚本：输入地面白模，生成20帧雷达点云、extend_data.json与powerLineGPS.json"
    )
    parser.add_argument("-i", "--input_dir", type=str, default="./terrain_models",
                        help="包含地面场景白模(.ply)的文件夹路径 (默认: ./terrain_models)")
    parser.add_argument("-o", "--output_dir", type=str, default="./sim_radar_output",
                        help="仿真数据输出根目录 (默认: ./sim_radar_output)")
    parser.add_argument("--speed", type=float, default=30.0,
                        help="无人机飞行速度 (米/秒，默认: 30.0)")
    parser.add_argument("--height", type=float, default=350.0,
                        help="无人机相对地面巡航高度 (米，默认: 350.0，要求300-400米)")
    parser.add_argument("--frames", type=int, default=20,
                        help="扫描帧数 (默认: 20)")
    parser.add_argument("--interval", type=float, default=2.0,
                        help="每帧扫描时间间隔 (秒，默认: 2.0)")
    parser.add_argument("--points", type=int, default=15000,
                        help="每帧目标雷达点云点数 (默认: 15000)")
    parser.add_argument("--fov", type=float, default=120.0,
                        help="雷达水平扫描角 (度，默认: 120.0)")
    parser.add_argument("--base_lat", type=float, default=0.0,
                        help="场景 (3000m, 0m) 对应的 GPS 纬度基准 (默认: 0.0)")
    parser.add_argument("--base_lon", type=float, default=0.0,
                        help="场景 (3000m, 0m) 对应的 GPS 经度基准 (默认: 0.0)")
    parser.add_argument("--coord_mode", type=str, default="local", choices=["local", "scene"],
                        help="点云坐标系: 'local' (机体相对坐标，对齐 Wild/04) 或 'scene' (场景全局坐标) (默认: local)")
    parser.add_argument("--radar_z_mode", type=str, default="agl", choices=["agl", "abs"],
                        help="雷达局部坐标Z轴基准: 'agl' (使用相对地面飞行高度，如350m，雷达位于(0,0,height)) 或 'abs' (使用绝对海拔高度，雷达位于(0,0,Altitude)) (默认: agl)")
    parser.add_argument("--power_lines", type=int, default=3,
                        help="生成的合法高压线数量 (默认: 3)")
    parser.add_argument("--tower_spacing", type=float, default=350.0,
                        help="高压铁塔之间档距 (米，默认: 350.0)")
    parser.add_argument("--binary", action="store_true",
                        help="将点云保存为二进制 PLY (默认保存为 ASCII PLY，方便查看)")

    args = parser.parse_args()

    input_dir = os.path.abspath(args.input_dir)
    output_dir = os.path.abspath(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    # 检索输入目录中的所有 .ply 白模文件
    ply_files = sorted(glob.glob(os.path.join(input_dir, "*.ply")))
    if not ply_files:
        print(f"[!] 错误: 在输入文件夹中未找到任何 .ply 白模文件: {input_dir}")
        print("    请先运行 generate_terrain_meshes.py 生成白模，或使用 -i 指定正确的白模文件夹。")
        sys.exit(1)

    print("=" * 75)
    print("无人机雷达机载扫描与场景数据模拟器 (UAV Radar Ground Scanning Simulator)")
    print(f"  * 输入白模文件夹: {input_dir}")
    print(f"  * 检索到白模数量: {len(ply_files)} 个")
    print(f"  * 输出保存目录: {output_dir}")
    print(f"  * 坐标系基准: (3km, 0) 映射为 GPS ({args.base_lat:.6f}, {args.base_lon:.6f})")
    print(f"  * 无人机航速: {args.speed} m/s, 离地高度: {args.height} m, 扫描帧数: {args.frames} 帧 (间隔 {args.interval}s)")
    print(f"  * 雷达点云位置基准: (0, 0, [{args.radar_z_mode.upper()}模式下的飞行高度])")
    print(f"  * 雷达视场: 水平 {args.fov}°，动态步长俯仰，每帧约 {args.points} 点")
    print("=" * 75)

    gps_converter = GPSConverter(base_lat=args.base_lat, base_lon=args.base_lon)
    start_total_time = time.time()

    for idx, ply_path in enumerate(ply_files, 1):
        t0 = time.time()
        process_single_terrain_scene(
            ply_path=ply_path,
            output_base_dir=output_dir,
            gps_converter=gps_converter,
            speed=args.speed,
            h_agl=args.height,
            total_frames=args.frames,
            interval_sec=args.interval,
            points_per_frame=args.points,
            fov_horizontal=args.fov,
            coord_mode=args.coord_mode,
            radar_z_mode=args.radar_z_mode,
            num_power_lines=args.power_lines,
            span_distance=args.tower_spacing,
            save_ascii_ply=(not args.binary)
        )
        print(f"[*] 场景 [{idx}/{len(ply_files)}] 完成，耗时: {time.time() - t0:.2f} 秒")

    total_elapsed = time.time() - start_total_time
    print("\n" + "=" * 75)
    print(f"全部 {len(ply_files)} 个场景的雷达点云与 GPS 元数据已全部成功生成！总耗时: {total_elapsed:.2f} 秒")
    print(f"结果保存在: {output_dir}")
    print("=" * 75)


if __name__ == "__main__":
    main()
