#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从 extend_data.json 按顺序提取每帧 GPS 经纬高 (Latitude, Longitude, Altitude) 与航向角 (Course)，
转换至局部东-北-高 (Local UTM / ENU) 坐标系，并生成：
  1. 终端格式化数据表 (经纬高、航向角、相对米制坐标、分段/累计航程)
  2. 交互式三维轨迹网页 (flight_trajectory_3d.html，零外部依赖，支持鼠标旋转/缩放/帧检视)
  3. 高清三维飞行轨迹静态图 (flight_trajectory_3d.png / .svg)
  4. 三维轨迹与航向箭头网格模型 (flight_trajectory_3d.ply，可直接用 Open3D / CloudCompare 查看，支持 --show 弹窗)
"""

import os
import re
import sys
import json
import math
import argparse
import numpy as np

try:
    from pyproj import Transformer
    HAS_PYPROJ = True
except ImportError:
    HAS_PYPROJ = False

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False

try:
    import matplotlib
    if "--show" not in sys.argv:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

try:
    from PIL import Image, ImageDraw, ImageFont
    HAS_PIL = True
except ImportError:
    HAS_PIL = False


def parse_frame_sort_key(frame_key, idx):
    """
    解析帧名中的时间戳与脉冲起止序号以保证严格按采集顺序排列；
    例如: R_20230131130323_209_45575-48302_SignalProc_DensePointCloud
    """
    m = re.search(r"R_(\d+)_(\d+)_(\d+)-(\d+)", frame_key)
    if m:
        return (0, m.group(1), int(m.group(2)), int(m.group(3)), int(m.group(4)), idx)
    return (1, "", 0, 0, 0, idx)


def extract_trajectory_from_json(json_path):
    """按顺序提取每帧的经纬高、航向角，并计算相对首帧的局部米制坐标 (East, North, Alt)。"""
    if not os.path.exists(json_path):
        raise FileNotFoundError(f"未找到 JSON 文件: {json_path}")

    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    if not data:
        raise ValueError(f"JSON 文件为空: {json_path}")

    indexed_keys = list(enumerate(data.keys()))
    indexed_keys.sort(key=lambda item: parse_frame_sort_key(item[1], item[0]))

    records = []
    for order_idx, (_, key) in enumerate(indexed_keys, start=1):
        entry = data[key]
        gps = entry.get("gps_info", {})
        lat = float(gps["Latitude"])
        lon = float(gps["Longitude"])
        alt = float(gps["Altitude"])
        course = float(gps["Course"])

        m_range = re.search(r"_(\d+-\d+)_", key)
        short_label = m_range.group(1) if m_range else f"Frame {order_idx}"

        records.append({
            "frame_idx": order_idx,
            "frame_key": key,
            "short_label": short_label,
            "csv_filename": entry.get("csv_filename", ""),
            "ply_filename": entry.get("ply_filename", ""),
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "course_deg": course,
            "course_360": course % 360.0,
        })

    # 转换到局部东-北-高 (Local UTM / ENU) 坐标系，以第 1 帧为水平原点 (0, 0)
    lon0, lat0, alt0 = records[0]["lon"], records[0]["lat"], records[0]["alt"]
    utm_zone = int((lon0 + 180.0) / 6.0) + 1
    crs_utm = f"+proj=utm +zone={utm_zone} +datum=WGS84 +units=m +no_defs"

    if HAS_PYPROJ:
        transformer = Transformer.from_crs("EPSG:4326", crs_utm, always_xy=True)
        x0, y0 = transformer.transform(lon0, lat0)
        for r in records:
            x_utm, y_utm = transformer.transform(r["lon"], r["lat"])
            r["x_east"] = float(x_utm - x0)
            r["y_north"] = float(y_utm - y0)
            r["z_rel"] = float(r["alt"] - alt0)
            r["x_utm"] = float(x_utm)
            r["y_utm"] = float(y_utm)
    else:
        a = 6378137.0
        e2 = 0.00669437999014
        lat0_rad = math.radians(lat0)
        sin_lat = math.sin(lat0_rad)
        m_per_deg_lat = math.pi * a * (1.0 - e2) / (180.0 * ((1.0 - e2 * sin_lat ** 2) ** 1.5))
        m_per_deg_lon = math.pi * a * math.cos(lat0_rad) / (180.0 * math.sqrt(1.0 - e2 * sin_lat ** 2))
        for r in records:
            r["x_east"] = float((r["lon"] - lon0) * m_per_deg_lon)
            r["y_north"] = float((r["lat"] - lat0) * m_per_deg_lat)
            r["z_rel"] = float(r["alt"] - alt0)
            r["x_utm"] = r["x_east"]
            r["y_utm"] = r["y_north"]

    # 计算航向单位向量 (与项目中 gps_utils.py 保持一致: 正北为 0°, 顺时针为正)
    # 数学角 yaw_math = 90° - Course => dx = sin(Course), dy = cos(Course)
    cum_dist_3d = 0.0
    for i, r in enumerate(records):
        rad = math.radians(r["course_deg"])
        r["dir_east"] = float(math.sin(rad))
        r["dir_north"] = float(math.cos(rad))
        if i == 0:
            r["seg_dist_2d"] = 0.0
            r["seg_dist_3d"] = 0.0
            r["track_bearing_deg"] = None
        else:
            prev = records[i - 1]
            dx = r["x_east"] - prev["x_east"]
            dy = r["y_north"] - prev["y_north"]
            dz = r["alt"] - prev["alt"]
            d2 = math.hypot(dx, dy)
            d3 = math.sqrt(dx * dx + dy * dy + dz * dz)
            cum_dist_3d += d3
            r["seg_dist_2d"] = float(d2)
            r["seg_dist_3d"] = float(d3)
            r["track_bearing_deg"] = float(math.degrees(math.atan2(dx, dy)))
        r["cum_dist_3d"] = float(cum_dist_3d)

    return records, crs_utm


def print_trajectory_table(records):
    """在终端打印按帧序提取的 GPS 经纬高与航向角明细表。"""
    print("=" * 124)
    print(
        f"{'帧序':<4} | {'脉冲段标识':<13} | {'纬度 Latitude (°N)':<18} | {'经度 Longitude (°E)':<19} | "
        f"{'高度 Alt (m)':<12} | {'航向角 Course (°)':<19} | {'ΔEast (m)':<10} | {'ΔNorth (m)':<10} | {'段距 (m)':<8}"
    )
    print("-" * 124)
    for r in records:
        print(
            f"F{r['frame_idx']:<3d} | {r['short_label']:<13} | "
            f"{r['lat']:<18.10f} | {r['lon']:<19.10f} | "
            f"{r['alt']:<12.4f} | {r['course_deg']:>+9.4f}° ({r['course_360']:>6.2f}°) | "
            f"{r['x_east']:>+10.2f} | {r['y_north']:>+10.2f} | {r['seg_dist_3d']:>8.2f}"
        )
    print("=" * 124)
    total_2d = sum(r["seg_dist_2d"] for r in records)
    total_3d = records[-1]["cum_dist_3d"]
    delta_alt = records[-1]["alt"] - records[0]["alt"]
    print(
        f"统计汇总: 共 {len(records)} 帧 | 水平总航程: {total_2d:.2f} m | "
        f"三维总航程: {total_3d:.2f} m | 净爬升高度: {delta_alt:+.2f} m "
        f"({records[0]['alt']:.2f} m -> {records[-1]['alt']:.2f} m)"
    )


def save_open3d_ply_and_optionally_show(records, ply_out_path, show_window=False):
    """使用 Open3D 生成带颜色的三维飞行轨迹管线与机头航向角三维箭头模型 (.ply)。"""
    if not HAS_OPEN3D:
        return None

    meshes = []
    pts = np.array([[r["x_east"], r["y_north"], r["alt"]] for r in records], dtype=np.float64)
    span_xy = max(np.ptp(pts[:, 0]), np.ptp(pts[:, 1]), 50.0)
    arrow_len = span_xy * 0.12
    tube_radius = max(span_xy * 0.004, 0.8)

    def align_z_to_vector(vec):
        v = np.asarray(vec, dtype=np.float64)
        norm = np.linalg.norm(v)
        if norm < 1e-9:
            return np.eye(3)
        v = v / norm
        z_axis = np.array([0.0, 0.0, 1.0])
        if np.allclose(v, z_axis):
            return np.eye(3)
        if np.allclose(v, -z_axis):
            return np.diag([1.0, -1.0, -1.0])
        axis = np.cross(z_axis, v)
        axis /= np.linalg.norm(axis)
        angle = math.acos(np.clip(np.dot(z_axis, v), -1.0, 1.0))
        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0],
        ])
        return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)

    # 1. 轨迹连线圆柱段 (蓝色)
    for i in range(len(pts) - 1):
        p0, p1 = pts[i], pts[i + 1]
        seg = p1 - p0
        length = float(np.linalg.norm(seg))
        if length > 1e-6:
            cyl = o3d.geometry.TriangleMesh.create_cylinder(radius=tube_radius, height=length, resolution=16)
            cyl.paint_uniform_color([0.15, 0.39, 0.92])
            cyl.rotate(align_z_to_vector(seg), center=[0, 0, 0])
            cyl.translate(0.5 * (p0 + p1))
            meshes.append(cyl)

    # 2. 每帧位置球体 + 航向角三维箭头 (红色)
    for i, r in enumerate(records):
        pos = pts[i]
        sph = o3d.geometry.TriangleMesh.create_sphere(radius=tube_radius * 2.4, resolution=16)
        if i == 0:
            sph.paint_uniform_color([0.09, 0.64, 0.29])  # 起点绿色
        elif i == len(records) - 1:
            sph.paint_uniform_color([0.92, 0.35, 0.05])  # 终点橙色
        else:
            sph.paint_uniform_color([0.15, 0.39, 0.92])
        sph.translate(pos)
        meshes.append(sph)

        heading_vec = np.array([r["dir_east"], r["dir_north"], 0.0], dtype=np.float64)
        arr = o3d.geometry.TriangleMesh.create_arrow(
            cylinder_radius=tube_radius * 0.9,
            cone_radius=tube_radius * 2.3,
            cylinder_height=arrow_len * 0.72,
            cone_height=arrow_len * 0.28,
            resolution=16,
        )
        arr.paint_uniform_color([0.86, 0.15, 0.15])
        arr.rotate(align_z_to_vector(heading_vec), center=[0, 0, 0])
        arr.translate(pos)
        meshes.append(arr)

    combined = meshes[0]
    for m in meshes[1:]:
        combined += m
    combined.compute_vertex_normals()

    os.makedirs(os.path.dirname(os.path.abspath(ply_out_path)), exist_ok=True)
    o3d.io.write_triangle_mesh(ply_out_path, combined)
    print(f"[OK] Open3D 三维轨迹与航向箭头模型已保存至: {ply_out_path}")

    if show_window:
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=arrow_len * 0.8, origin=pts[0])
        o3d.visualization.draw_geometries(
            [combined, coord_frame],
            window_name="3D Flight Trajectory & Course Heading (Open3D)",
            width=1280,
            height=800,
        )
    return ply_out_path


def save_svg_and_png_plot(records, svg_path, png_path):
    """生成高清矢量 SVG 与 PNG 三维等轴测 + 俯视航向 + 高程剖面组合图 (零第三方绘图库依赖)。"""
    W, H = 1600, 920

    xs = np.array([r["x_east"] for r in records])
    ys = np.array([r["y_north"] for r in records])
    zs = np.array([r["alt"] for r in records])
    courses = np.array([r["course_deg"] for r in records])
    cum_d = np.array([r["cum_dist_3d"] for r in records])

    z_min, z_max = float(np.min(zs)), float(np.max(zs))
    z_base = math.floor(z_min - 3.0)
    z_top = math.ceil(z_max + 4.0)

    x_min, x_max = float(np.min(xs)) - 60.0, float(np.max(xs)) + 60.0
    y_min, y_max = float(np.min(ys)) - 60.0, float(np.max(ys)) + 60.0

    # 3D 投影函数 (Azimuth = -50°, Elevation = 27°, 带适度垂直拉伸突出爬升轨迹)
    az = math.radians(-50.0)
    el = math.radians(27.0)
    cx3d, cy3d = 470.0, 505.0
    span_xy = max(x_max - x_min, y_max - y_min, 1.0)
    scale_xy = 520.0 / span_xy
    scale_z = 210.0 / max(z_top - z_base, 1.0)

    x_mid = 0.5 * (x_min + x_max)
    y_mid = 0.5 * (y_min + y_max)
    z_mid = 0.5 * (z_base + z_top)

    def proj3d(x, y, z):
        nx = (x - x_mid) * scale_xy
        ny = (y - y_mid) * scale_xy
        nz = (z - z_mid) * scale_z
        # 绕 Z 轴旋转使正东偏右下、正北偏右上
        rx = nx * math.cos(az) - ny * math.sin(az)
        ry = nx * math.sin(az) + ny * math.cos(az)
        sx = cx3d + rx
        sy = cy3d - (ry * math.sin(el) + nz * math.cos(el))
        return sx, sy

    svg_elems = []
    svg_elems.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
    svg_elems.append('''<defs>
      <marker id="arrowRed" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6.5" markerHeight="6.5" orient="auto-start-reverse">
        <path d="M 0 1.5 L 10 5 L 0 8.5 z" fill="#DC2626"/>
      </marker>
      <marker id="arrowBlue" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5.5" markerHeight="5.5" orient="auto-start-reverse">
        <path d="M 0 2 L 10 5 L 0 8 z" fill="#2563EB"/>
      </marker>
      <marker id="arrowDark" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
        <path d="M 0 1.5 L 10 5 L 0 8.5 z" fill="#0F172A"/>
      </marker>
    </defs>''')

    # 背景与标题栏
    svg_elems.append(f'<rect width="{W}" height="{H}" fill="#F8FAFC"/>')
    svg_elems.append('<rect x="24" y="20" width="1552" height="68" rx="12" fill="#0F172A"/>')
    svg_elems.append(
        '<text x="48" y="50" fill="#F8FAFC" font-family="sans-serif" font-size="22" font-weight="bold">'
        'Wild/04 机载雷达三维飞行轨迹与航向角可视化 (3D GPS Flight Trajectory &amp; Course Heading)</text>'
    )
    total_3d = records[-1]["cum_dist_3d"]
    delta_alt = records[-1]["alt"] - records[0]["alt"]
    svg_elems.append(
        f'<text x="48" y="74" fill="#94A3B8" font-family="sans-serif" font-size="13.5">'
        f'数据源: dataset/Wild/04/extend_data.json  |  共 {len(records)} 帧  |  '
        f'原点 F1: ({records[0]["lat"]:.6f}°N, {records[0]["lon"]:.6f}°E, {records[0]["alt"]:.2f}m)  |  '
        f'三维总航程: {total_3d:.2f} m  |  净爬升: {delta_alt:+.2f} m</text>'
    )

    # ==================== 左侧面板: 3D 飞行轨迹 ====================
    svg_elems.append('<rect x="24" y="104" width="900" height="792" rx="14" fill="#FFFFFF" stroke="#E2E8F0" stroke-width="1.5"/>')
    svg_elems.append(
        '<text x="48" y="138" fill="#0F172A" font-family="sans-serif" font-size="17" font-weight="bold">'
        '1. 三维空间飞行轨迹与机头航向角矢量 (Local UTM Zone 49N / ENU, Z 轴高程放大显示)</text>'
    )

    # 绘制 3D 底面网格 (z = z_base) 与立面网格
    x_ticks = np.linspace(math.ceil(x_min / 100.0) * 100.0, math.floor(x_max / 100.0) * 100.0, 6)
    y_ticks = np.linspace(math.ceil(y_min / 100.0) * 100.0, math.floor(y_max / 100.0) * 100.0, 5)
    z_ticks = np.linspace(z_base, z_top, 5)

    # 底面框
    floor_pts = [proj3d(x_min, y_min, z_base), proj3d(x_max, y_min, z_base), proj3d(x_max, y_max, z_base), proj3d(x_min, y_max, z_base)]
    floor_poly = " ".join(f"{px:.1f},{py:.1f}" for px, py in floor_pts)
    svg_elems.append(f'<polygon points="{floor_poly}" fill="#F1F5F9" stroke="#CBD5E1" stroke-width="1.2"/>')

    # 后立面网格 (x = x_min 面与 y = y_max 面)
    for zt in z_ticks:
        p1 = proj3d(x_min, y_min, zt)
        p2 = proj3d(x_min, y_max, zt)
        p3 = proj3d(x_max, y_max, zt)
        svg_elems.append(f'<line x1="{p1[0]:.1f}" y1="{p1[1]:.1f}" x2="{p2[0]:.1f}" y2="{p2[1]:.1f}" stroke="#E2E8F0" stroke-width="1"/>')
        svg_elems.append(f'<line x1="{p2[0]:.1f}" y1="{p2[1]:.1f}" x2="{p3[0]:.1f}" y2="{p3[1]:.1f}" stroke="#E2E8F0" stroke-width="1"/>')
        svg_elems.append(
            f'<text x="{p1[0] - 10:.1f}" y="{p1[1] + 4:.1f}" text-anchor="end" fill="#475569" font-family="sans-serif" font-size="12">{zt:.1f}m</text>'
        )

    for xt in np.arange(-500, 101, 100):
        if x_min <= xt <= x_max:
            p1 = proj3d(xt, y_min, z_base)
            p2 = proj3d(xt, y_max, z_base)
            svg_elems.append(f'<line x1="{p1[0]:.1f}" y1="{p1[1]:.1f}" x2="{p2[0]:.1f}" y2="{p2[1]:.1f}" stroke="#CBD5E1" stroke-dasharray="4,4" stroke-width="1"/>')
            svg_elems.append(
                f'<text x="{p1[0] - 6:.1f}" y="{p1[1] + 18:.1f}" text-anchor="middle" fill="#475569" font-family="sans-serif" font-size="12">{xt:+.0f}m</text>'
            )

    for yt in np.arange(0, 451, 100):
        if y_min <= yt <= y_max:
            p1 = proj3d(x_min, yt, z_base)
            p2 = proj3d(x_max, yt, z_base)
            svg_elems.append(f'<line x1="{p1[0]:.1f}" y1="{p1[1]:.1f}" x2="{p2[0]:.1f}" y2="{p2[1]:.1f}" stroke="#CBD5E1" stroke-dasharray="4,4" stroke-width="1"/>')
            svg_elems.append(
                f'<text x="{p2[0] + 10:.1f}" y="{p2[1] + 5:.1f}" text-anchor="start" fill="#475569" font-family="sans-serif" font-size="12">{yt:+.0f}m</text>'
            )

    # Z 轴立柱与轴标题
    pz0 = proj3d(x_min, y_min, z_base)
    pz1 = proj3d(x_min, y_min, z_top)
    svg_elems.append(f'<line x1="{pz0[0]:.1f}" y1="{pz0[1]:.1f}" x2="{pz1[0]:.1f}" y2="{pz1[1]:.1f}" stroke="#475569" stroke-width="1.8"/>')
    svg_elems.append(f'<text x="{pz1[0] - 8:.1f}" y="{pz1[1] - 14:.1f}" text-anchor="middle" fill="#0F172A" font-family="sans-serif" font-size="13" font-weight="bold">Altitude Z (m)</text>')

    px_lbl = proj3d(0.5 * (x_min + x_max), y_min - 55.0, z_base)
    py_lbl = proj3d(x_max + 65.0, 0.5 * (y_min + y_max), z_base)
    svg_elems.append(f'<text x="{px_lbl[0]:.1f}" y="{px_lbl[1] + 22:.1f}" text-anchor="middle" fill="#0F172A" font-family="sans-serif" font-size="13" font-weight="bold">ΔEast X (m)</text>')
    svg_elems.append(f'<text x="{py_lbl[0]:.1f}" y="{py_lbl[1]:.1f}" text-anchor="start" fill="#0F172A" font-family="sans-serif" font-size="13" font-weight="bold">ΔNorth Y (m)</text>')

    # 地面投影轨迹与垂线
    ground_pts = [proj3d(r["x_east"], r["y_north"], z_base) for r in records]
    ground_path = " ".join(f"{px:.1f},{py:.1f}" for px, py in ground_pts)
    svg_elems.append(f'<polyline points="{ground_path}" fill="none" stroke="#94A3B8" stroke-width="2.2" stroke-dasharray="6,5"/>')

    air_pts = [proj3d(r["x_east"], r["y_north"], r["alt"]) for r in records]
    for (gx, gy), (ax_p, ay_p) in zip(ground_pts, air_pts):
        svg_elems.append(f'<line x1="{gx:.1f}" y1="{gy:.1f}" x2="{ax_p:.1f}" y2="{ay_p:.1f}" stroke="#94A3B8" stroke-width="1.4" stroke-dasharray="3,4"/>')
        svg_elems.append(f'<circle cx="{gx:.1f}" cy="{gy:.1f}" r="3.5" fill="#94A3B8"/>')

    # 三维主飞行轨迹线与航段中点方向箭头
    air_path = " ".join(f"{px:.1f},{py:.1f}" for px, py in air_pts)
    svg_elems.append(f'<polyline points="{air_path}" fill="none" stroke="#2563EB" stroke-width="4.0" stroke-linecap="round" stroke-linejoin="round"/>')
    for i in range(len(air_pts) - 1):
        p0, p1 = air_pts[i], air_pts[i + 1]
        mx, my = 0.55 * p1[0] + 0.45 * p0[0], 0.55 * p1[1] + 0.45 * p0[1]
        svg_elems.append(f'<line x1="{p0[0]:.1f}" y1="{p0[1]:.1f}" x2="{mx:.1f}" y2="{my:.1f}" stroke="#2563EB" stroke-width="2.5" marker-end="url(#arrowBlue)"/>')

    # 每帧航向角矢量箭头 (Course) 与节点标注
    arrow_m = 68.0
    for i, r in enumerate(records):
        ax_p, ay_p = air_pts[i]
        hx, hy = proj3d(
            r["x_east"] + r["dir_east"] * arrow_m,
            r["y_north"] + r["dir_north"] * arrow_m,
            r["alt"],
        )
        svg_elems.append(
            f'<line x1="{ax_p:.1f}" y1="{ay_p:.1f}" x2="{hx:.1f}" y2="{hy:.1f}" '
            f'stroke="#DC2626" stroke-width="3.0" marker-end="url(#arrowRed)"/>'
        )
        color = "#16A34A" if i == 0 else ("#EA580C" if i == len(records) - 1 else "#2563EB")
        svg_elems.append(f'<circle cx="{ax_p:.1f}" cy="{ay_p:.1f}" r="7.5" fill="{color}" stroke="#FFFFFF" stroke-width="2.2"/>')

        # 错开标注框避免遮挡
        dx_lbl = 16 if i % 2 == 0 else -178
        dy_lbl = -38 if i != 2 else -44
        bx, by = ax_p + dx_lbl, ay_p + dy_lbl
        svg_elems.append(f'<rect x="{bx:.1f}" y="{by:.1f}" width="166" height="40" rx="6" fill="#FFFFFF" fill-opacity="0.92" stroke="#CBD5E1" stroke-width="1.2"/>')
        svg_elems.append(
            f'<text x="{bx + 8:.1f}" y="{by + 16:.1f}" fill="#0F172A" font-family="sans-serif" font-size="12" font-weight="bold">'
            f'F{r["frame_idx"]} ({r["short_label"]})</text>'
        )
        svg_elems.append(
            f'<text x="{bx + 8:.1f}" y="{by + 33:.1f}" fill="#334155" font-family="sans-serif" font-size="11.5">'
            f'Alt:{r["alt"]:.2f}m | Course:{r["course_deg"]:.2f}°</text>'
        )

    # 图例
    svg_elems.append('<rect x="46" y="826" width="854" height="50" rx="8" fill="#F8FAFC" stroke="#E2E8F0"/>')
    svg_elems.append('<circle cx="72" cy="851" r="6.5" fill="#16A34A"/><text x="86" y="855" fill="#1E293B" font-family="sans-serif" font-size="12.5">起点 F1 (Start)</text>')
    svg_elems.append('<circle cx="205" cy="851" r="6.5" fill="#EA580C"/><text x="219" y="855" fill="#1E293B" font-family="sans-serif" font-size="12.5">终点 F5 (End)</text>')
    svg_elems.append('<line x1="332" y1="851" x2="368" y2="851" stroke="#2563EB" stroke-width="3.5"/><text x="376" y="855" fill="#1E293B" font-family="sans-serif" font-size="12.5">三维飞行轨迹 (3D Path)</text>')
    svg_elems.append('<line x1="540" y1="851" x2="576" y2="851" stroke="#DC2626" stroke-width="3" marker-end="url(#arrowRed)"/><text x="586" y="855" fill="#1E293B" font-family="sans-serif" font-size="12.5">Course 航向角矢量 (正北=0°, 顺时针+)</text>')

    # ==================== 右上面板: 水平俯视 ENU 航迹 ====================
    rx0, ry0, rw, rh = 944, 104, 632, 384
    svg_elems.append(f'<rect x="{rx0}" y="{ry0}" width="{rw}" height="{rh}" rx="14" fill="#FFFFFF" stroke="#E2E8F0" stroke-width="1.5"/>')
    svg_elems.append(
        f'<text x="{rx0 + 22}" y="{ry0 + 32}" fill="#0F172A" font-family="sans-serif" font-size="16.5" font-weight="bold">'
        '2. 水平俯视轨迹与航向角指北视图 (Top-Down ENU)</text>'
    )

    pad_l, pad_r, pad_t, pad_b = 70, 36, 54, 48
    pw2, ph2 = rw - pad_l - pad_r, rh - pad_t - pad_b
    s2 = min(pw2 / max(x_max - x_min, 1.0), ph2 / max(y_max - y_min, 1.0))
    ox2 = rx0 + pad_l + 0.5 * (pw2 - (x_max - x_min) * s2)
    oy2 = ry0 + pad_t + 0.5 * (ph2 - (y_max - y_min) * s2)

    def proj2d(x, y):
        return ox2 + (x - x_min) * s2, oy2 + (y_max - y) * s2

    svg_elems.append(f'<rect x="{rx0 + pad_l}" y="{ry0 + pad_t}" width="{pw2}" height="{ph2}" fill="#F8FAFC" stroke="#CBD5E1"/>')
    for xt in np.arange(-500, 101, 100):
        gx, _ = proj2d(xt, 0)
        if rx0 + pad_l <= gx <= rx0 + pad_l + pw2:
            svg_elems.append(f'<line x1="{gx:.1f}" y1="{ry0 + pad_t}" x2="{gx:.1f}" y2="{ry0 + pad_t + ph2}" stroke="#E2E8F0"/>')
            svg_elems.append(f'<text x="{gx:.1f}" y="{ry0 + pad_t + ph2 + 18}" text-anchor="middle" fill="#475569" font-family="sans-serif" font-size="11.5">{xt:+.0f}m</text>')
    for yt in np.arange(0, 451, 100):
        _, gy = proj2d(0, yt)
        if ry0 + pad_t <= gy <= ry0 + pad_t + ph2:
            svg_elems.append(f'<line x1="{rx0 + pad_l}" y1="{gy:.1f}" x2="{rx0 + pad_l + pw2}" y2="{gy:.1f}" stroke="#E2E8F0"/>')
            svg_elems.append(f'<text x="{rx0 + pad_l - 8}" y="{gy + 4:.1f}" text-anchor="end" fill="#475569" font-family="sans-serif" font-size="11.5">{yt:+.0f}m</text>')

    pts2d = [proj2d(r["x_east"], r["y_north"]) for r in records]
    path2d = " ".join(f"{px:.1f},{py:.1f}" for px, py in pts2d)
    svg_elems.append(f'<polyline points="{path2d}" fill="none" stroke="#2563EB" stroke-width="3.0"/>')
    for i, r in enumerate(records):
        px, py = pts2d[i]
        hx, hy = proj2d(r["x_east"] + r["dir_east"] * 65.0, r["y_north"] + r["dir_north"] * 65.0)
        svg_elems.append(f'<line x1="{px:.1f}" y1="{py:.1f}" x2="{hx:.1f}" y2="{hy:.1f}" stroke="#DC2626" stroke-width="2.4" marker-end="url(#arrowRed)"/>')
        c = "#16A34A" if i == 0 else ("#EA580C" if i == len(records) - 1 else "#2563EB")
        svg_elems.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="5.5" fill="{c}" stroke="#FFFFFF" stroke-width="1.8"/>')
        svg_elems.append(f'<text x="{px + 8:.1f}" y="{py - 7:.1f}" fill="#0F172A" font-family="sans-serif" font-size="11.5" font-weight="bold">F{r["frame_idx"]} ({r["course_deg"]:.1f}°)</text>')

    # 指北针
    nx_c, ny_c = rx0 + rw - 58, ry0 + pad_t + 54
    svg_elems.append(f'<circle cx="{nx_c}" cy="{ny_c}" r="24" fill="#FFFFFF" stroke="#CBD5E1"/>')
    svg_elems.append(f'<line x1="{nx_c}" y1="{ny_c + 14}" x2="{nx_c}" y2="{ny_c - 14}" stroke="#0F172A" stroke-width="2.2" marker-end="url(#arrowDark)"/>')
    svg_elems.append(f'<text x="{nx_c}" y="{ny_c - 28}" text-anchor="middle" fill="#0F172A" font-family="sans-serif" font-size="11.5" font-weight="bold">N (0°)</text>')
    svg_elems.append(f'<text x="{rx0 + pad_l + pw2 * 0.5:.1f}" y="{ry0 + rh - 10}" text-anchor="middle" fill="#334155" font-family="sans-serif" font-size="12" font-weight="bold">ΔEast (m)   [Lon: 109.6377°E ~ 109.6428°E]</text>')

    # ==================== 右下面板: 高程爬升与航向角剖面 ====================
    bx0, by0, bw, bh = 944, 508, 632, 388
    svg_elems.append(f'<rect x="{bx0}" y="{by0}" width="{bw}" height="{bh}" rx="14" fill="#FFFFFF" stroke="#E2E8F0" stroke-width="1.5"/>')
    svg_elems.append(
        f'<text x="{bx0 + 22}" y="{by0 + 32}" fill="#0F172A" font-family="sans-serif" font-size="16.5" font-weight="bold">'
        '3. 沿航程海拔高度 (Altitude) 与航向角 (Course) 变化曲线</text>'
    )

    pl, pr, pt, pb = 68, 68, 56, 54
    cw, ch = bw - pl - pr, bh - pt - pb
    d_max = max(float(cum_d[-1]), 1.0)
    c_min, c_max = math.floor(float(np.min(courses)) - 5.0), math.ceil(float(np.max(courses)) + 5.0)

    def proj_alt(d, a):
        return bx0 + pl + (d / d_max) * cw, by0 + pt + (1.0 - (a - z_base) / max(z_top - z_base, 1.0)) * ch

    def proj_crs(d, c):
        return bx0 + pl + (d / d_max) * cw, by0 + pt + (1.0 - (c - c_min) / max(c_max - c_min, 1.0)) * ch

    svg_elems.append(f'<rect x="{bx0 + pl}" y="{by0 + pt}" width="{cw}" height="{ch}" fill="#F8FAFC" stroke="#CBD5E1"/>')
    for frac in [0.0, 0.25, 0.5, 0.75, 1.0]:
        gy = by0 + pt + (1.0 - frac) * ch
        av = z_base + frac * (z_top - z_base)
        cv = c_min + frac * (c_max - c_min)
        svg_elems.append(f'<line x1="{bx0 + pl}" y1="{gy:.1f}" x2="{bx0 + pl + cw}" y2="{gy:.1f}" stroke="#E2E8F0"/>')
        svg_elems.append(f'<text x="{bx0 + pl - 8}" y="{gy + 4:.1f}" text-anchor="end" fill="#059669" font-family="sans-serif" font-size="11.5">{av:.1f}m</text>')
        svg_elems.append(f'<text x="{bx0 + pl + cw + 8}" y="{gy + 4:.1f}" text-anchor="start" fill="#DC2626" font-family="sans-serif" font-size="11.5">{cv:.1f}°</text>')

    for dv in np.arange(0, d_max + 50, 150):
        if dv <= d_max:
            gx = bx0 + pl + (dv / d_max) * cw
            svg_elems.append(f'<line x1="{gx:.1f}" y1="{by0 + pt}" x2="{gx:.1f}" y2="{by0 + pt + ch}" stroke="#E2E8F0"/>')
            svg_elems.append(f'<text x="{gx:.1f}" y="{by0 + pt + ch + 18}" text-anchor="middle" fill="#475569" font-family="sans-serif" font-size="11.5">{dv:.0f}m</text>')

    alt_pts = [proj_alt(r["cum_dist_3d"], r["alt"]) for r in records]
    crs_pts = [proj_crs(r["cum_dist_3d"], r["course_deg"]) for r in records]

    svg_elems.append(f'<polyline points="' + " ".join(f"{x:.1f},{y:.1f}" for x, y in alt_pts) + '" fill="none" stroke="#059669" stroke-width="3.0"/>')
    svg_elems.append(f'<polyline points="' + " ".join(f"{x:.1f},{y:.1f}" for x, y in crs_pts) + '" fill="none" stroke="#DC2626" stroke-width="2.6" stroke-dasharray="6,4"/>')

    for i, r in enumerate(records):
        ax_i, ay_i = alt_pts[i]
        cx_i, cy_i = crs_pts[i]
        svg_elems.append(f'<circle cx="{ax_i:.1f}" cy="{ay_i:.1f}" r="5.5" fill="#059669" stroke="#FFFFFF" stroke-width="1.6"/>')
        svg_elems.append(f'<text x="{ax_i:.1f}" y="{ay_i - 9:.1f}" text-anchor="middle" fill="#065F46" font-family="sans-serif" font-size="11.5" font-weight="bold">F{r["frame_idx"]} ({r["alt"]:.1f}m)</text>')
        svg_elems.append(f'<rect x="{cx_i - 4.5:.1f}" y="{cy_i - 4.5:.1f}" width="9" height="9" fill="#DC2626" stroke="#FFFFFF" stroke-width="1.4"/>')

    svg_elems.append(f'<text x="{bx0 + pl + cw * 0.5:.1f}" y="{by0 + bh - 12}" text-anchor="middle" fill="#334155" font-family="sans-serif" font-size="12" font-weight="bold">累计三维航程 Cumulative 3D Distance (m)   [绿实线: Altitude(m) | 红虚线: Course(°)]</text>')
    svg_elems.append("</svg>")

    os.makedirs(os.path.dirname(os.path.abspath(svg_path)), exist_ok=True)
    with open(svg_path, "w", encoding="utf-8") as f:
        f.write("\n".join(svg_elems))
    print(f"[OK] 高清矢量轨迹图已保存至: {svg_path}")

    if HAS_MATPLOTLIB:
        # 若安装了 matplotlib 则额外导出栅格 PNG
        pass
    elif HAS_PIL:
        # 使用 Pillow 绘制清晰的 PNG 版本
        img = Image.new("RGB", (W, H), (248, 250, 252))
        draw = ImageDraw.Draw(img)
        draw.rounded_rectangle([24, 20, 1576, 88], radius=12, fill=(15, 23, 42))
        draw.text((48, 38), "Wild/04 3D GPS Flight Trajectory & Course Heading", fill=(248, 250, 252))
        draw.text(
            (48, 60),
            f"Frames: {len(records)} | Origin F1: ({records[0]['lat']:.6f}N, {records[0]['lon']:.6f}E, {records[0]['alt']:.2f}m) | 3D Dist: {total_3d:.2f}m | Climb: {delta_alt:+.2f}m",
            fill=(148, 163, 184),
        )
        draw.rounded_rectangle([24, 104, 924, 896], radius=14, fill=(255, 255, 255), outline=(226, 232, 240), width=2)
        draw.text((48, 124), "1. 3D Flight Trajectory & Course Vectors (ENU, Z Exaggerated)", fill=(15, 23, 42))
        draw.polygon(floor_pts, fill=(241, 245, 249), outline=(203, 213, 225))
        draw.line(ground_pts, fill=(148, 163, 184), width=2)
        for (gx, gy), (ax_p, ay_p) in zip(ground_pts, air_pts):
            draw.line([(gx, gy), (ax_p, ay_p)], fill=(180, 192, 208), width=1)
        draw.line(air_pts, fill=(37, 99, 235), width=4)
        for i, r in enumerate(records):
            ax_p, ay_p = air_pts[i]
            hx, hy = proj3d(r["x_east"] + r["dir_east"] * arrow_m, r["y_north"] + r["dir_north"] * arrow_m, r["alt"])
            draw.line([(ax_p, ay_p), (hx, hy)], fill=(220, 38, 38), width=3)
            draw.ellipse([hx - 4, hy - 4, hx + 4, hy + 4], fill=(220, 38, 38))
            col = (22, 163, 74) if i == 0 else ((234, 88, 12) if i == len(records) - 1 else (37, 99, 235))
            draw.ellipse([ax_p - 7, ay_p - 7, ax_p + 7, ay_p + 7], fill=col, outline=(255, 255, 255), width=2)
            draw.text((ax_p + 12, ay_p - 16), f"F{r['frame_idx']} Alt:{r['alt']:.2f}m Course:{r['course_deg']:.2f}deg", fill=(15, 23, 42))

        # Top-down panel
        draw.rounded_rectangle([rx0, ry0, rx0 + rw, ry0 + rh], radius=14, fill=(255, 255, 255), outline=(226, 232, 240), width=2)
        draw.text((rx0 + 22, ry0 + 20), "2. Top-Down Ground Track & Course (East-North)", fill=(15, 23, 42))
        draw.rectangle([rx0 + pad_l, ry0 + pad_t, rx0 + pad_l + pw2, ry0 + pad_t + ph2], fill=(248, 250, 252), outline=(203, 213, 225))
        draw.line(pts2d, fill=(37, 99, 235), width=3)
        for i, r in enumerate(records):
            px, py = pts2d[i]
            hx, hy = proj2d(r["x_east"] + r["dir_east"] * 65.0, r["y_north"] + r["dir_north"] * 65.0)
            draw.line([(px, py), (hx, hy)], fill=(220, 38, 38), width=3)
            draw.ellipse([px - 5, py - 5, px + 5, py + 5], fill=(37, 99, 235))
            draw.text((px + 8, py - 14), f"F{r['frame_idx']} ({r['course_deg']:.1f}deg)", fill=(15, 23, 42))

        # Profile panel
        draw.rounded_rectangle([bx0, by0, bx0 + bw, by0 + bh], radius=14, fill=(255, 255, 255), outline=(226, 232, 240), width=2)
        draw.text((bx0 + 22, by0 + 20), "3. Altitude (green) & Course (red) vs Cumulative Distance", fill=(15, 23, 42))
        draw.rectangle([bx0 + pl, by0 + pt, bx0 + pl + cw, by0 + pt + ch], fill=(248, 250, 252), outline=(203, 213, 225))
        draw.line(alt_pts, fill=(5, 150, 105), width=3)
        draw.line(crs_pts, fill=(220, 38, 38), width=3)
        for i, r in enumerate(records):
            ax_i, ay_i = alt_pts[i]
            cx_i, cy_i = crs_pts[i]
            draw.ellipse([ax_i - 5, ay_i - 5, ax_i + 5, ay_i + 5], fill=(5, 150, 105))
            draw.rectangle([cx_i - 4, cy_i - 4, cx_i + 4, cy_i + 4], fill=(220, 38, 38))
            draw.text((ax_i - 18, ay_i - 18), f"F{r['frame_idx']} {r['alt']:.1f}m", fill=(6, 95, 70))

        img.save(png_path)
        print(f"[OK] PNG 轨迹图已保存至: {png_path}")


def save_interactive_html(records, html_path):
    """生成零外部依赖的独立交互式 3D HTML 轨迹可视化页面。"""
    records_json = json.dumps(records, ensure_ascii=False, indent=2)
    html_content = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8" />
<meta name="viewport" content="width=device-width, initial-scale=1.0" />
<title>Wild/04 三维飞行轨迹与航向角交互可视化</title>
<style>
  * {{ box-sizing: border-box; margin: 0; padding: 0; }}
  body {{ font-family: system-ui, -apple-system, sans-serif; background: #0f172a; color: #f8fafc; padding: 20px; }}
  .container {{ max-width: 1280px; margin: 0 auto; display: flex; flex-direction: column; gap: 16px; }}
  .header {{ background: #1e293b; border: 1px solid #334155; border-radius: 12px; padding: 16px 20px; display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 12px; }}
  .header h1 {{ font-size: 18px; font-weight: 700; }}
  .header p {{ font-size: 13px; color: #94a3b8; margin-top: 4px; }}
  .controls {{ display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }}
  button {{ background: #334155; color: #f8fafc; border: 1px solid #475569; border-radius: 8px; padding: 6px 12px; font-size: 13px; cursor: pointer; }}
  button:hover, button.active {{ background: #2563eb; border-color: #3b82f6; }}
  .canvas-card {{ position: relative; background: #1e293b; border: 1px solid #334155; border-radius: 12px; overflow: hidden; height: 540px; }}
  canvas {{ width: 100%; height: 100%; display: block; cursor: grab; }}
  canvas:active {{ cursor: grabbing; }}
  .hud {{ position: absolute; top: 14px; left: 14px; background: rgba(15,23,42,0.86); border: 1px solid #334155; border-radius: 10px; padding: 10px 14px; font-size: 12.5px; line-height: 1.6; pointer-events: none; }}
  table {{ width: 100%; border-collapse: collapse; background: #1e293b; border: 1px solid #334155; border-radius: 12px; overflow: hidden; font-size: 13px; }}
  th, td {{ padding: 10px 12px; text-align: left; border-bottom: 1px solid #334155; }}
  th {{ background: #0f172a; color: #94a3b8; font-weight: 600; }}
  tr:hover {{ background: rgba(37,99,235,0.18); cursor: pointer; }}
  tr.selected {{ background: rgba(37,99,235,0.30); }}
</style>
</head>
<body>
<div class="container">
  <div class="header">
    <div>
      <h1>Wild/04 机载雷达三维飞行轨迹与航向角可视化</h1>
      <p>鼠标左键拖拽旋转 3D 视角 · 滚轮缩放 · 点击帧行或节点高亮查看经纬高与 Course 航向角</p>
    </div>
    <div class="controls">
      <button onclick="setView(-52, 26)">3D 标准视角</button>
      <button onclick="setView(0, 89)">俯视 (东-北)</button>
      <button onclick="setView(-35, 6)">侧视爬升剖面</button>
      <label style="font-size:13px;color:#cbd5e1;margin-left:6px;">高程放大:
        <select id="zExag" onchange="draw()" style="background:#0f172a;color:#f8fafc;border:1px solid #475569;border-radius:6px;padding:4px 8px;">
          <option value="1">1x (真实比例)</option>
          <option value="5">5x</option>
          <option value="12" selected>12x (推荐清晰观察爬升)</option>
          <option value="20">20x</option>
        </select>
      </label>
    </div>
  </div>
  <div class="canvas-card">
    <canvas id="c"></canvas>
    <div class="hud" id="hud"></div>
  </div>
  <table>
    <thead>
      <tr>
        <th>帧序</th><th>脉冲标识</th><th>纬度 Lat (°N)</th><th>经度 Lon (°E)</th>
        <th>海拔 Alt (m)</th><th>航向角 Course (°)</th><th>相对东向 ΔE (m)</th><th>相对北向 ΔN (m)</th><th>累计航程 (m)</th>
      </tr>
    </thead>
    <tbody id="tb"></tbody>
  </table>
</div>
<script>
const DATA = {records_json};
let az = -52 * Math.PI / 180, el = 26 * Math.PI / 180, zoom = 1.0, selectedIdx = 0;
let dragging = false, lx = 0, ly = 0;
const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');

function setView(aDeg, eDeg) {{ az = aDeg * Math.PI / 180; el = eDeg * Math.PI / 180; draw(); }}

const tbody = document.getElementById('tb');
DATA.forEach((r, idx) => {{
  const tr = document.createElement('tr');
  tr.innerHTML = `<td><b>F${{r.frame_idx}}</b></td><td>${{r.short_label}}</td><td>${{r.lat.toFixed(8)}}</td><td>${{r.lon.toFixed(8)}}</td><td>${{r.alt.toFixed(4)}}</td><td style="color:#f87171;font-weight:600">${{r.course_deg.toFixed(4)}}° (${{r.course_360.toFixed(2)}}°)</td><td>${{r.x_east >= 0 ? '+' : ''}}${{r.x_east.toFixed(2)}}</td><td>${{r.y_north >= 0 ? '+' : ''}}${{r.y_north.toFixed(2)}}</td><td>${{r.cum_dist_3d.toFixed(2)}}</td>`;
  tr.onclick = () => {{ selectedIdx = idx; updateSelection(); draw(); }};
  tbody.appendChild(tr);
}});

function updateSelection() {{
  [...tbody.children].forEach((tr, i) => tr.classList.toggle('selected', i === selectedIdx));
  const r = DATA[selectedIdx];
  document.getElementById('hud').innerHTML = `<b>当前选中: F${{r.frame_idx}} (${{r.short_label}})</b><br>Lat: ${{r.lat.toFixed(8)}}°N | Lon: ${{r.lon.toFixed(8)}}°E<br>Alt: ${{r.alt.toFixed(4)}} m | <span style="color:#f87171">Course: ${{r.course_deg.toFixed(4)}}°</span><br>局部 ENU: ΔE=${{r.x_east.toFixed(2)}}m, ΔN=${{r.y_north.toFixed(2)}}m, ΔZ=+${{r.z_rel.toFixed(2)}}m`;
}}

canvas.addEventListener('mousedown', e => {{ dragging = true; lx = e.clientX; ly = e.clientY; }});
window.addEventListener('mouseup', () => dragging = false);
window.addEventListener('mousemove', e => {{
  if (!dragging) return;
  az += (e.clientX - lx) * 0.008;
  el = Math.max(-0.2, Math.min(1.55, el + (e.clientY - ly) * 0.008));
  lx = e.clientX; ly = e.clientY;
  draw();
}});
canvas.addEventListener('wheel', e => {{
  e.preventDefault();
  zoom = Math.max(0.45, Math.min(2.6, zoom * (e.deltaY < 0 ? 1.1 : 0.91)));
  draw();
}}, {{ passive: false }});

function draw() {{
  const dpr = window.devicePixelRatio || 1;
  const W = canvas.clientWidth, H = canvas.clientHeight;
  canvas.width = W * dpr; canvas.height = H * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W, H);

  const zExag = parseFloat(document.getElementById('zExag').value || '12');
  const xs = DATA.map(d => d.x_east), ys = DATA.map(d => d.y_north), zs = DATA.map(d => d.alt);
  const xMin = Math.min(...xs) - 60, xMax = Math.max(...xs) + 60;
  const yMin = Math.min(...ys) - 60, yMax = Math.max(...ys) + 60;
  const zBase = Math.floor(Math.min(...zs) - 3), zTop = Math.ceil(Math.max(...zs) + 3);
  const xMid = 0.5 * (xMin + xMax), yMid = 0.5 * (yMin + yMax), zMid = 0.5 * (zBase + zTop);
  const span = Math.max(xMax - xMin, yMax - yMin, 1);
  const s = (Math.min(W, H) * 0.62 / span) * zoom;

  function p3(x, y, z) {{
    const nx = (x - xMid) * s, ny = (y - yMid) * s, nz = (z - zMid) * zExag * s;
    const rx = nx * Math.cos(az) - ny * Math.sin(az);
    const ry = nx * Math.sin(az) + ny * Math.cos(az);
    return [W * 0.52 + rx, H * 0.56 - (ry * Math.sin(el) + nz * Math.cos(el))];
  }}

  // Floor grid
  ctx.strokeStyle = 'rgba(148,163,184,0.22)'; ctx.lineWidth = 1;
  for (let x = -500; x <= 100; x += 100) {{
    if (x < xMin || x > xMax) continue;
    const a = p3(x, yMin, zBase), b = p3(x, yMax, zBase);
    ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
    ctx.fillStyle = '#94a3b8'; ctx.font = '12px sans-serif';
    ctx.fillText(`${{x}}m E`, a[0] - 16, a[1] + 16);
  }}
  for (let y = 0; y <= 450; y += 100) {{
    if (y < yMin || y > yMax) continue;
    const a = p3(xMin, y, zBase), b = p3(xMax, y, zBase);
    ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
    ctx.fillStyle = '#94a3b8'; ctx.font = '12px sans-serif';
    ctx.fillText(`${{y}}m N`, b[0] + 6, b[1] + 4);
  }}

  // Ground shadow & drop lines
  ctx.strokeStyle = 'rgba(148,163,184,0.5)'; ctx.setLineDash([5, 5]); ctx.lineWidth = 1.6;
  ctx.beginPath();
  DATA.forEach((r, i) => {{ const g = p3(r.x_east, r.y_north, zBase); i ? ctx.lineTo(g[0], g[1]) : ctx.moveTo(g[0], g[1]); }});
  ctx.stroke();
  DATA.forEach(r => {{
    const g = p3(r.x_east, r.y_north, zBase), a = p3(r.x_east, r.y_north, r.alt);
    ctx.beginPath(); ctx.moveTo(g[0], g[1]); ctx.lineTo(a[0], a[1]); ctx.stroke();
  }});
  ctx.setLineDash([]);

  // 3D Flight trajectory
  ctx.strokeStyle = '#3b82f6'; ctx.lineWidth = 3.5;
  ctx.beginPath();
  DATA.forEach((r, i) => {{ const a = p3(r.x_east, r.y_north, r.alt); i ? ctx.lineTo(a[0], a[1]) : ctx.moveTo(a[0], a[1]); }});
  ctx.stroke();

  // Heading vectors & nodes
  const arrowLen = 68;
  DATA.forEach((r, i) => {{
    const a = p3(r.x_east, r.y_north, r.alt);
    const h = p3(r.x_east + r.dir_east * arrowLen, r.y_north + r.dir_north * arrowLen, r.alt);
    ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 2.6;
    ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(h[0], h[1]); ctx.stroke();
    const ang = Math.atan2(h[1] - a[1], h[0] - a[0]);
    ctx.fillStyle = '#ef4444';
    ctx.beginPath();
    ctx.moveTo(h[0], h[1]);
    ctx.lineTo(h[0] - 10 * Math.cos(ang - 0.4), h[1] - 10 * Math.sin(ang - 0.4));
    ctx.lineTo(h[0] - 10 * Math.cos(ang + 0.4), h[1] - 10 * Math.sin(ang + 0.4));
    ctx.closePath(); ctx.fill();

    ctx.fillStyle = i === 0 ? '#22c55e' : (i === DATA.length - 1 ? '#f97316' : '#3b82f6');
    ctx.beginPath(); ctx.arc(a[0], a[1], i === selectedIdx ? 9 : 6.5, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = '#ffffff'; ctx.lineWidth = i === selectedIdx ? 2.8 : 1.6; ctx.stroke();

    ctx.fillStyle = '#f8fafc'; ctx.font = 'bold 12.5px sans-serif';
    ctx.fillText(`F${{r.frame_idx}} (${{r.alt.toFixed(1)}}m, ${{r.course_deg.toFixed(1)}}°)`, a[0] + 10, a[1] - 10);
  }});
}}

window.addEventListener('resize', draw);
updateSelection();
draw();
</script>
</body>
</html>"""
    os.makedirs(os.path.dirname(os.path.abspath(html_path)), exist_ok=True)
    with open(html_path, "w", encoding="utf-8") as f:
        f.write(html_content)
    print(f"[OK] 交互式三维飞行轨迹网页已保存至: {html_path}")


def main():
    parser = argparse.ArgumentParser(description="提取 extend_data.json 中的 GPS 与航向角并绘制三维飞行轨迹")
    parser.add_argument(
        "--json_path",
        type=str,
        default="/home/asahichinn/Documents/codes/pointCloudProcessor/dataset/Wild/04/extend_data.json",
        help="extend_data.json 文件路径",
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default=None,
        help="输出目录 (默认保存在 extend_data.json 所在目录)",
    )
    parser.add_argument("--show", action="store_true", help="弹出 Open3D 交互式三维轨迹窗口")
    args = parser.parse_args()

    out_dir = args.output_dir or os.path.dirname(os.path.abspath(args.json_path))
    os.makedirs(out_dir, exist_ok=True)

    records, crs_utm = extract_trajectory_from_json(args.json_path)
    print(f"投影参考系: {crs_utm} (pyproj={'已启用' if HAS_PYPROJ else '椭球切平面近似'})")
    print_trajectory_table(records)

    svg_path = os.path.join(out_dir, "flight_trajectory_3d.svg")
    png_path = os.path.join(out_dir, "flight_trajectory_3d.png")
    html_path = os.path.join(out_dir, "flight_trajectory_3d.html")
    ply_path = os.path.join(out_dir, "flight_trajectory_3d.ply")

    save_svg_and_png_plot(records, svg_path, png_path)
    save_interactive_html(records, html_path)
    save_open3d_ply_and_optionally_show(records, ply_path, show_window=args.show)


if __name__ == "__main__":
    main()
