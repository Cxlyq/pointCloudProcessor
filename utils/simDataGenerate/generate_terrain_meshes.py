#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地面地形白模生成脚本 (generate_terrain_meshes.py)

功能：
  输入生成场景个数，采用常见地形生成算法（多倍频Perlin/Simplex噪声fBm、Diamond-Square中点位移、
  山脊噪声Ridge Noise、断层位移Fault formation等）生成指定数量的地形三维白模。
  每个白模场景范围约为 6km * 8km (X: [0, 6000]m, Y: [0, 8000]m)，
  三角面数约为 8000 面 (50 x 80 网格剖分为 8000 个三角面)，
  输出保存为标准 .ply 文件。

运行环境：
  推荐使用 conda 环境: /home/asahichinn/miniconda3/envs/pointcloud_607
"""

import os
import sys
import argparse
import time
import numpy as np
from scipy.ndimage import gaussian_filter

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False


# ==============================================================================
# 1. 地形高程生成算法 (Terrain Elevation Generation Algorithms)
# ==============================================================================

def generate_perlin_noise_2d(shape, res, seed=None):
    """
    生成高品质二维矢量化 Perlin 噪声
    :param shape: (ny, nx) 输出栅格分辨率
    :param res: (ry, rx) 噪声网格频率周期数
    :param seed: 随机种子
    """
    if seed is not None:
        np.random.seed(seed)
    
    ny, nx = shape
    ry, rx = int(max(1, round(res[0]))), int(max(1, round(res[1])))
    
    # 随机单位梯度向量
    angles = 2.0 * np.pi * np.random.rand(ry + 1, rx + 1)
    gradients = np.stack((np.cos(angles), np.sin(angles)), axis=-1)
    
    # 网格采样坐标归一化到 [0, rx] x [0, ry]
    grid_y = np.linspace(0, ry, ny, endpoint=False)
    grid_x = np.linspace(0, rx, nx, endpoint=False)
    gx, gy = np.meshgrid(grid_x, grid_y)
    
    # 网格整数与小数部分
    x0 = np.floor(gx).astype(int)
    x1 = x0 + 1
    y0 = np.floor(gy).astype(int)
    y1 = y0 + 1
    
    dx0 = gx - x0
    dx1 = dx0 - 1.0
    dy0 = gy - y0
    dy1 = dy0 - 1.0
    
    # 平滑衰减函数 6t^5 - 15t^4 + 10t^3 (Improved Perlin fade)
    fade = lambda t: t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
    sx = fade(dx0)
    sy = fade(dy0)
    
    # 4个顶点的点积
    n00 = gradients[y0, x0, 0] * dx0 + gradients[y0, x0, 1] * dy0
    n10 = gradients[y0, x1, 0] * dx1 + gradients[y0, x1, 1] * dy0
    n01 = gradients[y1, x0, 0] * dx0 + gradients[y1, x0, 1] * dy1
    n11 = gradients[y1, x1, 0] * dx1 + gradients[y1, x1, 1] * dy1
    
    # 双线性插值
    ix0 = n00 + sx * (n10 - n00)
    ix1 = n01 + sx * (n11 - n01)
    noise = ix0 + sy * (ix1 - ix0)
    return noise


def generate_fbm_terrain(shape, octaves=6, persistence=0.5, lacunarity=2.0, base_res=(2, 2), seed=None):
    """
    分形布朗运动 (Fractal Brownian Motion - fBm)，模拟自然多尺度连绵山地/丘陵
    """
    if seed is not None:
        np.random.seed(seed)
    
    ny, nx = shape
    total = np.zeros(shape, dtype=np.float64)
    amplitude = 1.0
    freq_y, freq_x = base_res
    max_amp = 0.0
    
    for i in range(octaves):
        s = seed + i * 1337 if seed is not None else None
        noise = generate_perlin_noise_2d(shape, (int(freq_y), int(freq_x)), seed=s)
        total += amplitude * noise
        max_amp += amplitude
        amplitude *= persistence
        freq_y *= lacunarity
        freq_x *= lacunarity
        
    return total / max_amp


def generate_diamond_square_terrain(size=129, roughness=0.5, seed=None):
    """
    经典 Diamond-Square (菱形-方形 / 中点位移法) 分形地形
    :param size: 必须为 2^k + 1 (例如 65, 129, 257)
    :param roughness: 粗糙度参数 (0.0 ~ 1.0)
    """
    if seed is not None:
        np.random.seed(seed)
        
    grid = np.zeros((size, size), dtype=np.float64)
    # 初始化四个角点
    grid[0, 0] = np.random.uniform(-1.0, 1.0)
    grid[0, size - 1] = np.random.uniform(-1.0, 1.0)
    grid[size - 1, 0] = np.random.uniform(-1.0, 1.0)
    grid[size - 1, size - 1] = np.random.uniform(-1.0, 1.0)
    
    scale = 1.0
    step = size - 1
    
    while step > 1:
        half = step // 2
        
        # 1. Diamond step (菱形步)
        for y in range(half, size - 1, step):
            for x in range(half, size - 1, step):
                corners = [
                    grid[y - half, x - half],
                    grid[y - half, x + half],
                    grid[y + half, x - half],
                    grid[y + half, x + half]
                ]
                grid[y, x] = np.mean(corners) + np.random.uniform(-scale, scale)
                
        # 2. Square step (方形步)
        for y in range(0, size, half):
            shift = half if (y // half) % 2 == 0 else 0
            for x in range(shift, size, step):
                vals = []
                if y >= half:
                    vals.append(grid[y - half, x])
                if y + half < size:
                    vals.append(grid[y + half, x])
                if x >= half:
                    vals.append(grid[y, x - half])
                if x + half < size:
                    vals.append(grid[y, x + half])
                grid[y, x] = np.mean(vals) + np.random.uniform(-scale, scale)
                
        scale *= roughness
        step //= 2
        
    return grid


def generate_ridge_noise_terrain(shape, octaves=5, lacunarity=2.0, persistence=0.55, seed=None):
    """
    山脊/峡谷噪声 (Ridge Noise): 1.0 - |Noise|，产生陡峭险峻的山脊与山谷
    """
    if seed is not None:
        np.random.seed(seed)
    total = np.zeros(shape, dtype=np.float64)
    amplitude = 1.0
    weight = 1.0
    freq_y, freq_x = 2, 2
    
    for i in range(octaves):
        s = seed + i * 2026 if seed is not None else None
        n = generate_perlin_noise_2d(shape, (freq_y, freq_x), seed=s)
        signal = 1.0 - np.abs(n)
        signal = signal * signal  # 锐化山峰
        signal *= weight
        weight = np.clip(signal * 2.0, 0.0, 1.0)
        total += signal * amplitude
        amplitude *= persistence
        freq_y *= lacunarity
        freq_x *= lacunarity
        
    return (total - np.mean(total)) / (np.std(total) + 1e-6)


def generate_fault_formation_terrain(shape, iterations=150, seed=None):
    """
    断层位移算法 (Fault Formation): 模拟地质板块断层抬升与陷落
    """
    if seed is not None:
        np.random.seed(seed)
    ny, nx = shape
    heights = np.zeros(shape, dtype=np.float64)
    disp = 1.0
    
    gx, gy = np.meshgrid(np.arange(nx), np.arange(ny))
    
    for i in range(iterations):
        # 随机切割线: a * x + b * y + c = 0
        p1 = np.random.rand(2) * [nx, ny]
        p2 = np.random.rand(2) * [nx, ny]
        # 线方程 (y2 - y1)*(x - x1) - (x2 - x1)*(y - y1)
        side = (p2[1] - p1[1]) * (gx - p1[0]) - (p2[0] - p1[0]) * (gy - p1[1])
        mask = side > 0
        heights[mask] += disp
        heights[~mask] -= disp
        disp *= 0.985
        
    # 平滑滤波使其更具自然侵蚀感
    heights = gaussian_filter(heights, sigma=2.0)
    return (heights - np.mean(heights)) / (np.std(heights) + 1e-6)


# ==============================================================================
# 2. 地形风格合成 (Diverse Terrain Styles Synthesis)
# ==============================================================================

def synthesize_terrain_elevation(style, ny=81, nx=51, seed=None):
    """
    根据选定风格合成高程矩阵 (ny, nx)，返回高程值（米）
    :param style: 地形风格名称 ('rolling_hills', 'mountainous', 'ridge_valley', 'diamond_square', 'fault_plateau')
    :param ny: Y方向顶点数 (默认81)
    :param nx: X方向顶点数 (默认51)
    """
    if seed is None:
        seed = int(time.time() * 1000) % 1000000

    shape = (ny, nx)
    
    if style == 'rolling_hills':
        # 丘陵缓坡 (高程起伏约 80-220m，基准高程 150m)
        raw = generate_fbm_terrain(shape, octaves=4, persistence=0.45, lacunarity=2.0, base_res=(2, 2), seed=seed)
        elevation = 150.0 + 100.0 * raw
        elevation = gaussian_filter(elevation, sigma=1.2)
        
    elif style == 'mountainous':
        # 高山陡坡与山脉 (高程起伏约 200-600m，基准高程 200m)
        fbm = generate_fbm_terrain(shape, octaves=6, persistence=0.52, lacunarity=2.0, base_res=(3, 2), seed=seed)
        ridge = generate_ridge_noise_terrain(shape, octaves=5, seed=seed + 100)
        combined = 0.6 * fbm + 0.4 * (ridge / 3.0)
        elevation = 200.0 + 250.0 * combined
        
    elif style == 'ridge_valley':
        # 山脊与深谷地形
        ridge = generate_ridge_noise_terrain(shape, octaves=6, seed=seed)
        fbm = generate_fbm_terrain(shape, octaves=3, persistence=0.4, base_res=(2, 2), seed=seed + 200)
        combined = 0.7 * (ridge / 3.0) + 0.3 * fbm
        elevation = 220.0 + 200.0 * combined
        
    elif style == 'diamond_square':
        # 经典 Diamond-Square 分形地形
        # 生成 129x129 然后双三次/双线性重采样至 (ny, nx)
        ds = generate_diamond_square_terrain(size=129, roughness=0.55, seed=seed)
        from scipy.ndimage import zoom
        zy = ny / 129.0
        zx = nx / 129.0
        resampled = zoom(ds, (zy, zx), order=2)
        resampled = (resampled - np.mean(resampled)) / (np.std(resampled) + 1e-6)
        elevation = 180.0 + 160.0 * resampled
        
    elif style == 'fault_plateau':
        # 高原断层台地地形
        fault = generate_fault_formation_terrain(shape, iterations=160, seed=seed)
        fbm = generate_fbm_terrain(shape, octaves=4, persistence=0.45, base_res=(2, 2), seed=seed + 300)
        combined = 0.65 * (fault / 2.5) + 0.35 * fbm
        elevation = 250.0 + 180.0 * combined
        
    else:
        # 默认混合地形
        raw = generate_fbm_terrain(shape, octaves=5, persistence=0.5, base_res=(2, 2), seed=seed)
        elevation = 200.0 + 150.0 * raw

    # 保证最低高度大于零（通常大于50米）
    min_elev = np.min(elevation)
    if min_elev < 50.0:
        elevation += (50.0 - min_elev)

    return elevation


# ==============================================================================
# 3. 三维网格白模构建与导出 (Mesh Construction and PLY Export)
# ==============================================================================

def build_terrain_mesh(elevation, width_x=6000.0, height_y=8000.0):
    """
    根据高程矩阵构建 6km * 8km 的三维网格白模
    nx=51, ny=81 -> (nx-1)*(ny-1)*2 = 50 * 80 * 2 = 8000 面
    :return: vertices (N, 3), triangles (M, 3)
    """
    ny, nx = elevation.shape
    xs = np.linspace(0.0, width_x, nx, dtype=np.float32)
    ys = np.linspace(0.0, height_y, ny, dtype=np.float32)
    
    grid_x, grid_y = np.meshgrid(xs, ys)
    vertices = np.stack([grid_x, grid_y, elevation.astype(np.float32)], axis=-1).reshape(-1, 3)
    
    triangles = []
    for j in range(ny - 1):
        for i in range(nx - 1):
            # 矩形网格四个顶点编号
            v00 = j * nx + i
            v10 = j * nx + (i + 1)
            v01 = (j + 1) * nx + i
            v11 = (j + 1) * nx + (i + 1)
            
            # 每个矩形划分为2个三角形
            triangles.append([v00, v10, v11])
            triangles.append([v00, v11, v01])
            
    triangles = np.array(triangles, dtype=np.int32)
    return vertices, triangles


def save_mesh_to_ply(filepath, vertices, triangles, write_ascii=False):
    """
    保存三维网格白模至 .ply 文件。
    若 Open3D 可用则使用 Open3D 写入，否则使用原生矢量化 PLY 写入器。
    """
    os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
    
    if HAS_OPEN3D:
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(vertices.astype(np.float64))
        mesh.triangles = o3d.utility.Vector3iVector(triangles)
        mesh.compute_vertex_normals()
        # 赋予白模中性浅白/微灰纯色 (225, 225, 225)
        colors = np.ones((len(vertices), 3), dtype=np.float64) * 0.88
        mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
        o3d.io.write_triangle_mesh(filepath, mesh, write_ascii=write_ascii)
    else:
        # 原生高效写入 PLY (支持 ASCII / Binary Little Endian)
        num_verts = len(vertices)
        num_faces = len(triangles)
        
        if write_ascii:
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write("ply\nformat ascii 1.0\ncomment White model terrain mesh\n")
                f.write(f"element vertex {num_verts}\n")
                f.write("property float x\nproperty float y\nproperty float z\n")
                f.write(f"element face {num_faces}\n")
                f.write("property list uchar int vertex_indices\nend_header\n")
                for v in vertices:
                    f.write(f"{v[0]:.3f} {v[1]:.3f} {v[2]:.3f}\n")
                for tri in triangles:
                    f.write(f"3 {tri[0]} {tri[1]} {tri[2]}\n")
        else:
            header = (
                f"ply\nformat binary_little_endian 1.0\ncomment White model terrain mesh\n"
                f"element vertex {num_verts}\n"
                f"property float x\nproperty float y\nproperty float z\n"
                f"element face {num_faces}\n"
                f"property list uchar int vertex_indices\nend_header\n"
            ).encode('ascii')
            
            with open(filepath, 'wb') as f:
                f.write(header)
                # 写入顶点 (float32)
                f.write(vertices.astype('<f4').tobytes())
                # 写入三角面 (uint8 为3, int32 为顶点序号)
                faces_data = bytearray()
                three = np.uint8(3).tobytes()
                for tri in triangles:
                    faces_data.extend(three)
                    faces_data.extend(tri.astype('<i4').tobytes())
                f.write(faces_data)


# ==============================================================================
# 4. 主程序命令行入口
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="地面地形白模生成脚本：生成指定数量的 6km*8km、约8000面三维地形白模(.ply)"
    )
    parser.add_argument("-n", "--count", type=int, default=3, help="生成的地面场景白模个数 (默认: 3)")
    parser.add_argument("-o", "--output_dir", type=str, default="./terrain_models",
                        help="输出保存.ply白模的文件夹路径 (默认: ./terrain_models)")
    parser.add_argument("--width", type=float, default=6000.0, help="X轴范围米数 (默认: 6000.0)")
    parser.add_argument("--height", type=float, default=8000.0, help="Y轴范围米数 (默认: 8000.0)")
    parser.add_argument("--algorithm", type=str, default="random",
                        choices=["random", "rolling_hills", "mountainous", "ridge_valley", "diamond_square", "fault_plateau"],
                        help="地形生成算法风格 (默认: random 随机组合)")
    parser.add_argument("--seed", type=int, default=None, help="基础随机种子 (可选)")
    parser.add_argument("--ascii", action="store_true", help="保存为 ASCII PLY 格式 (默认保存为更小更快的 Binary PLY)")

    args = parser.parse_args()

    # 确定网格顶点数：nx=51, ny=81 -> (51-1)*(81-1)*2 = 50*80*2 = 8000 面
    nx, ny = 51, 81
    expected_faces = (nx - 1) * (ny - 1) * 2

    out_dir = os.path.abspath(args.output_dir)
    os.makedirs(out_dir, exist_ok=True)

    print("=" * 70)
    print("地面地形白模生成器 (Ground Terrain White-Model Generator)")
    print(f"  * 生成数量: {args.count} 个场景")
    print(f"  * 场景范围: {args.width:.1f} m (X) x {args.height:.1f} m (Y)")
    print(f"  * 目标三角面数: {expected_faces} 面 (网格顶点数: {nx} x {ny} = {nx * ny})")
    print(f"  * 输出目录: {out_dir}")
    print(f"  * 地形算法模式: {args.algorithm}")
    print("=" * 70)

    available_styles = ["rolling_hills", "mountainous", "ridge_valley", "diamond_square", "fault_plateau"]
    base_seed = args.seed if args.seed is not None else int(time.time())

    for idx in range(1, args.count + 1):
        scene_seed = base_seed + idx * 7919
        style = args.algorithm if args.algorithm != "random" else available_styles[(idx - 1) % len(available_styles)]

        print(f"\n[{idx}/{args.count}] 正在生成场景 scene_{idx:03d} (风格: {style}, 种子: {scene_seed})...")
        t0 = time.time()

        elevation = synthesize_terrain_elevation(style, ny=ny, nx=nx, seed=scene_seed)
        vertices, triangles = build_terrain_mesh(elevation, width_x=args.width, height_y=args.height)

        ply_filename = f"terrain_scene_{idx:03d}.ply"
        ply_path = os.path.join(out_dir, ply_filename)
        save_mesh_to_ply(ply_path, vertices, triangles, write_ascii=args.ascii)

        t_elapsed = time.time() - t0
        z_min, z_max = np.min(elevation), np.max(elevation)
        file_size_kb = os.path.getsize(ply_path) / 1024.0

        print(f"  -> 保存成功: {ply_path}")
        print(f"     顶点数: {len(vertices)}, 三角面数: {len(triangles)}, 高程范围: [{z_min:.1f}m, {z_max:.1f}m]")
        print(f"     文件大小: {file_size_kb:.1f} KB, 耗时: {t_elapsed:.2f}s")

    print("\n" + "=" * 70)
    print(f"所有 {args.count} 个地面场景白模已全部成功生成并存入: {out_dir}")
    print("=" * 70)


if __name__ == "__main__":
    main()
