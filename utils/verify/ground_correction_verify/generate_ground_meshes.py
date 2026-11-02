#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地面白模生成脚本 (generate_ground_meshes.py)

功能：
    根据输入的数量，使用多种常见地面地形生成算法（Perlin/fBm 噪声、Diamond-Square 分形、
    多高斯起伏丘陵、阶梯断层高原、尖锐山脊多重分形、风蚀沙丘波浪等），
    批量生成指定数量的 .ply 地面白模。
    每个白模的物理尺寸约为 6km * 4km，面数约为 4000 面。

使用环境：
    pointcloud_607 (/home/asahichinn/miniconda3/envs/pointcloud_607)
"""

import os
import sys
import argparse
import numpy as np
from scipy.ndimage import zoom, gaussian_filter
import open3d as o3d


def compute_grid_dimensions(length=6000.0, width=4000.0, target_faces=4000):
    """
    根据目标面数和长宽比计算最佳网格尺寸 (nx, ny)，使单元格尽量接近正方形。
    总面数 = 2 * (nx - 1) * (ny - 1)
    """
    aspect_ratio = length / width  # 6000 / 4000 = 1.5
    # 2 * (aspect_ratio * (ny - 1)) * (ny - 1) ~= target_faces
    ny_minus_1 = int(np.round(np.sqrt(target_faces / (2.0 * aspect_ratio))))
    nx_minus_1 = int(np.round(ny_minus_1 * aspect_ratio))
    
    nx = nx_minus_1 + 1
    ny = ny_minus_1 + 1
    actual_faces = 2 * (nx - 1) * (ny - 1)
    return nx, ny, actual_faces


# ==============================================================================
# 地形生成算法集合
# ==============================================================================

def generate_perlin_fbm(nx, ny, seed=None, octaves=4, persistence=0.5, lacunarity=2.0, height_scale=120.0):
    """
    算法 1: 分形布朗运动 (fBm) / 多频梯度噪声
    特点：自然的起伏山地与丘陵，具有多尺度自相似细节。
    """
    rng = np.random.RandomState(seed)
    height_map = np.zeros((nx, ny), dtype=np.float32)
    frequency = 1.0
    amplitude = 1.0
    total_amplitude = 0.0

    for _ in range(octaves):
        # 生成低频随机网格并进行双三次插值
        low_nx = max(3, int(nx * frequency / 8))
        low_ny = max(3, int(ny * frequency / 8))
        grid = rng.uniform(-1.0, 1.0, (low_nx, low_ny))
        
        # 放大插值到 (nx, ny)
        layer = zoom(grid, (nx / low_nx, ny / low_ny), order=3)[:nx, :ny]
        height_map += layer * amplitude
        total_amplitude += amplitude
        amplitude *= persistence
        frequency *= lacunarity

    height_map /= total_amplitude
    # 平移使最低点约为 0
    height_map -= height_map.min()
    height_map = height_map / (height_map.max() + 1e-6) * height_scale
    return height_map


def generate_diamond_square(nx, ny, seed=None, roughness=0.6, height_scale=180.0):
    """
    算法 2: Diamond-Square (菱形-方形) 分形高度场算法
    特点：经典计算机图形学分形地形，山峰险峻、沟壑纵横。
    """
    rng = np.random.RandomState(seed)
    # 取大于 max(nx, ny) 的 2^k + 1 尺寸
    k = int(np.ceil(np.log2(max(nx, ny) - 1)))
    grid_size = (1 << k) + 1
    
    H = np.zeros((grid_size, grid_size), dtype=np.float32)
    H[0, 0] = rng.uniform(-1.0, 1.0)
    H[0, -1] = rng.uniform(-1.0, 1.0)
    H[-1, 0] = rng.uniform(-1.0, 1.0)
    H[-1, -1] = rng.uniform(-1.0, 1.0)

    step = grid_size - 1
    scale = 1.0
    while step > 1:
        half = step // 2
        # Diamond step
        for i in range(half, grid_size, step):
            for j in range(half, grid_size, step):
                avg = (H[i-half, j-half] + H[i-half, j+half] + 
                       H[i+half, j-half] + H[i+half, j+half]) * 0.25
                H[i, j] = avg + rng.uniform(-scale, scale)

        # Square step
        for i in range(0, grid_size, half):
            for j in range((i + half) % step, grid_size, step):
                vals = []
                if i - half >= 0: vals.append(H[i-half, j])
                if i + half < grid_size: vals.append(H[i+half, j])
                if j - half >= 0: vals.append(H[i, j-half])
                if j + half < grid_size: vals.append(H[i, j+half])
                H[i, j] = np.mean(vals) + rng.uniform(-scale, scale)

        step //= 2
        scale *= roughness

    # 缩放到指定尺寸 (nx, ny)
    resampled = zoom(H, (nx / grid_size, ny / grid_size), order=3)[:nx, :ny]
    resampled -= resampled.min()
    resampled = resampled / (resampled.max() + 1e-6) * height_scale
    return resampled


def generate_multi_gaussian_hills(nx, ny, seed=None, num_hills=25, height_scale=90.0):
    """
    算法 3: 多高斯分布叠加的连绵丘陵与洼地
    特点：光滑温和的丘陵和平缓谷地，典型盆地/丘陵地貌。
    """
    rng = np.random.RandomState(seed)
    X, Y = np.meshgrid(np.linspace(0, 1, nx), np.linspace(0, 1, ny), indexing='ij')
    height_map = np.zeros((nx, ny), dtype=np.float32)

    # 叠加基础缓坡
    height_map += 0.2 * np.sin(2 * np.pi * X * rng.uniform(0.8, 1.5)) * np.cos(2 * np.pi * Y * rng.uniform(0.8, 1.5))

    for _ in range(num_hills):
        cx = rng.uniform(0.05, 0.95)
        cy = rng.uniform(0.05, 0.95)
        sx = rng.uniform(0.08, 0.25)
        sy = rng.uniform(0.08, 0.25)
        amp = rng.uniform(-0.8, 1.2)  # 包含山丘和凹陷洼地
        gauss = amp * np.exp(-(((X - cx) ** 2) / (2 * sx ** 2) + ((Y - cy) ** 2) / (2 * sy ** 2)))
        height_map += gauss

    height_map = gaussian_filter(height_map, sigma=1.0)
    height_map -= height_map.min()
    height_map = height_map / (height_map.max() + 1e-6) * height_scale
    return height_map


def generate_fault_formation_stepped(nx, ny, seed=None, num_faults=35, terrace_steps=6, height_scale=140.0):
    """
    算法 4: 断层位移叠加阶梯阶地算法
    特点：模拟地质断裂带、梯田式高原与峡谷断层阶地。
    """
    rng = np.random.RandomState(seed)
    X, Y = np.meshgrid(np.linspace(-1, 1, nx), np.linspace(-1, 1, ny), indexing='ij')
    height_map = np.zeros((nx, ny), dtype=np.float32)

    for i in range(num_faults):
        # 随机直线方程 a*x + b*y + d = 0
        theta = rng.uniform(0, 2 * np.pi)
        a = np.cos(theta)
        b = np.sin(theta)
        d = rng.uniform(-0.8, 0.8)
        disp = rng.uniform(0.05, 0.3) / (1.0 + 0.05 * i)
        dist = a * X + b * Y + d
        height_map += np.where(dist > 0, disp, -disp)

    # 平滑过渡
    height_map = gaussian_filter(height_map, sigma=1.5)
    
    # 局部阶梯化 (Terracing) 模拟高原台地
    if terrace_steps > 0:
        norm_h = (height_map - height_map.min()) / (height_map.max() - height_map.min() + 1e-6)
        stepped = np.round(norm_h * terrace_steps) / terrace_steps
        height_map = 0.65 * height_map + 0.35 * (stepped * (height_map.max() - height_map.min()) + height_map.min())
        height_map = gaussian_filter(height_map, sigma=0.8)

    height_map -= height_map.min()
    height_map = height_map / (height_map.max() + 1e-6) * height_scale
    return height_map


def generate_ridged_multifractal(nx, ny, seed=None, octaves=4, height_scale=160.0):
    """
    算法 5: 尖锐山脊多重分形算法 (Ridged Multifractal)
    特点：刀刃般陡峭的山脊、连绵的峡谷与尖锐山脊线。
    """
    rng = np.random.RandomState(seed)
    height_map = np.zeros((nx, ny), dtype=np.float32)
    weight = 1.0
    frequency = 1.0

    for o in range(octaves):
        low_nx = max(3, int(nx * frequency / 6))
        low_ny = max(3, int(ny * frequency / 6))
        grid = rng.uniform(-1.0, 1.0, (low_nx, low_ny))
        layer = zoom(grid, (nx / low_nx, ny / low_ny), order=3)[:nx, :ny]
        
        # 核心算子: 1 - |layer| 产生尖锐山脊
        signal = 1.0 - np.abs(layer)
        signal = signal ** 2
        height_map += signal * weight
        
        weight = np.clip(signal * 2.0, 0.1, 1.0) * 0.5
        frequency *= 2.0

    height_map -= height_map.min()
    height_map = height_map / (height_map.max() + 1e-6) * height_scale
    return height_map


def generate_dune_waves(nx, ny, seed=None, height_scale=60.0):
    """
    算法 6: 风成沙丘与波纹起伏地形
    特点：具有明显方向性波谷与不对称缓坡/陡坡沙丘。
    """
    rng = np.random.RandomState(seed)
    angle = rng.uniform(0.2, 0.8) * np.pi
    u = np.cos(angle)
    v = np.sin(angle)
    
    X, Y = np.meshgrid(np.linspace(0, 6000, nx), np.linspace(-2000, 2000, ny), indexing='ij')
    proj = (X * u + Y * v) / 800.0  # 波长约 800 米
    
    # 不对称三角波模拟迎风坡与背风坡
    wave = np.sin(2 * np.pi * proj) + 0.35 * np.sin(4 * np.pi * proj)
    
    # 叠加低频起伏扰动
    low_nx = max(4, nx // 6)
    low_ny = max(4, ny // 6)
    perturb = zoom(rng.uniform(-1, 1, (low_nx, low_ny)), (nx / low_nx, ny / low_ny), order=3)[:nx, :ny]
    
    height_map = wave * 0.7 + perturb * 0.3
    height_map = gaussian_filter(height_map, sigma=0.8)
    height_map -= height_map.min()
    height_map = height_map / (height_map.max() + 1e-6) * height_scale
    return height_map


TERRAIN_ALGORITHMS = [
    ("perlin_fbm", "分形布朗运动丘陵 (Perlin fBm Hills)", generate_perlin_fbm),
    ("diamond_square", "菱形-方形分形山脉 (Diamond-Square Mountains)", generate_diamond_square),
    ("multi_gaussian_hills", "多高斯起伏原野 (Multi-Gaussian Rolling Hills)", generate_multi_gaussian_hills),
    ("fault_formation_stepped", "断层阶梯台地 (Stepped Fault Plateaus)", generate_fault_formation_stepped),
    ("ridged_multifractal", "尖锐山脊峡谷 (Ridged Multifractal Valleys)", generate_ridged_multifractal),
    ("dune_waves", "风蚀波纹沙丘 (Wind-blown Sand Dunes)", generate_dune_waves),
]


# ==============================================================================
# 网格构建与 PLY 白模导出
# ==============================================================================

def create_terrain_mesh(height_map, length=6000.0, width=4000.0, white_color=(245, 245, 245)):
    """
    根据高度场构建三维网格：
    X 轴范围: [0, length] (6km)
    Y 轴范围: [-width/2, width/2] (4km，中心轴位于 Y = 0)
    Z 轴: 高程
    设置纯白/米白颜色构成白模。
    """
    nx, ny = height_map.shape
    xs = np.linspace(0.0, length, nx, dtype=np.float32)
    ys = np.linspace(-width / 2.0, width / 2.0, ny, dtype=np.float32)
    
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    vertices = np.stack([X.ravel(), Y.ravel(), height_map.ravel()], axis=-1)
    
    # 生成三角网格面 (每个矩形单元划分为 2 个三角形)
    triangles = []
    for i in range(nx - 1):
        for j in range(ny - 1):
            v0 = i * ny + j
            v1 = (i + 1) * ny + j
            v2 = i * ny + (j + 1)
            v3 = (i + 1) * ny + (j + 1)
            # 对角线划分: (v0, v1, v2) 与 (v1, v3, v2)
            triangles.append([v0, v1, v2])
            triangles.append([v1, v3, v2])
            
    triangles = np.array(triangles, dtype=np.int32)
    
    # 构造 Open3D TriangleMesh
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(triangles)
    
    # 计算法向量
    mesh.compute_vertex_normals()
    mesh.compute_triangle_normals()
    
    # 设置白模顶点颜色 (归一化到 0.0 - 1.0)
    color = np.array(white_color, dtype=np.float64) / 255.0
    colors = np.tile(color, (len(vertices), 1))
    mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
    
    return mesh


def generate_ground_meshes(count, output_dir, length=6000.0, width=4000.0, target_faces=4000, seed_base=1000):
    """
    批量生成指定数量的地貌白模并保存到目标文件夹
    """
    os.makedirs(output_dir, exist_ok=True)
    nx, ny, actual_faces = compute_grid_dimensions(length, width, target_faces)
    
    print("=" * 70)
    print("地面白模批量生成器 (Ground Mesh Generator)")
    print("=" * 70)
    print(f"计划生成数量   : {count} 个")
    print(f"地面空间范围   : 长度(X)={length/1000.0:.1f} km, 宽度(Y)={width/1000.0:.1f} km (范围 X:[0, {length:.0f}], Y:[{-width/2:.0f}, {width/2:.0f}])")
    print(f"网格顶点分辨率 : {nx} x {ny} = {nx * ny} 个顶点")
    print(f"网格三角面数   : {actual_faces} 面 (目标约 {target_faces} 面)")
    print(f"保存目标文件夹 : {os.path.abspath(output_dir)}")
    print("-" * 70)

    generated_files = []

    for idx in range(count):
        algo_name, algo_desc, algo_fn = TERRAIN_ALGORITHMS[idx % len(TERRAIN_ALGORITHMS)]
        current_seed = seed_base + idx * 137
        
        # 生成高度图
        height_map = algo_fn(nx, ny, seed=current_seed)
        
        # 构建 Open3D 网格
        mesh = create_terrain_mesh(height_map, length=length, width=width)
        
        file_name = f"ground_terrain_{idx+1:03d}_{algo_name}.ply"
        file_path = os.path.join(output_dir, file_name)
        
        # 写入 PLY 文件 (以二进制格式写入，速度快、精度高)
        success = o3d.io.write_triangle_mesh(file_path, mesh, write_vertex_normals=True, write_vertex_colors=True)
        if not success:
            raise IOError(f"无法保存 PLY 模型到: {file_path}")
            
        z_min = float(height_map.min())
        z_max = float(height_map.max())
        z_delta = z_max - z_min
        
        print(f"[{idx+1}/{count}] 成功生成: {file_name}")
        print(f"       算法: {algo_desc}")
        print(f"       高程范围: [{z_min:.1f}m ~ {z_max:.1f}m] (相对高差 {z_delta:.1f}m), 面数: {len(mesh.triangles)}")
        
        generated_files.append(file_path)

    print("=" * 70)
    print(f"全部 {count} 个地面白模生成完毕！均已保存在: {os.path.abspath(output_dir)}")
    print("=" * 70)
    return generated_files


def main():
    parser = argparse.ArgumentParser(
        description="地面白模批量生成脚本：输入个数，生成约6km*4km、约4000面的地面.ply白模。"
    )
    parser.add_argument(
        "-n", "--count",
        type=int,
        default=None,
        help="需要生成的地面白模个数 (例如: 5)"
    )
    parser.add_argument(
        "-o", "--output_dir",
        type=str,
        default=os.path.join(os.path.dirname(__file__), "output_ground_meshes"),
        help="地面白模保存目录 (默认: 当前脚本目录下的 output_ground_meshes)"
    )
    parser.add_argument(
        "--length",
        type=float,
        default=6000.0,
        help="地形长度 (X轴方向，单位: 米，默认 6000.0)"
    )
    parser.add_argument(
        "--width",
        type=float,
        default=4000.0,
        help="地形宽度 (Y轴方向，单位: 米，默认 4000.0)"
    )
    parser.add_argument(
        "--target_faces",
        type=int,
        default=4000,
        help="目标三角面数 (默认约 4000 面)"
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="随机数种子基数 (默认: 42)"
    )

    args = parser.parse_args()

    # 若未在命令行传入个数，则支持交互式输入或提示
    count = args.count
    if count is None:
        try:
            val = input("请输入要生成的地面白模个数 (默认 5): ").strip()
            count = int(val) if val else 5
        except (ValueError, EOFError):
            count = 5

    if count <= 0:
        print("[错误] 生成个数必须大于 0！")
        sys.exit(1)

    generate_ground_meshes(
        count=count,
        output_dir=args.output_dir,
        length=args.length,
        width=args.width,
        target_faces=args.target_faces,
        seed_base=args.seed
    )


if __name__ == "__main__":
    main()
