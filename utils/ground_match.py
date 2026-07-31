import open3d as o3d
import numpy as np
import rasterio
from pyproj import CRS, Transformer
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
import os

def evaluate_mesh_vs_geotiff(mesh_path, tiff_path, gps_lat, gps_lon, gps_alt, yaw_deg, save_dir):
    print("\n[*] 正在加载数据...")
    print(f"  - Mesh: {mesh_path}")
    print(f"  - TIFF: {tiff_path}")

    # 1. 读取 GeoTIFF
    try:
        src = rasterio.open(tiff_path)
        tiff_crs = src.crs
        print(f"[*] 成功读取 GeoTIFF, 目标投影坐标系为: {tiff_crs}")
    except Exception as e:
        raise RuntimeError(f"[!] GeoTIFF 读取失败: {e}")

    # 2. 读取 白模 Mesh
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    if not mesh.has_vertices():
        raise ValueError("[!] 读取到的 Mesh 为空或没有顶点数据！")
    vertices = np.asarray(mesh.vertices)
    print(f"[*] 成功读取 Mesh, 顶点数量: {len(vertices)}")

    # 3. 建立以 GPS 原点为中心的局部正交投影坐标系 (AEQD, 单位: 米)
    # 这将完美解决经纬度(度)与点云(米)之间的单位错位问题
    local_crs_str = f"+proj=aeqd +lat_0={gps_lat} +lon_0={gps_lon} +datum=WGS84 +units=m"
    local_crs = CRS.from_string(local_crs_str)

    # 构建从局部 AEQD(米) 到 TIFF 坐标系(无论度还是米) 的高精度转换器
    transformer_to_tiff = Transformer.from_crs(local_crs, tiff_crs, always_xy=True)

    # 4. 旋转 Mesh 到正北方向
    yaw_rad = np.radians(yaw_deg)
    # 绕 Z 轴的旋转矩阵
    R_z = np.array([
        [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
        [np.sin(yaw_rad),  np.cos(yaw_rad), 0],
        [0,                0,               1]
    ])
    rotated_vertices = np.dot(vertices, R_z.T)

    # 5. 将局部米制坐标转换为 TIFF 的全局坐标
    print("[*] 正在进行坐标系单位投影与匹配...")
    # rotated_vertices[:, 0] 是向东的米数, [:, 1] 是向北的米数
    tiff_x, tiff_y = transformer_to_tiff.transform(rotated_vertices[:, 0], rotated_vertices[:, 1])

    # 海拔高度直接加上基准海拔
    tiff_z = rotated_vertices[:, 2] + gps_alt
    trans_mesh_vertices = np.column_stack((tiff_x, tiff_y, tiff_z))

    # 验证是否在 TIFF 范围内
    min_x, min_y = np.min(tiff_x), np.min(tiff_y)
    max_x, max_y = np.max(tiff_x), np.max(tiff_y)

    if (min_x > src.bounds.right or max_x < src.bounds.left or
            min_y > src.bounds.top or max_y < src.bounds.bottom):
        raise ValueError(f"[!] 变换后的网格完全不在 GeoTIFF 的地理范围内！\n"
                         f"    TIFF 边界: X({src.bounds.left:.4f} ~ {src.bounds.right:.4f}), Y({src.bounds.bottom:.4f} ~ {src.bounds.top:.4f})\n"
                         f"    Mesh 边界: X({min_x:.4f} ~ {max_x:.4f}), Y({min_y:.4f} ~ {max_y:.4f})")

    # 6. 逆向栅格采样计算误差
    print("[*] 正在对齐真实高程进行采样...")
    errors = []
    sampled_indices = []

    # 获取图像数据 (波段 1)
    dem_data = src.read(1)
    nodata_val = src.nodata

    for idx, (x, y, z) in enumerate(trans_mesh_vertices):
        # 将地理/投影坐标转换为矩阵的 行(row), 列(col)
        row, col = src.index(x, y)

        # 边界保护与有效值检查
        if 0 <= row < src.height and 0 <= col < src.width:
            real_z = dem_data[row, col]
            if real_z != nodata_val:
                error = z - real_z  # 误差 = 重建高度 - 真实高度
                errors.append(error)
                sampled_indices.append(idx)

    if not errors:
        raise ValueError("[!] 采样失败，该区域的高程数据可能全部是无数据(NoData)。")

    errors = np.array(errors)

    # 7. 统计指标计算
    rmse = np.sqrt(np.mean(errors**2))
    mae = np.mean(np.abs(errors))
    print(f"\n" + "="*45)
    print(f"📊 评估报告:")
    print(f" - 成功匹配顶点数: {len(errors)}")
    print(f" - RMSE (均方根误差): {rmse:.4f} 米")
    print(f" - MAE  (平均绝对误差): {mae:.4f} 米")
    print(f" - 最大正偏差 (重建偏高): {np.max(errors):.4f} 米")
    print(f" - 最大负偏差 (重建偏低): {np.min(errors):.4f} 米")
    print("="*45)

    # 8. 生成带颜色的 3D 热力图
    # 将没有采样到的顶点涂成灰色
    vertex_colors = np.full((len(vertices), 3), 0.5)

    # 使用 cmap 将误差映射为颜色 (蓝色偏低，红色偏高，白色正常)
    norm = Normalize(vmin=-np.max(np.abs(errors)), vmax=np.max(np.abs(errors)))
    cmap = plt.get_cmap('coolwarm')

    for i, idx in enumerate(sampled_indices):
        color = cmap(norm(errors[i]))[:3]
        vertex_colors[idx] = color

    mesh.vertex_colors = o3d.utility.Vector3dVector(vertex_colors)
    mesh.compute_vertex_normals()

    # 9. 保存带颜色信息的白模文件
    os.makedirs(save_dir, exist_ok=True)
    file_name = os.path.basename(mesh_path)
    save_path = os.path.join(save_dir, file_name)
    o3d.io.write_triangle_mesh(save_path, mesh)
    print(f"[*] 带有误差热力颜色的白模已成功保存至: {save_path}")

    print("\n[*] 正在启动 3D 误差热力图渲染...")
    o3d.visualization.draw_geometries([mesh], window_name="GeoTIFF Evaluation Heatmap (Blue=Low, Red=High)")

if __name__ == "__main__":
    # 请填入真实测试数据
    MESH_FILE = "/home/cx/Documents/codes/pointcloud_607_roscpp/dataset/result/Wild/03/terrain_1784873715_215031351.ply"
    TIFF_FILE = "/home/cx/Documents/codes/pointcloud_607_roscpp/dataset/GeoInfo/Wild/01/n18_e109_1arc_v3.tif"
    SAVE_DIR = "/home/cx/Documents/codes/pointcloud_607_roscpp/dataset/result/Wild/03/evaluated_mesh"
    # 白模原点(0,0,0)对应的真实 GPS 坐标及海拔
    GPS_LAT = 18.265754699707031
    GPS_LON = 109.609924316406250
    GPS_ALT = 209.000000000000000

    # 局部坐标系 X 轴相对于正北方向的角度 (根据你的里程计/IMU校准)
    YAW_DEGREES = -15.661010742187500

    try:
        evaluate_mesh_vs_geotiff(MESH_FILE, TIFF_FILE, GPS_LAT, GPS_LON, GPS_ALT, YAW_DEGREES, SAVE_DIR)
    except Exception as e:
        print(f"\n[!] 发生错误: {e}")