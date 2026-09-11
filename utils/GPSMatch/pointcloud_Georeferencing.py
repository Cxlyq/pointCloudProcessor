import os
import json
import numpy as np
import open3d as o3d
import rasterio
from pyproj import Transformer
from scipy.optimize import dual_annealing
from scipy.spatial.transform import Rotation
from scipy.ndimage import map_coordinates

def create_initial_matrix(lon, lat, alt, heading):
    """
    根据无人机的经纬度、高度和航向角，生成初始的4x4变换矩阵。
    航向角(heading)通常是以正北为0度，顺时针增加。
    在数学坐标系中（X为东，Y为北），我们需要将其转换为绕Z轴的旋转角。
    """
    # 动态计算所在的UTM投影带
    utm_zone = int((lon + 180) / 6) + 1
    crs_utm = f"+proj=utm +zone={utm_zone} +datum=WGS84 +units=m +no_defs"

    # 建立经纬度到UTM的坐标转换器
    transformer_to_utm = Transformer.from_crs("EPSG:4326", crs_utm, always_xy=True)
    x_utm, y_utm = transformer_to_utm.transform(lon, lat)

    # 将航向角(正北为0, 顺时针) 转换为 数学角度(正东X轴为0, 逆时针)
    math_angle = 90.0 - heading
    r_z = Rotation.from_euler('z', math_angle, degrees=True).as_matrix()

    # 构建初始4x4变换矩阵
    M_init = np.eye(4)
    M_init[:3, :3] = r_z
    M_init[:3, 3] = [x_utm, y_utm, alt]

    return M_init, transformer_to_utm, crs_utm

def get_dem_interpolator(dem_path, crs_utm):
    """
    读取GeoTIFF DEM文件，并返回一个可以在UTM坐标下获取高程的闭包函数。
    """
    dataset = rasterio.open(dem_path)
    dem_data = dataset.read(1)
    dem_transform = dataset.transform
    dem_crs = dataset.crs

    # 从UTM转换回DEM所在坐标系（通常为WGS84经纬度）的转换器
    transformer_to_dem = Transformer.from_crs(crs_utm, dem_crs, always_xy=True)

    def sample_elevation(x_utm_array, y_utm_array):
        # 1. 转回DEM原生坐标系
        x_dem, y_dem = transformer_to_dem.transform(x_utm_array, y_utm_array)
        # 2. 将地理坐标转为像素行列号
        inv_transform = ~dem_transform
        cols, rows = inv_transform * (x_dem, y_dem)
        # 3. 使用scipy进行快速的双线性插值
        coords = np.vstack((rows, cols))
        # 避免超出边界
        z_dem = map_coordinates(dem_data, coords, order=1, mode='nearest')
        return z_dem

    return sample_elevation, dataset

def process_point_cloud_registration(ply_path, dem_path, gps_params):
    # ---------------- 1. 解析初始参数与构建初始世界坐标 ----------------
    print("正在加载初始参数与坐标系...")
    lon, lat, alt, heading = gps_params['lon'], gps_params['lat'], gps_params['alt'], gps_params['heading']
    M_init, _, crs_utm = create_initial_matrix(lon, lat, alt, heading)

    # ---------------- 2. 降采样点云以加速优化 ----------------
    print("正在读取并降采样点云...")
    pcd = o3d.io.read_point_cloud(ply_path)
    # 采用体素降采样，将点云数量控制在几千个以内，大幅提升优化速度
    pcd_down = pcd.voxel_down_sample(voxel_size=2.0)
    points_local = np.asarray(pcd_down.points)

    # 如果点云依然太大，进行随机抽样
    if len(points_local) > 5000:
        indices = np.random.choice(len(points_local), 5000, replace=False)
        points_local = points_local[indices]

    # 将局部点云转换到初始UTM地理世界坐标系
    ones = np.ones((points_local.shape[0], 1))
    points_homogeneous = np.hstack((points_local, ones))
    points_world_init = (M_init @ points_homogeneous.T).T[:, :3]

    # ---------------- 3. 准备DEM数据的高程插值器 ----------------
    print("正在加载DEM地图数据...")
    get_elevation, _ = get_dem_interpolator(dem_path, crs_utm)

    # ---------------- 4. 定义优化目标函数 (模拟退火) ----------------
    def objective_function(params):
        # params: [tx, ty, tz, rx, ry, rz]
        tx, ty, tz, rx, ry, rz = params

        # 构建微调矩阵
        R_opt = Rotation.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()

        # 对点云应用微调 (旋转 + 平移)
        points_transformed = points_world_init @ R_opt.T + np.array([tx, ty, tz])

        # 提取XY获取DEM对应高程，提取Z进行比对
        z_dem = get_elevation(points_transformed[:, 0], points_transformed[:, 1])
        z_points = points_transformed[:, 2]

        # 目标：最小化点云高度与DEM高度的平均绝对误差 (MAE)
        error = np.mean(np.abs(z_points - z_dem))
        return error

    # 设定优化边界 [tx, ty, tz, rx, ry, rz]
    # 平移限制在 ±30米，旋转限制在 ±5度，避免偏离无人机初始GPS太远导致算法滑动到底部深谷
    bounds = [
        (-30, 30),  # tx (m)
        (-30, 30),  # ty (m)
        (-30, 30),  # tz (m)
        (-5, 5),    # rx (deg)
        (-5, 5),    # ry (deg)
        (-5, 5)     # rz (deg)
    ]

    print("开始执行模拟退火全局优化 (Scipy dual_annealing)... 这可能需要一点时间。")
    # dual_annealing 是带有局部搜索策略的模拟退火算法，寻找全局最优解效果极佳
    result = dual_annealing(objective_function, bounds=bounds, maxiter=200)

    best_params = result.x
    best_error = result.fun
    print(f"优化完成！最小平均高程误差: {best_error:.3f} 米")
    print(f"微调参数: 平移 {best_params[:3].round(2)} m, 旋转 {best_params[3:].round(2)} 度")

    # ---------------- 5. 计算并合并最终的校正矩阵 ----------------
    tx, ty, tz, rx, ry, rz = best_params
    M_opt = np.eye(4)
    M_opt[:3, :3] = Rotation.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
    M_opt[:3, 3] = [tx, ty, tz]

    # 最终矩阵 = 微调矩阵 × 初始矩阵 (M_total将局部点云一步直接映射到准确的地理世界)
    M_final = M_opt @ M_init

    # ---------------- 6. 保存结果 ----------------
    base_filename = os.path.splitext(ply_path)[0]
    matrix_output_path = f"{base_filename}_correctionMetrix.json"
    deviation_output_path = f"{base_filename}_deviation.txt"

    # 保存矩阵为JSON格式，具有极高的人类可读性和跨平台兼容性
    matrix_data = {
        "correction_matrix_4x4": M_final.tolist(),
        "crs_info": crs_utm,
        "description": "4x4 Homogeneous Transformation Matrix. Multiply this with the original point cloud [x, y, z, 1]^T to get the optimized geographic coordinates (UTM).",
        "optimized_error_meters": float(best_error)
    }
    with open(matrix_output_path, 'w') as f:
        json.dump(matrix_data, f, indent=4)

    # 保存误差日志
    with open(deviation_output_path, 'w') as f:
        f.write(f"Optimization Method: Dual Annealing (Simulated Annealing variant)\n")
        f.write(f"Final Mean Absolute Elevation Deviation: {best_error:.6f} meters\n")
        f.write(f"Optimized Deltas (tx, ty, tz, rx_deg, ry_deg, rz_deg): {best_params.tolist()}\n")

    print(f"校正矩阵已保存至: {matrix_output_path}")
    print(f"高程误差日志已保存至: {deviation_output_path}")

if __name__ == "__main__":
    # === 使用示例 ===

    # 地面点云路径
    MY_PLY_FILE = "/home/cx/Documents/codes/pointcloud_607_roscpp/dataset/result/Wild/01/ground_points/sparse_gpcd_1785483404_225544964.ply"
    # DEM高程数据路径
    MY_DEM_FILE = "/home/cx/Documents/codes/pointcloud_607_roscpp/dataset/GeoInfo/Wild/01/n18_e109_1arc_v3.tif"

    # 填入无人机的真实记录参数
    DRONE_GPS = {
        'lon': 109.557266235351562,    # 经度
        'lat': 18.264570236206055,      # 纬度
        'alt': 584.500000000000000,          # 飞行高度 (海拔米)
        'heading': -132.572021484375000        # 航向角 (度)
    }

    # 为了防止文件不存在报错，这里加了一个存在性检查
    if os.path.exists(MY_PLY_FILE) and os.path.exists(MY_DEM_FILE):
        process_point_cloud_registration(MY_PLY_FILE, MY_DEM_FILE, DRONE_GPS)
    else:
        print("请提供实际的 .ply 和 .tif 文件路径后运行。")
