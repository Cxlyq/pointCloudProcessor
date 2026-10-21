import numpy as np
import open3d as o3d
from pyproj import Transformer
from scipy.spatial.transform import Rotation


def load_ply_with_power(ply_path):
    """
    读取带 power 属性的 PLY 点云文件。
    返回:
        points: (N, 3) float64 numpy 数组
        powers: (N,) float64 numpy 数组 (若无则为 None)
        pcd: open3d.geometry.PointCloud 对象
    """
    points = []
    powers = []
    header_ended = False

    with open(ply_path, 'r', encoding='utf-8') as f:
        for line in f:
            line_str = line.strip()
            if not header_ended:
                if line_str == 'end_header':
                    header_ended = True
                continue
            if not line_str:
                continue
            parts = line_str.split()
            if len(parts) >= 3:
                points.append([float(parts[0]), float(parts[1]), float(parts[2])])
                if len(parts) >= 4:
                    powers.append(float(parts[3]))

    points_np = np.array(points, dtype=np.float64)
    powers_np = np.array(powers, dtype=np.float64) if len(powers) == len(points) else None

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_np)

    if powers_np is not None:
        p_min, p_max = powers_np.min(), powers_np.max()
        norm_p = (powers_np - p_min) / (p_max - p_min + 1e-6)
        colors = np.zeros((len(points_np), 3))
        colors[:, 0] = norm_p
        pcd.colors = o3d.utility.Vector3dVector(colors)

    return points_np, powers_np, pcd


def save_ply_with_power(ply_path, points, powers=None):
    """
    保存包含 x, y, z, power 属性的 PLY 文件 (ASCII 格式)
    """
    num_points = len(points)
    with open(ply_path, 'w', encoding='utf-8') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write("comment Created by pointCloudProcessor registration module\n")
        f.write(f"element vertex {num_points}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property float power\n")
        f.write("end_header\n")

        for i in range(num_points):
            p = points[i]
            pw = powers[i] if powers is not None and i < len(powers) else 0.0
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {pw:.6f}\n")


def get_sensor_pose_matrix(lon, lat, alt, heading, ref_utm=None):
    """
    根据无人机/传感器 GPS 经纬度、高度和航向角，构建局部世界坐标系下的 4x4 位姿矩阵。
    heading: 正北为0度，顺时针为正 (航向角)
    ref_utm: (ref_x, ref_y) 参考原点，用于防止 UTM 坐标值过大导致浮点数失真
    """
    utm_zone = int((lon + 180) / 6) + 1
    crs_utm = f"+proj=utm +zone={utm_zone} +datum=WGS84 +units=m +no_defs"
    transformer = Transformer.from_crs("EPSG:4326", crs_utm, always_xy=True)

    x_utm, y_utm = transformer.transform(lon, lat)

    if ref_utm is not None:
        x_local = x_utm - ref_utm[0]
        y_local = y_utm - ref_utm[1]
    else:
        x_local = x_utm
        y_local = y_utm

    yaw_math_deg = 90.0 - heading
    r_z = Rotation.from_euler('z', yaw_math_deg, degrees=True).as_matrix()

    M = np.eye(4)
    M[:3, :3] = r_z
    M[:3, 3] = [x_local, y_local, alt]

    return M, (x_utm, y_utm)


def compute_relative_gps_transform(gps_src, gps_tgt):
    """
    计算从源帧 (Source/Frame 2) 到目标帧 (Target/Frame 1) 的 GPS 相对初始估计矩阵 T_init。
    使得 P_tgt_est = T_init @ P_src
    """
    lon_src, lat_src, alt_src, heading_src = gps_src['Longitude'], gps_src['Latitude'], gps_src['Altitude'], gps_src['Course']
    lon_tgt, lat_tgt, alt_tgt, heading_tgt = gps_tgt['Longitude'], gps_tgt['Latitude'], gps_tgt['Altitude'], gps_tgt['Course']

    # 两者统一使用目标帧的 UTM 作为参考原点
    M_tgt, ref_utm = get_sensor_pose_matrix(lon_tgt, lat_tgt, alt_tgt, heading_tgt, ref_utm=None)
    M_src, _ = get_sensor_pose_matrix(lon_src, lat_src, alt_src, heading_src, ref_utm=None)

    # 相对位姿变换矩阵 T_init = M_tgt^(-1) * M_src
    T_init = np.linalg.inv(M_tgt) @ M_src
    return T_init
