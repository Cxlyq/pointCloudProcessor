import time
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
from .gps_utils import load_ply_with_power, compute_relative_gps_transform


def register_scheme1_power_icp(
    ply_tgt_path,
    gps_tgt,
    ply_src_path,
    gps_src,
    power_percentile=95.0,
    max_correspondence_distance=50.0
):
    """
    方案一：GPS 粗初值 + 高 Power 强特征过滤 (前 5% 高功率点) + 适配射线跨度的 Point-to-Point ICP 精配准

    参数:
        ply_tgt_path: 第一帧/目标帧 PLY 路径
        gps_tgt: 第一帧 GPS 属性字典
        ply_src_path: 第二帧/源帧 PLY 路径
        gps_src: 第二帧 GPS 属性字典
        power_percentile: 提取强反射点的功率百分位数 (95.0 表示提取前 5% 最高功率点)
        max_correspondence_distance: 近邻搜索半径 (米)，设置为 50.0m 适应 1.6° 远端射线间距

    返回:
        results: 包含配准前后点云及配准变换矩阵的字典
    """
    start_time = time.time()

    # 1. 加载点云与属性
    pts_tgt, powers_tgt, pcd_tgt = load_ply_with_power(ply_tgt_path)
    pts_src, powers_src, pcd_src = load_ply_with_power(ply_src_path)

    # 2. 计算 GPS 粗估计相对变换矩阵 T_init
    T_init = compute_relative_gps_transform(gps_src, gps_tgt)

    # 3. 仅依 GPS 矩阵变换第二帧点云，并构建未配准合并点云
    pcd_src_gps = o3d.geometry.PointCloud(pcd_src)
    pcd_src_gps.transform(T_init)

    pts_src_gps = np.asarray(pcd_src_gps.points)
    unregistered_merged_points = np.vstack((pts_tgt, pts_src_gps))

    if powers_tgt is not None and powers_src is not None:
        unregistered_merged_powers = np.concatenate((powers_tgt, powers_src))
    else:
        unregistered_merged_powers = None

    # 4. 根据 Power 属性筛选强特征点
    if powers_tgt is not None and powers_src is not None:
        thresh_tgt = np.percentile(powers_tgt, power_percentile)
        thresh_src = np.percentile(powers_src, power_percentile)

        mask_tgt = powers_tgt >= thresh_tgt
        mask_src = powers_src >= thresh_src

        pcd_tgt_feat = o3d.geometry.PointCloud()
        pcd_tgt_feat.points = o3d.utility.Vector3dVector(pts_tgt[mask_tgt])

        pcd_src_feat = o3d.geometry.PointCloud()
        pcd_src_feat.points = o3d.utility.Vector3dVector(pts_src[mask_src])
    else:
        pcd_tgt_feat = pcd_tgt
        pcd_src_feat = pcd_src

    # 5. 执行 Point-to-Point ICP 精配准
    estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)

    reg_result = o3d.pipelines.registration.registration_icp(
        source=pcd_src_feat,
        target=pcd_tgt_feat,
        max_correspondence_distance=max_correspondence_distance,
        init=T_init,
        estimation_method=estimation_method,
        criteria=criteria
    )

    T_cand = reg_result.transformation
    fitness_cand = reg_result.fitness
    rmse_cand = reg_result.inlier_rmse

    # 检查配准姿态相对于 GPS 初值的位移，防止极端退化
    delta_T_step = T_cand @ np.linalg.inv(T_init)
    delta_t_norm = np.linalg.norm(delta_T_step[:3, 3])

    if delta_t_norm <= 30.0 and fitness_cand > 0.01:
        T_final = T_cand
        final_fitness = fitness_cand
        final_rmse = rmse_cand
    else:
        T_final = T_init
        final_fitness = fitness_cand
        final_rmse = rmse_cand

    # 6. 将最终精配准矩阵作用于全量第二帧点云
    pcd_src_registered = o3d.geometry.PointCloud(pcd_src)
    pcd_src_registered.transform(T_final)

    pts_src_reg = np.asarray(pcd_src_registered.points)
    registered_merged_points = np.vstack((pts_tgt, pts_src_reg))

    if powers_tgt is not None and powers_src is not None:
        registered_merged_powers = np.concatenate((powers_tgt, powers_src))
    else:
        registered_merged_powers = None

    # 7. 提取旋转与平移参数及相对修正量
    R_final = T_final[:3, :3]
    t_final = T_final[:3, 3]
    euler_deg = Rotation.from_matrix(R_final).as_euler('xyz', degrees=True)

    delta_T_gps = T_final @ np.linalg.inv(T_init)
    delta_t_gps = delta_T_gps[:3, 3]
    delta_r_gps = Rotation.from_matrix(delta_T_gps[:3, :3]).as_euler('xyz', degrees=True)

    elapsed_ms = (time.time() - start_time) * 1000.0

    return {
        'unregistered_points': pts_src_gps,
        'unregistered_merged_points': unregistered_merged_points,
        'unregistered_merged_powers': unregistered_merged_powers,
        'registered_points': pts_src_reg,
        'registered_merged_points': registered_merged_points,
        'registered_merged_powers': registered_merged_powers,
        'T_init': T_init,
        'T_final': T_final,
        'rotation_matrix': R_final,
        'translation_vector': t_final,
        'euler_angles_deg': euler_deg,
        'delta_translation_from_gps': delta_t_gps,
        'delta_euler_from_gps': delta_r_gps,
        'fitness': final_fitness,
        'inlier_rmse': final_rmse,
        'time_ms': elapsed_ms
    }
