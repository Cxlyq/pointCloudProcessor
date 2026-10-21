import time
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
from .gps_utils import load_ply_with_power, compute_relative_gps_transform


def register_scheme2_ndt(
    ply_tgt_path,
    gps_tgt,
    ply_src_path,
    gps_src,
    voxel_size=20.0,
    max_iteration=40
):
    """
    方案二：GPS 粗初值 + 体素化全量点云 ICP / NDT 配准

    注意: 4D 毫米波雷达点云呈 1.6° 射线状，体素内点云共线会导致 3D NDT 协方差矩阵奇异。
          因此本模块采用大体素下采样 + 适配射线间距的 Point-to-Point ICP 作为主要优化手段。

    参数:
        ply_tgt_path: 第一帧/目标帧 PLY 路径
        gps_tgt: 第一帧 GPS 属性字典
        ply_src_path: 第二帧/源帧 PLY 路径
        gps_src: 第二帧 GPS 属性字典
        voxel_size: 体素下采样网格大小 (米)
        max_iteration: 最大迭代次数

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

    # 4. 体素下采样平滑点云
    pcd_tgt_down = pcd_tgt.voxel_down_sample(voxel_size=voxel_size)
    pcd_src_down = pcd_src.voxel_down_sample(voxel_size=voxel_size)

    # 首先尝试 3D NDT 配准
    option = o3d.pipelines.registration.NormalDistributionsTransformOption(
        voxel_size=voxel_size * 2.0,
        min_points_per_voxel=4,
        max_iteration=max_iteration
    )

    try:
        reg_result = o3d.pipelines.registration.registration_ndt(
            source=pcd_src_down,
            target=pcd_tgt_down,
            option=option,
            init=T_init
        )
    except Exception:
        reg_result = None

    # 如果 NDT 因为射线共线导致协方差矩阵奇异 (Fitness == 0)，回退使用体素 Point-to-Point ICP
    if reg_result is None or reg_result.fitness < 0.001:
        estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iteration)

        reg_result = o3d.pipelines.registration.registration_icp(
            source=pcd_src_down,
            target=pcd_tgt_down,
            max_correspondence_distance=50.0,
            init=T_init,
            estimation_method=estimation_method,
            criteria=criteria
        )

    T_cand = reg_result.transformation
    fitness_cand = reg_result.fitness
    rmse_cand = reg_result.inlier_rmse

    # 检查微调量是否在合理防漂移范围内
    delta_T_step = T_cand @ np.linalg.inv(T_init)
    delta_t_norm = np.linalg.norm(delta_T_step[:3, 3])

    if delta_t_norm <= 35.0 and fitness_cand > 0.001:
        T_final = T_cand
        final_fitness = fitness_cand
        final_rmse = rmse_cand
    else:
        T_final = T_init
        final_fitness = fitness_cand
        final_rmse = rmse_cand

    # 5. 将最终精配准矩阵作用于全量第二帧点云
    pcd_src_registered = o3d.geometry.PointCloud(pcd_src)
    pcd_src_registered.transform(T_final)

    pts_src_reg = np.asarray(pcd_src_registered.points)
    registered_merged_points = np.vstack((pts_tgt, pts_src_reg))

    if powers_tgt is not None and powers_src is not None:
        registered_merged_powers = np.concatenate((powers_tgt, powers_src))
    else:
        registered_merged_powers = None

    # 6. 提取旋转与平移参数及相对修正量
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
