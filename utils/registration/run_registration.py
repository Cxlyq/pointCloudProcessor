import os
import json
import argparse
import numpy as np

from .gps_utils import save_ply_with_power
from .scheme1_power_icp import register_scheme1_power_icp
from .scheme2_ndt import register_scheme2_ndt


def run_pair_registration(dataset_dir="dataset/Wild/01", output_dir=None):
    """
    运行连续两帧点云的方案一与方案二配准测试
    """
    json_path = os.path.join(dataset_dir, "extend_data.json")
    if not os.path.exists(json_path):
        raise FileNotFoundError(f"未找到元数据文件: {json_path}")

    with open(json_path, 'r', encoding='utf-8') as f:
        meta_data = json.load(f)

    keys = list(meta_data.keys())
    if len(keys) < 2:
        raise ValueError("extend_data.json 中至少需要2帧点云数据记录！")

    frame1_key = keys[0]
    frame2_key = keys[1]

    info1 = meta_data[frame1_key]
    info2 = meta_data[frame2_key]

    ply1_path = os.path.join(dataset_dir, info1['ply_filename'])
    ply2_path = os.path.join(dataset_dir, info2['ply_filename'])

    gps1 = info1['gps_info']
    gps2 = info2['gps_info']

    print("=" * 70)
    print(f"对比数据帧 1 (Target): {info1['ply_filename']}")
    print(f"  GPS: Lat={gps1['Latitude']:.6f}, Lon={gps1['Longitude']:.6f}, Alt={gps1['Altitude']:.2f}, Course={gps1['Course']:.2f}°")
    print(f"对比数据帧 2 (Source): {info2['ply_filename']}")
    print(f"  GPS: Lat={gps2['Latitude']:.6f}, Lon={gps2['Longitude']:.6f}, Alt={gps2['Altitude']:.2f}, Course={gps2['Course']:.2f}°")
    print("=" * 70)

    if output_dir is None:
        output_dir = os.path.join(dataset_dir, "output_registered")
    os.makedirs(output_dir, exist_ok=True)

    # ---------------- 运行方案一 (Power ICP) ----------------
    print("\n>>> [方案一] 运行 GPS 初值 + 高 Power 强特征提取 + 多阶段粗精 ICP 配准...")
    res1 = register_scheme1_power_icp(
        ply_tgt_path=ply1_path,
        gps_tgt=gps1,
        ply_src_path=ply2_path,
        gps_src=gps2,
        power_percentile=95.0,
        max_correspondence_distance=50.0
    )

    print(f"└─ 方案一耗时: {res1['time_ms']:.2f} ms")
    print(f"└─ 匹配 Fitness: {res1['fitness']:.4f}, Inlier RMSE: {res1['inlier_rmse']:.4f} m")
    print(f"└─ GPS 变换平移向量 t: [{res1['T_init'][0,3]:.4f}, {res1['T_init'][1,3]:.4f}, {res1['T_init'][2,3]:.4f}] 米")
    print(f"└─ 精配准平移向量 t:   [{res1['translation_vector'][0]:.4f}, {res1['translation_vector'][1]:.4f}, {res1['translation_vector'][2]:.4f}] 米")
    print(f"└─ 精配准对 GPS 的修正量 Δt (x, y, z): [{res1['delta_translation_from_gps'][0]:.4f}, {res1['delta_translation_from_gps'][1]:.4f}, {res1['delta_translation_from_gps'][2]:.4f}] 米")
    print(f"└─ 精配准对 GPS 的修正角 ΔYaw: {res1['delta_euler_from_gps'][2]:.4f}°")
    print(f"└─ 变换矩阵 T_final (Frame 2 -> Frame 1):\n{np.array2string(res1['T_final'], precision=4, suppress_small=True)}")

    # ---------------- 运行方案二 (GICP/NDT) ----------------
    print("\n>>> [方案二] 运行 GPS 初值 + 大体素 3D GICP/NDT 配准...")
    res2 = register_scheme2_ndt(
        ply_tgt_path=ply1_path,
        gps_tgt=gps1,
        ply_src_path=ply2_path,
        gps_src=gps2,
        voxel_size=40.0,
        max_iteration=40
    )

    print(f"└─ 方案二耗时: {res2['time_ms']:.2f} ms")
    print(f"└─ 匹配 Fitness: {res2['fitness']:.4f}, Inlier RMSE: {res2['inlier_rmse']:.4f} m")
    print(f"└─ GPS 变换平移向量 t: [{res2['T_init'][0,3]:.4f}, {res2['T_init'][1,3]:.4f}, {res2['T_init'][2,3]:.4f}] 米")
    print(f"└─ 精配准平移向量 t:   [{res2['translation_vector'][0]:.4f}, {res2['translation_vector'][1]:.4f}, {res2['translation_vector'][2]:.4f}] 米")
    print(f"└─ 精配准对 GPS 的修正量 Δt (x, y, z): [{res2['delta_translation_from_gps'][0]:.4f}, {res2['delta_translation_from_gps'][1]:.4f}, {res2['delta_translation_from_gps'][2]:.4f}] 米")
    print(f"└─ 精配准对 GPS 的修正角 ΔYaw: {res2['delta_euler_from_gps'][2]:.4f}°")
    print(f"└─ 变换矩阵 T_final (Frame 2 -> Frame 1):\n{np.array2string(res2['T_final'], precision=4, suppress_small=True)}")

    # ---------------- 保存合并点云文件 ----------------
    unreg_path = os.path.join(output_dir, "unregistered_merged_gps.ply")
    save_ply_with_power(unreg_path, res1['unregistered_merged_points'], res1['unregistered_merged_powers'])

    scheme1_path = os.path.join(output_dir, "registered_merged_scheme1.ply")
    save_ply_with_power(scheme1_path, res1['registered_merged_points'], res1['registered_merged_powers'])

    scheme2_path = os.path.join(output_dir, "registered_merged_scheme2.ply")
    save_ply_with_power(scheme2_path, res2['registered_merged_points'], res2['registered_merged_powers'])

    print("\n" + "=" * 70)
    print(f"结果点云已成功保存至目录: {output_dir}")
    print(f"  ├─ 仅 GPS 未配准合并点云: {os.path.basename(unreg_path)}")
    print(f"  ├─ 方案一配准合并点云:     {os.path.basename(scheme1_path)}")
    print(f"  └─ 方案二配准合并点云:     {os.path.basename(scheme2_path)}")
    print("=" * 70)

    return res1, res2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="连续两帧 4D 毫米波雷达点云配准 (方案一 vs 方案二)")
    parser.add_argument("--dataset_dir", type=str, default="dataset/Wild/01", help="包含 ply 和 extend_data.json 的数据集路径")
    parser.add_argument("--output_dir", type=str, default=None, help="输出点云保存目录")
    args = parser.parse_args()

    run_pair_registration(args.dataset_dir, args.output_dir)
