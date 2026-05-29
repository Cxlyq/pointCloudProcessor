"""
PDBSCAN；聚类算法模块

使用PDBSCAN-并行DBSCAN算法，依据极坐标数据聚类点云

Classes:
    PolarDBSCANClusterer: PDBSCAN算法类

Author: cx
Date: May 29, 2026
Version: 0.0.0
"""
import time
import numpy as np
import open3d as o3d
from core.factory import BaseProcessor, AlgorithmFactory
from core.context import FramePayload
from dbscan import DBSCAN as ParallelDBSCAN




@AlgorithmFactory.register(category="clustering", name="polar_pdbscan")
class PolarDBSCANClusterer(BaseProcessor):
    def __init__(self, config: dict):
        super().__init__(config)
        # 极坐标权重参数
        self.r_weight = self.config.get('r_weight', 3.0)
        self.theta_weight = self.config.get('theta_weight', 650.0)
        self.phi_weight = self.config.get('phi_weight', 600.0)

        # DBSCAN 参数
        self.eps = self.config.get('eps', 18.0)
        self.min_samples = self.config.get('min_samples', 5)

        # 是否将地面也作为一个独立的 Cluster 拼装回来
        self.merge_ground = self.config.get('merge_ground', True)

    def process(self, payload: FramePayload) -> None:
        start_time = time.time()

        if payload.non_ground_pcd is None or len(payload.non_ground_pcd.points) == 0:
            payload.metrics['clustering_status'] = 'empty_input'
            payload.metrics['clustering_time'] = 0.0
            return

        points_array = np.asarray(payload.non_ground_pcd.points, dtype=np.float64)

        # 1. 转换为极坐标
        r = np.linalg.norm(points_array, axis=1)
        theta = np.arctan2(points_array[:, 1], points_array[:, 0])
        r_safe = np.clip(r, a_min=1e-6, a_max=None)
        phi = np.arcsin(points_array[:, 2] / r_safe)

        polar_features = np.column_stack((
            r * self.r_weight,
            theta * self.theta_weight,
            phi * self.phi_weight
        ))

        # 2. 执行聚类
        labels, _ = ParallelDBSCAN(polar_features, eps=self.eps, min_samples=self.min_samples)


        # 3. 合并地面点（原代码逻辑保留：赋予单独的 Label）
        if self.merge_ground and payload.ground_pcd and len(payload.ground_pcd.points) > 0:
            max_label = labels.max()
            ground_label = max_label + 1

            # 合并点云数据
            combined_points = np.vstack((np.asarray(payload.non_ground_pcd.points),
                                         np.asarray(payload.ground_pcd.points)))
            full_pcd = o3d.geometry.PointCloud()
            full_pcd.points = o3d.utility.Vector3dVector(combined_points)

            # 合并标签数据
            ground_labels_arr = np.full((len(payload.ground_pcd.points),), ground_label)
            combined_labels = np.concatenate((labels, ground_labels_arr))

            # 存入 Payload 的合并产物区
            payload.full_pcd = full_pcd
            payload.full_labels = combined_labels
            payload.metrics['total_clusters'] = ground_label + 1
        else:
            payload.full_pcd = payload.non_ground_pcd
            payload.full_labels = labels
            payload.metrics['total_clusters'] = labels.max() + 1 if len(labels) > 0 else 0

        payload.metrics['clustering_status'] = 'success'
        payload.metrics['clustering_time'] = time.time() - start_time