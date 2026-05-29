"""
convex模块

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


@AlgorithmFactory.register(category="reconstruction", name="convex_hull")
class ConvexHullBuilder(BaseProcessor):
    def __init__(self, config: dict):
        super().__init__(config)
        # 最小有效聚类点数，少于这个数目的碎块不生成白模（剔除悬浮飞蚊噪点）
        self.min_cluster_size = self.config.get('min_cluster_size', 15)

        # 白模的统一表面颜色 (RGB, 0-1 范围)
        self.paint_color = self.config.get('paint_color', [0.9, 0.9, 0.9])

    def process(self, payload: FramePayload) -> None:
        start_time = time.time()

        # 1. 前置依赖检查：确保聚类模块已经产出了合并后的点云和标签
        if getattr(payload, 'full_pcd', None) is None or getattr(payload, 'full_labels', None) is None:
            payload.metrics['mesh_status'] = 'missing_clustering_data'
            payload.metrics['mesh_time'] = 0.0
            return

        combined_mesh = o3d.geometry.TriangleMesh()
        valid_hull_count = 0

        # 获取当前帧中所有独一无二的聚类标签
        unique_labels = np.unique(payload.full_labels)

        for label in unique_labels:
            # 忽略 DBSCAN 标记的噪声点 (通常 label 为 -1)
            if label < 0:
                continue

            # 2. 提取当前簇的点云索引与数据
            cluster_indices = np.where(payload.full_labels == label)[0]

            # 忽略点数过少的碎块，节省算力并提升视觉纯净度
            if len(cluster_indices) < self.min_cluster_size:
                continue

            cluster_pcd = payload.full_pcd.select_by_index(cluster_indices)

            # 3. 核心重建逻辑与降级策略 (Convex Hull -> OBB)
            try:
                # 尝试计算 3D 凸包
                hull_mesh, _ = cluster_pcd.compute_convex_hull()
                hull_mesh.compute_vertex_normals()
                hull_mesh.paint_uniform_color(self.paint_color)
                combined_mesh += hull_mesh
                valid_hull_count += 1
            except Exception:
                # 降级策略：如果点云严格共面（常见于纯平的地面簇），3D 凸包算法会崩溃
                # 此时回退使用 OBB (有向包围盒) 生成替代 Mesh
                try:
                    obb = cluster_pcd.get_oriented_bounding_box()
                    obb_mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obb)
                    obb_mesh.compute_vertex_normals()
                    obb_mesh.paint_uniform_color(self.paint_color)
                    combined_mesh += obb_mesh
                    valid_hull_count += 1
                except Exception as e:
                    # 极端情况：如果连 OBB 都失败了（比如所有的点都在一条射线上），则直接抛弃该簇
                    pass

        # 4. 将最终产物装填回数据托盘
        payload.mesh_result = combined_mesh
        payload.metrics['mesh_status'] = 'success'
        payload.metrics['valid_hull_count'] = valid_hull_count
        payload.metrics['mesh_time'] = time.time() - start_time