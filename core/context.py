"""
数据托盘

点云处理中的每个步骤所产生的中间过程数据均会保存在该类中

Classes:
    FramePayload: 数据托盘类，存放点云处理中各个中间步骤产生的数据

Author: cx
Date: May 29, 2026
Version: 0.0.0
"""
from os import PathLike

import open3d as o3d
from pathlib import Path


class FramePayload:
    """
    数据上下文（托盘）：在流水线各个节点间流转，仅存储数据和中间产物，无逻辑。
    """

    def __init__(self, frame_id: str, file_path: Path):
        self.frame_id = frame_id # 单帧点云编号
        self.file_path = file_path # 单帧点云路径
        self.raw_pcd = o3d.geometry.PointCloud() # 原始点云

        # 中间处理产物
        self.ground_pcd = o3d.geometry.PointCloud() # 地面点云
        self.non_ground_pcd = o3d.geometry.PointCloud() # 去除地面剩余点云
        self.full_pcd = o3d.geometry.PointCloud() # 聚类后的总点云
        self.full_labels = None # 类别标签
        self.mesh_result = o3d.geometry.TriangleMesh() # 白模

        # 用于记录各节点的耗时、产出等指标
        self.metrics = {}