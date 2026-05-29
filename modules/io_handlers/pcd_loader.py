"""
pcd_loader 原始点云读取模块

根据总控传入的路径读取单帧点云数据

Classes:
    PCDLoader: 点云读取类

Author: cx
Date: May 29, 2026,
Version: 0.0.0
"""
import os
import time
import open3d as o3d
from core.factory import BaseProcessor, AlgorithmFactory
from core.context import FramePayload


@AlgorithmFactory.register(category="io_handlers", name="pcd_loader")
class PCDLoader(BaseProcessor):
    def __init__(self, config: dict):
        super().__init__(config)
        # 可以在 YAML 中配置是否在加载时直接进行体素降采样，如果不配则默认为 None
        self.voxel_downsample_size = self.config.get('voxel_downsample_size', None)

    def process(self, payload: FramePayload) -> None:
        start_time = time.time()

        # 1. 检查路径有效性
        if not hasattr(payload, 'file_path') or not os.path.exists(payload.file_path):
            payload.metrics['loader_status'] = 'file_not_found'
            payload.metrics['loader_time'] = 0.0
            return

        # 2. 读取点云
        pcd = o3d.io.read_point_cloud(payload.file_path)

        if len(pcd.points) == 0:
            payload.metrics['loader_status'] = 'empty_file'
            payload.raw_pcd = pcd
            payload.metrics['loader_time'] = time.time() - start_time
            return

        # 3. 可选的轻量预处理 (如初步降采样)
        if self.voxel_downsample_size is not None and self.voxel_downsample_size > 0:
            pcd = pcd.voxel_down_sample(self.voxel_downsample_size)

        # 4. 存入托盘
        payload.raw_pcd = pcd
        payload.metrics['loader_status'] = 'success'
        payload.metrics['loader_time'] = time.time() - start_time