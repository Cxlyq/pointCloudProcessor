"""
RANSAC地面分割算法模块

使用RANSAC处理点云，将点云地面分离出来

Classes:
    RansacSegmenter: RANSAC算法类

Author: cx
Date: May 29, 2026,
Version: 0.0.0
"""
import time
import open3d as o3d
from core.factory import BaseProcessor, AlgorithmFactory
from core.context import FramePayload


@AlgorithmFactory.register(category="ground_segmentation", name="ransac")
class RansacSegmenter(BaseProcessor):
    def __init__(self, config: dict):
        super().__init__(config)
        # 从配置中读取参数，如果没配则使用原代码的默认值
        self.distance_threshold = self.config.get('distance_threshold', 13.0)
        self.ransac_n = self.config.get('ransac_n', 3)
        self.num_iterations = self.config.get('num_iterations', 100)

    def process(self, payload: FramePayload) -> None:
        start_time = time.time()

        # 1. 安全检查：如果点云为空，直接跳过
        if len(payload.raw_pcd.points) == 0:
            payload.metrics['ransac_status'] = 'empty_input'
            payload.metrics['ransac_time'] = 0.0
            return

        try:
            # 2. 执行 RANSAC 算法
            _, inliers = payload.raw_pcd.segment_plane(
                distance_threshold=self.distance_threshold,
                ransac_n=self.ransac_n,
                num_iterations=self.num_iterations
            )

            # 3. 将结果分离并存入托盘 (Payload)
            payload.non_ground_pcd = payload.raw_pcd.select_by_index(inliers, invert=True)
            payload.ground_pcd = payload.raw_pcd.select_by_index(inliers)
            payload.metrics['ransac_status'] = 'success'

        except Exception as e:
            # 4. 异常降级策略：如果点云过少或无法拟合平面
            payload.non_ground_pcd = payload.raw_pcd
            payload.ground_pcd = o3d.geometry.PointCloud()
            payload.metrics['ransac_status'] = f'failed: {str(e)}'

        payload.metrics['ransac_status'] = 'success'
        payload.metrics['ransac_time'] = time.time() - start_time