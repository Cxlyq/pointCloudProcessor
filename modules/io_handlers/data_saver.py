"""
data_saver 多类型数据存储模块

根据yaml配置项，保存中间与最终产出数据结果

Classes:
    DataSaver: 数据储存类

Author: cx
Date: May 29, 2026,
Version: 0.0.0
"""
import os
import time
from pathlib import Path
import open3d as o3d
from core.factory import BaseProcessor, AlgorithmFactory
from core.context import FramePayload


@AlgorithmFactory.register(category="io_handlers", name="data_saver")
class DataSaver(BaseProcessor):
    def __init__(self, config: dict):
        super().__init__(config)
        # 根输出目录
        self.output_dir = Path(self.config.get('output_dir', './output'))

        # 核心：保存目标配置列表
        # 格式示例: [{'attr': 'non_ground_pcd', 'suffix': 'ngpcd', 'type': 'pcd'}, ...]
        self.save_targets = self.config.get('save_targets', [])

        os.makedirs(self.output_dir, exist_ok=True)

    def process(self, payload: FramePayload) -> None:
        start_time = time.time()
        saved_files = []

        # frame_id 在总控初始化时，通常就是去掉了 .ply 的 base_name
        base_name = payload.frame_id

        for target in self.save_targets:
            attr_name = target.get('attr')  # 对应 payload 中的属性名，如 'mesh_result'
            suffix = target.get('suffix')  # 文件名后缀，如 'hulls'
            data_type = target.get('type')  # 数据类型：'pcd' 或 'mesh'

            # 动态从托盘中获取对应的数据对象
            data_obj = getattr(payload, attr_name, None)

            if data_obj is None:
                continue

            # 安全检查：跳过空数据
            if data_type == 'pcd' and len(data_obj.points) == 0:
                continue
            if data_type == 'mesh' and len(data_obj.vertices) == 0:
                continue

            # 拼接最终保存的文件名和路径
            file_name = f"{base_name}_{suffix}.ply"
            save_path = Path(os.path.join(self.output_dir, file_name))

            # 根据类型调用对应的 Open3D 写入函数
            try:
                if data_type == 'pcd':
                    o3d.io.write_point_cloud(save_path, data_obj)
                    saved_files.append(file_name)
                elif data_type == 'mesh':
                    o3d.io.write_triangle_mesh(save_path, data_obj)
                    saved_files.append(file_name)
            except Exception as e:
                payload.metrics[f'save_error_{suffix}'] = str(e)

        payload.metrics['saver_status'] = 'success'
        payload.metrics['saved_files'] = saved_files
        payload.metrics['saver_time'] = time.time() - start_time