"""
数据处理流水线

点云处理中的每个步骤均实例化在该流水线上进行处理

Classes:
    PipelineController: 流水线总控类

Author: cx
Date: May 29, 2026
Version: 0.0.0
"""
import yaml
from pathlib import Path
from typing import List
from .context import FramePayload
from .factory import AlgorithmFactory, BaseProcessor


class PipelineController:
    def __init__(self, config_path: str):
        # 1. 解析 YAML 配置文件
        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = yaml.safe_load(f)

        self.processors: List[BaseProcessor] = []
        self.viewer_instance = None  # 单独记录可视化模块的引用

        # 2. 根据配置，动态组装流水线 (此时，StreamingViewer 被实例化，空白窗口立即弹出！)
        self._build_pipeline()

    def _build_pipeline(self):
        print("[INFO] 开始组装数据处理流水线...")
        pipeline_config = self.config.get('pipeline', [])

        for step in pipeline_config:
            category = step.get('category')
            name = step.get('name')
            params = step.get('params', {})

            try:
                # 通过工厂拿取算法实例
                processor = AlgorithmFactory.create(category, name, params)
                self.processors.append(processor)
                print(f"[INFO] 成功加载模块: [{category}] -> {name}")

                # 如果是可视化模块，记录下来以便在程序末尾保持窗口开启
                if category == 'visualization':
                    self.viewer_instance = processor

            except ValueError as e:
                print(f"[ERROR] 模块加载失败: {e}")

        print("[INFO] 流水线组装完毕！\n" + "-" * 40)

    def process_frame(self, frame_id: str, file_path: str) -> dict:
        """处理单帧数据的核心逻辑"""
        # 1. 发放一个新的数据托盘
        payload = FramePayload(frame_id=frame_id, file_path=Path(file_path))

        # 2. 托盘依次经过流水线上的每一个工位
        for processor in self.processors:
            processor.process(payload)

        # 3. 返回该帧的各项指标统计
        return payload.metrics

    def shutdown(self):
        """流式处理结束后的收尾工作"""
        if self.viewer_instance:
            self.viewer_instance.keep_alive()