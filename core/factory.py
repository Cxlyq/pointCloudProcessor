"""
算法工厂

点云处理中的每个步骤所需要的算法模块由该工厂进行实例化

Classes:
    BaseProcessor: 所有算法模块的抽象基类
    AlgorithmFactory: 全局算法注册与实例化工厂

Author: cx
Date: May 29, 2026
Version: 0.0.0
"""
from abc import ABC, abstractmethod
from typing import Dict, Type, Any
from .context import FramePayload


class BaseProcessor(ABC):
    """
    所有处理模块的抽象基类。
    强制所有子类必须实现 process 方法。
    """

    def __init__(self, config: dict):
        self.config = config

    @abstractmethod
    def process(self, payload: FramePayload) -> None:
        """
        处理数据并修改 payload 中的内容。
        """
        pass


class AlgorithmFactory:
    """
    全局算法注册与实例化工厂
    """
    # 存储注册的算法: {'ground_segmentation': {'ransac': RansacSegmenter, ...}, ...}
    _registry: Dict[str, Dict[str, Type[BaseProcessor]]] = {}

    @classmethod
    def register(cls, category: str, name: str):
        """
        类装饰器：用于在定义算法类时自动将其注册到工厂中。
        """
        def decorator(processor_class: Type[BaseProcessor]):
            if category not in cls._registry:
                cls._registry[category] = {}
            if name in cls._registry[category]:
                print(f"警告: 类别 '{category}' 中已存在名为 '{name}' 的算法，将被覆盖。")
            cls._registry[category][name] = processor_class
            return processor_class

        return decorator

    @classmethod
    def create(cls, category: str, name: str, config: dict) -> BaseProcessor:
        """
        根据配置动态实例化算法
        """
        if category not in cls._registry or name not in cls._registry[category]:
            raise ValueError(f"无法找到类别为 '{category}'，名称为 '{name}' 的注册算法。")

        processor_class = cls._registry[category][name]
        return processor_class(config)