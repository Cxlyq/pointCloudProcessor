"""
streaming viewer流式可视化窗口

通过流式传输读取不断到来的3D Mesh模型，根据视角配置渲染到窗口
Classes:
    StreamingViewer: 流式传输可视化类

Author: cx
Date: May 29, 2026,
Version: 0.0.0
"""
import time
import numpy as np
import open3d as o3d
from core.factory import BaseProcessor, AlgorithmFactory
from core.context import FramePayload


@AlgorithmFactory.register(category="visualization", name="streaming_viewer")
class StreamingViewer(BaseProcessor):
    def __init__(self, config: dict):
        super().__init__(config)

        # 窗口配置列表
        self.windows_config = self.config.get('windows', [])

        # 存储实例状态的字典
        self.visualizers = {}  # { window_name: o3d.visualization.Visualizer }
        self.is_first_frame = {}  # { window_name: bool }
        self.target_attrs = {}  # { window_name: str (例如 'mesh_result') }

        # 初始化所有配置的窗口
        self._initialize_windows()

    def _initialize_windows(self):
        """根据配置，动态创建并初始化多个可视化窗口"""
        for i, win_cfg in enumerate(self.windows_config):
            name = win_cfg.get('name', f"Window_{i}")
            width = win_cfg.get('width', 1280)
            height = win_cfg.get('height', 720)
            bg_color = win_cfg.get('bg_color', [0.1, 0.1, 0.1])

            # 要渲染 payload 中的哪个属性（默认渲染白模）
            self.target_attrs[name] = win_cfg.get('target_attr', 'mesh_result')

            # 创建 Open3D 窗口
            vis = o3d.visualization.Visualizer()
            vis.create_window(window_name=name, width=width, height=height)

            # 设置渲染选项
            opt = vis.get_render_option()
            opt.background_color = np.asarray(bg_color)
            opt.point_size = win_cfg.get('point_size', 2.0)

            self.visualizers[name] = vis
            self.is_first_frame[name] = True

    def process(self, payload: FramePayload) -> None:
        """流式接收数据托盘，并更新所有窗口"""
        start_time = time.time()

        for win_name, vis in self.visualizers.items():
            # 1. 从托盘中动态获取该窗口需要渲染的数据
            attr_name = self.target_attrs[win_name]
            geometry = getattr(payload, attr_name, None)

            # 安全检查：如果数据不存在或为空，跳过该窗口的本次渲染
            if geometry is None or geometry.is_empty():
                continue

            # 如果是 Mesh，计算法线以保证光照正常
            if isinstance(geometry, o3d.geometry.TriangleMesh):
                geometry.compute_vertex_normals()

            # 2. 更新几何体
            vis.clear_geometries()
            vis.add_geometry(geometry, reset_bounding_box=self.is_first_frame[win_name])

            # 3. 设置视角 (仅在第一帧或强制要求时设置)
            # 在连续流式传输中，通常只在第一帧设置初始视角，之后允许用户用鼠标自由旋转
            if self.is_first_frame[win_name]:
                self._apply_camera_settings(win_name, vis)
                self.is_first_frame[win_name] = False

            # 4. 触发 Open3D 非阻塞渲染事件循环
            vis.poll_events()
            vis.update_renderer()

        payload.metrics['render_status'] = 'success'
        payload.metrics['render_time'] = time.time() - start_time

    def _apply_camera_settings(self, win_name: str, vis: o3d.visualization.Visualizer):
        """应用 YAML 中配置的相机视角"""
        # 查找当前窗口的配置
        win_cfg = next((cfg for cfg in self.windows_config if cfg.get('name') == win_name), None)
        if not win_cfg or 'camera' not in win_cfg:
            return

        cam_cfg = win_cfg['camera']
        view_ctl = vis.get_view_control()

        if 'front' in cam_cfg: view_ctl.set_front(cam_cfg['front'])
        if 'lookat' in cam_cfg: view_ctl.set_lookat(cam_cfg['lookat'])
        if 'up' in cam_cfg: view_ctl.set_up(cam_cfg['up'])
        if 'zoom' in cam_cfg: view_ctl.set_zoom(cam_cfg['zoom'])

    def keep_alive(self):
        """
        阻塞主线程，保持窗口打开。
        通常在处理完所有数据帧后由总控制器调用，否则程序退出窗口会瞬间关闭。
        """
        print("所有帧流式渲染完毕。请手动关闭所有 3D 窗口以退出程序...")
        while True:
            windows_open = False
            for vis in self.visualizers.values():
                if vis.poll_events():
                    vis.update_renderer()
                    windows_open = True

            # 如果所有窗口都被用户手动关闭，则跳出死循环
            if not windows_open:
                break
            time.sleep(0.01)  # 降低 CPU 占用

        # 销毁所有窗口
        for vis in self.visualizers.values():
            vis.destroy_window()