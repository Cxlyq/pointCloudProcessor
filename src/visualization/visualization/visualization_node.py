#!/usr/bin/env python3
import time
import threading
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from pc_msgs.msg import O3DMesh


class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')

        self.declare_parameter('subscribe_topic', '/reconstruction/white_mesh')
        self.declare_parameter('camera_ids', ['cam_front'])

        sub_topic = self.get_parameter('subscribe_topic').value
        self.camera_ids = self.get_parameter('camera_ids').value

        # 用于存储多窗口的配置和实例
        self.visualizers = []

        # 1. 解析多机位配置并初始化窗口
        for cam_id in self.camera_ids:
            self.declare_parameter(f'{cam_id}.name', f'Render - {cam_id}')
            self.declare_parameter(f'{cam_id}.width', 1280)
            self.declare_parameter(f'{cam_id}.height', 720)
            self.declare_parameter(f'{cam_id}.background_color', [0.1, 0.1, 0.1])
            self.declare_parameter(f'{cam_id}.point_size', 2.0)
            self.declare_parameter(f'{cam_id}.front', [-0.0, -1.0, 0.35])
            self.declare_parameter(f'{cam_id}.lookat', [-17.5, 2213.0, -86.0])
            self.declare_parameter(f'{cam_id}.up', [-0.0, 0.35, 1.0])
            self.declare_parameter(f'{cam_id}.zoom', 0.7)

            cam_config = {
                'name': self.get_parameter(f'{cam_id}.name').value,
                'width': self.get_parameter(f'{cam_id}.width').value,
                'height': self.get_parameter(f'{cam_id}.height').value,
                'bg_color': self.get_parameter(f'{cam_id}.background_color').value,
                'pt_size': self.get_parameter(f'{cam_id}.point_size').value,
                'front': self.get_parameter(f'{cam_id}.front').value,
                'lookat': self.get_parameter(f'{cam_id}.lookat').value,
                'up': self.get_parameter(f'{cam_id}.up').value,
                'zoom': self.get_parameter(f'{cam_id}.zoom').value,
                'is_first_frame': True
            }

            # 实例化 Open3D Visualizer
            vis = o3d.visualization.Visualizer()
            vis.create_window(window_name=cam_config['name'],
                              width=cam_config['width'],
                              height=cam_config['height'])

            opt = vis.get_render_option()
            opt.background_color = np.asarray(cam_config['bg_color'])
            opt.point_size = cam_config['pt_size']

            self.visualizers.append({
                'config': cam_config,
                'vis': vis
            })

        # 2. 线程安全的数据交换区
        self.lock = threading.Lock()
        self.latest_mesh = None
        self.is_new_mesh_available = False

        # 3. 创建订阅者 (由后台 ROS 线程触发)
        self.subscription = self.create_subscription(
            O3DMesh,
            sub_topic,
            self.mesh_callback,
            10
        )
        self.get_logger().info(f"[*] Visualization started.  {len(self.visualizers)} windows has been bring up.")

    def mesh_callback(self, msg):
        """
        ROS 回调函数 (运行在子线程)。
        只负责反序列化数据，不进行任何界面渲染操作。
        """
        mesh = o3d.geometry.TriangleMesh()

        # 反序列化：还原顶点和面片
        vertices_array = np.array(msg.vertices, dtype=np.float32).reshape(-1, 3)
        mesh.vertices = o3d.utility.Vector3dVector(vertices_array)

        triangles_array = np.array(msg.triangles, dtype=np.int32).reshape(-1, 3)
        mesh.triangles = o3d.utility.Vector3iVector(triangles_array)

        if msg.vertex_normals:
            normals_array = np.array(msg.vertex_normals, dtype=np.float32).reshape(-1, 3)
            mesh.vertex_normals = o3d.utility.Vector3dVector(normals_array)

        if msg.vertex_colors:
            colors_array = np.array(msg.vertex_colors, dtype=np.float32).reshape(-1, 3)
            mesh.vertex_colors = o3d.utility.Vector3dVector(colors_array)

        # 加锁，将新 Mesh 放入交换区
        with self.lock:
            self.latest_mesh = mesh
            self.is_new_mesh_available = True

    def update_ui(self):
        """
        UI 刷新函数 (运行在主线程)。
        持续被循环调用，以维持窗口响应和画面更新。
        """
        # 尝试获取新数据
        mesh_to_render = None
        with self.lock:
            if self.is_new_mesh_available:
                mesh_to_render = self.latest_mesh
                self.is_new_mesh_available = False

        # 遍历所有窗口进行渲染
        for item in self.visualizers:
            vis = item['vis']
            cfg = item['config']

            # 如果有新数据，更新几何体
            if mesh_to_render is not None:
                vis.clear_geometries()
                vis.add_geometry(mesh_to_render, reset_bounding_box=cfg['is_first_frame'])

                # 如果是第一帧，设置相机视角
                if cfg['is_first_frame']:
                    view_ctl = vis.get_view_control()
                    view_ctl.set_front(cfg['front'])
                    view_ctl.set_lookat(cfg['lookat'])
                    view_ctl.set_up(cfg['up'])
                    view_ctl.set_zoom(cfg['zoom'])
                    cfg['is_first_frame'] = False

            # 维持窗口心跳（即使没有新数据，这一步也是防止窗口卡死的关键）
            vis.poll_events()
            vis.update_renderer()

    def cleanup(self):
        # 退出时销毁所有窗口
        for item in self.visualizers:
            item['vis'].destroy_window()


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()

    # 1. 将 ROS 2 的 spin 放进后台守护线程
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # 2. 将主线程留给 Open3D 的 UI 循环
    try:
        # 使用 rclpy.ok() 作为大循环条件
        while rclpy.ok():
            node.update_ui()
            # 极短的休眠以释放一点 CPU 资源，相当于 ~100Hz 刷新率
            time.sleep(0.01)
    except KeyboardInterrupt:
        node.get_logger().info("[*] Node stopped by user. Shutting down windows...")
    finally:
        node.cleanup()
        # 清理 ROS 节点
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()