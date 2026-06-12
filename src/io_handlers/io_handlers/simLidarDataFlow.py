import os
import glob
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from pc_msgs.msg import O3DPointCloud
from pathlib import Path


class IOHandlerNode(Node):
    def __init__(self):
        super().__init__('sim_lidar_data_flow_node')

        # 1. 声明并读取参数
        self.declare_parameter('dataset_dir', '')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('publish_topic', '/io/raw_pointcloud')
        self.declare_parameter('frame_id', 'map')

        self.dataset_dir = self.get_parameter('dataset_dir').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.publish_topic = self.get_parameter('publish_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        # 2. 搜索点云文件
        search_pattern = os.path.join(self.dataset_dir, "*.ply")
        self.ply_files = sorted(glob.glob(search_pattern))
        self.current_idx = 0

        if not self.ply_files:
            self.get_logger().error(f"[!] Cannot found any .ply file in {self.dataset_dir}. Please check the path in configuration.")
            return

        self.get_logger().info(f"[*] Found {len(self.ply_files)} .ply frames in {self.dataset_dir}")

        # 3.预加载所有点云数据到内存中
        self.frames_data = []
        self._preload_pointclouds()

        if not self.frames_data:
            self.get_logger().error("[!] Cannot found any valid point cloud file!")
            return

        self.get_logger().info(f"[*] Preload successfully！Loaded {len(self.frames_data)} frames valid data.")

        # 4. 创建发布者
        self.publisher_ = self.create_publisher(O3DPointCloud, self.publish_topic, 10)

        # 5. 创建定时器，模拟雷达实时数据流
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f"[*] Start publishing Lidar data as frequency {self.publish_rate} Hz...")

    def _preload_pointclouds(self):
        """
        在节点启动时，将所有 .ply 文件读取、解析、展平，并存入内存列表。
        这样在回调函数中只需要做极简的赋值操作。
        """
        for i, file_path in enumerate(self.ply_files):
            try:
                pcd = o3d.io.read_point_cloud(Path(file_path))

                if len(pcd.points) == 0:
                    self.get_logger().warn(f"[?] Scape empty ply file: {os.path.basename(file_path)}")
                    continue

                # 提取坐标并展平为 1D list (CPU 密集型操作，在此提前完成)
                points_array = np.asarray(pcd.points, dtype=np.float32)
                points_list = points_array.flatten().tolist()

                # 如果有颜色则提取
                colors_list = []
                if pcd.has_colors():
                    colors_array = np.asarray(pcd.colors, dtype=np.float32)
                    colors_list = colors_array.flatten().tolist()

                # 存入字典缓冲池
                self.frames_data.append({
                    'file_name': os.path.basename(file_path),
                    'points': points_list,
                    'colors': colors_list,
                    'num_points': len(pcd.points)
                })

                # 打印加载进度 (每 50 帧打印一次，避免日志刷屏)
                if (i + 1) % 50 == 0 or (i + 1) == len(self.ply_files):
                    self.get_logger().info(f"[*] 预加载进度: {i + 1}/{len(self.ply_files)} ...")

            except Exception as e:
                self.get_logger().error(f"[!] An error occurred while preloading {file_path}: {e}")

    def timer_callback(self):
        # 检查是否播放完毕
        if self.current_idx >= len(self.ply_files):
            self.get_logger().info("[*] All ply files have been published.")
            self.timer.cancel()  # 停止定时器
            return

        # 1. 直接从内存列表中 O(1) 获取已处理好的数据
        frame = self.frames_data[self.current_idx]

        # 2. 构建消息
        msg = O3DPointCloud()
        # 关键：时间戳必须在发布这一刻实时生成
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.points = frame['points']
        if frame['colors']:
            msg.colors = frame['colors']

        # 3. 发布并索引递增
        self.publisher_.publish(msg)
        self.get_logger().info(f"[*] {frame['file_name']} has been published | Point: {frame['num_points']}")

        self.current_idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = IOHandlerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[*] Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()