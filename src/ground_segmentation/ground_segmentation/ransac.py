import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from pc_msgs.msg import O3DPointCloud


class RansacNode(Node):
    def __init__(self):
        super().__init__('ransac_node')

        # 1. 声明并读取参数
        self.declare_parameter('subscribe_topic', '/io/raw_pointcloud')
        self.declare_parameter('publish_ground_topic', '/gs/ground_pointcloud')
        self.declare_parameter('publish_non_ground_topic', '/gs/non_ground_pointcloud')
        self.declare_parameter('distance_threshold', 13.0)
        self.declare_parameter('ransac_n', 3)
        self.declare_parameter('num_iterations', 100)

        sub_topic = self.get_parameter('subscribe_topic').value
        pub_g_topic = self.get_parameter('publish_ground_topic').value
        pub_ng_topic = self.get_parameter('publish_non_ground_topic').value

        self.dist_thresh = self.get_parameter('distance_threshold').value
        self.ransac_n = self.get_parameter('ransac_n').value
        self.num_iters = self.get_parameter('num_iterations').value

        # 2. 创建订阅者与发布者
        self.subscription = self.create_subscription(
            O3DPointCloud,
            sub_topic,
            self.pointcloud_callback,
            10
        )

        self.ground_pub = self.create_publisher(O3DPointCloud, pub_g_topic, 10)
        self.non_ground_pub = self.create_publisher(O3DPointCloud, pub_ng_topic, 10)

        self.get_logger().info(f"[*] RANSAC node has been bring up. Listen on: {sub_topic}...")

    def pointcloud_callback(self, msg):
        # 1. 检查输入是否为空
        if not msg.points:
            self.get_logger().warn("[?] Scape empty point cloud message.")
            return

        # 2. 反序列化：ROS 2 消息转 Open3D 点云
        pcd = o3d.geometry.PointCloud()
        points_array = np.array(msg.points, dtype=np.float32).reshape(-1, 3)
        pcd.points = o3d.utility.Vector3dVector(points_array)

        # 3. RANSAC 算法逻辑
        try:
            _, inliers = pcd.segment_plane(
                distance_threshold=self.dist_thresh,
                ransac_n=self.ransac_n,
                num_iterations=self.num_iters
            )
            # 提取非地面点和地面点
            non_ground_pcd = pcd.select_by_index(inliers, invert=True)
            ground_pcd = pcd.select_by_index(inliers)
        except Exception as e:
            self.get_logger().warn(f"[?] RANSAC run failed: {e}. All points will be regarded as non-ground points.")
            non_ground_pcd = pcd
            ground_pcd = o3d.geometry.PointCloud()

        if len(non_ground_pcd.points) == 0:
            self.get_logger().warn("[?] Empty non-ground point cloud. Scape publish")
            return

        # 4. 序列化：Open3D 点云转 ROS 2 消息并发布
        # 发布地面点云
        if len(ground_pcd.points) > 0:
            ground_msg = O3DPointCloud()
            ground_msg.header = msg.header  # 关键：完全继承原始时间戳和 frame_id
            ground_array = np.asarray(ground_pcd.points, dtype=np.float32)
            ground_msg.points = ground_array.flatten().tolist()
            self.ground_pub.publish(ground_msg)

        # 发布非地面点云
        non_ground_msg = O3DPointCloud()
        non_ground_msg.header = msg.header  # 关键：完全继承原始时间戳和 frame_id
        non_ground_array = np.asarray(non_ground_pcd.points, dtype=np.float32)
        non_ground_msg.points = non_ground_array.flatten().tolist()
        self.non_ground_pub.publish(non_ground_msg)

        self.get_logger().info(f"[*] Segmentation successful with Ground points {len(ground_pcd.points)}, non ground points {len(non_ground_pcd.points)}")


def main(args=None):
    rclpy.init(args=args)
    node = RansacNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[*] Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()