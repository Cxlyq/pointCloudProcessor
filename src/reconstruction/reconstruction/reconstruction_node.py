#!/usr/bin/env python3
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
import message_filters

# 导入自定义消息
from pc_msgs.msg import O3DPointCloud, ClusteredPointCloud, O3DMesh


class ReconstructionNode(Node):
    def __init__(self):
        super().__init__('reconstruction_node')

        # 1. 声明并读取参数
        self.declare_parameter('sub_ground_topic', '/gs/ground_pointcloud')
        self.declare_parameter('sub_cluster_topic', '/clustering/clustered_pointcloud')
        self.declare_parameter('pub_mesh_topic', '/reconstruction/white_mesh')
        self.declare_parameter('min_cluster_size', 15)

        sub_g_topic = self.get_parameter('sub_ground_topic').value
        sub_c_topic = self.get_parameter('sub_cluster_topic').value
        pub_m_topic = self.get_parameter('pub_mesh_topic').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value

        # 2. 创建消息同步订阅器
        self.ground_sub = message_filters.Subscriber(self, O3DPointCloud, sub_g_topic)
        self.cluster_sub = message_filters.Subscriber(self, ClusteredPointCloud, sub_c_topic)

        # 使用 ExactTimeSynchronizer 进行精确时间戳匹配，队列大小设为 10
        self.ts = message_filters.ApproximateTimeSynchronizer([self.ground_sub, self.cluster_sub], 10, 0.001)
        self.ts.registerCallback(self.sync_callback)

        # 3. 创建发布者
        self.publisher = self.create_publisher(O3DMesh, pub_m_topic, 10)

        self.get_logger().info(f"[*] Reconstruction node has been bring up. Listen on: 1. {sub_g_topic}  2. {sub_c_topic}...")

    def sync_callback(self, ground_msg, cluster_msg):
        # 1. 解析地面点云
        ground_points = np.array(ground_msg.points, dtype=np.float32).reshape(-1, 3)

        # 2. 解析非地面聚类点云及其标签
        cluster_points = np.array(cluster_msg.points, dtype=np.float32).reshape(-1, 3)
        cluster_labels = np.array(cluster_msg.labels, dtype=np.int32)

        # 为了与你的原代码逻辑保持一致，我们将地面点云赋予一个新的标签 (max_label + 1)
        ground_label = cluster_msg.max_label + 1
        ground_labels = np.full(ground_points.shape[0], ground_label, dtype=np.int32)

        # 拼合点云与标签
        all_points = np.vstack((cluster_points, ground_points))
        combined_labels = np.concatenate((cluster_labels, ground_labels))

        # 构建完整的 Open3D 点云对象
        full_pcd = o3d.geometry.PointCloud()
        full_pcd.points = o3d.utility.Vector3dVector(all_points)

        total_max_label = ground_label

        # 3. 核心重建逻辑 (原代码迁移)
        combined_mesh = o3d.geometry.TriangleMesh()
        valid_hull_count = 0

        # 遍历所有标签（排除 -1 噪声点）
        for i in range(total_max_label + 1):
            cluster_indices = np.where(combined_labels == i)[0]

            # 忽略点数过少的碎块
            if len(cluster_indices) < self.min_cluster_size:
                continue

            cluster_pcd = full_pcd.select_by_index(cluster_indices)

            try:
                hull_mesh, _ = cluster_pcd.compute_convex_hull()
                hull_mesh.compute_vertex_normals()
                hull_mesh.paint_uniform_color([0.9, 0.9, 0.9])
                combined_mesh += hull_mesh
                valid_hull_count += 1
            except Exception:
                # 降级策略：如果平面计算 3D 凸包失败，使用有向包围盒 (OBB) 生成 Mesh
                try:
                    obb = cluster_pcd.get_oriented_bounding_box()
                    obb_mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obb)
                    obb_mesh.compute_vertex_normals()
                    obb_mesh.paint_uniform_color([0.9, 0.9, 0.9])
                    combined_mesh += obb_mesh
                    valid_hull_count += 1
                except Exception:
                    pass

        # 4. 检查是否生成了有效的 Mesh
        if not combined_mesh.has_vertices():
            self.get_logger().warn("[?] No any valid data generated. Scape publish.")
            return

        # 5. 序列化为自定义 ROS 2 消息 O3DMesh
        out_msg = O3DMesh()
        out_msg.header = ground_msg.header  # 继承原始时间戳

        # 提取 Open3D 属性并展平为 1D list
        out_msg.vertices = np.asarray(combined_mesh.vertices, dtype=np.float32).flatten().tolist()
        out_msg.triangles = np.asarray(combined_mesh.triangles, dtype=np.int32).flatten().tolist()

        if combined_mesh.has_vertex_normals():
            out_msg.vertex_normals = np.asarray(combined_mesh.vertex_normals, dtype=np.float32).flatten().tolist()

        if combined_mesh.has_vertex_colors():
            out_msg.vertex_colors = np.asarray(combined_mesh.vertex_colors, dtype=np.float32).flatten().tolist()

        # 发布
        self.publisher.publish(out_msg)
        self.get_logger().info(f"[*] Reconstruction successful: Extract {valid_hull_count} obb.")


def main(args=None):
    rclpy.init(args=args)
    node = ReconstructionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[*] Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()