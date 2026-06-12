#!/home/cx/anaconda3/envs/pointcloud_607_ros2/bin/python
# -*- coding: utf-8 -*-
import numpy as np
import rclpy
from rclpy.node import Node
from pc_msgs.msg import O3DPointCloud, ClusteredPointCloud

# ==========================================================
# 动态加载并行 DBSCAN 模块 (带 Fallback 机制)
# ==========================================================
try:
    from dbscan import DBSCAN as ParallelDBSCAN

    USE_PYPI_DBSCAN = True
except ImportError:
    from sklearn.cluster import DBSCAN as SklearnDBSCAN

    USE_PYPI_DBSCAN = False


# ==========================================================

class ParallelDbscanNode(Node):
    def __init__(self):
        super().__init__('parallel_dbscan_node')

        # 1. 声明并读取参数
        self.declare_parameter('subscribe_topic', '/gs/non_ground_pointcloud')
        self.declare_parameter('publish_topic', '/clustering/clustered_pointcloud')
        self.declare_parameter('r_weight', 3.0)
        self.declare_parameter('theta_weight', 650.0)
        self.declare_parameter('phi_weight', 600.0)
        self.declare_parameter('cluster_eps', 18.0)
        self.declare_parameter('cluster_min_samples', 5)

        sub_topic = self.get_parameter('subscribe_topic').value
        pub_topic = self.get_parameter('publish_topic').value

        self.r_weight = self.get_parameter('r_weight').value
        self.theta_weight = self.get_parameter('theta_weight').value
        self.phi_weight = self.get_parameter('phi_weight').value
        self.cluster_eps = self.get_parameter('cluster_eps').value
        self.cluster_min_samples = self.get_parameter('cluster_min_samples').value

        # 2. 创建订阅者与发布者
        self.subscription = self.create_subscription(
            O3DPointCloud,
            sub_topic,
            self.pointcloud_callback,
            10
        )
        self.publisher = self.create_publisher(ClusteredPointCloud, pub_topic, 10)

        # 打印加速库状态
        if USE_PYPI_DBSCAN:
            self.get_logger().info("[*] Load 'dbscan' successfully (SIGMOD'20 Parallel DBSCAN)")
        else:
            self.get_logger().warn("[?] Cannot found 'dbscan', back to use 'scikit-learn' (n_jobs=-1)")
            import sys
            print(sys.executable)
            print(np.__version__)

        self.get_logger().info(f"[*] Clustering node has been bring up. Listen on: {sub_topic}")

    def pointcloud_callback(self, msg):
        # 1. 检查输入
        if not msg.points:
            self.get_logger().warn("[?] Received no points message. Scape clustering.")
            return

        # 2. 恢复 numpy 数组形式进行数学计算
        points_array = np.array(msg.points, dtype=np.float32).reshape(-1, 3)
        points_array_f64 = points_array.astype(np.float64)  # 转换以保证高精度计算

        # 3. 特征工程：转换为极坐标并加权
        r = np.linalg.norm(points_array_f64, axis=1)
        theta = np.arctan2(points_array_f64[:, 1], points_array_f64[:, 0])
        r_safe = np.clip(r, a_min=1e-6, a_max=None)
        phi = np.arcsin(points_array_f64[:, 2] / r_safe)

        polar_features = np.column_stack((
            r * self.r_weight,
            theta * self.theta_weight,
            phi * self.phi_weight
        ))

        # 4. 执行 DBSCAN 聚类
        try:
            if USE_PYPI_DBSCAN:
                labels, _ = ParallelDBSCAN(polar_features, eps=self.cluster_eps, min_samples=self.cluster_min_samples)
            else:
                clustering = SklearnDBSCAN(
                    eps=self.cluster_eps,
                    min_samples=self.cluster_min_samples,
                    n_jobs=-1
                ).fit(polar_features)
                labels = clustering.labels_
        except Exception as e:
            self.get_logger().error(f"[!] Failed to clustering: {e}")
            return

        # 获取最大类标签 (注意：如果没有有效聚类，只有噪声-1，则 max_label 将为 -1)
        max_label = int(np.max(labels))

        # 5. 构建并发布带有聚类标签的自定义消息
        out_msg = ClusteredPointCloud()
        # 完全继承原始的时间戳，保证下游按时间戳同步
        out_msg.header = msg.header

        # 直接使用输入消息中已展平的点数据，省去再次转换的开销
        out_msg.points = msg.points
        # 将聚类标签数组转为标准的 1D list 传给 ROS 2
        out_msg.labels = labels.astype(np.int32).tolist()
        out_msg.max_label = max_label

        self.publisher.publish(out_msg)

        num_clusters = max_label + 1 if max_label >= 0 else 0
        self.get_logger().info(f"[*] Clustering finished: Input points {len(points_array)}, Recover {num_clusters} clusters.")


def main(args=None):
    rclpy.init(args=args)
    node = ParallelDbscanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[*] Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()