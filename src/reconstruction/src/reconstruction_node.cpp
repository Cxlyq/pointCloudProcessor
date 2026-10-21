#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <tuple>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
// 引入 message_filters 实现时间同步
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include "pc_msgs/msg/o3_d_point_cloud.hpp"
#include "pc_msgs/msg/clustered_point_cloud.hpp"
#include "pc_msgs/msg/o3_d_mesh.hpp"

#include "open3d/Open3D.h"
#include <Eigen/Core>

using std::placeholders::_1;
using std::placeholders::_2;

class ReconstructionNode : public rclcpp::Node {
public:
    // 定义同步策略：近似时间同步
    typedef message_filters::sync_policies::ApproximateTime<
        pc_msgs::msg::O3DPointCloud,
        pc_msgs::msg::ClusteredPointCloud> SyncPolicy;

    ReconstructionNode() : Node("reconstruction_node") {
        // 1. 声明并读取参数
        this->declare_parameter<std::string>("sub_ground_topic", "/gs/ground_pointcloud");
        this->declare_parameter<std::string>("sub_cluster_topic", "/clustering/clustered_pointcloud");
        this->declare_parameter<std::string>("pub_mesh_topic", "/reconstruction/white_mesh");
        this->declare_parameter<int>("min_cluster_size", 15);

        auto sub_g_topic = this->get_parameter("sub_ground_topic").as_string();
        auto sub_c_topic = this->get_parameter("sub_cluster_topic").as_string();
        auto pub_m_topic = this->get_parameter("pub_mesh_topic").as_string();
        min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();

        // 2. 创建消息同步订阅器
        ground_sub_.subscribe(this, sub_g_topic);
        cluster_sub_.subscribe(this, sub_c_topic);

        // 队列大小设为 10
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), ground_sub_, cluster_sub_);
        sync_->registerCallback(std::bind(&ReconstructionNode::sync_callback, this, _1, _2));

        // 3. 创建发布者
        publisher_ = this->create_publisher<pc_msgs::msg::O3DMesh>(pub_m_topic, 10);

        RCLCPP_INFO(this->get_logger(), "[*] Reconstruction node has been brought up. Listening on: 1. %s  2. %s...",
                    sub_g_topic.c_str(), sub_c_topic.c_str());
    }

private:
    void sync_callback(
        const pc_msgs::msg::O3DPointCloud::ConstSharedPtr& ground_msg,
        const pc_msgs::msg::ClusteredPointCloud::ConstSharedPtr& cluster_msg) const
    {
        size_t ground_points_count = ground_msg->points.size() / 3;
        size_t cluster_points_count = cluster_msg->points.size() / 3;
        size_t total_points = ground_points_count + cluster_points_count;

        // 1 & 2. 解析并拼合点云与标签
        auto full_pcd = std::make_shared<open3d::geometry::PointCloud>();
        full_pcd->points_.reserve(total_points);

        std::vector<int> combined_labels;
        combined_labels.reserve(total_points);

        // 载入聚类非地面点
        for (size_t i = 0; i < cluster_points_count; ++i) {
            full_pcd->points_.emplace_back(
                static_cast<double>(cluster_msg->points[i * 3 + 0]),
                static_cast<double>(cluster_msg->points[i * 3 + 1]),
                static_cast<double>(cluster_msg->points[i * 3 + 2])
            );
            combined_labels.push_back(cluster_msg->labels[i]);
        }

        // 载入地面点并赋予新标签
        int ground_label = cluster_msg->max_label + 1;
        for (size_t i = 0; i < ground_points_count; ++i) {
            full_pcd->points_.emplace_back(
                static_cast<double>(ground_msg->points[i * 3 + 0]),
                static_cast<double>(ground_msg->points[i * 3 + 1]),
                static_cast<double>(ground_msg->points[i * 3 + 2])
            );
            combined_labels.push_back(ground_label);
        }

        int total_max_label = ground_label;

        // 3. 核心重建逻辑
        auto combined_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
        int valid_hull_count = 0;

        // 遍历所有标签 (i = 0 到 total_max_label)
        for (int i = 0; i <= total_max_label; ++i) {
            std::vector<size_t> cluster_indices;
            for (size_t j = 0; j < combined_labels.size(); ++j) {
                if (combined_labels[j] == i) {
                    cluster_indices.push_back(j);
                }
            }

            // 忽略点数过少的碎块
            if (cluster_indices.size() < static_cast<size_t>(min_cluster_size_)) {
                continue;
            }

            auto cluster_pcd = full_pcd->SelectByIndex(cluster_indices);

            try {
                // 计算 3D 凸包，返回 tuple(mesh, indices)
                auto result = cluster_pcd->ComputeConvexHull();
                auto hull_mesh = std::get<0>(result);

                hull_mesh->ComputeVertexNormals();
                hull_mesh->PaintUniformColor(Eigen::Vector3d(0.9, 0.9, 0.9));

                // C++ 中的 Open3D 支持直接通过 += 重载符合网格拼接
                *combined_mesh += *hull_mesh;
                valid_hull_count++;
            } catch (const std::exception& e) {
                // 降级策略：凸包失败（如点共面），使用 OBB
                try {
                    auto obb = cluster_pcd->GetOrientedBoundingBox();
                    auto obb_mesh = open3d::geometry::TriangleMesh::CreateFromOrientedBoundingBox(obb);

                    obb_mesh->ComputeVertexNormals();
                    obb_mesh->PaintUniformColor(Eigen::Vector3d(0.9, 0.9, 0.9));

                    *combined_mesh += *obb_mesh;
                    valid_hull_count++;
                } catch (...) {
                    // 彻底失败则跳过该聚类块
                }
            }
        }

        // 4. 检查是否生成了有效的 Mesh
        if (combined_mesh->vertices_.empty()) {
            RCLCPP_WARN(this->get_logger(), "[?] No any valid data generated. Scaped publish.");
            return;
        }

        // 5. 序列化为自定义 ROS 2 消息
        auto out_msg = pc_msgs::msg::O3DMesh();
        out_msg.header = ground_msg->header; // 继承地面点的时间戳

        // 预分配内存以加速
        out_msg.vertices.reserve(combined_mesh->vertices_.size() * 3);
        for (const auto& v : combined_mesh->vertices_) {
            out_msg.vertices.push_back(static_cast<float>(v.x()));
            out_msg.vertices.push_back(static_cast<float>(v.y()));
            out_msg.vertices.push_back(static_cast<float>(v.z()));
        }

        out_msg.triangles.reserve(combined_mesh->triangles_.size() * 3);
        for (const auto& t : combined_mesh->triangles_) {
            out_msg.triangles.push_back(t.x());
            out_msg.triangles.push_back(t.y());
            out_msg.triangles.push_back(t.z());
        }

        if (combined_mesh->HasVertexNormals()) {
            out_msg.vertex_normals.reserve(combined_mesh->vertex_normals_.size() * 3);
            for (const auto& n : combined_mesh->vertex_normals_) {
                out_msg.vertex_normals.push_back(static_cast<float>(n.x()));
                out_msg.vertex_normals.push_back(static_cast<float>(n.y()));
                out_msg.vertex_normals.push_back(static_cast<float>(n.z()));
            }
        }

        if (combined_mesh->HasVertexColors()) {
            out_msg.vertex_colors.reserve(combined_mesh->vertex_colors_.size() * 3);
            for (const auto& c : combined_mesh->vertex_colors_) {
                out_msg.vertex_colors.push_back(static_cast<float>(c.x()));
                out_msg.vertex_colors.push_back(static_cast<float>(c.y()));
                out_msg.vertex_colors.push_back(static_cast<float>(c.z()));
            }
        }

        // 发布
        publisher_->publish(out_msg);
        RCLCPP_INFO(this->get_logger(), "[*] Reconstruction successful: Extract %d shapes.", valid_hull_count);
    }

    int min_cluster_size_;

    // ROS 2 对象
    message_filters::Subscriber<pc_msgs::msg::O3DPointCloud> ground_sub_;
    message_filters::Subscriber<pc_msgs::msg::ClusteredPointCloud> cluster_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Publisher<pc_msgs::msg::O3DMesh>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReconstructionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
