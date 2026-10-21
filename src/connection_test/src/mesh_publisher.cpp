#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "pc_msgs/msg/o3_d_mesh.hpp" // 包含 pc_msgs 接口头文件

using namespace std::chrono_literals;

class MeshPublisher : public rclcpp::Node {
public:
    MeshPublisher() : Node("mesh_publisher"), running_(true) {
        // 设置 QoS 深度为 10，采用 Best Effort 策略测试物理极限传输能力
        auto qos = rclcpp::QoS(rclcpp::KeepLast(5))
             .reliable()                    // 保证分片完整性
             .durability_volatile();
        publisher_ = this->create_publisher<pc_msgs::msg::O3DMesh>("mesh_topic", qos);

        // 预填 4000 个面（12000 个索引点）的测试数据
        prebuild_mesh_data();

        RCLCPP_INFO(this->get_logger(), "开始发布 O3DMesh 测试数据 (4000 面)...");
        send_thread_ = std::thread(&MeshPublisher::run_benchmark, this);
    }

    ~MeshPublisher() {
        running_ = false;
        if (send_thread_.joinable()) {
            send_thread_.join();
        }
    }

private:
    void prebuild_mesh_data() {
        // 4000 个三角形，每个三角形包含 3 个顶点索引
        const size_t num_indices = 4000 * 3;
        mesh_template_.triangles.resize(num_indices, 1);
        mesh_template_.vertices.resize(num_indices, 1.0f);
        mesh_template_.vertex_normals.resize(num_indices, 0.577f);
        mesh_template_.vertex_colors.resize(num_indices, 0.8f);
    }

    void run_benchmark() {
        while (running_ && rclcpp::ok()) {
            auto msg = mesh_template_;
            // 打上当前系统纳秒时间戳，用于单向延迟评测
            msg.header.stamp = this->now();
            publisher_->publish(msg);

            // 控制推送频率：500 微秒（理论最高 2000 FPS）
            // 如需跑满网卡物理上限，可调低至 200 或 100 微秒
            std::this_thread::sleep_for(std::chrono::microseconds(20000));
        }
    }

    bool running_;
    std::thread send_thread_;
    rclcpp::Publisher<pc_msgs::msg::O3DMesh>::SharedPtr publisher_;
    pc_msgs::msg::O3DMesh mesh_template_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MeshPublisher>());
    rclcpp::shutdown();
    return 0;
}