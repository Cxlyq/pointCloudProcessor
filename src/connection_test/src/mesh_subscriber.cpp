#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "pc_msgs/msg/o3_d_mesh.hpp" // 包含 pc_msgs 接口头文件

using namespace std::chrono_literals;

class MeshSubscriber : public rclcpp::Node {
public:
  MeshSubscriber()
  : Node("mesh_subscriber"),
    frame_count_(0),
    total_bytes_(0),
    latency_sum_ms_(0.0)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5))
             .reliable()                    // 保证分片完整性
             .durability_volatile();
    subscription_ = this->create_subscription<pc_msgs::msg::O3DMesh>(
      "mesh_topic", qos,
      std::bind(&MeshSubscriber::topic_callback, this, std::placeholders::_1));

    last_stat_time_ = std::chrono::steady_clock::now();
    // 每秒输出一次性能报告
    stat_timer_ = this->create_wall_timer(
      1000ms, std::bind(&MeshSubscriber::print_statistics, this));
  }

private:
  void topic_callback(const pc_msgs::msg::O3DMesh::SharedPtr msg) {
    auto now = this->now();
    // 计算单向传输时间差（毫秒）
    double latency_ms = (now - msg->header.stamp).seconds() * 1000.0;

    frame_count_++;
    latency_sum_ms_ += latency_ms;

    // 统计接收字节数 (Bytes)
    size_t payload_bytes = msg->vertices.size() * sizeof(float) +
                           msg->triangles.size() * sizeof(int32_t) +
                           msg->vertex_normals.size() * sizeof(float) +
                           msg->vertex_colors.size() * sizeof(float);
    total_bytes_ += payload_bytes;
  }

  void print_statistics() {
    auto now = std::chrono::steady_clock::now();
    double elapsed_sec = std::chrono::duration<double>(now - last_stat_time_).count();

    if (frame_count_ > 0) {
      double fps = frame_count_ / elapsed_sec;
      double avg_latency = latency_sum_ms_ / frame_count_;
      double bandwidth_mb = (total_bytes_ / (1024.0 * 1024.0)) / elapsed_sec;

      RCLCPP_INFO(this->get_logger(),
        "[吞吐统计] 帧率: %.1f FPS | 平均网络延迟: %.3f ms | 带宽负载: %.2f MB/s",
        fps, avg_latency, bandwidth_mb);
    } else {
      RCLCPP_WARN(this->get_logger(), "等待接收数据中... (当前尚未收到 mesh 数据帧)");
    }

    // 重置统计计数器
    frame_count_ = 0;
    total_bytes_ = 0;
    latency_sum_ms_ = 0.0;
    last_stat_time_ = now;
  }

  rclcpp::Subscription<pc_msgs::msg::O3DMesh>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr stat_timer_;
  std::chrono::steady_clock::time_point last_stat_time_;
  uint32_t frame_count_;
  size_t total_bytes_;
  double latency_sum_ms_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MeshSubscriber>());
  rclcpp::shutdown();
  return 0;
}