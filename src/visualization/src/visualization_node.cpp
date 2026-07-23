#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "pc_msgs/msg/o3_d_mesh.hpp"
#include "visualization/dis_frame_interpolator.hpp"

// 引入 Open3D 核心与可视化头文件
#include "open3d/Open3D.h"
#include <Eigen/Core>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

using std::placeholders::_1;

namespace {

int parse_dis_preset(std::string preset) {
    std::transform(
        preset.begin(), preset.end(), preset.begin(),
        [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });

    if (preset == "ultrafast") {
        return cv::DISOpticalFlow::PRESET_ULTRAFAST;
    }
    if (preset == "fast") {
        return cv::DISOpticalFlow::PRESET_FAST;
    }
    if (preset == "medium") {
        return cv::DISOpticalFlow::PRESET_MEDIUM;
    }

    throw std::invalid_argument(
        "interpolation_dis_preset must be ultrafast, fast, or medium");
}

cv::Mat open3d_float_rgb_to_bgr8(open3d::geometry::Image& image) {
    if (image.width_ <= 0 || image.height_ <= 0) {
        throw std::invalid_argument("captured Open3D image is empty");
    }
    if (image.num_of_channels_ != 3 ||
        image.bytes_per_channel_ != static_cast<int>(sizeof(float))) {
        throw std::invalid_argument(
            "captured Open3D image must contain three float RGB channels");
    }

    cv::Mat rgb_float(
        image.height_, image.width_, CV_32FC3,
        static_cast<void*>(image.data_.data()));
    cv::Mat rgb8;
    rgb_float.convertTo(rgb8, CV_8UC3, 255.0);

    cv::Mat bgr8;
    cv::cvtColor(rgb8, bgr8, cv::COLOR_RGB2BGR);
    return bgr8;
}

}  // namespace

// 用于存储单一相机窗口的配置与实例状态
struct CamConfig {
    std::string name;
    int width;
    int height;
    Eigen::Vector3d bg_color;
    double pt_size;
    Eigen::Vector3d front;
    Eigen::Vector3d lookat;
    Eigen::Vector3d up;
    double zoom;
    bool is_first_frame = true;
    std::shared_ptr<open3d::visualization::Visualizer> vis;
    std::string interpolation_window_name;
    std::unique_ptr<pointcloud_visualization::DisFrameInterpolator> interpolator;
    cv::Mat displayed_interpolated_frame;
};

class VisualizationNode : public rclcpp::Node {
public:
    VisualizationNode() : Node("visualization_node"), new_mesh_available_(false) {
        // 1. 声明节点的基础话题和相机列表
        this->declare_parameter<std::string>("subscribe_topic", "/reconstruction/white_mesh");
        this->declare_parameter<std::vector<std::string>>("camera_ids", {"cam_front"});
        this->declare_parameter<bool>("interpolation_enabled", false);
        this->declare_parameter<double>("interpolation_output_fps", 10.0);
        this->declare_parameter<double>("interpolation_duration_sec", 2.0);
        this->declare_parameter<double>("interpolation_flow_scale", 0.25);
        this->declare_parameter<std::string>("interpolation_dis_preset", "ultrafast");
        this->declare_parameter<bool>("interpolation_lock_camera", true);
        this->declare_parameter<std::string>(
            "interpolation_window_suffix", " - DIS Interpolated");

        auto sub_topic = this->get_parameter("subscribe_topic").as_string();
        auto camera_ids = this->get_parameter("camera_ids").as_string_array();
        interpolation_enabled_ =
            this->get_parameter("interpolation_enabled").as_bool();
        interpolation_output_fps_ =
            this->get_parameter("interpolation_output_fps").as_double();
        interpolation_duration_sec_ =
            this->get_parameter("interpolation_duration_sec").as_double();
        interpolation_flow_scale_ =
            this->get_parameter("interpolation_flow_scale").as_double();
        interpolation_dis_preset_name_ =
            this->get_parameter("interpolation_dis_preset").as_string();
        interpolation_lock_camera_ =
            this->get_parameter("interpolation_lock_camera").as_bool();
        interpolation_window_suffix_ =
            this->get_parameter("interpolation_window_suffix").as_string();

        if (interpolation_enabled_) {
            interpolation_dis_preset_ =
                parse_dis_preset(interpolation_dis_preset_name_);
        }

        // 2. 遍历参数，动态加载所有视角的窗口配置
        for (const auto& cam_id : camera_ids) {
            CamConfig cfg;
            this->declare_parameter<std::string>(cam_id + ".name", "Render - " + cam_id);
            this->declare_parameter<int>(cam_id + ".width", 1280);
            this->declare_parameter<int>(cam_id + ".height", 720);
            this->declare_parameter<std::vector<double>>(cam_id + ".background_color", {0.1, 0.1, 0.1});
            this->declare_parameter<double>(cam_id + ".point_size", 2.0);
            this->declare_parameter<std::vector<double>>(cam_id + ".front", {0.0, -1.0, 0.35});
            this->declare_parameter<std::vector<double>>(cam_id + ".lookat", {-17.5, 2213.0, -86.0});
            this->declare_parameter<std::vector<double>>(cam_id + ".up", {0.0, 0.35, 1.0});
            this->declare_parameter<double>(cam_id + ".zoom", 0.7);

            cfg.name = this->get_parameter(cam_id + ".name").as_string();
            cfg.width = this->get_parameter(cam_id + ".width").as_int();
            cfg.height = this->get_parameter(cam_id + ".height").as_int();

            auto bg = this->get_parameter(cam_id + ".background_color").as_double_array();
            cfg.bg_color = Eigen::Vector3d(bg[0], bg[1], bg[2]);
            cfg.pt_size = this->get_parameter(cam_id + ".point_size").as_double();

            auto front = this->get_parameter(cam_id + ".front").as_double_array();
            cfg.front = Eigen::Vector3d(front[0], front[1], front[2]);

            auto lookat = this->get_parameter(cam_id + ".lookat").as_double_array();
            cfg.lookat = Eigen::Vector3d(lookat[0], lookat[1], lookat[2]);

            auto up = this->get_parameter(cam_id + ".up").as_double_array();
            cfg.up = Eigen::Vector3d(up[0], up[1], up[2]);

            cfg.zoom = this->get_parameter(cam_id + ".zoom").as_double();

            // 实例化并创建 Open3D 窗口
            cfg.vis = std::make_shared<open3d::visualization::Visualizer>();
            cfg.vis->CreateVisualizerWindow(cfg.name, cfg.width, cfg.height);

            // 配置渲染选项 (对应原 Python 代码的 vis.get_render_option())
            auto& opt = cfg.vis->GetRenderOption();
            opt.background_color_ = cfg.bg_color;
            opt.point_size_ = cfg.pt_size;
            opt.light_on_ = true;
            opt.mesh_show_wireframe_ = true;
            opt.line_width_ = 1.5;
            opt.mesh_show_back_face_ = true;
            opt.mesh_color_option_ = open3d::visualization::RenderOption::MeshColorOption::Color;

            if (interpolation_enabled_) {
                pointcloud_visualization::DisInterpolationConfig interpolation_config;
                interpolation_config.output_fps = interpolation_output_fps_;
                interpolation_config.duration_sec = interpolation_duration_sec_;
                interpolation_config.flow_scale = interpolation_flow_scale_;
                interpolation_config.dis_preset = interpolation_dis_preset_;
                interpolation_config.border_color_bgr = cv::Scalar(
                    cfg.bg_color.z() * 255.0,
                    cfg.bg_color.y() * 255.0,
                    cfg.bg_color.x() * 255.0);

                cfg.interpolation_window_name =
                    cfg.name + interpolation_window_suffix_;
                cfg.interpolator =
                    std::make_unique<
                        pointcloud_visualization::DisFrameInterpolator>(
                        interpolation_config);

                cv::namedWindow(
                    cfg.interpolation_window_name, cv::WINDOW_NORMAL);
                cv::resizeWindow(
                    cfg.interpolation_window_name, cfg.width, cfg.height);
            }

            visualizers_.push_back(std::move(cfg));
        }

        // 3. 订阅话题 (运行在 ROS 线程)
        subscription_ = this->create_subscription<pc_msgs::msg::O3DMesh>(
            sub_topic, 10, std::bind(&VisualizationNode::mesh_callback, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "[*] Visualization started. %zu windows have been brought up.", visualizers_.size());
        if (interpolation_enabled_) {
            const int interval_count = std::max(
                1, static_cast<int>(std::lround(
                       interpolation_output_fps_ *
                       interpolation_duration_sec_)));
            RCLCPP_INFO(
                this->get_logger(),
                "[*] DIS interpolation enabled: %.1f FPS, %.2f s per pair, "
                "%d intermediate frames, scale %.2f, preset %s.",
                interpolation_output_fps_,
                interpolation_duration_sec_,
                std::max(0, interval_count - 1),
                interpolation_flow_scale_,
                interpolation_dis_preset_name_.c_str());
        }
    }

    // 维持 UI 心跳的主循环函数
    void update_ui() {
        std::shared_ptr<open3d::geometry::TriangleMesh> mesh_to_render = nullptr;

        // 从交换区安全地取出新数据
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (new_mesh_available_) {
                mesh_to_render = latest_mesh_;
                new_mesh_available_ = false;
            }
        }

        // 遍历更新所有渲染窗口
        for (auto& item : visualizers_) {
            if (mesh_to_render != nullptr) {
                item.vis->ClearGeometries();
                item.vis->AddGeometry(mesh_to_render, item.is_first_frame);

                // 光流要求相邻两张图使用相同视角；启用插帧时可锁定相机。
                if (item.is_first_frame || interpolation_lock_camera_) {
                    auto& view_ctl = item.vis->GetViewControl();
                    view_ctl.SetFront(item.front);
                    view_ctl.SetLookat(item.lookat);
                    view_ctl.SetUp(item.up);
                    view_ctl.SetZoom(item.zoom);
                }
                item.is_first_frame = false;
            }

            // 响应窗口事件，防止 UI 卡死
            item.vis->PollEvents();
            item.vis->UpdateRender();

            if (mesh_to_render != nullptr && item.interpolator != nullptr) {
                try {
                    auto captured_image =
                        item.vis->CaptureScreenFloatBuffer(true);
                    if (captured_image == nullptr) {
                        throw std::runtime_error(
                            "Open3D returned no captured RGB image");
                    }
                    item.interpolator->SubmitFrame(
                        open3d_float_rgb_to_bgr8(*captured_image));
                } catch (const std::exception& error) {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "[!] Failed to capture rendered frame for interpolation: %s",
                        error.what());
                }
            }

            if (item.interpolator != nullptr) {
                cv::Mat interpolated_frame;
                if (item.interpolator->TryGetDisplayFrame(
                        interpolated_frame)) {
                    item.displayed_interpolated_frame =
                        std::move(interpolated_frame);
                    cv::imshow(
                        item.interpolation_window_name,
                        item.displayed_interpolated_frame);
                }

                const std::string interpolation_status =
                    item.interpolator->ConsumeStatus();
                if (!interpolation_status.empty()) {
                    RCLCPP_INFO(
                        this->get_logger(), "[*] %s",
                        interpolation_status.c_str());
                }

                const std::string interpolation_error =
                    item.interpolator->ConsumeError();
                if (!interpolation_error.empty()) {
                    RCLCPP_WARN(
                        this->get_logger(), "[?] %s",
                        interpolation_error.c_str());
                }
            }
        }

        if (interpolation_enabled_) {
            cv::waitKey(1);
        }
    }

    void cleanup() {
        for (auto& item : visualizers_) {
            item.interpolator.reset();
            if (!item.interpolation_window_name.empty()) {
                cv::destroyWindow(item.interpolation_window_name);
            }
            item.vis->DestroyVisualizerWindow();
        }
    }

private:
    void mesh_callback(const pc_msgs::msg::O3DMesh::SharedPtr msg) {
        auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();

        // 1. 反序列化：还原顶点
        size_t num_vertices = msg->vertices.size() / 3;
        mesh->vertices_.reserve(num_vertices);
        for (size_t i = 0; i < num_vertices; ++i) {
            mesh->vertices_.emplace_back(msg->vertices[i*3], msg->vertices[i*3+1], msg->vertices[i*3+2]);
        }

        // 2. 反序列化：还原面片 (Triangle indices)
        size_t num_triangles = msg->triangles.size() / 3;
        mesh->triangles_.reserve(num_triangles);
        for (size_t i = 0; i < num_triangles; ++i) {
            mesh->triangles_.emplace_back(msg->triangles[i*3], msg->triangles[i*3+1], msg->triangles[i*3+2]);
        }

        // 3. 反序列化：还原法线
        if (!msg->vertex_normals.empty()) {
            size_t num_normals = msg->vertex_normals.size() / 3;
            mesh->vertex_normals_.reserve(num_normals);
            for (size_t i = 0; i < num_normals; ++i) {
                mesh->vertex_normals_.emplace_back(msg->vertex_normals[i*3], msg->vertex_normals[i*3+1], msg->vertex_normals[i*3+2]);
            }
        }

        // 4. 反序列化：还原颜色
        if (!msg->vertex_colors.empty()) {
            size_t num_colors = msg->vertex_colors.size() / 3;
            mesh->vertex_colors_.reserve(num_colors);
            for (size_t i = 0; i < num_colors; ++i) {
                mesh->vertex_colors_.emplace_back(msg->vertex_colors[i*3], msg->vertex_colors[i*3+1], msg->vertex_colors[i*3+2]);
            }
        }

        // 5. 线程安全的数据投递
        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_mesh_ = mesh;
            new_mesh_available_ = true;
        }
    }

    std::vector<CamConfig> visualizers_;
    bool interpolation_enabled_ = false;
    double interpolation_output_fps_ = 10.0;
    double interpolation_duration_sec_ = 2.0;
    double interpolation_flow_scale_ = 0.25;
    int interpolation_dis_preset_ =
        cv::DISOpticalFlow::PRESET_ULTRAFAST;
    std::string interpolation_dis_preset_name_ = "ultrafast";
    bool interpolation_lock_camera_ = true;
    std::string interpolation_window_suffix_ = " - DIS Interpolated";

    // 线程同步
    std::mutex mutex_;
    std::shared_ptr<open3d::geometry::TriangleMesh> latest_mesh_;
    bool new_mesh_available_;

    rclcpp::Subscription<pc_msgs::msg::O3DMesh>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisualizationNode>();

    // 1. 将 ROS 2 的通讯自旋(spin) 放到后台线程
    std::thread ros_thread([&node]() {
        rclcpp::spin(node);
    });

    // 2. 将主线程保留给 Open3D，维持 UI 刷新
    while (rclcpp::ok()) {
        node->update_ui();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 3. 清理流程
    node->cleanup();
    rclcpp::shutdown();

    if (ros_thread.joinable()) {
        ros_thread.join();
    }

    return 0;
}
