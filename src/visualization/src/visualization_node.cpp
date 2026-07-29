#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "pc_msgs/msg/o3_d_mesh.hpp"
#include "visualization/display_frame_sequence.hpp"
#include "visualization/ros_image_conversion.hpp"

// 引入 Open3D 核心与可视化头文件
#include "open3d/Open3D.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/msg/image.hpp>

using std::placeholders::_1;

namespace {

constexpr char kSourceFrameTopic[] = "interpolation_source_frames";

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

struct FpsWindow {
    bool initialized = false;
    std::chrono::steady_clock::time_point window_started_at;
    std::chrono::steady_clock::time_point last_frame_at;
    double maximum_gap_ms = 0.0;
    std::size_t operation_count = 0;
    double capture_sum_ms = 0.0;
    double maximum_capture_ms = 0.0;
    double encode_sum_ms = 0.0;
    double maximum_encode_ms = 0.0;
    double publish_sum_ms = 0.0;
    double maximum_publish_ms = 0.0;
};

// 用于存储单一相机窗口的配置与实例状态
struct CamConfig {
    std::string id;
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
    std::shared_ptr<open3d::geometry::TriangleMesh> interpolation_render_mesh;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
        source_frame_publisher;
    FpsWindow source_fps;
    std::uint64_t last_reported_superseded_mesh_count = 0;
    std::uint64_t next_source_frame_sequence = 1;
    std::uint64_t total_source_frames_published = 0;
    std::size_t last_source_subscriber_count = 0;
};

class VisualizationNode : public rclcpp::Node {
public:
    VisualizationNode() : Node("visualization_node"), new_mesh_available_(false) {
        // 1. 声明节点的基础话题和相机列表
        this->declare_parameter<std::string>("subscribe_topic", "/reconstruction/white_mesh");
        this->declare_parameter<std::vector<std::string>>("camera_ids", {"cam_front"});
        this->declare_parameter<bool>("fps_logging_enabled", true);
        this->declare_parameter<double>("fps_logging_interval_sec", 5.0);
        this->declare_parameter<bool>("interpolation_enabled", false);
        this->declare_parameter<bool>("interpolation_lock_camera", false);
        this->declare_parameter<bool>("interpolation_show_source_window", false);

        auto sub_topic = this->get_parameter("subscribe_topic").as_string();
        auto camera_ids = this->get_parameter("camera_ids").as_string_array();
        if (camera_ids.empty()) {
            throw std::invalid_argument(
                "camera_ids must contain at least one camera");
        }
        std::unordered_set<std::string> unique_camera_ids;
        for (const auto& camera_id : camera_ids) {
            if (camera_id.empty() ||
                !unique_camera_ids.insert(camera_id).second) {
                throw std::invalid_argument(
                    "camera_ids must be non-empty and unique");
            }
        }
        fps_logging_enabled_ =
            this->get_parameter("fps_logging_enabled").as_bool();
        fps_logging_interval_sec_ =
            this->get_parameter("fps_logging_interval_sec").as_double();
        interpolation_enabled_ =
            this->get_parameter("interpolation_enabled").as_bool();
        interpolation_lock_camera_ =
            this->get_parameter("interpolation_lock_camera").as_bool();
        interpolation_show_source_window_ =
            this->get_parameter("interpolation_show_source_window").as_bool();

        if (fps_logging_enabled_ &&
            (!std::isfinite(fps_logging_interval_sec_) ||
             fps_logging_interval_sec_ <= 0.0)) {
            throw std::invalid_argument(
                "fps_logging_interval_sec must be greater than zero");
        }

        // 2. 遍历参数，动态加载所有视角的窗口配置
        const auto read_vector3_parameter =
            [this](const std::string& parameter_name) {
                const auto values =
                    this->get_parameter(
                        parameter_name).as_double_array();
                if (values.size() != 3 ||
                    !std::all_of(
                        values.begin(),
                        values.end(),
                        [](double value) {
                            return std::isfinite(value);
                        })) {
                    throw std::invalid_argument(
                        parameter_name +
                        " must contain exactly three finite values");
                }
                return Eigen::Vector3d(
                    values[0], values[1], values[2]);
            };

        std::unordered_set<std::string> unique_window_names;
        for (const auto& cam_id : camera_ids) {
            CamConfig cfg;
            cfg.id = cam_id;
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
            if (cfg.name.empty() ||
                !unique_window_names.insert(cfg.name).second) {
                throw std::invalid_argument(
                    "camera window names must be non-empty and unique");
            }
            cfg.width = this->get_parameter(cam_id + ".width").as_int();
            cfg.height = this->get_parameter(cam_id + ".height").as_int();
            if (cfg.width <= 0 || cfg.height <= 0) {
                throw std::invalid_argument(
                    cam_id + " width and height must be positive");
            }

            cfg.bg_color =
                read_vector3_parameter(
                    cam_id + ".background_color");
            if ((cfg.bg_color.array() < 0.0).any() ||
                (cfg.bg_color.array() > 1.0).any()) {
                throw std::invalid_argument(
                    cam_id +
                    ".background_color values must be in [0, 1]");
            }
            cfg.pt_size = this->get_parameter(cam_id + ".point_size").as_double();
            if (!std::isfinite(cfg.pt_size) || cfg.pt_size <= 0.0) {
                throw std::invalid_argument(
                    cam_id +
                    ".point_size must be positive and finite");
            }

            cfg.front =
                read_vector3_parameter(cam_id + ".front");
            cfg.lookat =
                read_vector3_parameter(cam_id + ".lookat");
            cfg.up =
                read_vector3_parameter(cam_id + ".up");
            if (cfg.front.norm() <= 1e-9 ||
                cfg.up.norm() <= 1e-9 ||
                cfg.front.cross(cfg.up).norm() <= 1e-9) {
                throw std::invalid_argument(
                    cam_id +
                    " front and up must be non-zero and non-parallel");
            }

            cfg.zoom = this->get_parameter(cam_id + ".zoom").as_double();
            if (!std::isfinite(cfg.zoom) || cfg.zoom <= 0.0) {
                throw std::invalid_argument(
                    cam_id + ".zoom must be positive and finite");
            }

            // 实例化并创建 Open3D 窗口
            cfg.vis = std::make_shared<open3d::visualization::Visualizer>();
            const bool source_window_visible =
                !interpolation_enabled_ || interpolation_show_source_window_;
            if (!cfg.vis->CreateVisualizerWindow(
                    cfg.name,
                    cfg.width,
                    cfg.height,
                    50,
                    50,
                    source_window_visible)) {
                throw std::runtime_error(
                    "failed to create Open3D source renderer for " + cam_id);
            }

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
                rclcpp::QoS image_qos{rclcpp::KeepAll()};
                image_qos.reliable();
                cfg.source_frame_publisher =
                    this->create_publisher<
                        sensor_msgs::msg::Image>(
                        std::string(kSourceFrameTopic) +
                            "/" + cam_id,
                        image_qos);
            }

            visualizers_.push_back(std::move(cfg));
        }

        // 3. 订阅话题 (运行在 ROS 线程)
        subscription_ = this->create_subscription<pc_msgs::msg::O3DMesh>(
            sub_topic,
            rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&VisualizationNode::mesh_callback, this, _1)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "[*] Visualization initialized for %zu camera pipeline(s).",
            visualizers_.size());
        if (fps_logging_enabled_) {
            RCLCPP_INFO(
                this->get_logger(),
                "[FPS] Logging enabled with a %.1f s measurement window.",
                fps_logging_interval_sec_);
        }
        if (interpolation_enabled_) {
            RCLCPP_INFO(
                this->get_logger(),
                "[*] Visualization publishes only real rendered frames "
                "with reliable KeepAll QoS to \"%s/<camera_id>\". DIS "
                "generation and playback are owned by the display process.",
                kSourceFrameTopic);
            RCLCPP_DEBUG(
                this->get_logger(),
                "[*] Source render window is %s; only the interpolated "
                "window is intended for normal viewing.",
                interpolation_show_source_window_ ? "visible" : "hidden");
        }
    }

    ~VisualizationNode() override {
        cleanup();
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
            const auto source_processing_started_at =
                std::chrono::steady_clock::now();

            try {
            if (mesh_to_render != nullptr) {
                if (interpolation_enabled_) {
                    // Keep one Open3D geometry and update its buffers. Recreating
                    // all render resources for every source mesh blocks image
                    // playback on large meshes.
                    item.vis->PollEvents();
                    if (item.interpolation_render_mesh == nullptr) {
                        item.interpolation_render_mesh =
                            std::make_shared<
                                open3d::geometry::TriangleMesh>(
                                *mesh_to_render);
                        if (!item.vis->AddGeometry(
                                item.interpolation_render_mesh, true)) {
                            throw std::runtime_error(
                                "failed to add interpolation source mesh");
                        }
                    } else {
                        *item.interpolation_render_mesh = *mesh_to_render;
                        if (!item.vis->UpdateGeometry(
                                item.interpolation_render_mesh)) {
                            throw std::runtime_error(
                                "failed to update interpolation source mesh");
                        }
                    }
                } else {
                    item.vis->ClearGeometries();
                    if (!item.vis->AddGeometry(
                            mesh_to_render, item.is_first_frame)) {
                        throw std::runtime_error(
                            "failed to add source mesh");
                    }
                }

                // 光流要求相邻两张图使用相同视角；启用插帧时可锁定相机。
                if (item.is_first_frame ||
                    (interpolation_enabled_ && interpolation_lock_camera_)) {
                    auto& view_ctl = item.vis->GetViewControl();
                    view_ctl.SetFront(item.front);
                    view_ctl.SetLookat(item.lookat);
                    view_ctl.SetUp(item.up);
                    view_ctl.SetZoom(item.zoom);
                }
                item.is_first_frame = false;
            }

            if (!interpolation_enabled_) {
                // Preserve the original visualizer behavior when interpolation
                // is disabled.
                item.vis->PollEvents();
                item.vis->UpdateRender();
                if (mesh_to_render != nullptr) {
                    RecordSourceFrame(
                        item, std::chrono::steady_clock::now());
                }
            } else if (interpolation_show_source_window_) {
                // A visible debug source window still needs event handling,
                // but it must not request a continuous redraw.
                item.vis->PollEvents();
            }

            if (mesh_to_render != nullptr &&
                item.source_frame_publisher != nullptr) {
                try {
                    auto captured_image =
                        item.vis->CaptureScreenFloatBuffer(true);
                    if (captured_image == nullptr) {
                        throw std::runtime_error(
                            "Open3D returned no captured RGB image");
                    }
                    cv::Mat rendered_frame =
                        open3d_float_rgb_to_bgr8(*captured_image);
                    cv::Mat background_difference;
                    cv::absdiff(
                        rendered_frame,
                        cv::Scalar(
                            item.bg_color.z() * 255.0,
                            item.bg_color.y() * 255.0,
                            item.bg_color.x() * 255.0),
                        background_difference);
                    double maximum_background_difference = 0.0;
                    cv::minMaxLoc(
                        background_difference.reshape(1),
                        nullptr,
                        &maximum_background_difference);
                    if (maximum_background_difference <= 1.0) {
                        throw std::runtime_error(
                            "Open3D captured only the background; "
                            "the source mesh was not rendered");
                    }
                    const double capture_ms =
                        std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() -
                            source_processing_started_at).count();
                    PublishSourceFrame(
                        item, rendered_frame, capture_ms);
                    RCLCPP_DEBUG(
                        this->get_logger(),
                        "[*] Source mesh update, render, and capture "
                        "completed in %.1f ms.",
                        capture_ms);
                } catch (const std::exception& error) {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "[!] Failed to publish rendered source frame: %s",
                        error.what());
                }
            }
            } catch (const std::exception& error) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "[!] Source visualization update failed: %s",
                    error.what());
            }

        }

    }

    void cleanup() {
        if (cleanup_completed_) {
            return;
        }
        cleanup_completed_ = true;
        for (auto& item : visualizers_) {
            item.source_frame_publisher.reset();
            if (item.vis != nullptr) {
                item.vis->DestroyVisualizerWindow();
            }
        }
    }

private:
    void PublishSourceFrame(
        CamConfig& item,
        const cv::Mat& source_frame,
        double capture_ms) {
        const auto encode_started_at =
            std::chrono::steady_clock::now();
        sensor_msgs::msg::Image message =
            pointcloud_visualization::BgrMatToImageMessage(
                source_frame);
        const auto encode_finished_at =
            std::chrono::steady_clock::now();
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id =
            pointcloud_visualization::EncodeDisplayFrameSequence(
                item.next_source_frame_sequence++);

        const std::size_t subscriber_count =
            item.source_frame_publisher->get_subscription_count();
        const auto publish_started_at =
            std::chrono::steady_clock::now();
        item.source_frame_publisher->publish(message);
        const auto published_at =
            std::chrono::steady_clock::now();
        ++item.total_source_frames_published;
        item.last_source_subscriber_count =
            subscriber_count;

        const double encode_ms =
            std::chrono::duration<double, std::milli>(
                encode_finished_at -
                encode_started_at).count();
        const double publish_ms =
            std::chrono::duration<double, std::milli>(
                published_at -
                publish_started_at).count();
        RecordSourceFrame(
            item,
            published_at,
            capture_ms,
            encode_ms,
            publish_ms);
    }

    void InitializeFpsWindow(
        FpsWindow& window,
        std::chrono::steady_clock::time_point now) {
        window.initialized = true;
        window.window_started_at = now;
        window.last_frame_at = now;
        window.maximum_gap_ms = 0.0;
        window.operation_count = 0;
        window.capture_sum_ms = 0.0;
        window.maximum_capture_ms = 0.0;
        window.encode_sum_ms = 0.0;
        window.maximum_encode_ms = 0.0;
        window.publish_sum_ms = 0.0;
        window.maximum_publish_ms = 0.0;
    }

    void RecordSourceFrame(
        CamConfig& item,
        std::chrono::steady_clock::time_point now,
        double capture_ms = 0.0,
        double encode_ms = 0.0,
        double publish_ms = 0.0) {
        if (!fps_logging_enabled_) {
            return;
        }

        FpsWindow& window = item.source_fps;
        if (!window.initialized) {
            item.last_reported_superseded_mesh_count =
                superseded_mesh_count_.load();
            InitializeFpsWindow(window, now);
        } else {
            const double gap_sec =
                std::chrono::duration<double>(
                    now - window.last_frame_at).count();
            window.last_frame_at = now;
            window.maximum_gap_ms =
                std::max(
                    window.maximum_gap_ms,
                    gap_sec * 1000.0);
        }
        ++window.operation_count;
        window.capture_sum_ms += capture_ms;
        window.maximum_capture_ms = std::max(
            window.maximum_capture_ms, capture_ms);
        window.encode_sum_ms += encode_ms;
        window.maximum_encode_ms = std::max(
            window.maximum_encode_ms, encode_ms);
        window.publish_sum_ms += publish_ms;
        window.maximum_publish_ms = std::max(
            window.maximum_publish_ms, publish_ms);

        const double elapsed_sec = std::chrono::duration<double>(
            now - window.window_started_at).count();
        if (elapsed_sec < fps_logging_interval_sec_) {
            return;
        }

        const double measured_fps =
            static_cast<double>(window.operation_count) /
            elapsed_sec;
        const std::uint64_t superseded_mesh_total =
            superseded_mesh_count_.load();
        const std::uint64_t superseded_mesh_delta =
            superseded_mesh_total -
            item.last_reported_superseded_mesh_count;
        item.last_reported_superseded_mesh_count =
            superseded_mesh_total;
        if (interpolation_enabled_) {
            RCLCPP_INFO(
                this->get_logger(),
                "[SRC-TX] \"%s\": published %.2f FPS "
                "(+%zu, total %llu) | %.1f s window | "
                "max source gap %.0f ms | render/capture avg/max "
                "%.1f/%.1f ms | image encode avg/max %.1f/%.1f ms | "
                "DDS publish-call avg/max %.1f/%.1f ms | "
                "last source #%llu | subscribers %zu | "
                "superseded meshes +%llu "
                "(total %llu).",
                item.name.c_str(),
                measured_fps,
                window.operation_count,
                static_cast<unsigned long long>(
                    item.total_source_frames_published),
                elapsed_sec,
                window.maximum_gap_ms,
                window.capture_sum_ms /
                    static_cast<double>(window.operation_count),
                window.maximum_capture_ms,
                window.encode_sum_ms /
                    static_cast<double>(window.operation_count),
                window.maximum_encode_ms,
                window.publish_sum_ms /
                    static_cast<double>(window.operation_count),
                window.maximum_publish_ms,
                static_cast<unsigned long long>(
                    item.next_source_frame_sequence - 1),
                item.last_source_subscriber_count,
                static_cast<unsigned long long>(
                    superseded_mesh_delta),
                static_cast<unsigned long long>(
                    superseded_mesh_total));
        } else {
            RCLCPP_INFO(
                this->get_logger(),
                "[FPS] Display \"%s\" (no interpolation): "
                "presented %.2f FPS | %.1f s window | "
                "max Open3D display-boundary gap %.0f ms | "
                "superseded meshes +%llu (total %llu).",
                item.name.c_str(),
                measured_fps,
                elapsed_sec,
                window.maximum_gap_ms,
                static_cast<unsigned long long>(
                    superseded_mesh_delta),
                static_cast<unsigned long long>(
                    superseded_mesh_total));
        }

        InitializeFpsWindow(window, now);
    }

    void mesh_callback(const pc_msgs::msg::O3DMesh::SharedPtr msg) {
        if (msg->vertices.empty() ||
            msg->vertices.size() % 3 != 0) {
            RCLCPP_WARN(
                this->get_logger(),
                "[?] Rejected mesh with an empty or malformed "
                "vertex array.");
            return;
        }
        if (msg->triangles.empty() ||
            msg->triangles.size() % 3 != 0) {
            RCLCPP_WARN(
                this->get_logger(),
                "[?] Rejected mesh with an empty or malformed "
                "triangle array.");
            return;
        }
        if ((!msg->vertex_normals.empty() &&
             msg->vertex_normals.size() != msg->vertices.size()) ||
            (!msg->vertex_colors.empty() &&
             msg->vertex_colors.size() != msg->vertices.size())) {
            RCLCPP_WARN(
                this->get_logger(),
                "[?] Rejected mesh whose normal/color count does "
                "not match its vertex count.");
            return;
        }

        auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();

        // 1. 反序列化：还原顶点
        size_t num_vertices = msg->vertices.size() / 3;
        mesh->vertices_.reserve(num_vertices);
        for (size_t i = 0; i < num_vertices; ++i) {
            if (!std::isfinite(msg->vertices[i*3]) ||
                !std::isfinite(msg->vertices[i*3+1]) ||
                !std::isfinite(msg->vertices[i*3+2])) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "[?] Rejected mesh containing a non-finite vertex.");
                return;
            }
            mesh->vertices_.emplace_back(msg->vertices[i*3], msg->vertices[i*3+1], msg->vertices[i*3+2]);
        }

        // 2. 反序列化：还原面片 (Triangle indices)
        size_t num_triangles = msg->triangles.size() / 3;
        mesh->triangles_.reserve(num_triangles);
        for (size_t i = 0; i < num_triangles; ++i) {
            const auto first_index = msg->triangles[i*3];
            const auto second_index = msg->triangles[i*3+1];
            const auto third_index = msg->triangles[i*3+2];
            if (first_index < 0 ||
                second_index < 0 ||
                third_index < 0 ||
                static_cast<size_t>(first_index) >= num_vertices ||
                static_cast<size_t>(second_index) >= num_vertices ||
                static_cast<size_t>(third_index) >= num_vertices) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "[?] Rejected mesh containing an out-of-range "
                    "triangle index.");
                return;
            }
            mesh->triangles_.emplace_back(msg->triangles[i*3], msg->triangles[i*3+1], msg->triangles[i*3+2]);
        }

        // 3. 反序列化：还原法线
        if (!msg->vertex_normals.empty()) {
            size_t num_normals = msg->vertex_normals.size() / 3;
            mesh->vertex_normals_.reserve(num_normals);
            for (size_t i = 0; i < num_normals; ++i) {
                if (!std::isfinite(msg->vertex_normals[i*3]) ||
                    !std::isfinite(msg->vertex_normals[i*3+1]) ||
                    !std::isfinite(msg->vertex_normals[i*3+2])) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "[?] Rejected mesh containing a non-finite "
                        "vertex normal.");
                    return;
                }
                mesh->vertex_normals_.emplace_back(msg->vertex_normals[i*3], msg->vertex_normals[i*3+1], msg->vertex_normals[i*3+2]);
            }
        }

        // 4. 反序列化：还原颜色
        if (!msg->vertex_colors.empty()) {
            size_t num_colors = msg->vertex_colors.size() / 3;
            mesh->vertex_colors_.reserve(num_colors);
            for (size_t i = 0; i < num_colors; ++i) {
                if (!std::isfinite(msg->vertex_colors[i*3]) ||
                    !std::isfinite(msg->vertex_colors[i*3+1]) ||
                    !std::isfinite(msg->vertex_colors[i*3+2])) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "[?] Rejected mesh containing a non-finite "
                        "vertex color.");
                    return;
                }
                mesh->vertex_colors_.emplace_back(msg->vertex_colors[i*3], msg->vertex_colors[i*3+1], msg->vertex_colors[i*3+2]);
            }
        }

        // 5. 线程安全的数据投递
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (new_mesh_available_) {
                ++superseded_mesh_count_;
            }
            latest_mesh_ = mesh;
            new_mesh_available_ = true;
        }
    }

    bool cleanup_completed_ = false;
    std::vector<CamConfig> visualizers_;
    bool fps_logging_enabled_ = true;
    double fps_logging_interval_sec_ = 5.0;
    bool interpolation_enabled_ = false;
    bool interpolation_lock_camera_ = false;
    bool interpolation_show_source_window_ = false;
    // 线程同步
    std::mutex mutex_;
    std::shared_ptr<open3d::geometry::TriangleMesh> latest_mesh_;
    std::atomic<std::uint64_t> superseded_mesh_count_{0};
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
