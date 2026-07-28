#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "pc_msgs/msg/o3_d_mesh.hpp"
#include "visualization/dis_frame_interpolator.hpp"

// 引入 Open3D 核心与可视化头文件
#include "open3d/Open3D.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
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

struct FpsWindow {
    bool initialized = false;
    std::chrono::steady_clock::time_point window_started_at;
    std::chrono::steady_clock::time_point last_frame_at;
    std::size_t interval_count = 0;
    double maximum_gap_ms = 0.0;
    std::size_t playback_interval_count = 0;
    double playback_interval_sum_sec = 0.0;
};

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
    std::shared_ptr<open3d::geometry::TriangleMesh> interpolation_render_mesh;
    std::string interpolation_window_name;
    std::unique_ptr<pointcloud_visualization::DisFrameInterpolator> interpolator;
    cv::Mat displayed_interpolated_frame;
    FpsWindow source_fps;
    FpsWindow interpolation_fps;
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
        this->declare_parameter<int>("interpolation_intermediate_frames", 4);
        this->declare_parameter<double>("interpolation_flow_scale", 0.25);
        this->declare_parameter<std::string>("interpolation_dis_preset", "ultrafast");
        this->declare_parameter<bool>("interpolation_bidirectional_flow", false);
        this->declare_parameter<std::string>(
            "interpolation_timing_source", "arrival");
        this->declare_parameter<bool>("interpolation_lock_camera", false);
        this->declare_parameter<bool>("interpolation_show_source_window", false);
        this->declare_parameter<std::string>(
            "interpolation_window_suffix", " - DIS Interpolated");

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
        interpolation_intermediate_frames_ =
            this->get_parameter(
                "interpolation_intermediate_frames").as_int();
        interpolation_flow_scale_ =
            this->get_parameter("interpolation_flow_scale").as_double();
        interpolation_dis_preset_name_ =
            this->get_parameter("interpolation_dis_preset").as_string();
        interpolation_bidirectional_flow_ =
            this->get_parameter("interpolation_bidirectional_flow").as_bool();
        interpolation_timing_source_ =
            this->get_parameter(
                "interpolation_timing_source").as_string();
        interpolation_lock_camera_ =
            this->get_parameter("interpolation_lock_camera").as_bool();
        interpolation_show_source_window_ =
            this->get_parameter("interpolation_show_source_window").as_bool();
        interpolation_window_suffix_ =
            this->get_parameter("interpolation_window_suffix").as_string();

        if (fps_logging_enabled_ &&
            (!std::isfinite(fps_logging_interval_sec_) ||
             fps_logging_interval_sec_ <= 0.0)) {
            throw std::invalid_argument(
                "fps_logging_interval_sec must be greater than zero");
        }

        if (interpolation_enabled_) {
            if (interpolation_intermediate_frames_ < 0) {
                throw std::invalid_argument(
                    "interpolation_intermediate_frames must be "
                    "greater than or equal to zero");
            }
            if (interpolation_intermediate_frames_ > 120) {
                throw std::invalid_argument(
                    "interpolation_intermediate_frames must not "
                    "exceed 120");
            }
            if (!std::isfinite(interpolation_flow_scale_) ||
                interpolation_flow_scale_ <= 0.0 ||
                interpolation_flow_scale_ > 1.0) {
                throw std::invalid_argument(
                    "interpolation_flow_scale must be in (0, 1]");
            }
            interpolation_dis_preset_ =
                parse_dis_preset(interpolation_dis_preset_name_);
            std::transform(
                interpolation_timing_source_.begin(),
                interpolation_timing_source_.end(),
                interpolation_timing_source_.begin(),
                [](unsigned char character) {
                    return static_cast<char>(
                        std::tolower(character));
                });
            if (interpolation_timing_source_ == "arrival") {
                interpolation_use_message_timestamps_ = false;
            } else if (
                interpolation_timing_source_ == "message_stamp") {
                interpolation_use_message_timestamps_ = true;
            } else {
                throw std::invalid_argument(
                    "interpolation_timing_source must be arrival "
                    "or message_stamp");
            }
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
                pointcloud_visualization::DisInterpolationConfig interpolation_config;
                interpolation_config.intermediate_frame_count =
                    static_cast<std::size_t>(
                        interpolation_intermediate_frames_);
                interpolation_config.flow_scale = interpolation_flow_scale_;
                interpolation_config.dis_preset = interpolation_dis_preset_;
                interpolation_config.use_bidirectional_flow =
                    interpolation_bidirectional_flow_;
                interpolation_config.use_source_timestamps =
                    interpolation_use_message_timestamps_;
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

        if (interpolation_enabled_) {
            highgui_window_thread_started_ =
                cv::startWindowThread() > 0;
        }

        // 3. 订阅话题 (运行在 ROS 线程)
        subscription_ = this->create_subscription<pc_msgs::msg::O3DMesh>(
            sub_topic,
            rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&VisualizationNode::mesh_callback, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "[*] Visualization started. %zu windows have been brought up.", visualizers_.size());
        if (fps_logging_enabled_) {
            RCLCPP_INFO(
                this->get_logger(),
                "[FPS] Logging enabled with a %.1f s measurement window.",
                fps_logging_interval_sec_);
        }
        if (interpolation_enabled_) {
            RCLCPP_INFO(
                this->get_logger(),
                "[*] %s DIS interpolation enabled: %d intermediate "
                "frames per real-frame pair (approximately %dx source "
                "frame count), scale %.2f, preset %s.",
                interpolation_bidirectional_flow_ ?
                    "Bidirectional" : "Single-direction",
                interpolation_intermediate_frames_,
                interpolation_intermediate_frames_ + 1,
                interpolation_flow_scale_,
                interpolation_dis_preset_name_.c_str());
            RCLCPP_INFO(
                this->get_logger(),
                "[*] Interpolation timing source: %s.",
                interpolation_timing_source_.c_str());
            RCLCPP_DEBUG(
                this->get_logger(),
                "[*] Source render window is %s; only the interpolated "
                "window is intended for normal viewing.",
                interpolation_show_source_window_ ? "visible" : "hidden");
            RCLCPP_DEBUG(
                this->get_logger(),
                "[*] OpenCV HighGUI event thread is %s.",
                highgui_window_thread_started_ ?
                    "enabled" : "not available; using waitKey fallback");
        }
    }

    // 维持 UI 心跳的主循环函数
    void update_ui() {
        std::shared_ptr<open3d::geometry::TriangleMesh> mesh_to_render = nullptr;
        std::chrono::steady_clock::time_point mesh_received_at;
        std::optional<std::chrono::nanoseconds>
            mesh_source_timestamp;

        // 从交换区安全地取出新数据
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (new_mesh_available_) {
                mesh_to_render = latest_mesh_;
                mesh_received_at = latest_mesh_received_at_;
                mesh_source_timestamp =
                    latest_mesh_source_timestamp_;
                new_mesh_available_ = false;
            }
        }

        // 遍历更新所有渲染窗口
        // Present every due interpolation frame before any Open3D source
        // rendering. A slow source capture must not delay another camera's
        // already-ready HighGUI frame.
        if (interpolation_enabled_) {
            for (auto& item : visualizers_) {
                if (item.interpolator != nullptr) {
                    ShowReadyInterpolatedFrame(item);
                }
            }
            ProcessHighGuiEvents();
        }

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

            if (mesh_to_render != nullptr && item.interpolator != nullptr) {
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
                    item.interpolator->SubmitFrame(
                        rendered_frame,
                        mesh_received_at,
                        mesh_source_timestamp);
                    RecordSourceFrame(
                        item, std::chrono::steady_clock::now());
                    const double capture_ms =
                        std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() -
                            source_processing_started_at).count();
                    RCLCPP_DEBUG(
                        this->get_logger(),
                        "[*] Source mesh update, render, and capture "
                        "completed in %.1f ms.",
                        capture_ms);
                } catch (const std::exception& error) {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "[!] Failed to capture rendered frame for interpolation: %s",
                        error.what());
                }
            }
            } catch (const std::exception& error) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "[!] Source visualization update failed: %s",
                    error.what());
            }

            if (item.interpolator != nullptr) {
                const std::string interpolation_status =
                    item.interpolator->ConsumeStatus();
                if (!interpolation_status.empty()) {
                    RCLCPP_INFO(
                        this->get_logger(), "[GEN] %s",
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

        // Source rendering may have taken longer than one display period.
        // Give the continuous timeline one immediate catch-up opportunity.
        if (interpolation_enabled_) {
            for (auto& item : visualizers_) {
                if (item.interpolator != nullptr) {
                    ShowReadyInterpolatedFrame(item);
                }
            }
            ProcessHighGuiEvents();
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
    void ProcessHighGuiEvents() {
        if (highgui_window_thread_started_) {
            return;
        }

        const auto event_processing_started_at =
            std::chrono::steady_clock::now();
        cv::waitKey(1);
        const auto event_processing_finished_at =
            std::chrono::steady_clock::now();
        const double event_processing_ms =
            std::chrono::duration<double, std::milli>(
                event_processing_finished_at -
                event_processing_started_at).count();
        if (event_processing_ms > 100.0 &&
            event_processing_finished_at -
                last_highgui_slow_warning_at_ >
                std::chrono::seconds(2)) {
            RCLCPP_WARN(
                this->get_logger(),
                "[?] OpenCV waitKey blocked for %.1f ms; "
                "the GUI/X11 path is limiting image presentation.",
                event_processing_ms);
            last_highgui_slow_warning_at_ =
                event_processing_finished_at;
        }
    }

    void ShowReadyInterpolatedFrame(CamConfig& item) {
        cv::Mat interpolated_frame;
        pointcloud_visualization::DisDisplayTiming timing;
        if (!item.interpolator->TryGetDisplayFrame(
                interpolated_frame, &timing)) {
            return;
        }

        item.displayed_interpolated_frame =
            std::move(interpolated_frame);
        cv::imshow(
            item.interpolation_window_name,
            item.displayed_interpolated_frame);
        RecordInterpolatedFrame(
            item,
            std::chrono::steady_clock::now(),
            timing.starts_new_sequence);
    }

    void InitializeFpsWindow(
        FpsWindow& window,
        std::chrono::steady_clock::time_point now) {
        window.initialized = true;
        window.window_started_at = now;
        window.last_frame_at = now;
        window.interval_count = 0;
        window.maximum_gap_ms = 0.0;
        window.playback_interval_count = 0;
        window.playback_interval_sum_sec = 0.0;
    }

    void RecordSourceFrame(
        CamConfig& item,
        std::chrono::steady_clock::time_point now) {
        if (!fps_logging_enabled_) {
            return;
        }

        FpsWindow& window = item.source_fps;
        if (!window.initialized) {
            InitializeFpsWindow(window, now);
            return;
        }

        const double gap_sec = std::chrono::duration<double>(
            now - window.last_frame_at).count();
        window.last_frame_at = now;
        ++window.interval_count;
        window.maximum_gap_ms =
            std::max(window.maximum_gap_ms, gap_sec * 1000.0);

        const double elapsed_sec = std::chrono::duration<double>(
            now - window.window_started_at).count();
        if (elapsed_sec < fps_logging_interval_sec_) {
            return;
        }

        const double measured_fps =
            static_cast<double>(window.interval_count) / elapsed_sec;
        RCLCPP_INFO(
            this->get_logger(),
            "[FPS] Source \"%s\": %.2f FPS | %.1f s window | "
            "max gap %.0f ms | superseded meshes %llu.",
            item.name.c_str(),
            measured_fps,
            elapsed_sec,
            window.maximum_gap_ms,
            static_cast<unsigned long long>(
                superseded_mesh_count_.load()));

        InitializeFpsWindow(window, now);
    }

    void RecordInterpolatedFrame(
        CamConfig& item,
        std::chrono::steady_clock::time_point now,
        bool starts_new_sequence) {
        if (!fps_logging_enabled_) {
            return;
        }

        FpsWindow& window = item.interpolation_fps;
        if (!window.initialized) {
            InitializeFpsWindow(window, now);
            return;
        }

        const double gap_sec = std::chrono::duration<double>(
            now - window.last_frame_at).count();
        window.last_frame_at = now;
        ++window.interval_count;
        window.maximum_gap_ms =
            std::max(window.maximum_gap_ms, gap_sec * 1000.0);
        if (!starts_new_sequence) {
            ++window.playback_interval_count;
            window.playback_interval_sum_sec += gap_sec;
        }

        const double elapsed_sec = std::chrono::duration<double>(
            now - window.window_started_at).count();
        if (elapsed_sec < fps_logging_interval_sec_) {
            return;
        }

        const double effective_fps =
            static_cast<double>(window.interval_count) / elapsed_sec;
        const double playback_fps =
            window.playback_interval_count > 0 &&
                    window.playback_interval_sum_sec > 0.0 ?
                static_cast<double>(
                    window.playback_interval_count) /
                    window.playback_interval_sum_sec :
                effective_fps;
        const auto queue_stats =
            item.interpolator->GetQueueStats();
        RCLCPP_INFO(
            this->get_logger(),
            "[FPS] Interpolated \"%s\": playback %.1f FPS | "
            "effective %.1f FPS | %.1f s window | max gap %.0f ms | "
            "coalesced %llu | queue %zu pending / %zu ready.",
            item.interpolation_window_name.c_str(),
            playback_fps,
            effective_fps,
            elapsed_sec,
            window.maximum_gap_ms,
            static_cast<unsigned long long>(
                queue_stats.coalesced_source_frames),
            queue_stats.pending_pairs,
            queue_stats.ready_sequences);

        InitializeFpsWindow(window, now);
    }

    void mesh_callback(const pc_msgs::msg::O3DMesh::SharedPtr msg) {
        const auto mesh_received_at =
            std::chrono::steady_clock::now();
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

        std::optional<std::chrono::nanoseconds>
            mesh_source_timestamp;
        if (msg->header.stamp.sec != 0 ||
            msg->header.stamp.nanosec != 0U) {
            mesh_source_timestamp =
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::seconds(
                        msg->header.stamp.sec) +
                    std::chrono::nanoseconds(
                        msg->header.stamp.nanosec));
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
            latest_mesh_received_at_ = mesh_received_at;
            latest_mesh_source_timestamp_ =
                mesh_source_timestamp;
            new_mesh_available_ = true;
        }
    }

    std::vector<CamConfig> visualizers_;
    bool fps_logging_enabled_ = true;
    double fps_logging_interval_sec_ = 5.0;
    bool interpolation_enabled_ = false;
    int interpolation_intermediate_frames_ = 4;
    double interpolation_flow_scale_ = 0.25;
    int interpolation_dis_preset_ =
        cv::DISOpticalFlow::PRESET_ULTRAFAST;
    std::string interpolation_dis_preset_name_ = "ultrafast";
    bool interpolation_bidirectional_flow_ = false;
    std::string interpolation_timing_source_ = "arrival";
    bool interpolation_use_message_timestamps_ = false;
    bool interpolation_lock_camera_ = false;
    bool interpolation_show_source_window_ = false;
    std::string interpolation_window_suffix_ = " - DIS Interpolated";
    bool highgui_window_thread_started_ = false;
    std::chrono::steady_clock::time_point
        last_highgui_slow_warning_at_;

    // 线程同步
    std::mutex mutex_;
    std::shared_ptr<open3d::geometry::TriangleMesh> latest_mesh_;
    std::chrono::steady_clock::time_point latest_mesh_received_at_;
    std::optional<std::chrono::nanoseconds>
        latest_mesh_source_timestamp_;
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
