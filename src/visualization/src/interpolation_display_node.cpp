#include "visualization/display_frame_sequence.hpp"
#include "visualization/ros_image_conversion.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace {

constexpr char kInterpolatedFrameTopic[] = "interpolated_frames";
constexpr auto kNoWindowPollInterval = std::chrono::milliseconds(5);

struct DisplayStats {
    bool window_initialized = false;
    std::chrono::steady_clock::time_point window_started_at;
    bool has_last_presented_frame = false;
    std::chrono::steady_clock::time_point last_presented_at;
    std::uint64_t window_received_frames = 0;
    std::uint64_t window_presented_frames = 0;
    std::uint64_t window_overwritten_frames = 0;
    std::uint64_t window_missing_frames = 0;
    std::uint64_t window_out_of_order_frames = 0;
    std::uint64_t window_unsequenced_frames = 0;
    std::uint64_t window_rejected_frames = 0;
    std::uint64_t total_received_frames = 0;
    std::uint64_t total_presented_frames = 0;
    std::uint64_t total_overwritten_frames = 0;
    std::uint64_t total_missing_frames = 0;
    std::uint64_t total_out_of_order_frames = 0;
    std::uint64_t total_unsequenced_frames = 0;
    std::uint64_t total_rejected_frames = 0;
    std::optional<std::uint64_t> last_received_sequence;
    double maximum_display_gap_ms = 0.0;
    double maximum_imshow_ms = 0.0;
    double maximum_wait_key_ms = 0.0;
};

struct WindowState {
    std::string camera_id;
    std::string window_name;
    sensor_msgs::msg::Image::ConstSharedPtr pending_frame;
    sensor_msgs::msg::Image::ConstSharedPtr displayed_frame;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
        subscription;
    bool created = false;
    bool submitted_this_cycle = false;
    DisplayStats stats;
};

class InterpolationDisplayNode : public rclcpp::Node {
public:
    InterpolationDisplayNode()
        : Node("interpolation_display_node") {
        const auto camera_ids =
            this->declare_parameter<std::vector<std::string>>(
                "camera_ids", {"cam_front"});
        if (camera_ids.empty()) {
            throw std::invalid_argument(
                "camera_ids must contain at least one camera");
        }
        const std::string window_suffix =
            this->declare_parameter<std::string>(
                "interpolation_window_suffix",
                " - DIS Interpolated");
        fps_logging_enabled_ =
            this->declare_parameter<bool>(
                "fps_logging_enabled", true);
        fps_logging_interval_sec_ =
            this->declare_parameter<double>(
                "fps_logging_interval_sec", 5.0);
        if (fps_logging_enabled_ &&
            (!std::isfinite(fps_logging_interval_sec_) ||
             fps_logging_interval_sec_ <= 0.0)) {
            throw std::invalid_argument(
                "fps_logging_interval_sec must be greater than zero");
        }

        windows_.reserve(camera_ids.size());
        std::unordered_set<std::string> unique_camera_ids;
        std::unordered_set<std::string> unique_window_names;
        for (const auto& camera_id : camera_ids) {
            if (camera_id.empty() ||
                !unique_camera_ids.insert(camera_id).second) {
                throw std::invalid_argument(
                    "camera_ids must be non-empty and unique");
            }

            const std::string source_window_name =
                this->declare_parameter<std::string>(
                    camera_id + ".name",
                    "Render - " + camera_id);
            const std::string interpolation_window_name =
                source_window_name + window_suffix;
            if (source_window_name.empty() ||
                !unique_window_names.insert(
                    interpolation_window_name).second) {
                throw std::invalid_argument(
                    "interpolation window names must be non-empty "
                    "and unique");
            }

            WindowState state;
            state.camera_id = camera_id;
            state.window_name =
                interpolation_window_name;
            windows_.push_back(std::move(state));
        }

        rclcpp::QoS image_qos(rclcpp::KeepLast(1));
        image_qos.best_effort();
        for (std::size_t index = 0;
             index < windows_.size();
             ++index) {
            WindowState& state = windows_.at(index);
            const std::string input_topic =
                std::string(kInterpolatedFrameTopic) +
                "/" + state.camera_id;
            state.subscription =
                this->create_subscription<
                    sensor_msgs::msg::Image>(
                    input_topic,
                    image_qos,
                    [this, index](
                        sensor_msgs::msg::Image::ConstSharedPtr message) {
                        if (message == nullptr) {
                            return;
                        }
                        WindowState& state = windows_.at(index);
                        RecordReceivedFrame(
                            state,
                            *message,
                            std::chrono::steady_clock::now());
                        state.pending_frame = std::move(message);
                    });

            RCLCPP_INFO(
                this->get_logger(),
                "[*] Interpolation display \"%s\" is listening on "
                "\"%s\".",
                state.window_name.c_str(),
                input_topic.c_str());
        }
        RCLCPP_INFO(
            this->get_logger(),
            "[*] All HighGUI calls run on this process's main thread.");
        if (fps_logging_enabled_) {
            RCLCPP_INFO(
                this->get_logger(),
                "[FPS] Display-side logging enabled with a %.1f s "
                "measurement window.",
                fps_logging_interval_sec_);
        }
    }

    ~InterpolationDisplayNode() override {
        DestroyWindows();
    }

    bool PresentReadyFramesAndProcessEvents() {
        bool has_created_window = false;
        for (auto& state : windows_) {
            state.submitted_this_cycle = false;
            if (state.pending_frame == nullptr) {
                has_created_window =
                    has_created_window || state.created;
                continue;
            }

            state.displayed_frame =
                std::move(state.pending_frame);
            try {
                const cv::Mat frame =
                    pointcloud_visualization::
                        BgrImageMessageView(
                            *state.displayed_frame);
                if (!state.created) {
                    cv::namedWindow(
                        state.window_name,
                        cv::WINDOW_NORMAL);
                    // Record ownership immediately: if resizeWindow throws,
                    // shutdown must still destroy the window just created.
                    state.created = true;
                    cv::resizeWindow(
                        state.window_name,
                        frame.cols,
                        frame.rows);
                }
                const auto imshow_started_at =
                    std::chrono::steady_clock::now();
                cv::imshow(state.window_name, frame);
                const auto imshow_finished_at =
                    std::chrono::steady_clock::now();
                state.stats.maximum_imshow_ms = std::max(
                    state.stats.maximum_imshow_ms,
                    std::chrono::duration<double, std::milli>(
                        imshow_finished_at -
                        imshow_started_at).count());
                state.submitted_this_cycle = true;
            } catch (const std::exception& error) {
                RecordRejectedFrame(state);
                RCLCPP_ERROR(
                    this->get_logger(),
                    "[!] Rejected interpolated frame for \"%s\": %s",
                    state.window_name.c_str(),
                    error.what());
            }
            has_created_window =
                has_created_window || state.created;
        }

        if (!has_created_window) {
            // waitKey does not provide a portable delay until at least one
            // HighGUI window exists, so explicitly throttle the no-frame
            // startup path.
            std::this_thread::sleep_for(kNoWindowPollInterval);
            const auto now = std::chrono::steady_clock::now();
            for (auto& state : windows_) {
                ReportDisplayStatsIfDue(state, now);
            }
            return true;
        }

        const auto wait_key_started_at =
            std::chrono::steady_clock::now();
        const int key = cv::waitKey(1);
        const auto wait_key_finished_at =
            std::chrono::steady_clock::now();
        const double wait_key_ms =
            std::chrono::duration<double, std::milli>(
                wait_key_finished_at -
                wait_key_started_at).count();
        for (auto& state : windows_) {
            if (state.created) {
                state.stats.maximum_wait_key_ms = std::max(
                    state.stats.maximum_wait_key_ms,
                    wait_key_ms);
            }
            if (state.submitted_this_cycle) {
                RecordPresentedFrame(
                    state, wait_key_finished_at);
            }
            ReportDisplayStatsIfDue(
                state, wait_key_finished_at);
        }
        return key != 27 && key != 'q' && key != 'Q';
    }

    void DestroyWindows() noexcept {
        for (auto& state : windows_) {
            if (!state.created) {
                continue;
            }
            try {
                cv::destroyWindow(state.window_name);
            } catch (const std::exception& error) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "[?] Failed to destroy interpolation window "
                    "\"%s\": %s",
                    state.window_name.c_str(),
                    error.what());
            } catch (...) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "[?] Failed to destroy interpolation window "
                    "\"%s\" with an unknown exception.",
                    state.window_name.c_str());
            }
            state.created = false;
        }
    }

private:
    void InitializeStatsWindow(
        DisplayStats& stats,
        std::chrono::steady_clock::time_point now) {
        stats.window_initialized = true;
        stats.window_started_at = now;
    }

    void RecordReceivedFrame(
        WindowState& state,
        const sensor_msgs::msg::Image& message,
        std::chrono::steady_clock::time_point now) {
        if (!fps_logging_enabled_) {
            return;
        }

        DisplayStats& stats = state.stats;
        if (!stats.window_initialized) {
            InitializeStatsWindow(stats, now);
        }
        ++stats.window_received_frames;
        ++stats.total_received_frames;

        if (state.pending_frame != nullptr) {
            ++stats.window_overwritten_frames;
            ++stats.total_overwritten_frames;
        }

        const auto sequence =
            pointcloud_visualization::
                DecodeDisplayFrameSequence(
                    message.header.frame_id);
        if (!sequence.has_value()) {
            ++stats.window_unsequenced_frames;
            ++stats.total_unsequenced_frames;
            return;
        }
        if (!stats.last_received_sequence.has_value()) {
            stats.last_received_sequence = *sequence;
            return;
        }

        const std::uint64_t previous_sequence =
            *stats.last_received_sequence;
        if (*sequence > previous_sequence) {
            const std::uint64_t missing_frames =
                *sequence - previous_sequence - 1;
            stats.window_missing_frames += missing_frames;
            stats.total_missing_frames += missing_frames;
            stats.last_received_sequence = *sequence;
        } else {
            ++stats.window_out_of_order_frames;
            ++stats.total_out_of_order_frames;
        }
    }

    void RecordRejectedFrame(WindowState& state) {
        if (!fps_logging_enabled_) {
            return;
        }
        ++state.stats.window_rejected_frames;
        ++state.stats.total_rejected_frames;
    }

    void RecordPresentedFrame(
        WindowState& state,
        std::chrono::steady_clock::time_point now) {
        if (!fps_logging_enabled_) {
            return;
        }

        DisplayStats& stats = state.stats;
        if (!stats.window_initialized) {
            InitializeStatsWindow(stats, now);
        }
        if (stats.has_last_presented_frame) {
            const double display_gap_ms =
                std::chrono::duration<double, std::milli>(
                    now - stats.last_presented_at).count();
            stats.maximum_display_gap_ms = std::max(
                stats.maximum_display_gap_ms,
                display_gap_ms);
        }
        stats.has_last_presented_frame = true;
        stats.last_presented_at = now;
        ++stats.window_presented_frames;
        ++stats.total_presented_frames;
    }

    void ReportDisplayStatsIfDue(
        WindowState& state,
        std::chrono::steady_clock::time_point now) {
        if (!fps_logging_enabled_ ||
            !state.stats.window_initialized) {
            return;
        }

        DisplayStats& stats = state.stats;
        const double elapsed_sec =
            std::chrono::duration<double>(
                now - stats.window_started_at).count();
        if (elapsed_sec < fps_logging_interval_sec_) {
            return;
        }

        const double received_fps =
            static_cast<double>(
                stats.window_received_frames) /
            elapsed_sec;
        const double presented_fps =
            static_cast<double>(
                stats.window_presented_frames) /
            elapsed_sec;
        RCLCPP_INFO(
            this->get_logger(),
            "[FPS] Display \"%s\": received %.1f FPS "
            "(+%llu, total %llu) | presented %.1f FPS "
            "(+%llu, total %llu) | %.1f s window | "
            "max display gap %.0f ms | HighGUI max imshow %.1f ms / "
            "waitKey %.1f ms | overwritten +%llu (total %llu) | "
            "missing +%llu (total %llu) | out-of-order +%llu "
            "(total %llu) | unsequenced +%llu (total %llu) | "
            "rejected +%llu (total %llu) | pending %d.",
            state.window_name.c_str(),
            received_fps,
            static_cast<unsigned long long>(
                stats.window_received_frames),
            static_cast<unsigned long long>(
                stats.total_received_frames),
            presented_fps,
            static_cast<unsigned long long>(
                stats.window_presented_frames),
            static_cast<unsigned long long>(
                stats.total_presented_frames),
            elapsed_sec,
            stats.maximum_display_gap_ms,
            stats.maximum_imshow_ms,
            stats.maximum_wait_key_ms,
            static_cast<unsigned long long>(
                stats.window_overwritten_frames),
            static_cast<unsigned long long>(
                stats.total_overwritten_frames),
            static_cast<unsigned long long>(
                stats.window_missing_frames),
            static_cast<unsigned long long>(
                stats.total_missing_frames),
            static_cast<unsigned long long>(
                stats.window_out_of_order_frames),
            static_cast<unsigned long long>(
                stats.total_out_of_order_frames),
            static_cast<unsigned long long>(
                stats.window_unsequenced_frames),
            static_cast<unsigned long long>(
                stats.total_unsequenced_frames),
            static_cast<unsigned long long>(
                stats.window_rejected_frames),
            static_cast<unsigned long long>(
                stats.total_rejected_frames),
            state.pending_frame != nullptr ? 1 : 0);

        stats.window_started_at = now;
        stats.window_received_frames = 0;
        stats.window_presented_frames = 0;
        stats.window_overwritten_frames = 0;
        stats.window_missing_frames = 0;
        stats.window_out_of_order_frames = 0;
        stats.window_unsequenced_frames = 0;
        stats.window_rejected_frames = 0;
        stats.maximum_display_gap_ms = 0.0;
        stats.maximum_imshow_ms = 0.0;
        stats.maximum_wait_key_ms = 0.0;
    }

    std::vector<WindowState> windows_;
    bool fps_logging_enabled_ = true;
    double fps_logging_interval_sec_ = 5.0;
};

}  // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    int exit_code = 0;
    std::shared_ptr<InterpolationDisplayNode> node;

    try {
        node = std::make_shared<InterpolationDisplayNode>();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);

        while (rclcpp::ok()) {
            // spin_some invokes every image callback on this main thread.
            executor.spin_some();
            if (!node->PresentReadyFramesAndProcessEvents()) {
                RCLCPP_INFO(
                    node->get_logger(),
                    "[*] Interpolation display requested shutdown.");
                break;
            }
        }

        executor.remove_node(node);
        node->DestroyWindows();
    } catch (const std::exception& error) {
        RCLCPP_FATAL(
            rclcpp::get_logger("interpolation_display_node"),
            "[!] Interpolation display failed: %s",
            error.what());
        exit_code = 1;
    } catch (...) {
        RCLCPP_FATAL(
            rclcpp::get_logger("interpolation_display_node"),
            "[!] Interpolation display failed with an unknown exception.");
        exit_code = 1;
    }

    node.reset();
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return exit_code;
}
