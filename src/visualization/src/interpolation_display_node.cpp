#include "visualization/display_frame_sequence.hpp"
#include "visualization/dis_frame_interpolator.hpp"
#include "visualization/ros_image_conversion.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include <opencv2/core/version.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace {

constexpr char kSourceFrameTopic[] = "interpolation_source_frames";
constexpr auto kHighGuiPollInterval = std::chrono::milliseconds(5);
constexpr double kSlowOperationThresholdMs = 100.0;
constexpr std::int64_t kMaximumQueueCapacity = 1000;

std::string Lowercase(std::string value) {
    std::transform(
        value.begin(), value.end(), value.begin(),
        [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    return value;
}

int ParseDisPreset(const std::string& configured_preset) {
    const std::string preset = Lowercase(configured_preset);
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

std::optional<std::chrono::nanoseconds> MessageTimestamp(
    const sensor_msgs::msg::Image& message) {
    const std::int64_t timestamp_ns =
        rclcpp::Time(message.header.stamp).nanoseconds();
    if (timestamp_ns <= 0) {
        return std::nullopt;
    }
    return std::chrono::nanoseconds(timestamp_ns);
}

double Average(double sum, std::uint64_t count) {
    return count > 0 ? sum / static_cast<double>(count) : 0.0;
}

struct DisplayStats {
    bool window_initialized = false;
    std::chrono::steady_clock::time_point window_started_at;
    bool has_last_presented_frame = false;
    std::chrono::steady_clock::time_point last_presented_at;

    std::uint64_t window_received_source_frames = 0;
    std::uint64_t window_presented_frames = 0;
    std::uint64_t window_missing_source_frames = 0;
    std::uint64_t window_out_of_order_source_frames = 0;
    std::uint64_t window_unsequenced_source_frames = 0;
    std::uint64_t window_display_sequence_breaks = 0;
    std::uint64_t window_rejected_source_frames = 0;
    std::uint64_t window_display_errors = 0;
    std::uint64_t total_received_source_frames = 0;
    std::uint64_t total_presented_frames = 0;
    std::uint64_t total_missing_source_frames = 0;
    std::uint64_t total_out_of_order_source_frames = 0;
    std::uint64_t total_unsequenced_source_frames = 0;
    std::uint64_t total_display_sequence_breaks = 0;
    std::uint64_t total_rejected_source_frames = 0;
    std::uint64_t total_display_errors = 0;
    std::optional<std::uint64_t> last_received_source_sequence;
    std::optional<std::uint64_t> last_completed_source_sequence;
    std::size_t expected_active_frame_index = 0;

    double maximum_display_gap_ms = 0.0;

    std::uint64_t window_imshow_samples = 0;
    double window_imshow_sum_ms = 0.0;
    double maximum_imshow_ms = 0.0;
    std::uint64_t window_slow_imshow_calls = 0;

    std::uint64_t window_event_samples = 0;
    double window_event_sum_ms = 0.0;
    double maximum_event_ms = 0.0;
    std::uint64_t window_slow_event_calls = 0;

    std::uint64_t window_submit_samples = 0;
    double window_submit_sum_ms = 0.0;
    double maximum_submit_ms = 0.0;

    std::uint64_t window_transport_samples = 0;
    double window_transport_sum_ms = 0.0;
    double maximum_transport_ms = 0.0;

    std::uint64_t window_source_age_samples = 0;
    double window_source_age_sum_ms = 0.0;
    double maximum_source_age_ms = 0.0;

    std::uint64_t last_reported_source_backpressure_waits = 0;
    double last_reported_source_backpressure_wait_ms = 0.0;
};

struct WindowState {
    std::string camera_id;
    std::string window_name;
    cv::Scalar border_color_bgr;
    std::unique_ptr<pointcloud_visualization::DisFrameInterpolator>
        interpolator;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
        subscription;
    cv::Mat displayed_frame;
    pointcloud_visualization::DisDisplayTiming submitted_timing;
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
        const std::int64_t intermediate_frame_count =
            this->declare_parameter<std::int64_t>(
                "interpolation_intermediate_frames", 4);
        const double flow_scale =
            this->declare_parameter<double>(
                "interpolation_flow_scale", 0.25);
        const std::string dis_preset_name =
            this->declare_parameter<std::string>(
                "interpolation_dis_preset", "ultrafast");
        const bool use_bidirectional_flow =
            this->declare_parameter<bool>(
                "interpolation_bidirectional_flow", true);
        const bool use_flow_consistency_mask =
            this->declare_parameter<bool>(
                "interpolation_flow_consistency_mask", true);
        const std::string timing_source = Lowercase(
            this->declare_parameter<std::string>(
                "interpolation_timing_source", "message_stamp"));
        const std::int64_t pending_pair_capacity =
            this->declare_parameter<std::int64_t>(
                "interpolation_pending_pair_capacity", 3);
        const std::int64_t ready_sequence_capacity =
            this->declare_parameter<std::int64_t>(
                "interpolation_ready_sequence_capacity", 2);
        configured_highgui_event_mode_ = Lowercase(
            this->declare_parameter<std::string>(
                "interpolation_highgui_event_mode", "wait_key"));
        use_wait_key_event_pump_ =
            configured_highgui_event_mode_ == "wait_key";

        if (fps_logging_enabled_ &&
            (!std::isfinite(fps_logging_interval_sec_) ||
             fps_logging_interval_sec_ <= 0.0)) {
            throw std::invalid_argument(
                "fps_logging_interval_sec must be greater than zero");
        }
        if (intermediate_frame_count < 0 ||
            intermediate_frame_count > 120) {
            throw std::invalid_argument(
                "interpolation_intermediate_frames must be in [0, 120]");
        }
        if (!std::isfinite(flow_scale) ||
            flow_scale <= 0.0 ||
            flow_scale > 1.0) {
            throw std::invalid_argument(
                "interpolation_flow_scale must be in (0, 1]");
        }
        if (use_flow_consistency_mask &&
            !use_bidirectional_flow) {
            throw std::invalid_argument(
                "interpolation_flow_consistency_mask requires "
                "interpolation_bidirectional_flow");
        }
        if (timing_source != "arrival" &&
            timing_source != "message_stamp") {
            throw std::invalid_argument(
                "interpolation_timing_source must be arrival "
                "or message_stamp");
        }
        if (pending_pair_capacity <= 0 ||
            pending_pair_capacity > kMaximumQueueCapacity) {
            throw std::invalid_argument(
                "interpolation_pending_pair_capacity must be in [1, 1000]");
        }
        if (ready_sequence_capacity <= 0 ||
            ready_sequence_capacity > kMaximumQueueCapacity) {
            throw std::invalid_argument(
                "interpolation_ready_sequence_capacity must be in [1, 1000]");
        }
        if (configured_highgui_event_mode_ != "wait_key" &&
            configured_highgui_event_mode_ != "poll_key" &&
            configured_highgui_event_mode_ !=
                "start_window_thread") {
            throw std::invalid_argument(
                "interpolation_highgui_event_mode must be wait_key, "
                "poll_key, or start_window_thread");
        }

        intermediate_frame_count_ =
            static_cast<std::size_t>(intermediate_frame_count);
        pending_pair_capacity_ =
            static_cast<std::size_t>(pending_pair_capacity);
        ready_sequence_capacity_ =
            static_cast<std::size_t>(ready_sequence_capacity);
        const int dis_preset = ParseDisPreset(dis_preset_name);
        const bool use_message_timestamps =
            timing_source == "message_stamp";

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
            const std::string window_name =
                source_window_name + window_suffix;
            if (source_window_name.empty() ||
                !unique_window_names.insert(window_name).second) {
                throw std::invalid_argument(
                    "interpolation window names must be non-empty "
                    "and unique");
            }

            const auto background_color =
                this->declare_parameter<std::vector<double>>(
                    camera_id + ".background_color",
                    {0.1, 0.1, 0.1});
            if (background_color.size() != 3 ||
                !std::all_of(
                    background_color.begin(),
                    background_color.end(),
                    [](double value) {
                        return std::isfinite(value) &&
                               value >= 0.0 && value <= 1.0;
                    })) {
                throw std::invalid_argument(
                    camera_id +
                    ".background_color must contain three finite "
                    "values in [0, 1]");
            }

            WindowState state;
            state.camera_id = camera_id;
            state.window_name = window_name;
            state.border_color_bgr = cv::Scalar(
                background_color.at(2) * 255.0,
                background_color.at(1) * 255.0,
                background_color.at(0) * 255.0);
            windows_.push_back(std::move(state));
        }

        rclcpp::QoS image_qos{rclcpp::KeepAll()};
        image_qos.reliable();
        for (std::size_t index = 0;
             index < windows_.size();
             ++index) {
            WindowState& state = windows_.at(index);

            pointcloud_visualization::DisInterpolationConfig config;
            config.intermediate_frame_count =
                intermediate_frame_count_;
            config.flow_scale = flow_scale;
            config.dis_preset = dis_preset;
            config.use_bidirectional_flow =
                use_bidirectional_flow;
            config.use_flow_consistency_mask =
                use_flow_consistency_mask;
            config.use_source_timestamps =
                use_message_timestamps;
            config.border_color_bgr = state.border_color_bgr;
            config.max_pending_pairs = pending_pair_capacity_;
            config.max_ready_sequences = ready_sequence_capacity_;
            state.interpolator = std::make_unique<
                pointcloud_visualization::DisFrameInterpolator>(
                config,
                [this]() {
                    WakeDisplayThread();
                });

            const std::string input_topic =
                std::string(kSourceFrameTopic) +
                "/" + state.camera_id;
            state.subscription =
                this->create_subscription<
                    sensor_msgs::msg::Image>(
                    input_topic,
                    image_qos,
                    [this, index](
                        sensor_msgs::msg::Image::ConstSharedPtr message) {
                        if (message != nullptr) {
                            ReceiveSourceFrame(
                                index, std::move(message));
                        }
                    });

            RCLCPP_INFO(
                this->get_logger(),
                "[*] Display \"%s\" receives source frames from "
                "\"%s\" with reliable KeepAll QoS.",
                state.window_name.c_str(),
                input_topic.c_str());
        }

        RCLCPP_INFO(
            this->get_logger(),
            "[*] Local %s DIS interpolation: %zu intermediate "
            "frames (%zux source frame count), scale %.2f, preset %s, "
            "timing %s.",
            use_bidirectional_flow ?
                "bidirectional" : "single-direction",
            intermediate_frame_count_,
            intermediate_frame_count_ + 1,
            flow_scale,
            dis_preset_name.c_str(),
            timing_source.c_str());
        RCLCPP_INFO(
            this->get_logger(),
            "[*] Ordered queues: %zu pending pairs / %zu ready "
            "sequences. Full queues block the source callback; no source "
            "pair or generated display frame is coalesced or skipped.",
            pending_pair_capacity_,
            ready_sequence_capacity_);
        RCLCPP_INFO(
            this->get_logger(),
            "[*] HighGUI requested mode is %s with OpenCV %s. The "
            "active event-pump mode is logged after the first window "
            "is created.",
            configured_highgui_event_mode_.c_str(),
            CV_VERSION);
        if (fps_logging_enabled_) {
            RCLCPP_INFO(
                this->get_logger(),
                "[FPS] Display diagnostics use a %.1f s window. "
                "[FPS] reports the application display boundary named "
                "in each line; [PIPE] separates DDS, interpolation "
                "queues, source age, and HighGUI cost.",
                fps_logging_interval_sec_);
        }
    }

    ~InterpolationDisplayNode() override {
        RequestStop();
        DestroyWindows();
        for (auto& state : windows_) {
            state.interpolator.reset();
        }
    }

    void RequestStop() noexcept {
        stopping_.store(true);
        for (auto& state : windows_) {
            if (state.interpolator != nullptr) {
                state.interpolator->RequestStop();
            }
        }
        display_condition_.notify_all();
    }

    bool StopRequested() const noexcept {
        return stopping_.load();
    }

    bool PresentReadyFramesAndProcessEvents() {
        bool has_created_window = false;
        double slowest_imshow_ms = 0.0;

        for (auto& state : windows_) {
            state.submitted_this_cycle = false;
            ReportInterpolatorMessages(state);

            cv::Mat next_frame;
            pointcloud_visualization::DisDisplayTiming timing;
            if (!state.interpolator->TryGetDisplayFrame(
                    next_frame, &timing)) {
                has_created_window =
                    has_created_window || state.created;
                continue;
            }

            try {
                if (!state.created) {
                    cv::namedWindow(
                        state.window_name,
                        cv::WINDOW_NORMAL);
                    state.created = true;
                    cv::resizeWindow(
                        state.window_name,
                        next_frame.cols,
                        next_frame.rows);
                    InitializeHighGuiBackend();
                }

                state.displayed_frame = std::move(next_frame);
                const auto imshow_started_at =
                    std::chrono::steady_clock::now();
                cv::imshow(
                    state.window_name,
                    state.displayed_frame);
                const auto imshow_finished_at =
                    std::chrono::steady_clock::now();
                const double imshow_ms =
                    std::chrono::duration<double, std::milli>(
                        imshow_finished_at -
                        imshow_started_at).count();
                RecordImshowDuration(state, imshow_ms);
                slowest_imshow_ms =
                    std::max(slowest_imshow_ms, imshow_ms);
                state.submitted_timing = timing;
                state.submitted_this_cycle = true;
            } catch (const std::exception& error) {
                RecordDisplayError(state);
                RCLCPP_ERROR(
                    this->get_logger(),
                    "[!] Failed to submit display frame for \"%s\": %s",
                    state.window_name.c_str(),
                    error.what());
            }
            has_created_window =
                has_created_window || state.created;
        }

        int key = -1;
        double event_ms = 0.0;
        auto presentation_boundary =
            std::chrono::steady_clock::now();
        if (has_created_window &&
            !highgui_event_thread_started_) {
            const auto event_started_at =
                std::chrono::steady_clock::now();
            if (use_wait_key_event_pump_) {
                key = cv::waitKey(1);
            } else {
#if CV_VERSION_MAJOR > 4 || \
    (CV_VERSION_MAJOR == 4 && \
     (CV_VERSION_MINOR > 5 || \
      (CV_VERSION_MINOR == 5 && CV_VERSION_REVISION >= 1)))
                key = cv::pollKey();
#else
                key = cv::waitKey(1);
#endif
            }
            presentation_boundary =
                std::chrono::steady_clock::now();
            event_ms =
                std::chrono::duration<double, std::milli>(
                    presentation_boundary -
                    event_started_at).count();
            for (auto& state : windows_) {
                if (state.created) {
                    RecordEventDuration(state, event_ms);
                }
            }
        }

        for (auto& state : windows_) {
            if (state.submitted_this_cycle) {
                RecordPresentedFrame(
                    state,
                    state.submitted_timing,
                    presentation_boundary);
            }
            ReportDisplayStatsIfDue(
                state, presentation_boundary);
            ReportInterpolatorMessages(state);
        }

        WarnIfHighGuiIsSlow(
            slowest_imshow_ms,
            event_ms,
            presentation_boundary);
        if (key == 27 || key == 'q' || key == 'Q') {
            return false;
        }

        const auto wait_duration =
            CalculateNextWaitDuration(
                std::chrono::steady_clock::now());
        if (wait_duration >
            std::chrono::steady_clock::duration::zero()) {
            WaitForDisplayWork(wait_duration);
        }
        return !StopRequested();
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
    void WakeDisplayThread() {
        {
            std::lock_guard<std::mutex> lock(display_mutex_);
            ++display_wakeup_generation_;
        }
        display_condition_.notify_one();
    }

    void ReceiveSourceFrame(
        std::size_t index,
        sensor_msgs::msg::Image::ConstSharedPtr message) {
        const auto received_at =
            std::chrono::steady_clock::now();
        WindowState& state = windows_.at(index);
        const auto sequence =
            pointcloud_visualization::DecodeDisplayFrameSequence(
                message->header.frame_id);

        std::optional<double> transport_ms;
        const auto source_timestamp = MessageTimestamp(*message);
        if (source_timestamp.has_value()) {
            const std::int64_t now_ns =
                this->get_clock()->now().nanoseconds();
            const std::int64_t latency_ns =
                now_ns - source_timestamp->count();
            if (latency_ns >= 0) {
                transport_ms =
                    static_cast<double>(latency_ns) / 1.0e6;
            }
        }

        bool accept_frame = true;
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            DisplayStats& stats = state.stats;
            InitializeStatsWindowIfNeeded(stats, received_at);
            ++stats.window_received_source_frames;
            ++stats.total_received_source_frames;

            if (!sequence.has_value()) {
                ++stats.window_unsequenced_source_frames;
                ++stats.total_unsequenced_source_frames;
            } else if (
                !stats.last_received_source_sequence.has_value()) {
                stats.last_received_source_sequence = sequence;
            } else {
                const std::uint64_t previous_sequence =
                    *stats.last_received_source_sequence;
                if (*sequence > previous_sequence) {
                    const std::uint64_t missing_frames =
                        *sequence - previous_sequence - 1;
                    stats.window_missing_source_frames +=
                        missing_frames;
                    stats.total_missing_source_frames +=
                        missing_frames;
                    stats.last_received_source_sequence =
                        sequence;
                } else {
                    ++stats.window_out_of_order_source_frames;
                    ++stats.total_out_of_order_source_frames;
                    ++stats.window_rejected_source_frames;
                    ++stats.total_rejected_source_frames;
                    accept_frame = false;
                }
            }

            if (transport_ms.has_value()) {
                ++stats.window_transport_samples;
                stats.window_transport_sum_ms += *transport_ms;
                stats.maximum_transport_ms = std::max(
                    stats.maximum_transport_ms,
                    *transport_ms);
            }
        }

        if (!accept_frame || StopRequested()) {
            return;
        }

        try {
            const cv::Mat source_frame =
                pointcloud_visualization::BgrImageMessageView(
                    *message);
            const auto submit_started_at =
                std::chrono::steady_clock::now();
            state.interpolator->SubmitFrame(
                source_frame,
                received_at,
                source_timestamp,
                sequence);
            const double submit_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() -
                    submit_started_at).count();
            RecordSubmitDuration(state, submit_ms);
            if (submit_ms > kSlowOperationThresholdMs &&
                !StopRequested()) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "[FLOW] Source callback for \"%s\" spent %.1f ms "
                    "inside ordered interpolation submission. This is "
                    "intentional backpressure; frames were not dropped.",
                    state.window_name.c_str(),
                    submit_ms);
            }
        } catch (const std::exception& error) {
            RecordRejectedSourceFrame(state);
            RCLCPP_ERROR(
                this->get_logger(),
                "[!] Rejected source frame for \"%s\": %s",
                state.window_name.c_str(),
                error.what());
        }
    }

    void InitializeHighGuiBackend() {
        if (highgui_backend_initialized_) {
            return;
        }
        highgui_backend_initialized_ = true;

        if (configured_highgui_event_mode_ ==
            "start_window_thread") {
            highgui_event_thread_started_ =
                cv::startWindowThread() > 0;
            if (highgui_event_thread_started_) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "[*] HighGUI backend started its own event thread. "
                    "The FPS boundary is now imshow submission because "
                    "HighGUI provides no event-thread presentation "
                    "completion callback.");
                return;
            }
            RCLCPP_WARN(
                this->get_logger(),
                "[?] HighGUI backend does not support an event thread; "
                "falling back to measured main-thread waitKey(1).");
            use_wait_key_event_pump_ = true;
        }

        if (use_wait_key_event_pump_) {
            RCLCPP_INFO(
                this->get_logger(),
                "[*] HighGUI events are pumped with waitKey(1) on the "
                "display main thread. This avoids the long pollKey() "
                "cycle observed in the current runtime while preserving "
                "a measured display boundary.");
            return;
        }

#if CV_VERSION_MAJOR > 4 || \
    (CV_VERSION_MAJOR == 4 && \
     (CV_VERSION_MINOR > 5 || \
      (CV_VERSION_MINOR == 5 && CV_VERSION_REVISION >= 1)))
        RCLCPP_INFO(
            this->get_logger(),
            "[*] HighGUI events are pumped with pollKey() on the "
            "display main thread.");
#else
        RCLCPP_WARN(
            this->get_logger(),
            "[?] OpenCV predates pollKey(); HighGUI events use the "
            "measured waitKey(1) fallback.");
        use_wait_key_event_pump_ = true;
#endif
    }

    const char* ActiveHighGuiEventMode() const noexcept {
        if (highgui_event_thread_started_) {
            return "window-thread";
        }
        return use_wait_key_event_pump_ ?
            "main-thread-waitKey(1)" :
            "main-thread-pollKey()";
    }

    const char* ActiveDisplayBoundary() const noexcept {
        return highgui_event_thread_started_ ?
            "imshow-submit(window-thread)" :
            "imshow+event-pump";
    }

    std::chrono::steady_clock::duration CalculateNextWaitDuration(
        std::chrono::steady_clock::time_point now) {
        auto wait_duration =
            std::chrono::duration_cast<
                std::chrono::steady_clock::duration>(
                kHighGuiPollInterval);

        for (const auto& state : windows_) {
            const auto deadline =
                state.interpolator->GetNextDisplayDeadline();
            if (!deadline.has_value()) {
                continue;
            }
            if (*deadline <= now) {
                return std::chrono::steady_clock::duration::zero();
            }
            wait_duration = std::min(
                wait_duration, *deadline - now);
        }
        return wait_duration;
    }

    void WaitForDisplayWork(
        std::chrono::steady_clock::duration timeout) {
        std::unique_lock<std::mutex> lock(display_mutex_);
        const std::uint64_t observed_generation =
            display_wakeup_generation_;
        display_condition_.wait_for(
            lock,
            timeout,
            [this, observed_generation]() {
                return StopRequested() ||
                       display_wakeup_generation_ !=
                           observed_generation;
            });
    }

    void ReportInterpolatorMessages(WindowState& state) {
        const std::string status =
            state.interpolator->ConsumeStatus();
        if (!status.empty()) {
            RCLCPP_INFO(
                this->get_logger(),
                "[GEN] \"%s\": %s",
                state.window_name.c_str(),
                status.c_str());
        }

        const std::string error =
            state.interpolator->ConsumeError();
        if (!error.empty()) {
            RCLCPP_WARN(
                this->get_logger(),
                "[?] \"%s\": %s",
                state.window_name.c_str(),
                error.c_str());
        }
    }

    void InitializeStatsWindowIfNeeded(
        DisplayStats& stats,
        std::chrono::steady_clock::time_point now) {
        if (stats.window_initialized) {
            return;
        }
        stats.window_initialized = true;
        stats.window_started_at = now;
    }

    void RecordImshowDuration(
        WindowState& state,
        double duration_ms) {
        if (!fps_logging_enabled_) {
            return;
        }
        std::lock_guard<std::mutex> lock(stats_mutex_);
        DisplayStats& stats = state.stats;
        ++stats.window_imshow_samples;
        stats.window_imshow_sum_ms += duration_ms;
        stats.maximum_imshow_ms = std::max(
            stats.maximum_imshow_ms, duration_ms);
        if (duration_ms > kSlowOperationThresholdMs) {
            ++stats.window_slow_imshow_calls;
        }
    }

    void RecordEventDuration(
        WindowState& state,
        double duration_ms) {
        if (!fps_logging_enabled_) {
            return;
        }
        std::lock_guard<std::mutex> lock(stats_mutex_);
        DisplayStats& stats = state.stats;
        ++stats.window_event_samples;
        stats.window_event_sum_ms += duration_ms;
        stats.maximum_event_ms = std::max(
            stats.maximum_event_ms, duration_ms);
        if (duration_ms > kSlowOperationThresholdMs) {
            ++stats.window_slow_event_calls;
        }
    }

    void RecordSubmitDuration(
        WindowState& state,
        double duration_ms) {
        if (!fps_logging_enabled_) {
            return;
        }
        std::lock_guard<std::mutex> lock(stats_mutex_);
        DisplayStats& stats = state.stats;
        ++stats.window_submit_samples;
        stats.window_submit_sum_ms += duration_ms;
        stats.maximum_submit_ms = std::max(
            stats.maximum_submit_ms, duration_ms);
    }

    void RecordRejectedSourceFrame(WindowState& state) {
        if (!fps_logging_enabled_) {
            return;
        }
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++state.stats.window_rejected_source_frames;
        ++state.stats.total_rejected_source_frames;
    }

    void RecordDisplayError(WindowState& state) {
        if (!fps_logging_enabled_) {
            return;
        }
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++state.stats.window_display_errors;
        ++state.stats.total_display_errors;
    }

    void RecordPresentedFrame(
        WindowState& state,
        const pointcloud_visualization::DisDisplayTiming& timing,
        std::chrono::steady_clock::time_point now) {
        if (!fps_logging_enabled_) {
            return;
        }

        std::lock_guard<std::mutex> lock(stats_mutex_);
        DisplayStats& stats = state.stats;
        InitializeStatsWindowIfNeeded(stats, now);
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

        bool sequence_break = false;
        if (timing.starts_new_sequence) {
            if (timing.frame_index != 1) {
                sequence_break = true;
            }
            if (timing.source_sequence_start.has_value() &&
                stats.last_completed_source_sequence.has_value() &&
                *timing.source_sequence_start !=
                    *stats.last_completed_source_sequence) {
                sequence_break = true;
            }
            stats.expected_active_frame_index =
                timing.frame_index + 1;
        } else {
            if (stats.expected_active_frame_index == 0 ||
                timing.frame_index !=
                    stats.expected_active_frame_index) {
                sequence_break = true;
            }
            stats.expected_active_frame_index =
                timing.frame_index + 1;
        }
        if (sequence_break) {
            ++stats.window_display_sequence_breaks;
            ++stats.total_display_sequence_breaks;
        }
        if (timing.frame_count > 0 &&
            timing.frame_index == timing.frame_count) {
            stats.expected_active_frame_index = 0;
            if (timing.source_sequence_end.has_value()) {
                stats.last_completed_source_sequence =
                    timing.source_sequence_end;
            }
        }

        if (timing.newest_source_arrival_time.has_value() &&
            now >= *timing.newest_source_arrival_time) {
            const double source_age_ms =
                std::chrono::duration<double, std::milli>(
                    now -
                    *timing.newest_source_arrival_time).count();
            ++stats.window_source_age_samples;
            stats.window_source_age_sum_ms += source_age_ms;
            stats.maximum_source_age_ms = std::max(
                stats.maximum_source_age_ms,
                source_age_ms);
        }
    }

    void ReportDisplayStatsIfDue(
        WindowState& state,
        std::chrono::steady_clock::time_point now) {
        if (!fps_logging_enabled_ ||
            StopRequested()) {
            return;
        }

        const auto queue_stats =
            state.interpolator->GetQueueStats();
        std::lock_guard<std::mutex> lock(stats_mutex_);
        DisplayStats& stats = state.stats;
        if (!stats.window_initialized) {
            return;
        }

        const double elapsed_sec =
            std::chrono::duration<double>(
                now - stats.window_started_at).count();
        if (elapsed_sec < fps_logging_interval_sec_) {
            return;
        }

        const double source_fps =
            static_cast<double>(
                stats.window_received_source_frames) /
            elapsed_sec;
        const double presented_fps =
            static_cast<double>(
                stats.window_presented_frames) /
            elapsed_sec;
        const double measured_multiplier =
            source_fps > 0.0 ?
                presented_fps / source_fps : 0.0;
        const std::uint64_t backpressure_wait_delta =
            queue_stats.source_backpressure_waits -
            stats.last_reported_source_backpressure_waits;
        const double backpressure_wait_ms_delta =
            std::max(
                0.0,
                queue_stats.source_backpressure_wait_ms -
                    stats.last_reported_source_backpressure_wait_ms);
        const std::string last_received_sequence =
            stats.last_received_source_sequence.has_value() ?
                std::to_string(
                    *stats.last_received_source_sequence) :
                "n/a";
        const std::string last_displayed_sequence =
            stats.last_completed_source_sequence.has_value() ?
                std::to_string(
                    *stats.last_completed_source_sequence) :
                "n/a";
        const std::string source_sequence_backlog =
            stats.last_received_source_sequence.has_value() &&
                    stats.last_completed_source_sequence.has_value() &&
                    *stats.last_received_source_sequence >=
                        *stats.last_completed_source_sequence ?
                std::to_string(
                    *stats.last_received_source_sequence -
                    *stats.last_completed_source_sequence) :
                "n/a";

        RCLCPP_INFO(
            this->get_logger(),
            "[FPS] Display \"%s\": source RX %.2f FPS "
            "(+%llu, total %llu) | display-boundary %.2f FPS "
            "(+%llu, total %llu, %s) | measured x%.2f / target x%zu | "
            "%.1f s window | max display gap %.0f ms | "
            "source missing +%llu (total %llu), out-of-order +%llu "
            "(total %llu), unsequenced +%llu (total %llu) | "
            "display sequence breaks +%llu (total %llu) | "
            "last source RX/display #%s/#%s "
            "(backlog %s source intervals).",
            state.window_name.c_str(),
            source_fps,
            static_cast<unsigned long long>(
                stats.window_received_source_frames),
            static_cast<unsigned long long>(
                stats.total_received_source_frames),
            presented_fps,
            static_cast<unsigned long long>(
                stats.window_presented_frames),
            static_cast<unsigned long long>(
                stats.total_presented_frames),
            ActiveDisplayBoundary(),
            measured_multiplier,
            intermediate_frame_count_ + 1,
            elapsed_sec,
            stats.maximum_display_gap_ms,
            static_cast<unsigned long long>(
                stats.window_missing_source_frames),
            static_cast<unsigned long long>(
                stats.total_missing_source_frames),
            static_cast<unsigned long long>(
                stats.window_out_of_order_source_frames),
            static_cast<unsigned long long>(
                stats.total_out_of_order_source_frames),
            static_cast<unsigned long long>(
                stats.window_unsequenced_source_frames),
            static_cast<unsigned long long>(
                stats.total_unsequenced_source_frames),
            static_cast<unsigned long long>(
                stats.window_display_sequence_breaks),
            static_cast<unsigned long long>(
                stats.total_display_sequence_breaks),
            last_received_sequence.c_str(),
            last_displayed_sequence.c_str(),
            source_sequence_backlog.c_str());

        RCLCPP_INFO(
            this->get_logger(),
            "[PIPE] \"%s\": DDS transport avg/max %.1f/%.1f ms "
            "(%llu samples) | source submit avg/max %.1f/%.1f ms | "
            "newest-source age avg/max %.0f/%.0f ms | "
            "HighGUI imshow avg/max %.1f/%.1f ms, slow>100ms %llu | "
            "event avg/max %.1f/%.1f ms, slow>100ms %llu, mode %s | "
            "queue %zu pending (max %zu/%zu), %zu ready "
            "(max %zu/%zu), %zu active; worker %s; lag %.1f ms | "
            "source backpressure +%llu waits / +%.1f ms "
            "(total %llu / %.1f ms, max %.1f ms, %s) | "
            "rejected source +%llu (total %llu) | display errors "
            "+%llu (total %llu).",
            state.window_name.c_str(),
            Average(
                stats.window_transport_sum_ms,
                stats.window_transport_samples),
            stats.maximum_transport_ms,
            static_cast<unsigned long long>(
                stats.window_transport_samples),
            Average(
                stats.window_submit_sum_ms,
                stats.window_submit_samples),
            stats.maximum_submit_ms,
            Average(
                stats.window_source_age_sum_ms,
                stats.window_source_age_samples),
            stats.maximum_source_age_ms,
            Average(
                stats.window_imshow_sum_ms,
                stats.window_imshow_samples),
            stats.maximum_imshow_ms,
            static_cast<unsigned long long>(
                stats.window_slow_imshow_calls),
            Average(
                stats.window_event_sum_ms,
                stats.window_event_samples),
            stats.maximum_event_ms,
            static_cast<unsigned long long>(
                stats.window_slow_event_calls),
            ActiveHighGuiEventMode(),
            queue_stats.pending_pairs,
            queue_stats.maximum_pending_pairs,
            pending_pair_capacity_,
            queue_stats.ready_sequences,
            queue_stats.maximum_ready_sequences,
            ready_sequence_capacity_,
            queue_stats.active_frames_remaining,
            queue_stats.worker_busy ? "busy" : "idle",
            queue_stats.playback_lag_ms,
            static_cast<unsigned long long>(
                backpressure_wait_delta),
            backpressure_wait_ms_delta,
            static_cast<unsigned long long>(
                queue_stats.source_backpressure_waits),
            queue_stats.source_backpressure_wait_ms,
            queue_stats.maximum_source_backpressure_wait_ms,
            queue_stats.source_backpressure_active ?
                "ACTIVE" : "idle",
            static_cast<unsigned long long>(
                stats.window_rejected_source_frames),
            static_cast<unsigned long long>(
                stats.total_rejected_source_frames),
            static_cast<unsigned long long>(
                stats.window_display_errors),
            static_cast<unsigned long long>(
                stats.total_display_errors));

        stats.last_reported_source_backpressure_waits =
            queue_stats.source_backpressure_waits;
        stats.last_reported_source_backpressure_wait_ms =
            queue_stats.source_backpressure_wait_ms;
        stats.window_started_at = now;
        stats.window_received_source_frames = 0;
        stats.window_presented_frames = 0;
        stats.window_missing_source_frames = 0;
        stats.window_out_of_order_source_frames = 0;
        stats.window_unsequenced_source_frames = 0;
        stats.window_display_sequence_breaks = 0;
        stats.window_rejected_source_frames = 0;
        stats.window_display_errors = 0;
        stats.maximum_display_gap_ms = 0.0;
        stats.window_imshow_samples = 0;
        stats.window_imshow_sum_ms = 0.0;
        stats.maximum_imshow_ms = 0.0;
        stats.window_slow_imshow_calls = 0;
        stats.window_event_samples = 0;
        stats.window_event_sum_ms = 0.0;
        stats.maximum_event_ms = 0.0;
        stats.window_slow_event_calls = 0;
        stats.window_submit_samples = 0;
        stats.window_submit_sum_ms = 0.0;
        stats.maximum_submit_ms = 0.0;
        stats.window_transport_samples = 0;
        stats.window_transport_sum_ms = 0.0;
        stats.maximum_transport_ms = 0.0;
        stats.window_source_age_samples = 0;
        stats.window_source_age_sum_ms = 0.0;
        stats.maximum_source_age_ms = 0.0;
    }

    void WarnIfHighGuiIsSlow(
        double imshow_ms,
        double event_ms,
        std::chrono::steady_clock::time_point now) {
        const double slowest_ms =
            std::max(imshow_ms, event_ms);
        if (slowest_ms <= kSlowOperationThresholdMs ||
            now - last_highgui_slow_warning_at_ <=
                std::chrono::seconds(2)) {
            return;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "[HIGHGUI] Slow display boundary: imshow %.1f ms, "
            "event pump %.1f ms. Ordered playback is preserved; "
            "latency/backpressure may grow instead of dropping frames.",
            imshow_ms,
            event_ms);
        last_highgui_slow_warning_at_ = now;
    }

    // Synchronization is declared before windows_ so interpolator callbacks
    // remain safe while WindowState instances are being destroyed.
    std::mutex display_mutex_;
    std::condition_variable display_condition_;
    std::uint64_t display_wakeup_generation_ = 0;
    std::mutex stats_mutex_;
    std::vector<WindowState> windows_;
    std::atomic<bool> stopping_{false};
    std::size_t intermediate_frame_count_ = 4;
    std::size_t pending_pair_capacity_ = 3;
    std::size_t ready_sequence_capacity_ = 2;
    bool fps_logging_enabled_ = true;
    double fps_logging_interval_sec_ = 5.0;
    std::string configured_highgui_event_mode_ = "wait_key";
    bool highgui_backend_initialized_ = false;
    bool highgui_event_thread_started_ = false;
    bool use_wait_key_event_pump_ = true;
    std::chrono::steady_clock::time_point
        last_highgui_slow_warning_at_;
};

}  // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    int exit_code = 0;
    std::shared_ptr<InterpolationDisplayNode> node;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::thread ros_receiver_thread;
    std::exception_ptr ros_receiver_failure;
    bool node_added_to_executor = false;

    try {
        node = std::make_shared<InterpolationDisplayNode>();
        executor.add_node(node);
        node_added_to_executor = true;
        ros_receiver_thread = std::thread(
            [&executor,
             &node,
             &ros_receiver_failure]() {
                try {
                    executor.spin();
                } catch (...) {
                    ros_receiver_failure =
                        std::current_exception();
                    node->RequestStop();
                    if (rclcpp::ok()) {
                        rclcpp::shutdown();
                    }
                }
            });

        while (rclcpp::ok() &&
               !node->StopRequested()) {
            if (!node->PresentReadyFramesAndProcessEvents()) {
                RCLCPP_INFO(
                    node->get_logger(),
                    "[*] Interpolation display requested shutdown.");
                break;
            }
        }

        node->RequestStop();
        executor.cancel();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        if (ros_receiver_thread.joinable()) {
            ros_receiver_thread.join();
        }
        if (node_added_to_executor) {
            executor.remove_node(node);
            node_added_to_executor = false;
        }
        if (ros_receiver_failure != nullptr) {
            std::rethrow_exception(ros_receiver_failure);
        }
        node->DestroyWindows();
    } catch (const std::exception& error) {
        if (node != nullptr) {
            node->RequestStop();
        }
        executor.cancel();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        if (ros_receiver_thread.joinable()) {
            ros_receiver_thread.join();
        }
        if (node_added_to_executor && node != nullptr) {
            executor.remove_node(node);
            node_added_to_executor = false;
        }
        RCLCPP_FATAL(
            rclcpp::get_logger("interpolation_display_node"),
            "[!] Interpolation display failed: %s",
            error.what());
        exit_code = 1;
    } catch (...) {
        if (node != nullptr) {
            node->RequestStop();
        }
        executor.cancel();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        if (ros_receiver_thread.joinable()) {
            ros_receiver_thread.join();
        }
        if (node_added_to_executor && node != nullptr) {
            executor.remove_node(node);
        }
        RCLCPP_FATAL(
            rclcpp::get_logger("interpolation_display_node"),
            "[!] Interpolation display failed with an unknown exception.");
        exit_code = 1;
    }

    node.reset();
    return exit_code;
}
