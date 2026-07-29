#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

namespace pointcloud_visualization {

struct DisInterpolationConfig {
    std::size_t intermediate_frame_count = 4;
    double flow_scale = 0.25;
    int dis_preset = 0;
    bool use_bidirectional_flow = false;
    bool use_flow_consistency_mask = false;
    bool use_source_timestamps = false;
    cv::Scalar border_color_bgr = cv::Scalar(0.0, 0.0, 0.0);
    std::size_t max_pending_pairs = 1;
    std::size_t max_ready_sequences = 1;
    std::chrono::steady_clock::duration max_playback_lag =
        std::chrono::seconds(1);
};

struct DisDisplayTiming {
    bool starts_new_sequence = false;
    bool latency_reset = false;
    std::size_t skipped_frames = 0;
    double lag_before_reset_ms = 0.0;
};

struct DisQueueStats {
    std::uint64_t coalesced_source_frames = 0;
    std::uint64_t skipped_display_frames = 0;
    std::uint64_t latency_resets = 0;
    std::size_t pending_pairs = 0;
    std::size_t ready_sequences = 0;
    bool worker_busy = false;
    std::size_t active_frames_remaining = 0;
    double playback_lag_ms = 0.0;
};

class DisFrameInterpolator {
public:
    explicit DisFrameInterpolator(
        DisInterpolationConfig config,
        std::function<void()> ready_callback = {});
    ~DisFrameInterpolator();

    DisFrameInterpolator(const DisFrameInterpolator&) = delete;
    DisFrameInterpolator& operator=(const DisFrameInterpolator&) = delete;

    void SubmitFrame(
        const cv::Mat& bgr_frame,
        std::chrono::steady_clock::time_point frame_arrival_time,
        std::optional<std::chrono::nanoseconds> source_timestamp =
            std::nullopt);
    bool TryGetDisplayFrame(
        cv::Mat& bgr_frame,
        DisDisplayTiming* timing = nullptr);
    std::string ConsumeStatus();
    std::string ConsumeError();
    DisQueueStats GetQueueStats();
    std::optional<std::chrono::steady_clock::time_point>
    GetNextDisplayDeadline();

private:
    struct WorkerCache;

    struct FramePair {
        cv::Mat first;
        cv::Mat second;
        std::chrono::steady_clock::duration source_interval_sum;
        std::size_t source_interval_count = 1;
        std::uint64_t generation = 0;
    };

    struct FrameSequence {
        std::vector<cv::Mat> frames;
        std::chrono::steady_clock::duration frame_period{};
        bool delay_before_first_frame = false;
    };

    void WorkerLoop();
    FrameSequence BuildSequence(const FramePair& pair);
    bool PushReadySequence(
        FrameSequence sequence, std::uint64_t generation);
    void SetWorkerBusy(bool busy);
    void SetStatus(std::string status);
    void SetError(std::string error);
    void NotifyReady();

    DisInterpolationConfig config_;
    std::function<void()> ready_callback_;
    std::unique_ptr<WorkerCache> worker_cache_;

    std::mutex mutex_;
    std::condition_variable condition_;
    bool stopping_ = false;
    bool worker_busy_ = false;
    std::uint64_t generation_ = 0;
    cv::Mat previous_real_frame_;
    std::chrono::steady_clock::time_point previous_frame_arrival_time_;
    std::optional<std::chrono::nanoseconds>
        previous_source_timestamp_;
    bool source_timestamp_fallback_active_ = false;
    std::uint64_t coalesced_source_frames_ = 0;
    std::uint64_t skipped_display_frames_ = 0;
    std::uint64_t latency_resets_ = 0;
    std::deque<FramePair> pending_pairs_;
    std::deque<FrameSequence> ready_sequences_;
    std::string last_status_;
    std::string last_error_;
    std::thread worker_;

    // Playback is driven by the presentation thread; the mutex also protects
    // snapshots requested through GetQueueStats().
    std::vector<cv::Mat> active_frames_;
    std::size_t active_frame_index_ = 0;
    std::chrono::steady_clock::duration active_frame_period_{};
    std::chrono::steady_clock::time_point next_frame_deadline_;
    bool playback_timeline_initialized_ = false;
};

}  // namespace pointcloud_visualization
