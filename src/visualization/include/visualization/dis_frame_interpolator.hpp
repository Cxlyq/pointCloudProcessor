#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
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
    bool use_source_timestamps = false;
    cv::Scalar border_color_bgr = cv::Scalar(0.0, 0.0, 0.0);
    std::size_t max_pending_pairs = 1;
    std::size_t max_ready_sequences = 1;
};

struct DisDisplayTiming {
    bool starts_new_sequence = false;
    std::uint64_t sequence_id = 0;
    std::uint64_t source_begin_id = 0;
    std::uint64_t source_end_id = 0;
    std::size_t frame_index = 0;
    std::size_t frame_count = 0;
    std::size_t coalesced_source_frames = 0;
    double source_span_ms = 0.0;
    double source_end_age_ms = 0.0;
    double scheduled_lateness_ms = 0.0;
};

struct DisQueueStats {
    std::uint64_t coalesced_source_frames = 0;
    std::uint64_t generated_sequences = 0;
    std::size_t pending_pairs = 0;
    std::size_t ready_sequences = 0;
    std::size_t active_frames_remaining = 0;
    bool worker_busy = false;
};

class DisFrameInterpolator {
public:
    explicit DisFrameInterpolator(DisInterpolationConfig config);
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

private:
    struct FramePair {
        cv::Mat first;
        cv::Mat second;
        std::chrono::steady_clock::duration source_interval;
        std::uint64_t generation = 0;
        std::uint64_t sequence_id = 0;
        std::uint64_t source_begin_id = 0;
        std::uint64_t source_end_id = 0;
        std::size_t coalesced_source_frames = 0;
        std::chrono::steady_clock::time_point
            source_end_arrival_time;
    };

    struct FrameSequence {
        std::vector<cv::Mat> frames;
        std::chrono::steady_clock::duration frame_period{};
        bool delay_before_first_frame = false;
        std::uint64_t sequence_id = 0;
        std::uint64_t source_begin_id = 0;
        std::uint64_t source_end_id = 0;
        std::size_t coalesced_source_frames = 0;
        double source_span_ms = 0.0;
        std::chrono::steady_clock::time_point
            source_end_arrival_time;
    };

    void WorkerLoop();
    FrameSequence BuildSequence(const FramePair& pair) const;
    bool PushReadySequence(
        FrameSequence sequence, std::uint64_t generation);
    void SetStatus(std::string status);
    void SetError(std::string error);

    DisInterpolationConfig config_;

    std::mutex mutex_;
    std::condition_variable condition_;
    bool stopping_ = false;
    std::uint64_t generation_ = 0;
    cv::Mat previous_real_frame_;
    std::uint64_t previous_real_frame_id_ = 0;
    std::uint64_t submitted_source_frames_ = 0;
    std::uint64_t next_sequence_id_ = 1;
    std::chrono::steady_clock::time_point previous_frame_arrival_time_;
    std::optional<std::chrono::nanoseconds>
        previous_source_timestamp_;
    bool source_timestamp_fallback_active_ = false;
    std::uint64_t coalesced_source_frames_ = 0;
    std::uint64_t generated_sequences_ = 0;
    bool worker_busy_ = false;
    std::deque<FramePair> pending_pairs_;
    std::deque<FrameSequence> ready_sequences_;
    std::string last_status_;
    std::string last_error_;
    std::thread worker_;

    // These fields are accessed only by the UI thread.
    std::vector<cv::Mat> active_frames_;
    std::size_t active_frame_index_ = 0;
    std::chrono::steady_clock::duration active_frame_period_{};
    std::chrono::steady_clock::time_point next_frame_deadline_;
    bool playback_timeline_initialized_ = false;
    std::uint64_t active_sequence_id_ = 0;
    std::uint64_t active_source_begin_id_ = 0;
    std::uint64_t active_source_end_id_ = 0;
    std::size_t active_coalesced_source_frames_ = 0;
    double active_source_span_ms_ = 0.0;
    std::chrono::steady_clock::time_point
        active_source_end_arrival_time_;
};

}  // namespace pointcloud_visualization
