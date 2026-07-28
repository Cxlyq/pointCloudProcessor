#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
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
    cv::Scalar border_color_bgr = cv::Scalar(0.0, 0.0, 0.0);
    std::size_t max_pending_pairs = 2;
    std::size_t max_ready_sequences = 2;
};

struct DisDisplayTiming {
    bool starts_new_sequence = false;
};

class DisFrameInterpolator {
public:
    explicit DisFrameInterpolator(DisInterpolationConfig config);
    ~DisFrameInterpolator();

    DisFrameInterpolator(const DisFrameInterpolator&) = delete;
    DisFrameInterpolator& operator=(const DisFrameInterpolator&) = delete;

    void SubmitFrame(
        const cv::Mat& bgr_frame,
        std::chrono::steady_clock::time_point source_frame_time);
    bool TryGetDisplayFrame(
        cv::Mat& bgr_frame,
        DisDisplayTiming* timing = nullptr);
    std::string ConsumeStatus();
    std::string ConsumeError();

private:
    struct FramePair {
        cv::Mat first;
        cv::Mat second;
        std::chrono::steady_clock::duration source_interval;
        std::uint64_t generation = 0;
    };

    struct FrameSequence {
        std::vector<cv::Mat> frames;
        std::chrono::steady_clock::duration frame_period{};
        bool delay_before_first_frame = false;
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
    std::chrono::steady_clock::time_point previous_real_frame_time_;
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
};

}  // namespace pointcloud_visualization
