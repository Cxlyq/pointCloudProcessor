#include "visualization/dis_frame_interpolator.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <stdexcept>
#include <sstream>
#include <utility>

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace pointcloud_visualization {

DisFrameInterpolator::DisFrameInterpolator(DisInterpolationConfig config)
    : config_(std::move(config)) {
    if (config_.flow_scale <= 0.0 || config_.flow_scale > 1.0) {
        throw std::invalid_argument("interpolation flow_scale must be in (0, 1]");
    }
    if (config_.max_pending_pairs == 0 || config_.max_ready_sequences == 0) {
        throw std::invalid_argument("interpolation queue limits must be greater than zero");
    }

    worker_ = std::thread(&DisFrameInterpolator::WorkerLoop, this);
}

DisFrameInterpolator::~DisFrameInterpolator() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    condition_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
}

void DisFrameInterpolator::SubmitFrame(
    const cv::Mat& bgr_frame,
    std::chrono::steady_clock::time_point source_frame_time) {
    if (bgr_frame.empty()) {
        SetError("cannot interpolate an empty rendered frame");
        return;
    }
    if (bgr_frame.type() != CV_8UC3) {
        SetError("rendered frame must use CV_8UC3 BGR format");
        return;
    }

    bool has_new_pair = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (previous_real_frame_.empty()) {
            previous_real_frame_ = bgr_frame.clone();
            previous_real_frame_time_ = source_frame_time;

            FrameSequence first_frame;
            first_frame.frames.push_back(previous_real_frame_.clone());
            ready_sequences_.push_back(std::move(first_frame));
            return;
        }

        if (previous_real_frame_.size() != bgr_frame.size()) {
            ++generation_;
            previous_real_frame_ = bgr_frame.clone();
            previous_real_frame_time_ = source_frame_time;
            pending_pairs_.clear();
            ready_sequences_.clear();
            active_frames_.clear();
            active_frame_index_ = 0;

            FrameSequence reset_frame;
            reset_frame.frames.push_back(previous_real_frame_.clone());
            ready_sequences_.push_back(std::move(reset_frame));
            last_error_ = "rendered frame size changed; interpolation state was reset";
            condition_.notify_all();
            return;
        }

        const auto source_delta =
            source_frame_time - previous_real_frame_time_;
        if (pending_pairs_.size() >= config_.max_pending_pairs) {
            // Preserve the sequence boundary instead of dropping an interior
            // pair. For example, replace C->D with C->E so that a preceding
            // B->C sequence still connects to the newest accepted frame.
            FramePair& newest_pending_pair = pending_pairs_.back();
            newest_pending_pair.second = bgr_frame.clone();
            newest_pending_pair.source_interval += source_delta;
        } else {
            pending_pairs_.push_back(
                FramePair{
                    previous_real_frame_.clone(),
                    bgr_frame.clone(),
                    source_delta,
                    generation_});
        }

        previous_real_frame_ = bgr_frame.clone();
        previous_real_frame_time_ = source_frame_time;
        has_new_pair = true;
    }

    if (has_new_pair) {
        condition_.notify_one();
    }
}

bool DisFrameInterpolator::TryGetDisplayFrame(
    cv::Mat& bgr_frame,
    DisDisplayTiming* timing) {
    const auto now = std::chrono::steady_clock::now();

    if (active_frames_.empty()) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (ready_sequences_.empty()) {
                return false;
            }

            active_frames_ =
                std::move(ready_sequences_.front().frames);
            active_frame_period_ =
                ready_sequences_.front().frame_period;
            const bool delay_before_first_frame =
                ready_sequences_.front().delay_before_first_frame;
            ready_sequences_.pop_front();
            active_frame_index_ = 0;
            next_frame_deadline_ =
                delay_before_first_frame ?
                    now + active_frame_period_ : now;
        }
        // A blocked producer may now publish the next contiguous sequence.
        condition_.notify_all();
    }

    if (now < next_frame_deadline_) {
        return false;
    }

    if (timing != nullptr) {
        timing->starts_new_sequence = active_frame_index_ == 0;
    }

    bgr_frame = active_frames_[active_frame_index_];
    ++active_frame_index_;

    if (active_frame_index_ >= active_frames_.size()) {
        active_frames_.clear();
        active_frame_index_ = 0;
    } else {
        // Fixed-count mode never drops an interpolated frame. If rendering or
        // GUI presentation was late, continue from the actual presentation
        // time so the remaining virtual frames stay evenly spaced.
        next_frame_deadline_ = now + active_frame_period_;
    }

    return true;
}

std::string DisFrameInterpolator::ConsumeError() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string error = std::move(last_error_);
    last_error_.clear();
    return error;
}

std::string DisFrameInterpolator::ConsumeStatus() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string status = std::move(last_status_);
    last_status_.clear();
    return status;
}

void DisFrameInterpolator::WorkerLoop() {
    while (true) {
        FramePair pair;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this]() {
                return stopping_ || !pending_pairs_.empty();
            });

            if (stopping_) {
                return;
            }

            pair = std::move(pending_pairs_.front());
            pending_pairs_.pop_front();
        }

        try {
            const auto start_time = std::chrono::steady_clock::now();
            FrameSequence sequence = BuildSequence(pair);
            const auto elapsed_time = std::chrono::steady_clock::now() - start_time;
            const double elapsed_ms =
                std::chrono::duration<double, std::milli>(elapsed_time).count();

            if (!PushReadySequence(
                    std::move(sequence), pair.generation)) {
                continue;
            }
            std::ostringstream status;
            status << (config_.use_bidirectional_flow ?
                           "Bidirectional DIS" :
                           "Single-direction DIS")
                   << ": " << config_.intermediate_frame_count
                   << " intermediate display frames in "
                   << std::fixed << std::setprecision(1)
                   << elapsed_ms << " ms";
            SetStatus(status.str());
        } catch (const cv::Exception& error) {
            FrameSequence fallback;
            fallback.frames.push_back(pair.second.clone());
            if (PushReadySequence(
                    std::move(fallback), pair.generation)) {
                SetError(
                    std::string("OpenCV DIS interpolation failed: ") +
                    error.what());
            }
        } catch (const std::exception& error) {
            FrameSequence fallback;
            fallback.frames.push_back(pair.second.clone());
            if (PushReadySequence(
                    std::move(fallback), pair.generation)) {
                SetError(
                    std::string("frame interpolation failed: ") +
                    error.what());
            }
        }
    }
}

DisFrameInterpolator::FrameSequence DisFrameInterpolator::BuildSequence(
    const FramePair& pair) const {
    if (pair.first.empty() || pair.second.empty()) {
        throw std::invalid_argument("interpolation pair contains an empty frame");
    }
    if (pair.first.size() != pair.second.size() ||
        pair.first.type() != pair.second.type()) {
        throw std::invalid_argument("interpolation pair has incompatible frames");
    }

    const int minimum_flow_dimension = 8;
    const int small_width = std::min(
        pair.first.cols,
        std::max(minimum_flow_dimension,
                 static_cast<int>(std::lround(pair.first.cols * config_.flow_scale))));
    const int small_height = std::min(
        pair.first.rows,
        std::max(minimum_flow_dimension,
                 static_cast<int>(std::lround(pair.first.rows * config_.flow_scale))));

    if (small_width < minimum_flow_dimension ||
        small_height < minimum_flow_dimension) {
        throw std::invalid_argument("rendered frame is too small for DIS optical flow");
    }

    cv::Mat first_small;
    cv::Mat second_small;
    cv::resize(pair.first, first_small, cv::Size(small_width, small_height),
               0.0, 0.0, cv::INTER_AREA);
    cv::resize(pair.second, second_small, cv::Size(small_width, small_height),
               0.0, 0.0, cv::INTER_AREA);

    cv::Mat first_gray;
    cv::Mat second_gray;
    cv::cvtColor(first_small, first_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(second_small, second_gray, cv::COLOR_BGR2GRAY);

    cv::Mat flow_small;
    auto dis = cv::DISOpticalFlow::create(config_.dis_preset);
    dis->calc(first_gray, second_gray, flow_small);

    const auto resize_flow_to_full_resolution =
        [&pair, small_width, small_height](const cv::Mat& small_flow) {
            cv::Mat full_flow;
            cv::resize(
                small_flow,
                full_flow,
                pair.first.size(),
                0.0,
                0.0,
                cv::INTER_LINEAR);

            std::vector<cv::Mat> channels;
            cv::split(full_flow, channels);
            channels[0] *=
                static_cast<float>(pair.first.cols) /
                static_cast<float>(small_width);
            channels[1] *=
                static_cast<float>(pair.first.rows) /
                static_cast<float>(small_height);
            return channels;
        };

    const std::vector<cv::Mat> forward_flow_channels =
        resize_flow_to_full_resolution(flow_small);

    std::vector<cv::Mat> backward_flow_channels;
    if (config_.use_bidirectional_flow) {
        cv::Mat backward_flow_small;
        auto backward_dis =
            cv::DISOpticalFlow::create(config_.dis_preset);
        backward_dis->calc(
            second_gray, first_gray, backward_flow_small);
        backward_flow_channels =
            resize_flow_to_full_resolution(backward_flow_small);
    }

    cv::Mat grid_x(pair.first.rows, pair.first.cols, CV_32FC1);
    cv::Mat grid_y(pair.first.rows, pair.first.cols, CV_32FC1);
    for (int y = 0; y < pair.first.rows; ++y) {
        float* grid_x_row = grid_x.ptr<float>(y);
        float* grid_y_row = grid_y.ptr<float>(y);
        for (int x = 0; x < pair.first.cols; ++x) {
            grid_x_row[x] = static_cast<float>(x);
            grid_y_row[x] = static_cast<float>(y);
        }
    }

    const std::size_t interval_count =
        config_.intermediate_frame_count + 1;

    FrameSequence sequence;
    sequence.frames.reserve(interval_count);
    sequence.frame_period =
        pair.source_interval /
        static_cast<std::chrono::steady_clock::duration::rep>(
            interval_count);
    if (sequence.frame_period <=
        std::chrono::steady_clock::duration::zero()) {
        sequence.frame_period =
            std::chrono::milliseconds(1);
    }
    sequence.delay_before_first_frame = true;

    for (std::size_t k = 1; k < interval_count; ++k) {
        const float alpha =
            static_cast<float>(k) / static_cast<float>(interval_count);

        cv::Mat first_map_x =
            grid_x - alpha * forward_flow_channels[0];
        cv::Mat first_map_y =
            grid_y - alpha * forward_flow_channels[1];

        cv::Mat second_map_x;
        cv::Mat second_map_y;
        if (config_.use_bidirectional_flow) {
            second_map_x =
                grid_x -
                (1.0F - alpha) * backward_flow_channels[0];
            second_map_y =
                grid_y -
                (1.0F - alpha) * backward_flow_channels[1];
        } else {
            second_map_x =
                grid_x +
                (1.0F - alpha) * forward_flow_channels[0];
            second_map_y =
                grid_y +
                (1.0F - alpha) * forward_flow_channels[1];
        }

        cv::Mat warped_first;
        cv::Mat warped_second;
        cv::remap(pair.first, warped_first, first_map_x, first_map_y,
                  cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                  config_.border_color_bgr);
        cv::remap(pair.second, warped_second, second_map_x, second_map_y,
                  cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                  config_.border_color_bgr);

        cv::Mat blended;
        cv::addWeighted(warped_first, 1.0 - alpha,
                        warped_second, alpha, 0.0, blended);
        sequence.frames.push_back(std::move(blended));
    }

    sequence.frames.push_back(pair.second.clone());
    return sequence;
}

bool DisFrameInterpolator::PushReadySequence(
    FrameSequence sequence, std::uint64_t generation) {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this, generation]() {
        return stopping_ ||
               generation != generation_ ||
               ready_sequences_.size() <
                   config_.max_ready_sequences;
    });

    if (stopping_ || generation != generation_) {
        return false;
    }

    ready_sequences_.push_back(std::move(sequence));
    return true;
}

void DisFrameInterpolator::SetStatus(std::string status) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_status_ = std::move(status);
}

void DisFrameInterpolator::SetError(std::string error) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_ = std::move(error);
}

}  // namespace pointcloud_visualization
