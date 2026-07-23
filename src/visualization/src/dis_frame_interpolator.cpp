#include "visualization/dis_frame_interpolator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace pointcloud_visualization {

DisFrameInterpolator::DisFrameInterpolator(DisInterpolationConfig config)
    : config_(std::move(config)) {
    if (config_.output_fps <= 0.0) {
        throw std::invalid_argument("interpolation output_fps must be greater than zero");
    }
    if (config_.duration_sec <= 0.0) {
        throw std::invalid_argument("interpolation duration_sec must be greater than zero");
    }
    if (config_.flow_scale <= 0.0 || config_.flow_scale > 1.0) {
        throw std::invalid_argument("interpolation flow_scale must be in (0, 1]");
    }
    if (config_.max_pending_pairs == 0 || config_.max_ready_sequences == 0) {
        throw std::invalid_argument("interpolation queue limits must be greater than zero");
    }

    frame_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / config_.output_fps));
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

void DisFrameInterpolator::SubmitFrame(const cv::Mat& bgr_frame) {
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

            FrameSequence first_frame;
            first_frame.frames.push_back(previous_real_frame_.clone());
            ready_sequences_.push_back(std::move(first_frame));
            return;
        }

        if (previous_real_frame_.size() != bgr_frame.size()) {
            ++generation_;
            previous_real_frame_ = bgr_frame.clone();
            pending_pairs_.clear();
            ready_sequences_.clear();
            active_frames_.clear();
            active_frame_index_ = 0;

            FrameSequence reset_frame;
            reset_frame.frames.push_back(previous_real_frame_.clone());
            ready_sequences_.push_back(std::move(reset_frame));
            last_error_ = "rendered frame size changed; interpolation state was reset";
            return;
        }

        if (pending_pairs_.size() >= config_.max_pending_pairs) {
            pending_pairs_.pop_front();
        }

        pending_pairs_.push_back(
            FramePair{
                previous_real_frame_.clone(),
                bgr_frame.clone(),
                generation_});
        previous_real_frame_ = bgr_frame.clone();
        has_new_pair = true;
    }

    if (has_new_pair) {
        condition_.notify_one();
    }
}

bool DisFrameInterpolator::TryGetDisplayFrame(cv::Mat& bgr_frame) {
    const auto now = std::chrono::steady_clock::now();

    if (active_frames_.empty()) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (ready_sequences_.empty()) {
            return false;
        }

        active_frames_ = std::move(ready_sequences_.front().frames);
        ready_sequences_.pop_front();
        active_frame_index_ = 0;
        next_frame_deadline_ = now;
    }

    if (now < next_frame_deadline_) {
        return false;
    }

    while (active_frame_index_ + 1 < active_frames_.size() &&
           now >= next_frame_deadline_ + frame_period_) {
        ++active_frame_index_;
        next_frame_deadline_ += frame_period_;
    }

    bgr_frame = active_frames_[active_frame_index_];
    ++active_frame_index_;
    next_frame_deadline_ += frame_period_;

    if (active_frame_index_ >= active_frames_.size()) {
        active_frames_.clear();
        active_frame_index_ = 0;
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
            const std::size_t generated_frame_count = sequence.frames.size();
            const auto elapsed_time = std::chrono::steady_clock::now() - start_time;
            const double elapsed_ms =
                std::chrono::duration<double, std::milli>(elapsed_time).count();

            PushReadySequence(std::move(sequence), pair.generation);
            SetStatus(
                std::string(config_.use_bidirectional_flow ?
                                "Bidirectional DIS generated " :
                                "Single-direction DIS generated ") +
                std::to_string(generated_frame_count) +
                " display frames in " + std::to_string(elapsed_ms) + " ms");
        } catch (const cv::Exception& error) {
            FrameSequence fallback;
            fallback.frames.push_back(pair.second.clone());
            PushReadySequence(std::move(fallback), pair.generation);
            SetError(std::string("OpenCV DIS interpolation failed: ") + error.what());
        } catch (const std::exception& error) {
            FrameSequence fallback;
            fallback.frames.push_back(pair.second.clone());
            PushReadySequence(std::move(fallback), pair.generation);
            SetError(std::string("frame interpolation failed: ") + error.what());
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

    const int interval_count = std::max(
        1, static_cast<int>(std::lround(
               config_.duration_sec * config_.output_fps)));

    FrameSequence sequence;
    sequence.frames.reserve(static_cast<std::size_t>(interval_count + 1));
    sequence.frames.push_back(pair.first.clone());

    for (int k = 1; k < interval_count; ++k) {
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

void DisFrameInterpolator::PushReadySequence(
    FrameSequence sequence, std::uint64_t generation) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation != generation_) {
        return;
    }
    if (ready_sequences_.size() >= config_.max_ready_sequences) {
        ready_sequences_.pop_front();
    }
    ready_sequences_.push_back(std::move(sequence));
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
