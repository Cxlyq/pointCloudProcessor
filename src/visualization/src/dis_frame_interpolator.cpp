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
    if (!std::isfinite(config_.flow_scale) ||
        config_.flow_scale <= 0.0 ||
        config_.flow_scale > 1.0) {
        throw std::invalid_argument("interpolation flow_scale must be in (0, 1]");
    }
    if (config_.intermediate_frame_count > 120) {
        throw std::invalid_argument(
            "interpolation intermediate_frame_count must not exceed 120");
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
    std::chrono::steady_clock::time_point frame_arrival_time,
    std::optional<std::chrono::nanoseconds> source_timestamp) {
    if (bgr_frame.empty()) {
        SetError("cannot interpolate an empty rendered frame");
        return;
    }
    if (bgr_frame.type() != CV_8UC3) {
        SetError("rendered frame must use CV_8UC3 BGR format");
        return;
    }

    // Own one immutable copy of the renderer buffer. All queued cv::Mat
    // instances below share this reference-counted storage instead of cloning
    // the same real frame several times.
    cv::Mat immutable_frame = bgr_frame.clone();
    bool has_new_pair = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::uint64_t current_source_frame_id =
            ++submitted_source_frames_;
        if (previous_real_frame_.empty()) {
            previous_real_frame_ = immutable_frame;
            previous_real_frame_id_ = current_source_frame_id;
            previous_frame_arrival_time_ = frame_arrival_time;
            previous_source_timestamp_ = source_timestamp;

            FrameSequence first_frame;
            first_frame.frames.push_back(previous_real_frame_);
            first_frame.sequence_id = next_sequence_id_++;
            first_frame.source_begin_id = current_source_frame_id;
            first_frame.source_end_id = current_source_frame_id;
            first_frame.source_end_arrival_time =
                frame_arrival_time;
            ready_sequences_.push_back(std::move(first_frame));
            return;
        }

        if (previous_real_frame_.size() != immutable_frame.size()) {
            ++generation_;
            previous_real_frame_ = immutable_frame;
            previous_real_frame_id_ = current_source_frame_id;
            previous_frame_arrival_time_ = frame_arrival_time;
            previous_source_timestamp_ = source_timestamp;
            source_timestamp_fallback_active_ = false;
            pending_pairs_.clear();
            ready_sequences_.clear();
            active_frames_.clear();
            active_frame_index_ = 0;

            FrameSequence reset_frame;
            reset_frame.frames.push_back(previous_real_frame_);
            reset_frame.sequence_id = next_sequence_id_++;
            reset_frame.source_begin_id = current_source_frame_id;
            reset_frame.source_end_id = current_source_frame_id;
            reset_frame.source_end_arrival_time =
                frame_arrival_time;
            ready_sequences_.push_back(std::move(reset_frame));
            last_error_ = "rendered frame size changed; interpolation state was reset";
            condition_.notify_all();
            return;
        }

        auto source_delta =
            frame_arrival_time - previous_frame_arrival_time_;
        if (config_.use_source_timestamps) {
            const bool source_timestamps_are_usable =
                source_timestamp.has_value() &&
                previous_source_timestamp_.has_value() &&
                *source_timestamp > *previous_source_timestamp_;
            if (source_timestamps_are_usable) {
                source_delta =
                    std::chrono::duration_cast<
                        std::chrono::steady_clock::duration>(
                        *source_timestamp -
                        *previous_source_timestamp_);
                source_timestamp_fallback_active_ = false;
            } else if (!source_timestamp_fallback_active_) {
                last_error_ =
                    "message timestamps are missing or non-monotonic; "
                    "interpolation timing fell back to frame arrival times";
                source_timestamp_fallback_active_ = true;
            }
        }
        if (pending_pairs_.size() >= config_.max_pending_pairs) {
            // Preserve the sequence boundary instead of dropping an interior
            // pair. For example, replace C->D with C->E so that a preceding
            // B->C sequence still connects to the newest accepted frame.
            FramePair& newest_pending_pair = pending_pairs_.back();
            newest_pending_pair.second = immutable_frame;
            newest_pending_pair.source_interval += source_delta;
            newest_pending_pair.source_end_id =
                current_source_frame_id;
            newest_pending_pair.source_end_arrival_time =
                frame_arrival_time;
            ++newest_pending_pair.coalesced_source_frames;
            ++coalesced_source_frames_;
        } else {
            pending_pairs_.push_back(
                FramePair{
                    previous_real_frame_,
                    immutable_frame,
                    source_delta,
                    generation_,
                    0,
                    previous_real_frame_id_,
                    current_source_frame_id,
                    0,
                    frame_arrival_time});
        }

        previous_real_frame_ = immutable_frame;
        previous_real_frame_id_ = current_source_frame_id;
        previous_frame_arrival_time_ = frame_arrival_time;
        previous_source_timestamp_ = source_timestamp;
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

            FrameSequence& ready_sequence = ready_sequences_.front();
            active_frames_ =
                std::move(ready_sequence.frames);
            active_frame_period_ =
                ready_sequence.frame_period;
            const bool delay_before_first_frame =
                ready_sequence.delay_before_first_frame;
            active_sequence_id_ = ready_sequence.sequence_id;
            active_source_begin_id_ =
                ready_sequence.source_begin_id;
            active_source_end_id_ =
                ready_sequence.source_end_id;
            active_coalesced_source_frames_ =
                ready_sequence.coalesced_source_frames;
            active_source_span_ms_ =
                ready_sequence.source_span_ms;
            active_source_end_arrival_time_ =
                ready_sequence.source_end_arrival_time;
            ready_sequences_.pop_front();
            active_frame_index_ = 0;
            if (delay_before_first_frame) {
                if (playback_timeline_initialized_) {
                    next_frame_deadline_ += active_frame_period_;
                } else {
                    next_frame_deadline_ =
                        now + active_frame_period_;
                    playback_timeline_initialized_ = true;
                }
            } else {
                next_frame_deadline_ = now;
                playback_timeline_initialized_ = false;
            }
        }
        // A blocked producer may now publish the next contiguous sequence.
        condition_.notify_all();
    }

    if (now < next_frame_deadline_) {
        return false;
    }

    if (timing != nullptr) {
        timing->starts_new_sequence = active_frame_index_ == 0;
        timing->sequence_id = active_sequence_id_;
        timing->source_begin_id = active_source_begin_id_;
        timing->source_end_id = active_source_end_id_;
        timing->frame_index = active_frame_index_ + 1;
        timing->frame_count = active_frames_.size();
        timing->coalesced_source_frames =
            active_coalesced_source_frames_;
        timing->source_span_ms = active_source_span_ms_;
        timing->source_end_age_ms =
            std::max(
                0.0,
                std::chrono::duration<double, std::milli>(
                    now - active_source_end_arrival_time_).count());
        timing->scheduled_lateness_ms =
            std::max(
                0.0,
                std::chrono::duration<double, std::milli>(
                    now - next_frame_deadline_).count());
    }

    bgr_frame = active_frames_[active_frame_index_];
    ++active_frame_index_;

    if (active_frame_index_ >= active_frames_.size()) {
        active_frames_.clear();
        active_frame_index_ = 0;
    } else {
        // Keep one continuous playback timeline. If presentation was late,
        // later calls catch up instead of permanently stretching the rest of
        // this sequence and forcing newer sequences to wait.
        next_frame_deadline_ += active_frame_period_;
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

DisQueueStats DisFrameInterpolator::GetQueueStats() {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::size_t active_frames_remaining =
        active_frames_.size() > active_frame_index_ ?
            active_frames_.size() - active_frame_index_ :
            0;
    return DisQueueStats{
        coalesced_source_frames_,
        generated_sequences_,
        pending_pairs_.size(),
        ready_sequences_.size(),
        active_frames_remaining,
        worker_busy_};
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
            pair.sequence_id = next_sequence_id_++;
            worker_busy_ = true;
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
                   << ": sequence " << pair.sequence_id
                   << " source " << pair.source_begin_id
                   << "->" << pair.source_end_id
                   << " | span "
                   << std::fixed << std::setprecision(1)
                   << std::chrono::duration<double, std::milli>(
                          pair.source_interval).count()
                   << " ms | coalesced "
                   << pair.coalesced_source_frames
                   << " | " << config_.intermediate_frame_count
                   << " intermediate display frames in "
                   << elapsed_ms << " ms";
            SetStatus(status.str());
        } catch (const cv::Exception& error) {
            FrameSequence fallback;
            fallback.frames.push_back(pair.second);
            fallback.sequence_id = pair.sequence_id;
            fallback.source_begin_id = pair.source_begin_id;
            fallback.source_end_id = pair.source_end_id;
            fallback.coalesced_source_frames =
                pair.coalesced_source_frames;
            fallback.source_span_ms =
                std::chrono::duration<double, std::milli>(
                    pair.source_interval).count();
            fallback.source_end_arrival_time =
                pair.source_end_arrival_time;
            if (PushReadySequence(
                    std::move(fallback), pair.generation)) {
                SetError(
                    std::string("OpenCV DIS interpolation failed: ") +
                    error.what());
            }
        } catch (const std::exception& error) {
            FrameSequence fallback;
            fallback.frames.push_back(pair.second);
            fallback.sequence_id = pair.sequence_id;
            fallback.source_begin_id = pair.source_begin_id;
            fallback.source_end_id = pair.source_end_id;
            fallback.coalesced_source_frames =
                pair.coalesced_source_frames;
            fallback.source_span_ms =
                std::chrono::duration<double, std::milli>(
                    pair.source_interval).count();
            fallback.source_end_arrival_time =
                pair.source_end_arrival_time;
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

    const auto set_sequence_diagnostics =
        [&pair](FrameSequence& sequence) {
            sequence.sequence_id = pair.sequence_id;
            sequence.source_begin_id = pair.source_begin_id;
            sequence.source_end_id = pair.source_end_id;
            sequence.coalesced_source_frames =
                pair.coalesced_source_frames;
            sequence.source_span_ms =
                std::chrono::duration<double, std::milli>(
                    pair.source_interval).count();
            sequence.source_end_arrival_time =
                pair.source_end_arrival_time;
        };

    if (config_.intermediate_frame_count == 0) {
        FrameSequence passthrough;
        set_sequence_diagnostics(passthrough);
        passthrough.frames.push_back(pair.second);
        return passthrough;
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
    set_sequence_diagnostics(sequence);
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

    sequence.frames.push_back(pair.second);
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
        worker_busy_ = false;
        return false;
    }

    ready_sequences_.push_back(std::move(sequence));
    ++generated_sequences_;
    worker_busy_ = false;
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
