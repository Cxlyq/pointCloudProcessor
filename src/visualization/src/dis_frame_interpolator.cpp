#include "visualization/dis_frame_interpolator.hpp"
#include "visualization/interpolation_timing.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <stdexcept>
#include <sstream>
#include <utility>

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace pointcloud_visualization {

struct DisFrameInterpolator::WorkerCache {
    explicit WorkerCache(const DisInterpolationConfig& config)
        : forward_dis(cv::DISOpticalFlow::create(config.dis_preset)) {
        if (config.use_bidirectional_flow) {
            backward_dis =
                cv::DISOpticalFlow::create(config.dis_preset);
        }
    }

    cv::Ptr<cv::DISOpticalFlow> forward_dis;
    cv::Ptr<cv::DISOpticalFlow> backward_dis;

    cv::Size flow_input_size;
    cv::Mat first_small;
    cv::Mat second_small;
    cv::Mat first_gray;
    cv::Mat second_gray;
    cv::Mat forward_flow_full;
    cv::Mat backward_flow_full;
    std::vector<cv::Mat> forward_flow_channels;
    std::vector<cv::Mat> backward_flow_channels;

    cv::Size grid_size;
    cv::Mat grid_x;
    cv::Mat grid_y;
    cv::Mat first_map_x;
    cv::Mat first_map_y;
    cv::Mat second_map_x;
    cv::Mat second_map_y;
    cv::Mat warped_first;
    cv::Mat warped_second;

    cv::Mat consistency_map_x;
    cv::Mat consistency_map_y;
    cv::Mat sampled_opposite_x;
    cv::Mat sampled_opposite_y;
    cv::Mat consistency_error_x;
    cv::Mat consistency_error_y;
    cv::Mat consistency_error_squared;
    cv::Mat consistency_magnitude_squared;
    cv::Mat consistency_threshold_squared;
    cv::Mat consistency_valid_x;
    cv::Mat consistency_valid_y;
    cv::Mat forward_confidence;
    cv::Mat backward_confidence;
    cv::Mat warped_forward_confidence;
    cv::Mat warped_backward_confidence;
    cv::Mat inverse_confidence;
    cv::Mat only_first_confident;
    cv::Mat only_second_confident;
};

DisFrameInterpolator::DisFrameInterpolator(
    DisInterpolationConfig config,
    std::function<void()> ready_callback)
    : config_(std::move(config)),
      ready_callback_(std::move(ready_callback)) {
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
    if (config_.use_flow_consistency_mask &&
        !config_.use_bidirectional_flow) {
        throw std::invalid_argument(
            "flow consistency masking requires bidirectional flow");
    }

    worker_cache_ = std::make_unique<WorkerCache>(config_);
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
    bool notify_worker = false;
    bool notify_all_workers = false;
    bool notify_ready = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (previous_real_frame_.empty()) {
            previous_real_frame_ = immutable_frame;
            previous_frame_arrival_time_ = frame_arrival_time;
            previous_source_timestamp_ = source_timestamp;

            FrameSequence first_frame;
            // The returned display frame is writable from OpenCV's point of
            // view. Give the display queue its own storage so a future overlay
            // cannot mutate the real-frame endpoint retained for interpolation.
            first_frame.frames.push_back(previous_real_frame_.clone());
            ready_sequences_.push_back(std::move(first_frame));
            notify_ready = true;
        } else if (
            previous_real_frame_.size() != immutable_frame.size()) {
            ++generation_;
            previous_real_frame_ = immutable_frame;
            previous_frame_arrival_time_ = frame_arrival_time;
            previous_source_timestamp_ = source_timestamp;
            source_timestamp_fallback_active_ = false;
            pending_pairs_.clear();
            ready_sequences_.clear();
            active_frames_.clear();
            active_frame_index_ = 0;

            FrameSequence reset_frame;
            reset_frame.frames.push_back(previous_real_frame_.clone());
            ready_sequences_.push_back(std::move(reset_frame));
            last_error_ = "rendered frame size changed; interpolation state was reset";
            notify_all_workers = true;
            notify_ready = true;
        } else {
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
                // Preserve the sequence boundary instead of dropping an
                // interior pair. For example, replace C->D with C->E so that a
                // preceding B->C sequence still connects to the newest
                // accepted frame. Track how many adjacent source intervals
                // were merged so playback can use their average interval
                // instead of reducing its nominal frame rate in proportion to
                // the number of coalesced frames.
                FramePair& newest_pending_pair = pending_pairs_.back();
                newest_pending_pair.second = immutable_frame;
                newest_pending_pair.source_interval_sum += source_delta;
                ++newest_pending_pair.source_interval_count;
                ++coalesced_source_frames_;
            } else {
                pending_pairs_.push_back(
                    FramePair{
                        previous_real_frame_,
                        immutable_frame,
                        source_delta,
                        1,
                        generation_});
            }

            previous_real_frame_ = immutable_frame;
            previous_frame_arrival_time_ = frame_arrival_time;
            previous_source_timestamp_ = source_timestamp;
            notify_worker = true;
        }
    }

    if (notify_all_workers) {
        condition_.notify_all();
    } else if (notify_worker) {
        condition_.notify_one();
    }
    if (notify_ready) {
        NotifyReady();
    }
}

bool DisFrameInterpolator::TryGetDisplayFrame(
    cv::Mat& bgr_frame,
    DisDisplayTiming* timing) {
    if (timing != nullptr) {
        *timing = DisDisplayTiming{};
    }

    bool released_ready_slot = false;
    cv::Mat display_frame;
    std::vector<std::vector<cv::Mat>> retired_active_sequences;
    std::unique_lock<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();

    const auto activate_next_sequence = [this, now,
                                         &released_ready_slot]() {
        while (active_frames_.empty() &&
               !ready_sequences_.empty()) {
            FrameSequence sequence =
                std::move(ready_sequences_.front());
            ready_sequences_.pop_front();
            released_ready_slot = true;

            // Every sequence produced internally contains at least one frame.
            // Ignore an empty sequence defensively without changing the
            // playback timeline.
            if (sequence.frames.empty()) {
                continue;
            }

            active_frames_ = std::move(sequence.frames);
            active_frame_period_ = sequence.frame_period;
            active_frame_index_ = 0;
            if (sequence.delay_before_first_frame) {
                next_frame_deadline_ =
                    now + active_frame_period_;
                playback_timeline_initialized_ = true;
            } else {
                next_frame_deadline_ = now;
                playback_timeline_initialized_ = false;
            }
        }
    };

    activate_next_sequence();
    if (active_frames_.empty()) {
        lock.unlock();
        if (released_ready_slot) {
            condition_.notify_all();
        }
        return false;
    }

    if (now < next_frame_deadline_) {
        lock.unlock();
        if (released_ready_slot) {
            condition_.notify_all();
        }
        return false;
    }

    if (timing != nullptr) {
        timing->starts_new_sequence =
            active_frame_index_ == 0;
    }
    display_frame =
        active_frames_[active_frame_index_];
    ++active_frame_index_;

    if (active_frame_index_ >= active_frames_.size()) {
        // Move the completed image batch out while locked, then release its
        // storage after unlocking. The next sequence starts one complete
        // frame period later instead of catching up by dropping frames.
        retired_active_sequences.push_back(
            std::move(active_frames_));
        active_frames_.clear();
        active_frame_index_ = 0;
        playback_timeline_initialized_ = false;
        activate_next_sequence();
    } else {
        next_frame_deadline_ =
            now + active_frame_period_;
        playback_timeline_initialized_ = true;
    }

    lock.unlock();
    if (released_ready_slot) {
        condition_.notify_all();
    }
    bgr_frame = std::move(display_frame);
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
    const auto now = std::chrono::steady_clock::now();
    const std::size_t active_frames_remaining =
        active_frames_.size() > active_frame_index_ ?
            active_frames_.size() - active_frame_index_ :
            0;
    double playback_lag_ms = 0.0;
    if (active_frames_remaining > 0 &&
        playback_timeline_initialized_ &&
        now > next_frame_deadline_) {
        playback_lag_ms =
            std::chrono::duration<double, std::milli>(
                now - next_frame_deadline_).count();
    }

    DisQueueStats stats;
    stats.coalesced_source_frames = coalesced_source_frames_;
    stats.pending_pairs = pending_pairs_.size();
    stats.ready_sequences = ready_sequences_.size();
    stats.worker_busy = worker_busy_;
    stats.active_frames_remaining = active_frames_remaining;
    stats.playback_lag_ms = playback_lag_ms;
    return stats;
}

std::optional<std::chrono::steady_clock::time_point>
DisFrameInterpolator::GetNextDisplayDeadline() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_frames_.empty() &&
        active_frame_index_ < active_frames_.size()) {
        return next_frame_deadline_;
    }
    if (!ready_sequences_.empty()) {
        // The presentation thread must activate the sequence before its exact
        // first-frame deadline can be known.
        return std::chrono::steady_clock::now();
    }
    return std::nullopt;
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
            worker_busy_ = true;
        }

        FrameSequence sequence;
        std::string success_status;
        std::string failure_message;
        try {
            const auto start_time = std::chrono::steady_clock::now();
            sequence = BuildSequence(pair);
            const auto elapsed_time = std::chrono::steady_clock::now() - start_time;
            const double elapsed_ms =
                std::chrono::duration<double, std::milli>(elapsed_time).count();

            std::ostringstream status;
            status << (config_.use_bidirectional_flow ?
                           "Bidirectional DIS" :
                           "Single-direction DIS")
                   << (config_.use_flow_consistency_mask ?
                           " + consistency mask" : "")
                   << ": " << config_.intermediate_frame_count
                   << " intermediate display frames in "
                   << std::fixed << std::setprecision(1)
                   << elapsed_ms << " ms";
            success_status = status.str();
        } catch (const cv::Exception& error) {
            sequence = FrameSequence{};
            sequence.frames.push_back(pair.second.clone());
            failure_message =
                std::string("OpenCV DIS interpolation failed: ") +
                error.what();
        } catch (const std::exception& error) {
            sequence = FrameSequence{};
            sequence.frames.push_back(pair.second.clone());
            failure_message =
                std::string("frame interpolation failed: ") +
                error.what();
        }

        const bool sequence_was_published =
            PushReadySequence(
                std::move(sequence), pair.generation);
        if (sequence_was_published) {
            if (failure_message.empty()) {
                SetStatus(std::move(success_status));
            } else {
                SetError(std::move(failure_message));
            }
        }
        SetWorkerBusy(false);
        if (sequence_was_published) {
            // The complete sequence is already visible in ready_sequences_.
            // The callback only wakes the external presentation scheduler; it
            // does not transfer ownership or bypass deadline-based playback.
            NotifyReady();
        }
    }
}

DisFrameInterpolator::FrameSequence DisFrameInterpolator::BuildSequence(
    const FramePair& pair) {
    if (pair.first.empty() || pair.second.empty()) {
        throw std::invalid_argument("interpolation pair contains an empty frame");
    }
    if (pair.first.size() != pair.second.size() ||
        pair.first.type() != pair.second.type()) {
        throw std::invalid_argument("interpolation pair has incompatible frames");
    }

    if (config_.intermediate_frame_count == 0) {
        FrameSequence passthrough;
        passthrough.frames.push_back(pair.second.clone());
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

    WorkerCache& cache = *worker_cache_;
    const cv::Size small_size(small_width, small_height);
    if (cache.flow_input_size != small_size) {
        if (cache.flow_input_size.width > 0 &&
            cache.flow_input_size.height > 0) {
            // Recreate only on a resolution transition. Some OpenCV releases
            // cannot safely call calc() again after collectGarbage(), while a
            // fresh instance avoids carrying size-specific internal buffers.
            cache.forward_dis =
                cv::DISOpticalFlow::create(config_.dis_preset);
            if (config_.use_bidirectional_flow) {
                cache.backward_dis =
                    cv::DISOpticalFlow::create(config_.dis_preset);
            }
        }
        cache.flow_input_size = small_size;
    }
    cv::resize(
        pair.first, cache.first_small, small_size,
        0.0, 0.0, cv::INTER_AREA);
    cv::resize(
        pair.second, cache.second_small, small_size,
        0.0, 0.0, cv::INTER_AREA);

    cv::cvtColor(
        cache.first_small, cache.first_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(
        cache.second_small, cache.second_gray, cv::COLOR_BGR2GRAY);

    // Keep the output empty on every call. DIS can interpret a non-empty
    // InputOutputArray as an initial flow estimate; only the algorithm object
    // and its internal work buffers are intentionally reused here.
    cv::Mat flow_small;
    cache.forward_dis->calc(
        cache.first_gray, cache.second_gray, flow_small);

    const auto resize_flow_to_full_resolution =
        [&pair, small_width, small_height](
            const cv::Mat& small_flow,
            cv::Mat& full_flow,
            std::vector<cv::Mat>& channels) {
            cv::resize(
                small_flow,
                full_flow,
                pair.first.size(),
                0.0,
                0.0,
                cv::INTER_LINEAR);

            cv::split(full_flow, channels);
            if (channels.size() != 2) {
                throw std::runtime_error(
                    "DIS optical flow did not return two channels");
            }
            channels.at(0) *=
                static_cast<float>(pair.first.cols) /
                static_cast<float>(small_width);
            channels.at(1) *=
                static_cast<float>(pair.first.rows) /
                static_cast<float>(small_height);
        };

    resize_flow_to_full_resolution(
        flow_small,
        cache.forward_flow_full,
        cache.forward_flow_channels);

    if (config_.use_bidirectional_flow) {
        // As above, do not seed this pair with the previous pair's flow.
        cv::Mat backward_flow_small;
        cache.backward_dis->calc(
            cache.second_gray,
            cache.first_gray,
            backward_flow_small);
        resize_flow_to_full_resolution(
            backward_flow_small,
            cache.backward_flow_full,
            cache.backward_flow_channels);
    }

    if (cache.grid_size != pair.first.size()) {
        cache.grid_size = pair.first.size();
        cache.grid_x.create(pair.first.size(), CV_32FC1);
        cache.grid_y.create(pair.first.size(), CV_32FC1);
        for (int y = 0; y < pair.first.rows; ++y) {
            float* grid_x_row = cache.grid_x.ptr<float>(y);
            float* grid_y_row = cache.grid_y.ptr<float>(y);
            for (int x = 0; x < pair.first.cols; ++x) {
                grid_x_row[x] = static_cast<float>(x);
                grid_y_row[x] = static_cast<float>(y);
            }
        }
    }

    if (config_.use_flow_consistency_mask) {
        const auto build_flow_confidence =
            [&cache, &pair](
                const std::vector<cv::Mat>& primary_flow,
                const std::vector<cv::Mat>& opposite_flow,
                cv::Mat& confidence) {
                constexpr double kRelativeConsistencyThreshold = 0.01;
                constexpr double
                    kAbsoluteConsistencyThresholdSquared = 0.5;
                cv::add(
                    cache.grid_x,
                    primary_flow.at(0),
                    cache.consistency_map_x);
                cv::add(
                    cache.grid_y,
                    primary_flow.at(1),
                    cache.consistency_map_y);

                cv::remap(
                    opposite_flow.at(0),
                    cache.sampled_opposite_x,
                    cache.consistency_map_x,
                    cache.consistency_map_y,
                    cv::INTER_LINEAR,
                    cv::BORDER_CONSTANT,
                    cv::Scalar(0.0));
                cv::remap(
                    opposite_flow.at(1),
                    cache.sampled_opposite_y,
                    cache.consistency_map_x,
                    cache.consistency_map_y,
                    cv::INTER_LINEAR,
                    cv::BORDER_CONSTANT,
                    cv::Scalar(0.0));

                cv::add(
                    primary_flow.at(0),
                    cache.sampled_opposite_x,
                    cache.consistency_error_x);
                cv::add(
                    primary_flow.at(1),
                    cache.sampled_opposite_y,
                    cache.consistency_error_y);
                cv::multiply(
                    cache.consistency_error_x,
                    cache.consistency_error_x,
                    cache.consistency_error_squared);
                cv::multiply(
                    cache.consistency_error_y,
                    cache.consistency_error_y,
                    cache.consistency_threshold_squared);
                cv::add(
                    cache.consistency_error_squared,
                    cache.consistency_threshold_squared,
                    cache.consistency_error_squared);

                cv::multiply(
                    primary_flow.at(0),
                    primary_flow.at(0),
                    cache.consistency_magnitude_squared);
                cv::multiply(
                    primary_flow.at(1),
                    primary_flow.at(1),
                    cache.consistency_threshold_squared);
                cv::add(
                    cache.consistency_magnitude_squared,
                    cache.consistency_threshold_squared,
                    cache.consistency_magnitude_squared);
                cv::multiply(
                    cache.sampled_opposite_x,
                    cache.sampled_opposite_x,
                    cache.consistency_threshold_squared);
                cv::add(
                    cache.consistency_magnitude_squared,
                    cache.consistency_threshold_squared,
                    cache.consistency_magnitude_squared);
                cv::multiply(
                    cache.sampled_opposite_y,
                    cache.sampled_opposite_y,
                    cache.consistency_threshold_squared);
                cv::add(
                    cache.consistency_magnitude_squared,
                    cache.consistency_threshold_squared,
                    cache.consistency_magnitude_squared);
                cv::addWeighted(
                    cache.consistency_magnitude_squared,
                    kRelativeConsistencyThreshold,
                    cache.consistency_magnitude_squared,
                    0.0,
                    kAbsoluteConsistencyThresholdSquared,
                    cache.consistency_threshold_squared);

                cv::compare(
                    cache.consistency_error_squared,
                    cache.consistency_threshold_squared,
                    confidence,
                    cv::CMP_LE);

                cv::compare(
                    cache.consistency_map_x,
                    cv::Scalar(0.0),
                    cache.consistency_valid_x,
                    cv::CMP_GE);
                cv::compare(
                    cache.consistency_map_x,
                    cv::Scalar(
                        static_cast<double>(pair.first.cols - 1)),
                    cache.consistency_valid_y,
                    cv::CMP_LE);
                cv::bitwise_and(
                    cache.consistency_valid_x,
                    cache.consistency_valid_y,
                    cache.consistency_valid_x);
                cv::compare(
                    cache.consistency_map_y,
                    cv::Scalar(0.0),
                    cache.consistency_valid_y,
                    cv::CMP_GE);
                cv::bitwise_and(
                    cache.consistency_valid_x,
                    cache.consistency_valid_y,
                    cache.consistency_valid_x);
                cv::compare(
                    cache.consistency_map_y,
                    cv::Scalar(
                        static_cast<double>(pair.first.rows - 1)),
                    cache.consistency_valid_y,
                    cv::CMP_LE);
                cv::bitwise_and(
                    cache.consistency_valid_x,
                    cache.consistency_valid_y,
                    cache.consistency_valid_x);
                cv::bitwise_and(
                    confidence,
                    cache.consistency_valid_x,
                    confidence);
            };

        build_flow_confidence(
            cache.forward_flow_channels,
            cache.backward_flow_channels,
            cache.forward_confidence);
        build_flow_confidence(
            cache.backward_flow_channels,
            cache.forward_flow_channels,
            cache.backward_confidence);
    }

    const std::size_t interval_count =
        config_.intermediate_frame_count + 1;

    FrameSequence sequence;
    sequence.frames.reserve(interval_count);
    sequence.frame_period = CalculateInterpolatedFramePeriod(
        pair.source_interval_sum,
        pair.source_interval_count,
        config_.intermediate_frame_count);
    sequence.delay_before_first_frame = true;

    for (std::size_t k = 1; k < interval_count; ++k) {
        const float alpha =
            static_cast<float>(k) / static_cast<float>(interval_count);

        cache.first_map_x =
            cache.grid_x -
            alpha * cache.forward_flow_channels.at(0);
        cache.first_map_y =
            cache.grid_y -
            alpha * cache.forward_flow_channels.at(1);

        if (config_.use_bidirectional_flow) {
            cache.second_map_x =
                cache.grid_x -
                (1.0F - alpha) *
                    cache.backward_flow_channels.at(0);
            cache.second_map_y =
                cache.grid_y -
                (1.0F - alpha) *
                    cache.backward_flow_channels.at(1);
        } else {
            cache.second_map_x =
                cache.grid_x +
                (1.0F - alpha) *
                    cache.forward_flow_channels.at(0);
            cache.second_map_y =
                cache.grid_y +
                (1.0F - alpha) *
                    cache.forward_flow_channels.at(1);
        }

        cv::remap(
            pair.first,
            cache.warped_first,
            cache.first_map_x,
            cache.first_map_y,
            cv::INTER_LINEAR,
            cv::BORDER_CONSTANT,
            config_.border_color_bgr);
        cv::remap(
            pair.second,
            cache.warped_second,
            cache.second_map_x,
            cache.second_map_y,
            cv::INTER_LINEAR,
            cv::BORDER_CONSTANT,
            config_.border_color_bgr);

        cv::Mat blended;
        cv::addWeighted(
            cache.warped_first,
            1.0 - alpha,
            cache.warped_second,
            alpha,
            0.0,
            blended);

        if (config_.use_flow_consistency_mask) {
            cv::remap(
                cache.forward_confidence,
                cache.warped_forward_confidence,
                cache.first_map_x,
                cache.first_map_y,
                cv::INTER_NEAREST,
                cv::BORDER_CONSTANT,
                cv::Scalar(0.0));
            cv::remap(
                cache.backward_confidence,
                cache.warped_backward_confidence,
                cache.second_map_x,
                cache.second_map_y,
                cv::INTER_NEAREST,
                cv::BORDER_CONSTANT,
                cv::Scalar(0.0));

            cv::bitwise_not(
                cache.warped_backward_confidence,
                cache.inverse_confidence);
            cv::bitwise_and(
                cache.warped_forward_confidence,
                cache.inverse_confidence,
                cache.only_first_confident);
            cv::bitwise_not(
                cache.warped_forward_confidence,
                cache.inverse_confidence);
            cv::bitwise_and(
                cache.warped_backward_confidence,
                cache.inverse_confidence,
                cache.only_second_confident);

            // Prefer the single consistent endpoint at disocclusions. Where
            // both endpoints agree (or neither is trustworthy), retain the
            // original blend as the conservative fallback.
            cache.warped_first.copyTo(
                blended, cache.only_first_confident);
            cache.warped_second.copyTo(
                blended, cache.only_second_confident);
        }
        sequence.frames.push_back(std::move(blended));
    }

    // Intermediate frames own their blended buffers already. Only the retained
    // real endpoint needs a copy before it crosses the display boundary.
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

void DisFrameInterpolator::SetWorkerBusy(bool busy) {
    std::lock_guard<std::mutex> lock(mutex_);
    worker_busy_ = busy;
}

void DisFrameInterpolator::SetStatus(std::string status) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_status_ = std::move(status);
}

void DisFrameInterpolator::SetError(std::string error) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_ = std::move(error);
}

void DisFrameInterpolator::NotifyReady() {
    if (!ready_callback_) {
        return;
    }

    try {
        ready_callback_();
    } catch (const std::exception& error) {
        SetError(
            std::string("interpolation ready callback failed: ") +
            error.what());
    } catch (...) {
        SetError("interpolation ready callback failed");
    }
}

}  // namespace pointcloud_visualization
