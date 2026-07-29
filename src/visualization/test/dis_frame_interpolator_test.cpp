#include "visualization/dis_frame_interpolator.hpp"
#include "visualization/interpolation_timing.hpp"

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <future>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

namespace pointcloud_visualization {
namespace {

using namespace std::chrono_literals;

bool WaitUntil(
    const std::function<bool()>& predicate,
    std::chrono::milliseconds timeout = 3s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    do {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(1ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return predicate();
}

bool WaitForDisplayFrame(
    DisFrameInterpolator& interpolator,
    cv::Mat& frame,
    std::chrono::milliseconds timeout = 3s) {
    return WaitUntil(
        [&interpolator, &frame]() {
            return interpolator.TryGetDisplayFrame(frame);
        },
        timeout);
}

cv::Mat SolidFrame(
    int rows,
    int cols,
    unsigned char value) {
    return cv::Mat(
        rows,
        cols,
        CV_8UC3,
        cv::Scalar(value, value, value)).clone();
}

TEST(InterpolationTimingTest, AveragesAccumulatedSourceIntervals) {
    const auto normal_period = CalculateInterpolatedFramePeriod(
        std::chrono::seconds(2), 1, 4);
    const auto averaged_period = CalculateInterpolatedFramePeriod(
        std::chrono::seconds(4), 2, 4);

    EXPECT_EQ(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            normal_period).count(),
        400);
    EXPECT_EQ(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            averaged_period).count(),
        400);
}

TEST(InterpolationTimingTest, NonPositivePeriodIsClamped) {
    EXPECT_EQ(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            CalculateInterpolatedFramePeriod(
                std::chrono::steady_clock::duration::zero(),
                1,
                4)).count(),
        1);
    EXPECT_EQ(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            CalculateInterpolatedFramePeriod(
                -std::chrono::seconds(1),
                1,
                4)).count(),
        1);
}

TEST(InterpolationTimingTest, RejectsInvalidCounts) {
    EXPECT_THROW(
        CalculateInterpolatedFramePeriod(
            std::chrono::seconds(1), 0, 4),
        std::invalid_argument);
    EXPECT_THROW(
        CalculateInterpolatedFramePeriod(
            std::chrono::seconds(1),
            1,
            std::numeric_limits<std::size_t>::max()),
        std::overflow_error);
}

TEST(DisFrameInterpolatorTest, ConsistencyMaskRequiresBidirectionalFlow) {
    DisInterpolationConfig config;
    config.use_flow_consistency_mask = true;

    EXPECT_THROW(
        {
            DisFrameInterpolator interpolator(config);
        },
        std::invalid_argument);
}

TEST(DisFrameInterpolatorTest, NotifiesAfterCompleteSequenceIsReady) {
    std::mutex notification_mutex;
    std::condition_variable notification_condition;
    std::size_t notification_count = 0;

    DisInterpolationConfig config;
    config.intermediate_frame_count = 1;
    config.flow_scale = 1.0;
    config.dis_preset = cv::DISOpticalFlow::PRESET_ULTRAFAST;
    DisFrameInterpolator interpolator(
        config,
        [&notification_mutex,
         &notification_condition,
         &notification_count]() {
            {
                std::lock_guard<std::mutex> lock(
                    notification_mutex);
                ++notification_count;
            }
            notification_condition.notify_one();
        });

    const auto first_arrival = std::chrono::steady_clock::now();
    interpolator.SubmitFrame(
        SolidFrame(24, 24, 10), first_arrival);
    {
        std::unique_lock<std::mutex> lock(notification_mutex);
        ASSERT_TRUE(notification_condition.wait_for(
            lock,
            1s,
            [&notification_count]() {
                return notification_count >= 1;
            }));
    }
    EXPECT_EQ(interpolator.GetQueueStats().ready_sequences, 1U);

    cv::Mat first;
    ASSERT_TRUE(WaitForDisplayFrame(interpolator, first));
    interpolator.SubmitFrame(
        SolidFrame(24, 24, 20),
        first_arrival + 20ms);

    {
        std::unique_lock<std::mutex> lock(notification_mutex);
        ASSERT_TRUE(notification_condition.wait_for(
            lock,
            3s,
            [&notification_count]() {
                return notification_count >= 2;
            }));
    }

    const DisQueueStats stats = interpolator.GetQueueStats();
    EXPECT_EQ(stats.ready_sequences, 1U);
    EXPECT_FALSE(stats.worker_busy);
}

TEST(DisFrameInterpolatorTest, ReadyQueueBackpressurePreservesSequences) {
    DisInterpolationConfig config;
    config.intermediate_frame_count = 0;
    config.max_pending_pairs = 2;
    config.max_ready_sequences = 1;
    DisFrameInterpolator interpolator(config);

    const auto first_arrival = std::chrono::steady_clock::now();
    interpolator.SubmitFrame(
        SolidFrame(16, 16, 10), first_arrival);
    interpolator.SubmitFrame(
        SolidFrame(16, 16, 20),
        first_arrival + 20ms);
    interpolator.SubmitFrame(
        SolidFrame(16, 16, 30),
        first_arrival + 40ms);

    cv::Mat first;
    cv::Mat second;
    cv::Mat third;
    ASSERT_TRUE(WaitForDisplayFrame(interpolator, first));
    ASSERT_TRUE(WaitForDisplayFrame(interpolator, second));
    ASSERT_TRUE(WaitForDisplayFrame(interpolator, third));

    EXPECT_NEAR(cv::mean(first)[0], 10.0, 1.0);
    EXPECT_NEAR(cv::mean(second)[0], 20.0, 1.0);
    EXPECT_NEAR(cv::mean(third)[0], 30.0, 1.0);
    EXPECT_EQ(
        interpolator.GetQueueStats().source_backpressure_waits,
        0U);
}

TEST(DisFrameInterpolatorTest, LatePlaybackPresentsEveryFrameInOrder) {
    DisInterpolationConfig config;
    config.intermediate_frame_count = 4;
    config.flow_scale = 1.0;
    config.dis_preset = cv::DISOpticalFlow::PRESET_ULTRAFAST;
    DisFrameInterpolator interpolator(config);

    const auto first_arrival = std::chrono::steady_clock::now();
    interpolator.SubmitFrame(
        SolidFrame(24, 24, 0),
        first_arrival,
        std::nullopt,
        10);

    cv::Mat first;
    ASSERT_TRUE(WaitForDisplayFrame(interpolator, first));

    interpolator.SubmitFrame(
        SolidFrame(24, 24, 200),
        first_arrival + 100ms,
        std::nullopt,
        11);
    ASSERT_TRUE(WaitUntil([&interpolator]() {
        const DisQueueStats stats = interpolator.GetQueueStats();
        return stats.ready_sequences == 1 && !stats.worker_busy;
    }));

    // Activating an interpolated sequence schedules its first intermediate
    // frame one frame period after the preceding real frame.
    cv::Mat not_due;
    EXPECT_FALSE(interpolator.TryGetDisplayFrame(
        not_due));
    const auto deadline =
        interpolator.GetNextDisplayDeadline();
    ASSERT_TRUE(deadline.has_value());

    // Deliberately miss more than two nominal 20 ms deadlines. A single
    // presentation call must consume exactly one frame, not catch up by
    // silently skipping the overdue intermediates.
    std::this_thread::sleep_for(55ms);
    cv::Mat first_intermediate;
    DisDisplayTiming timing;
    ASSERT_TRUE(interpolator.TryGetDisplayFrame(
        first_intermediate, &timing));
    EXPECT_TRUE(timing.starts_new_sequence);
    EXPECT_EQ(timing.frame_index, 1U);
    EXPECT_EQ(timing.frame_count, 5U);
    ASSERT_TRUE(timing.source_sequence_start.has_value());
    ASSERT_TRUE(timing.source_sequence_end.has_value());
    EXPECT_EQ(*timing.source_sequence_start, 10U);
    EXPECT_EQ(*timing.source_sequence_end, 11U);
    EXPECT_TRUE(
        timing.newest_source_arrival_time.has_value());
    EXPECT_NEAR(timing.source_interval_ms, 100.0, 0.1);
    EXPECT_FALSE(first_intermediate.empty());
    EXPECT_EQ(first_intermediate.type(), CV_8UC3);

    DisQueueStats stats = interpolator.GetQueueStats();
    EXPECT_EQ(stats.active_frames_remaining, 4U);
    cv::Mat still_not_due;
    EXPECT_FALSE(interpolator.TryGetDisplayFrame(still_not_due));

    double previous_mean = cv::mean(first_intermediate)[0];
    cv::Mat final_frame;
    for (std::size_t frame_index = 1;
         frame_index < 5;
         ++frame_index) {
        cv::Mat next_frame;
        ASSERT_TRUE(WaitForDisplayFrame(interpolator, next_frame));
        const double next_mean = cv::mean(next_frame)[0];
        EXPECT_GT(next_mean, previous_mean);
        previous_mean = next_mean;
        final_frame = std::move(next_frame);
    }

    EXPECT_EQ(
        cv::norm(
            final_frame,
            SolidFrame(24, 24, 200),
            cv::NORM_INF),
        0.0);
    stats = interpolator.GetQueueStats();
    EXPECT_EQ(stats.active_frames_remaining, 0U);
    EXPECT_FALSE(
        interpolator.GetNextDisplayDeadline().has_value());
}

TEST(DisFrameInterpolatorTest, DisplayMutationDoesNotAlterRetainedEndpoint) {
    DisInterpolationConfig config;
    config.intermediate_frame_count = 1;
    config.flow_scale = 1.0;
    config.dis_preset = cv::DISOpticalFlow::PRESET_ULTRAFAST;
    DisFrameInterpolator interpolator(config);

    const auto first_arrival = std::chrono::steady_clock::now();
    interpolator.SubmitFrame(
        SolidFrame(32, 32, 20), first_arrival);

    cv::Mat displayed_first;
    ASSERT_TRUE(WaitForDisplayFrame(interpolator, displayed_first));
    displayed_first.setTo(cv::Scalar(240, 240, 240));

    interpolator.SubmitFrame(
        SolidFrame(32, 32, 20),
        first_arrival + 20ms);

    cv::Mat intermediate;
    ASSERT_TRUE(WaitForDisplayFrame(interpolator, intermediate));
    ASSERT_TRUE(WaitUntil([&interpolator]() {
        const DisQueueStats stats = interpolator.GetQueueStats();
        return stats.pending_pairs == 0 && !stats.worker_busy;
    }));
    EXPECT_TRUE(interpolator.ConsumeError().empty());
    EXPECT_NE(
        interpolator.ConsumeStatus().find(
            "1 intermediate display frames"),
        std::string::npos);
    const cv::Scalar mean = cv::mean(intermediate);
    EXPECT_NEAR(mean[0], 20.0, 1.0);
    EXPECT_NEAR(mean[1], 20.0, 1.0);
    EXPECT_NEAR(mean[2], 20.0, 1.0);
}

TEST(DisFrameInterpolatorTest, ReportsTimestampFallback) {
    DisInterpolationConfig config;
    config.intermediate_frame_count = 0;
    config.use_source_timestamps = true;
    DisFrameInterpolator interpolator(config);

    const auto first_arrival = std::chrono::steady_clock::now();
    interpolator.SubmitFrame(
        SolidFrame(16, 16, 10),
        first_arrival,
        std::chrono::nanoseconds(100));

    cv::Mat first;
    ASSERT_TRUE(WaitForDisplayFrame(interpolator, first));

    interpolator.SubmitFrame(
        SolidFrame(16, 16, 20),
        first_arrival + 20ms,
        std::chrono::nanoseconds(100));

    EXPECT_NE(
        interpolator.ConsumeError().find("fell back"),
        std::string::npos);
}

TEST(DisFrameInterpolatorTest, ValidMessageTimestampsControlPlaybackPeriod) {
    DisInterpolationConfig config;
    config.intermediate_frame_count = 4;
    config.flow_scale = 1.0;
    config.dis_preset =
        cv::DISOpticalFlow::PRESET_ULTRAFAST;
    config.use_source_timestamps = true;
    DisFrameInterpolator interpolator(config);

    const auto first_arrival =
        std::chrono::steady_clock::now();
    interpolator.SubmitFrame(
        SolidFrame(24, 24, 10),
        first_arrival,
        1s);
    cv::Mat first;
    ASSERT_TRUE(WaitForDisplayFrame(interpolator, first));

    // Callback arrival is intentionally five seconds later, while the
    // publisher timestamp advances only 100 ms. Playback must use the source
    // timestamp so callback/DDS backpressure cannot stretch the frame period.
    interpolator.SubmitFrame(
        SolidFrame(24, 24, 20),
        first_arrival + 5s,
        1100ms);
    ASSERT_TRUE(WaitUntil([&interpolator]() {
        const DisQueueStats stats =
            interpolator.GetQueueStats();
        return stats.ready_sequences == 1 &&
               !stats.worker_busy;
    }));

    cv::Mat activates_sequence;
    EXPECT_FALSE(
        interpolator.TryGetDisplayFrame(
            activates_sequence));
    cv::Mat intermediate;
    DisDisplayTiming timing;
    ASSERT_TRUE(WaitUntil(
        [&interpolator, &intermediate, &timing]() {
            return interpolator.TryGetDisplayFrame(
                intermediate, &timing);
        },
        250ms));
    EXPECT_NEAR(timing.source_interval_ms, 100.0, 0.1);
    EXPECT_TRUE(interpolator.ConsumeError().empty());
}

TEST(DisFrameInterpolatorTest, SizeChangeResetsQueuedState) {
    DisInterpolationConfig config;
    config.intermediate_frame_count = 1;
    config.flow_scale = 1.0;
    config.dis_preset = cv::DISOpticalFlow::PRESET_ULTRAFAST;
    DisFrameInterpolator interpolator(config);

    const auto first_arrival = std::chrono::steady_clock::now();
    interpolator.SubmitFrame(
        SolidFrame(16, 16, 10), first_arrival);

    cv::Mat first;
    ASSERT_TRUE(WaitForDisplayFrame(interpolator, first));

    interpolator.SubmitFrame(
        SolidFrame(16, 16, 20),
        first_arrival + 20ms);
    cv::Mat old_size_intermediate;
    ASSERT_TRUE(WaitForDisplayFrame(
        interpolator, old_size_intermediate));
    cv::Mat old_size_endpoint;
    ASSERT_TRUE(WaitForDisplayFrame(
        interpolator, old_size_endpoint));

    interpolator.SubmitFrame(
        SolidFrame(20, 24, 30),
        first_arrival + 40ms);

    cv::Mat reset;
    ASSERT_TRUE(WaitForDisplayFrame(interpolator, reset));
    EXPECT_EQ(reset.rows, 20);
    EXPECT_EQ(reset.cols, 24);
    EXPECT_NE(
        interpolator.ConsumeError().find("size changed"),
        std::string::npos);

    interpolator.SubmitFrame(
        SolidFrame(20, 24, 40),
        first_arrival + 60ms);
    cv::Mat new_size_intermediate;
    ASSERT_TRUE(WaitForDisplayFrame(
        interpolator, new_size_intermediate));
    cv::Mat new_size_endpoint;
    ASSERT_TRUE(WaitForDisplayFrame(
        interpolator, new_size_endpoint));
    EXPECT_EQ(new_size_endpoint.rows, 20);
    EXPECT_EQ(new_size_endpoint.cols, 24);
    EXPECT_TRUE(interpolator.ConsumeError().empty());
}

TEST(DisFrameInterpolatorTest, QueueStatsExposeActiveTimelineLag) {
    DisInterpolationConfig config;
    config.intermediate_frame_count = 2;
    config.flow_scale = 1.0;
    config.dis_preset = cv::DISOpticalFlow::PRESET_ULTRAFAST;
    DisFrameInterpolator interpolator(config);

    const auto first_arrival = std::chrono::steady_clock::now();
    interpolator.SubmitFrame(
        SolidFrame(24, 24, 10), first_arrival);

    cv::Mat first;
    ASSERT_TRUE(WaitForDisplayFrame(interpolator, first));

    interpolator.SubmitFrame(
        SolidFrame(24, 24, 20),
        first_arrival + 300ms);
    ASSERT_TRUE(WaitUntil([&interpolator]() {
        const DisQueueStats stats = interpolator.GetQueueStats();
        return stats.ready_sequences == 1 && !stats.worker_busy;
    }));

    cv::Mat not_due;
    EXPECT_FALSE(interpolator.TryGetDisplayFrame(not_due));
    const DisQueueStats active_stats = interpolator.GetQueueStats();
    EXPECT_EQ(active_stats.ready_sequences, 0U);
    EXPECT_EQ(active_stats.active_frames_remaining, 3U);
    EXPECT_FALSE(active_stats.worker_busy);
    EXPECT_DOUBLE_EQ(active_stats.playback_lag_ms, 0.0);

    std::this_thread::sleep_for(125ms);
    const DisQueueStats late_stats = interpolator.GetQueueStats();
    EXPECT_GT(late_stats.playback_lag_ms, 0.0);
}

TEST(DisFrameInterpolatorTest, PendingQueueBackpressurePreservesEveryPair) {
    DisInterpolationConfig config;
    config.intermediate_frame_count = 0;
    config.max_pending_pairs = 1;
    config.max_ready_sequences = 1;
    DisFrameInterpolator interpolator(config);

    const auto first_arrival = std::chrono::steady_clock::now();
    interpolator.SubmitFrame(
        SolidFrame(16, 16, 10), first_arrival);
    interpolator.SubmitFrame(
        SolidFrame(16, 16, 20),
        first_arrival + 20ms);

    // The first display frame fills the ready queue, so the worker is
    // deterministically blocked while placing the second frame there.
    ASSERT_TRUE(WaitUntil([&interpolator]() {
        const DisQueueStats stats = interpolator.GetQueueStats();
        return stats.worker_busy &&
               stats.pending_pairs == 0 &&
               stats.ready_sequences == 1;
    }));

    interpolator.SubmitFrame(
        SolidFrame(16, 16, 30),
        first_arrival + 40ms);
    auto fourth_submission = std::async(
        std::launch::async,
        [&interpolator, first_arrival]() {
            interpolator.SubmitFrame(
                SolidFrame(16, 16, 40),
                first_arrival + 60ms);
        });

    const bool submission_blocked = WaitUntil([&interpolator]() {
        const DisQueueStats stats = interpolator.GetQueueStats();
        return stats.source_backpressure_active &&
               stats.source_backpressure_waits == 1 &&
               stats.pending_pairs == 1 &&
               stats.ready_sequences == 1;
    });

    cv::Mat first;
    cv::Mat second;
    cv::Mat third;
    cv::Mat fourth;
    const bool got_first =
        WaitForDisplayFrame(interpolator, first);
    const bool submission_released =
        fourth_submission.wait_for(3s) ==
        std::future_status::ready;
    if (submission_released) {
        fourth_submission.get();
    } else {
        interpolator.RequestStop();
        fourth_submission.wait();
    }
    const bool got_second =
        WaitForDisplayFrame(interpolator, second);
    const bool got_third =
        WaitForDisplayFrame(interpolator, third);
    const bool got_fourth =
        WaitForDisplayFrame(interpolator, fourth);

    ASSERT_TRUE(submission_blocked);
    ASSERT_TRUE(submission_released);
    ASSERT_TRUE(got_first);
    ASSERT_TRUE(got_second);
    ASSERT_TRUE(got_third);
    ASSERT_TRUE(got_fourth);
    EXPECT_NEAR(cv::mean(first)[0], 10.0, 1.0);
    EXPECT_NEAR(cv::mean(second)[0], 20.0, 1.0);
    EXPECT_NEAR(cv::mean(third)[0], 30.0, 1.0);
    EXPECT_NEAR(cv::mean(fourth)[0], 40.0, 1.0);

    const DisQueueStats stats = interpolator.GetQueueStats();
    EXPECT_EQ(stats.source_backpressure_waits, 1U);
    EXPECT_GT(stats.source_backpressure_wait_ms, 0.0);
    EXPECT_GT(
        stats.maximum_source_backpressure_wait_ms, 0.0);
    EXPECT_FALSE(stats.source_backpressure_active);
}

TEST(DisFrameInterpolatorTest, RequestStopUnblocksPendingSourceSubmission) {
    DisInterpolationConfig config;
    config.intermediate_frame_count = 0;
    config.max_pending_pairs = 1;
    config.max_ready_sequences = 1;
    DisFrameInterpolator interpolator(config);

    const auto first_arrival = std::chrono::steady_clock::now();
    interpolator.SubmitFrame(
        SolidFrame(16, 16, 10), first_arrival);
    interpolator.SubmitFrame(
        SolidFrame(16, 16, 20),
        first_arrival + 20ms);
    ASSERT_TRUE(WaitUntil([&interpolator]() {
        const DisQueueStats stats = interpolator.GetQueueStats();
        return stats.worker_busy &&
               stats.pending_pairs == 0 &&
               stats.ready_sequences == 1;
    }));
    interpolator.SubmitFrame(
        SolidFrame(16, 16, 30),
        first_arrival + 40ms);

    auto blocked_submission = std::async(
        std::launch::async,
        [&interpolator, first_arrival]() {
            interpolator.SubmitFrame(
                SolidFrame(16, 16, 40),
                first_arrival + 60ms);
        });
    const bool submission_blocked =
        WaitUntil([&interpolator]() {
            const DisQueueStats stats =
                interpolator.GetQueueStats();
            return stats.source_backpressure_active;
        });

    interpolator.RequestStop();
    const bool submission_released =
        blocked_submission.wait_for(3s) ==
        std::future_status::ready;
    EXPECT_TRUE(submission_blocked);
    EXPECT_TRUE(submission_released);
    if (!submission_released) {
        blocked_submission.wait();
    }
    blocked_submission.get();
}

TEST(DisFrameInterpolatorTest, BidirectionalConsistencyPathBuildsSequence) {
    DisInterpolationConfig config;
    config.intermediate_frame_count = 1;
    config.flow_scale = 1.0;
    config.dis_preset = cv::DISOpticalFlow::PRESET_ULTRAFAST;
    config.use_bidirectional_flow = true;
    config.use_flow_consistency_mask = true;
    DisFrameInterpolator interpolator(config);

    cv::Mat first = SolidFrame(32, 32, 0);
    cv::Mat second = SolidFrame(32, 32, 0);
    first(cv::Rect(6, 10, 8, 8)).setTo(cv::Scalar(255, 255, 255));
    second(cv::Rect(10, 10, 8, 8)).setTo(cv::Scalar(255, 255, 255));

    const auto first_arrival = std::chrono::steady_clock::now();
    interpolator.SubmitFrame(first, first_arrival);

    cv::Mat displayed_first;
    ASSERT_TRUE(WaitForDisplayFrame(interpolator, displayed_first));

    interpolator.SubmitFrame(second, first_arrival + 20ms);

    cv::Mat intermediate;
    ASSERT_TRUE(WaitForDisplayFrame(
        interpolator, intermediate, 5s));
    EXPECT_EQ(intermediate.size(), first.size());
    EXPECT_EQ(intermediate.type(), CV_8UC3);

    cv::Mat endpoint;
    ASSERT_TRUE(WaitForDisplayFrame(interpolator, endpoint));
    EXPECT_EQ(cv::norm(endpoint, second, cv::NORM_INF), 0.0);

    // A second pair exercises reuse of the DIS instances, grid, maps, masks,
    // and warp buffers without changing their ownership semantics.
    interpolator.SubmitFrame(first, first_arrival + 40ms);

    cv::Mat reused_intermediate;
    ASSERT_TRUE(WaitForDisplayFrame(
        interpolator, reused_intermediate, 5s));
    EXPECT_EQ(reused_intermediate.size(), first.size());

    cv::Mat reused_endpoint;
    ASSERT_TRUE(WaitForDisplayFrame(interpolator, reused_endpoint));
    EXPECT_EQ(cv::norm(reused_endpoint, first, cv::NORM_INF), 0.0);
}

}  // namespace
}  // namespace pointcloud_visualization
