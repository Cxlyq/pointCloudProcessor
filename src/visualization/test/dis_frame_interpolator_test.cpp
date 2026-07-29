#include "visualization/dis_frame_interpolator.hpp"
#include "visualization/interpolation_timing.hpp"

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

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

TEST(InterpolationTimingTest, CoalescingUsesAverageSourceInterval) {
    const auto normal_period = CalculateInterpolatedFramePeriod(
        std::chrono::seconds(2), 1, 4);
    const auto coalesced_period = CalculateInterpolatedFramePeriod(
        std::chrono::seconds(4), 2, 4);

    EXPECT_EQ(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            normal_period).count(),
        400);
    EXPECT_EQ(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            coalesced_period).count(),
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

TEST(DisFrameInterpolatorTest, RejectsNonPositiveMaximumPlaybackLag) {
    DisInterpolationConfig config;
    config.max_playback_lag =
        std::chrono::steady_clock::duration::zero();

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

TEST(DisFrameInterpolatorTest, PresentsOnlyLatestCurrentlyDueFrame) {
    DisInterpolationConfig config;
    config.intermediate_frame_count = 4;
    config.flow_scale = 1.0;
    config.dis_preset = cv::DISOpticalFlow::PRESET_ULTRAFAST;
    config.max_playback_lag = 5s;
    DisFrameInterpolator interpolator(config);

    const auto first_arrival = std::chrono::steady_clock::now();
    interpolator.SubmitFrame(
        SolidFrame(24, 24, 0), first_arrival);

    cv::Mat first;
    ASSERT_TRUE(WaitForDisplayFrame(interpolator, first));

    interpolator.SubmitFrame(
        SolidFrame(24, 24, 200),
        first_arrival + 100ms);
    ASSERT_TRUE(WaitUntil([&interpolator]() {
        const DisQueueStats stats = interpolator.GetQueueStats();
        return stats.ready_sequences == 1 && !stats.worker_busy;
    }));

    cv::Mat not_due;
    DisDisplayTiming not_due_timing;
    EXPECT_FALSE(interpolator.TryGetDisplayFrame(
        not_due, &not_due_timing));
    const auto deadline =
        interpolator.GetNextDisplayDeadline();
    ASSERT_TRUE(deadline.has_value());

    std::this_thread::sleep_for(55ms);
    cv::Mat latest_due;
    DisDisplayTiming timing;
    ASSERT_TRUE(interpolator.TryGetDisplayFrame(
        latest_due, &timing));
    EXPECT_FALSE(timing.latency_reset);
    EXPECT_GE(timing.skipped_frames, 1U);
    EXPECT_FALSE(latest_due.empty());
    EXPECT_EQ(latest_due.type(), CV_8UC3);

    const DisQueueStats stats = interpolator.GetQueueStats();
    EXPECT_EQ(
        stats.skipped_display_frames,
        timing.skipped_frames);
    EXPECT_EQ(stats.latency_resets, 0U);
}

TEST(DisFrameInterpolatorTest, ExcessiveLagResetsToLatestRealFrame) {
    DisInterpolationConfig config;
    config.intermediate_frame_count = 4;
    config.flow_scale = 1.0;
    config.dis_preset = cv::DISOpticalFlow::PRESET_ULTRAFAST;
    config.max_playback_lag = 20ms;
    DisFrameInterpolator interpolator(config);

    const auto first_arrival = std::chrono::steady_clock::now();
    interpolator.SubmitFrame(
        SolidFrame(24, 24, 10), first_arrival);

    cv::Mat first;
    ASSERT_TRUE(WaitForDisplayFrame(interpolator, first));

    const cv::Mat second = SolidFrame(24, 24, 110);
    interpolator.SubmitFrame(
        second, first_arrival + 50ms);
    ASSERT_TRUE(WaitUntil([&interpolator]() {
        const DisQueueStats stats = interpolator.GetQueueStats();
        return stats.ready_sequences == 1 && !stats.worker_busy;
    }));

    cv::Mat not_due;
    EXPECT_FALSE(interpolator.TryGetDisplayFrame(not_due));
    std::this_thread::sleep_for(45ms);

    cv::Mat reset_frame;
    DisDisplayTiming timing;
    ASSERT_TRUE(interpolator.TryGetDisplayFrame(
        reset_frame, &timing));
    EXPECT_TRUE(timing.latency_reset);
    EXPECT_GT(timing.lag_before_reset_ms, 20.0);
    EXPECT_GT(timing.skipped_frames, 0U);
    EXPECT_EQ(cv::norm(reset_frame, second, cv::NORM_INF), 0.0);

    const DisQueueStats reset_stats =
        interpolator.GetQueueStats();
    EXPECT_EQ(reset_stats.latency_resets, 1U);
    EXPECT_EQ(
        reset_stats.skipped_display_frames,
        timing.skipped_frames);
    EXPECT_EQ(reset_stats.pending_pairs, 0U);
    EXPECT_EQ(reset_stats.ready_sequences, 0U);
    EXPECT_EQ(reset_stats.active_frames_remaining, 0U);
    EXPECT_FALSE(
        interpolator.GetNextDisplayDeadline().has_value());

    // The reset display copy must not alias the retained endpoint, which
    // still seeds the next pair.
    reset_frame.setTo(cv::Scalar(240, 240, 240));
    interpolator.SubmitFrame(
        SolidFrame(24, 24, 110),
        first_arrival + 100ms);
    cv::Mat recovered;
    ASSERT_TRUE(WaitForDisplayFrame(interpolator, recovered));
    EXPECT_NEAR(cv::mean(recovered)[0], 110.0, 1.0);
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

TEST(DisFrameInterpolatorTest, QueueStatsExposeDeterministicCoalescing) {
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
    // deterministically blocked while publishing the second frame.
    ASSERT_TRUE(WaitUntil([&interpolator]() {
        const DisQueueStats stats = interpolator.GetQueueStats();
        return stats.worker_busy &&
               stats.pending_pairs == 0 &&
               stats.ready_sequences == 1;
    }));

    interpolator.SubmitFrame(
        SolidFrame(16, 16, 30),
        first_arrival + 40ms);
    interpolator.SubmitFrame(
        SolidFrame(16, 16, 40),
        first_arrival + 60ms);

    const DisQueueStats stats = interpolator.GetQueueStats();
    EXPECT_TRUE(stats.worker_busy);
    EXPECT_EQ(stats.pending_pairs, 1U);
    EXPECT_EQ(stats.ready_sequences, 1U);
    EXPECT_EQ(stats.coalesced_source_frames, 1U);
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
