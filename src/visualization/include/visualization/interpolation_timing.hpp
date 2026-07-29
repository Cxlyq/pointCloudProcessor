#pragma once

#include <chrono>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace pointcloud_visualization {

inline std::chrono::steady_clock::duration
CalculateInterpolatedFramePeriod(
    std::chrono::steady_clock::duration source_interval_sum,
    std::size_t source_interval_count,
    std::size_t intermediate_frame_count) {
    if (source_interval_count == 0) {
        throw std::invalid_argument(
            "source interval count must be greater than zero");
    }
    if (intermediate_frame_count ==
        std::numeric_limits<std::size_t>::max()) {
        throw std::overflow_error(
            "intermediate frame count is too large");
    }

    using Duration = std::chrono::steady_clock::duration;
    using FloatingDuration =
        std::chrono::duration<long double, Duration::period>;
    const long double divisor =
        static_cast<long double>(source_interval_count) *
        static_cast<long double>(intermediate_frame_count + 1);
    const FloatingDuration floating_period{
        FloatingDuration(source_interval_sum).count() / divisor};
    auto frame_period =
        std::chrono::duration_cast<Duration>(floating_period);
    if (frame_period <= Duration::zero()) {
        frame_period =
            std::chrono::duration_cast<Duration>(
                std::chrono::milliseconds(1));
    }
    return frame_period;
}

}  // namespace pointcloud_visualization
