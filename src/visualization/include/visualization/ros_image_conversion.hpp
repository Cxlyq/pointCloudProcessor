#pragma once

#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace pointcloud_visualization {

inline sensor_msgs::msg::Image BgrMatToImageMessage(
    const cv::Mat& bgr_frame) {
    if (bgr_frame.empty()) {
        throw std::invalid_argument(
            "cannot convert an empty BGR frame");
    }
    if (bgr_frame.type() != CV_8UC3) {
        throw std::invalid_argument(
            "BGR frame must use CV_8UC3 format");
    }
    if (bgr_frame.rows < 0 || bgr_frame.cols < 0) {
        throw std::invalid_argument(
            "BGR frame has invalid dimensions");
    }

    const auto width = static_cast<std::uint64_t>(bgr_frame.cols);
    const auto height = static_cast<std::uint64_t>(bgr_frame.rows);
    constexpr std::uint64_t kBytesPerPixel = 3;
    if (width >
            std::numeric_limits<std::uint32_t>::max() /
                kBytesPerPixel ||
        height > std::numeric_limits<std::uint32_t>::max()) {
        throw std::overflow_error(
            "BGR frame dimensions exceed sensor_msgs/Image");
    }

    const std::uint64_t row_bytes = width * kBytesPerPixel;
    if (height > 0 &&
        row_bytes >
            std::numeric_limits<std::size_t>::max() / height) {
        throw std::overflow_error(
            "BGR frame byte count exceeds addressable memory");
    }

    sensor_msgs::msg::Image message;
    message.height = static_cast<std::uint32_t>(height);
    message.width = static_cast<std::uint32_t>(width);
    message.encoding = sensor_msgs::image_encodings::BGR8;
    message.is_bigendian = false;
    message.step = static_cast<std::uint32_t>(row_bytes);
    message.data.resize(
        static_cast<std::size_t>(row_bytes * height));

    if (bgr_frame.isContinuous() &&
        bgr_frame.step == static_cast<std::size_t>(row_bytes)) {
        std::memcpy(
            message.data.data(),
            bgr_frame.data,
            message.data.size());
        return message;
    }

    for (int row = 0; row < bgr_frame.rows; ++row) {
        std::memcpy(
            message.data.data() +
                static_cast<std::size_t>(row) *
                    static_cast<std::size_t>(row_bytes),
            bgr_frame.ptr(row),
            static_cast<std::size_t>(row_bytes));
    }
    return message;
}

inline cv::Mat BgrImageMessageView(
    const sensor_msgs::msg::Image& message) {
    if (message.height == 0 || message.width == 0) {
        throw std::invalid_argument(
            "received an empty BGR image");
    }
    if (message.height >
            static_cast<std::uint32_t>(
                std::numeric_limits<int>::max()) ||
        message.width >
            static_cast<std::uint32_t>(
                std::numeric_limits<int>::max())) {
        throw std::overflow_error(
            "BGR image dimensions exceed OpenCV limits");
    }
    if (message.encoding != sensor_msgs::image_encodings::BGR8) {
        throw std::invalid_argument(
            "BGR image encoding must be bgr8, got " +
            message.encoding);
    }

    constexpr std::uint64_t kBytesPerPixel = 3;
    const std::uint64_t minimum_step =
        static_cast<std::uint64_t>(message.width) * kBytesPerPixel;
    if (message.step < minimum_step) {
        throw std::invalid_argument(
            "BGR image row step is too small");
    }
    if (message.height > 0 &&
        message.step >
            std::numeric_limits<std::size_t>::max() /
                message.height) {
        throw std::overflow_error(
            "BGR image byte count exceeds addressable memory");
    }

    const std::size_t required_bytes =
        static_cast<std::size_t>(message.step) *
        static_cast<std::size_t>(message.height);
    if (message.data.size() < required_bytes) {
        throw std::invalid_argument(
            "BGR image data is shorter than height * step");
    }

    // OpenCV's const-input APIs still require a mutable pointer when creating
    // a Mat header. The returned view must remain read-only and must not
    // outlive the sensor_msgs/Image that owns the storage.
    return cv::Mat(
        static_cast<int>(message.height),
        static_cast<int>(message.width),
        CV_8UC3,
        const_cast<std::uint8_t*>(message.data.data()),
        static_cast<std::size_t>(message.step));
}

}  // namespace pointcloud_visualization
