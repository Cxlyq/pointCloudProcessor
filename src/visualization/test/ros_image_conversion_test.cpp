#include "visualization/display_frame_sequence.hpp"
#include "visualization/ros_image_conversion.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace pointcloud_visualization {
namespace {

TEST(DisplayFrameSequenceTest, RoundTripsSequenceNumbers) {
    const std::uint64_t maximum_sequence =
        std::numeric_limits<std::uint64_t>::max();

    const auto zero_sequence =
        DecodeDisplayFrameSequence(
            EncodeDisplayFrameSequence(0));
    ASSERT_TRUE(zero_sequence.has_value());
    EXPECT_EQ(*zero_sequence, 0U);

    const auto decoded_maximum_sequence =
        DecodeDisplayFrameSequence(
            EncodeDisplayFrameSequence(maximum_sequence));
    ASSERT_TRUE(decoded_maximum_sequence.has_value());
    EXPECT_EQ(
        *decoded_maximum_sequence,
        maximum_sequence);
}

TEST(DisplayFrameSequenceTest, RejectsMalformedFrameIds) {
    EXPECT_FALSE(
        DecodeDisplayFrameSequence("").has_value());
    EXPECT_FALSE(
        DecodeDisplayFrameSequence(
            "visualization/display_sequence/").has_value());
    EXPECT_FALSE(
        DecodeDisplayFrameSequence(
            "visualization/display_sequence/not-a-number").has_value());
    EXPECT_FALSE(
        DecodeDisplayFrameSequence(
            "visualization/display_sequence/12suffix").has_value());
    EXPECT_FALSE(
        DecodeDisplayFrameSequence(
            "visualization/display_sequence/"
            "18446744073709551616").has_value());
    EXPECT_FALSE(
        DecodeDisplayFrameSequence(
            "unrelated/12").has_value());
}

TEST(RosImageConversionTest, RoundTripsContinuousBgrFrame) {
    cv::Mat source(
        3,
        4,
        CV_8UC3,
        cv::Scalar(10, 20, 30));

    const sensor_msgs::msg::Image message =
        BgrMatToImageMessage(source);
    EXPECT_EQ(message.height, 3U);
    EXPECT_EQ(message.width, 4U);
    EXPECT_EQ(
        message.encoding,
        sensor_msgs::image_encodings::BGR8);
    EXPECT_EQ(message.step, 12U);
    EXPECT_EQ(message.data.size(), 36U);

    const cv::Mat view = BgrImageMessageView(message);
    ASSERT_EQ(view.type(), CV_8UC3);
    ASSERT_EQ(view.rows, source.rows);
    ASSERT_EQ(view.cols, source.cols);
    EXPECT_EQ(cv::norm(view, source, cv::NORM_INF), 0.0);
}

TEST(RosImageConversionTest, CopiesOnlyRoiPixelsWithoutParentStride) {
    cv::Mat parent(
        4,
        8,
        CV_8UC3,
        cv::Scalar(1, 2, 3));
    const cv::Mat roi = parent(cv::Rect(2, 1, 3, 2));
    ASSERT_FALSE(roi.isContinuous());

    const sensor_msgs::msg::Image message =
        BgrMatToImageMessage(roi);
    EXPECT_EQ(message.height, 2U);
    EXPECT_EQ(message.width, 3U);
    EXPECT_EQ(message.step, 9U);
    EXPECT_EQ(message.data.size(), 18U);

    const cv::Mat view = BgrImageMessageView(message);
    EXPECT_EQ(cv::norm(view, roi, cv::NORM_INF), 0.0);
}

TEST(RosImageConversionTest, AcceptsPaddedIncomingRows) {
    sensor_msgs::msg::Image message;
    message.height = 2;
    message.width = 2;
    message.encoding = sensor_msgs::image_encodings::BGR8;
    message.step = 8;
    message.data.assign(16, 0);
    message.data[0] = 10;
    message.data[1] = 20;
    message.data[2] = 30;
    message.data[8] = 40;
    message.data[9] = 50;
    message.data[10] = 60;

    const cv::Mat view = BgrImageMessageView(message);
    EXPECT_EQ(view.step, 8U);
    const cv::Vec3b first_pixel = view.at<cv::Vec3b>(0, 0);
    const cv::Vec3b second_pixel = view.at<cv::Vec3b>(1, 0);
    EXPECT_EQ(first_pixel[0], 10U);
    EXPECT_EQ(first_pixel[1], 20U);
    EXPECT_EQ(first_pixel[2], 30U);
    EXPECT_EQ(second_pixel[0], 40U);
    EXPECT_EQ(second_pixel[1], 50U);
    EXPECT_EQ(second_pixel[2], 60U);
}

TEST(RosImageConversionTest, RejectsMalformedMessages) {
    sensor_msgs::msg::Image wrong_encoding;
    wrong_encoding.height = 1;
    wrong_encoding.width = 1;
    wrong_encoding.encoding = "rgb8";
    wrong_encoding.step = 3;
    wrong_encoding.data.assign(3, 0);
    EXPECT_THROW(
        BgrImageMessageView(wrong_encoding),
        std::invalid_argument);

    sensor_msgs::msg::Image short_row;
    short_row.height = 1;
    short_row.width = 2;
    short_row.encoding = sensor_msgs::image_encodings::BGR8;
    short_row.step = 5;
    short_row.data.assign(5, 0);
    EXPECT_THROW(
        BgrImageMessageView(short_row),
        std::invalid_argument);

    sensor_msgs::msg::Image short_data;
    short_data.height = 2;
    short_data.width = 2;
    short_data.encoding = sensor_msgs::image_encodings::BGR8;
    short_data.step = 6;
    short_data.data.assign(11, 0);
    EXPECT_THROW(
        BgrImageMessageView(short_data),
        std::invalid_argument);
}

TEST(RosImageConversionTest, RejectsUnsupportedSourceType) {
    const cv::Mat grayscale(
        4,
        4,
        CV_8UC1,
        cv::Scalar(10));
    EXPECT_THROW(
        BgrMatToImageMessage(grayscale),
        std::invalid_argument);
}

}  // namespace
}  // namespace pointcloud_visualization
