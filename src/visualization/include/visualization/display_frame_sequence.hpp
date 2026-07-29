#pragma once

#include <charconv>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>

namespace pointcloud_visualization {

constexpr std::string_view kDisplayFrameSequencePrefix =
    "visualization/display_sequence/";

inline std::string EncodeDisplayFrameSequence(std::uint64_t sequence) {
    return std::string(kDisplayFrameSequencePrefix) +
           std::to_string(sequence);
}

inline std::optional<std::uint64_t> DecodeDisplayFrameSequence(
    std::string_view frame_id) {
    if (frame_id.size() <= kDisplayFrameSequencePrefix.size() ||
        frame_id.substr(0, kDisplayFrameSequencePrefix.size()) !=
            kDisplayFrameSequencePrefix) {
        return std::nullopt;
    }

    const std::string_view digits =
        frame_id.substr(kDisplayFrameSequencePrefix.size());
    std::uint64_t sequence = 0;
    const auto parse_result = std::from_chars(
        digits.data(),
        digits.data() + digits.size(),
        sequence);
    if (parse_result.ec != std::errc{} ||
        parse_result.ptr != digits.data() + digits.size()) {
        return std::nullopt;
    }
    return sequence;
}

}  // namespace pointcloud_visualization
