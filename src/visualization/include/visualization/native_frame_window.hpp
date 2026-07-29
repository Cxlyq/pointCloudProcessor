#pragma once

#include <memory>
#include <string>

#include <opencv2/core.hpp>

namespace pointcloud_visualization {

// A small same-thread image window used to bypass blocking OpenCV HighGUI
// backends. Windows uses GDI and Linux uses X11; unsupported platforms compile
// to a stub so that "auto" can retain the HighGUI fallback.
class NativeFrameWindow {
public:
    NativeFrameWindow();
    ~NativeFrameWindow();

    NativeFrameWindow(const NativeFrameWindow&) = delete;
    NativeFrameWindow& operator=(const NativeFrameWindow&) = delete;

    static bool IsSupported();
    static const char* BackendName();

    bool Create(
        const std::string& title,
        int client_width,
        int client_height);
    bool Present(const cv::Mat& bgr_frame);
    void PollEvents();
    void Destroy();

    bool IsOpen() const;
    const std::string& LastError() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace pointcloud_visualization
