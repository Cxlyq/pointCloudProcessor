#include "visualization/native_frame_window.hpp"

#ifdef _WIN32

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <opencv2/imgproc.hpp>

namespace {

constexpr wchar_t kFrameWindowClassName[] =
    L"PointCloudProcessor.NativeFrameWindow.v1";

std::wstring utf8_to_wide(const std::string& text) {
    if (text.empty()) {
        return {};
    }

    const int character_count = MultiByteToWideChar(
        CP_UTF8,
        MB_ERR_INVALID_CHARS,
        text.data(),
        static_cast<int>(text.size()),
        nullptr,
        0);
    if (character_count <= 0) {
        return std::wstring(text.begin(), text.end());
    }

    std::wstring wide_text(
        static_cast<std::size_t>(character_count), L'\0');
    MultiByteToWideChar(
        CP_UTF8,
        MB_ERR_INVALID_CHARS,
        text.data(),
        static_cast<int>(text.size()),
        wide_text.data(),
        character_count);
    return wide_text;
}

std::string windows_error(const std::string& operation) {
    const DWORD error_code = GetLastError();
    return operation + " failed with Windows error " +
        std::to_string(static_cast<unsigned long>(error_code));
}

}  // namespace

namespace pointcloud_visualization {

class NativeFrameWindow::Impl {
public:
    ~Impl() {
        Destroy();
    }

    bool Create(
        const std::string& title,
        int client_width,
        int client_height) {
        last_error_.clear();
        if (client_width <= 0 || client_height <= 0) {
            last_error_ =
                "native frame window dimensions must be positive";
            return false;
        }
        if (window_ != nullptr) {
            last_error_ = "native frame window is already open";
            return false;
        }

        instance_ = GetModuleHandleW(nullptr);
        if (instance_ == nullptr) {
            last_error_ = windows_error("GetModuleHandleW");
            return false;
        }
        if (!RegisterWindowClass()) {
            return false;
        }

        constexpr DWORD style = WS_OVERLAPPEDWINDOW;
        RECT window_rect{0, 0, client_width, client_height};
        if (AdjustWindowRectEx(
                &window_rect, style, FALSE, 0) == FALSE) {
            last_error_ = windows_error("AdjustWindowRectEx");
            return false;
        }

        const std::wstring wide_title = utf8_to_wide(title);
        window_ = CreateWindowExW(
            0,
            kFrameWindowClassName,
            wide_title.c_str(),
            style,
            CW_USEDEFAULT,
            CW_USEDEFAULT,
            window_rect.right - window_rect.left,
            window_rect.bottom - window_rect.top,
            nullptr,
            nullptr,
            instance_,
            this);
        if (window_ == nullptr) {
            last_error_ = windows_error("CreateWindowExW");
            return false;
        }

        ShowWindow(window_, SW_SHOW);
        UpdateWindow(window_);
        return true;
    }

    bool Present(const cv::Mat& bgr_frame) {
        last_error_.clear();
        if (!IsOpen()) {
            last_error_ = "native frame window is closed";
            return false;
        }
        if (bgr_frame.empty() || bgr_frame.type() != CV_8UC3) {
            last_error_ =
                "native frame window requires a non-empty CV_8UC3 frame";
            return false;
        }

        try {
            cv::cvtColor(
                bgr_frame, bgra_frame_, cv::COLOR_BGR2BGRA);
        } catch (const cv::Exception& error) {
            last_error_ =
                std::string("BGR to BGRA conversion failed: ") +
                error.what();
            return false;
        }

        bitmap_info_ = {};
        bitmap_info_.bmiHeader.biSize =
            sizeof(bitmap_info_.bmiHeader);
        bitmap_info_.bmiHeader.biWidth = bgra_frame_.cols;
        // A negative height requests a top-down DIB, matching cv::Mat.
        bitmap_info_.bmiHeader.biHeight = -bgra_frame_.rows;
        bitmap_info_.bmiHeader.biPlanes = 1;
        bitmap_info_.bmiHeader.biBitCount = 32;
        bitmap_info_.bmiHeader.biCompression = BI_RGB;

        paint_succeeded_ = false;
        if (RedrawWindow(
                window_,
                nullptr,
                nullptr,
                RDW_INVALIDATE | RDW_UPDATENOW | RDW_NOERASE) ==
            FALSE) {
            last_error_ = windows_error("RedrawWindow");
            return false;
        }
        if (!paint_succeeded_) {
            last_error_ = "native frame window paint did not complete";
            return false;
        }
        return true;
    }

    void PollEvents() {
        if (window_ == nullptr) {
            return;
        }

        MSG message;
        while (PeekMessageW(
                   &message, window_, 0, 0, PM_REMOVE) != FALSE) {
            TranslateMessage(&message);
            DispatchMessageW(&message);
            if (window_ == nullptr) {
                break;
            }
        }
    }

    void Destroy() {
        if (window_ != nullptr && IsWindow(window_) != FALSE) {
            DestroyWindow(window_);
        }
        window_ = nullptr;
        bgra_frame_.release();
    }

    bool IsOpen() const {
        return window_ != nullptr &&
            IsWindow(window_) != FALSE;
    }

    const std::string& LastError() const {
        return last_error_;
    }

private:
    bool RegisterWindowClass() {
        WNDCLASSEXW existing_class{};
        existing_class.cbSize = sizeof(existing_class);
        if (GetClassInfoExW(
                instance_,
                kFrameWindowClassName,
                &existing_class) != FALSE) {
            return true;
        }

        WNDCLASSEXW window_class{};
        window_class.cbSize = sizeof(window_class);
        window_class.style = CS_HREDRAW | CS_VREDRAW;
        window_class.lpfnWndProc = &Impl::WindowProcedure;
        window_class.hInstance = instance_;
        window_class.hCursor =
            LoadCursorW(nullptr, IDC_ARROW);
        window_class.hbrBackground =
            reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
        window_class.lpszClassName =
            kFrameWindowClassName;

        if (RegisterClassExW(&window_class) == 0) {
            const DWORD error_code = GetLastError();
            if (error_code != ERROR_CLASS_ALREADY_EXISTS) {
                last_error_ =
                    "RegisterClassExW failed with Windows error " +
                    std::to_string(
                        static_cast<unsigned long>(error_code));
                return false;
            }
        }
        return true;
    }

    void Paint() {
        PAINTSTRUCT paint{};
        HDC device_context = BeginPaint(window_, &paint);
        if (device_context == nullptr) {
            paint_succeeded_ = false;
            return;
        }

        RECT client_rect{};
        GetClientRect(window_, &client_rect);
        const int client_width =
            client_rect.right - client_rect.left;
        const int client_height =
            client_rect.bottom - client_rect.top;

        if (bgra_frame_.empty() ||
            client_width <= 0 ||
            client_height <= 0) {
            FillRect(
                device_context,
                &client_rect,
                reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1));
            paint_succeeded_ = true;
        } else {
            SetStretchBltMode(device_context, COLORONCOLOR);
            const int copied_scan_lines = StretchDIBits(
                device_context,
                0,
                0,
                client_width,
                client_height,
                0,
                0,
                bgra_frame_.cols,
                bgra_frame_.rows,
                bgra_frame_.data,
                &bitmap_info_,
                DIB_RGB_COLORS,
                SRCCOPY);
            paint_succeeded_ =
                copied_scan_lines != 0 &&
                copied_scan_lines != GDI_ERROR;
        }

        EndPaint(window_, &paint);
    }

    static LRESULT CALLBACK WindowProcedure(
        HWND window,
        UINT message,
        WPARAM w_parameter,
        LPARAM l_parameter) {
        Impl* implementation = reinterpret_cast<Impl*>(
            GetWindowLongPtrW(window, GWLP_USERDATA));
        if (message == WM_NCCREATE) {
            const auto* create = reinterpret_cast<CREATESTRUCTW*>(
                l_parameter);
            implementation =
                static_cast<Impl*>(create->lpCreateParams);
            SetWindowLongPtrW(
                window,
                GWLP_USERDATA,
                reinterpret_cast<LONG_PTR>(implementation));
            implementation->window_ = window;
        }

        if (implementation != nullptr) {
            switch (message) {
                case WM_PAINT:
                    implementation->Paint();
                    return 0;
                case WM_ERASEBKGND:
                    return 1;
                case WM_CLOSE:
                    DestroyWindow(window);
                    return 0;
                case WM_NCDESTROY:
                    implementation->window_ = nullptr;
                    SetWindowLongPtrW(
                        window, GWLP_USERDATA, 0);
                    break;
                default:
                    break;
            }
        }

        return DefWindowProcW(
            window, message, w_parameter, l_parameter);
    }

    HINSTANCE instance_ = nullptr;
    HWND window_ = nullptr;
    cv::Mat bgra_frame_;
    BITMAPINFO bitmap_info_{};
    bool paint_succeeded_ = false;
    std::string last_error_;
};

NativeFrameWindow::NativeFrameWindow()
    : impl_(std::make_unique<Impl>()) {}

NativeFrameWindow::~NativeFrameWindow() = default;

bool NativeFrameWindow::IsSupported() {
    return true;
}

bool NativeFrameWindow::Create(
    const std::string& title,
    int client_width,
    int client_height) {
    return impl_->Create(title, client_width, client_height);
}

bool NativeFrameWindow::Present(const cv::Mat& bgr_frame) {
    return impl_->Present(bgr_frame);
}

void NativeFrameWindow::PollEvents() {
    impl_->PollEvents();
}

void NativeFrameWindow::Destroy() {
    impl_->Destroy();
}

bool NativeFrameWindow::IsOpen() const {
    return impl_->IsOpen();
}

const std::string& NativeFrameWindow::LastError() const {
    return impl_->LastError();
}

}  // namespace pointcloud_visualization

#else

namespace pointcloud_visualization {

class NativeFrameWindow::Impl {
public:
    std::string last_error =
        "native frame window is only available on Windows";
};

NativeFrameWindow::NativeFrameWindow()
    : impl_(std::make_unique<Impl>()) {}

NativeFrameWindow::~NativeFrameWindow() = default;

bool NativeFrameWindow::IsSupported() {
    return false;
}

bool NativeFrameWindow::Create(
    const std::string&,
    int,
    int) {
    return false;
}

bool NativeFrameWindow::Present(const cv::Mat&) {
    return false;
}

void NativeFrameWindow::PollEvents() {}

void NativeFrameWindow::Destroy() {}

bool NativeFrameWindow::IsOpen() const {
    return false;
}

const std::string& NativeFrameWindow::LastError() const {
    return impl_->last_error;
}

}  // namespace pointcloud_visualization

#endif
