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

const char* NativeFrameWindow::BackendName() {
    return "native-win32";
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

#elif defined(VISUALIZATION_NATIVE_X11)

#include <X11/Xlib.h>
#include <X11/Xutil.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>

#include <opencv2/imgproc.hpp>

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
                "native X11 window dimensions must be positive";
            return false;
        }
        if (display_ != nullptr) {
            last_error_ = "native X11 window is already open";
            return false;
        }

        display_ = XOpenDisplay(nullptr);
        if (display_ == nullptr) {
            const char* display_name = std::getenv("DISPLAY");
            last_error_ =
                "XOpenDisplay failed for DISPLAY=" +
                std::string(
                    display_name != nullptr ? display_name : "<unset>");
            return false;
        }

        screen_ = DefaultScreen(display_);
        visual_ = DefaultVisual(display_, screen_);
        depth_ = DefaultDepth(display_, screen_);
        if (visual_ == nullptr || depth_ <= 0) {
            last_error_ =
                "X11 default visual or color depth is unavailable";
            ReleaseResources();
            return false;
        }

        window_ = XCreateSimpleWindow(
            display_,
            RootWindow(display_, screen_),
            50,
            50,
            static_cast<unsigned int>(client_width),
            static_cast<unsigned int>(client_height),
            0,
            BlackPixel(display_, screen_),
            BlackPixel(display_, screen_));
        if (window_ == 0) {
            last_error_ = "XCreateSimpleWindow failed";
            ReleaseResources();
            return false;
        }

        XSelectInput(
            display_,
            window_,
            ExposureMask | StructureNotifyMask);
        XStoreName(display_, window_, title.c_str());

        wm_delete_window_ =
            XInternAtom(display_, "WM_DELETE_WINDOW", False);
        if (wm_delete_window_ != None) {
            Atom protocol = wm_delete_window_;
            XSetWMProtocols(display_, window_, &protocol, 1);
        }

        graphics_context_ =
            XCreateGC(display_, window_, 0, nullptr);
        if (graphics_context_ == nullptr) {
            last_error_ = "XCreateGC failed";
            ReleaseResources();
            return false;
        }

        client_width_ = client_width;
        client_height_ = client_height;
        is_open_ = true;
        XMapRaised(display_, window_);
        XFlush(display_);
        return true;
    }

    bool Present(const cv::Mat& bgr_frame) {
        last_error_.clear();
        if (!IsOpen()) {
            last_error_ = "native X11 window is closed";
            return false;
        }
        if (bgr_frame.empty() || bgr_frame.type() != CV_8UC3) {
            last_error_ =
                "native X11 window requires a non-empty CV_8UC3 frame";
            return false;
        }
        if (client_width_ <= 0 || client_height_ <= 0) {
            last_error_ = "native X11 window has an invalid client size";
            return false;
        }

        if (!EnsureImage(client_width_, client_height_)) {
            return false;
        }

        const cv::Mat* display_frame = &bgr_frame;
        if (bgr_frame.cols != client_width_ ||
            bgr_frame.rows != client_height_) {
            try {
                cv::resize(
                    bgr_frame,
                    resized_frame_,
                    cv::Size(client_width_, client_height_),
                    0.0,
                    0.0,
                    cv::INTER_LINEAR);
            } catch (const cv::Exception& error) {
                last_error_ =
                    std::string("X11 frame resize failed: ") +
                    error.what();
                return false;
            }
            display_frame = &resized_frame_;
        }

        if (!CopyFrameToImage(*display_frame)) {
            return false;
        }

        XPutImage(
            display_,
            window_,
            graphics_context_,
            image_,
            0,
            0,
            0,
            0,
            static_cast<unsigned int>(image_->width),
            static_cast<unsigned int>(image_->height));
        // XFlush submits the complete frame without waiting for a compositor
        // round trip. XPutImage owns a protocol-side copy, so the image buffer
        // can be safely reused by the next same-thread Present() call.
        XFlush(display_);
        return true;
    }

    void PollEvents() {
        if (display_ == nullptr) {
            return;
        }

        bool redraw_requested = false;
        while (XPending(display_) > 0) {
            XEvent event{};
            XNextEvent(display_, &event);
            if (event.xany.window != window_) {
                continue;
            }

            switch (event.type) {
                case ClientMessage:
                    if (wm_delete_window_ != None &&
                        static_cast<Atom>(
                            event.xclient.data.l[0]) ==
                            wm_delete_window_) {
                        CloseWindow();
                    }
                    break;
                case ConfigureNotify:
                    client_width_ =
                        std::max(1, event.xconfigure.width);
                    client_height_ =
                        std::max(1, event.xconfigure.height);
                    break;
                case Expose:
                    redraw_requested =
                        redraw_requested || event.xexpose.count == 0;
                    break;
                case DestroyNotify:
                    window_ = 0;
                    is_open_ = false;
                    break;
                default:
                    break;
            }
        }

        if (redraw_requested && IsOpen() && image_ != nullptr) {
            const unsigned int redraw_width =
                static_cast<unsigned int>(
                    std::min(client_width_, image_->width));
            const unsigned int redraw_height =
                static_cast<unsigned int>(
                    std::min(client_height_, image_->height));
            XPutImage(
                display_,
                window_,
                graphics_context_,
                image_,
                0,
                0,
                0,
                0,
                redraw_width,
                redraw_height);
            XFlush(display_);
        }
    }

    void Destroy() {
        ReleaseResources();
        last_error_.clear();
    }

    bool IsOpen() const {
        return display_ != nullptr &&
            window_ != 0 &&
            is_open_;
    }

    const std::string& LastError() const {
        return last_error_;
    }

private:
    struct ChannelEncoding {
        unsigned int shift = 0;
        unsigned long maximum = 0;
        bool valid = false;
    };

    static ChannelEncoding DescribeMask(unsigned long mask) {
        ChannelEncoding encoding;
        if (mask == 0) {
            return encoding;
        }

        while ((mask & 1UL) == 0UL) {
            ++encoding.shift;
            mask >>= 1U;
        }
        if ((mask & (mask + 1UL)) != 0UL) {
            return encoding;
        }

        encoding.maximum = mask;
        encoding.valid = true;
        return encoding;
    }

    static unsigned long EncodeChannel(
        std::uint8_t value,
        const ChannelEncoding& encoding) {
        const auto scaled =
            (static_cast<unsigned long long>(value) *
                 encoding.maximum +
             127ULL) /
            255ULL;
        return static_cast<unsigned long>(
            scaled << encoding.shift);
    }

    bool BuildColorLookup() {
        const ChannelEncoding red =
            DescribeMask(image_->red_mask);
        const ChannelEncoding green =
            DescribeMask(image_->green_mask);
        const ChannelEncoding blue =
            DescribeMask(image_->blue_mask);
        if (!red.valid || !green.valid || !blue.valid) {
            last_error_ =
                "native X11 window requires contiguous RGB color masks";
            return false;
        }

        for (std::size_t value = 0;
             value < red_lookup_.size();
             ++value) {
            const auto channel =
                static_cast<std::uint8_t>(value);
            red_lookup_[value] = EncodeChannel(channel, red);
            green_lookup_[value] = EncodeChannel(channel, green);
            blue_lookup_[value] = EncodeChannel(channel, blue);
        }
        return true;
    }

    bool EnsureImage(int width, int height) {
        if (image_ != nullptr &&
            image_->width == width &&
            image_->height == height) {
            return true;
        }

        DestroyImage();
        image_ = XCreateImage(
            display_,
            visual_,
            static_cast<unsigned int>(depth_),
            ZPixmap,
            0,
            nullptr,
            static_cast<unsigned int>(width),
            static_cast<unsigned int>(height),
            32,
            0);
        if (image_ == nullptr) {
            last_error_ = "XCreateImage failed";
            return false;
        }

        if (image_->bits_per_pixel != 16 &&
            image_->bits_per_pixel != 24 &&
            image_->bits_per_pixel != 32) {
            last_error_ =
                "unsupported X11 image pixel format: " +
                std::to_string(image_->bits_per_pixel) +
                " bits per pixel";
            DestroyImage();
            return false;
        }
        pixel_bytes_ = image_->bits_per_pixel / 8;
        if (image_->bytes_per_line <= 0 ||
            width >
                image_->bytes_per_line / pixel_bytes_) {
            last_error_ = "invalid X11 image scanline stride";
            DestroyImage();
            return false;
        }

        const auto bytes_per_line =
            static_cast<std::size_t>(image_->bytes_per_line);
        const auto image_height =
            static_cast<std::size_t>(height);
        if (image_height >
            std::numeric_limits<std::size_t>::max() /
                bytes_per_line) {
            last_error_ = "native X11 image size overflow";
            DestroyImage();
            return false;
        }

        image_->data = static_cast<char*>(
            std::calloc(image_height, bytes_per_line));
        if (image_->data == nullptr) {
            last_error_ =
                "failed to allocate native X11 image buffer";
            DestroyImage();
            return false;
        }
        if (!BuildColorLookup()) {
            DestroyImage();
            return false;
        }
        direct_bgrx_copy_ =
            pixel_bytes_ == 4 &&
            image_->byte_order == LSBFirst &&
            image_->red_mask == 0x00ff0000UL &&
            image_->green_mask == 0x0000ff00UL &&
            image_->blue_mask == 0x000000ffUL;
        direct_bgr_copy_ =
            pixel_bytes_ == 3 &&
            image_->byte_order == LSBFirst &&
            image_->red_mask == 0x00ff0000UL &&
            image_->green_mask == 0x0000ff00UL &&
            image_->blue_mask == 0x000000ffUL;
        return true;
    }

    bool CopyFrameToImage(const cv::Mat& bgr_frame) {
        if (direct_bgrx_copy_) {
            cv::Mat x11_frame(
                image_->height,
                image_->width,
                CV_8UC4,
                image_->data,
                static_cast<std::size_t>(image_->bytes_per_line));
            try {
                cv::cvtColor(
                    bgr_frame,
                    x11_frame,
                    cv::COLOR_BGR2BGRA);
            } catch (const cv::Exception& error) {
                last_error_ =
                    std::string("X11 BGR to BGRX conversion failed: ") +
                    error.what();
                return false;
            }
            return true;
        }

        if (direct_bgr_copy_) {
            const auto row_bytes =
                static_cast<std::size_t>(bgr_frame.cols) * 3U;
            for (int row = 0; row < bgr_frame.rows; ++row) {
                std::memcpy(
                    image_->data +
                        static_cast<std::ptrdiff_t>(row) *
                            image_->bytes_per_line,
                    bgr_frame.ptr<std::uint8_t>(row),
                    row_bytes);
            }
            return true;
        }

        const bool least_significant_byte_first =
            image_->byte_order == LSBFirst;
        for (int row = 0; row < bgr_frame.rows; ++row) {
            const std::uint8_t* source =
                bgr_frame.ptr<std::uint8_t>(row);
            auto* destination =
                reinterpret_cast<std::uint8_t*>(
                    image_->data +
                    static_cast<std::ptrdiff_t>(row) *
                        image_->bytes_per_line);
            for (int column = 0;
                 column < bgr_frame.cols;
                 ++column) {
                const std::uint8_t blue = source[column * 3];
                const std::uint8_t green = source[column * 3 + 1];
                const std::uint8_t red = source[column * 3 + 2];
                const unsigned long pixel =
                    red_lookup_[red] |
                    green_lookup_[green] |
                    blue_lookup_[blue];
                std::uint8_t* pixel_destination =
                    destination + column * pixel_bytes_;

                if (least_significant_byte_first) {
                    for (int byte = 0;
                         byte < pixel_bytes_;
                         ++byte) {
                        pixel_destination[byte] =
                            static_cast<std::uint8_t>(
                                pixel >> (byte * 8));
                    }
                } else {
                    for (int byte = 0;
                         byte < pixel_bytes_;
                         ++byte) {
                        pixel_destination[byte] =
                            static_cast<std::uint8_t>(
                                pixel >>
                                ((pixel_bytes_ - 1 - byte) * 8));
                    }
                }
            }
        }
        return true;
    }

    void CloseWindow() {
        if (display_ != nullptr && window_ != 0) {
            const Window closing_window = window_;
            window_ = 0;
            is_open_ = false;
            XDestroyWindow(display_, closing_window);
            XFlush(display_);
        }
    }

    void DestroyImage() {
        if (image_ != nullptr) {
            XDestroyImage(image_);
            image_ = nullptr;
        }
        pixel_bytes_ = 0;
        direct_bgrx_copy_ = false;
        direct_bgr_copy_ = false;
        resized_frame_.release();
    }

    void ReleaseResources() {
        DestroyImage();
        if (display_ != nullptr && graphics_context_ != nullptr) {
            XFreeGC(display_, graphics_context_);
        }
        graphics_context_ = nullptr;
        if (display_ != nullptr && window_ != 0) {
            XDestroyWindow(display_, window_);
        }
        window_ = 0;
        is_open_ = false;
        if (display_ != nullptr) {
            XCloseDisplay(display_);
        }
        display_ = nullptr;
        visual_ = nullptr;
        depth_ = 0;
        client_width_ = 0;
        client_height_ = 0;
        wm_delete_window_ = None;
    }

    Display* display_ = nullptr;
    int screen_ = 0;
    Visual* visual_ = nullptr;
    int depth_ = 0;
    Window window_ = 0;
    GC graphics_context_ = nullptr;
    Atom wm_delete_window_ = None;
    int client_width_ = 0;
    int client_height_ = 0;
    bool is_open_ = false;
    XImage* image_ = nullptr;
    int pixel_bytes_ = 0;
    bool direct_bgrx_copy_ = false;
    bool direct_bgr_copy_ = false;
    cv::Mat resized_frame_;
    std::array<unsigned long, 256> red_lookup_{};
    std::array<unsigned long, 256> green_lookup_{};
    std::array<unsigned long, 256> blue_lookup_{};
    std::string last_error_;
};

NativeFrameWindow::NativeFrameWindow()
    : impl_(std::make_unique<Impl>()) {}

NativeFrameWindow::~NativeFrameWindow() = default;

bool NativeFrameWindow::IsSupported() {
    return true;
}

const char* NativeFrameWindow::BackendName() {
    return "native-x11";
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
        "native frame window is not enabled for this platform";
};

NativeFrameWindow::NativeFrameWindow()
    : impl_(std::make_unique<Impl>()) {}

NativeFrameWindow::~NativeFrameWindow() = default;

bool NativeFrameWindow::IsSupported() {
    return false;
}

const char* NativeFrameWindow::BackendName() {
    return "native-unavailable";
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
