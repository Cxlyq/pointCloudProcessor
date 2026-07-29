#include "visualization/ros_image_conversion.hpp"

#include <cstddef>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace {

constexpr char kInterpolatedFrameTopic[] = "interpolated_frames";

struct WindowState {
    std::string camera_id;
    std::string window_name;
    sensor_msgs::msg::Image::ConstSharedPtr pending_frame;
    sensor_msgs::msg::Image::ConstSharedPtr displayed_frame;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
        subscription;
    bool created = false;
};

class InterpolationDisplayNode : public rclcpp::Node {
public:
    InterpolationDisplayNode()
        : Node("interpolation_display_node") {
        const auto camera_ids =
            this->declare_parameter<std::vector<std::string>>(
                "camera_ids", {"cam_front"});
        if (camera_ids.empty()) {
            throw std::invalid_argument(
                "camera_ids must contain at least one camera");
        }
        const std::string window_suffix =
            this->declare_parameter<std::string>(
                "interpolation_window_suffix",
                " - DIS Interpolated");

        windows_.reserve(camera_ids.size());
        std::unordered_set<std::string> unique_camera_ids;
        std::unordered_set<std::string> unique_window_names;
        for (const auto& camera_id : camera_ids) {
            if (camera_id.empty() ||
                !unique_camera_ids.insert(camera_id).second) {
                throw std::invalid_argument(
                    "camera_ids must be non-empty and unique");
            }

            const std::string source_window_name =
                this->declare_parameter<std::string>(
                    camera_id + ".name",
                    "Render - " + camera_id);
            const std::string interpolation_window_name =
                source_window_name + window_suffix;
            if (source_window_name.empty() ||
                !unique_window_names.insert(
                    interpolation_window_name).second) {
                throw std::invalid_argument(
                    "interpolation window names must be non-empty "
                    "and unique");
            }

            WindowState state;
            state.camera_id = camera_id;
            state.window_name =
                interpolation_window_name;
            windows_.push_back(std::move(state));
        }

        rclcpp::QoS image_qos(rclcpp::KeepLast(1));
        image_qos.best_effort();
        for (std::size_t index = 0;
             index < windows_.size();
             ++index) {
            WindowState& state = windows_.at(index);
            const std::string input_topic =
                std::string(kInterpolatedFrameTopic) +
                "/" + state.camera_id;
            state.subscription =
                this->create_subscription<
                    sensor_msgs::msg::Image>(
                    input_topic,
                    image_qos,
                    [this, index](
                        sensor_msgs::msg::Image::ConstSharedPtr message) {
                        if (message == nullptr) {
                            return;
                        }
                        windows_.at(index).pending_frame =
                            std::move(message);
                    });

            RCLCPP_INFO(
                this->get_logger(),
                "[*] Interpolation display \"%s\" is listening on "
                "\"%s\".",
                state.window_name.c_str(),
                input_topic.c_str());
        }
        RCLCPP_INFO(
            this->get_logger(),
            "[*] All HighGUI calls run on this process's main thread.");
    }

    ~InterpolationDisplayNode() override {
        DestroyWindows();
    }

    bool PresentReadyFramesAndProcessEvents() {
        for (auto& state : windows_) {
            if (state.pending_frame == nullptr) {
                continue;
            }

            state.displayed_frame =
                std::move(state.pending_frame);
            try {
                const cv::Mat frame =
                    pointcloud_visualization::
                        BgrImageMessageView(
                            *state.displayed_frame);
                if (!state.created) {
                    cv::namedWindow(
                        state.window_name,
                        cv::WINDOW_NORMAL);
                    // Record ownership immediately: if resizeWindow throws,
                    // shutdown must still destroy the window just created.
                    state.created = true;
                    cv::resizeWindow(
                        state.window_name,
                        frame.cols,
                        frame.rows);
                }
                cv::imshow(state.window_name, frame);
            } catch (const std::exception& error) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "[!] Rejected interpolated frame for \"%s\": %s",
                    state.window_name.c_str(),
                    error.what());
            }
        }

        const int key = cv::waitKey(1);
        return key != 27 && key != 'q' && key != 'Q';
    }

    void DestroyWindows() noexcept {
        for (auto& state : windows_) {
            if (!state.created) {
                continue;
            }
            try {
                cv::destroyWindow(state.window_name);
            } catch (const std::exception& error) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "[?] Failed to destroy interpolation window "
                    "\"%s\": %s",
                    state.window_name.c_str(),
                    error.what());
            } catch (...) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "[?] Failed to destroy interpolation window "
                    "\"%s\" with an unknown exception.",
                    state.window_name.c_str());
            }
            state.created = false;
        }
    }

private:
    std::vector<WindowState> windows_;
};

}  // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    int exit_code = 0;
    std::shared_ptr<InterpolationDisplayNode> node;

    try {
        node = std::make_shared<InterpolationDisplayNode>();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);

        while (rclcpp::ok()) {
            // spin_some invokes every image callback on this main thread.
            executor.spin_some();
            if (!node->PresentReadyFramesAndProcessEvents()) {
                RCLCPP_INFO(
                    node->get_logger(),
                    "[*] Interpolation display requested shutdown.");
                break;
            }
        }

        executor.remove_node(node);
        node->DestroyWindows();
    } catch (const std::exception& error) {
        RCLCPP_FATAL(
            rclcpp::get_logger("interpolation_display_node"),
            "[!] Interpolation display failed: %s",
            error.what());
        exit_code = 1;
    } catch (...) {
        RCLCPP_FATAL(
            rclcpp::get_logger("interpolation_display_node"),
            "[!] Interpolation display failed with an unknown exception.");
        exit_code = 1;
    }

    node.reset();
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return exit_code;
}
