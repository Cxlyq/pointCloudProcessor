#include <algorithm>
#include <chrono>
#include <csignal>
#include <cctype>
#include <cstdio>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "pc_msgs/msg/o3_d_mesh.hpp"

// 子节点静态信息与运行时配置结构体
struct SubNodeConfig {
    std::string package_name;
    std::string launch_param_key;
    std::string default_launch_file;
    std::string enable_param_key;
    bool default_enable;
    std::string priority_param_key;
    int default_priority;
    std::string show_output_param_key;
    bool default_show_output;
    std::string log_level_param_key;
    std::string default_log_level;

    // 运行时参数与状态
    std::string launch_file{""};
    bool enabled{true};
    int priority{0};
    bool show_output{true};
    std::string log_level{"INFO"};
    pid_t pid{-1};
    int pipe_fd{-1};

    SubNodeConfig(std::string pkg, std::string l_key, std::string def_l,
                  std::string e_key, bool def_e,
                  std::string p_key, int def_p,
                  std::string s_key, bool def_s,
                  std::string log_key, std::string def_log)
        : package_name(std::move(pkg)),
          launch_param_key(std::move(l_key)),
          default_launch_file(std::move(def_l)),
          enable_param_key(std::move(e_key)),
          default_enable(def_e),
          priority_param_key(std::move(p_key)),
          default_priority(def_p),
          show_output_param_key(std::move(s_key)),
          default_show_output(def_s),
          log_level_param_key(std::move(log_key)),
          default_log_level(std::move(def_log)) {}
};

// 字符串工具函数
static inline std::string to_upper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::toupper(c); });
    return s;
}

static inline bool contains_ignore_case(const std::string & haystack, const std::string & needle) {
    auto it = std::search(
        haystack.begin(), haystack.end(),
        needle.begin(), needle.end(),
        [](char ch1, char ch2) { return std::toupper(static_cast<unsigned char>(ch1)) == std::toupper(static_cast<unsigned char>(ch2)); }
    );
    return (it != haystack.end());
}

// 终端节点名称颜色映射
static const char * get_pkg_color(const std::string & pkg) {
    if (pkg == "visualization")       return "\033[1;36m"; // Cyan
    if (pkg == "flow_alignment")      return "\033[1;35m"; // Magenta
    if (pkg == "reconstruction")      return "\033[1;34m"; // Blue
    if (pkg == "clustering")          return "\033[1;33m"; // Yellow
    if (pkg == "ground_segmentation") return "\033[1;32m"; // Green
    if (pkg == "io_handlers")         return "\033[1;31m"; // Red
    return "\033[1;37m";                                   // White
}

class BringupNode : public rclcpp::Node {
public:
    BringupNode() : Node("bringup_node") {
        // =====================================================================
        // 1. 声明并获取全局控制参数
        // =====================================================================
        this->declare_parameter<double>("launch_interval_sec", 1.0);
        this->declare_parameter<bool>("shutdown_in_reverse", false);
        this->declare_parameter<std::vector<std::string>>("custom_launch_order", std::vector<std::string>{});

        // 终端输出与日志等级过滤参数
        this->declare_parameter<bool>("global_show_output", true);
        this->declare_parameter<std::string>("global_log_level", "INFO");
        this->declare_parameter<bool>("enable_color_prefix", true);

        // 系统运行效率 / FPS 监控参数
        this->declare_parameter<bool>("enable_efficiency_monitor", true);
        this->declare_parameter<std::string>("efficiency_monitor_topic", "/flow/aligned_mesh");
        this->declare_parameter<double>("efficiency_report_interval_sec", 3.0);
        this->declare_parameter<std::string>("efficiency_log_signal_keyword", "[RX] Accepted mesh");

        double launch_interval = this->get_parameter("launch_interval_sec").as_double();
        shutdown_in_reverse_ = this->get_parameter("shutdown_in_reverse").as_bool();
        std::vector<std::string> custom_order = this->get_parameter("custom_launch_order").as_string_array();

        global_show_output_ = this->get_parameter("global_show_output").as_bool();
        global_log_level_ = to_upper(this->get_parameter("global_log_level").as_string());
        enable_color_prefix_ = this->get_parameter("enable_color_prefix").as_bool();

        enable_efficiency_monitor_ = this->get_parameter("enable_efficiency_monitor").as_bool();
        efficiency_monitor_topic_ = this->get_parameter("efficiency_monitor_topic").as_string();
        double report_interval = this->get_parameter("efficiency_report_interval_sec").as_double();
        efficiency_signal_keyword_ = this->get_parameter("efficiency_log_signal_keyword").as_string();

        // =====================================================================
        // 2. 定义 6 个子节点的静态信息与默认配置
        // 数据流顺序: io_handlers -> ground_segmentation -> clustering -> reconstruction -> flow_alignment -> visualization
        // 默认按数据流逆序设定优先级 1 ~ 6
        // =====================================================================
        std::vector<SubNodeConfig> all_nodes = {
            {"visualization",       "visualization_launch",       "v_vis.launch.py",                 "visualization_enable",       true, "visualization_priority",       1, "visualization_show_output",       true, "visualization_log_level",       ""},
            {"flow_alignment",      "flow_alignment_launch",      "flow_alignment.launch.py",        "flow_alignment_enable",      true, "flow_alignment_priority",      2, "flow_alignment_show_output",      true, "flow_alignment_log_level",      ""},
            {"reconstruction",      "reconstruction_launch",      "r_g_poisson_ng_obb_pt.launch.py", "reconstruction_enable",      true, "reconstruction_priority",      3, "reconstruction_show_output",      true, "reconstruction_log_level",      ""},
            {"clustering",          "clustering_launch",          "c_pdbscan.launch.py",             "clustering_enable",          true, "clustering_priority",          4, "clustering_show_output",          true, "clustering_log_level",          ""},
            {"ground_segmentation", "ground_segmentation_launch", "gs_gmz.launch.py",                "ground_segmentation_enable", true, "ground_segmentation_priority", 5, "ground_segmentation_show_output", true, "ground_segmentation_log_level", ""},
            {"io_handlers",         "io_handlers_launch",         "io_simPlaneDataFlow.launch.py",    "io_handlers_enable",         true, "io_handlers_priority",         6, "io_handlers_show_output",         true, "io_handlers_log_level",         ""}
        };

        // =====================================================================
        // 3. 声明各子节点独立参数并读取
        // =====================================================================
        for (auto & node_cfg : all_nodes) {
            this->declare_parameter<std::string>(node_cfg.launch_param_key, node_cfg.default_launch_file);
            this->declare_parameter<bool>(node_cfg.enable_param_key, node_cfg.default_enable);
            this->declare_parameter<int>(node_cfg.priority_param_key, node_cfg.default_priority);
            this->declare_parameter<bool>(node_cfg.show_output_param_key, node_cfg.default_show_output);
            this->declare_parameter<std::string>(node_cfg.log_level_param_key, node_cfg.default_log_level);

            node_cfg.launch_file = this->get_parameter(node_cfg.launch_param_key).as_string();
            node_cfg.enabled = this->get_parameter(node_cfg.enable_param_key).as_bool();
            node_cfg.priority = this->get_parameter(node_cfg.priority_param_key).as_int();

            // 若全局关闭输出，则以全局为主；否则尊重节点级 show_output
            bool node_show = this->get_parameter(node_cfg.show_output_param_key).as_bool();
            node_cfg.show_output = global_show_output_ && node_show;

            // 若节点未指定特定日志级别，则继承全局等级
            std::string n_level = this->get_parameter(node_cfg.log_level_param_key).as_string();
            node_cfg.log_level = n_level.empty() ? global_log_level_ : to_upper(n_level);
        }

        // =====================================================================
        // 4. 筛选已启用的节点
        // =====================================================================
        for (const auto & node_cfg : all_nodes) {
            if (node_cfg.enabled) {
                active_nodes_.push_back(node_cfg);
            } else {
                RCLCPP_INFO(this->get_logger(), "[-] Sub-node [%s] is DISABLED, skipping.", node_cfg.package_name.c_str());
            }
        }

        if (active_nodes_.empty()) {
            RCLCPP_WARN(this->get_logger(), "[!] No sub-nodes are enabled! Bringup node will remain idle.");
            return;
        }

        // =====================================================================
        // 5. 排序拉起顺序（custom_launch_order 或 priority）
        // =====================================================================
        if (!custom_order.empty()) {
            RCLCPP_INFO(this->get_logger(), "[*] Ordering nodes by custom_launch_order parameter.");
            std::vector<SubNodeConfig> sorted_nodes;
            for (const auto & name : custom_order) {
                auto it = std::find_if(active_nodes_.begin(), active_nodes_.end(),
                    [&name](const SubNodeConfig & c) { return c.package_name == name; });
                if (it != active_nodes_.end()) {
                    sorted_nodes.push_back(*it);
                    active_nodes_.erase(it);
                }
            }
            for (const auto & rem : active_nodes_) {
                sorted_nodes.push_back(rem);
            }
            active_nodes_ = std::move(sorted_nodes);
        } else {
            std::stable_sort(active_nodes_.begin(), active_nodes_.end(),
                [](const SubNodeConfig & a, const SubNodeConfig & b) {
                    return a.priority < b.priority;
                });
        }

        // 打印启动摘要信息
        std::string seq_str;
        for (size_t i = 0; i < active_nodes_.size(); ++i) {
            seq_str += active_nodes_[i].package_name + "(pri=" + std::to_string(active_nodes_[i].priority) +
                       ", out=" + (active_nodes_[i].show_output ? active_nodes_[i].log_level : "NONE") + ")";
            if (i + 1 < active_nodes_.size()) {
                seq_str += " -> ";
            }
        }

        RCLCPP_INFO(this->get_logger(), "=========================================================");
        RCLCPP_INFO(this->get_logger(), "[*] Master Bringup Node Initialized.");
        RCLCPP_INFO(this->get_logger(), "[*] Active sub-nodes count: %zu / %zu", active_nodes_.size(), all_nodes.size());
        RCLCPP_INFO(this->get_logger(), "[*] Global log filter: %s | Color prefix: %s",
                    global_log_level_.c_str(), enable_color_prefix_ ? "ON" : "OFF");
        RCLCPP_INFO(this->get_logger(), "[*] Launch sequence: %s", seq_str.c_str());
        RCLCPP_INFO(this->get_logger(), "=========================================================");

        // =====================================================================
        // 6. 依次拉起所有启用的子节点，并绑定输出过滤管道
        // =====================================================================
        for (size_t i = 0; i < active_nodes_.size(); ++i) {
            auto & node_cfg = active_nodes_[i];
            RCLCPP_INFO(this->get_logger(), "[%zu/%zu] Spawning %s (priority: %d, output: %s) launch: %s...",
                        i + 1, active_nodes_.size(), node_cfg.package_name.c_str(),
                        node_cfg.priority, node_cfg.show_output ? node_cfg.log_level.c_str() : "MUTED",
                        node_cfg.launch_file.c_str());

            int pfd[2] = {-1, -1};
            bool use_pipe = node_cfg.show_output && (node_cfg.log_level != "NONE");

            if (use_pipe) {
                if (pipe(pfd) < 0) {
                    RCLCPP_ERROR(this->get_logger(), "[!] Failed to create pipe for %s", node_cfg.package_name.c_str());
                    use_pipe = false;
                }
            }

            pid_t pid = fork();
            if (pid == 0) {
                // 子进程
                setpgid(0, 0);

                if (use_pipe) {
                    close(pfd[0]);
                    dup2(pfd[1], STDOUT_FILENO);
                    dup2(pfd[1], STDERR_FILENO);
                    close(pfd[1]);
                } else {
                    // 完全静音该节点输出：重定向到 /dev/null
                    int dev_null = open("/dev/null", O_WRONLY);
                    if (dev_null >= 0) {
                        dup2(dev_null, STDOUT_FILENO);
                        dup2(dev_null, STDERR_FILENO);
                        close(dev_null);
                    }
                }

                // 强制子进程行缓冲与颜色支持
                setenv("PYTHONUNBUFFERED", "1", 1);
                setenv("RCUTILS_COLORIZED_OUTPUT", "1", 1);

                execlp("ros2", "ros2", "launch", node_cfg.package_name.c_str(), node_cfg.launch_file.c_str(), (char *)NULL);
                perror("[!] Failed to exec ros2 launch");
                _exit(127);
            } else if (pid > 0) {
                // 父进程
                setpgid(pid, pid);
                node_cfg.pid = pid;
                RCLCPP_INFO(this->get_logger(), "[+] Successfully spawned %s (PID: %d)", node_cfg.package_name.c_str(), pid);

                if (use_pipe) {
                    close(pfd[1]);
                    node_cfg.pipe_fd = pfd[0];
                    reader_threads_.emplace_back(&BringupNode::read_child_output, this,
                                                 node_cfg.package_name, node_cfg.log_level, pfd[0]);
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "[!] Failed to fork process for %s", node_cfg.package_name.c_str());
                if (use_pipe) {
                    close(pfd[0]);
                    close(pfd[1]);
                }
            }

            if (launch_interval > 0.0 && i + 1 < active_nodes_.size()) {
                std::this_thread::sleep_for(std::chrono::duration<double>(launch_interval));
            }
        }

        RCLCPP_INFO(this->get_logger(), "=========================================================");
        RCLCPP_INFO(this->get_logger(), "[*] All %zu active sub-node(s) spawned.", active_nodes_.size());

        // =====================================================================
        // 7. 初始化系统总体运行效率 / 帧率监控
        // =====================================================================
        if (enable_efficiency_monitor_ && !efficiency_monitor_topic_.empty()) {
            mesh_sub_ = this->create_subscription<pc_msgs::msg::O3DMesh>(
                efficiency_monitor_topic_,
                rclcpp::QoS(rclcpp::KeepLast(1)),
                std::bind(&BringupNode::on_mesh_frame_received, this, std::placeholders::_1)
            );
            window_start_time_ = std::chrono::steady_clock::now();
            RCLCPP_INFO(this->get_logger(),
                "[*] Efficiency Monitor activated! Tracking topic: \033[1;32m%s\033[0m (report interval: %.1f s)",
                efficiency_monitor_topic_.c_str(), report_interval);

            efficiency_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(report_interval),
                std::bind(&BringupNode::report_efficiency, this)
            );
        }

        RCLCPP_INFO(this->get_logger(), "=========================================================");

        // 子进程生命周期轮询监控定时器
        monitor_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&BringupNode::monitor_children, this)
        );
    }

    ~BringupNode() override {
        stop_all_sub_nodes();
    }

    // 优雅退出关闭所有子节点
    void stop_all_sub_nodes() {
        if (stopped_) {
            return;
        }
        stopped_ = true;

        if (active_nodes_.empty()) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "=========================================================");
        RCLCPP_INFO(this->get_logger(), "[*] Shutting down Bringup Node. Closing sub-nodes %s...",
                    shutdown_in_reverse_ ? "in reverse launch order" : "sequentially");
        RCLCPP_INFO(this->get_logger(), "=========================================================");

        std::vector<size_t> stop_indices;
        stop_indices.reserve(active_nodes_.size());
        if (shutdown_in_reverse_) {
            for (size_t i = active_nodes_.size(); i > 0; --i) {
                stop_indices.push_back(i - 1);
            }
        } else {
            for (size_t i = 0; i < active_nodes_.size(); ++i) {
                stop_indices.push_back(i);
            }
        }

        for (size_t idx = 0; idx < stop_indices.size(); ++idx) {
            size_t i = stop_indices[idx];
            auto & node_cfg = active_nodes_[i];
            if (node_cfg.pid > 0) {
                RCLCPP_INFO(this->get_logger(), "[%zu/%zu] Stopping %s (PID %d)...",
                            idx + 1, stop_indices.size(), node_cfg.package_name.c_str(), node_cfg.pid);

                kill(-node_cfg.pid, SIGINT);

                int status = 0;
                int wait_count = 0;
                bool exited = false;
                while (wait_count < 30) {
                    pid_t res = waitpid(node_cfg.pid, &status, WNOHANG);
                    if (res == node_cfg.pid || res == -1) {
                        exited = true;
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    wait_count++;
                }

                if (!exited && kill(node_cfg.pid, 0) == 0) {
                    RCLCPP_WARN(this->get_logger(), "[!] Sub-node %s didn't stop in time. Force killing (SIGKILL)...",
                                node_cfg.package_name.c_str());
                    kill(-node_cfg.pid, SIGKILL);
                    waitpid(node_cfg.pid, &status, 0);
                }

                RCLCPP_INFO(this->get_logger(), "[-] Sub-node %s closed successfully.", node_cfg.package_name.c_str());
                node_cfg.pid = -1;
            }
        }

        // 等待所有管道读取线程退出
        for (auto & t : reader_threads_) {
            if (t.joinable()) {
                t.join();
            }
        }
        reader_threads_.clear();

        RCLCPP_INFO(this->get_logger(), "=========================================================");
        RCLCPP_INFO(this->get_logger(), "[*] All sub-nodes closed cleanly. Master bringup node exit.");
        RCLCPP_INFO(this->get_logger(), "=========================================================");
    }

private:
    // 判定某行日志在指定等级下是否应予显示
    bool should_display_log(const std::string & level_str, const std::string & line) {
        if (level_str == "NONE" || level_str == "OFF") {
            return false;
        }
        if (level_str == "ALL" || level_str == "DEBUG" || level_str == "INFO") {
            return true;
        }

        bool is_info = (line.find("[INFO]") != std::string::npos ||
                        line.find("[DEBUG]") != std::string::npos);
        bool is_warn = (line.find("[WARN]") != std::string::npos ||
                        line.find("[WARNING]") != std::string::npos);
        bool is_err  = (line.find("[ERROR]") != std::string::npos ||
                        line.find("[FATAL]") != std::string::npos ||
                        line.find("Exception") != std::string::npos ||
                        line.find("Traceback") != std::string::npos);

        if (level_str == "WARN") {
            // 过滤掉普通 INFO / DEBUG，保留 WARN、ERROR 以及异常
            if (is_err || is_warn) return true;
            if (is_info) return false;
            return contains_ignore_case(line, "warn") ||
                   contains_ignore_case(line, "error") ||
                   contains_ignore_case(line, "fail");
        }
        if (level_str == "ERROR") {
            // 过滤掉 INFO、DEBUG、WARN，仅保留 ERROR 以及异常
            if (is_err) return true;
            if (is_info || is_warn) return false;
            return contains_ignore_case(line, "error") ||
                   contains_ignore_case(line, "fatal") ||
                   contains_ignore_case(line, "fail");
        }

        return true;
    }

    // 子进程输出行读取与过滤线程
    void read_child_output(std::string pkg_name, std::string log_level, int fd) {
        FILE * fp = fdopen(fd, "r");
        if (!fp) {
            close(fd);
            return;
        }

        char * line_buf = nullptr;
        size_t line_len = 0;
        ssize_t n_read;
        std::string level = to_upper(log_level);

        while ((n_read = getline(&line_buf, &line_len, fp)) != -1) {
            if (n_read <= 0) continue;
            while (n_read > 0 && (line_buf[n_read - 1] == '\n' || line_buf[n_read - 1] == '\r')) {
                line_buf[--n_read] = '\0';
            }
            if (n_read == 0) continue;

            std::string line_str(line_buf, n_read);

            // 检查是否包含接收帧信号（如 [RX] Accepted mesh）
            if (!efficiency_signal_keyword_.empty() &&
                line_str.find(efficiency_signal_keyword_) != std::string::npos) {
                std::lock_guard<std::mutex> lock(stdout_mutex_);
                std::cout << "\033[1;35m[SYSTEM SIGNAL from " << pkg_name << "]\033[0m "
                          << line_str << std::endl;
            }

            // 日志等级过滤
            if (should_display_log(level, line_str)) {
                std::lock_guard<std::mutex> lock(stdout_mutex_);
                if (enable_color_prefix_) {
                    std::cout << get_pkg_color(pkg_name) << "[" << pkg_name << "]\033[0m "
                              << line_str << std::endl;
                } else {
                    std::cout << line_str << std::endl;
                }
            }
        }

        if (line_buf) {
            free(line_buf);
        }
        fclose(fp);
    }

    // 接收到末端 mesh 回调：计算总体处理帧率与端到端延迟
    void on_mesh_frame_received(const pc_msgs::msg::O3DMesh::SharedPtr msg) {
        auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(fps_mutex_);
        total_frames_++;
        window_frames_++;

        // 若上游携带了时间戳，计算端到端总延迟 (ms)
        if (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) {
            rclcpp::Time stamp(msg->header.stamp);
            double latency_ms = (this->now() - stamp).seconds() * 1000.0;
            if (latency_ms >= 0.0) {
                latency_sum_ms_ += latency_ms;
                min_latency_ms_ = std::min(min_latency_ms_, latency_ms);
                max_latency_ms_ = std::max(max_latency_ms_, latency_ms);
                latency_sample_count_++;
            }
        }

        // 瞬时帧率计算
        if (last_frame_time_.time_since_epoch().count() > 0) {
            double dt = std::chrono::duration<double>(now - last_frame_time_).count();
            if (dt > 0.0001) {
                instant_fps_ = 1.0 / dt;
            }
        }
        last_frame_time_ = now;
    }

    // 定期汇报系统运行效率
    void report_efficiency() {
        std::lock_guard<std::mutex> lock(fps_mutex_);
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - window_start_time_).count();
        if (elapsed < 0.5) return;

        if (window_frames_ > 0) {
            double avg_fps = static_cast<double>(window_frames_) / elapsed;
            if (latency_sample_count_ > 0) {
                double avg_lat = latency_sum_ms_ / latency_sample_count_;
                RCLCPP_INFO(this->get_logger(),
                    "\033[1;32m[SYSTEM EFFICIENCY]\033[0m Window: %.1fs | Throughput: \033[1;32m%.2f FPS\033[0m (Instant: %.2f FPS) | "
                    "E2E Latency: %.1f ms [min: %.1f, max: %.1f] | Frames: %zu (Total: %zu)",
                    elapsed, avg_fps, instant_fps_, avg_lat, min_latency_ms_, max_latency_ms_,
                    window_frames_, total_frames_);
            } else {
                RCLCPP_INFO(this->get_logger(),
                    "\033[1;32m[SYSTEM EFFICIENCY]\033[0m Window: %.1fs | Throughput: \033[1;32m%.2f FPS\033[0m (Instant: %.2f FPS) | "
                    "Frames: %zu (Total: %zu)",
                    elapsed, avg_fps, instant_fps_, window_frames_, total_frames_);
            }
        } else if (total_frames_ > 0) {
            RCLCPP_WARN(this->get_logger(),
                "\033[1;33m[SYSTEM EFFICIENCY]\033[0m No new frames received in the last %.1fs. Total: %zu",
                elapsed, total_frames_);
        }

        // 重置窗口计数
        window_frames_ = 0;
        window_start_time_ = now;
        latency_sum_ms_ = 0.0;
        latency_sample_count_ = 0;
        min_latency_ms_ = 1e9;
        max_latency_ms_ = 0.0;
    }

    // 监控子进程退出状态
    void monitor_children() {
        for (auto & node_cfg : active_nodes_) {
            if (node_cfg.pid > 0) {
                int status = 0;
                pid_t res = waitpid(node_cfg.pid, &status, WNOHANG);
                if (res == node_cfg.pid) {
                    if (WIFEXITED(status)) {
                        RCLCPP_WARN(this->get_logger(), "[!] Sub-node %s (PID %d) exited with code %d",
                                    node_cfg.package_name.c_str(), node_cfg.pid, WEXITSTATUS(status));
                    } else if (WIFSIGNALED(status)) {
                        RCLCPP_WARN(this->get_logger(), "[!] Sub-node %s (PID %d) was terminated by signal %d",
                                    node_cfg.package_name.c_str(), node_cfg.pid, WTERMSIG(status));
                    }
                    node_cfg.pid = -1;
                }
            }
        }
    }

    std::vector<SubNodeConfig> active_nodes_;
    std::vector<std::thread> reader_threads_;
    std::mutex stdout_mutex_;

    rclcpp::TimerBase::SharedPtr monitor_timer_;
    rclcpp::TimerBase::SharedPtr efficiency_timer_;
    rclcpp::Subscription<pc_msgs::msg::O3DMesh>::SharedPtr mesh_sub_;

    bool stopped_ = false;
    bool shutdown_in_reverse_ = false;

    // 终端控制配置
    bool global_show_output_ = true;
    std::string global_log_level_ = "INFO";
    bool enable_color_prefix_ = true;

    // 系统运行效率统计
    bool enable_efficiency_monitor_ = true;
    std::string efficiency_monitor_topic_ = "/flow/aligned_mesh";
    std::string efficiency_signal_keyword_ = "[RX] Accepted mesh";

    std::mutex fps_mutex_;
    size_t total_frames_ = 0;
    size_t window_frames_ = 0;
    double instant_fps_ = 0.0;
    std::chrono::steady_clock::time_point last_frame_time_;
    std::chrono::steady_clock::time_point window_start_time_;
    double latency_sum_ms_ = 0.0;
    size_t latency_sample_count_ = 0;
    double min_latency_ms_ = 1e9;
    double max_latency_ms_ = 0.0;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BringupNode>();

    rclcpp::on_shutdown([node]() {
        node->stop_all_sub_nodes();
    });

    rclcpp::spin(node);
    node->stop_all_sub_nodes();
    rclcpp::shutdown();
    return 0;
}
