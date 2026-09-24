#include <chrono>
#include <csignal>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

struct SubNodeConfig {
    std::string package_name;
    std::string param_key;
    std::string default_launch_file;
    std::string launch_file;
    pid_t pid = -1;
};

class BringupNode : public rclcpp::Node {
public:
    BringupNode() : Node("bringup_node") {
        // 1. 声明参数及其默认启动文件名
        this->declare_parameter<std::string>("visualization_launch", "v_dis_bidirectional.launch.py");
        this->declare_parameter<std::string>("reconstruction_launch", "r_g_poisson_ng_obb.launch.py");
        this->declare_parameter<std::string>("clustering_launch", "c_pdbscan.launch.py");
        this->declare_parameter<std::string>("ground_segmentation_launch", "gs_gmz.launch.py");
        this->declare_parameter<std::string>("io_handlers_launch", "io_simPlaneDataFlow.launch.py");
        this->declare_parameter<double>("launch_interval_sec", 1.0);

        double launch_interval = this->get_parameter("launch_interval_sec").as_double();

        // 严格按照要求的顺序定义 5 个子节点：
        // 依次拉起: visualization -> reconstruction -> clustering -> ground_segmentation -> io_handlers
        sub_nodes_ = {
            {"visualization",       "visualization_launch",       "v_dis_bidirectional.launch.py", "", -1},
            {"reconstruction",      "reconstruction_launch",      "r_g_poisson_ng_obb.launch.py",  "", -1},
            {"clustering",          "clustering_launch",          "c_pdbscan.launch.py",           "", -1},
            {"ground_segmentation", "ground_segmentation_launch", "gs_gmz.launch.py",              "", -1},
            {"io_handlers",         "io_handlers_launch",         "io_simPlaneDataFlow.launch.py",  "", -1}
        };

        for (auto & node_cfg : sub_nodes_) {
            node_cfg.launch_file = this->get_parameter(node_cfg.param_key).as_string();
        }

        RCLCPP_INFO(this->get_logger(), "=========================================================");
        RCLCPP_INFO(this->get_logger(), "[*] Master Bringup Node Initialized.");
        RCLCPP_INFO(this->get_logger(), "[*] Launch sequence: visualization -> reconstruction -> clustering -> ground_segmentation -> io_handlers");
        RCLCPP_INFO(this->get_logger(), "=========================================================");

        // 2. 依次拉起 5 个子节点
        for (size_t i = 0; i < sub_nodes_.size(); ++i) {
            auto & node_cfg = sub_nodes_[i];
            RCLCPP_INFO(this->get_logger(), "[%zu/5] Spawning %s launch file: %s...",
                        i + 1, node_cfg.package_name.c_str(), node_cfg.launch_file.c_str());

            pid_t pid = fork();
            if (pid == 0) {
                // 子进程：放入独立进程组并执行 ros2 launch
                setpgid(0, 0);
                execlp("ros2", "ros2", "launch", node_cfg.package_name.c_str(), node_cfg.launch_file.c_str(), (char *)NULL);
                perror("[!] Failed to exec ros2 launch");
                _exit(127);
            } else if (pid > 0) {
                // 父进程：记录 PID
                setpgid(pid, pid);
                node_cfg.pid = pid;
                RCLCPP_INFO(this->get_logger(), "[+] Successfully spawned %s (PID: %d)", node_cfg.package_name.c_str(), pid);
            } else {
                RCLCPP_ERROR(this->get_logger(), "[!] Failed to fork process for %s", node_cfg.package_name.c_str());
            }

            if (launch_interval > 0.0 && i + 1 < sub_nodes_.size()) {
                std::this_thread::sleep_for(std::chrono::duration<double>(launch_interval));
            }
        }

        RCLCPP_INFO(this->get_logger(), "=========================================================");
        RCLCPP_INFO(this->get_logger(), "[*] All 5 sub-nodes spawned successfully. Monitoring child processes...");
        RCLCPP_INFO(this->get_logger(), "=========================================================");

        // 定时轮询监控子进程状态
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&BringupNode::monitor_children, this)
        );
    }

    ~BringupNode() override {
        stop_all_sub_nodes();
    }

    // 依次关闭所有子节点
    void stop_all_sub_nodes() {
        if (stopped_) {
            return;
        }
        stopped_ = true;

        RCLCPP_INFO(this->get_logger(), "=========================================================");
        RCLCPP_INFO(this->get_logger(), "[*] Shutting down Bringup Node. Closing sub-nodes sequentially...");
        RCLCPP_INFO(this->get_logger(), "=========================================================");

        // 依次关闭节点：visualization -> reconstruction -> clustering -> ground_segmentation -> io_handlers
        for (size_t i = 0; i < sub_nodes_.size(); ++i) {
            auto & node_cfg = sub_nodes_[i];
            if (node_cfg.pid > 0) {
                RCLCPP_INFO(this->get_logger(), "[%zu/5] Stopping %s (PID %d)...",
                            i + 1, node_cfg.package_name.c_str(), node_cfg.pid);

                // 向进程组发送 SIGINT (模拟 Ctrl+C 优雅退出)
                kill(-node_cfg.pid, SIGINT);

                // 等待进程退出
                int status = 0;
                int wait_count = 0;
                bool exited = false;
                while (wait_count < 30) { // 最多等待 3 秒
                    pid_t res = waitpid(node_cfg.pid, &status, WNOHANG);
                    if (res == node_cfg.pid || res == -1) {
                        exited = true;
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    wait_count++;
                }

                // 若在超时时间内未正常退出，强制使用 SIGKILL 杀死
                if (!exited && kill(node_cfg.pid, 0) == 0) {
                    RCLCPP_WARN(this->get_logger(), "[!] Sub-node %s didn't stop in time. Force killing (SIGKILL)...", node_cfg.package_name.c_str());
                    kill(-node_cfg.pid, SIGKILL);
                    waitpid(node_cfg.pid, &status, 0);
                }

                RCLCPP_INFO(this->get_logger(), "[-] Sub-node %s closed successfully.", node_cfg.package_name.c_str());
                node_cfg.pid = -1;
            }
        }

        RCLCPP_INFO(this->get_logger(), "=========================================================");
        RCLCPP_INFO(this->get_logger(), "[*] All sub-nodes closed cleanly. Master bringup node exit.");
        RCLCPP_INFO(this->get_logger(), "=========================================================");
    }

private:
    void monitor_children() {
        for (auto & node_cfg : sub_nodes_) {
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

    std::vector<SubNodeConfig> sub_nodes_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool stopped_ = false;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BringupNode>();

    // 注册关闭回调，确保 Ctrl+C 时依次关闭所有子节点
    rclcpp::on_shutdown([node]() {
        node->stop_all_sub_nodes();
    });

    rclcpp::spin(node);
    node->stop_all_sub_nodes();
    rclcpp::shutdown();
    return 0;
}
