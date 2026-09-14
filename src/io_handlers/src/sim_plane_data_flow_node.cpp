//
// sim_plane_data_flow_node.cpp
// Created by cx on 9/14/26.
//
// 该节点负责从数据集目录中读取:
//   1. 雷达点云数据（.ply）—— 按顺序预加载后以固定频率向 topic 发布
//   2. 载机状态数据（extend_data.json 中的 gps_info）—— 与点云同步打时间戳向另一 topic 发布
//   3. 高压塔 GPS 数据（powerLineGPS.json）—— 以 Service 形式对外提供，
//      接收载机经纬度后返回附近一定距离内的高压塔位置信息
//

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// nlohmann/json 是一个 header-only 库，可通过 apt 安装 nlohmann-json3-dev
#include "../include/nlohmann/json.hpp"

#include "open3d/Open3D.h"
#include "pc_msgs/msg/o3_d_point_cloud.hpp"
#include "pc_msgs/msg/plane_state.hpp"
#include "pc_msgs/msg/power_tower_gps.hpp"
#include "pc_msgs/srv/query_nearby_towers.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;
using json = nlohmann::json;

// ─────────────────────────────────────────────────────────────
// 辅助数据结构
// ─────────────────────────────────────────────────────────────

// 缓存一帧点云及其对应位置信息
struct FrameData {
    std::string file_name;
    std::vector<float> points;
    std::vector<float> colors;
    size_t num_points{0};

    // 对应的载机状态
    double latitude{0.0};
    double longitude{0.0};
    double altitude{0.0};
    double course{0.0};
};

// 高压塔记录
struct TowerRecord {
    double latitude{0.0};
    double longitude{0.0};
    double altitude{0.0};
};

// ─────────────────────────────────────────────────────────────
// 地理计算：Haversine 公式（单位：千米）
// ─────────────────────────────────────────────────────────────
static constexpr double kEarthRadiusKm = 6371.0;

static double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

static double haversine_km(double lat1, double lon1, double lat2, double lon2) {
    const double dlat = deg2rad(lat2 - lat1);
    const double dlon = deg2rad(lon2 - lon1);
    const double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
                     std::cos(deg2rad(lat1)) * std::cos(deg2rad(lat2)) *
                     std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
    const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return kEarthRadiusKm * c;
}

// ─────────────────────────────────────────────────────────────
// 节点类
// ─────────────────────────────────────────────────────────────
class SimPlaneDataFlowNode : public rclcpp::Node {
public:
    SimPlaneDataFlowNode() : Node("sim_plane_data_flow_node"), current_idx_(0) {
        // ── 1. 声明参数 ──────────────────────────────────────
        this->declare_parameter<std::string>("dataset_dir", "");
        this->declare_parameter<double>("publish_rate", 2.0);
        this->declare_parameter<std::string>("pointcloud_topic", "/io/plane/raw_pointcloud");
        this->declare_parameter<std::string>("plane_state_topic", "/io/plane/state");
        this->declare_parameter<std::string>("query_service_name", "/io/query_nearby_towers");
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<bool>("is_loop", false);
        this->declare_parameter<double>("default_query_radius_km", 5.0);

        dataset_dir_           = this->get_parameter("dataset_dir").as_string();
        publish_rate_          = this->get_parameter("publish_rate").as_double();
        pointcloud_topic_      = this->get_parameter("pointcloud_topic").as_string();
        plane_state_topic_     = this->get_parameter("plane_state_topic").as_string();
        query_service_name_    = this->get_parameter("query_service_name").as_string();
        frame_id_              = this->get_parameter("frame_id").as_string();
        is_loop_               = this->get_parameter("is_loop").as_bool();
        default_query_radius_km_ = this->get_parameter("default_query_radius_km").as_double();

        // ── 2. 校验数据集目录 ─────────────────────────────────
        if (dataset_dir_.empty() || !fs::exists(dataset_dir_)) {
            RCLCPP_ERROR(this->get_logger(), "[!] Invalid dataset_dir: '%s'", dataset_dir_.c_str());
            return;
        }

        // ── 3. 解析 extend_data.json ─────────────────────────
        if (!parse_extend_data()) {
            RCLCPP_ERROR(this->get_logger(), "[!] Failed to parse extend_data.json");
            return;
        }

        if (frames_data_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "[!] No valid frame data found after parsing.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "[*] Preloaded %zu frames (point cloud + GPS).", frames_data_.size());

        // ── 4. 解析 powerLineGPS.json ─────────────────────────
        if (!parse_power_line_gps()) {
            RCLCPP_WARN(this->get_logger(), "[?] Failed to parse powerLineGPS.json; tower query service will return empty results.");
        } else {
            RCLCPP_INFO(this->get_logger(), "[*] Loaded %zu power towers in total.", all_towers_.size());
        }

        // ── 5. 创建发布者 ─────────────────────────────────────
        pc_publisher_ = this->create_publisher<pc_msgs::msg::O3DPointCloud>(pointcloud_topic_, 10);
        state_publisher_ = this->create_publisher<pc_msgs::msg::PlaneState>(plane_state_topic_, 10);

        // ── 6. 创建 Service 服务端 ────────────────────────────
        tower_service_ = this->create_service<pc_msgs::srv::QueryNearbyTowers>(
            query_service_name_,
            std::bind(&SimPlaneDataFlowNode::handle_tower_query, this,
                      std::placeholders::_1, std::placeholders::_2)
        );
        RCLCPP_INFO(this->get_logger(), "[*] Tower query service ready at '%s'.", query_service_name_.c_str());

        // ── 7. 创建定时器，驱动点云+状态发布 ──────────────────
        auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
            std::bind(&SimPlaneDataFlowNode::timer_callback, this)
        );
        RCLCPP_INFO(this->get_logger(), "[*] Start publishing at %.1f Hz on topics '%s' & '%s'.",
                    publish_rate_, pointcloud_topic_.c_str(), plane_state_topic_.c_str());
    }

private:
    // ── 解析 extend_data.json，按 ply_filename 顺序预加载点云和 GPS ──
    bool parse_extend_data() {
        const fs::path json_path = fs::path(dataset_dir_) / "extend_data.json";
        if (!fs::exists(json_path)) {
            RCLCPP_ERROR(this->get_logger(), "[!] extend_data.json not found at: %s", json_path.c_str());
            return false;
        }

        std::ifstream ifs(json_path);
        if (!ifs.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "[!] Cannot open extend_data.json");
            return false;
        }

        json j;
        try {
            ifs >> j;
        } catch (const json::parse_error& e) {
            RCLCPP_ERROR(this->get_logger(), "[!] JSON parse error in extend_data.json: %s", e.what());
            return false;
        }

        // 收集 <ply_filename, gps_info> 对，并按 ply_filename 排序保证时序
        struct Entry {
            std::string ply_filename;
            double latitude{0.0}, longitude{0.0}, altitude{0.0}, course{0.0};
        };
        std::vector<Entry> entries;

        for (auto& [key, val] : j.items()) {
            try {
                Entry entry;
                entry.ply_filename = val.at("ply_filename").get<std::string>();
                const auto& gps = val.at("gps_info");
                entry.latitude   = gps.at("Latitude").get<double>();
                entry.longitude  = gps.at("Longitude").get<double>();
                entry.altitude   = gps.at("Altitude").get<double>();
                entry.course     = gps.at("Course").get<double>();
                entries.push_back(entry);
            } catch (const json::exception& e) {
                RCLCPP_WARN(this->get_logger(), "[?] Skipping entry '%s': %s", key.c_str(), e.what());
            }
        }

        // 按 ply 文件名升序排序，保证时序一致
        std::sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b) {
            return a.ply_filename < b.ply_filename;
        });

        // 预加载点云数据
        size_t load_idx = 0;
        for (const auto& entry : entries) {
            const fs::path ply_path = fs::path(dataset_dir_) / entry.ply_filename;
            if (!fs::exists(ply_path)) {
                RCLCPP_WARN(this->get_logger(), "[?] PLY file not found, skipping: %s", ply_path.c_str());
                ++load_idx;
                continue;
            }

            try {
                auto pcd = open3d::io::CreatePointCloudFromFile(ply_path.string());
                if (!pcd || pcd->points_.empty()) {
                    RCLCPP_WARN(this->get_logger(), "[?] Empty point cloud, skipping: %s", entry.ply_filename.c_str());
                    ++load_idx;
                    continue;
                }

                FrameData frame;
                frame.file_name  = entry.ply_filename;
                frame.num_points = pcd->points_.size();
                frame.latitude   = entry.latitude;
                frame.longitude  = entry.longitude;
                frame.altitude   = entry.altitude;
                frame.course     = entry.course;

                frame.points.reserve(frame.num_points * 3);
                for (const auto& pt : pcd->points_) {
                    frame.points.push_back(static_cast<float>(pt.x()));
                    frame.points.push_back(static_cast<float>(pt.y()));
                    frame.points.push_back(static_cast<float>(pt.z()));
                }

                if (pcd->HasColors()) {
                    frame.colors.reserve(frame.num_points * 3);
                    for (const auto& c : pcd->colors_) {
                        frame.colors.push_back(static_cast<float>(c.x()));
                        frame.colors.push_back(static_cast<float>(c.y()));
                        frame.colors.push_back(static_cast<float>(c.z()));
                    }
                }

                frames_data_.push_back(std::move(frame));

                ++load_idx;
                if (load_idx % 10 == 0 || load_idx == entries.size()) {
                    RCLCPP_INFO(this->get_logger(), "[*] Preload progress: %zu/%zu ...", load_idx, entries.size());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "[!] Error loading PLY '%s': %s",
                             entry.ply_filename.c_str(), e.what());
                ++load_idx;
            }
        }

        return !frames_data_.empty();
    }

    // ── 解析 powerLineGPS.json，展平所有高压线的所有高压塔 ──
    bool parse_power_line_gps() {
        const fs::path json_path = fs::path(dataset_dir_) / "powerLineGPS.json";
        if (!fs::exists(json_path)) {
            RCLCPP_WARN(this->get_logger(), "[?] powerLineGPS.json not found at: %s", json_path.c_str());
            return false;
        }

        std::ifstream ifs(json_path);
        if (!ifs.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "[!] Cannot open powerLineGPS.json");
            return false;
        }

        json j;
        try {
            ifs >> j;
        } catch (const json::parse_error& e) {
            RCLCPP_ERROR(this->get_logger(), "[!] JSON parse error in powerLineGPS.json: %s", e.what());
            return false;
        }

        // 遍历每条高压线 HVL_n，展平所有高压塔记录
        for (auto& [line_name, towers_arr] : j.items()) {
            if (!towers_arr.is_array()) {
                RCLCPP_WARN(this->get_logger(), "[?] Unexpected format for '%s', skipping.", line_name.c_str());
                continue;
            }
            for (const auto& tower_json : towers_arr) {
                try {
                    TowerRecord rec;
                    rec.latitude  = tower_json.at("Latitude").get<double>();
                    rec.longitude = tower_json.at("Longitude").get<double>();
                    rec.altitude  = tower_json.at("Altitude").get<double>();
                    all_towers_.push_back(rec);
                } catch (const json::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "[?] Skipping malformed tower in '%s': %s",
                                line_name.c_str(), e.what());
                }
            }
        }

        return !all_towers_.empty();
    }

    // ── 定时器回调：发布点云 + 载机状态 ──────────────────────
    void timer_callback() {
        if (current_idx_ >= frames_data_.size()) {
            if (is_loop_) {
                current_idx_ = 0;
            } else {
                RCLCPP_INFO(this->get_logger(), "[*] All frames have been published. Stopping timer.");
                timer_->cancel();
                return;
            }
        }

        const auto& frame = frames_data_[current_idx_];
        const auto now_stamp = this->get_clock()->now();

        // —— 发布点云消息 ——
        auto pc_msg = pc_msgs::msg::O3DPointCloud();
        pc_msg.header.stamp    = now_stamp;
        pc_msg.header.frame_id = frame_id_;
        pc_msg.points          = frame.points;
        if (!frame.colors.empty()) {
            pc_msg.colors = frame.colors;
        }
        pc_publisher_->publish(pc_msg);

        // —— 发布载机状态消息（与点云同一时间戳）——
        auto state_msg = pc_msgs::msg::PlaneState();
        state_msg.header.stamp    = now_stamp;
        state_msg.header.frame_id = frame_id_;
        state_msg.latitude        = frame.latitude;
        state_msg.longitude       = frame.longitude;
        state_msg.altitude        = frame.altitude;
        state_msg.course          = frame.course;
        state_publisher_->publish(state_msg);

        RCLCPP_INFO(this->get_logger(),
                    "[*] Frame[%zu] '%s' published | Points: %zu | GPS: (%.9f, %.9f, %.4f) Course: %.6f",
                    current_idx_, frame.file_name.c_str(), frame.num_points,
                    frame.latitude, frame.longitude, frame.altitude, frame.course);

        ++current_idx_;
    }

    // ── Service 回调：查询附近高压塔 ─────────────────────────
    void handle_tower_query(
        const std::shared_ptr<pc_msgs::srv::QueryNearbyTowers::Request> request,
        std::shared_ptr<pc_msgs::srv::QueryNearbyTowers::Response> response)
    {
        const double query_lat = request->latitude;
        const double query_lon = request->longitude;
        const double radius_km = (request->radius_km > 0.0)
                                 ? request->radius_km
                                 : default_query_radius_km_;

        RCLCPP_INFO(this->get_logger(),
                    "[*] Tower query: lat=%.9f lon=%.9f radius=%.3f km",
                    query_lat, query_lon, radius_km);

        response->latitudes.clear();
        response->longitudes.clear();
        response->altitudes.clear();

        if (all_towers_.empty()) {
            response->success = false;
            response->message = "No tower data loaded.";
            RCLCPP_WARN(this->get_logger(), "[?] Tower query: no tower data available.");
            return;
        }

        size_t found_count = 0;
        for (const auto& tower : all_towers_) {
            const double dist = haversine_km(query_lat, query_lon, tower.latitude, tower.longitude);
            if (dist <= radius_km) {
                response->latitudes.push_back(tower.latitude);
                response->longitudes.push_back(tower.longitude);
                response->altitudes.push_back(tower.altitude);
                ++found_count;
            }
        }

        response->success = true;
        response->message = "Found " + std::to_string(found_count) + " tower(s) within "
                            + std::to_string(radius_km) + " km.";
        RCLCPP_INFO(this->get_logger(), "[*] Tower query result: %s", response->message.c_str());
    }

    // ── 成员变量 ──────────────────────────────────────────────
    std::string dataset_dir_;
    double      publish_rate_;
    std::string pointcloud_topic_;
    std::string plane_state_topic_;
    std::string query_service_name_;
    std::string frame_id_;
    bool        is_loop_;
    double      default_query_radius_km_;

    std::vector<FrameData>   frames_data_;
    std::vector<TowerRecord> all_towers_;
    size_t current_idx_;

    rclcpp::Publisher<pc_msgs::msg::O3DPointCloud>::SharedPtr pc_publisher_;
    rclcpp::Publisher<pc_msgs::msg::PlaneState>::SharedPtr     state_publisher_;
    rclcpp::Service<pc_msgs::srv::QueryNearbyTowers>::SharedPtr tower_service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// ─────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimPlaneDataFlowNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
