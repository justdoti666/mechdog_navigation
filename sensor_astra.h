/**
 * Astra Pro 深度相机驱动模块 (C++ 版)
 * 基于奥比中光 Astra Pro (单目结构光)
 * 通过 OpenNI2 获取深度图/RGB图
 */
#pragma once

#include "config.h"
#include <string>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <random>
#include <vector>
#include <thread>
#include <atomic>

namespace mechdog {

/** 环境类型枚举 */
enum class EnvironmentType {
    INDOOR,
    SEMI_INDOOR,
    OUTDOOR,
    UNKNOWN
};

/** 深度图中区域分析结果 */
struct DepthRegion {
    double center_distance_m  = 8.0;
    double min_distance_m     = 8.0;
    double max_distance_m     = 8.0;
    int    obstacle_count     = 0;
    double valid_pixel_ratio  = 0.0;
    double quality_score      = 0.0;
};

/** Astra Pro 单帧数据 */
struct AstraFrame {
    double        timestamp        = 0.0;
    bool          valid            = true;
    EnvironmentType environment     = EnvironmentType::UNKNOWN;
    double        ambient_light_level = 0.0;

    DepthRegion   center_region;
    DepthRegion   left_region;
    DepthRegion   right_region;

    // 深度图数据 (模拟模式使用)
    std::vector<uint16_t> depth_map;
    int depth_width  = 640;
    int depth_height = 480;
};

/** Astra Pro 深度相机驱动 */
class AstraProDriver {
public:
    static constexpr int    DEPTH_WIDTH         = 640;
    static constexpr int    DEPTH_HEIGHT        = 480;
    static constexpr int    DEPTH_FPS           = 30;
    static constexpr double MIN_VALID_DISTANCE_MM = 600.0;
    static constexpr double MAX_VALID_DISTANCE_MM = 8000.0;
    static constexpr double DEPTH_FOV_H         = 58.4;
    static constexpr double DEPTH_FOV_V         = 45.5;

    explicit AstraProDriver(bool use_simulated = true);
    ~AstraProDriver();

    /** 启动连续采集线程 */
    void start();

    /** 停止采集 */
    void stop();

    /** 采集一帧数据 */
    AstraFrame capture_frame();

    /** 获取最新帧（线程安全） */
    AstraFrame get_latest_frame() const;

    /** 获取三个方向的障碍物距离 */
    std::unordered_map<std::string, double> get_obstacle_distances() const;

private:
    bool use_simulated_;
    std::atomic<bool> running_{false};
    mutable std::mutex lock_;

    AstraFrame latest_frame_;
    std::unique_ptr<std::thread> capture_thread_;
    std::mt19937 rng_;

    void capture_loop();
    AstraFrame simulate_frame();

    DepthRegion analyze_region(const std::vector<uint16_t>& depth_map,
                               int width, int height, const std::string& region);
    double calc_quality(const std::vector<double>& valid_values);
    double estimate_ambient_light(const std::vector<uint16_t>& depth_map,
                                  int width, int height);
    EnvironmentType classify_environment(double light_level);
};

} // namespace mechdog