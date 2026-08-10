/**
 * Astra Pro 深度相机驱动模块 (C++ 版)
 * 基于奥比中光 Astra Pro (单目结构光)
 * 通过 Astra SDK (astra:: API) 获取深度图 (真机模式) / 模拟生成 (模拟模式)
 *
 * 真机模式 (USE_ASTRA_SDK):
 *   使用官方 FrameListener 回调模式 (与 SDK 示例 DepthReaderEventCPP 一致):
 *   - 首次调用 capture_real 时初始化 StreamSet/Reader + 启动 Depth/Color 双流
 *   - 独立 update 线程持续 astra_update() 驱动帧回调
 *   - 回调内填充 latest_frame_ (深度) 与 color_cache_ (彩色 RGB + 距离叠加)
 *   - 该模式经真机验证: depth+color 双流 640x480 正常出帧
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

/** 彩色帧数据 (RGB888, 用于可视化显示) */
struct ColorFrameData {
    bool valid = false;
    int  width  = 640;
    int  height = 480;
    std::vector<uint8_t> rgb;   // width*height*3, R G B 连续
    double center_distance_m = -1.0;   // 中央区域平均距离 (m)
    double nearest_distance_m = -1.0;  // 中央区域最近障碍 (m)
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

    /** 预初始化硬件 (真机模式: 在 main 线程启动 Astra SDK + 双流, 避免采集线程里 start 阻塞) */
    void init_hardware();

    /** 停止采集 */
    void stop();

    /** 采集一帧数据 */
    AstraFrame capture_frame();

    /** 获取最新帧（线程安全） */
    AstraFrame get_latest_frame() const;

    /** 获取最新彩色帧 (RGB888) + 中央距离/最近障碍, 用于可视化 (真机模式有效) */
    ColorFrameData get_color_frame();

    /** 是否真机模式 (非模拟) */
    bool is_real() const { return !use_simulated_; }

private:
    bool use_simulated_;
    std::atomic<bool> running_{false};
    mutable std::mutex lock_;

    AstraFrame latest_frame_;
    ColorFrameData color_cache_;
    std::unique_ptr<std::thread> capture_thread_;
    std::mt19937 rng_;

    void capture_loop();
    AstraFrame simulate_frame();

    // 真机模式 (Astra SDK, 仅 USE_ASTRA_SDK 编译时启用)
    AstraFrame capture_real();
    void real_update_loop();

    DepthRegion analyze_region(const std::vector<uint16_t>& depth_map,
                               int width, int height, const std::string& region);
    double calc_quality(const std::vector<double>& valid_values);
    double estimate_ambient_light(const std::vector<uint16_t>& depth_map,
                                  int width, int height);
    EnvironmentType classify_environment(double light_level);
};

} // namespace mechdog
