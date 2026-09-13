/**
 * Astra Pro 深度相机驱动模块 (C++ 版)
 * 基于奥比中光 Astra Pro (单目结构光)
 * 通过 Astra SDK (astra:: API) 获取深度图 (真机模式) / 模拟生成 (模拟模式)
 *
 * 真机模式 (USE_ASTRA_SDK):
 *   DepthReaderPoll 自 pump 轮询 (单执行者, 无独立线程):
 *   - 首次调用 capture_real 时初始化 StreamSet/Reader + 启动 Depth/Color 双流
 *   - capture_real 在 reader_mutex 内循环 astra_update() + open_frame, 最长 80ms 窗口
 *     (帧周期名义 33ms, 真机取帧 ~50ms → 窗口需覆盖一帧)
 *   - 真机实测: depth+color 双流正常出帧 (SDK 未设 mode → 深度默认 320x240; 模拟帧 640x480)
 *   - 注: FrameListener 回调模式下 Astra Pro 深度值恒 0 (SDK 已知行为), 已弃用;
 *     也不设独立 update 线程 (D1 真机证伪"删线程自 pump 10ms 窗口", 见 capture_real 注释)
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
    // ALG-8 (v2.2): 默认 valid=false (fail-closed; 原默认 true 是语义陷阱, 空帧被误判有效)
    bool          valid            = false;
    // ALG-8 (v2.2): 帧序号, 供消费者判新鲜度/丢帧 (capture_real/simulate 递增)
    uint64_t      frame_seq        = 0;
    EnvironmentType environment     = EnvironmentType::UNKNOWN;
    // ALG-3 (v2.2): 删除 ambient_light_level 字段 —— 无外部消费方 (fusion 只读 environment),
    // 仅 capture_real 内作瞬态局部派生 environment; 现改为局部变量 (见 sensor_astra.cpp)

    DepthRegion   center_region;
    DepthRegion   left_region;
    DepthRegion   right_region;

    // 深度图数据 (模拟模式使用)
    std::vector<uint16_t> depth_map;
    int depth_width  = 640;
    int depth_height = 480;
};

// ------------------------------------------------------------
// FIX-01: 深度帧是否可用于按 (depth_width, depth_height) 索引 depth_map。
// 三态失效帧 (capture_real 取帧失败) 的 depth_map 为空, 但 depth_width/
// depth_height 仍是默认 640x480 —— 主循环热力图分支 (D 键) 曾直接按
// y*hw+x 索引 → 空 vector 越界读 (UB)。守卫口径集中在此一处, 避免
// 主循环各分支 (点云/热力图) 各写一份而漏改。
// ------------------------------------------------------------
inline bool depth_frame_usable(const AstraFrame& frame) {
    if (!frame.valid || frame.depth_width <= 0 || frame.depth_height <= 0) return false;
    if (frame.depth_map.empty()) return false;
    return frame.depth_map.size() >=
           static_cast<size_t>(frame.depth_width) * static_cast<size_t>(frame.depth_height);
}

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

    /**
     * 外部注入深度帧 (无 Astra SDK 场景: ROS 话题 / 离线回放)。
     * 与 UltrasonicArrayDriver::inject_external_data 同思路: 注入后立即生效于
     * get_latest_frame(), 区域分析 (analyze_region) 与环境判定
     * (estimate_ambient_light → classify_environment) 走与真机 capture_real 相同的链路。
     *
     * 用法: 订阅深度话题 → 转 uint16(mm, 行主序) → 调用本函数;
     *       ⚠ **不要**同时调用 start() —— 采集线程会覆盖注入帧 (topic 源应由话题驱动)。
     *
     * @param depth_map uint16 深度(mm), 0 = 无效; 长度须 >= width*height
     * @param stamp_s   采集时刻(秒); < 0 = 取当前系统时间
     * @return true = 已接受并更新最新帧; false = 尺寸/长度非法 (原帧保持不动)
     */
    bool inject_depth_frame(const std::vector<uint16_t>& depth_map, int width, int height,
                            double stamp_s = -1.0);

    /**
     * 标记当前帧失效 (话题超时 / 深度源断开)。
     * 深度链路随即退出融合 (fail-closed): fuse() 的 all_sensors_invalid 会看到
     * astra 无有效像素; 超声波链路不受影响 (仍按超声决策)。
     */
    void invalidate_frame();

    /** 是否真机模式 (非模拟) */
    bool is_real() const { return !use_simulated_; }

private:
    bool use_simulated_;
    std::atomic<bool> running_{false};
    mutable std::mutex lock_;

    AstraFrame latest_frame_;
    std::unique_ptr<std::thread> capture_thread_;
    std::mt19937 rng_;
    // ALG-8 (v2.2): 帧序号计数 (capture_loop 递增, 写入 AstraFrame::frame_seq)
    std::atomic<uint64_t> frame_seq_counter_{0};

    void capture_loop();
    AstraFrame simulate_frame();

    // 真机模式 (Astra SDK, 仅 USE_ASTRA_SDK 编译时启用)
    AstraFrame capture_real();

    DepthRegion analyze_region(const std::vector<uint16_t>& depth_map,
                               int width, int height, const std::string& region);
    double calc_quality(const std::vector<double>& valid_values, int region_pixels);
    double estimate_ambient_light(const std::vector<uint16_t>& depth_map,
                                  int width, int height);
    EnvironmentType classify_environment(double light_level);
};

} // namespace mechdog
