/**
 * 超声波传感器驱动模块 (HC-SR04)
 * 4颗传感器阵列, 通过 GPIO (Trig/Echo) 接口
 */
#pragma once

#include "config.h"
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <unordered_map>

namespace mechdog {

/** 单次超声波读数 */
struct UltrasonicReading {
    std::string sensor_name;
    double      distance_cm        = 0.0;
    double      timestamp          = 0.0;
    bool        valid              = false;
    double      yaw_offset_deg     = 0.0;
    double      pitch_offset_deg   = 0.0;
};

/** 4颗超声波传感器的综合数据 */
struct UltrasonicArrayData {
    UltrasonicReading front_left;
    UltrasonicReading front_center;
    UltrasonicReading front_right;
    UltrasonicReading bottom;
    double            timestamp = 0.0;

    /** 获取前方最小距离（三颗前向传感器取最小值） */
    double get_min_forward_distance_cm() const;

    /** 检测是否处于悬崖/台阶边缘 */
    bool get_cliff_detected() const;
};

/** 单个超声波传感器驱动 */
class UltrasonicSensor {
public:
    // 常量统一引用 config.h UltrasonicConfig (去重: 原各存一份易失同步)
    static constexpr double SPEED_OF_SOUND  = UltrasonicConfig::speed_of_sound;    // cm/s at 20°C
    static constexpr double TIMEOUT_SEC     = UltrasonicConfig::timeout_sec;       // ~4m 最大量程
    static constexpr double MIN_INTERVAL_SEC = UltrasonicConfig::min_interval_sec; // 50ms

    UltrasonicSensor(const std::string& name, int trig_pin, int echo_pin,
                     double yaw_offset_deg = 0.0, double pitch_offset_deg = 0.0);
    ~UltrasonicSensor();

    /** 执行一次距离测量 */
    UltrasonicReading measure();

    /** 清理资源 */
    void cleanup();

private:
    std::string name_;
    int         trig_pin_;
    int         echo_pin_;
    double      yaw_offset_deg_;
    double      pitch_offset_deg_;
    std::chrono::steady_clock::time_point last_measure_{};  // 上次测量时刻 (修复: 原 double 存纳秒计数被当秒读, 单位错乱)
    std::mt19937 rng_;  // 模拟模式用

#ifdef USE_WIRINGPI
    void setup_gpio();
#endif
    double measure_distance();
    double simulate_measure();
};

/** 4颗超声波传感器阵列驱动 */
class UltrasonicArrayDriver {
public:
    explicit UltrasonicArrayDriver(const std::unordered_map<std::string, UltrasonicSensorEntry>& layout);
    // ALG-1 (v2.2): 析构 join bottom 线程 (F9 落地: 底部独立高频通路)
    ~UltrasonicArrayDriver();

    // ALG-1 (v2.2): 底部跌落风险独立判定 (fail-closed, 沿用 F4 get_cliff_detected 语义)。
    // 由独立 bottom_loop @20Hz 线程刷新, 不再依赖 read_all 同周期; 无数据/无效读数均判有风险。
    bool is_fall_risk() const;

    // ALG-1 (v2.2): 取底部最新读数 (供 build_bottom_obstacle 等消费方; 未就绪返回 valid=false)
    UltrasonicReading get_bottom_reading() const;

    /** 分时轮询读取前向 3 颗传感器 (串行, 间隔 30ms 防串扰, 一轮 ~60ms / 16Hz);
     *  底部已移至独立 20Hz 线程 (is_fall_risk), data.bottom 由 get_bottom_reading 缓存填充 */
    UltrasonicArrayData read_all();

    /** 清理所有传感器资源 */
    void cleanup();

private:
    std::unordered_map<std::string, std::unique_ptr<UltrasonicSensor>> sensors_;
    std::mutex read_mutex_;

    // ---- ALG-1 (v2.2): 底部独立高频通路 (F9) ----
    UltrasonicSensor* bottom_sensor_ = nullptr;   // 指向 sensors_["bottom"] (构造时取出, 生命周期随 sensors_)
    std::thread bottom_thread_;
    std::atomic<bool> bottom_running_{false};
    mutable std::mutex bottom_mutex_;              // mutable: const 访问器 is_fall_risk/get_bottom_reading 加锁
    UltrasonicReading bottom_latest_{};
    std::atomic<bool> bottom_have_{false};

    void bottom_loop();                            // 20Hz 独立刷新底部读数
    void stop_bottom();                            // 置位 + join (析构/cleanup 调用)
};

} // namespace mechdog
