/**
 * 超声波传感器驱动模块 (HC-SR04)
 * 4颗传感器阵列, 通过 GPIO (Trig/Echo) 接口
 */
#pragma once

#include "config.h"
#include <string>
#include <unordered_map>
#include <memory>
#include <thread>
#include <mutex>
#include <random>

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

    /** 根据前方传感器判断哪个方向有空间 */
    std::string get_available_direction() const;
};

/** 单个超声波传感器驱动 */
class UltrasonicSensor {
public:
    static constexpr double SPEED_OF_SOUND  = 34300.0;  // cm/s at 20°C
    static constexpr double TIMEOUT_SEC     = 0.025;    // ~4m 最大量程
    static constexpr double MIN_INTERVAL_SEC = 0.05;    // 50ms

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
    double      last_measure_time_ = 0.0;
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

    /** 并行读取所有4颗传感器 */
    UltrasonicArrayData read_all();

    /** 清理所有传感器资源 */
    void cleanup();

private:
    std::unordered_map<std::string, std::unique_ptr<UltrasonicSensor>> sensors_;
    std::mutex read_mutex_;
};

} // namespace mechdog