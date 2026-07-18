/**
 * 超声波传感器驱动模块实现 (HC-SR04)
 */
#include "sensor_ultrasonic.h"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <iostream>

#ifdef USE_WIRINGPI
#include <wiringPi.h>
#endif

namespace mechdog {

// ============================================================
// UltrasonicArrayData
// ============================================================

double UltrasonicArrayData::get_min_forward_distance_cm() const {
    return std::min({front_left.distance_cm, front_center.distance_cm, front_right.distance_cm});
}

bool UltrasonicArrayData::get_cliff_detected() const {
    // 底部传感器读数 > 30cm 认为有跌落风险
    return bottom.distance_cm > 30.0;
}

std::string UltrasonicArrayData::get_available_direction() const {
    const double threshold = 30.0;  // 30cm 以内认为有障碍
    std::unordered_map<std::string, double> dirs = {
        {"center", front_center.distance_cm},
        {"left",   front_left.distance_cm},
        {"right",  front_right.distance_cm},
    };

    std::string best_dir = "center";
    double      best_val = 0.0;
    for (const auto& kv : dirs) {
        if (kv.second > best_val) {
            best_val = kv.second;
            best_dir = kv.first;
        }
    }

    if (dirs[best_dir] < threshold)
        return "";  // 全堵死
    return best_dir;
}

// ============================================================
// UltrasonicSensor
// ============================================================

UltrasonicSensor::UltrasonicSensor(const std::string& name, int trig_pin, int echo_pin,
                                   double yaw_offset_deg, double pitch_offset_deg)
    : name_(name), trig_pin_(trig_pin), echo_pin_(echo_pin),
      yaw_offset_deg_(yaw_offset_deg), pitch_offset_deg_(pitch_offset_deg),
      rng_(std::random_device{}())
{
#ifdef USE_WIRINGPI
    setup_gpio();
#else
    std::cout << "[超声] RPi.GPIO 不可用，使用模拟模式" << std::endl;
#endif
}

UltrasonicSensor::~UltrasonicSensor() {
    cleanup();
}

#ifdef USE_WIRINGPI
void UltrasonicSensor::setup_gpio() {
    pinMode(trig_pin_, OUTPUT);
    pinMode(echo_pin_, INPUT);
    digitalWrite(trig_pin_, LOW);
}
#endif

void UltrasonicSensor::cleanup() {
#ifdef USE_WIRINGPI
    // WiringPi 的 cleanup 在 Arduino-like 模式下不需要单独操作
#endif
}

UltrasonicReading UltrasonicSensor::measure() {
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(
        now - std::chrono::steady_clock::time_point(
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(last_measure_time_)))).count();

    if (elapsed < MIN_INTERVAL_SEC) {
        std::this_thread::sleep_for(
            std::chrono::duration<double>(MIN_INTERVAL_SEC - elapsed));
    }

    double timestamp = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    double distance_cm = measure_distance();

    last_measure_time_ = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    bool valid = (distance_cm >= 2.0) && (distance_cm <= 400.0);

    return UltrasonicReading{
        .sensor_name      = name_,
        .distance_cm      = std::round(distance_cm * 100.0) / 100.0,
        .timestamp        = timestamp,
        .valid            = valid,
        .yaw_offset_deg   = yaw_offset_deg_,
        .pitch_offset_deg = pitch_offset_deg_,
    };
}

double UltrasonicSensor::measure_distance() {
#ifdef USE_WIRINGPI
    // 发送 10μs 触发脉冲
    digitalWrite(trig_pin_, HIGH);
    std::this_thread::sleep_for(std::chrono::microseconds(10));
    digitalWrite(trig_pin_, LOW);

    // 等待 Echo 变高
    auto start_time = std::chrono::high_resolution_clock::now();
    while (digitalRead(echo_pin_) == LOW) {
        auto now = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration<double>(now - start_time).count() > TIMEOUT_SEC)
            return -1.0;
    }

    // 记录高电平持续时间
    auto pulse_start = std::chrono::high_resolution_clock::now();
    while (digitalRead(echo_pin_) == HIGH) {
        auto now = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration<double>(now - pulse_start).count() > TIMEOUT_SEC)
            return -1.0;
    }

    auto pulse_end = std::chrono::high_resolution_clock::now();
    double duration = std::chrono::duration<double>(pulse_end - pulse_start).count();

    return (duration * SPEED_OF_SOUND) / 2.0;
#else
    return simulate_measure();
#endif
}

double UltrasonicSensor::simulate_measure() {
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    if (name_ == "bottom") {
        // 底部传感器：5% 概率模拟悬崖
        if (dist(rng_) < 0.05) {
            std::uniform_real_distribution<double> cliff(200.0, 350.0);
            return cliff(rng_);
        }
        std::uniform_real_distribution<double> ground(12.0, 28.0);
        return ground(rng_);
    }

    // 前向传感器
    double r = dist(rng_);
    if (r < 0.85) {
        std::uniform_real_distribution<double> open(180.0, 400.0);
        return open(rng_);
    } else if (r < 0.95) {
        std::uniform_real_distribution<double> mid(50.0, 150.0);
        return mid(rng_);
    } else {
        std::uniform_real_distribution<double> close(5.0, 40.0);
        return close(rng_);
    }
}

// ============================================================
// UltrasonicArrayDriver
// ============================================================

UltrasonicArrayDriver::UltrasonicArrayDriver(
    const std::unordered_map<std::string, UltrasonicSensorEntry>& layout)
{
    for (const auto& kv : layout) {
        const auto& name = kv.first;
        const auto& cfg  = kv.second;
        sensors_[name] = std::make_unique<UltrasonicSensor>(
            name, cfg.trig_pin, cfg.echo_pin,
            cfg.yaw_offset_deg, cfg.pitch_offset_deg);
    }
}

UltrasonicArrayData UltrasonicArrayDriver::read_all() {
    UltrasonicArrayData data;
    data.timestamp = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    // 默认读数（传感器不存在时使用）
    UltrasonicReading default_reading;
    default_reading.distance_cm = 400.0;
    default_reading.valid = false;

    // 互斥锁防止外部并发调用
    std::lock_guard<std::mutex> lock(read_mutex_);

    // 分时轮询：固定顺序依次触发，每颗间隔 30ms 避免串扰
    // 原因：HC-SR04 最大回波时间 ~25ms，间隔 ≥30ms 确保前一颗回波完全衰减
    // 4 颗完整一轮 ~120ms → 更新率约 8Hz，步行速度足够
    static constexpr double CROSSTALK_GAP_MS = 30.0;

    auto read_sensor = [&](const std::string& key) -> UltrasonicReading {
        auto it = sensors_.find(key);
        if (it == sensors_.end()) return default_reading;
        return it->second->measure();
    };

    // 底部传感器先读（结果不受声波串扰影响，方向不同）
    data.bottom       = read_sensor("bottom");
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(CROSSTALK_GAP_MS));

    // 前向三颗依次轮询
    data.front_center = read_sensor("front_center");
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(CROSSTALK_GAP_MS));

    data.front_left   = read_sensor("front_left");
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(CROSSTALK_GAP_MS));

    data.front_right  = read_sensor("front_right");

    return data;
}

void UltrasonicArrayDriver::cleanup() {
    for (auto& kv : sensors_) {
        auto& sensor = kv.second;
        sensor->cleanup();
    }
}

} // namespace mechdog