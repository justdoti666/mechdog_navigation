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
    // 修复(F5): 过滤无效读数(真机超时返回 -1.0), 避免参与 min 后触发假性急停
    double best = 400.0;  // 量程上限
    bool any_valid = false;
    for (const auto* r : {&front_left, &front_center, &front_right}) {
        if (r->valid) {
            any_valid = true;
            best = std::min(best, r->distance_cm);
        }
    }
    return any_valid ? best : 400.0;
}

bool UltrasonicArrayData::get_cliff_detected() const {
    // 底部传感器读数 > 阈值 认为有跌落风险 (阈值见 config.h UltrasonicConfig, 安装高度相关)
    // 修复(F4): 必须检查 valid —— 真机超时返回 -1.0, 若只看数值会被判"安全",
    // 而回波超时往往正是量程内无可反射地面(深坑/大台阶), 应默认判为有风险
    return !bottom.valid || bottom.distance_cm > UltrasonicConfig::cliff_threshold_cm;
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
    // 修复(F7): 直接保存 steady_clock::time_point 做差值, 修复原"存纳秒计数当秒读"的单位错乱
    auto now = std::chrono::steady_clock::now();
    if (last_measure_.time_since_epoch().count() != 0) {
        auto elapsed = now - last_measure_;
        if (elapsed < std::chrono::duration<double>(MIN_INTERVAL_SEC)) {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(MIN_INTERVAL_SEC) - elapsed);
        }
    }

    double timestamp = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    double distance_cm = measure_distance();

    last_measure_ = std::chrono::steady_clock::now();

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
    // ALG-1 (v2.2): 底部独立高频通路 (F9)。取出 bottom 裸指针, 启动 20Hz 线程。
    // bottom_sensor_ 生命周期随 sensors_ (unique_ptr), 析构时先 stop_bottom() 再释放 sensors_。
    auto it = sensors_.find("bottom");
    if (it != sensors_.end()) {
        bottom_sensor_ = it->second.get();
        bottom_running_ = true;
        bottom_thread_ = std::thread(&UltrasonicArrayDriver::bottom_loop, this);
    }
}

UltrasonicArrayDriver::~UltrasonicArrayDriver() {
    stop_bottom();  // 先停线程, 再由成员析构释放 sensors_ (避免线程访问已释放对象)
}

// ALG-1 (v2.2): 底部独立 20Hz 刷新线程 (F9)。与 read_all 完全分开, 独立锁, 不参与前向串扰时序。
// measure() 内 50ms MIN_INTERVAL_SEC 门控在 50ms 周期下不触发额外 sleep (elapsed ≈ 50ms, 不 < 50ms)。
void UltrasonicArrayDriver::bottom_loop() {
    while (bottom_running_.load()) {
        UltrasonicReading r = bottom_sensor_->measure();
        {
            std::lock_guard<std::mutex> lk(bottom_mutex_);
            bottom_latest_ = r;
            bottom_have_ = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 20Hz
    }
}

// ALG-1 (v2.2): fail-closed 跌落风险判定, 沿用 F4 get_cliff_detected 语义:
// 无数据 / 无效读数 / 距离 > 阈值 一律判有风险 (宁可误判不漏判)。
bool UltrasonicArrayDriver::is_fall_risk() const {
    if (!bottom_have_.load()) return true;  // 线程未就绪 (启动初/无 bottom 传感器) -> 有风险
    std::lock_guard<std::mutex> lk(bottom_mutex_);
    const auto& b = bottom_latest_;
    return !b.valid || b.distance_cm > UltrasonicConfig::cliff_threshold_cm;
}

UltrasonicReading UltrasonicArrayDriver::get_bottom_reading() const {
    if (!bottom_have_.load()) return UltrasonicReading{};  // 未就绪 -> valid=false 默认读数
    std::lock_guard<std::mutex> lk(bottom_mutex_);
    return bottom_latest_;
}

void UltrasonicArrayDriver::stop_bottom() {
    bottom_running_ = false;
    if (bottom_thread_.joinable()) bottom_thread_.join();
}

UltrasonicArrayData UltrasonicArrayDriver::read_all() {
    UltrasonicArrayData data;
    data.timestamp = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    // ALG-1 (v2.2): 底部读数从独立线程缓存取 (不再在本轮触发新测量), 与前向解耦
    data.bottom = get_bottom_reading();

    // 默认读数（传感器不存在时使用）
    UltrasonicReading default_reading;
    default_reading.distance_cm = 400.0;
    default_reading.valid = false;

    // 互斥锁防止外部并发调用
    std::lock_guard<std::mutex> lock(read_mutex_);

    // 分时轮询前向 3 颗：固定顺序依次触发，每颗间隔 30ms 避免串扰
    // 原因：HC-SR04 最大回波时间 ~25ms，间隔 ≥30ms 确保前一颗回波完全衰减
    // 3 颗完整一轮 ~60ms → 更新率约 16Hz (底部已独立 20Hz, 见 is_fall_risk)
    static constexpr double CROSSTALK_GAP_MS = 30.0;

    auto read_sensor = [&](const std::string& key) -> UltrasonicReading {
        auto it = sensors_.find(key);
        if (it == sensors_.end()) return default_reading;
        return it->second->measure();
    };

    // 前向三颗依次轮询
    data.front_center = read_sensor("front_center");
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(CROSSTALK_GAP_MS));

    data.front_left   = read_sensor("front_left");
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(CROSSTALK_GAP_MS));

    data.front_right  = read_sensor("front_right");

    return data;
}

void UltrasonicArrayDriver::cleanup() {
    stop_bottom();  // 先停 bottom 线程
    for (auto& kv : sensors_) {
        auto& sensor = kv.second;
        sensor->cleanup();
    }
}

} // namespace mechdog
