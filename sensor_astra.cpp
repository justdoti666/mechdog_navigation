/**
 * Astra Pro 深度相机驱动模块实现
 */
#include "sensor_astra.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>

namespace mechdog {

AstraProDriver::AstraProDriver(bool use_simulated)
    : use_simulated_(use_simulated)
    , rng_(std::random_device{}()) {
    if (!use_simulated_) {
        std::cout << "[Astra] 真实硬件模式（需 OpenNI2 支持）" << std::endl;
    }
}

AstraProDriver::~AstraProDriver() {
    stop();
}

void AstraProDriver::start() {
    if (running_) return;
    running_ = true;
    capture_thread_ = std::make_unique<std::thread>(&AstraProDriver::capture_loop, this);
    std::cout << "[Astra] 采集线程已启动" << std::endl;
}

void AstraProDriver::stop() {
    running_ = false;
    if (capture_thread_ && capture_thread_->joinable()) {
        capture_thread_->join();
        capture_thread_.reset();
    }
    std::cout << "[Astra] 采集已停止" << std::endl;
}

void AstraProDriver::capture_loop() {
    while (running_) {
        auto frame = capture_frame();
        {
            std::lock_guard<std::mutex> guard(lock_);
            latest_frame_ = std::move(frame);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / DEPTH_FPS));
    }
}

AstraFrame AstraProDriver::capture_frame() {
    if (use_simulated_) {
        return simulate_frame();
    }
    // 真实硬件模式需 OpenNI2 SDK
    AstraFrame frame;
    frame.timestamp = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    frame.valid = false;
    return frame;
}

AstraFrame AstraProDriver::get_latest_frame() const {
    std::lock_guard<std::mutex> guard(lock_);
    return latest_frame_;
}

std::unordered_map<std::string, double> AstraProDriver::get_obstacle_distances() const {
    auto frame = get_latest_frame();
    if (!frame.valid) {
        return {{"left", 8.0}, {"center", 8.0}, {"right", 8.0}};
    }
    return {
        {"left",   frame.left_region.min_distance_m},
        {"center", frame.center_region.min_distance_m},
        {"right",  frame.right_region.min_distance_m}
    };
}

// ========== 模拟深度帧 ==========
AstraFrame AstraProDriver::simulate_frame() {
    int w = DEPTH_WIDTH, h = DEPTH_HEIGHT;
    std::vector<uint16_t> depth_map(static_cast<size_t>(w * h), 5000);

    int third = w / 3;
    std::uniform_real_distribution<double> dist_01(0.0, 1.0);

    // 中心偶尔有障碍
    if (dist_01(rng_) > 0.5) {
        std::uniform_int_distribution<uint16_t> obstacle(800, 1500);
        for (int row = 0; row < h; ++row) {
            auto* base = depth_map.data() + static_cast<size_t>(row * w);
            for (int col = third; col < 2 * third; ++col) {
                base[col] = obstacle(rng_);
            }
        }
    }

    // 左侧可能有墙
    if (dist_01(rng_) > 0.7) {
        std::uniform_int_distribution<uint16_t> wall(500, 1000);
        for (int row = 0; row < h; ++row) {
            auto* base = depth_map.data() + static_cast<size_t>(row * w);
            for (int col = 0; col < third; ++col) {
                base[col] = wall(rng_);
            }
        }
    }

    // 添加噪声
    std::uniform_int_distribution<int16_t> noise(-50, 50);
    for (auto& v : depth_map) {
        int val = static_cast<int>(v) + noise(rng_);
        v = static_cast<uint16_t>(std::clamp(val, 0, 8000));
    }

    AstraFrame frame;
    frame.timestamp = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    frame.valid = true;
    frame.depth_map = std::move(depth_map);
    frame.depth_width = w;
    frame.depth_height = h;
    frame.environment = EnvironmentType::INDOOR;
    frame.ambient_light_level = 0.1;

    frame.center_region = analyze_region(frame.depth_map, w, h, "center");
    frame.left_region   = analyze_region(frame.depth_map, w, h, "left");
    frame.right_region  = analyze_region(frame.depth_map, w, h, "right");

    return frame;
}

// ========== 区域分析 ==========
DepthRegion AstraProDriver::analyze_region(
    const std::vector<uint16_t>& depth_map,
    int width, int height, const std::string& region) {

    int third_w = width / 3;
    int col_start, col_end;

    if (region == "left") {
        col_start = 0;          col_end = third_w;
    } else if (region == "right") {
        col_start = 2 * third_w; col_end = width;
    } else { // center
        col_start = third_w;     col_end = 2 * third_w;
    }

    std::vector<double> valid_pixels;
    int obstacle_count = 0;
    int total_roi = height * (col_end - col_start);

    for (int row = 0; row < height; ++row) {
        auto* base = depth_map.data() + static_cast<size_t>(row * width);
        for (int col = col_start; col < col_end; ++col) {
            uint16_t val = base[col];
            if (val >= MIN_VALID_DISTANCE_MM && val <= MAX_VALID_DISTANCE_MM) {
                valid_pixels.push_back(static_cast<double>(val));
                if (val < 1000) {
                    ++obstacle_count;
                }
            }
        }
    }

    DepthRegion dr;
    if (valid_pixels.empty()) {
        return dr; // 全部默认 8.0m
    }

    auto mid = valid_pixels.begin() + valid_pixels.size() / 2;
    std::nth_element(valid_pixels.begin(), mid, valid_pixels.end());
    double median = *mid;

    auto [min_it, max_it] = std::minmax_element(valid_pixels.begin(), valid_pixels.end());

    dr.center_distance_m  = std::round(median / 1000.0 * 1000.0) / 1000.0;
    dr.min_distance_m     = std::round(*min_it / 1000.0 * 1000.0) / 1000.0;
    dr.max_distance_m     = std::round(*max_it / 1000.0 * 1000.0) / 1000.0;
    dr.obstacle_count     = obstacle_count;
    dr.valid_pixel_ratio  = std::round(static_cast<double>(valid_pixels.size()) / total_roi * 1000.0) / 1000.0;
    dr.quality_score      = calc_quality(valid_pixels);

    return dr;
}

double AstraProDriver::calc_quality(const std::vector<double>& valid_values) {
    if (valid_values.size() <= 1) return 0.0;
    double sum = 0.0, sq_sum = 0.0;
    for (auto v : valid_values) {
        sum += v;
        sq_sum += v * v;
    }
    double mean = sum / valid_values.size();
    double variance = sq_sum / valid_values.size() - mean * mean;
    double noise = std::min(std::sqrt(variance) / 1000.0, 1.0);
    return std::max(0.0, 1.0 - noise * 0.7);
}

double AstraProDriver::estimate_ambient_light(
    const std::vector<uint16_t>& depth_map, int width, int height) {
    size_t total = static_cast<size_t>(width * height);
    size_t valid_count = 0;
    for (auto v : depth_map) {
        if (v >= MIN_VALID_DISTANCE_MM && v <= MAX_VALID_DISTANCE_MM) {
            ++valid_count;
        }
    }
    double invalid_ratio = 1.0 - static_cast<double>(valid_count) / total;
    return std::round(invalid_ratio * 1000.0) / 1000.0;
}

EnvironmentType AstraProDriver::classify_environment(double light_level) {
    if (light_level < 0.15)      return EnvironmentType::INDOOR;
    else if (light_level < 0.4)  return EnvironmentType::SEMI_INDOOR;
    else                         return EnvironmentType::OUTDOOR;
}

} // namespace mechdog