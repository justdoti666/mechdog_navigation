/**
 * 多传感器数据融合模块实现
 */


#include "sensor_fusion.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <boost/optional.hpp>

namespace mechdog {

SensorFusion::SensorFusion(AstraProDriver* astra, UltrasonicArrayDriver* ultrasonic)
    : astra_(astra), ultrasonic_(ultrasonic) {
    distance_history_["left"]   = {};
    distance_history_["center"] = {};
    distance_history_["right"]  = {};
    distance_history_["bottom"] = {};
}

boost::optional<FusionResult> SensorFusion::get_latest_result() const {
    return last_fusion_;
}

// ========== 核心融合 ==========
FusionResult SensorFusion::fuse() {
    FusionResult result;
    result.timestamp = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    // 1. 获取原始传感器数据
    auto astra_frame = astra_->get_latest_frame();
    auto ultrasonic_data = ultrasonic_->read_all();

    // 2. 判断环境类型
    auto env_type = determine_environment(astra_frame);
    result.environment = env_type;

    // 3. 获取自适应权重
    auto [astra_w, ultra_w] = get_adaptive_weights(env_type, astra_frame);
    result.effective_astra_weight = astra_w;
    result.effective_ultrasonic_weight = ultra_w;
    result.astra_valid = astra_frame.valid;

    // 4. 对各方向进行融合
    for (const auto& direction : {"left", "center", "right"}) {
        result.obstacles[direction] = fuse_direction(
            direction, astra_frame, ultrasonic_data, astra_w, ultra_w);
    }

    // 5. 底部悬崖检测（仅超声波）
    const auto& bottom = ultrasonic_data.bottom;
    result.cliff_detected = ultrasonic_data.get_cliff_detected();
    FusedObstacle bottom_obs;
    bottom_obs.direction = "bottom";
    bottom_obs.distance_m = bottom.distance_cm * kCmToM;
    bottom_obs.confidence = bottom.valid ? 1.0 : 0.0;
    bottom_obs.ultrasonic_dist_cm = bottom.distance_cm;
    bottom_obs.astra_dist_m = 8.0;
    bottom_obs.source = "仅超声波（底部）";
    bottom_obs.level = result.cliff_detected ? ObstacleLevel::CRITICAL : ObstacleLevel::SAFE;
    result.obstacles["bottom"] = std::move(bottom_obs);

    // 6. 计算综合决策
    result.min_forward_distance_m = std::min({
        result.obstacles["left"].distance_m,
        result.obstacles["center"].distance_m,
        result.obstacles["right"].distance_m,
    });

    // 7. 紧急避障检查
    double min_ultrasonic_cm = ultrasonic_data.get_min_forward_distance_cm();
    result.recommended_action = determine_action(
        result.min_forward_distance_m, min_ultrasonic_cm, result.cliff_detected);

    last_fusion_ = result;
    return result;
}

// ========== 环境判断 ==========
EnvironmentType SensorFusion::determine_environment(const AstraFrame& frame) {
    if (!frame.valid) {
        return EnvironmentType::OUTDOOR;
    }
    return frame.environment;
}

std::pair<double, double> SensorFusion::get_adaptive_weights(
    EnvironmentType env_type, const AstraFrame& frame) {
    const auto& base = get_environment_weights().at(env_to_key(env_type));
    double astra_w = base.astra;
    double ultra_w = base.ultrasonic;

    if (frame.valid) {
        double quality = frame.center_region.quality_score;
        astra_w *= quality;
        ultra_w = 1.0 - astra_w;
    }

    return {astra_w, ultra_w};
}

std::string SensorFusion::env_to_key(EnvironmentType env_type) const {
    switch (env_type) {
        case EnvironmentType::INDOOR:       return "indoor";
        case EnvironmentType::SEMI_INDOOR:  return "semi_indoor";
        case EnvironmentType::OUTDOOR:      return "outdoor";
        default:                            return "outdoor";
    }
}

// ========== 方向融合 ==========
FusedObstacle SensorFusion::fuse_direction(
    const std::string& direction,
    const AstraFrame& astra_frame,
    const UltrasonicArrayData& ultra_data,
    double astra_w, double ultra_w) {

    FusedObstacle obs;
    obs.direction = direction;

    // 获取超声波读数
    const auto* ultra_reading = get_ultrasonic_reading(ultra_data, direction);
    double ultra_dist_m = ultra_reading
        ? ultra_reading->distance_cm * kCmToM : 4.5;

    // 获取 Astra 读数
    double astra_dist_m = 8.0;
    bool astra_valid = false;
    if (astra_frame.valid) {
        const DepthRegion* region = nullptr;
        if (direction == "left")        region = &astra_frame.left_region;
        else if (direction == "center") region = &astra_frame.center_region;
        else if (direction == "right")  region = &astra_frame.right_region;

        if (region) {
            astra_dist_m = region->min_distance_m;
            astra_valid = true;
        }
    }

    // 距离分层融合
    auto [fused_dist, source] = layer_fusion(
        ultra_dist_m, astra_dist_m, astra_valid, astra_w, ultra_w);

    // 计算置信度
    double confidence = calc_confidence(
        ultra_dist_m, astra_dist_m, astra_valid, astra_w, ultra_w);

    // 判断障碍物等级
    auto level = classify_obstacle_level(fused_dist);

    obs.distance_m = std::round(fused_dist * 1000.0) / 1000.0;
    obs.confidence = std::round(confidence * 1000.0) / 1000.0;
    obs.ultrasonic_dist_cm = std::round(ultra_dist_m * 1000.0) / 10.0;
    obs.astra_dist_m = std::round(astra_dist_m * 1000.0) / 1000.0;
    obs.source = source;
    obs.level = level;

    return obs;
}

const UltrasonicReading* SensorFusion::get_ultrasonic_reading(
    const UltrasonicArrayData& ultra_data, const std::string& direction) {
    if (direction == "left")        return &ultra_data.front_left;
    if (direction == "center")      return &ultra_data.front_center;
    if (direction == "right")       return &ultra_data.front_right;
    return nullptr;
}

// ========== 分层融合策略 ==========
std::pair<double, std::string> SensorFusion::layer_fusion(
    double ultra_m, double astra_m, bool astra_valid,
    double astra_w, double ultra_w) {

    // L0: 超声盲区补偿
    if (ultra_m < 0.6) {
        std::ostringstream oss;
        oss << "仅超声波 (Astra盲区,超声=" << std::fixed << std::setprecision(1)
            << ultra_m * 100 << "cm)";
        return {ultra_m, oss.str()};
    }

    bool ultra_valid = (ultra_m < 4.5);

    if (!astra_valid) {
        if (ultra_valid) {
            std::ostringstream oss;
            oss << "仅超声波 (Astra不可用,超声=" << ultra_m * 100 << "cm)";
            return {ultra_m, oss.str()};
        } else {
            return {8.0, "无有效数据 (假设无障碍)"};
        }
    }

    // L1 区间 (0.6-3m)：保守取最小值
    if (astra_m < 3.0) {
        double conservative = std::min(ultra_m, astra_m);
        std::ostringstream oss;
        oss << "融合(L1): 取保守值 min(超声" << static_cast<int>(ultra_m * 100)
            << "cm, Astra" << static_cast<int>(astra_m * 100) << "cm) = "
            << static_cast<int>(conservative * 100) << "cm";
        return {conservative, oss.str()};
    }

    // L2 区间 (3-8m)：加权平均
    if (astra_m < 8.0) {
        if (ultra_valid) {
            double weighted = astra_m * astra_w + ultra_m * ultra_w;
            std::ostringstream oss;
            oss << "融合(L2): 加权(Astra" << std::fixed << std::setprecision(1)
                << astra_m << "m×" << astra_w << " + 超声" << ultra_m
                << "m×" << ultra_w << ") = " << weighted << "m";
            return {weighted, oss.str()};
        } else {
            std::ostringstream oss;
            oss << "仅Astra (L2,超声超量程,Astra=" << astra_m << "m)";
            return {astra_m, oss.str()};
        }
    }

    // L3 (>8m)
    std::ostringstream oss;
    oss << "仅Astra (L3,远距离=" << astra_m << "m)";
    return {astra_m, oss.str()};
}

// ========== 置信度计算 ==========
double SensorFusion::calc_confidence(
    double ultra_m, double astra_m, bool astra_valid,
    double astra_w, double ultra_w) {
    if (!astra_valid) {
        if (ultra_m < 4.5) {
            if (ultra_m < 0.6)      return 0.95;
            else if (ultra_m < 2.0) return 0.85;
            else                    return 0.70;
        }
        return 0.1;
    }

    double diff = std::abs(ultra_m - astra_m);
    double consistency = std::max(0.0, 1.0 - diff / 2.0);
    double base_confidence = (ultra_m < 4.5) ? 0.95 : 0.8;
    return base_confidence * consistency;
}

// ========== 障碍物等级 ==========
ObstacleLevel SensorFusion::classify_obstacle_level(double distance_m) {
    double dist_cm = distance_m * 100;
    if (dist_cm <= EmergencyConfig::critical_dist_cm)           return ObstacleLevel::CRITICAL;
    else if (dist_cm <= EmergencyConfig::warning_dist_cm)       return ObstacleLevel::DANGER;
    else if (dist_cm <= EmergencyConfig::safe_dist_cm)          return ObstacleLevel::WARNING;
    else                                      return ObstacleLevel::SAFE;
}

// ========== 动作决策 ==========
NavigationAction SensorFusion::determine_action(
    double min_forward_m, double min_ultrasonic_cm, bool cliff_detected) {

    // 最高优先级：悬崖检测
    if (cliff_detected) {
        return NavigationAction::STOP;
    }

    // 超声波独立紧急检查
    if (min_ultrasonic_cm <= EmergencyConfig::critical_dist_cm) {
        return NavigationAction::STOP;
    }
    if (min_ultrasonic_cm <= EmergencyConfig::warning_dist_cm) {
        return NavigationAction::BACKWARD;
    }

    // 融合距离判断
    double dist_cm = min_forward_m * 100;

    if (dist_cm <= EmergencyConfig::critical_dist_cm) {
        return NavigationAction::STOP;
    } else if (dist_cm <= EmergencyConfig::warning_dist_cm) {
        return NavigationAction::BACKWARD;
    } else if (dist_cm <= EmergencyConfig::safe_dist_cm) {
        return choose_direction();
    } else {
        return NavigationAction::FORWARD;
    }
}

NavigationAction SensorFusion::choose_direction() {
    if (!last_fusion_) {
        return NavigationAction::SLOW_FORWARD;
    }

    const auto& obstacles = last_fusion_->obstacles;

    double left_dist = 8.0, center_dist = 8.0, right_dist = 8.0;
    auto it = obstacles.find("left");
    if (it != obstacles.end()) left_dist = it->second.distance_m;
    it = obstacles.find("center");
    if (it != obstacles.end()) center_dist = it->second.distance_m;
    it = obstacles.find("right");
    if (it != obstacles.end()) right_dist = it->second.distance_m;

    if (center_dist > EmergencyConfig::safe_dist_cm / 100.0) {
        return NavigationAction::SLOW_FORWARD;
    }

    if (left_dist > right_dist && left_dist > EmergencyConfig::warning_dist_cm / 100.0) {
        return NavigationAction::TURN_LEFT;
    } else if (right_dist > EmergencyConfig::warning_dist_cm / 100.0) {
        return NavigationAction::TURN_RIGHT;
    } else {
        return NavigationAction::BACKWARD;
    }
}

} // namespace mechdog