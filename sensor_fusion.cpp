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
#include <limits>

namespace mechdog {

SensorFusion::SensorFusion(AstraProDriver* astra, UltrasonicArrayDriver* ultrasonic,
                           InfraRedSensor* ir)
    : astra_(astra), ultrasonic_(ultrasonic), ir_(ir) {
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
    result.obstacles["bottom"] = build_bottom_obstacle(bottom, result.cliff_detected);

    // 6. 计算综合决策 (A1: 只统计有效方向的距离; 全失效的 8.0m 兜底值不参与)
    double min_forward = std::numeric_limits<double>::max();
    for (const auto& dir : {"left", "center", "right"}) {
        const auto& o = result.obstacles.at(dir);
        if (!o.valid) continue;             // A1: 排除双侧失效的盲区方向
        min_forward = std::min(min_forward, o.distance_m);
    }
    result.min_forward_distance_m = (min_forward == std::numeric_limits<double>::max())
        ? 8.0 : min_forward;

    // 7. 紧急避障检查
    double min_ultrasonic_cm = ultrasonic_data.get_min_forward_distance_cm();
    // M1: 传感器有效性 (fail-closed: 全失效时 determine_action 直接 STOP)
    result.sensors_valid = !all_sensors_invalid(astra_frame, ultrasonic_data);
    result.recommended_action = determine_action(
        result.min_forward_distance_m, min_ultrasonic_cm, result.cliff_detected,
        result.obstacles, result.sensors_valid);

    return result;
}

// ========== 环境判断 ==========
EnvironmentType SensorFusion::determine_environment(const AstraFrame& frame) {
    // 修复(F3): 环境判定改用 TSL2591 实测红外强度(设计意图), 而非深度图无效像素比估算
    if (ir_ && !ir_->is_simulated()) {
        double light = ir_->read_normalized_light();  // 0.0~1.0; -1.0 = 读取失败
        if (light < 0) {
            // 修复D3: 红外传感器故障时回退室外(超声波主导), 安全侧
            return EnvironmentType::OUTDOOR;
        }
        if (light <= IrConfig::ir_indoor_max)  return EnvironmentType::INDOOR;
        if (light >= IrConfig::ir_outdoor_min) return EnvironmentType::OUTDOOR;
        return EnvironmentType::SEMI_INDOOR;
    }
    // 红外不可用(模拟/无传感器)时回退: 帧有效用深度图估算, 无效视为室外(超声波主导, 安全侧)
    if (!frame.valid || frame.environment == EnvironmentType::UNKNOWN) {
        // 帧未就绪或环境未初始化 (如采集线程启动初帧): 若红外是模拟的,
        // 用其随机值保证环境可判定 (保持 test_fusion 对"红外模拟值全覆盖三环境"的语义);
        // 若红外真机失败, 回退室外(安全侧)
        if (ir_) {
            double light = ir_->read_normalized_light();
            if (light >= 0) {
                if (light <= IrConfig::ir_indoor_max)  return EnvironmentType::INDOOR;
                if (light >= IrConfig::ir_outdoor_min) return EnvironmentType::OUTDOOR;
                return EnvironmentType::SEMI_INDOOR;
            }
        }
        return EnvironmentType::OUTDOOR;
    }
    return frame.environment;
}

std::pair<double, double> SensorFusion::get_adaptive_weights(
    EnvironmentType env_type, const AstraFrame& frame) {
    // 修复(F6): 值拷贝而非引用 —— 原代码对 get_environment_weights() 返回的临时 map 取引用, 是悬垂引用(UB)
    auto base = get_environment_weights().at(env_to_key(env_type));
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

    // 获取超声波读数 (修复D1: 必须检查 valid —— 真机超时返回 -1.0, 不查 valid 会被 L0 误判为超近距离障碍)
    const auto* ultra_reading = get_ultrasonic_reading(ultra_data, direction);
    double ultra_dist_m = (ultra_reading && ultra_reading->valid)
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
            // H1 修复: 区域无有效深度像素时 (valid_pixel_ratio==0, analyze_region
            // 兜底 min_distance_m=8.0) 不得判为有效 —— 否则 8.0m 假读数会走 L3
            // 把近处超声障碍整个丢弃 (报告 probe①: astra=8.0 无像素 + ultra=1.0 -> 8.0)
            astra_valid = (region->valid_pixel_ratio > 0.0);
        }
    }

    // 距离分层融合
    auto [fused_dist, source] = layer_fusion(
        ultra_dist_m, astra_dist_m, astra_valid, astra_w, ultra_w);

    // A1: 方向级失效标记 —— 双侧均无有效数据时, fused_dist 是 8.0m "假设无障碍" 兜底值,
    // 绝非真实距离。标记 valid=false, 该方向不得参与方向决策 (否则盲区被当最开阔)。
    obs.valid = astra_valid || (ultra_reading && ultra_reading->valid);

    // 计算置信度
    double confidence = calc_confidence(ultra_dist_m, astra_dist_m, astra_valid);

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

    // L0: 超声盲区补偿 (修复D2: 仅当超声有效且为合理正值时才进入盲区分支,
    // 否则负距离/无效值会被当成超近障碍触发假性急停)
    if (ultra_m >= 0.02 && ultra_m < 0.6) {
        std::ostringstream oss;
        oss << "仅超声波 (Astra盲区,超声=" << std::fixed << std::setprecision(1)
            << ultra_m * 100 << "cm)";
        return {ultra_m, oss.str()};
    }

    // 有效区间: [0.02m(2cm), 4.5m)
    bool ultra_valid = (ultra_m >= 0.02) && (ultra_m < 4.5);

    if (!astra_valid) {
        if (ultra_valid) {
            std::ostringstream oss;
            oss << "仅超声波 (Astra不可用,超声=" << ultra_m * 100 << "cm)";
            return {ultra_m, oss.str()};
        } else {
            return {8.0, "无有效数据 (假设无障碍)"};
        }
    }

    // L1 区间 (0.6-3m)：保守取最小值 (FIX-11: 超声无效时不再误标"融合")
    if (astra_m < 3.0) {
        if (ultra_valid) {
            double conservative = std::min(ultra_m, astra_m);
            std::ostringstream oss;
            oss << "融合(L1): 取保守值 min(超声" << static_cast<int>(ultra_m * 100)
                << "cm, Astra" << static_cast<int>(astra_m * 100) << "cm) = "
                << static_cast<int>(conservative * 100) << "cm";
            return {conservative, oss.str()};
        } else {
            std::ostringstream oss;
            oss << "仅Astra (L1,超声超量程,Astra=" << astra_m << "m)";
            return {astra_m, oss.str()};
        }
    }

    // L2/L3 区间 (>=3m): 保守取近 (H1 修复)
    // 修复前 L2 加权平均/L3 直接返回 astra_m, 近处超声障碍(薄杆/低矮/玻璃/盲区边缘,
    // Astra 测不到但超声测得到)会被远处 Astra 读数平均掉或直接丢弃 —— 违背 L1 已有的
    // "取近" 设计意图, 且 L3 丢弃超声最危险。
    // 守卫: ultra 有效(真实距离, 非 4.5 兜底)且比 Astra 更近 -> 直接取 ultra_m。
    if (ultra_valid && ultra_m < astra_m) {
        std::ostringstream oss;
        oss << "融合(取近): 超声" << static_cast<int>(ultra_m * 100)
            << "cm < Astra " << astra_m << "m";
        return {ultra_m, oss.str()};
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

// ========== 底部障碍物构造 (R5-1: 无效读数兜底 0.0, 不产出负距离) ==========
FusedObstacle SensorFusion::build_bottom_obstacle(
    const UltrasonicReading& bottom, bool cliff_detected) {
    FusedObstacle obs;
    obs.direction = "bottom";
    // R5-1: 真机超时返回 -1.0cm (valid=false), 直接乘 kCmToM 会得到 -0.01m
    // 写进融合结果; 无效时兜底 0.0 (不参与 min_forward, 可视化 d<=0 跳过)
    obs.distance_m = bottom.valid ? bottom.distance_cm * kCmToM : 0.0;
    obs.confidence = bottom.valid ? 1.0 : 0.0;
    // Low 清理: 无效读数不再存 -1.0 (与 distance_m 兜底口径一致, 避免字段含噪声)
    obs.ultrasonic_dist_cm = bottom.valid ? bottom.distance_cm : 0.0;
    obs.astra_dist_m = 8.0;
    obs.source = "仅超声波（底部）";
    obs.level = cliff_detected ? ObstacleLevel::CRITICAL : ObstacleLevel::SAFE;
    return obs;
}

// ========== 置信度计算 ==========
double SensorFusion::calc_confidence(
    double ultra_m, double astra_m, bool astra_valid) {
    // 修复D2: 超声有效区间统一为 [0.02m, 4.5m)
    const bool ultra_valid = (ultra_m >= 0.02) && (ultra_m < 4.5);

    if (!astra_valid) {
        if (ultra_valid) {
            if (ultra_m < 0.6)      return 0.95;
            else if (ultra_m < 2.0) return 0.85;
            else                    return 0.70;
        }
        return 0.1;
    }

    // R-1: 超声无效(兜底 4.5)时不参与一致性计算——兜底值不是真实距离,
    // 用 |4.5-astra_m| 算 diff 会把 Astra 的置信度错误压到 0 (如 astra 8m 时)
    if (!ultra_valid) return 0.8;  // 仅 Astra 单独可信度

    double diff = std::abs(ultra_m - astra_m);
    double consistency = std::max(0.0, 1.0 - diff / 2.0);
    return 0.95 * consistency;
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
    double min_forward_m, double min_ultrasonic_cm, bool cliff_detected,
    const std::unordered_map<std::string, FusedObstacle>& obstacles,
    bool sensors_valid) {

    // M1 fail-closed: 全部传感器均无有效数据时, 兜底值 (8.0m/400cm) 不可信,
    // 不得"假设无障碍"继续前进 —— 停车等待数据恢复 (上游闸门超时另有兜底)
    if (!sensors_valid) {
        return NavigationAction::STOP;
    }

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
        return choose_direction(obstacles);
    } else {
        return NavigationAction::FORWARD;
    }
}

NavigationAction SensorFusion::choose_direction(
    const std::unordered_map<std::string, FusedObstacle>& obstacles) {
    // M6: 直接使用当前帧障碍数据 (由 fuse() 传入), 不再读 last_fusion_ ——
    // 旧实现读上一帧, 方向决策滞后一帧 (8Hz 下 ~125ms)
    // A1: 忽略双侧失效 (valid=false) 的盲区方向, 否则 8.0m 兜底值会被当作最开阔转向。

    double left_dist = 8.0, center_dist = 8.0, right_dist = 8.0;
    bool  left_ok = false, center_ok = false, right_ok = false;
    auto it = obstacles.find("left");
    if (it != obstacles.end() && it->second.valid) { left_dist = it->second.distance_m; left_ok = true; }
    it = obstacles.find("center");
    if (it != obstacles.end() && it->second.valid) { center_dist = it->second.distance_m; center_ok = true; }
    it = obstacles.find("right");
    if (it != obstacles.end() && it->second.valid) { right_dist = it->second.distance_m; right_ok = true; }

    // A1: 所有前向方向均失效 -> 无可靠方向信息, 保守 (调用方 also 会因 sensors_valid=false STOP)
    if (!left_ok && !center_ok && !right_ok) {
        return NavigationAction::SLOW_FORWARD;
    }

    // 中央有效且开阔 -> 缓行 (A1: 中央盲区时不得据此缓行)
    if (center_ok && center_dist > EmergencyConfig::safe_dist_cm / 100.0) {
        return NavigationAction::SLOW_FORWARD;
    }

    // 在有效方向中选择较开阔侧转向 (A1: 只比较有效方向, 退出优先)
    if (left_ok && right_ok) {
        if (left_dist > right_dist && left_dist > EmergencyConfig::warning_dist_cm / 100.0) {
            return NavigationAction::TURN_LEFT;
        }
        if (right_dist > EmergencyConfig::warning_dist_cm / 100.0) {
            return NavigationAction::TURN_RIGHT;
        }
        return NavigationAction::BACKWARD;
    }
    // 单侧有效: 仅依据该侧; 无效侧不参与抉择
    if (left_ok && left_dist > EmergencyConfig::warning_dist_cm / 100.0) {
        return NavigationAction::TURN_LEFT;
    }
    if (right_ok && right_dist > EmergencyConfig::warning_dist_cm / 100.0) {
        return NavigationAction::TURN_RIGHT;
    }
    return NavigationAction::BACKWARD;
}

// M1: 全部传感器均无有效数据?
bool SensorFusion::all_sensors_invalid(const AstraFrame& astra_frame,
                                       const UltrasonicArrayData& ultra_data) {
    // 与融合层 (fuse_direction) 同一口径: astra 有效 = 任一前向区域有有效深度像素。
    // 不信任 frame.valid 本身 —— H1 后融合层把"帧有效但区域无有效像素" (镜头被挡/
    // 全黑/深度全失效) 也按无效处理, 此处必须同口径, 否则 8.0m 兜底 + 超声全失效
    // 组合下会漏判为"传感器有效"继续 FORWARD。
    const bool astra_has_pixels =
        astra_frame.left_region.valid_pixel_ratio > 0.0 ||
        astra_frame.center_region.valid_pixel_ratio > 0.0 ||
        astra_frame.right_region.valid_pixel_ratio > 0.0;
    if (astra_has_pixels) return false;
    return !(ultra_data.front_left.valid || ultra_data.front_center.valid ||
             ultra_data.front_right.valid || ultra_data.bottom.valid);
}

} // namespace mechdog
