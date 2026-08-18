/**
 * 多传感器数据融合模块 (C++ 版)
 * 融合 Astra Pro 深度相机 + HC-SR04 超声波传感器阵列 + TSL2591 环境红外
 */
#pragma once

#include "config.h"
#include "sensor_ultrasonic.h"
#include "sensor_astra.h"
#include "sensor_ir.h"
#include <string>
#include <unordered_map>

namespace mechdog {

/** 障碍物危险等级 */
enum class ObstacleLevel {
    SAFE,
    WARNING,
    DANGER,
    CRITICAL
};

/** 导航动作指令 */
enum class NavigationAction {
    STOP,
    BACKWARD,
    TURN_LEFT,
    TURN_RIGHT,
    SLOW_FORWARD,
    FORWARD,
    REACHED_GOAL
};

/** 融合后的单个方向障碍物信息 */
struct FusedObstacle {
    std::string direction;
    double      distance_m          = 8.0;
    double      confidence          = 0.0;
    double      ultrasonic_dist_cm  = 400.0;
    double      astra_dist_m        = 8.0;
    std::string source;
    ObstacleLevel level            = ObstacleLevel::SAFE;
    // A1: 方向级失效标记。某个方向传感器双侧失效 (Astra 无有效像素 + 超声无效) 时置 false,
    // 该方向的 8.0m "假设无障碍" 兜底值不得参与方向决策 (否则会被当作最开阔盲区转向)。
    bool        valid              = true;
};

/** 融合结果 */
struct FusionResult {
    double timestamp                           = 0.0;
    std::unordered_map<std::string, FusedObstacle> obstacles;
    EnvironmentType environment                = EnvironmentType::UNKNOWN;
    bool astra_valid                           = false;
    bool cliff_detected                        = false;
    bool sensors_valid                         = true;   // M1: 任一传感器有有效数据 (全失效 -> false)
    double effective_astra_weight              = 0.0;
    double effective_ultrasonic_weight          = 0.0;
    NavigationAction recommended_action        = NavigationAction::FORWARD;
    double min_forward_distance_m              = 8.0;
};

/**
 * 传感器融合核心类
 *
 * 使用方式:
 *   SensorFusion fusion(&astra_driver, &ultrasonic_driver, &ir_sensor);
 *   auto result = fusion.fuse();
 */
class SensorFusion {
public:
    SensorFusion(AstraProDriver* astra, UltrasonicArrayDriver* ultrasonic,
                 InfraRedSensor* ir);

    /** 执行一次传感器融合 */
    FusionResult fuse();

private:
    // 单元测试访问 (tests/test_fusion.cpp 专用, R-3: 测试调用真函数而非复刻逻辑)
    friend class SensorFusionTestAccess;

    AstraProDriver* astra_;
    UltrasonicArrayDriver* ultrasonic_;
    InfraRedSensor* ir_;

    static constexpr double kCmToM = 0.01;

    EnvironmentType determine_environment(const AstraFrame& frame);
    std::pair<double, double> get_adaptive_weights(EnvironmentType env_type,
                                                    const AstraFrame& frame);
    std::string env_to_key(EnvironmentType env_type) const;

    FusedObstacle fuse_direction(const std::string& direction,
                                  const AstraFrame& astra_frame,
                                  const UltrasonicArrayData& ultra_data,
                                  double astra_w, double ultra_w);

    const UltrasonicReading* get_ultrasonic_reading(
        const UltrasonicArrayData& ultra_data, const std::string& direction);

    std::pair<double, std::string> layer_fusion(
        double ultra_m, double astra_m, bool astra_valid,
        double astra_w, double ultra_w);

    FusedObstacle build_bottom_obstacle(const UltrasonicReading& bottom,
                                        bool cliff_detected);

    double calc_confidence(double ultra_m, double astra_m, bool astra_valid);

    ObstacleLevel classify_obstacle_level(double distance_m);
    // R3: front_valid —— 前向 (left/center/right) 是否有任一有效方向。
    // 旧判定把 bottom 计入有效性: 底部超声有效 + 前向全盲时 sensors_valid=true,
    // min_forward=8.0 兜底 -> 直接 FORWARD (fail-open)。前向失明必须保守, bottom 无效力。
    NavigationAction determine_action(double min_forward_m,
                                       double min_ultrasonic_cm,
                                       bool cliff_detected,
                                       const std::unordered_map<std::string, FusedObstacle>& obstacles,
                                       bool sensors_valid,
                                       bool front_valid);
    NavigationAction choose_direction(
        const std::unordered_map<std::string, FusedObstacle>& obstacles);

    // M1: 全部传感器均无有效数据 (fail-closed 判定, 独立可测)
    static bool all_sensors_invalid(const AstraFrame& astra_frame,
                                    const UltrasonicArrayData& ultra_data);
};

} // namespace mechdog
