/**
 * 多传感器数据融合模块 (C++ 版)
 * 融合 Astra Pro 深度相机 + HC-SR04 超声波传感器阵列
 */
#pragma once

#include "config.h"
#include "sensor_ultrasonic.h"
#include "sensor_astra.h"
#include <string>
#include <unordered_map>
#include <boost/optional.hpp>
#include <vector>

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
};

/** 融合结果 */
struct FusionResult {
    double timestamp                           = 0.0;
    std::unordered_map<std::string, FusedObstacle> obstacles;
    EnvironmentType environment                = EnvironmentType::UNKNOWN;
    bool astra_valid                           = false;
    bool cliff_detected                        = false;
    double effective_astra_weight              = 0.0;
    double effective_ultrasonic_weight          = 0.0;
    NavigationAction recommended_action        = NavigationAction::FORWARD;
    double min_forward_distance_m              = 8.0;
};

/**
 * 传感器融合核心类
 *
 * 使用方式:
 *   SensorFusion fusion(astra_driver, ultrasonic_driver);
 *   auto result = fusion.fuse();
 */
class SensorFusion {
public:
    SensorFusion(AstraProDriver* astra, UltrasonicArrayDriver* ultrasonic);

    /** 执行一次传感器融合 */
    FusionResult fuse();

    /** 获取最近一次融合结果 */
    boost::optional<FusionResult> get_latest_result() const;

private:
    AstraProDriver* astra_;
    UltrasonicArrayDriver* ultrasonic_;
    boost::optional<FusionResult> last_fusion_;

    // 滑动平均缓存
    std::unordered_map<std::string, std::vector<double>> distance_history_;
    static constexpr int kHistoryMaxLen = 5;

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

    double calc_confidence(double ultra_m, double astra_m,
                           bool astra_valid, double astra_w, double ultra_w);

    ObstacleLevel classify_obstacle_level(double distance_m);
    NavigationAction determine_action(double min_forward_m,
                                       double min_ultrasonic_cm,
                                       bool cliff_detected);
    NavigationAction choose_direction();
};

} // namespace mechdog