/**
 * 路径规划模块 (C++ 版)
 *
 * 第一版为反应式映射: 依据 sensor_fusion 输出的 NavigationAction 生成
 * 线速度/角速度指令。config.h 中 PlannerConfig 预留了完整 DWA 参数,
 * 后续可在 plan() 内实现真正的 DWA 采样规划。
 */
#pragma once

#include "config.h"
#include "sensor_fusion.h"

namespace mechdog {

/** 速度指令 */
struct VelocityCmd {
    double linear  = 0.0;  // 线速度 m/s
    double angular = 0.0;  // 角速度 rad/s
};

/** 路径规划器 (第一版: 反应式动作 -> 速度映射) */
class PathPlanner {
public:
    PathPlanner() = default;

    /** 依据融合结果生成速度指令 */
    VelocityCmd plan(const FusionResult& fusion);

private:
    // 反应式映射: NavigationAction -> (linear, angular)
    VelocityCmd action_to_cmd(NavigationAction action) const;
};

} // namespace mechdog
