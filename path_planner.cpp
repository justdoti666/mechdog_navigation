/**
 * 路径规划模块实现 (第一版: 反应式动作 -> 速度映射)
 */
#include "path_planner.h"

namespace mechdog {

VelocityCmd PathPlanner::plan(const FusionResult& fusion) {
    VelocityCmd target = action_to_cmd(fusion.recommended_action);
    // ALG-6 (v2.2): 速度 ramp —— 用闲置的 linear_accel/angular_accel 一阶限幅,
    // 消除 FORWARD→STOP→BACKWARD 瞬时跳变。safety_node timer 5Hz, dt=0.2s。
    constexpr double dt = 0.2;
    const double max_dv = PlannerConfig::linear_accel  * dt;  // 0.3*0.2 = 0.06 m/s 每步
    const double max_dw = PlannerConfig::angular_accel * dt;  // 0.5*0.2 = 0.10 rad/s 每步
    target.linear  = clamp_step(target.linear,  last_linear_,  max_dv);
    target.angular = clamp_step(target.angular, last_angular_, max_dw);
    last_linear_  = target.linear;
    last_angular_ = target.angular;
    return target;
}

// ALG-6 (v2.2): 单步限幅 —— 把 target 朝 last 限制在 ±max_delta 内
double PathPlanner::clamp_step(double target, double last, double max_delta) {
    double diff = target - last;
    if (diff >  max_delta) return last + max_delta;
    if (diff < -max_delta) return last - max_delta;
    return target;
}

VelocityCmd PathPlanner::action_to_cmd(NavigationAction action) const {
    // 速度限制取自 config.h PlannerConfig
    const double v_max = PlannerConfig::max_linear_velocity;
    const double w_max = PlannerConfig::max_angular_velocity;

    switch (action) {
        case NavigationAction::FORWARD:
            return {v_max, 0.0};
        case NavigationAction::SLOW_FORWARD:
            return {v_max * 0.5, 0.0};
        case NavigationAction::TURN_LEFT:
            return {v_max * 0.2, w_max};
        case NavigationAction::TURN_RIGHT:
            return {v_max * 0.2, -w_max};
        case NavigationAction::BACKWARD:
            return {-v_max * 0.4, 0.0};
        case NavigationAction::STOP:
            return {0.0, 0.0};
        case NavigationAction::REACHED_GOAL:
        default:
            return {0.0, 0.0};
    }
}

} // namespace mechdog
