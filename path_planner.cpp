/**
 * 路径规划模块实现 (第一版: 反应式动作 -> 速度映射)
 */
#include "path_planner.h"

namespace mechdog {

VelocityCmd PathPlanner::plan(const FusionResult& fusion) {
    return action_to_cmd(fusion.recommended_action);
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
