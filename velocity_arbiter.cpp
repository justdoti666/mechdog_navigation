/**
 * velocity_arbiter.cpp — L0 速度仲裁层实现 (v2.9.25)
 *
 * 处理顺序 (每一步都有单测锁定, 见 tests/test_velocity_arbiter.cpp):
 *   0) 非法输入 (NaN/Inf) → {0,0}          —— fail-closed
 *   1) 硬限幅: 前进 ≤ v_max / 后退 ≤ 0.4·v_max / 角 ≤ w_max  —— 防上游参数配错
 *   2) 一票否决: 传感器全失效 / 悬崖 / STOP / REACHED_GOAL → {0,0}
 *   3) 禁止前进: 感知层给 TURN_* / BACKWARD 时, 前进分量归零 (后退与角速度保留)
 *   4) 限速取严: SLOW_FORWARD / 深度降级 / 隐身障碍减速 → 前进 ≤ 0.5·v_max
 *
 * 设计约束: 全程不放大、不改符号、不生成动作 —— 只有"归零"和"削掉超出的部分"。
 */
#include "velocity_arbiter.h"

#include <algorithm>
#include <cmath>

namespace mechdog {

namespace {
/** 对称限幅 (保持符号) */
inline double clamp_sym(double v, double lim) {
    return std::max(-lim, std::min(v, lim));
}
} // namespace

VelocityCmd VelocityArbiter::arbitrate(const VelocityCmd& desired, const FusionResult& fusion) const {
    // 0) 非法输入 fail-closed: NaN/Inf 一旦流进下游会是"看起来正常"的毒值
    //    (NaN 比较恒 false ⇒ 所有 if / downstream clamp 都会静默放过它)。
    if (!std::isfinite(desired.linear) || !std::isfinite(desired.angular)) {
        return {0.0, 0.0};
    }

    // 1) 硬限幅 (与 PlannerConfig / 底盘能力同口径)
    const double v_max = PlannerConfig::max_linear_velocity;
    const double w_max = PlannerConfig::max_angular_velocity;
    const double v_back = v_max * ArbiterConfig::max_backward_scale;
    double v = clamp_sym(desired.linear, v_max);
    double w = clamp_sym(desired.angular, w_max);
    if (v < -v_back) v = -v_back;   // 后退单独上限 (0.4·v_max, 与 planner BACKWARD 一致)

    // 2) 一票否决
    if (!fusion.sensors_valid || fusion.cliff_detected) return {0.0, 0.0};
    switch (fusion.recommended_action) {
        case NavigationAction::STOP:
        case NavigationAction::REACHED_GOAL:
            return {0.0, 0.0};
        default:
            break;
    }

    // 3) 感知层不允许前进 ⇒ 只削前进分量。
    //    注意: 不在这里生成 TURN/BACKWARD 指令 —— 转向由 Nav2 决定, 本层只拦"往前冲"。
    switch (fusion.recommended_action) {
        case NavigationAction::TURN_LEFT:
        case NavigationAction::TURN_RIGHT:
        case NavigationAction::BACKWARD:
            if (v > 0.0) v = 0.0;
            break;
        default:
            break;
    }

    // 4) 限速取严 (三条来源共用同一档位 = 0.5·v_max, 不引入新阈值)
    if (fusion.recommended_action == NavigationAction::SLOW_FORWARD ||
        fusion.depth_degraded ||
        fusion.hidden_slowdown) {
        const double v_cap = v_max * ArbiterConfig::slow_scale;
        if (v > v_cap) v = v_cap;
    }

    return {v, w};
}

} // namespace mechdog
