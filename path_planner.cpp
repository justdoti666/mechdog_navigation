/**
 * 路径规划模块实现 (第一版: 反应式动作 -> 速度映射)
 */
#include "path_planner.h"

namespace mechdog {

VelocityCmd PathPlanner::plan(const FusionResult& fusion) {
    // A2 (v2.9.19, 复审批): STOP / REACHED_GOAL **直达零速**, 不受 ramp 限幅。
    //   旧版连 STOP 也过 ramp: 从 0.2 m/s 降到零需 4 个 200ms 周期(实测非零窗口
    //   ~600ms / 到零 ~800ms / 滑行 ~6.8cm) —— 急停场景多滑行数厘米。急停优先于平滑。
    if (fusion.recommended_action == NavigationAction::STOP ||
        fusion.recommended_action == NavigationAction::REACHED_GOAL) {
        last_linear_  = 0.0;    // 同步清零 ramp 状态: 下次起步从零重新步进
        last_angular_ = 0.0;
        return {0.0, 0.0};
    }
    VelocityCmd target = action_to_cmd(fusion.recommended_action);
    // ALG-6 (v2.2): 速度 ramp —— 用闲置的 linear_accel/angular_accel 一阶限幅,
    // 消除 FORWARD→STOP→BACKWARD 瞬时跳变。safety_node timer 5Hz, dt=0.2s。
    // P3: dt 与 ROS 胶水包 safety_node 的 200ms 发布周期隐式耦合 —— 改 on_timer
    //     周期必须同步此处, 否则 ramp 斜率静默失真 (第五轮 review 注记)。
    constexpr double dt = 0.2;
    const double max_dv = PlannerConfig::linear_accel  * dt;  // 0.3*0.2 = 0.06 m/s 每步
    const double max_dw = PlannerConfig::angular_accel * dt;  // 0.5*0.2 = 0.10 rad/s 每步
    const double intended_linear = target.linear;   // ramp 前目标 (方向反转判据, 批B A2残)
    target.linear  = clamp_step(target.linear,  last_linear_,  max_dv);
    target.angular = clamp_step(target.angular, last_angular_, max_dw);
    // v2.9.23 (批B A2 残, 二轮审查): 方向反转的**零穿越闸门** ——
    //   旧版 ramp 对"目标与当前输出异号"无处理: 命令已 BACKWARD, 输出仍按 max_dv
    //   从 +0.20 逐格滑向 0 (+0.14/+0.08/+0.02, 共多向前 ~4.8cm) 才真正反向。
    //   修复: 异号 ⇒ 本步输出 0 (一步到停; 停优先于平滑), 下一步从零向目标 ramp。
    //   注: 仅线速度; 角速度反转残留不产生位移级风险, 保持原 ramp 行为。
    if ((last_linear_ > 0.0 && intended_linear < 0.0) ||
        (last_linear_ < 0.0 && intended_linear > 0.0)) {
        target.linear = 0.0;
    }
    // S1 (N2) + T-A2a (v2.9.21 复审批): 退化期限速 —— **硬上限**, ramp 之后再封顶。
    //   旧版在 ramp 之前对目标封顶: 从 0.2 m/s 切入降级时 target 先变 0.1, 再被 ramp
    //   从 0.2 只降 0.06 ⇒ 首步输出 0.14 越限 (实测; 正确应一步到 0.10)。
    //   转向 (0.2×v_max) / 后退 (0.4×v_max) 本就在该上限之下, 不受影响。
    if (fusion.depth_degraded) {
        const double v_cap = PlannerConfig::max_linear_velocity * DegradedPolicyConfig::speed_scale;
        if (target.linear > v_cap) target.linear = v_cap;
    }
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
