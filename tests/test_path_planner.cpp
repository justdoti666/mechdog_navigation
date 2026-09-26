/**
 * 路径规划单测 (A2 回归, v2.9.19)
 *   T1 STOP 直停: 高速态下 STOP 必须**本帧即为零** —— 旧版 ramp 从 0.2 m/s 降到零需
 *      4 个 200ms 周期(实测非零窗口 ~600ms / 到零 ~800ms / 滑行 ~6.8cm)。
 *   T2 连续 STOP: 恒为 {0,0} (ramp 状态已清零, 不应"回弹")。
 *   T3 STOP→FORWARD: ramp 从零重新起步, 首步 = linear_accel*dt = 0.06 m/s。
 *   T4 平滑不回归: FORWARD 步进 / FORWARD→SLOW_FORWARD 仍按 max_dv 限幅(不跳变)。
 *   T5 REACHED_GOAL: 同样直达零速。
 *   T6 角速度: TURN 仍 ramp; STOP 时角速度也立即归零。
 *   T7-T9 S1 (N2): 退化期限速 —— FORWARD 封顶 SLOW 档(v_max*0.5); STOP 仍直达零速;
 *        解除(好帧)后能升回 v_max。
 */
#include "path_planner.h"

#include <cmath>
#include <iostream>

using namespace mechdog;

static int g_checks = 0, g_fail = 0;
#define CHECK(cond) do { ++g_checks; if (!(cond)) { ++g_fail; \
    std::cout << "  FAIL " << __func__ << ":" << __LINE__ << "  " << #cond << std::endl; } } while (0)

static constexpr double kEps = 1e-9;

static FusionResult with_action(NavigationAction a) {
    FusionResult fr;
    fr.recommended_action = a;
    return fr;
}

// T1: STOP 直达零速 (高速度态下)
static void test_stop_is_immediate() {
    PathPlanner p;
    VelocityCmd c;
    for (int i = 0; i < 10; ++i) c = p.plan(with_action(NavigationAction::FORWARD));
    CHECK(std::fabs(c.linear - PlannerConfig::max_linear_velocity) < kEps);   // ramp 已到稳态
    c = p.plan(with_action(NavigationAction::STOP));
    CHECK(std::fabs(c.linear) < kEps);       // 本帧即零 (旧版会是 ~0.14)
    CHECK(std::fabs(c.angular) < kEps);
}

// T2: 连续 STOP 恒零
static void test_stop_stays_zero() {
    PathPlanner p;
    for (int i = 0; i < 10; ++i) p.plan(with_action(NavigationAction::FORWARD));
    p.plan(with_action(NavigationAction::STOP));
    for (int i = 0; i < 3; ++i) {
        VelocityCmd c = p.plan(with_action(NavigationAction::STOP));
        CHECK(std::fabs(c.linear) < kEps && std::fabs(c.angular) < kEps);
    }
}

// T3: STOP 后重新起步走 ramp (不是瞬间满速)
static void test_restart_after_stop_ramps() {
    PathPlanner p;
    for (int i = 0; i < 6; ++i) p.plan(with_action(NavigationAction::FORWARD));
    p.plan(with_action(NavigationAction::STOP));
    VelocityCmd c = p.plan(with_action(NavigationAction::FORWARD));
    CHECK(std::fabs(c.linear - 0.06) < 1e-9);      // linear_accel(0.3)*dt(0.2)
    c = p.plan(with_action(NavigationAction::FORWARD));
    CHECK(std::fabs(c.linear - 0.12) < 1e-9);
}

// T4: 平滑性不回归
static void test_ramp_smoothness_kept() {
    PathPlanner p;
    VelocityCmd c = p.plan(with_action(NavigationAction::FORWARD));
    CHECK(std::fabs(c.linear - 0.06) < 1e-9);      // 从零起步首步限幅
    for (int i = 0; i < 10; ++i) c = p.plan(with_action(NavigationAction::FORWARD));
    c = p.plan(with_action(NavigationAction::SLOW_FORWARD));   // 0.2 → 0.1
    CHECK(std::fabs(c.linear - 0.14) < 1e-9);      // 降速也按 max_dv 限幅
}

// T5: REACHED_GOAL 直达零速
static void test_reached_goal_is_immediate() {
    PathPlanner p;
    for (int i = 0; i < 10; ++i) p.plan(with_action(NavigationAction::FORWARD));
    VelocityCmd c = p.plan(with_action(NavigationAction::REACHED_GOAL));
    CHECK(std::fabs(c.linear) < kEps && std::fabs(c.angular) < kEps);
}

// T6: 角速度 ramp 保留 + STOP 立即归零
static void test_angular_ramp_and_stop() {
    PathPlanner p;
    VelocityCmd c = p.plan(with_action(NavigationAction::TURN_LEFT));
    CHECK(std::fabs(c.angular - 0.10) < 1e-9);     // angular_accel(0.5)*dt(0.2)
    for (int i = 0; i < 10; ++i) c = p.plan(with_action(NavigationAction::TURN_LEFT));
    CHECK(std::fabs(c.angular - PlannerConfig::max_angular_velocity) < kEps);
    c = p.plan(with_action(NavigationAction::STOP));
    CHECK(std::fabs(c.angular) < kEps);            // 旧版会是 ~0.5
    CHECK(std::fabs(c.linear) < kEps);
}


// ===== S1 (N2): 退化期限速 (2026-09-26) =====
// T7: 降级期 FORWARD 封顶 SLOW 档 (v_max*0.5=0.10), 不再升到 0.20
static void test_degraded_speed_cap() {
    PathPlanner p;
    FusionResult fr = with_action(NavigationAction::FORWARD);
    fr.depth_degraded = true;
    const double cap = PlannerConfig::max_linear_velocity * DegradedPolicyConfig::speed_scale;
    VelocityCmd c;
    for (int i = 0; i < 8; ++i) {
        c = p.plan(fr);
        CHECK(c.linear <= cap + kEps);              // 全程不越限
    }
    CHECK(std::fabs(c.linear - cap) < 1e-9);        // 稳态 = 0.10 (未降级时为 0.20)
}

// T8: 降级期 STOP 仍直达零速 (急停优先于任何限速)
static void test_degraded_stop_still_immediate() {
    PathPlanner p;
    FusionResult fwd = with_action(NavigationAction::FORWARD);
    fwd.depth_degraded = true;
    for (int i = 0; i < 6; ++i) p.plan(fwd);
    FusionResult st = with_action(NavigationAction::STOP);
    st.depth_degraded = true;
    VelocityCmd c = p.plan(st);
    CHECK(std::fabs(c.linear) < kEps && std::fabs(c.angular) < kEps);
}

// T9: 恢复后 FORWARD 升回 v_max (解除即复原)
static void test_degraded_release_restores() {
    PathPlanner p;
    FusionResult fr = with_action(NavigationAction::FORWARD);
    fr.depth_degraded = true;
    for (int i = 0; i < 8; ++i) p.plan(fr);
    fr.depth_degraded = false;
    VelocityCmd c;
    for (int i = 0; i < 4; ++i) c = p.plan(fr);
    CHECK(std::fabs(c.linear - PlannerConfig::max_linear_velocity) < kEps);
}

int main() {
    std::cout << "== path planner (A2) tests ==" << std::endl;
    test_stop_is_immediate();
    test_stop_stays_zero();
    test_restart_after_stop_ramps();
    test_ramp_smoothness_kept();
    test_reached_goal_is_immediate();
    test_angular_ramp_and_stop();
    test_degraded_speed_cap();          // S1: 降级限速
    test_degraded_stop_still_immediate(); // S1: 急停不受限速影响
    test_degraded_release_restores();     // S1: 解除复原
    std::cout << "\n" << g_checks << " checks, " << g_fail << " failed\n" << std::endl;
    return g_fail ? 1 : 0;
}
