/**
 * L0 速度仲裁层单测 (v2.9.25, 升级路线 L0)
 *
 * 语义约定 (与 velocity_arbiter.h 头注释一致):
 *   ① 只减不增: |out| <= |desired|, 不改变符号 (除非归零)
 *   ② 不产生动作: 感知层 TURN/BACKWARD 只能"禁止前进分量", 不生成转向/后退
 *   ③ 不引入新阈值: 只消费 FusionResult 既有字段
 *
 * T1  健康帧直通: 无危险 ⇒ 输出与输入逐位相同 (零行为变化的前提)
 * T2  STOP 一票否决: 任意期望速度 ⇒ {0,0}
 * T3  悬崖 / 全失效 ⇒ {0,0} (fail-closed)
 * T4  REACHED_GOAL ⇒ {0,0}
 * T5  TURN_* / BACKWARD ⇒ 前进分量归零, 角速度与后退保留
 * T6  SLOW_FORWARD ⇒ 前进封顶 SLOW 档 (0.10), 不放大小速度
 * T7  depth_degraded / hidden_slowdown ⇒ 同样封顶, 双开取严
 * T8  硬限幅: 超出底盘能力的期望被截断 (前进 0.20 / 后退 0.08 / 角 0.60)
 * T9  非法输入 (NaN/Inf) ⇒ {0,0}
 * T10 性质测试: 全动作 × 全期望网格上, 只减不增/符号不变/输出有限
 * T11 不自主动作: 期望为零 ⇒ 输出恒零 (即使 action=FORWARD)
 */
#include "velocity_arbiter.h"

#include <cmath>
#include <iostream>
#include <limits>

using namespace mechdog;

static int g_checks = 0, g_fail = 0;
#define CHECK(cond) do { ++g_checks; if (!(cond)) { ++g_fail; \
    std::cout << "  FAIL " << __func__ << ":" << __LINE__ << "  " << #cond << std::endl; } } while (0)

static constexpr double kEps = 1e-9;
static const double kVM = PlannerConfig::max_linear_velocity;   // 0.20
static const double kWM = PlannerConfig::max_angular_velocity;  // 0.60
static const double kSlow = kVM * ArbiterConfig::slow_scale;    // 0.10
static const double kBack = kVM * ArbiterConfig::max_backward_scale; // 0.08

static FusionResult healthy() {
    FusionResult fr;
    fr.sensors_valid = true;
    fr.recommended_action = NavigationAction::FORWARD;
    return fr;
}
static FusionResult with_action(NavigationAction a) {
    FusionResult fr = healthy();
    fr.recommended_action = a;
    return fr;
}
static VelocityCmd cmd(double v, double w) { return VelocityCmd{v, w}; }

// T1: 健康帧直通 (逐位相同 —— 这是"默认接入零行为变化"的前提)
static void test_passthrough_when_healthy() {
    VelocityArbiter a;
    const VelocityCmd d = cmd(0.18, -0.25);
    VelocityCmd o = a.arbitrate(d, healthy());
    CHECK(std::fabs(o.linear - d.linear) < kEps);
    CHECK(std::fabs(o.angular - d.angular) < kEps);
}

// T2: STOP 一票否决 (含高速态)
static void test_stop_vetoes_everything() {
    VelocityArbiter a;
    const double vs[] = {-0.08, 0.0, 0.05, 0.18, 0.20};
    const double ws[] = {-0.6, 0.0, 0.3, 0.6};
    for (double v : vs) for (double w : ws) {
        VelocityCmd o = a.arbitrate(cmd(v, w), with_action(NavigationAction::STOP));
        CHECK(std::fabs(o.linear) < kEps && std::fabs(o.angular) < kEps);
    }
}

// T3: 悬崖 / 传感器全失效 ⇒ fail-closed
static void test_cliff_and_invalid_fail_closed() {
    VelocityArbiter a;
    FusionResult fr = healthy();
    fr.cliff_detected = true;
    VelocityCmd o = a.arbitrate(cmd(0.18, 0.0), fr);
    CHECK(std::fabs(o.linear) < kEps && std::fabs(o.angular) < kEps);

    FusionResult bad = healthy();
    bad.sensors_valid = false;
    o = a.arbitrate(cmd(0.18, 0.3), bad);
    CHECK(std::fabs(o.linear) < kEps && std::fabs(o.angular) < kEps);
}

// T4: 到达目标 ⇒ 零速
static void test_reached_goal_zero() {
    VelocityArbiter a;
    VelocityCmd o = a.arbitrate(cmd(0.12, 0.2), with_action(NavigationAction::REACHED_GOAL));
    CHECK(std::fabs(o.linear) < kEps && std::fabs(o.angular) < kEps);
}

// T5: 转向/后退建议 ⇒ 只禁止前进分量, 不放行自身动作
static void test_turn_and_backward_only_block_forward() {
    VelocityArbiter a;
    for (NavigationAction act : {NavigationAction::TURN_LEFT, NavigationAction::TURN_RIGHT,
                                 NavigationAction::BACKWARD}) {
        // 期望前进 ⇒ 前进分量归零, 角速度保留 (Nav2 可原地转向重规划)
        VelocityCmd o = a.arbitrate(cmd(0.18, 0.25), with_action(act));
        CHECK(std::fabs(o.linear) < kEps);
        CHECK(std::fabs(o.angular - 0.25) < kEps);

        // 期望后退 ⇒ 保留 (允许倒车脱困; 但不得越过后退上限)
        o = a.arbitrate(cmd(-0.05, 0.0), with_action(act));
        CHECK(std::fabs(o.linear + 0.05) < kEps);

        // 期望为零 ⇒ 输出为零 (不主动生成 TURN 的角速度)
        o = a.arbitrate(cmd(0.0, 0.0), with_action(act));
        CHECK(std::fabs(o.linear) < kEps && std::fabs(o.angular) < kEps);
    }
}

// T6: SLOW_FORWARD ⇒ 封顶 SLOW 档; 小速度不被放大
static void test_slow_forward_caps_only() {
    VelocityArbiter a;
    VelocityCmd o = a.arbitrate(cmd(0.20, 0.1), with_action(NavigationAction::SLOW_FORWARD));
    CHECK(std::fabs(o.linear - kSlow) < kEps);
    CHECK(std::fabs(o.angular - 0.1) < kEps);

    o = a.arbitrate(cmd(0.05, 0.0), with_action(NavigationAction::SLOW_FORWARD));
    CHECK(std::fabs(o.linear - 0.05) < kEps);   // 只减不增
}

// T7: 降级 / 隐身障碍减速 ⇒ 封顶; 双开取严
static void test_degraded_and_hidden_caps() {
    VelocityArbiter a;
    FusionResult fr = healthy();
    fr.depth_degraded = true;
    VelocityCmd o = a.arbitrate(cmd(0.20, 0.0), fr);
    CHECK(std::fabs(o.linear - kSlow) < kEps);

    FusionResult fr2 = healthy();
    fr2.hidden_slowdown = true;
    o = a.arbitrate(cmd(0.18, 0.0), fr2);
    CHECK(std::fabs(o.linear - kSlow) < kEps);

    fr2.depth_degraded = true;
    o = a.arbitrate(cmd(0.18, 0.0), fr2);
    CHECK(o.linear <= kSlow + kEps);
}

// T8: 硬限幅 (防上游参数配错)
static void test_hard_clamps() {
    VelocityArbiter a;
    VelocityCmd o = a.arbitrate(cmd(0.9, 2.0), healthy());
    CHECK(std::fabs(o.linear - kVM) < kEps);
    CHECK(std::fabs(o.angular - kWM) < kEps);

    o = a.arbitrate(cmd(-0.5, -2.0), healthy());
    CHECK(std::fabs(o.linear + kBack) < kEps);
    CHECK(std::fabs(o.angular + kWM) < kEps);
}

// T9: 非法输入 ⇒ fail-closed
static void test_non_finite_inputs() {
    VelocityArbiter a;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    CHECK(std::fabs(a.arbitrate(cmd(nan, 0.1), healthy()).linear) < kEps);
    CHECK(std::fabs(a.arbitrate(cmd(0.1, nan), healthy()).angular) < kEps);
    CHECK(std::fabs(a.arbitrate(cmd(inf, 0.0), healthy()).linear) < kEps);
    CHECK(std::fabs(a.arbitrate(cmd(0.0, -inf), healthy()).angular) < kEps);
}

// T10: 性质测试 —— 只减不增 / 符号不变 / 输出有限
static void test_monotonic_property() {
    VelocityArbiter a;
    const double vs[] = {-0.5, -0.3, -0.08, -0.01, 0.0, 0.01, 0.05, 0.1, 0.2, 0.5};
    const double ws[] = {-2.0, -0.6, -0.1, 0.0, 0.1, 0.6, 2.0};
    const NavigationAction acts[] = {
        NavigationAction::FORWARD, NavigationAction::SLOW_FORWARD,
        NavigationAction::TURN_LEFT, NavigationAction::TURN_RIGHT,
        NavigationAction::BACKWARD, NavigationAction::STOP,
        NavigationAction::REACHED_GOAL};
    for (NavigationAction act : acts) {
        FusionResult fr = with_action(act);
        for (double v : vs) for (double w : ws) {
            const VelocityCmd d = cmd(v, w);
            const VelocityCmd o = a.arbitrate(d, fr);
            CHECK(std::isfinite(o.linear) && std::isfinite(o.angular));
            CHECK(std::fabs(o.linear) <= std::fabs(d.linear) + kEps);
            CHECK(std::fabs(o.angular) <= std::fabs(d.angular) + kEps);
            if (std::fabs(o.linear) > kEps) CHECK((o.linear > 0) == (d.linear > 0));
            if (std::fabs(o.angular) > kEps) CHECK((o.angular > 0) == (d.angular > 0));
        }
    }
}

// T11: 不自主动作 —— 期望为零时, 任何 action 都不得产生速度
static void test_no_self_generated_motion() {
    VelocityArbiter a;
    const NavigationAction acts[] = {
        NavigationAction::FORWARD, NavigationAction::TURN_LEFT, NavigationAction::TURN_RIGHT,
        NavigationAction::BACKWARD, NavigationAction::SLOW_FORWARD};
    for (NavigationAction act : acts) {
        VelocityCmd o = a.arbitrate(cmd(0.0, 0.0), with_action(act));
        CHECK(std::fabs(o.linear) < kEps && std::fabs(o.angular) < kEps);
    }
}

int main() {
    std::cout << "== velocity arbiter (L0) tests ==" << std::endl;
    test_passthrough_when_healthy();
    test_stop_vetoes_everything();
    test_cliff_and_invalid_fail_closed();
    test_reached_goal_zero();
    test_turn_and_backward_only_block_forward();
    test_slow_forward_caps_only();
    test_degraded_and_hidden_caps();
    test_hard_clamps();
    test_non_finite_inputs();
    test_monotonic_property();
    test_no_self_generated_motion();
    std::cout << "\n" << g_checks << " checks, " << g_fail << " failed\n" << std::endl;
    return g_fail ? 1 : 0;
}
