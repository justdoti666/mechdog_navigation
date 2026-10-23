/**
 * velocity_arbiter.h — L0 速度仲裁层 (v2.9.25)
 *
 * 背景 (OFFLINE_TODO #16 / 升级路线 L0):
 *   safety_node 当前是"决策者": 200 ms 定时器无条件发布自己算的 action→速度,
 *   全文没有 Twist 订阅 ⇒ 师兄的 Nav2 一旦接管 /cmd_vel, 两路指令必然打架,
 *   或者只能把本节点停掉(等于旁路整套深度安全层)。
 *
 * 本类把"决策者"改成"安全包络": 输入 = 外部期望速度 (Nav2/遥控),
 * 输出 = 经过感知层约束后的速度。三条硬语义:
 *   ① 只减不增 —— 任何输出分量 |out| <= |desired|, 且不改变符号
 *      (唯一例外: 输出归零)。
 *   ② 不产生动作 —— 感知层的 TURN/BACKWARD 只用于"禁止前进分量",
 *      绝不会自己生成转向或后退指令 (那是 Nav2 的职责)。
 *      注: determine_action 里 TURN/BACKWARD 只会在 min_fwd ≤ 50cm 时出现
 *      (STOP ≤10cm / BACKWARD ≤25cm / TURN ≤50cm), 所以"禁止前进"不会变成
 *      远距离误刹; 50cm 以内侧向有障碍本来就不该继续按原速往前推。
 *   ③ 不引入新阈值 —— 只消费 FusionResult 既有语义 (recommended_action /
 *      cliff_detected / sensors_valid / depth_degraded / hidden_slowdown),
 *      不再造第二套距离判据。
 *
 * 默认关 (ROS 参数 arbiter_enable=false) ⇒ 接入前行为逐位一致。
 */
#pragma once

#include "path_planner.h"   // VelocityCmd (避免重复定义同一语义类型)
#include "sensor_fusion.h"

namespace mechdog {

/** L0 速度仲裁器 (无状态纯函数; 不持有 ramp 状态 —— 平滑由上游 Nav2 负责) */
class VelocityArbiter {
public:
    /**
     * @param desired 外部期望速度 (m/s, rad/s)
     * @param fusion  当前帧融合结果 (只读)
     * @return        安全包络后的速度; 非法输入/全失效 ⇒ {0,0}
     */
    VelocityCmd arbitrate(const VelocityCmd& desired, const FusionResult& fusion) const;
};

} // namespace mechdog
