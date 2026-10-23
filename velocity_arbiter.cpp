/**
 * velocity_arbiter.cpp — L0 速度仲裁层实现 (v2.9.25)
 *
 * RED 阶段桩: 直通 (尚未实现任何安全包络) —— 仅用于先红后绿的第一笔。
 */
#include "velocity_arbiter.h"

namespace mechdog {

VelocityCmd VelocityArbiter::arbitrate(const VelocityCmd& desired, const FusionResult&) const {
    return desired;
}

} // namespace mechdog
