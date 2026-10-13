/**
 * U-5 过渡期放宽单测 (2026-10-13 落地; 会签口径 = 默认严格, 放宽显式开启)
 *   背景: is_fall_risk() 是 fail-closed —— 无数据 / 无效读数 / 距离超阈值一律判"有坠落风险"
 *         (宁可误判不漏判)。副作用: 过渡期台架只接部分超声(无底部)时, 永久 STOP 无法行走。
 *   放宽 (relax_missing_bottom=true, 仅过渡期台架): 仅"有效(valid&&finite)且距离>阈值"的
 *         真实悬崖读数才判风险; "无信息"(无底部线程/无效/NaN) 不再拦。
 *   默认 false = 与修复前行为**完全一致** (本套用例同时是严格模式回归)。
 *   矩阵: {注入无效 / 注入NaN / 有效25cm / 有效35cm / 无bottom布局} × {strict / relax}
 */
#include "sensor_ultrasonic.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

using namespace mechdog;

static int g_checks = 0, g_fail = 0;
#define CHECK(cond) do { ++g_checks; if (!(cond)) { ++g_fail; \
    std::cout << "  FAIL " << __func__ << ":" << __LINE__ << "  " << #cond << std::endl; } } while (0)

static UltrasonicArrayData mk_bottom(bool valid, double cm) {
    UltrasonicArrayData d;
    d.bottom.valid = valid;
    d.bottom.distance_cm = cm;
    return d;
}

// 严格(默认): 无效 ⇒ 判坠; 有效近距 ⇒ 不判; 有效超阈 ⇒ 判坠 —— 修复前行为回归
static void test_strict_default() {
    UltrasonicArrayDriver drv(get_ultrasonic_layout());
    drv.set_inject_timeout_sec(10.0);
    CHECK(drv.relax_missing_bottom() == false);          // 默认关
    drv.inject_external_data(mk_bottom(false, 400.0));
    CHECK(drv.is_fall_risk() == true);                   // 无效 ⇒ 判坠 (fail-closed)
    drv.inject_external_data(mk_bottom(true, 25.0));
    CHECK(drv.is_fall_risk() == false);                  // 有效近距 ⇒ 不判
    drv.inject_external_data(mk_bottom(true, 35.0));
    CHECK(drv.is_fall_risk() == true);                   // 有效超阈(>30cm) ⇒ 判坠
}

// 放宽: 注入路径 —— 无信息(无效/NaN)放行; 真悬崖(有效>阈值)照停
static void test_relax_inject_path() {
    UltrasonicArrayDriver drv(get_ultrasonic_layout());
    drv.set_inject_timeout_sec(10.0);
    drv.set_relax_missing_bottom(true);
    drv.inject_external_data(mk_bottom(false, 400.0));
    CHECK(drv.is_fall_risk() == false);                  // 无信息 ⇒ 不判 (放宽核心)
    drv.inject_external_data(mk_bottom(true, 25.0));
    CHECK(drv.is_fall_risk() == false);                  // 有效近距 ⇒ 不判
    drv.inject_external_data(mk_bottom(true, 35.0));
    CHECK(drv.is_fall_risk() == true);                   // 真悬崖 ⇒ 照停 (放宽不放纵)
    drv.inject_external_data(mk_bottom(false, std::numeric_limits<double>::quiet_NaN()));
    CHECK(drv.is_fall_risk() == false);                  // NaN(入口消毒为无效) ⇒ 不判
}

// 放宽: 无 bottom 传感器布局 —— !bottom_have_ 路径 (过渡期"没接底部"的真实形态)
static void test_relax_no_bottom_path() {
    auto layout = get_ultrasonic_layout();
    layout.erase("bottom");
    {
        UltrasonicArrayDriver drv(layout);
        CHECK(drv.is_fall_risk() == true);               // 严格: 没接底部 ⇒ 判坠
    }
    {
        UltrasonicArrayDriver drv(layout);
        drv.set_relax_missing_bottom(true);
        CHECK(drv.is_fall_risk() == false);              // 放宽: 没接底部 ⇒ 不判
    }
}

int main() {
    test_strict_default();
    test_relax_inject_path();
    test_relax_no_bottom_path();
    std::cout << "U-5 relax_missing_bottom: " << (g_checks - g_fail) << "/" << g_checks << " passed";
    if (g_fail) { std::cout << "  (" << g_fail << " FAILED)" << std::endl; return 1; }
    std::cout << std::endl;
    return 0;
}
