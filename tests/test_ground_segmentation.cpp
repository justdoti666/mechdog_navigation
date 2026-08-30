/**
 * 地面分割与负障碍检测单测 (P1)
 *
 * 全部使用合成点云 (base_link 系), 不依赖相机:
 *   T1 基线: 平地+墙          → 平面正确, 零负障碍
 *   T2 坑:   平地+1.5m处20cm深坑 → 坑被标, 位置正确
 *   T3 下行台阶: 3 级 15cm      → 逐级标出
 *   T4 门口试金石: 无回波带+后方地面同高 → 必须不标
 *   T5 俯仰误差 3°: 平地倾斜    → RANSAC 恢复真实平面, 零负障碍
 *   T6 只看墙: 无地面          → fail-closed, 无输出
 *   T7 空点云 / 退化输入       → 不崩溃
 *
 * 运行: ctest 或直接执行 test_ground_segmentation (风格与 test_fusion 一致: CHECK + 计数)
 */
#include "ground_segmentation.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

using namespace mechdog;

static int g_checks = 0;
static int g_fail = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        ++g_checks;                                                        \
        if (!(cond)) {                                                     \
            ++g_fail;                                                      \
            std::cout << "  FAIL " << __func__ << ":" << __LINE__ << "  "  \
                      << #cond << std::endl;                               \
        }                                                                  \
    } while (0)

// ---- 合成场景构造工具 ----

// 平地点阵: x∈[x0,x1], y∈[y0,y1], 间距 step, 高度 z(x) 由闭包给 (支持倾斜/台阶)
template <typename F>
static void add_ground_patch(PointCloud& c, double x0, double x1,
                             double y0, double y1, double step, F&& z_of) {
    for (double x = x0; x <= x1 + 1e-9; x += step) {
        for (double y = y0; y <= y1 + 1e-9; y += step) {
            Point3D p;
            p.x = x; p.y = y; p.z = z_of(x, y);
            c.points.push_back(p);
        }
    }
}

// 竖直墙面点阵: x = wx, y∈[y0,y1], z∈[z0,z1]
static void add_wall(PointCloud& c, double wx, double y0, double y1,
                     double z0, double z1, double step = 0.08) {
    for (double y = y0; y <= y1 + 1e-9; y += step) {
        for (double z = z0; z <= z1 + 1e-9; z += step) {
            Point3D p;
            p.x = wx; p.y = y; p.z = z;
            c.points.push_back(p);
        }
    }
}

// 去除矩形区域内的点 (挖坑/开门用)
static void remove_region(PointCloud& c, double x0, double x1,
                          double y0, double y1) {
    std::vector<Point3D> keep;
    keep.reserve(c.points.size());
    for (const auto& p : c.points) {
        if (p.x >= x0 && p.x <= x1 && p.y >= y0 && p.y <= y1) continue;
        keep.push_back(p);
    }
    c.points = std::move(keep);
}

static GroundSegResult run_seg(const PointCloud& c, double prior_window = 0.10) {
    GroundSegParams p;
    p.prior_window = prior_window;  // 手持/宽松场景可放大
    GroundSegResult r;
    segment_ground(c, p, r);
    return r;
}

// ---- T1 基线: 平地 + 远处墙 ----
static void test_baseline_flat_ground() {
    PointCloud c;
    add_ground_patch(c, 0.5, 2.2, -1.5, 1.5, 0.06, [](double, double) { return -0.18; });
    add_wall(c, 2.5, -1.0, 1.0, -0.18, 1.0);
    auto r = run_seg(c);
    CHECK(r.plane.valid);
    CHECK(std::abs(r.plane.height_at_origin() + 0.18) < 0.01);
    CHECK(r.plane.nz > 0.99);                    // 法向竖直
    CHECK(r.negative_points.empty());            // 平地零负障碍
    CHECK(r.ground_indices.size() > 500);        // 地面主体被识别
}

// ---- T2 坑: 1.5m 处 0.4m 宽、20cm 深的坑 ----
static void test_pit_detected() {
    PointCloud c;
    add_ground_patch(c, 0.5, 2.2, -1.5, 1.5, 0.06, [](double, double) { return -0.18; });
    // 挖坑 + 坑底 (低 20cm) + 近侧坑壁若干点
    remove_region(c, 1.3, 1.7, -0.2, 0.2);
    add_ground_patch(c, 1.32, 1.68, -0.18, 0.18, 0.05,
                     [](double, double) { return -0.38; });
    for (double z = -0.20; z >= -0.36; z -= 0.04) {
        Point3D p;
        p.x = 1.31; p.y = 0.0; p.z = z;
        c.points.push_back(p);
    }
    auto r = run_seg(c);
    CHECK(r.plane.valid);
    CHECK(std::abs(r.plane.height_at_origin() + 0.18) < 0.01);  // 主平面仍是地面
    CHECK(!r.negative_points.empty());
    double x_min = 1e9, x_max = -1e9, y_min = 1e9, y_max = -1e9;
    for (const auto& p : r.negative_points) {
        x_min = std::min(x_min, p.x); x_max = std::max(x_max, p.x);
        y_min = std::min(y_min, p.y); y_max = std::max(y_max, p.y);
    }
    CHECK(x_min >= 1.2 && x_max <= 1.8);         // 位置落在坑区域 (±1 cell)
    CHECK(y_min >= -0.3 && y_max <= 0.3);
    for (const auto& p : r.negative_points) {
        CHECK(std::abs(p.z + 0.18) < 0.02);      // 标记点位于地面平面高度
    }
}

// ---- T3 下行台阶: 3 级, 每级 15cm ----
static void test_stairs_down() {
    PointCloud c;
    add_ground_patch(c, 0.5, 1.0, -1.2, 1.2, 0.06, [](double, double) { return -0.18; });
    add_ground_patch(c, 1.0, 1.5, -1.2, 1.2, 0.06, [](double, double) { return -0.33; });
    add_ground_patch(c, 1.5, 2.0, -1.2, 1.2, 0.06, [](double, double) { return -0.48; });
    add_ground_patch(c, 2.0, 2.5, -1.2, 1.2, 0.06, [](double, double) { return -0.63; });
    auto r = run_seg(c);
    CHECK(r.plane.valid);
    CHECK(std::abs(r.plane.height_at_origin() + 0.18) < 0.01);  // 约束保证第一级被选中
    // 逐级触发: 负障碍点应从第一级台阶沿 (x≈1.0) 一路延伸到最远级
    CHECK(r.negative_points.size() > 30);
    double x_min = 1e9;
    for (const auto& p : r.negative_points) x_min = std::min(x_min, p.x);
    CHECK(x_min >= 0.95 && x_min <= 1.15);       // 首个标记在第一级台阶沿
    CHECK(r.negative_points.back().x > 2.0);     // 一直标到深处
}

// ---- T4 试金石: 门口/无回波带, 后方地面同高 → 必须不标 ----
static void test_doorway_not_marked() {
    PointCloud c;
    add_ground_patch(c, 0.5, 2.2, -1.5, 1.5, 0.06, [](double, double) { return -0.18; });
    // 挖掉一条 0.3m 宽的无回波带 (模拟门框阴影/遮挡), 后方地面连续同高
    remove_region(c, 1.5, 1.8, -0.4, 0.4);
    auto r = run_seg(c);
    CHECK(r.plane.valid);
    CHECK(r.negative_points.empty());            // 同高地面绝不能标负障碍
}

// ---- T5 俯仰误差 3°: 平面倾斜, RANSAC 应恢复真实平面 ----
static void test_tilt_tolerance() {
    PointCloud c;
    constexpr double kTan3 = 0.05240777928304121;
    add_ground_patch(c, 0.5, 2.5, -1.5, 1.5, 0.06,
                     [](double x, double) { return -0.18 - x * kTan3; });
    auto r = run_seg(c);
    CHECK(r.plane.valid);
    CHECK(std::abs(r.plane.height_at_origin() + 0.18) < 0.01);
    CHECK(r.plane.nz >= std::cos(5.0 * 0.01745329251994329576));  // 恢复出 ~3° 而非拒判
    CHECK(r.negative_points.empty());
}

// ---- T6 只看墙: 无地面 → fail-closed 无输出 ----
static void test_wall_only_fail_closed() {
    PointCloud c;
    add_wall(c, 2.0, -1.2, 1.2, -0.10, 1.2);
    auto r = run_seg(c);
    CHECK(!r.plane.valid);
    CHECK(r.ground_indices.empty());
    CHECK(r.negative_points.empty());
}

// ---- T7 退化输入: 空云 / 少量点 ----
static void test_degenerate_inputs() {
    PointCloud empty;
    auto r0 = run_seg(empty);
    CHECK(!r0.plane.valid);
    CHECK(r0.negative_points.empty());

    PointCloud tiny;
    Point3D a; a.x = 1.0; a.y = 0.0; a.z = -0.18;
    Point3D b; b.x = 1.1; b.y = 0.1; b.z = -0.18;
    tiny.points = {a, b};
    auto r1 = run_seg(tiny);
    CHECK(!r1.plane.valid);
}

// ---- 性能冒烟: 3 万+点全流程 (只打印, 不做脆断言) ----
static void test_perf_smoke() {
    PointCloud c;
    add_ground_patch(c, 0.5, 3.0, -2.0, 2.0, 0.03, [](double, double) { return -0.18; });
    remove_region(c, 1.3, 1.7, -0.2, 0.2);                   // 先挖坑
    add_ground_patch(c, 1.3, 1.7, -0.2, 0.2, 0.03,
                     [](double, double) { return -0.38; });  // 再铺坑底
    std::cout << "  perf: pts=" << c.points.size();
    auto t0 = std::chrono::steady_clock::now();
    auto r = run_seg(c);
    auto ms = std::chrono::duration<double, std::milli>(
                  std::chrono::steady_clock::now() - t0).count();
    std::cout << "  seg=" << ms << "ms  neg=" << r.negative_points.size() << std::endl;
    CHECK(r.plane.valid);
    CHECK(!r.negative_points.empty());
}

int main() {
    std::cout << "=== ground segmentation tests ===" << std::endl;
    test_baseline_flat_ground();
    test_pit_detected();
    test_stairs_down();
    test_doorway_not_marked();
    test_tilt_tolerance();
    test_wall_only_fail_closed();
    test_degenerate_inputs();
    test_perf_smoke();
    std::cout << "=== " << g_checks << " checks, " << g_fail << " failed ===" << std::endl;
    return g_fail == 0 ? 0 : 1;
}
