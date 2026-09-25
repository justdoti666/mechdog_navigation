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

// ============================================================
// 用法A: 机体姿态重力对齐 —— 恢复被机身姿态吃掉的坡度预算
//
// 场景构造: 先在"重力对齐系"里造地面, 再用 Ry(pitch)·Rx(roll) 转到
//   "含机身姿态的机体系" (v_B = R(θ)·v_G), 等价于狗在姿态 θ 下看到的点云。
// 对照: 同一份点云 ①不补偿(= 现状, FIX-11 的失效) ②补偿后(= 目标行为)。
// ============================================================
static const double kAttD2R = 0.01745329251994329576;

// 重力系点阵 → 机体系 (机体抬头 pitch_deg / 左倾 roll_deg)
static PointCloud body_frame_from_gravity(const PointCloud& g,
                                          double pitch_deg, double roll_deg) {
    const double cp = std::cos(pitch_deg * kAttD2R), sp = std::sin(pitch_deg * kAttD2R);
    const double cr = std::cos(roll_deg * kAttD2R),  sr = std::sin(roll_deg * kAttD2R);
    PointCloud out;
    out.points.reserve(g.points.size());
    for (const auto& q : g.points) {
        const double x1 = q.x;                                  // Rx(roll)
        const double y1 = q.y * cr - q.z * sr;
        const double z1 = q.y * sr + q.z * cr;
        Point3D p;
        p.x =  x1 * cp + z1 * sp;                                // Ry(pitch)
        p.y =  y1;
        p.z = -x1 * sp + z1 * cp;
        out.points.push_back(p);
    }
    return out;
}

// 施加补偿 (姿态新鲜有效): 期望 gate.used == true
static void compensate(PointCloud& c, double pitch_deg, double roll_deg) {
    AttitudeSample att;
    att.pitch_deg = pitch_deg;
    att.roll_deg  = roll_deg;
    att.stamp_s   = 100.0;
    att.valid     = true;
    AttitudeGate gate;
    align_to_gravity(c, att, 100.0, 0.30, 25.0, gate);
    CHECK(gate.used);
}

// A2: 平地 + 机体俯仰/横滚 10° —— 不补偿时平面"看起来"倾斜 10°, 补偿后回到竖直
static void test_attitude_flat_budget() {
    PointCloud g;
    add_ground_patch(g, 0.5, 2.2, -1.5, 1.5, 0.06, [](double, double) { return -0.18; });

    // 俯仰
    PointCloud body_p = body_frame_from_gravity(g, 10.0, 0.0);
    auto r0 = run_seg(body_p);
    CHECK(r0.plane.valid);
    CHECK(r0.plane.nz < std::cos(9.0 * kAttD2R));      // 机体系里倾斜 ≈10°

    PointCloud c_p = body_p; compensate(c_p, 10.0, 0.0);
    auto r1 = run_seg(c_p);
    CHECK(r1.plane.valid);
    CHECK(r1.plane.nz > 0.999);                        // 补偿后法向竖直

    // 横滚 (同一个正交变换的另一轴)
    PointCloud body_r = body_frame_from_gravity(g, 0.0, 10.0);
    PointCloud c_r = body_r; compensate(c_r, 0.0, 10.0);
    auto r2 = run_seg(c_r);
    CHECK(r2.plane.valid);
    CHECK(r2.plane.nz > 0.999);
}

// A3: 物理上坡 10° + 机体**前俯** 10° (姿态与坡度反向) → 机体系表观倾角 20° > 15°
//     推导: 重力系坡面法向 n_G=(-sinφ,0,cosφ), 机体系 n_B=Ry(θ)·n_G
//           ⇒ 表观倾角 = |θ - φ| (θ 抬头为正, φ 上坡为正)
//     不补偿: 丢平面(坡被当障碍); 补偿后: 恢复为 10° 坡 → 可通行
static void test_attitude_slope_budget() {
    PointCloud g;
    add_ground_patch(g, 0.5, 2.2, -1.5, 1.5, 0.06,
                     [](double x, double) { return -0.18 + std::tan(10.0 * kAttD2R) * x; });
    PointCloud body = body_frame_from_gravity(g, -10.0, 0.0);   // 机体前俯 10°

    auto r0 = run_seg(body);
    CHECK(!r0.plane.valid);                // ← FIX-11 失效复现: |−10−10| = 20° > 15°

    PointCloud c = body; compensate(c, -10.0, 0.0);
    auto r1 = run_seg(c);
    CHECK(r1.plane.valid);                             // 补偿后只剩 10° 坡度, 可识别
    CHECK(r1.plane.nz > std::cos(11.0 * kAttD2R));
    CHECK(r1.ground_indices.size() > 500);
    CHECK(r1.negative_points.empty());                 // 坡不是坑
}

// A3b: 姿态与坡度**一致** (机体抬头 10° 走 10° 上坡) → 表观倾角 |10-10| = 0°
//      两态都能识别 ⇒ 诚实标注适用范围: 补偿的价值在"姿态与坡度反向"
//      或"姿态本身超过容限(见 A4)", 而非所有坡道场景
static void test_attitude_slope_aligned() {
    PointCloud g;
    add_ground_patch(g, 0.5, 2.2, -1.5, 1.5, 0.06,
                     [](double x, double) { return -0.18 + std::tan(10.0 * kAttD2R) * x; });
    PointCloud body = body_frame_from_gravity(g, 10.0, 0.0);

    auto r0 = run_seg(body);
    CHECK(r0.plane.valid);                             // 不补偿也已"平" (表观 0°)

    PointCloud c = body; compensate(c, 10.0, 0.0);
    auto r1 = run_seg(c);
    CHECK(r1.plane.valid);
    CHECK(r1.plane.nz > std::cos(11.0 * kAttD2R));     // 补偿后 = 真实 10° 坡, 仍在容限内
}

// A4: 平地 + 机体俯仰 16° (> 15° 容限) —— 不补偿时连平地都被拒
static void test_attitude_extreme_pitch() {
    PointCloud g;
    add_ground_patch(g, 0.5, 2.2, -1.5, 1.5, 0.06, [](double, double) { return -0.18; });
    PointCloud body = body_frame_from_gravity(g, 16.0, 0.0);

    auto r0 = run_seg(body);
    CHECK(!r0.plane.valid);                            // 平地也判"过陡" → 丢平面

    PointCloud c = body; compensate(c, 16.0, 0.0);
    auto r1 = run_seg(c);
    CHECK(r1.plane.valid);
    CHECK(r1.ground_indices.size() > 500);
}

// A5: 高度先验一致性 —— 补偿后 height_at_origin ≈ -0.18 (θ ∈ {0,10,20})
static void test_attitude_prior_consistency() {
    PointCloud g;
    add_ground_patch(g, 0.5, 2.2, -1.5, 1.5, 0.06, [](double, double) { return -0.18; });
    for (double th : {0.0, 10.0, 20.0}) {
        PointCloud body = body_frame_from_gravity(g, th, 0.0);
        compensate(body, th, 0.0);
        auto r = run_seg(body);
        CHECK(r.plane.valid);
        CHECK(std::abs(r.plane.height_at_origin() + 0.18) < 0.02);
    }
}

// A2c: 组合俯仰 + 横滚 —— 锁死旋转**顺序**(ZYX 逆序不可交换)
//      平地在机体 15°俯仰+15°横滚 下变成斜面; 补偿后必须严格回到水平;
//      若顺序写反, 残差 ~ p·r (≈3.9°) → z 起伏 ~0.14m, 必然被抓。
static void test_attitude_combined_tilt() {
    PointCloud g;
    add_ground_patch(g, 0.5, 2.2, -1.5, 1.5, 0.06, [](double, double) { return -0.18; });
    PointCloud body = body_frame_from_gravity(g, 15.0, 15.0);

    double zmin = 1e9, zmax = -1e9, zsum = 0.0;
    for (const auto& p : body.points) {
        zmin = (std::min)(zmin, p.z); zmax = (std::max)(zmax, p.z); zsum += p.z;
    }
    CHECK(zmax - zmin > 0.5);                 // 机体系里确实变成了斜面

    compensate(body, 15.0, 15.0);
    zmin = 1e9; zmax = -1e9; zsum = 0.0;
    for (const auto& p : body.points) {
        zmin = (std::min)(zmin, p.z); zmax = (std::max)(zmax, p.z); zsum += p.z;
    }
    CHECK(zmax - zmin < 1e-6);                // 补偿后严格水平 (顺序错必失败)
    CHECK(std::fabs(zsum / static_cast<double>(body.points.size()) + 0.18) < 1e-6);
}

// ============================================================
// v2.7: 确定性地面提取 (格最小拟合) —— 锁真机痛点
//   真机实测 (2026-09-13, 帧6 相机俯视地板): RANSAC 从 1.2m 高候选带盲抽三点、
//   却要求 h0 落在 ±prior_window 内 ⇒ 命中靠运气: step=8 能拟合而 step=1(更稠密)
//   反而"找不到平面"; 窗口 0.10↔0.15 结果翻车; 拟合平面偏高/偏斜 (tilt 7~15°),
//   导致地板点全判"凸起"、2.5D traversable 恒 0。
// ============================================================

static Point3D mkpt(double x, double y, double z) {
    Point3D p; p.x = x; p.y = y; p.z = z; return p;
}

// 平地板(z=floor_z, 5cm 网格) + **更稠密的倾斜干扰面** (h0 也落在先验窗内,
// 倾角 11.3° < 15° 容限 ⇒ 旧 RANSAC 会被它骗走; 但它始终在地板之上 ⇒ 格最低值仍是地板)
static PointCloud make_floor_with_contamination(double floor_z) {
    PointCloud c;
    c.frame_id = "base_link";
    for (double x = 0.6; x <= 3.0 + 1e-9; x += 0.05)
        for (double y = -1.0; y <= 1.0 + 1e-9; y += 0.05)
            c.points.push_back(mkpt(x, y, floor_z));
    for (int i = 0; i < 4000; ++i) {          // 干扰面点数远多于格数 → 内点更多
        const double x = 0.6 + 2.4 * ((i % 200) / 200.0);
        const double y = -1.0 + 2.0 * ((i / 200) % 20) / 20.0;
        c.points.push_back(mkpt(x, y, floor_z + 0.20 * x));   // 斜率 0.2 → 11.3°
    }
    return c;
}

static void test_cell_min_fit_finds_floor_under_tilted_contamination() {
    GroundSegParams p;                        // prior_z=-0.18 default; 这里把先验对准地板
    p.ground_prior_z = -0.60;
    p.prior_window = 0.10;
    const PointCloud c = make_floor_with_contamination(-0.60);

    GroundPlane plane;
    CHECK(fit_ground_plane_cells(c, p, plane) == true);       // 必须找到 (确定性)
    const double tilt = std::acos(std::min(1.0, std::max(-1.0, plane.nz))) / 0.01745329251994329576;
    CHECK(tilt < 3.0);                                        // 是**平地板**, 不是 11° 干扰面
    CHECK(std::abs(plane.height_at_origin() - (-0.60)) < 0.02);

    // 接进 segment_ground: 地板点应被判为地面 (多数), 干扰面点判障碍
    GroundSegParams p2 = p;
    p2.use_cell_min_fit = true;
    GroundSegResult seg;
    segment_ground(c, p2, seg);
    CHECK(seg.plane.valid == true);
    const double tilt2 = std::acos(std::min(1.0, std::max(-1.0, seg.plane.nz))) / 0.01745329251994329576;
    CHECK(tilt2 < 3.0);
    // 地板 49×41 = 2009 个点应全部判为地面 (零噪声, |s|=0 ≤ eps); 干扰面点 (s≥0.12) 判障碍
    CHECK(seg.ground_indices.size() >= 1900);
    CHECK(seg.obstacle_indices.size() >= 3900);
}

static void test_cell_min_fit_falls_back_when_too_few_cells() {
    GroundSegParams p;
    p.ground_prior_z = -0.60;
    PointCloud c;
    for (int i = 0; i < 10; ++i) c.points.push_back(mkpt(1.0 + 0.01 * i, 0.0, -0.60));
    GroundPlane plane;
    CHECK(fit_ground_plane_cells(c, p, plane) == false);      // 样本不足 → 交回 RANSAC
}

static void test_cell_min_fit_enforces_tilt_and_prior() {
    GroundPlane plane;
    // ① 陡坡 (30°) 作为最低表面 → 倾角超限, 必须拒绝
    {
        GroundSegParams p;
        p.ground_prior_z = -0.60;
        p.prior_window = 1.0;
        PointCloud c;
        for (double x = 0.6; x <= 3.0; x += 0.05)
            for (double y = -0.5; y <= 0.5; y += 0.1)
                c.points.push_back(mkpt(x, y, -0.60 + 0.577 * (x - 0.6)));   // tan30°
        CHECK(fit_ground_plane_cells(c, p, plane) == false);
    }
    // ② 高度先验不合格 (地板 -0.90, 先验 -0.60 ± 0.10) → 拒绝
    {
        GroundSegParams p;
        p.ground_prior_z = -0.60;
        p.prior_window = 0.10;
        PointCloud c;
        for (double x = 0.6; x <= 3.0; x += 0.05)
            for (double y = -0.5; y <= 0.5; y += 0.1)
                c.points.push_back(mkpt(x, y, -0.90));
        CHECK(fit_ground_plane_cells(c, p, plane) == false);
    }
}

// v2.7: **偏航位形** —— 相机除俯仰外还可能偏航 (实测台架 base 系 y∈[-4.58,-0.60])。
//   新路径不得依赖 base 系的 x/y 足迹 (盒/锥), 只按高度先验带过滤 ⇒ 转多少度都该找到地板。
static void test_cell_min_fit_yawed_floor() {
    GroundSegParams p;
    p.ground_prior_z = -0.60;
    p.prior_window = 0.10;
    const double yaw = 20.0 * 0.01745329251994329576;
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    PointCloud c;
    for (double x = 0.6; x <= 3.0 + 1e-9; x += 0.05)
        for (double y = -1.5; y <= 1.5 + 1e-9; y += 0.05)
            c.points.push_back(mkpt(cy * x - sy * y, sy * x + cy * y, -0.60));  // 地板绕 z 转 20°
    GroundPlane plane;
    CHECK(fit_ground_plane_cells(c, p, plane) == true);
    const double tilt = std::acos(std::min(1.0, std::max(-1.0, plane.nz))) / 0.01745329251994329576;
    CHECK(tilt < 1.5);                                   // 水平地板转多少度仍是水平
    CHECK(std::abs(plane.height_at_origin() - (-0.60)) < 0.02);
}

// v2.7: **单边下包络** —— 地面是"最低的那层大面"。
//   造: 地板(-0.60, 稀疏) + 桌面(-0.42, 仍在高度带内, 点更多) ⇒ 必须仍拟合到**地板**
//   (对称带/质心法会被拽到 -0.45 附近)。
static void test_cell_min_fit_lower_envelope_beats_dense_upper_surface() {
    GroundSegParams p;
    p.ground_prior_z = -0.60;
    p.prior_window   = 0.25;                 // 高度带 z∈[-1.45,-0.35] ⇒ 桌面(-0.42)在带内
    PointCloud c;
    for (double x = 0.6; x <= 3.0; x += 0.10)         // 地板: 粗网格 (稀疏)
        for (double y = -1.0; y <= 1.0; y += 0.10)
            c.points.push_back(mkpt(x, y, -0.60));
    for (double x = 0.7; x <= 2.5; x += 0.05)         // 桌面: 细网格 (点更多) + 高 0.18m
        for (double y = -0.6; y <= 0.6; y += 0.05)
            c.points.push_back(mkpt(x, y, -0.42));
    GroundPlane plane;
    CHECK(fit_ground_plane_cells(c, p, plane) == true);
    if (plane.valid) {
        const double tilt = std::acos(std::min(1.0, std::max(-1.0, plane.nz))) / 0.01745329251994329576;
        CHECK(tilt < 2.0);
        CHECK(std::abs(plane.height_at_origin() - (-0.60)) < 0.05);   // 在地板上, 不是 -0.45
    }
}


// ============================================================
// v2.9.16 (TDD 红->绿; 2026-09-25 师兄口径): **cell 成功时跳过 RANSAC**。
//   构造 "RANSAC 会选错" 的场景: 真地板(-0.60, 稀疏 0.10 格) + 桌面(-0.42, 更密 0.05 格, 在高度带内)。
//   RANSAC 按内点数取胜 => 会被拽到桌面; cell(下包络) 给出的是**地板**。
//   => 只有 "cell 成功就不再被 RANSAC 覆盖" 时, 平面才停在地板上。
static void test_cell_skip_ransac_keeps_cell_plane() {
    GroundSegParams p;
    p.ground_prior_z   = -0.60;
    p.prior_window     = 0.25;
    p.use_cell_min_fit = true;
    PointCloud c;
    for (double x = 0.6; x <= 3.0 + 1e-9; x += 0.10)      // 真地板: 稀疏
        for (double y = -1.0; y <= 1.0 + 1e-9; y += 0.10)
            c.points.push_back(mkpt(x, y, -0.60));
    for (double x = 0.7; x <= 2.5 + 1e-9; x += 0.05)      // 桌面: 更密 (RANSAC 内点数更多)
        for (double y = -0.6; y <= 0.6 + 1e-9; y += 0.05)
            c.points.push_back(mkpt(x, y, -0.42));
    GroundSegResult seg;
    segment_ground(c, p, seg);
    CHECK(seg.plane.valid == true);
    const double h = seg.plane.height_at_origin();
    std::cout << "  [skip_ransac] plane h0 = " << h << " m (expect floor -0.60)" << std::endl;
    CHECK(std::abs(h - (-0.60)) < 0.05);   // 只有 "跳过 RANSAC" 才成立
    CHECK(seg.used_cell == true);
    CHECK(seg.used_ransac == false);
    // 反向: 显式关掉 (回到旧行为) => RANSAC 覆盖, 平面被拽到桌面
    GroundSegParams q = p;
    q.cell_skip_ransac = false;
    GroundSegResult seg2;
    segment_ground(c, q, seg2);
    CHECK(seg2.plane.valid == true);
    const double h2 = seg2.plane.height_at_origin();
    std::cout << "  [skip_ransac=false] plane h0 = " << h2 << " m (旧行为, 期望被拽到桌面 -0.42)" << std::endl;
    CHECK(std::abs(h2 - (-0.42)) < 0.05);  // 旧行为回归保护 (种子固定, 确定性)
    CHECK(seg2.used_ransac == true);
}
// v2.9 (TDD 红→绿): **重力约束的地面拟合**
//   动机: 实机地板补丁极小(0.1~0.3 m², 21~76 格)且掠射 ⇒ 自由 3 自由度平面拟合的
//         法向不可信(实测 tilt 8.8~14.3° 乱跳、RMS 2~5cm、支撑/残差守门全过不了)。
//   方案: 地面法向由"机体姿态(IMU) + 相机安装外参"**直接给定** ⇒ 只拟合高度 d,
//         平面拟合从 3 自由度降为 1 自由度 ⇒ 小补丁也稳定。
//   本用例: 仅 5x5 格(0.25m×0.25m) + ±4mm 噪声; 约束后必须恢复 <1° 的真实水平面。
// ============================================================
static void test_gravity_constrained_fit_on_tiny_patch() {
    GroundSegParams p;
    p.ground_prior_z = -0.60;
    p.prior_window   = 0.25;
    p.use_cell_min_fit = true;
    p.use_expected_normal = true;            // ← 新增: 用已知法向(来自 IMU+外参)
    p.exp_nx = 0.0; p.exp_ny = 0.0; p.exp_nz = 1.0;

    PointCloud c;
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j) {
            Point3D q;
            q.x = 1.00 + i * 0.05;
            q.y = -0.10 + j * 0.05;
            // 关键: 真实深度误差是**空间相关**的(平滑), 不是白噪声 —— 白噪声会被最小二乘平掉,
            //   而平滑偏差会被 3 自由度拟合**误当成倾角**(实机就是这么出 8~14° 的)。
            const double sy = (j - 2) * 0.05;                    // -0.10 .. +0.10 m
            q.z = -0.60 + 0.15 * sy;                             // 0.2m 上 3cm 平滑偏差 ⇒ 约 8.5°
            c.points.push_back(q);
        }

    GroundPlane pl;
    CHECK(fit_ground_plane_cells(c, p, pl) == true);
    if (pl.valid) {
        const double tilt = std::acos(std::min(1.0, std::max(-1.0, pl.nz))) / 0.01745329251994329576;
        CHECK(tilt < 1.0);                                    // 约束后必须接近真实水平
        CHECK(std::abs(pl.height_at_origin() - (-0.60)) < 0.05);
    }

    // 对照组: 关掉约束 ⇒ 记录自由拟合在这点小补丁上的抖动
    GroundSegParams q2 = p; q2.use_expected_normal = false;
    GroundPlane pq;
    if (fit_ground_plane_cells(c, q2, pq) && pq.valid) {
        const double tq = std::acos(std::min(1.0, std::max(-1.0, pq.nz))) / 0.01745329251994329576;
        std::cout << "  [对照] 自由拟合 tilt=" << tq << " deg  (约束后应 <1 deg)" << std::endl;
    }
}

int main() {
    test_gravity_constrained_fit_on_tiny_patch();   // v2.9
    test_cell_min_fit_finds_floor_under_tilted_contamination();   // v2.7
    test_cell_min_fit_falls_back_when_too_few_cells();            // v2.7
    test_cell_min_fit_enforces_tilt_and_prior();                  // v2.7
    test_cell_min_fit_yawed_floor();                              // v2.7: 偏航位形
    test_cell_min_fit_lower_envelope_beats_dense_upper_surface();  // v2.7: 单边下包络
    test_cell_skip_ransac_keeps_cell_plane();                      // v2.9.16: cell 成功跳过 RANSAC
    std::cout << "=== ground segmentation tests ===" << std::endl;
    test_baseline_flat_ground();
    test_pit_detected();
    test_stairs_down();
    test_doorway_not_marked();
    test_tilt_tolerance();
    test_wall_only_fail_closed();
    test_degenerate_inputs();
    test_perf_smoke();
    test_attitude_flat_budget();          // 用法A A2
    test_attitude_combined_tilt();        // 用法A A2c (锁旋转顺序)
    test_attitude_slope_budget();         // 用法A A3
    test_attitude_slope_aligned();        // 用法A A3b (姿态与坡度一致)
    test_attitude_extreme_pitch();        // 用法A A4
    test_attitude_prior_consistency();    // 用法A A5
    std::cout << "=== " << g_checks << " checks, " << g_fail << " failed ===" << std::endl;
    return g_fail == 0 ? 0 : 1;
}
