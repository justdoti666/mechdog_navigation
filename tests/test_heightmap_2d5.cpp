/**
 * 2.5D 高程/可通行地形单测 (P1.5)
 * 全部合成点云 (base_link 系), 不依赖相机. 复用 P1 的 segment_ground 拿平面.
 *   T1 平地: 全 Traversable, 无 Up/Down
 *   T2 凸起台阶(20cm): 台阶处 ObstacleUp
 *   T3 沟(25cm深): 沟处 CliffDown
 *   T4 无地面/只墙: fail-closed (plane invalid → hm25 不输出)
 *   T5 空/退化输入: 不崩溃
 *   T6 整体坡度过陡 (FIX-09): 外部构造的 seg 倾角 >slope_max(20°) → 已测格全翻 TooSteep;
 *      倾角 = 上游容限 15° → 不触发 (证明该分支在生产路径不可达)
 */
#include "heightmap_2d5.h"
#include "ground_segmentation.h"

#include <cmath>
#include <iostream>
#include <string>

using namespace mechdog;

static int g_checks = 0, g_fail = 0;
#define CHECK(cond) do { ++g_checks; if (!(cond)) { ++g_fail; \
    std::cout << "  FAIL " << __func__ << ":" << __LINE__ << "  " << #cond << std::endl; } } while (0)

template <typename F>
static void add_ground_patch(PointCloud& c, double x0, double x1,
                             double y0, double y1, double step, F&& z_of) {
    for (double x = x0; x <= x1 + 1e-9; x += step)
        for (double y = y0; y <= y1 + 1e-9; y += step) {
            Point3D p; p.x = x; p.y = y; p.z = z_of(x, y); c.points.push_back(p);
        }
}
static void add_wall(PointCloud& c, double wx, double y0, double y1,
                     double z0, double z1, double step = 0.08) {
    for (double y = y0; y <= y1 + 1e-9; y += step)
        for (double z = z0; z <= z1 + 1e-9; z += step) {
            Point3D p; p.x = wx; p.y = y; p.z = z; c.points.push_back(p);
        }
}

static void run(const PointCloud& c, HeightMap25Result& hm) {
    GroundSegParams g; g.prior_window = 0.10;
    GroundSegResult seg;
    segment_ground(c, g, seg);
    HeightMap25Config cfg;
    build_heightmap_25(c, seg, cfg, hm);
}

// T1 平地
static void test_flat() {
    PointCloud c;
    add_ground_patch(c, 0.5, 3.0, -1.5, 1.5, 0.05, [](double, double) { return -0.18; });
    HeightMap25Result hm; run(c, hm);
    CHECK(hm.valid);
    CHECK(hm.count_traversable > 0);
    CHECK(hm.count_up == 0 && hm.count_down == 0);
}

// T2 凸起台阶: 1.2m 处一个 20cm 高的凸起(盒)
static void test_step_up() {
    PointCloud c;
    add_ground_patch(c, 0.5, 3.0, -1.5, 1.5, 0.05, [](double, double) { return -0.18; });
    // 1.2~1.5m 放一个半米高盒
    for (double x = 1.2; x <= 1.5; x += 0.05)
        for (double y = -0.3; y <= 0.3; y += 0.05)
            for (double z = -0.18; z <= 0.3; z += 0.05) {
                Point3D p; p.x = x; p.y = y; p.z = z; c.points.push_back(p);
            }
    HeightMap25Result hm; run(c, hm);
    CHECK(hm.valid);
    // 凸起处应有 ObstacleUp
    int col, row;
    CHECK(hm.world_to_index(1.35, 0.0, col, row));
    CHECK(hm.flag[static_cast<size_t>(row) * hm.cols + col] == CellFlag::ObstacleUp);
}

// T3 沟: 1.5~1.8m 处 25cm 深 (沟底可扫到)
static void test_cliff_down() {
    PointCloud c;
    add_ground_patch(c, 0.5, 1.5, -1.5, 1.5, 0.05, [](double, double) { return -0.18; });
    add_ground_patch(c, 1.8, 3.0, -1.5, 1.5, 0.05, [](double, double) { return -0.44; }); // 低 26cm
    // 沟底 (可扫到的浅沟底, 低于主地面 25cm)
    add_ground_patch(c, 1.55, 1.75, -1.0, 1.0, 0.05, [](double, double) { return -0.43; });
    // 沟壁
    for (double z = -0.20; z >= -0.43; z -= 0.04) {
        Point3D p; p.x = 1.51; p.y = 0.0; p.z = z; c.points.push_back(p);
    }
    HeightMap25Result hm; run(c, hm);
    CHECK(hm.valid);
    int col, row;
    // 沟中心应判 CliffDown
    CHECK(hm.world_to_index(1.65, 0.0, col, row));
    CHECK(hm.flag[static_cast<size_t>(row) * hm.cols + col] == CellFlag::CliffDown);
}

// T4 只墙 → fail-closed
static void test_wall_only() {
    PointCloud c;
    add_wall(c, 2.0, -1.2, 1.2, -0.10, 1.2);
    HeightMap25Result hm; run(c, hm);
    CHECK(!hm.valid);  // 平面无效, 不输出
}

// T5 空/退化
static void test_degenerate() {
    HeightMap25Result hm;
    build_heightmap_25(PointCloud{}, GroundSegResult{}, HeightMap25Config{}, hm);
    CHECK(!hm.valid);  // 空平面无效 → 不输出
}

// ============================================================
// T6 TooSteep 语义锁定 (FIX-09)
// 生产路径不可达: segment_ground 的 plane_max_tilt_deg(默认 15°) 会先拒掉 >15° 的
// 平面, 而本处阈值 slope_max 默认 20° → acos(nz) 永不 > 20°。故这里**直接构造 seg**
// 调用 build_heightmap_25, 把现有语义钉住, 以免将来放宽上游容限时踩到"整图一刀切"。
// 关键: 点云必须落在所给平面上 (带符号距离 s≈0), 否则会先被判 ObstacleUp/CliffDown。
// ============================================================
static const double kDeg2Rad = 3.14159265358979323846 / 180.0;

// 造"点云贴合该倾角平面"的场景: 平面 nx*x + nz*z + d = 0, 原点处高度 = -0.18
static void make_tilted_scene(PointCloud& c, GroundSegResult& seg, double tilt_deg) {
    const double nx = std::sin(tilt_deg * kDeg2Rad);
    const double nz = std::cos(tilt_deg * kDeg2Rad);
    const double d  = 0.18 * nz;
    seg = GroundSegResult{};
    seg.plane.valid   = true;
    seg.plane.nx      = nx;
    seg.plane.ny      = 0.0;
    seg.plane.nz      = nz;
    seg.plane.d       = d;
    seg.plane.inliers = 1000;
    add_ground_patch(c, 0.5, 3.0, -1.5, 1.5, 0.05,
                     [nx, nz, d](double x, double) { return -(nx * x + d) / nz; });
}

static void test_too_steep() {
    // ① 倾角 25° (> slope_max 20°) → 已测到的可通行格全部翻成 TooSteep
    {
        PointCloud c; GroundSegResult seg; make_tilted_scene(c, seg, 25.0);
        HeightMap25Result hm;
        build_heightmap_25(c, seg, HeightMap25Config{}, hm);
        CHECK(hm.valid);
        CHECK(hm.count_steep > 0);          // 触发
        CHECK(hm.count_traversable == 0);   // 一刀切: 全被翻走 (含脚下平地)
        CHECK(hm.count_unknown > 0);        // 未扫到的格仍 Unknown (不是整图清空)
        CHECK(hm.count_unknown + hm.count_traversable + hm.count_steep +
              hm.count_up + hm.count_down == hm.cols * hm.rows);   // 计数自洽
    }
    // ② 倾角 15° (= 上游 plane_max_tilt_deg 上限) → 不触发: 证明生产路径不可达
    {
        PointCloud c; GroundSegResult seg; make_tilted_scene(c, seg, 15.0);
        HeightMap25Result hm;
        build_heightmap_25(c, seg, HeightMap25Config{}, hm);
        CHECK(hm.valid);
        CHECK(hm.count_steep == 0);
        CHECK(hm.count_traversable > 0);    // 正常可通行
    }
    // ③ 阈值两侧: 19° 不翻 / 21° 翻 (比较为严格 >; 避开 20.0° 的浮点边界)
    {
        PointCloud c19; GroundSegResult s19; make_tilted_scene(c19, s19, 19.0);
        HeightMap25Result h19;
        build_heightmap_25(c19, s19, HeightMap25Config{}, h19);
        CHECK(h19.count_steep == 0);
        PointCloud c21; GroundSegResult s21; make_tilted_scene(c21, s21, 21.0);
        HeightMap25Result h21;
        build_heightmap_25(c21, s21, HeightMap25Config{}, h21);
        CHECK(h21.count_steep > 0);
    }
}


// ============================================================
// v2.8: 视场楔形 (IMPL_PLAN_GRID_WEDGE) —— TDD 红→绿
//   问题: 网格 4848 格中实测只有 65 格有数据, 其余"未知"来自"网格远超视场"。
//   方案: 只在视场楔形内统计/判可通行; 覆盖率口径改 in-FOV; 默认关闭=历史行为不变。
// ============================================================
static HeightMap25Result run_wedge(const PointCloud& c, bool wedge) {
    GroundSegParams g; g.prior_window = 0.10;
    GroundSegResult seg;
    segment_ground(c, g, seg);
    HeightMap25Config cfg;          // 默认 0.6~3.0m, y±2.5m, 5cm
    cfg.wedge_only = wedge;
    HeightMap25Result hm;
    build_heightmap_25(c, seg, cfg, hm);
    return hm;
}

// T7 楔形: 只统计视场内的格; 覆盖率、格归属都可测
static void test_wedge_in_fov_and_coverage() {
    PointCloud c;
    // 注意: 点距必须 < 格距(0.05), 否则"点距=格距"会被浮点除法切成空格 ⇒ 假未知(踩过)
    add_ground_patch(c, 0.6, 3.0, -2.4, 2.4, 0.025, [](double, double) { return -0.18; });
    HeightMap25Result on = run_wedge(c, true);
    CHECK(on.valid);
    CHECK(on.count_in_fov > 0);
    CHECK(on.count_in_fov < on.cols * on.rows);          // 楔形确实排除了格
    std::cout << "  [in-FOV 诊断] 格=" << on.cols * on.rows
              << " in_fov=" << on.count_in_fov
              << " fov_unknown=" << on.count_fov_unknown
              << " cov=" << on.fov_coverage()
              << " unknown(全体)=" << on.count_unknown
              << " trav=" << on.count_traversable
              << " up=" << on.count_up << " down=" << on.count_down << std::endl;
    // 定位未知格分布 (排查: 均匀地板不应有未知格)
    double ux0 = 1e9, ux1 = -1e9, uy0 = 1e9, uy1 = -1e9; int ucnt = 0;
    for (int rr = 0; rr < on.rows; ++rr)
        for (int cc = 0; cc < on.cols; ++cc)
            if (on.in_fov(cc, rr) && on.flag[rr * on.cols + cc] == CellFlag::Unknown) {
                double wx = 0, wy = 0; on.index_to_world(cc, rr, wx, wy); ++ucnt;
                if (wx < ux0) ux0 = wx; if (wx > ux1) ux1 = wx;
                if (wy < uy0) uy0 = wy; if (wy > uy1) uy1 = wy;
            }
    std::cout << "  [未知格分布] n=" << ucnt << "  x[" << ux0 << "," << ux1
              << "]  y[" << uy0 << "," << uy1 << "]" << std::endl;
    CHECK(on.fov_coverage() > 0.95);                     // 楔形内几乎全被地面覆盖
    // 逐格归属: |y| <= 0.561*x + 0.15
    int c1 = 0, r1 = 0;
    CHECK(on.world_to_index(1.0, 0.40, c1, r1) && on.in_fov(c1, r1));    // 0.40 <= 0.711 ✓
    CHECK(on.world_to_index(1.0, 0.90, c1, r1) && !on.in_fov(c1, r1));   // 0.90 > 0.711 ✗
    CHECK(on.world_to_index(2.0, 1.20, c1, r1) && on.in_fov(c1, r1));    // 1.20 <= 1.272 ✓
    CHECK(on.world_to_index(2.0, 1.50, c1, r1) && !on.in_fov(c1, r1));   // 1.50 > 1.272 ✗
}

// T8 legacy 默认: 完全等价历史 (全格统计), 且计数自洽不变
static void test_wedge_legacy_equivalence() {
    PointCloud c;
    add_ground_patch(c, 0.6, 3.0, -2.4, 2.4, 0.025, [](double, double) { return -0.18; });
    HeightMap25Result off = run_wedge(c, false);
    CHECK(off.valid);
    CHECK(off.count_in_fov == off.cols * off.rows);      // legacy: 不排除任何格
    CHECK(off.count_unknown + off.count_traversable + off.count_up +
          off.count_down + off.count_steep == off.cols * off.rows);
    // 与历史一致: 未知格 = in_fov 内未知格 (legacy 下二者相同)
    CHECK(off.count_fov_unknown == off.count_unknown);
}

// v2.9.3 深度质量守门: 用例数字取自实机真值 (对地帧 valid≈74%/点≈2.5万; 事故帧全 0)
static void test_depth_quality_gate() {
    DepthQualityIssue why = DepthQualityIssue::Ok;

    // 实机事故复盘: 深度全 0 (USB 重枚举) ⇒ 曾造假坑 down=22~27 → 13/13 STOP
    CHECK(!depth_quality_ok(0.0, 0, why));
    CHECK(why == DepthQualityIssue::NoValidPixels);

    // 大面积失效 (5% 有效) ⇒ 未就绪
    CHECK(!depth_quality_ok(0.05, 800, why));

    // 帧"可用"但点数塌缩 ⇒ 未就绪
    CHECK(!depth_quality_ok(0.80, 100, why));
    CHECK(why == DepthQualityIssue::TooFewPoints);

    // NaN (上游算出坏值) ⇒ fail-closed
    CHECK(!depth_quality_ok(std::nan(""), 5000, why));

    // 实机对地帧 (valid=0.744, 点=25689) ⇒ 放行
    CHECK(depth_quality_ok(0.744, 25689, why));
    CHECK(why == DepthQualityIssue::Ok);

    // 边界: 恰好达标 ⇒ 放行; 略低 ⇒ 拦截
    CHECK(depth_quality_ok(0.25, 300, why));
    CHECK(!depth_quality_ok(0.249, 5000, why));
}

int main() {
    test_depth_quality_gate();
    std::cout << "=== heightmap 2.5d tests ===" << std::endl;
    test_flat(); test_step_up(); test_cliff_down(); test_wall_only(); test_degenerate();
    test_too_steep();
    test_wedge_in_fov_and_coverage();   // v2.8
    test_wedge_legacy_equivalence();    // v2.8
    std::cout << "=== " << g_checks << " checks, " << g_fail << " failed ===" << std::endl;
    return g_fail == 0 ? 0 : 1;
}
