/**
 * 2.5D 高程/可通行地形单测 (P1.5)
 * 全部合成点云 (base_link 系), 不依赖相机. 复用 P1 的 segment_ground 拿平面.
 *   T1 平地: 全 Traversable, 无 Up/Down
 *   T2 凸起台阶(20cm): 台阶处 ObstacleUp
 *   T3 沟(25cm深): 沟处 CliffDown
 *   T4 无地面/只墙: fail-closed (plane invalid → hm25 不输出)
 *   T5 空/退化输入: 不崩溃
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

int main() {
    std::cout << "=== heightmap 2.5d tests ===" << std::endl;
    test_flat(); test_step_up(); test_cliff_down(); test_wall_only(); test_degenerate();
    std::cout << "=== " << g_checks << " checks, " << g_fail << " failed ===" << std::endl;
    return g_fail == 0 ? 0 : 1;
}
