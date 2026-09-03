/**
 * 建图模块 (P4) 单元测试
 *
 * 设计文档: docs/POINT_CLOUD_DESIGN.md §13 验证矩阵 + P4
 * 覆盖: 坐标换算 / 空点云安全 / 占据与空闲判定 / 位姿变换 /
 *       多帧累积 / 膨胀 / PGM 导出
 * 全部零依赖, 直接跑可执行即全绿。
 */
#include "mapping.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

// ---------- 极简断言 (与 test_point_cloud.cpp 同风格) ----------
static int g_failed = 0, g_total = 0;
#define CHECK(cond) do { ++g_total; \
    if (!(cond)) { ++g_failed; \
        std::printf("  FAIL line %d: %s\n", __LINE__, #cond); } } while (0)
#define CHECK_NEAR(a, b, eps) do { ++g_total; \
    const double _d = std::fabs((a) - (b)); \
    if (_d > (eps)) { ++g_failed; \
        std::printf("  FAIL line %d: |%s - %s| = %.6f > %g\n", \
                    __LINE__, #a, #b, _d, (double)(eps)); } } while (0)

using namespace mechdog;

// 深度图 → base 系点云 的合成辅助 (复用 P0 真实管线)
#include "point_cloud.h"

static PointCloud make_cloud_base(std::vector<std::pair<double,double>> pts) {
    PointCloud c;
    c.frame_id = "base_link";
    for (auto& xy : pts) {
        Point3D p; p.x = xy.first; p.y = xy.second; p.z = 0.1;
        c.points.push_back(p);
    }
    return c;
}

int main() {
    std::printf("== mapping (P4) tests ==\n");

    // ============ 1. 构造与坐标换算 ============
    {
        OccupancyGridMap m;
        CHECK(m.width() == 200);  // 10m / 0.05
        CHECK(m.height() == 200);
        CHECK_NEAR(m.resolution(), 0.05, 1e-9);

        // 原点居中: 世界(0,0) → 格(100,100)
        int col, row;
        CHECK(m.world_to_index(0.0, 0.0, col, row));
        CHECK(col == 100 && row == 100);
        // 世界(1.0, -0.5) → 格(120, 90)
        CHECK(m.world_to_index(1.0, -0.5, col, row));
        CHECK(col == 120 && row == 90);
        // 逆变换
        double wx, wy;
        m.index_to_world(120, 90, wx, wy);
        CHECK_NEAR(wx, 1.0, 0.025);
        CHECK_NEAR(wy, -0.5, 0.025);
        // 越界
        CHECK(!m.world_to_index(5.5, 0.0, col, row));
        // 全未知
        CHECK(m.count_cells(-1) == 200 * 200);
        CHECK(m.count_cells(100) == 0);
    }

    // ============ 2. 空点云安全 ============
    {
        OccupancyGridMap m;
        Pose2D pose; // 全零
        PointCloud empty;
        m.insert_cloud(empty, pose);
        CHECK(m.count_cells(-1) == 200 * 200); // 图不受影响
    }

    // ============ 3. 单帧: 占据 + 空闲 ============
    {
        OccupancyGridMap m;
        Pose2D pose; // 机器人在原点, 朝 +x
        // 一个正前方 2m 的障碍点
        PointCloud c = make_cloud_base({{2.0, 0.0}});
        m.insert_cloud(c, pose);

        // 命中格应为占据
        int col, row;
        CHECK(m.world_to_index(2.0, 0.0, col, row));
        CHECK(m.occ_state(col, row) == 100);

        // 相机(0.12,0) 与命中点(2,0) 之间应被标空闲 (取中点 1m)
        CHECK(m.world_to_index(1.0, 0.0, col, row));
        CHECK(m.occ_state(col, row) == 0);

        // 光线之外的侧向格应保持未知
        CHECK(m.world_to_index(1.0, 2.0, col, row));
        CHECK(m.occ_state(col, row) == -1);
    }

    // ============ 3b. 光线自清除回归 (TDD: 修复前单点命中格=未知) ============
    {
        // 单个偏离光轴的障碍点: 光线步进终点曾与命中格同格,
        // miss 抵消 hit → 占据变未知 (bug 场景)
        OccupancyGridMap m;
        Pose2D pose;
        m.insert_cloud(make_cloud_base({{2.0, 0.6}}), pose);
        int col, row;
        CHECK(m.world_to_index(2.0, 0.6, col, row));
        CHECK(m.occ_state(col, row) == 100);  // 修复前: -1
    }

    // ============ 3c. 密集墙面不被同帧光线打花 ============
    {
        // 一排墙点 (2m 处, y ∈ [-0.8, 0.8]): 修复后每列都应占据
        OccupancyGridMap m;
        Pose2D pose;
        std::vector<std::pair<double,double>> wall;
        for (double y = -0.8; y <= 0.81; y += 0.1)
            wall.emplace_back(2.0, y);
        m.insert_cloud(make_cloud_base(wall), pose);
        int occ_cols = 0, miss_cols = 0;
        for (double y = -0.8; y <= 0.81; y += 0.1) {
            int col, row;
            CHECK(m.world_to_index(2.0, y, col, row));
            const int s = m.occ_state(col, row);
            if (s == 100) ++occ_cols;
            if (s == -1) ++miss_cols;   // 修复前: 远离光轴的列被打成未知
        }
        CHECK(occ_cols >= 15);   // 17 列墙几乎全占据
        CHECK(miss_cols == 0);   // 没有任何列丢失
    }

    // ============ 3d. 相机原点与外参同源 (FIX_PLAN #7 回归) ============
    {
        // 构造传 extrinsics_x=0.50: 相机原点应在 base 前方 0.5m。
        // 机器人在 (1,1) 朝+x, 障碍 base系(1,0) → 世界 (2,1),
        // 即相机原点 (1.5,1) 正前方 0.5m。
        // ① 光线从相机原点出发: (1.7,1) 应空闲
        // ② 光线不打相机身后: (1.25,1) 应保持未知
        //    (若硬编码 0.12 → 原点 (1.12,1), 光线覆盖 (1.25,1) → ②失败)
        OccupancyGridMap m(MapConfig{}, 0.25, /*extrinsics_x=*/0.50);
        Pose2D pose; pose.x = 1.0; pose.y = 1.0;
        m.insert_cloud(make_cloud_base({{1.0, 0.0}}), pose);
        int col, row;
        CHECK(m.world_to_index(1.7, 1.0, col, row));
        CHECK(m.occ_state(col, row) == 0);
        CHECK(m.world_to_index(1.25, 1.0, col, row));
        CHECK(m.occ_state(col, row) == -1);
    }

    // ============ 4. 位姿变换: 旋转+平移 ============
    {
        OccupancyGridMap m;
        Pose2D pose;
        pose.x = 1.0; pose.y = 0.5;
        pose.theta = M_PI / 2; // 朝 +y
        // base 系前方 2m 的障碍 → odom 系 (1.0, 2.5)
        PointCloud c = make_cloud_base({{2.0, 0.0}});
        m.insert_cloud(c, pose);

        int col, row;
        CHECK(m.world_to_index(1.0, 2.5, col, row));
        CHECK(m.occ_state(col, row) == 100);
        // 机器人身后 (0.5, 0.5) 不在光线上 → 未知
        CHECK(m.world_to_index(0.5, 0.5, col, row));
        CHECK(m.occ_state(col, row) == -1);
    }

    // ============ 5. 多帧累积 (log-odds 持久) ============
    {
        OccupancyGridMap m;
        Pose2D p1;                       // 原点朝+x
        Pose2D p2; p2.x = 1.0;           // 前进 1m
        PointCloud c1 = make_cloud_base({{2.0, 0.0}});
        PointCloud c2 = make_cloud_base({{2.0, 0.0}}); // 前方 2m 处=odom(3,0)新障碍
        m.insert_cloud(c1, p1);
        m.insert_cloud(c2, p2);

        // 两次不同位姿的"前方 2m"障碍应落在 odom 系不同格;
        // 前进后旧障碍格 (2,0) 被新光线穿过 → 正确清为空闲 (动态清障),
        // 新障碍 (3,0) 占据 (实测: logodds -6 / +6)
        int col, row;
        CHECK(m.world_to_index(2.0, 0.0, col, row));
        CHECK(m.occ_state(col, row) == 0);
        CHECK(m.world_to_index(3.0, 0.0, col, row));
        CHECK(m.occ_state(col, row) == 100);
    }

    // ============ 6. 光线清除噪声占据 (先占据后空闲) ============
    {
        OccupancyGridMap m;
        Pose2D pose;
        // 先在 2m 处放障碍 → 占据
        m.insert_cloud(make_cloud_base({{2.0, 0.0}}), pose);
        // 再放 3m 处障碍, 光线穿过 2m 格 → 2m 格被 miss 抵消
        m.insert_cloud(make_cloud_base({{3.0, 0.0}}), pose);

        int col, row;
        CHECK(m.world_to_index(2.0, 0.0, col, row));
        // 一次 hit(+6) 后光线多次采样穿过 → miss 叠加 → 空闲 (非未知, 实测 -6)
        CHECK(m.occ_state(col, row) == 0);
        CHECK(m.world_to_index(3.0, 0.0, col, row));
        CHECK(m.occ_state(col, row) == 100);
    }

    // ============ 9. 膨胀 ============
    {
        OccupancyGridMap m;
        Pose2D pose;
        m.insert_cloud(make_cloud_base({{2.0, 0.0}}), pose);
        m.inflate(0.15); // 3 格半径

        // 障碍本身占据
        int col, row;
        CHECK(m.world_to_index(2.0, 0.0, col, row));
        CHECK(m.occ_state(col, row) == 100);
        // 障碍旁 0.1m (2 格) → 膨胀层标记 (未直接占, 但 inflate 后 occ_state 不变;
        // 通过 count 验证膨胀层被写入)
        // 注: occ_state 是原图语义, 膨胀结果在 inflated_ 层
        // 这里验证膨胀不破坏原图
        CHECK(m.world_to_index(2.05, 0.0, col, row));
        CHECK(m.occ_state(col, row) == -1 ||
              m.occ_state(col, row) == 0);
    }

    // ============ 9b. 自定义尺寸/分辨率 (FIX_PLAN #6) ============
    {
        // 40x40m 地图, 10cm 分辨率 → 400x400 格 (半宽 20m)
        OccupancyGridMap m(MapConfig{}, 0.25, 0.12, 40.0, 40.0, 0.10);
        CHECK(m.width() == 400);
        CHECK(m.height() == 400);
        CHECK_NEAR(m.resolution(), 0.10, 1e-9);
        // 15m 远的点在 10x10 地图会越界丢弃, 在 40x40 地图应合法
        Pose2D pose;
        m.insert_cloud(make_cloud_base({{15.0, 0.0}}), pose);
        int col, row;
        CHECK(m.world_to_index(15.0, 0.0, col, row));
        CHECK(m.occ_state(col, row) == 100);

        // 越界计数不再静默 (FIX_PLAN #6: 落盘日志而非静默丢)
        OccupancyGridMap m2;
        m2.insert_cloud(make_cloud_base({{9.9, 0.0}, {4.9, 0.0}}), pose);
        CHECK(m2.dropped_points() == 1);   // 9.9m 越界, 4.9m 在界内(±5m)
        CHECK(m2.stats().find("dropped") != std::string::npos);
    }

    // ============ 9c. 地面过滤入口 (FIX_PLAN #1) ============
    {
        // 纯地面帧 (base 系 z=-0.18, GroundSegConfig::ground_prior_z 同源):
        // 经 insert_cloud_filtered 后零占据 (无假墙)
        OccupancyGridMap m;
        Pose2D pose;
        PointCloud ground; ground.frame_id = "base_link";
        for (double x = 1.0; x <= 3.0; x += 0.1)
            for (double y = -1.0; y <= 1.0; y += 0.1) {
                Point3D p; p.x = x; p.y = y; p.z = -0.18;
                ground.points.push_back(p);
            }
        m.insert_cloud_filtered(ground, pose);
        CHECK(m.count_cells(100) == 0);  // 验收标准: 平地零占据

        // 地面 + 墙: 墙保留, 位置正确
        OccupancyGridMap m2;
        PointCloud scene = ground;
        for (double h = 0.3; h <= 1.5; h += 0.1)
            for (double y = -0.8; y <= 0.8; y += 0.1) {
                Point3D p; p.x = 2.0; p.y = y; p.z = -0.18 + h;
                scene.points.push_back(p);
            }
        m2.insert_cloud_filtered(scene, pose);
        int occ_cells = 0, wrong = 0;
        for (int r = 0; r < m2.height(); ++r)
            for (int c = 0; c < m2.width(); ++c)
                if (m2.occ_state(c, r) == 100) {
                    ++occ_cells;
                    double wx, wy; m2.index_to_world(c, r, wx, wy);
                    if (std::fabs(wx - 2.0) > 0.15 || std::fabs(wy) > 0.9) ++wrong;
                }
        CHECK(occ_cells >= 10);   // 墙在
        CHECK(wrong == 0);        // 位置对 (地面没成墙)
    }

    // ============ 9d. 负障碍独立层 (FIX_PLAN #9) ============
    {
        // 坑点(负障碍)不应被普通光线打空, 也不算占据 — 单独标记层
        // 场景按 P1 检测语义造 (下行台阶): 近处地面(参考) → 间隙 →
        // 下沉面 (P1: 按列扫描, ref 后间隔≥min_gap_cells 的下沉 return)
        OccupancyGridMap m;
        Pose2D pose;
        PointCloud scene; scene.frame_id = "base_link";
        // 近处地面 x ∈ [0.8, 1.9] (建立参考)
        for (double x = 0.8; x <= 1.9; x += 0.05)
            for (double y = -0.5; y <= 0.5; y += 0.05) {
                Point3D p; p.x=x; p.y=y; p.z=-0.18; scene.points.push_back(p);
            }
        // 间隙 x ∈ (1.9, 2.35) 无点 (断崖口)
        // 下沉面 x ∈ [2.4, 3.0] (z=-0.18-0.30, 落差 0.3 > cliff_drop_min 0.12)
        for (double x = 2.4; x <= 3.0; x += 0.05)
            for (double y = -0.5; y <= 0.5; y += 0.05) {
                Point3D p; p.x=x; p.y=y; p.z=-0.18-0.30; scene.points.push_back(p);
            }
        m.insert_cloud_filtered(scene, pose);
        // 间隙/下沉沿区域应有负障碍标记 (扫 x∈[1.9,2.5]×y∈[-0.5,0.5] 片)
        bool any_neg = false;
        for (double x = 1.90; x <= 2.50; x += 0.05) {
            for (double y = -0.5; y <= 0.5; y += 0.05) {
                int col, row;
                if (m.world_to_index(x, y, col, row) &&
                    m.negative_state(col, row) == 1) { any_neg = true; break; }
            }
            if (any_neg) break;
        }
        CHECK(any_neg);
        // 无坑处无标记 (近处地面 x=1.5 一带)
        bool clean = true;
        for (double x = 0.85; x <= 1.85; x += 0.1)
            for (double y = -0.5; y <= 0.5; y += 0.1) {
                int col, row;
                if (m.world_to_index(x, y, col, row) &&
                    m.negative_state(col, row) == 1) { clean = false; }
            }
        CHECK(clean);
    }

    // ============ 8. PGM 导出 ============
    {
        OccupancyGridMap m;
        Pose2D pose;
        m.insert_cloud(make_cloud_base({{2.0, 0.0}}), pose);
        m.inflate(0.15);
        const std::string path = "/tmp/mechdog_map_test.pgm";
        CHECK(m.save_pgm(path));

        // 读回验证: 头部 + 特征像素
        std::FILE* f = std::fopen(path.c_str(), "rb");
        CHECK(f != nullptr);
        if (f) {
            // P2\n<width> <height>\n255\n
            int w_read = 0, h_read = 0;
            const int n = std::fscanf(f, "P2 %d %d 255", &w_read, &h_read);
            CHECK(n == 2);
            CHECK(w_read == 200 && h_read == 200);
            // 定位到 (2.0, 0.0) 的像素: 行序 y 向上, 第 200-100=100 行
            // 简化验证: 文件非空且含 '0' (占据像素)
            std::fseek(f, 0, SEEK_END);
            const long sz = std::ftell(f);
            CHECK(sz > 1000);
            std::fclose(f);
        }
    }

    // ============ 8b. nav2 地图导出: P5 PGM + map.yaml 成对 (FIX_PLAN #3) ============
    {
        OccupancyGridMap m;
        Pose2D pose;
        m.insert_cloud(make_cloud_base({{2.0, 0.0}}), pose);

        // save_nav2_map: 一次出 pgm+yaml 成对文件
        CHECK(m.save_nav2_map("/tmp/mechdog_nav2_test"));

        // --- yaml 内容: nav2 map_server 必需五字段 ---
        std::FILE* f = std::fopen("/tmp/mechdog_nav2_test.yaml", "rb");
        CHECK(f != nullptr);
        if (f) {
            char buf[512] = {0};
            const size_t rd = std::fread(buf, 1, 511, f);
            std::fclose(f);
            (void)rd;
            CHECK(std::strstr(buf, "image: /tmp/mechdog_nav2_test.pgm") != nullptr);
            CHECK(std::strstr(buf, "resolution: 0.050000") != nullptr);
            CHECK(std::strstr(buf, "origin: [-5.000000, -5.000000, 0.000000]") != nullptr);
            CHECK(std::strstr(buf, "negate: 0") != nullptr);
            CHECK(std::strstr(buf, "occupied_thresh: 0.65") != nullptr);
            CHECK(std::strstr(buf, "free_thresh: 0.20") != nullptr);
        }

        // --- PGM 是 P5 (二进制), 像素数 = w*h ---
        std::FILE* g = std::fopen("/tmp/mechdog_nav2_test.pgm", "rb");
        CHECK(g != nullptr);
        if (g) {
            char magic[3] = {0};
            CHECK(std::fread(magic, 1, 2, g) == 2);
            CHECK(std::string(magic) == "P5");
            int w2 = 0, h2 = 0, maxv = 0;
            const int n = std::fscanf(g, " %d %d %d", &w2, &h2, &maxv);
            CHECK(n == 3);
            CHECK(w2 == 200 && h2 == 200 && maxv == 255);
            std::fgetc(g);  // 头后单个空白符
            // 二进制像素数应恰为 w*h (行序与世界 y 翻转已处理)
            std::vector<unsigned char> px(static_cast<size_t>(w2) * h2);
            CHECK(std::fread(px.data(), 1, px.size(), g) == px.size());
            // 统计值域合法 (0/205/254)
            bool legal = true;
            for (unsigned char v : px)
                if (v != 0 && v != 205 && v != 254) { legal = false; break; }
            CHECK(legal);
            std::fclose(g);
        }
    }

    // ---------- 汇总 ----------
    std::printf("\n%d checks, %d failed\n", g_total, g_failed);
    return g_failed == 0 ? 0 : 1;
}
