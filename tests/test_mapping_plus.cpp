/**
 * mapping_plus 验收测试 (最小) — 验证 FIX_PLAN #1/#3 交付物
 *
 * 验收标准 (FIX_PLAN 原文):
 *   #1 "平地帧建图后零占据格(无假墙); 墙上放障碍才有占据"
 *   #3 "nav2 map_server 需要的 image/resolution/origin/negate/thresholds yaml"
 */
#include "mapping_plus.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

static int failed = 0, total = 0;
#define CHECK(c) do { ++total; if (!(c)) { ++failed; \
    std::printf("  FAIL line %d: %s\n", __LINE__, #c); } } while (0)

using namespace mechdog;

// 合成一帧 base 系纯地面点 (前方 1~3m 均匀地面)
// 地面高度: base_link 离地 0.18m → 地面点 z = -0.18 (与 GroundSegConfig::ground_prior_z 一致)
static PointCloud make_ground() {
    PointCloud c; c.frame_id = "base_link";
    for (double x = 1.0; x <= 3.0; x += 0.1)
        for (double y = -1.0; y <= 1.0; y += 0.1) {
            Point3D p; p.x = x; p.y = y; p.z = -0.18;  // 地面 (base 系)
            c.points.push_back(p);
        }
    return c;
}

// 合成: 地面 + 前方 2m 一面离地 0.3~1.5m 的墙 (墙点 z = 地面+0.3 起算)
static PointCloud make_ground_and_wall() {
    PointCloud c = make_ground();
    for (double h = 0.3; h <= 1.5; h += 0.1)   // h = 离地高度
        for (double y = -0.8; y <= 0.8; y += 0.1) {
            Point3D p; p.x = 2.0; p.y = y; p.z = -0.18 + h;
            c.points.push_back(p);
        }
    return c;
}

int main() {
    std::printf("== mapping_plus acceptance (FIX_PLAN #1/#3) ==\n");

    // --- #1: 平地帧 → 零占据 ---
    {
        MappingPipeline pipe;
        MappingPose pose;  // 原点静止
        pipe.insert(make_ground(), pose);
        const int occ = pipe.map.stats().find("occ=") != std::string::npos
            ? std::atoi(pipe.map.stats().c_str() + pipe.map.stats().find("occ=") + 4) : -1;
        std::printf("纯地面帧: %s\n", pipe.map.stats().c_str());
        CHECK(occ == 0);   // 验收: 平地零占据
    }

    // --- #1b: 地面+墙 → 有占据且只在墙处 ---
    {
        MappingPipeline pipe;
        MappingPose pose;
        pipe.insert(make_ground_and_wall(), pose);
        int occ_cells = 0, wrong_place = 0;
        for (int r = 0; r < pipe.map.height(); ++r)
            for (int c = 0; c < pipe.map.width(); ++c)
                if (pipe.map.is_occupied(c, r)) {
                    ++occ_cells;
                    double wx, wy; pipe.map.index_to_world(c, r, wx, wy);
                    // 墙在 x≈2.0 ±0.15, |y|<=0.85
                    if (std::fabs(wx - 2.0) > 0.15 || std::fabs(wy) > 0.9) ++wrong_place;
                }
        std::printf("地面+墙帧: occ=%d (墙外错位 %d)\n", occ_cells, wrong_place);
        CHECK(occ_cells > 10);      // 有墙必有占据
        CHECK(wrong_place == 0);    // 全部落在墙体位置
    }

    // --- #3: yaml+pgm 成对导出且内容正确 ---
    {
        MappingPipeline pipe;
        MappingPose pose;
        pipe.insert(make_ground_and_wall(), pose);
        CHECK(pipe.save("/tmp/plus_accept"));
        // 读 yaml 关键字段
        std::FILE* f = std::fopen("/tmp/plus_accept.yaml", "rb");
        CHECK(f != nullptr);
        if (f) {
            char buf[512] = {0};
            std::fread(buf, 1, 511, f); std::fclose(f);
            CHECK(std::strstr(buf, "image: /tmp/plus_accept.pgm") != nullptr);
            CHECK(std::strstr(buf, "resolution: 0.05") != nullptr);
            CHECK(std::strstr(buf, "origin: [-5.0") != nullptr);
            CHECK(std::strstr(buf, "occupied_thresh: 0.65") != nullptr);
            CHECK(std::strstr(buf, "free_thresh: 0.20") != nullptr);
        }
        // PGM 是 P5 二进制
        std::FILE* g = std::fopen("/tmp/plus_accept.pgm", "rb");
        CHECK(g != nullptr);
        if (g) {
            char magic[3] = {0};
            std::fread(magic, 1, 2, g); std::fclose(g);
            CHECK(std::string(magic) == "P5");
        }
    }

    std::printf("\n%d checks, %d failed\n", total, failed);
    return failed ? 1 : 0;
}
