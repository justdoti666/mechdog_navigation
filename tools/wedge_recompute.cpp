// wedge_recompute.cpp —— 离线重算: 真机帧 → 节点同款管线 → 「感知」字段逐字段可比
//
// 用途 (OFFLINE_TODO #2 / PLAN_2026-09-21 C6):
//   把抓到的真机深度帧 (real_depth.raw / depth_*.npy) 离线喂进与 safety_node 完全同源的
//   核心管线 (depth_to_cloud_strided → transform_to_base → segment_ground → build_heightmap_25,
//   全部直接复用 core 源码, 不含任何重写), 输出与节点日志「感知:」行同格式的战报, 供 diff:
//     验收口径 = 与节点日志「同帧逐字段一致」(点云数 / 平面 tilt·n·h0·内点·fit / 2.5D 统计 / 楔形 cov)。
//   交付物 = 楔形 cov + 可通行基线 数字表; 装机后复测 = 同一工具、同一口径, 新旧帧对比。
//
// 构建 (WSL / Linux; 需 core 仓根目录为 -I):
//   g++ -std=c++17 -O2 -I <core-repo> tools/wedge_recompute.cpp \
//       <core-repo>/point_cloud.cpp <core-repo>/ground_segmentation.cpp \
//       <core-repo>/heightmap_2d5.cpp -o wedge_recompute
//
// 用法:
//   ./wedge_recompute <frame.raw|depth_000.npy> --pitch-deg <度> [--cam-h 0.75] [--cloud-z 0.0]
//        [--cloud-x 0.12] [--prior-window 0.10] [--step 8] [--wedge-only 0|1] [--prior-z <直接指定>]
//   # 台架回放口径 (对齐宿舍回放矩阵): --cam-h 0.75 --cloud-z 0.0 --pitch-deg 15 --step 8
//
// 与节点参数的对齐依据 (2026-10-09 核, 台账见 楔形离线重算_数字表_2026-10-09.md):
//   step=8 (cloud_downsample_step 默认);  K=[570.3422047415297129,319.5,239.5] (pi_replay 同款),
//   min/max=0.6/8.0 (AstraProConfig);  地面先验 = -(cam_h - cloud_z) (sensor_geometry.hpp),
//   gseg = GroundSegParams 默认 + use_cell_min_fit=true + cell_skip_ransac=true (节点 ground_fit_method=cell);
//   wedge_only=true (节点 grid_wedge_only 默认 true)。
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <limits>

#include "point_cloud.h"
#include "ground_segmentation.h"
#include "heightmap_2d5.h"

using namespace mechdog;

static const double kPi = 3.14159265358979323846;

// ---------- 帧加载: .raw (640x480 uint16 LE) 或 .npy (uint16, C 顺序) ----------
static std::vector<uint16_t> load_frame(const std::string& path, int& w, int& h) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("无法打开: " + path);
    std::vector<char> head(8);
    f.read(head.data(), 8);
    if (f.gcount() >= 6 && (unsigned char)head[0] == 0x93 &&
        std::strncmp(head.data() + 1, "NUMPY", 5) == 0) {
        // ---- npy: 解析 header ----
        f.seekg(0);
        std::vector<char> magic(8);
        f.read(magic.data(), 8);
        const int major = (unsigned char)magic[6];
        uint32_t hlen = 0;
        if (major == 1) {
            unsigned char l[2];
            f.read(reinterpret_cast<char*>(l), 2);
            hlen = l[0] | (l[1] << 8);
        } else {
            unsigned char l[4];
            f.read(reinterpret_cast<char*>(l), 4);
            hlen = l[0] | (l[1] << 8) | (l[2] << 16) | ((uint32_t)l[3] << 24);
        }
        std::string hdr(hlen, '\0');
        f.read(&hdr[0], hlen);
        // shape
        const size_t sp = hdr.find("'shape': (");
        if (sp == std::string::npos) throw std::runtime_error("npy: 缺 shape");
        int a = 0, b = 0;
        if (std::sscanf(hdr.c_str() + sp + 10, "%d, %d", &a, &b) != 2)
            throw std::runtime_error("npy: shape 解析失败");
        if (hdr.find(">u2") != std::string::npos || hdr.find("|u2") != std::string::npos)
            throw std::runtime_error("npy: 仅支持 <u2 (小端 uint16)");
        if (hdr.find("'fortran_order': True") != std::string::npos)
            throw std::runtime_error("npy: 不支持 fortran_order");
        h = a; w = b;
        std::vector<uint16_t> data(static_cast<size_t>(w) * h);
        f.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(data.size()) * 2);
        if (f.gcount() != static_cast<std::streamsize>(data.size()) * 2)
            throw std::runtime_error("npy: 数据段长度不符");
        return data;
    }
    // ---- raw 640x480 ----
    w = 640; h = 480;
    f.seekg(0, std::ios::end);
    const std::streamoff sz = f.tellg();
    if (sz != static_cast<std::streamoff>(w) * h * 2)
        throw std::runtime_error("raw: 尺寸非 640x480x2 (=614400B), 实际 " + std::to_string(sz));
    f.seekg(0);
    std::vector<uint16_t> data(static_cast<size_t>(w) * h);
    f.read(reinterpret_cast<char*>(data.data()), sz);
    return data;
}

int main(int argc, char** argv) {
    if (argc < 2) { std::fprintf(stderr, "用法: %s <frame.raw|.npy> --pitch-deg <度> [--cam-h 0.75] [--cloud-z 0.0] [--cloud-x 0.12] [--prior-window 0.10] [--step 8] [--wedge-only 1] [--prior-z <覆盖>]\n", argv[0]); return 2; }
    const std::string frame_path = argv[1];
    double pitch_deg = 0.0; bool have_pitch = false;
    double pitch_rad_lit = std::numeric_limits<double>::quiet_NaN();   // --pitch-rad: 直传字面量 (与节点 -p 值逐位一致)
    double cam_h = 0.75, cloud_z = 0.0, cloud_x = 0.12, prior_window = 0.10;
    double prior_z_override = -999.0;
    int step = 8, wedge_only = 1;
    for (int i = 2; i + 1 < argc; i += 2) {
        const std::string k = argv[i], v = argv[i + 1];
        if (k == "--pitch-deg") { pitch_deg = std::atof(v.c_str()); have_pitch = true; }
        else if (k == "--pitch-rad") { pitch_rad_lit = std::atof(v.c_str()); have_pitch = true; }
        else if (k == "--cam-h") cam_h = std::atof(v.c_str());
        else if (k == "--cloud-z") cloud_z = std::atof(v.c_str());
        else if (k == "--cloud-x") cloud_x = std::atof(v.c_str());
        else if (k == "--prior-window") prior_window = std::atof(v.c_str());
        else if (k == "--step") step = std::atoi(v.c_str());
        else if (k == "--wedge-only") wedge_only = std::atoi(v.c_str());
        else if (k == "--prior-z") prior_z_override = std::atof(v.c_str());
        else { std::fprintf(stderr, "未知参数: %s\n", k.c_str()); return 2; }
    }
    if (!have_pitch) { std::fprintf(stderr, "必须显式给 --pitch-deg (不回退默认, 防口径混淆)\n"); return 2; }

    try {
        int w = 0, h = 0;
        const std::vector<uint16_t> depth = load_frame(frame_path, w, h);

        // ---- 与节点同源: K (pi_replay 同款) / 外参 / 先验 ----
        CameraIntrinsics K;
        K.fx = 570.3422047415297129; K.fy = 570.3422047415297129;
        K.cx = 319.5; K.cy = 239.5;
        K.min_depth_m = AstraProConfig::min_valid_mm / 1000.0;   // 0.6
        K.max_depth_m = AstraProConfig::max_valid_mm / 1000.0;   // 8.0

        CameraExtrinsics E;    // 节点 cloud_E_ 同款: x=cloud_x, y=0, z=cloud_z, roll=yaw=0
        E.x = cloud_x; E.y = 0.0; E.z = cloud_z;
        E.roll = 0.0;
        E.pitch = std::isnan(pitch_rad_lit) ? (pitch_deg * kPi / 180.0) : pitch_rad_lit;
        E.yaw = 0.0;

        GroundSegParams g;
        g.ground_prior_z = (prior_z_override > -900.0)
            ? prior_z_override : -(cam_h - cloud_z);   // sensor_geometry.hpp 同式
        g.prior_window       = prior_window;
        g.use_cell_min_fit   = true;    // 节点 ground_fit_method=cell
        g.cell_skip_ransac   = true;    // 节点默认 true

        PointCloud cloud_opt, cloud_base;
        depth_to_cloud_strided(depth.data(), w, h, K, step, cloud_opt);
        transform_to_base(cloud_opt, E, cloud_base);
        GroundSegResult seg;
        segment_ground(cloud_base, g, seg);

        HeightMap25Config hcfg;
        hcfg.wedge_only = (wedge_only != 0);
        HeightMap25Result hm;
        build_heightmap_25(cloud_base, seg, hcfg, hm);

        // ---- 与节点 safety_node.cpp 同一格式输出 (逐字段可比) ----
        std::printf("感知: 点云=%zu 平面valid=%d tilt=%.2f° [n=(%.3f,%.3f,%.3f) 分量 %.2f°/%.2f°] h0=%.3fm 内点=%zu neg=%zu fit=%s | 2.5D %s\n",
            cloud_base.points.size(), static_cast<int>(seg.plane.valid),
            std::acos(std::min(1.0, std::max(-1.0, static_cast<double>(seg.plane.nz)))) * 180.0 / kPi,
            static_cast<double>(seg.plane.nx), static_cast<double>(seg.plane.ny),
            static_cast<double>(seg.plane.nz),
            std::asin(std::min(1.0, std::max(-1.0, static_cast<double>(seg.plane.nx)))) * 180.0 / kPi,
            std::asin(std::min(1.0, std::max(-1.0, static_cast<double>(seg.plane.ny)))) * 180.0 / kPi,
            static_cast<double>(seg.plane.height_at_origin()),
            static_cast<size_t>(seg.plane.inliers), seg.negative_points.size(),
            (seg.used_ransac ? "ransac" : (seg.used_cell ? "cell" : "none")),
            hm.valid ? hm.stats().c_str() : "invalid (无平面→fail-closed, 路1 静默)");
        // 诊断附行 (不参与逐字段 diff)
        std::printf("// 回算: pitch=%.6f° (%.10f rad) cam_h=%.2f cloud_z=%.2f prior_z=%.3f step=%d wedge_only=%d | used_cell=%d used_ransac=%d ransac_iters=%d\n",
            static_cast<double>(E.pitch) * 180.0 / kPi, static_cast<double>(E.pitch), cam_h, cloud_z, g.ground_prior_z, step, wedge_only,
            static_cast<int>(seg.used_cell), static_cast<int>(seg.used_ransac), seg.ransac_iters);
        (void)w; (void)h;
        return (seg.plane.valid || cloud_base.points.empty()) ? 0 : 0;   // 信息系工具: 恒 0 (fail 与否由 diff 判)
    } catch (const std::exception& e) {
        std::fprintf(stderr, "错误: %s\n", e.what());
        return 1;
    }
}
