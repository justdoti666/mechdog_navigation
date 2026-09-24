// cell_synth_check.cpp —— 离线"自造标注帧"实验（不需要 Pi / 相机）
//   目的（承接 2026-09-23 真机发现）：
//     ① 判定"确定性 cell 路径"在**地板+先验带正确**时到底命中不命中（真机日志待测，此处离线先答）；
//     ② **直接检验**"偏航不影响 cell"这条（我今天两次改口的那个问题）—— 扫 yaw ±30°；
//     ③ 顺带得到"装机几何(0.18m/15°)"与"台架几何"下的真值对照（供标定/单测复用）。
//   真值：帧由**已知位姿/镜头高**渲染（射线-平面求交），噪声 + 丢点可调。
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
#include <array>
#include <algorithm>
#include <random>
#include "point_cloud.h"
#include "ground_segmentation.h"
#include "heightmap_2d5.h"

using namespace mechdog;

static const double D2R = 0.01745329251994329576;

// ---------- 渲染: 地平面 z_base = -H 的深度帧(16UC1 mm) ----------
static std::vector<uint16_t> render_floor(int W, int H_px, double fx, double fy, double cx, double cy,
                                          double cam_h, double pitch_deg, double roll_deg, double yaw_deg,
                                          double noise_mm, double dropout, unsigned seed) {
    std::vector<uint16_t> depth(static_cast<size_t>(W) * H_px, 0);
    // 相机外参: p_base = Rz(yaw)·Ry(pitch)·Rx(roll)·p_opt + t   (t = 相机在 base 的位置)
    // 与 point_cloud.cpp 的 transform_to_base 同约定(ZYX)。相机原点设为 (0,0,0)->t=(0,0,0),
    // 高度用"地面在 -cam_h"表达, 与节点里 ground_prior_z=-cam_h 同口径。
    const double r = roll_deg * D2R, pch = pitch_deg * D2R, y = yaw_deg * D2R;
    const double cr = std::cos(r), sr = std::sin(r);
    const double cp = std::cos(pch), sp = std::sin(pch);
    const double cyaw = std::cos(y), syaw = std::sin(y);
    auto rot = [&](double x, double yy, double z, double& ox, double& oy, double& oz) {
        // ★ 先做 optical→link 固定旋转 (与 point_cloud.cpp 同约定): X_link=Z_opt, Y_link=-X_opt, Z_link=-Y_opt
        { double xo = z, yo = -x, zo = -yy; x = xo; yy = yo; z = zo; }
        // 再做外参 Rz(yaw)·Ry(pitch)·Rx(roll)
        // Rx
        double x1 = x, y1 = cr * yy - sr * z, z1 = sr * yy + cr * z;
        // Ry
        double x2 = cp * x1 + sp * z1, y2 = y1, z2 = -sp * x1 + cp * z1;
        // Rz
        ox = cyaw * x2 - syaw * y2; oy = syaw * x2 + cyaw * y2; oz = z2;
    };
    std::mt19937 rng(seed);
    std::normal_distribution<double> nz(0.0, noise_mm);
    std::uniform_real_distribution<double> un(0.0, 1.0);
    for (int v = 0; v < H_px; ++v) {
        for (int u = 0; u < W; ++u) {
            const double dx = (u - cx) / fx, dy = (v - cy) / fy, dz = 1.0;   // 光学: x右 y下 z前
            double ox, oy, oz;
            rot(dx, dy, dz, ox, oy, oz);
            if (oz >= -1e-6) continue;                       // 射线不朝下 ⇒ 看不到地面
            const double s = (-cam_h - 0.0) / oz;            // 相机在 z=0 ⇒ 求交 z=-cam_h
            if (!(s > 0.0) || s > 8.0) continue;             // 量程
            if (un(rng) < dropout) continue;                 // 丢点(反光/无回波)
            double mm = s * 1000.0 + nz(rng);
            if (mm < 300.0 || mm > 8000.0) continue;
            depth[static_cast<size_t>(v) * W + u] = static_cast<uint16_t>(mm);
        }
    }
    return depth;
}

static void run_case(const char* name, double cam_h, double pitch, double roll, double yaw,
                     bool installed_spec) {
    const int W = 640, HP = 480;
    const double fx = 570.3422047415297129, fy = 570.3422047415297129, ccx = 319.5, ccy = 239.5;
    auto depth = render_floor(W, HP, fx, fy, ccx, ccy, cam_h, pitch, roll, yaw, 8.0, 0.10, 12345);

    CameraIntrinsics K; K.fx = fx; K.fy = fy; K.cx = ccx; K.cy = ccy;
    K.min_depth_m = 0.3; K.max_depth_m = 8.0;
    PointCloud opt, base;
    depth_to_cloud_strided(depth.data(), W, HP, K, 8, opt);
    CameraExtrinsics E;                       // 与节点同源: 相机在机体前上方
    E.x = 0.12; E.y = 0.0; E.z = 0.0;
    E.roll = roll * D2R; E.pitch = pitch * D2R; E.yaw = yaw * D2R;
    transform_to_base(opt, E, base);

    GroundSegParams g;                        // 与节点一致
    g.ground_prior_z = -cam_h;
    g.prior_window = 0.10;
    g.plane_max_tilt_deg = 15.0;
    g.point_on_plane_eps = 0.02;
    g.cell_size = 0.05;
    g.use_cell_min_fit = true;

    GroundPlane plane;
    const bool hit = fit_ground_plane_cells(base, g, plane);          // ★ 只看 cell 路径
    GroundSegResult seg;
    segment_ground(base, g, seg);                                     // 整套(含 RANSAC)
    printf("%-30s pts=%6zu | cell命中=%-3s inl=%5d tilt=%5.2f° h0=%+.3f | 整套: plane=%d tilt=%5.2f° h0=%+.3f\n",
           name, base.points.size(), hit ? "是" : "否", plane.inliers,
           hit ? std::acos(std::fabs(plane.nz)) / D2R : 0.0, hit ? -plane.d / plane.nz : 0.0,
           seg.plane.valid ? 1 : 0,
           seg.plane.valid ? std::acos(std::fabs(seg.plane.nz)) / D2R : 0.0,
           seg.plane.valid ? -seg.plane.d / seg.plane.nz : 0.0);
}


// ============================================================
// 追加 (OFFLINE_TODO #3 / 选项 C): 坡面与倾斜墙 —— 量化"若放宽 plane_max_tilt_deg 会发生什么"
//   场景A 坡面: 通过 (0,0,-H)、法向绕 y 倾斜 θ 的平面 (θ=15/20/25°)  ⇒ 该不该算"可走的地面"?
//   场景B 地板+斜墙: 水平地板 + 前方 1.5m 处一面从竖直倾斜 φ 的墙 (φ=0/15/25°) ⇒ 会不会被当地面?
//   对每个场景扫 plane_max_tilt_deg ∈ {15,20,25} (上游容限), 输出平面是否有效 / tilt / h0 / 2.5D 统计。
//   ⚠ 本工装**只观测、不改行为**; 是否放宽口径属安全/需求决策 (师兄)。
// ============================================================
static std::vector<uint16_t> render_planes(int W, int Hpx, double fx, double fy, double cx, double cy,
                                           const std::vector<std::array<double, 4>>& planes,  // {a,b,c,d}
                                           double pitch_deg, double roll_deg, double yaw_deg,
                                           double noise_mm, double dropout, unsigned seed) {
    std::vector<uint16_t> depth(static_cast<size_t>(W) * Hpx, 0);
    const double r = roll_deg * D2R, pch = pitch_deg * D2R, y = yaw_deg * D2R;
    const double cr = std::cos(r), sr = std::sin(r);
    const double cp = std::cos(pch), sp = std::sin(pch);
    const double cyaw = std::cos(y), syaw = std::sin(y);
    std::mt19937 rng(seed);
    std::normal_distribution<double> nz(0.0, noise_mm);
    std::uniform_real_distribution<double> un(0.0, 1.0);
    for (int v = 0; v < Hpx; ++v) {
        for (int u = 0; u < W; ++u) {
            double dx = (u - cx) / fx, dy = (v - cy) / fy, dz = 1.0;    // 光学系
            { double xo = dz, yo = -dx, zo = -dy; dx = xo; dy = yo; dz = zo; }   // optical→link
            double x1 = dx, y1 = cr * dy - sr * dz, z1 = sr * dy + cr * dz;      // Rx
            double x2 = cp * x1 + sp * z1, y2 = y1, z2 = -sp * x1 + cp * z1;     // Ry
            double bx = cyaw * x2 - syaw * y2, by = syaw * x2 + cyaw * y2, bz = z2;  // Rz
            double best = -1.0;
            for (const auto& pl : planes) {
                const double den = pl[0] * bx + pl[1] * by + pl[2] * bz;
                if (std::fabs(den) < 1e-9) continue;
                const double s = -pl[3] / den;                    // 相机在原点
                if (s > 0.0 && s < 8.0 && (best < 0.0 || s < best)) best = s;
            }
            if (best < 0.0 || un(rng) < dropout) continue;
            const double mm = best * 1000.0 + nz(rng);
            if (mm < 300.0 || mm > 8000.0) continue;
            depth[static_cast<size_t>(v) * W + u] = static_cast<uint16_t>(mm);
        }
    }
    return depth;
}

static void slope_case(const char* tag, const std::vector<std::array<double, 4>>& planes,
                       double cam_h, double max_tilt_deg) {
    const int W = 640, HP = 480;
    const double fx = 570.3422047415297129, fy = fx, ccx = 319.5, ccy = 239.5;
    // 相机装成俯角 15°(装机规格)
    auto depth = render_planes(W, HP, fx, fy, ccx, ccy, planes, 15.0, 0.0, 0.0, 8.0, 0.10, 777);
    CameraIntrinsics K; K.fx = fx; K.fy = fy; K.cx = ccx; K.cy = ccy;
    K.min_depth_m = 0.3; K.max_depth_m = 8.0;
    PointCloud opt, base;
    depth_to_cloud_strided(depth.data(), W, HP, K, 8, opt);
    CameraExtrinsics E; E.x = 0.12; E.y = 0.0; E.z = 0.0;
    E.roll = 0.0; E.pitch = 15.0 * D2R; E.yaw = 0.0;
    transform_to_base(opt, E, base);
    GroundSegParams g;
    g.ground_prior_z = -cam_h; g.prior_window = 0.10;
    g.plane_max_tilt_deg = max_tilt_deg;          // ★ 扫的量
    g.point_on_plane_eps = 0.02; g.cell_size = 0.05; g.use_cell_min_fit = true;
    GroundSegResult seg; segment_ground(base, g, seg);
    HeightMap25Config cfg; cfg.wedge_only = false;
    HeightMap25Result hm; build_heightmap_25(base, seg, cfg, hm);
    int tr = 0, up = 0, dn = 0, st = 0, unk = 0;
    for (CellFlag f : hm.flag) {
        if (f == CellFlag::Traversable) ++tr; else if (f == CellFlag::ObstacleUp) ++up;
        else if (f == CellFlag::CliffDown) ++dn; else if (f == CellFlag::TooSteep) ++st; else ++unk;
    }
    const double tilt = seg.plane.valid ? std::acos(std::fabs(seg.plane.nz)) / D2R : 0.0;
    printf("  %-26s max_tilt=%2.0f° | 平面=%-3s tilt=%5.2f° h0=%+.3f inl=%5d | trav=%5d up=%4d down=%3d steep=%4d\n",
           tag, max_tilt_deg, seg.plane.valid ? "有效" : "无效", tilt,
           seg.plane.valid ? -seg.plane.d / seg.plane.nz : 0.0, seg.plane.inliers, tr, up, dn, st);
}

static void run_slope_experiment() {
    const double H = 0.18;                                  // 装机镜头高
    printf("\n=== C: 坡面(该不该当地面?) —— 相机 0.18m/俯角15° ===\n");
    for (double th : {15.0, 20.0, 25.0}) {
        const double t = th * D2R;
        std::vector<std::array<double, 4>> pl = {{std::sin(t), 0.0, std::cos(t), H * std::cos(t)}};
        char buf[64]; snprintf(buf, sizeof(buf), "坡面 %.0f°", th);
        for (double mt : {15.0, 20.0, 25.0}) slope_case(buf, pl, H, mt);
    }
    printf("\n=== C: 水平地板 + 前方 1.5m 处斜墙(会不会被当地面?) ===\n");
    for (double ph : {0.0, 15.0, 25.0}) {
        const double p = ph * D2R;
        std::vector<std::array<double, 4>> pl = {
            {0.0, 0.0, 1.0, H},                              // 地板 z = -H
            {std::cos(p), 0.0, std::sin(p), -1.5 * std::cos(p)}   // 墙 x≈1.5m, 法向倾斜 φ
        };
        char buf[64]; snprintf(buf, sizeof(buf), "地板+墙倾斜%.0f°", ph);
        for (double mt : {15.0, 20.0, 25.0}) slope_case(buf, pl, H, mt);
    }
}


static GroundSegResult seg_dummy(const PointCloud& base, const GroundSegParams& g) {
    GroundSegResult s; segment_ground(base, g, s); return s;
}

// ============================================================
// 追加 (OFFLINE_TODO #5 证据): 视场楔形在"相机偏航"下的失配有多大?
//   现状: in_fov() 假设楔形**对称于 y=0** (|wy| <= 0.561*wx + 0.15), 与 yaw 无关;
//         config 注释承认"yaw 未标定时用 0.15m 余量避免切掉真实视野"。
//   实验: 合成地板 + 相机偏航 φ ∈ {0,5,10,20,30}°, 统计
//         ① 真实有数据的格 (known) ② 楔形声称在视场内的格 (in_fov)
//         ③ **有数据却在楔形外** (漏掉的真视野, 越多越糟) ④ 楔形内却没数据
// ============================================================
static void wedge_yaw_case(double yaw_deg) {
    const int W = 640, HP = 480;
    const double fx = 570.3422047415297129, fy = fx, ccx = 319.5, ccy = 239.5;
    const double H = 0.18, pitch = 15.0, roll = 0.0;
    // 地板 z = -H (相机在原点, 俯角 15°, 偏航 yaw_deg)
    std::vector<std::array<double, 4>> pl = {{0.0, 0.0, 1.0, H}};
    auto depth = render_planes(W, HP, fx, fy, ccx, ccy, pl, pitch, roll, yaw_deg, 8.0, 0.10, 4242);
    CameraIntrinsics K; K.fx = fx; K.fy = fy; K.cx = ccx; K.cy = ccy;
    K.min_depth_m = 0.3; K.max_depth_m = 8.0;
    PointCloud opt, base;
    depth_to_cloud_strided(depth.data(), W, HP, K, 8, opt);
    CameraExtrinsics E; E.x = 0.12; E.y = 0.0; E.z = 0.0;
    E.roll = roll * D2R; E.pitch = pitch * D2R; E.yaw = yaw_deg * D2R;
    transform_to_base(opt, E, base);
    GroundSegParams g; g.ground_prior_z = -H; g.prior_window = 0.10;
    g.plane_max_tilt_deg = 15.0; g.point_on_plane_eps = 0.02; g.cell_size = 0.05;
    g.use_cell_min_fit = true;
    HeightMap25Config cfg; cfg.wedge_only = false;      // 保留全体格以做对比
    HeightMap25Result hm; build_heightmap_25(base, seg_dummy(base, g), cfg, hm);
    int known = 0, infov = 0, known_outside = 0, infov_empty = 0;
    for (int r = 0; r < hm.rows; ++r)
        for (int c = 0; c < hm.cols; ++c) {
            const size_t i = static_cast<size_t>(r) * hm.cols + c;
            const bool is_known = (hm.flag[i] != CellFlag::Unknown);
            const bool is_fov = hm.in_fov(c, r);
            if (is_known) ++known;
            if (is_fov) ++infov;
            if (is_known && !is_fov) ++known_outside;   // ★ 真视野被楔形漏掉
            if (is_fov && !is_known) ++infov_empty;
        }
    printf("  yaw=%4.0f° | 有数据格 known=%5d | 楔形 in_fov=%5d | ★真视野被漏掉=%5d (%.1f%% of known) | 楔形内空=%5d\n",
           yaw_deg, known, infov, known_outside, known ? 100.0 * known_outside / known : 0.0, infov_empty);
}

static void run_wedge_yaw_experiment() {
    printf("\n=== D: 楔形 vs 相机偏航 (装机几何 0.18m/俯角15°) ===\n");
    for (double yy : {0.0, 5.0, 10.0, 20.0, 30.0}) wedge_yaw_case(yy);
}

int main() {
    run_wedge_yaw_experiment();
    run_slope_experiment();
    printf("=== 装机几何 (相机 0.18m / 俯角 15°) ===\n");
    run_case("装机/理想(yaw=0)",        0.18, 15.0, 0.0,   0.0, true);
    run_case("装机/yaw=+5°",            0.18, 15.0, 0.0,   5.0, true);
    run_case("装机/yaw=+15°",           0.18, 15.0, 0.0,  15.0, true);
    run_case("装机/yaw=-15°",           0.18, 15.0, 0.0, -15.0, true);
    run_case("装机/yaw=+30°",           0.18, 15.0, 0.0,  30.0, true);
    run_case("装机/roll=+3°",           0.18, 15.0, 3.0,   0.0, true);
    run_case("装机/俯角 5°(装歪)",       0.18,  5.0, 0.0,   0.0, true);
    printf("\n=== 台架几何 (当前实验室: 0.75m / 水平) ===\n");
    run_case("台架/0.75 水平(yaw=0)",    0.75,  0.0, 0.0,   0.0, false);
    run_case("台架/0.75 水平(yaw=+15°)", 0.75,  0.0, 0.0,  15.0, false);
    run_case("台架/0.39 水平",           0.39,  0.0, 0.0,   0.0, false);
    return 0;
}
