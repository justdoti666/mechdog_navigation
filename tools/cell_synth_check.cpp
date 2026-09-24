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
#include <random>
#include "point_cloud.h"
#include "ground_segmentation.h"

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

int main() {
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
