/**
 * 点云模块实现 (P0)
 *
 * 设计文档: docs/POINT_CLOUD_DESIGN.md (§5 反投影 / §6 坐标变换)
 *
 * 零依赖: 仅 <cmath> + <cassert>, 不引 PCL/Eigen。
 * 可用 g++ -std=c++20 -fsanitize=address,undefined 模拟单测。
 */
#include "point_cloud.h"

#include <cassert>
#include <cmath>
#include <unordered_set>

namespace mechdog {

// ============================================================
// 深度图(mm) → 点云(camera_optical 系, 米)
//
// 反投影公式 (§5.1, 基于 optical 系 Z前 Y下):
//   X = (u - cx) * d / fx     // 右
//   Y = (v - cy) * d / fy     // 下
//   Z = d                     // 前
// 其中 d = px / 1000.0 (mm → m)。
//
// 有效性口径 (§5.2): px==0 或 d<min_depth_m 或 d>max_depth_m → 丢弃
// 与 analyze_region 完全一致, 保证点云与三区域感知对"有效"的定义相同。
// ============================================================
void depth_to_cloud(const uint16_t* depth, int w, int h,
                    const CameraIntrinsics& K, PointCloud& out) {
    out.points.clear();
    out.frame_id = "camera_optical";

    // 空输入防御 (失效哨兵: 返回空点云, 不崩溃)
    if (!depth || w <= 0 || h <= 0) return;

    // fx/fy 非零防御 (除零保护; 正常 CameraIntrinsics 默认值保证非零)
    assert(K.fx > 0.0 && K.fy > 0.0 && "CameraIntrinsics fx/fy must be positive");

    // 粗略预留 (假设 ~25% 有效像素, 避免反复 realloc)
    out.points.reserve(static_cast<size_t>(w) * h / 4);

    for (int v = 0; v < h; ++v) {
        for (int u = 0; u < w; ++u) {
            uint16_t px = depth[static_cast<size_t>(v) * w + u];
            if (px == 0) continue;  // 无效像素 (与 analyze_region 同口径)

            double d = px / 1000.0;  // mm → m (§5.2 显式单位转换)
            if (d < K.min_depth_m || d > K.max_depth_m) continue;  // 超范围

            Point3D p;
            p.x = (u - K.cx) * d / K.fx;  // 右
            p.y = (v - K.cy) * d / K.fy;  // 下
            p.z = d;                      // 前
            out.points.push_back(p);
        }
    }
}

// ============================================================
// camera_optical → camera_link 固定旋转 (§6.1)
//
// 矩阵 [[0,0,1],[-1,0,0],[0,-1,0]] = Rz(-90°)·Rx(-90°):
//   X_link =  Z_opt  (前 ← 前)
//   Y_link = -X_opt  (左 ← 右反)
//   Z_link = -Y_opt  (上 ← 下反)
//
// ⚠️ 不是单次 roll — 单 roll 只能得到 {X右,Y前,Z上}, Y/Z 仍错。
// 此旋转是相机坐标系固定属性, 与安装无关, 可直接硬编码。
// ============================================================
void transform_optical_to_link(const PointCloud& in, PointCloud& out) {
    out.seq = in.seq;
    out.stamp = in.stamp;
    out.frame_id = "camera_link";
    out.points.resize(in.points.size());

    for (size_t i = 0; i < in.points.size(); ++i) {
        const auto& s = in.points[i];
        auto& d = out.points[i];
        d.x =  s.z;   // X_link =  Z_opt
        d.y = -s.x;   // Y_link = -X_opt
        d.z = -s.y;   // Z_link = -Y_opt
        d.r = s.r; d.g = s.g; d.b = s.b;  // 保色
    }
}

// ============================================================
// camera_optical → base_link 全链路 (§6.1 + §6.2)
//
// Step 1: optical → link (固定旋转, 调用 transform_optical_to_link)
// Step 2: link → base (CameraExtrinsics 的 ZYX 内旋 + 平移)
//
// ZYX 内旋: R = Rz(yaw) · Ry(pitch) · Rx(roll)
//   Rx(roll)  = [[1,0,0], [0,cr,-sr], [0,sr,cr]]
//   Ry(pitch) = [[cp,0,sp], [0,1,0], [-sp,0,cp]]
//   Rz(yaw)   = [[cy,-sy,0], [sy,cy,0], [0,0,1]]
//
// 展开后:
//   R00 = cy*cp        R01 = cy*sp*sr - sy*cr   R02 = cy*sp*cr + sy*sr
//   R10 = sy*cp        R11 = sy*sp*sr + cy*cr   R12 = sy*sp*cr - cy*sr
//   R20 = -sp          R21 = cp*sr              R22 = cp*cr
//
// p_base = R · p_link + t
//
// pitch 方向验证: 前向轴 [1,0,0] → z 分量 = R20 = -sin(pitch)。
// pitch > 0 → z < 0 (光轴朝下), 故前倾俯视取正值 (+15°)。见 §4。
// ============================================================
void transform_to_base(const PointCloud& in, const CameraExtrinsics& E,
                       PointCloud& out) {
    // Step 1: optical → link
    PointCloud link;
    transform_optical_to_link(in, link);

    // Step 2: link → base (ZYX Euler + translation)
    const double cr = std::cos(E.roll),  sr = std::sin(E.roll);
    const double cp = std::cos(E.pitch), sp = std::sin(E.pitch);
    const double cy = std::cos(E.yaw),   sy = std::sin(E.yaw);

    // ZYX 内旋展开 (见上方注释)
    const double R00 = cy * cp;
    const double R01 = cy * sp * sr - sy * cr;
    const double R02 = cy * sp * cr + sy * sr;
    const double R10 = sy * cp;
    const double R11 = sy * sp * sr + cy * cr;
    const double R12 = sy * sp * cr - cy * sr;
    const double R20 = -sp;
    const double R21 = cp * sr;
    const double R22 = cp * cr;

    out.seq = link.seq;
    out.stamp = link.stamp;
    out.frame_id = "base_link";
    out.points.resize(link.points.size());

    for (size_t i = 0; i < link.points.size(); ++i) {
        const auto& s = link.points[i];
        auto& d = out.points[i];
        d.x = R00 * s.x + R01 * s.y + R02 * s.z + E.x;
        d.y = R10 * s.x + R11 * s.y + R12 * s.z + E.y;
        d.z = R20 * s.x + R21 * s.y + R22 * s.z + E.z;
        d.r = s.r; d.g = s.g; d.b = s.b;  // 保色
    }
}

// 全帧有效深度像素计数 (600~8000mm 口径与 depth_to_cloud 一致; 可视化空态诊断数据源)
size_t count_valid_pixels(const std::vector<uint16_t>& depth_map) {
    const uint16_t lo = static_cast<uint16_t>(AstraProConfig::min_valid_mm);
    const uint16_t hi = static_cast<uint16_t>(AstraProConfig::max_valid_mm);
    size_t n = 0;
    for (uint16_t v : depth_map) {
        if (v >= lo && v <= hi) ++n;
    }
    return n;
}

// 点云空态标签 (可视化状态栏): 0=无帧 1=深度全无有效像素 2=正常
const char* cloud_state_label(int state) {
    switch (state) {
        case 0: return "[无帧 取帧失败]";
        case 1: return "[深度无有效像素]";   // 近界<0.6m 或暗区/窗帘/门洞无回波
        default: return "[正常]";
    }
}

// 屏幕空间抽稀: 按俯视投影 (y,x)→2px 像素格去重, 格内保留首个点 (可视化专用, 纯显示层)
// px_per_m = 视图每米像素; 与 draw_cloud_view 的 sx/sy 映射 (sx∝y, sy∝x) 同口径
void screen_decimate(const PointCloud& in, double px_per_m, PointCloud& out) {
    out.seq = in.seq;
    out.stamp = in.stamp;
    out.frame_id = in.frame_id;
    out.points.clear();
    if (px_per_m <= 0.0) return;   // 无视图缩放 → 保空 (防御)
    constexpr int cell = 2;        // 2px 网格: 视觉等价, 38400 点 → 约 5000-9000 点
    std::unordered_set<int64_t> used;
    for (const auto& p : in.points) {
        // 与绘制裁剪同口径, 先粗滤 (视口外/近界不参与去重占位)
        if (p.x <= 0.05 || p.x > 8.0) continue;
        if (std::abs(p.y) > 8.0) continue;
        const int gx = (int)std::floor(p.y * px_per_m / cell);
        const int gy = (int)std::floor(p.x * px_per_m / cell);
        const int64_t key = ((int64_t)gx << 32) ^ (gy & 0xFFFFFFFFll);
        if (used.insert(key).second) out.points.push_back(p);
    }
}

} // namespace mechdog
