/**
 * 地面分割与负障碍检测实现 (P1)
 *
 * 零依赖: 仅 <cmath>/<vector>/<random>/<limits>, 不引 PCL (与点云模块同风格).
 */
#include "ground_segmentation.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace mechdog {

namespace {

constexpr double kDegToRad = 0.01745329251994329576;

/** 平面方程过三点; 返回 false = 三点近共线 (退化采样, RANSAC 中跳过) */
bool plane_from_three(const Point3D& a, const Point3D& b, const Point3D& c,
                      double& nx, double& ny, double& nz, double& d) {
    const double ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
    const double vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
    nx = uy * vz - uz * vy;
    ny = uz * vx - ux * vz;
    nz = ux * vy - uy * vx;
    const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len < 1e-6) return false;
    nx /= len; ny /= len; nz /= len;
    d = -(nx * a.x + ny * a.y + nz * a.z);
    return true;
}

// 2.5D 栅格单元: 记录落在格内的最低表面 (min_s = 最小带符号平面距离, 上正下负)
struct GridCell {
    bool   has_return = false;
    bool   has_ground = false;   // 含 |s| <= point_on_plane_eps 的点
    double min_s = std::numeric_limits<double>::infinity();
};

} // namespace

void segment_ground(const PointCloud& cloud, const GroundSegParams& p,
                    GroundSegResult& out) {
    out = GroundSegResult{};
    const int n = static_cast<int>(cloud.points.size());
    if (n < 3) return;
    const auto& pts = cloud.points;

    // ---- ① 受约束 RANSAC 拟合地面平面 ----
    // 候选预过滤: 地面点只可能出现在先验带及其下方 ~1m 内 (容忍先验偏差/坑底),
    // 高处点(墙/桌)不参与采样 —— 提高命中率并省算力.
    std::vector<int> cand;
    cand.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        const double z = pts[i].z;
        if (z <= p.ground_prior_z + p.prior_window &&
            z >= p.ground_prior_z - p.prior_window - 1.0) {
            cand.push_back(i);
        }
    }

    const double cos_max_tilt = std::cos(p.plane_max_tilt_deg * kDegToRad);
    // 最少内点数: 过少的空间_patch不配叫"地面" (防止把 3 个孤立噪点拟合成平面)
    // 内点域已收窄到候选集 (下方扫描), 基线随候选集规模同步, 保留"空间 patch 最小规模"语义
    const int min_inliers = std::max(30, static_cast<int>(cand.size()) / 50);

    if (static_cast<int>(cand.size()) >= 3) {
        std::mt19937 rng(p.seed);
        std::uniform_int_distribution<int> pick(0, static_cast<int>(cand.size()) - 1);
        GroundPlane best;
        for (int it = 0; it < p.ransac_max_iters; ++it) {
            const int i1 = cand[pick(rng)], i2 = cand[pick(rng)], i3 = cand[pick(rng)];
            if (i1 == i2 || i2 == i3 || i1 == i3) continue;
            double nx, ny, nz, d;
            if (!plane_from_three(pts[i1], pts[i2], pts[i3], nx, ny, nz, d)) continue;
            if (nz < 0.0) { nx = -nx; ny = -ny; nz = -nz; d = -d; }  // 法向统一朝上
            if (nz < cos_max_tilt) continue;                          // 倾角约束 (nz=cos(tilt))
            const double h0 = -d / nz;                                // 原点处平面高度
            if (std::abs(h0 - p.ground_prior_z) > p.prior_window) continue;  // 高度先验约束

            int inl = 0;
            for (int i : cand) {   // 内点统计域: 全点 n → 候选集 (采样域=内点域, 语义一致, ~2x 提速)
                const auto& q = pts[i];
                if (std::abs(nx * q.x + ny * q.y + nz * q.z + d) <= p.ransac_inlier_dist) {
                    ++inl;
                }
            }
            if (inl > best.inliers) {
                best.valid = true;
                best.nx = nx; best.ny = ny; best.nz = nz; best.d = d;
                best.inliers = inl;
            }
            if (best.inliers >= min_inliers &&
                static_cast<double>(best.inliers) / n >= p.ransac_early_ratio) {
                break;  // 内点率达标, 提前退出
            }
        }

        if (best.valid && best.inliers >= min_inliers) {
            out.plane = best;
            for (int i = 0; i < n; ++i) {
                const auto& q = pts[i];
                const double s = best.nx * q.x + best.ny * q.y + best.nz * q.z + best.d;
                if (std::abs(s) <= p.point_on_plane_eps) {
                    out.ground_indices.push_back(i);
                } else {
                    out.obstacle_indices.push_back(i);
                }
            }
        }
    }

    // fail-closed: 平面没拟合出来 → 不输出负障碍 (底部 HC-SR04 独立兜底)
    if (!out.plane.valid) return;

    // ---- ③ 2.5D 栅格 + 按列扫描判负障碍 ----
    // 网格: x ∈ [0, neg_far+0.5], y ∈ [-2.5, +2.5] (近场 FOV 内; 5cm cell)
    const double y_half = 2.5;
    const double x_max = p.neg_far_m + 0.5;
    const int cols = static_cast<int>(x_max / p.cell_size) + 1;
    const int rows = static_cast<int>(2.0 * y_half / p.cell_size) + 1;
    std::vector<GridCell> grid(static_cast<size_t>(cols) * rows);

    for (int i : out.ground_indices) {
        const auto& q = pts[i];
        const int cx = static_cast<int>(q.x / p.cell_size);
        const int cy = static_cast<int>((q.y + y_half) / p.cell_size);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;
        GridCell& cell = grid[static_cast<size_t>(cy) * cols + cx];
        cell.has_return = true;
        cell.has_ground = true;  // 地面点必然 |s| <= eps
    }
    for (int i : out.obstacle_indices) {
        const auto& q = pts[i];
        const int cx = static_cast<int>(q.x / p.cell_size);
        const int cy = static_cast<int>((q.y + y_half) / p.cell_size);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;
        GridCell& cell = grid[static_cast<size_t>(cy) * cols + cx];
        const double s = out.plane.nx * q.x + out.plane.ny * q.y +
                         out.plane.nz * q.z + out.plane.d;
        if (s < cell.min_s) cell.min_s = s;
        cell.has_return = true;
    }

    // 按列 (固定 y, x 由近及远) 扫描:
    //   遇地面 cell → 更新参考点 ref (门口: 后方地面同高, ref 平滑接续, 不标)
    //   遇下沉 cell (min_s <= -cliff_drop_min) → (ref, c] 区间判负障碍,
    //     并把下沉面作为新参考 (下一级台阶/坑底继续扫描, 避免重复标记)
    //   空白 cell → 挂起 (持续空白不算证据; 深坑底部不可见时保守不标, 超声兜底)
    // 说明: 障碍物(如纸箱)所在 cell 无地面但 min_s > -cliff, 不触发; 若其后紧跟
    //       真实落差, 标记区间会含障碍 cell —— 保守方向(扩大危险区), 安全可接受.
    const int ref_gap = std::max(1, p.min_gap_cells);
    for (int r = 0; r < rows; ++r) {
        int ref = -1;  // 最近参考 cell (地面 或 已确认的下沉面); -1 = 尚未建立参考
        for (int c = 0; c < cols; ++c) {
            const GridCell& cell = grid[static_cast<size_t>(r) * cols + c];
            const double cell_x = (c + 0.5) * p.cell_size;
            if (cell_x < p.neg_near_m || cell_x > p.neg_far_m) {
                // 只在近场带内判负障碍; 带外的地面点仍可建立参考 (给带内首个落差用)
                if (cell.has_ground) ref = c;
                continue;
            }
            if (cell.has_ground) {
                ref = c;
                continue;
            }
            if (cell.has_return && cell.min_s <= -p.cliff_drop_min) {
                if (ref >= 0 && (c - ref) >= ref_gap) {
                    // (ref, c] 判负障碍: 标记点放在各 cell 中心的平面高度处
                    for (int m = ref + 1; m <= c; ++m) {
                        const double mx = (m + 0.5) * p.cell_size;
                        const double my = -y_half + (r + 0.5) * p.cell_size;
                        const double mz = -(out.plane.nx * mx + out.plane.ny * my +
                                            out.plane.d) / out.plane.nz;
                        Point3D np;
                        np.x = mx; np.y = my; np.z = mz;
                        out.negative_points.push_back(np);
                    }
                }
                ref = c;  // 下沉面成为新参考 (台阶逐级下探时逐级触发)
            }
            // 空白/悬空障碍 cell: 保持 ref 不变
        }
    }
}

} // namespace mechdog
