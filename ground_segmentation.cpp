/**
 * 地面分割与负障碍检测实现 (P1)
 *
 * 零依赖: 仅 <cmath>/<vector>/<random>/<limits>, 不引 PCL (与点云模块同风格).
 */
#include "ground_segmentation.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
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

// ============================================================
// v2.7: 确定性地面提取 (格最小拟合) —— 见头文件说明
// ============================================================
namespace {
// 最小二乘 z = a*x + b*y + c (对给定点集); 返回 false = 退化 (点太少/共线)
bool lsq_plane_zy(const std::vector<std::array<double, 3>>& q,
                  double& a, double& b, double& c) {
    if (q.size() < 3) return false;
    double Sx = 0, Sy = 0, Sxx = 0, Syy = 0, Sxy = 0, Sz = 0, Sxz = 0, Syz = 0;
    const double n = static_cast<double>(q.size());
    for (const auto& p : q) {
        Sx += p[0]; Sy += p[1]; Sxx += p[0] * p[0]; Syy += p[1] * p[1];
        Sxy += p[0] * p[1]; Sz += p[2]; Sxz += p[0] * p[2]; Syz += p[1] * p[2];
    }
    double M[3][4] = {{Sxx, Sxy, Sx, Sxz}, {Sxy, Syy, Sy, Syz}, {Sx, Sy, n, Sz}};
    for (int i = 0; i < 3; ++i) {
        int piv = i;
        for (int r = i + 1; r < 3; ++r) if (std::abs(M[r][i]) > std::abs(M[piv][i])) piv = r;
        for (int cc = 0; cc < 4; ++cc) std::swap(M[i][cc], M[piv][cc]);
        if (std::abs(M[i][i]) < 1e-12) return false;
        for (int r = 0; r < 3; ++r) {
            if (r == i) continue;
            const double f = M[r][i] / M[i][i];
            for (int cc = i; cc < 4; ++cc) M[r][cc] -= f * M[i][cc];
        }
    }
    a = M[0][3] / M[0][0]; b = M[1][3] / M[1][1]; c = M[2][3] / M[2][2];
    return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
}
} // namespace

bool fit_ground_plane_cells(const PointCloud& cloud, const GroundSegParams& p,
                            GroundPlane& out) {
    out = GroundPlane{};
    const double cs = (p.cell_size > 0.01) ? p.cell_size : 0.01;

    // ① 只按**高度先验带**过滤 —— 不假设任何 x/y 足迹 (盒/锥)。
    //    理由 (真机实测 2026-09-13): 台架相机除俯仰外还明显**偏航** (base 系 y∈[-4.58,-0.60]),
    //    任何"近场 x/y 盒"都会把地板整片漏掉 (逐级过滤最后一步 过高度带=0)。
    //    地面在物理上就是"某个高度上的最低大面", 与它落在视野何处无关 ⇒ 只按 z 过滤。
    //    margin 0.6 容忍比先验更低的面 (坑底/下行台阶)。
    const double z_lo = p.ground_prior_z - p.prior_window - 0.6;
    const double z_hi = p.ground_prior_z + p.prior_window;
    std::map<std::pair<int, int>, double> lowest;
    for (const auto& q : cloud.points) {
        if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) continue;
        if (q.z < z_lo || q.z > z_hi) continue;
        const auto key = std::make_pair(static_cast<int>(std::floor(q.x / cs)),
                                        static_cast<int>(std::floor(q.y / cs)));
        auto it = lowest.find(key);
        if (it == lowest.end() || q.z < it->second) it = lowest.emplace(key, q.z).first, it->second = q.z;
    }
    if (lowest.size() < 12) return false;   // 样本不足 → 交给 RANSAC
                                            // (阈值取 12: 台架相机 0.6m 高近水平看时地板只占
                                            //  0.2~0.5 m² ≈ 20 格; 装机 0.2m 俯视会大得多)

    std::vector<std::array<double, 3>> q;
    q.reserve(lowest.size());
    for (const auto& [key, z] : lowest) {
        q.push_back({(key.first + 0.5) * cs, (key.second + 0.5) * cs, z});
    }

    // ② 稳健播种: 取格最小值的 **25 百分位**作高度种子 (地面是最低面) + 先假设水平 (a=b=0)。
    //    不用普通 LSQ 当种子 —— 它会被桌面/物体格拉偏, 直接收敛到错面 (实测踩过)。
    std::vector<double> zs;
    zs.reserve(q.size());
    for (const auto& pt : q) zs.push_back(pt[2]);
    std::sort(zs.begin(), zs.end());
    double a = 0.0, b = 0.0, c = zs[zs.size() / 4];

    // ③ 逐级收敛 (0.40 → 0.20 → 0.10 → 0.04 m): 每轮"保留带内格 → 重拟合"。
    //    第一轮放宽是为了让倾斜地面也能被整片收进来 (水平种子对 13° 地面会先丢远处),
    //    第二轮起平面已经贴近真实地面, 后续轮次只做精修与离群剔除。
    const double pass_thr[4] = {0.40, 0.20, 0.10, 0.04};
    const double seed_z = c;                 // 25 百分位种子高度 (地面附近)
    std::vector<std::array<double, 3>> keep;
    for (int pass = 0; pass < 4; ++pass) {
        keep.clear();
        for (const auto& pt : q) {
            // ★ 地面是**最低面**: 第一轮额外限制"不高于种子 + 0.15m", 防止桌面/房间被收进来
            //   后把最小二乘拽向陡面 (实测: 不设此约束时收敛到 tilt 39°, h0 为正)。
            if (pass == 0 && pt[2] > seed_z + 0.15) continue;
            const double r = pt[2] - (a * pt[0] + b * pt[1] + c);
            if (std::abs(r) <= pass_thr[pass]) keep.push_back(pt);
        }
        if (keep.size() < 12) return false;
        q = keep;
        if (!lsq_plane_zy(q, a, b, c)) return false;
    }
    // 残差守门: 小幅面也允许, 但拟合必须真的"平" (防 12 格噪声被拟合成歪面)
    {
        double ss = 0.0;
        for (const auto& pt : q) {
            const double r = pt[2] - (a * pt[0] + b * pt[1] + c);
            ss += r * r;
        }
        if (std::sqrt(ss / static_cast<double>(q.size())) > 0.05) return false;
    }

    // ③ 归一化 + 约束校验 (与 RANSAC 同一套先验, 口径一致)
    //    先把候选写进 out (即使校验不通过) —— 便于调用方/诊断看到"为什么被拒"
    const double len = std::sqrt(a * a + b * b + 1.0);
    const double nx = -a / len, ny = -b / len, nz = 1.0 / len;   // n ∝ (-a, -b, 1), nz>0
    const double d = -c / len;                                    // z = ax+by+c ⇔ n·X + d = 0
    const double h0 = -d / nz;                                    // = c
    out.nx = nx; out.ny = ny; out.nz = nz; out.d = d;
    {
        int inl = 0;
        for (const auto& pt : cloud.points) {
            if (std::abs(nx * pt.x + ny * pt.y + nz * pt.z + d) <= p.ransac_inlier_dist) ++inl;
        }
        out.inliers = inl;
    }
    const double cos_max_tilt = std::cos(p.plane_max_tilt_deg * kDegToRad);
    if (nz < cos_max_tilt) return false;                          // 倾角超限 → 交给 RANSAC
    if (std::abs(h0 - p.ground_prior_z) > p.prior_window) return false;  // 高度先验
    out.valid = true;
    return true;
}

void segment_ground(const PointCloud& cloud, const GroundSegParams& p,
                    GroundSegResult& out) {
    out = GroundSegResult{};
    const int n = static_cast<int>(cloud.points.size());
    if (n < 3) return;
    const auto& pts = cloud.points;

    // ---- ⓪ (v2.7, 可选) 确定性地面提取优先, 失败回退 RANSAC ----
    GroundPlane plane;
    bool have_plane = false;
    if (p.use_cell_min_fit) {
        have_plane = fit_ground_plane_cells(cloud, p, plane);
    }

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

        if (best.valid && best.inliers >= min_inliers) { plane = best; have_plane = true; }
    }

    if (have_plane) {
        out.plane = plane;
            for (int i = 0; i < n; ++i) {
                const auto& q = pts[i];
                const double s = plane.nx * q.x + plane.ny * q.y + plane.nz * q.z + plane.d;
                if (std::abs(s) <= p.point_on_plane_eps) {
                    out.ground_indices.push_back(i);
                } else {
                    out.obstacle_indices.push_back(i);
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
