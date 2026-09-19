/**
 * 2.5D 高程/可通行地形实现 (P1.5)
 * 零依赖: 仅 <cmath>/<algorithm>/<limits>/<vector>.
 */
#include "heightmap_2d5.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mechdog {

bool HeightMap25Result::world_to_index(double wx, double wy,
                                       int& col, int& row) const {
    col = static_cast<int>((wx - min_x_m) / cell_size);
    row = static_cast<int>((wy + y_half_m) / cell_size);
    return col >= 0 && col < cols && row >= 0 && row < rows;
}

void HeightMap25Result::index_to_world(int col, int row,
                                       double& wx, double& wy) const {
    wx = min_x_m + (col + 0.5) * cell_size;
    wy = -y_half_m + (row + 0.5) * cell_size;
}

bool HeightMap25Result::in_fov(int col, int row) const {
    if (col < 0 || col >= cols || row < 0 || row >= rows) return false;
    double wx = 0.0, wy = 0.0;
    index_to_world(col, row, wx, wy);
    return std::abs(wy) <= wedge_y_slope * wx + wedge_margin_m;
}

double HeightMap25Result::fov_coverage() const {
    if (count_in_fov <= 0) return 0.0;
    return static_cast<double>(count_in_fov - count_fov_unknown) /
           static_cast<double>(count_in_fov);
}

std::string HeightMap25Result::stats() const {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "hm25 %dx%d res=%.2fm  x[%.1f,%.1f]  unknown=%d traversable=%d up=%d down=%d steep=%d"
        "  in_fov=%d/%d cov=%.0f%%",
        cols, rows, cell_size, min_x_m, min_x_m + cols * cell_size,
        count_unknown, count_traversable, count_up, count_down, count_steep,
        count_in_fov, cols * rows, fov_coverage() * 100.0);
    return std::string(buf);
}

// v2.9.3 深度质量守门 (见头文件说明)
bool depth_quality_ok(double valid_ratio, int point_count, DepthQualityIssue& issue) {
    if (!(valid_ratio >= DepthQualityConfig::min_valid_ratio)) {   // NaN 也拦 (fail-closed)
        issue = DepthQualityIssue::NoValidPixels;
        return false;
    }
    if (point_count < DepthQualityConfig::min_points) {
        issue = DepthQualityIssue::TooFewPoints;
        return false;
    }
    issue = DepthQualityIssue::Ok;
    return true;
}

void build_heightmap_25(const PointCloud& cloud, const GroundSegResult& seg,
                        const HeightMap25Config& cfg, HeightMap25Result& out) {
    out = HeightMap25Result{};
    // P1 fail-closed: 无地面平面 → 不出 2.5D (保守, 底部超声兜底)
    if (!seg.plane.valid) return;

    // ---- 栅格初始化 ----
    const double x_max = cfg.max_x_m;
    out.cols = static_cast<int>((x_max - cfg.min_x_m) / cfg.cell_size) + 1;
    out.rows = static_cast<int>(2.0 * cfg.y_half_m / cfg.cell_size) + 1;
    out.cell_size = cfg.cell_size;
    out.min_x_m = cfg.min_x_m;
    out.y_half_m = cfg.y_half_m;
    out.wedge_y_slope  = cfg.wedge_y_slope;   // v2.8
    out.wedge_margin_m = cfg.wedge_margin_m;
    out.height.assign(static_cast<size_t>(out.cols) * out.rows,
                      std::numeric_limits<float>::quiet_NaN());
    out.flag.assign(static_cast<size_t>(out.cols) * out.rows, CellFlag::Unknown);
    out.valid = true;

    // 逐点: 算带符号平面距离 s (上正下负, 相对拟合地面), 记录每格最高/最低表面
    // 用 s 而非绝对 z: s 直接是"相对地面高度", 与 P1 语义一致, 对外参/倾斜更鲁棒.
    struct CellRaw {
        bool  has_any   = false;
        bool  has_ground = false;   // |s| <= plane_eps
        double min_s = std::numeric_limits<double>::infinity();   // 最低(低于地面, 坑)
        double max_s = std::numeric_limits<double>::lowest();     // 最高(高于地面, 障碍)
    };
    std::vector<CellRaw> raw(static_cast<size_t>(out.cols) * out.rows);

    for (const auto& p : cloud.points) {
        int c, r;
        if (!out.world_to_index(p.x, p.y, c, r)) continue;
        CellRaw& cell = raw[static_cast<size_t>(r) * out.cols + c];
        const double s = seg.plane.nx * p.x + seg.plane.ny * p.y +
                         seg.plane.nz * p.z + seg.plane.d;
        cell.has_any = true;
        if (std::abs(s) <= cfg.plane_eps_m) cell.has_ground = true;
        if (s < cell.min_s) cell.min_s = s;
        if (s > cell.max_s) cell.max_s = s;
    }

    const double step_up = cfg.step_up_max_m;
    const double drop_down = cfg.drop_down_max_m;

    // 逐列(固定 y, x 由近及远) 判可通行
    for (int r = 0; r < out.rows; ++r) {
        // ref_plane_z = 当前列在该格 x 处的地面平面高度 (单平面)
        // 用平面方程反推: z = -(nx*x + ny*y + d)/nz
        for (int c = 0; c < out.cols; ++c) {
            double wx, wy;
            out.index_to_world(c, r, wx, wy);
            const size_t idx = static_cast<size_t>(r) * out.cols + c;
            const CellRaw& cell = raw[idx];

            // v2.8: 视场楔形统计 (只统计, 不改判据; legacy 时全格计入 ⇒ 与历史一致)
            const bool cell_in_fov = !cfg.wedge_only ||
                std::abs(wy) <= cfg.wedge_y_slope * wx + cfg.wedge_margin_m;
            if (cell_in_fov) ++out.count_in_fov;

            if (!cell.has_any) {
                out.flag[idx] = CellFlag::Unknown;
                ++out.count_unknown;
                if (cell_in_fov) ++out.count_fov_unknown;
                continue;
            }

            // 参考地面平面高度 (相机坐标系下, 上方为正)
            const double plane_z = -(seg.plane.nx * wx + seg.plane.ny * wy +
                                     seg.plane.d) / seg.plane.nz;
            out.height[idx] = static_cast<float>(plane_z);

            // 用带符号距离 s 判凸起/坑 (相对地面, 上正下负)
            const double up_h   = (cell.max_s > std::numeric_limits<double>::lowest())
                                      ? std::max(0.0, cell.max_s) : 0.0;   // 最高凸起
            const double down_h = (cell.min_s < std::numeric_limits<double>::infinity())
                                      ? std::max(0.0, -cell.min_s) : 0.0;  // 最深凹陷

            if (up_h > step_up) {
                out.flag[idx] = CellFlag::ObstacleUp;
                ++out.count_up;
            } else if (down_h > drop_down) {
                out.flag[idx] = CellFlag::CliffDown;
                ++out.count_down;
            } else {
                out.flag[idx] = CellFlag::Traversable;
                ++out.count_traversable;
            }
        }
    }

    // ---- 整体坡度检查: 拟合平面自身倾角过大 → 判 TooSteep ----
    // 口径说明 (FIX-09 核实): 这是**整体平面**级判断, 不是逐格/相邻格坡度。
    // 在"每格高度相对拟合平面(带符号距离 s)"的基准下, 逐格坡度与已有
    // step_up_max_m(0.10)/drop_down_max_m(0.15) 重复, 故未实现(原注释曾误称
    // "相邻列高度差过大", 已更正)。
    // 可达性: 上游 segment_ground 的 plane_max_tilt_deg(默认 15°) 会先拒掉 >15°
    // 的平面, 而本处阈值 slope_max 默认 20° ⇒ 生产路径下本分支不可达(死代码级),
    // 只有外部构造的 seg 才能触发 —— 见 tests/test_heightmap_2d5.cpp 的 T6。
    // 若将来放宽上游容限以支持爬坡, 必须同批改为"只标有样本的格 + 单独输出坡度角"。
    const double plane_tilt = std::acos(std::clamp(seg.plane.nz, -1.0, 1.0));
    if (plane_tilt > cfg.slope_max * 0.01745329251994329576) {
        // 地面本身过陡 → 所有 Traversable 改为 TooSteep (保守)
        for (size_t i = 0; i < out.flag.size(); ++i) {
            if (out.flag[i] == CellFlag::Traversable) {
                out.flag[i] = CellFlag::TooSteep;
                --out.count_traversable;
                ++out.count_steep;
            }
        }
    }
}

// ============================================================
// 走廊扫描 (近场地形避障; 见 config.h TerrainAvoidConfig)
// ============================================================
CorridorScan scan_corridor(const HeightMap25Result& hm,
                           double x_lo, double x_hi, double y_half) {
    CorridorScan sc;
    if (!hm.valid || hm.cols <= 0 || hm.rows <= 0) return sc;
    if (x_hi < x_lo) std::swap(x_lo, x_hi);
    double sum_y = 0.0;
    for (int r = 0; r < hm.rows; ++r) {
        for (int c = 0; c < hm.cols; ++c) {
            const CellFlag f = hm.flag[static_cast<size_t>(r) * hm.cols + c];
            if (f != CellFlag::CliffDown && f != CellFlag::ObstacleUp &&
                f != CellFlag::TooSteep) continue;
            double wx = 0.0, wy = 0.0;
            hm.index_to_world(c, r, wx, wy);
            if (wx < x_lo || wx > x_hi) continue;
            if (std::fabs(wy) > y_half) continue;
            if (!sc.blocked || wx < sc.nearest_x_m) sc.nearest_x_m = wx;
            sc.blocked = true;
            ++sc.count;
            sum_y += wy;
        }
    }
    if (sc.count > 0) sc.mean_y_m = sum_y / sc.count;
    return sc;
}

CorridorScan scan_corridor_points(const std::vector<Point3D>& points,
                                  double x_lo, double x_hi, double y_half) {
    CorridorScan sc;
    if (x_hi < x_lo) std::swap(x_lo, x_hi);
    double sum_y = 0.0;
    for (const auto& p : points) {
        if (p.x < x_lo || p.x > x_hi) continue;
        if (std::fabs(p.y) > y_half) continue;
        if (!sc.blocked || p.x < sc.nearest_x_m) sc.nearest_x_m = p.x;
        sc.blocked = true;
        ++sc.count;
        sum_y += p.y;
    }
    if (sc.count > 0) sc.mean_y_m = sum_y / sc.count;
    return sc;
}

CorridorScan merge_corridor(const CorridorScan& a, const CorridorScan& b) {
    CorridorScan m;
    m.count   = a.count + b.count;
    m.blocked = a.blocked || b.blocked;
    if (a.blocked && b.blocked)      m.nearest_x_m = std::min(a.nearest_x_m, b.nearest_x_m);
    else if (a.blocked)              m.nearest_x_m = a.nearest_x_m;
    else if (b.blocked)              m.nearest_x_m = b.nearest_x_m;
    if (m.count > 0) {
        const double wa = static_cast<double>(a.count);
        const double wb = static_cast<double>(b.count);
        m.mean_y_m = (a.mean_y_m * wa + b.mean_y_m * wb) / (wa + wb);
    }
    return m;
}

} // namespace mechdog
