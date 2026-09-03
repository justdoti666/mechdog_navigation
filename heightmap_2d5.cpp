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

std::string HeightMap25Result::stats() const {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "hm25 %dx%d res=%.2fm  x[%.1f,%.1f]  unknown=%d traversable=%d up=%d down=%d steep=%d",
        cols, rows, cell_size, min_x_m, min_x_m + cols * cell_size,
        count_unknown, count_traversable, count_up, count_down, count_steep);
    return std::string(buf);
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
    out.height.assign(static_cast<size_t>(out.cols) * out.rows,
                      std::numeric_limits<float>::quiet_NaN());
    out.flag.assign(static_cast<size_t>(out.cols) * out.rows, CellFlag::Unknown);
    out.valid = true;

    const double cos_max_slope = std::cos(cfg.slope_max * 0.01745329251994329576);

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

            if (!cell.has_any) {
                out.flag[idx] = CellFlag::Unknown;
                ++out.count_unknown;
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

    // ---- 跨格坡度检查: 相邻列高度差过大 → 判 TooSteep ----
    // 单平面假设下, 平面本身是斜面; 若坡度超过 slope_max → 整列区域判陡。
    // 这里简化: 用平面法向与竖直的夹角判断整体坡度; 超过则按列传播。
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

} // namespace mechdog
