/**
 * 建图模块 (P4) — 实现
 *
 * 设计文档: docs/POINT_CLOUD_DESIGN.md §6.3 / §220 / P4
 * 依赖: 仅 point_cloud.h 类型 + 标准库 (cmath, cstdio, algorithm)
 */
#include "mapping.h"

#include <cmath>
#include <cstdio>
#include <algorithm>

namespace mechdog {

// ============================================================
// 辅助: 2D 旋转 (base→odom 变换用机器人 yaw)
// ============================================================
namespace {

inline void rotate_2d(double x, double y, double theta,
                      double& ox, double& oy) {
    const double c = std::cos(theta), s = std::sin(theta);
    ox = c * x - s * y;
    oy = s * x + c * y;
}

} // namespace

// ============================================================
// OccupancyGridMap
// ============================================================

OccupancyGridMap::OccupancyGridMap(double robot_radius_m)
    : robot_radius_m_(robot_radius_m) {
    const int dim = dim_cells_();
    width_ = dim;
    height_ = dim;
    grid_.assign(static_cast<size_t>(dim) * dim, 0); // 0 = 未知
    inflated_.assign(static_cast<size_t>(dim) * dim, 0);
}

// ------------------------------------------------------------
// 相机原点 (odom 系): base 前方 0.12m (CameraExtrinsics::x 同源)
// ------------------------------------------------------------
namespace {
inline void camera_origin(const Pose2D& pose, double& ox, double& oy) {
    rotate_2d(0.12, 0.0, pose.theta, ox, oy);
    ox += pose.x;
    oy += pose.y;
}
} // namespace

void OccupancyGridMap::insert_cloud(const PointCloud& cloud_base,
                                    const Pose2D& robot_pose) {
    if (cloud_base.points.empty()) return; // 空帧跳过, 不清图

    const RaycastConfig rc;

    // 相机原点 (光线起点)
    double ox, oy;
    camera_origin(robot_pose, ox, oy);

    for (const auto& p : cloud_base.points) {
        // --- base 系 → odom 系: 旋转(yaw) + 平移(位姿) ---
        double wx, wy;
        rotate_2d(p.x, p.y, robot_pose.theta, wx, wy);
        wx += robot_pose.x;
        wy += robot_pose.y;

        // --- 光线空闲标记 (仅近距离命中, 防远噪声误清) ---
        const double dx = wx - ox, dy = wy - oy;
        const double dist = std::hypot(dx, dy);
        if (dist > 1e-6 && dist <= rc.max_free_range_m) {
            // 步进标记沿途格为 miss (log-odds 减)
            const int steps = static_cast<int>(
                dist / rc.step_m);
            for (int s = 1; s < steps; ++s) {
                const double t = static_cast<double>(s) / steps;
                const double fx = ox + t * dx, fy = oy + t * dy;
                int fc, fr;
                if (!world_to_index(fx, fy, fc, fr)) continue;
                const size_t fidx =
                    static_cast<size_t>(fr) * width_ + fc;
                grid_[fidx] = static_cast<int8_t>(
                    clamp_l_(grid_[fidx] - LFREE));
            }
        }

        // --- 占据标记 (hit, log-odds 加) ---
        int col, row;
        if (!world_to_index(wx, wy, col, row)) continue;
        const size_t idx = static_cast<size_t>(row) * width_ + col;
        grid_[idx] = static_cast<int8_t>(clamp_l_(grid_[idx] + LOCC));
    }
}

void OccupancyGridMap::inflate(double radius_m) {
    const double res = MapConfig::grid_size_m;
    const int r_cells = static_cast<int>(
        std::lround(radius_m / res));
    if (r_cells <= 0) return;

    inflated_.assign(inflated_.size(), 0);

    for (int row = 0; row < height_; ++row) {
        for (int col = 0; col < width_; ++col) {
            if (occ_state(col, row) != 100) continue;
            inflated_[static_cast<size_t>(row) * width_ + col] = 1;
        }
    }

    // 距离变换: 对每个占据格, 半径内邻域标记膨胀
    // (O(N·r²) 朴素实现, 200×200 图 < 1ms 量级, P4 够用)
    std::vector<uint8_t> result = inflated_;
    for (int row = 0; row < height_; ++row) {
        for (int col = 0; col < width_; ++col) {
            if (!inflated_[static_cast<size_t>(row) * width_ + col]) continue;
            for (int dr = -r_cells; dr <= r_cells; ++dr) {
                for (int dc = -r_cells; dc <= r_cells; ++dc) {
                    if (dr == 0 && dc == 0) continue;
                    // 圆形核: 排除方形角
                    if (dr * dr + dc * dc > r_cells * r_cells) continue;
                    const int nr = row + dr, nc = col + dc;
                    if (nr < 0 || nr >= height_ || nc < 0 || nc >= width_) continue;
                    result[static_cast<size_t>(nr) * width_ + nc] = 1;
                }
            }
        }
    }
    inflated_ = std::move(result);
}

int OccupancyGridMap::occ_state(int idx) const {
    const int l = grid_[idx];
    if (l >= L_OCC_TH) return 100;
    if (l <= L_FREE_TH) return 0;
    return -1;
}

int OccupancyGridMap::occ_state(int col, int row) const {
    if (col < 0 || col >= width_ || row < 0 || row >= height_) return -1;
    return occ_state(static_cast<int>(static_cast<size_t>(row) * width_ + col));
}

bool OccupancyGridMap::world_to_index(double wx, double wy,
                                      int& col, int& row) const {
    col = world_to_col_row_(wx);
    row = world_to_col_row_(wy);
    return col >= 0 && col < width_ && row >= 0 && row < height_;
}

void OccupancyGridMap::index_to_world(int col, int row,
                                      double& wx, double& wy) const {
    wx = col_row_to_world_(col);
    wy = col_row_to_world_(row);
}

int OccupancyGridMap::count_cells(int state) const {
    int n = 0;
    for (size_t i = 0; i < grid_.size(); ++i) {
        if (occ_state(static_cast<int>(i)) == state) ++n;
    }
    return n;
}

std::string OccupancyGridMap::stats() const {
    char buf[160];
    std::snprintf(buf, sizeof(buf),
        "map %dx%d res=%.2fm  unknown=%d free=%d occ=%d",
        width_, height_, MapConfig::grid_size_m,
        count_cells(-1), count_cells(0), count_cells(100));
    return std::string(buf);
}

bool OccupancyGridMap::save_pgm(const std::string& path) const {
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;

    // P2 (ASCII) 格式: 205=未知, 254=空闲, 0=占据 (ROS map_server 惯例)
    std::fprintf(f, "P2\n%d %d\n255\n", width_, height_);
    for (int row = height_ - 1; row >= 0; --row) { // PGM 行序=世界+y向上
        for (int col = 0; col < width_; ++col) {
            const int s = occ_state(col, row);
            const int v = (s == 100) ? 0 : (s == 0 ? 254 : 205);
            std::fprintf(f, "%d%c", v,
                (col + 1 == width_) ? '\n' : ' ');
        }
    }
    std::fclose(f);
    return true;
}

} // namespace mechdog
