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

OccupancyGridMap::OccupancyGridMap(const MapConfig& /*cfg*/,
                                   double robot_radius_m,
                                   double extrinsics_x_m,
                                   double width_m, double height_m,
                                   double resolution_m)
    : robot_radius_m_(robot_radius_m),
      extrinsics_x_m_(extrinsics_x_m),
      resolution_m_(resolution_m > 1e-6 ? resolution_m
                                        : MapConfig::grid_size_m) {
    width_  = std::max(2, static_cast<int>(
        std::lround(width_m / resolution_m_)) & ~1);   // 偶数格, 原点居中
    height_ = std::max(2, static_cast<int>(
        std::lround(height_m / resolution_m_)) & ~1);
    const size_t n = static_cast<size_t>(width_) * height_;
    grid_.assign(n, 0);       // 0 = 未知
    inflated_.assign(n, 0);
    negative_.assign(n, 0);
}

// ------------------------------------------------------------
// 相机原点 (odom 系): base 前方 extrinsics_x_m_ (构造注入,
// 与 CameraExtrinsics::x 同源 — FIX_PLAN #7, 不再硬编码 0.12)
// ------------------------------------------------------------
namespace {
inline void camera_origin(const Pose2D& pose, double cam_x,
                          double& ox, double& oy) {
    rotate_2d(cam_x, 0.0, pose.theta, ox, oy);
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
    camera_origin(robot_pose, extrinsics_x_m_, ox, oy);

    // ---- 第一遍: base→odom 世界坐标 + 本帧命中格集合 ----
    // (fix 光线自清除: hit 延后统一应用; 光线 miss 跳过本帧命中格)
    std::vector<std::pair<double, double>> world;
    world.reserve(cloud_base.points.size());
    std::vector<int> hit_cells;
    hit_cells.reserve(cloud_base.points.size());
    for (const auto& p : cloud_base.points) {
        double wx, wy;
        rotate_2d(p.x, p.y, robot_pose.theta, wx, wy);
        wx += robot_pose.x;
        wy += robot_pose.y;
        world.emplace_back(wx, wy);
        int col, row;
        if (world_to_index(wx, wy, col, row))
            hit_cells.push_back(static_cast<int>(
                static_cast<size_t>(row) * width_ + col));
        else
            ++dropped_points_;   // FIX_PLAN #6: 越界计数, 不静默
    }
    std::vector<int> hit_set = hit_cells;
    std::sort(hit_set.begin(), hit_set.end());
    hit_set.erase(std::unique(hit_set.begin(), hit_set.end()),
                  hit_set.end());

    // ---- 第二遍: 光线空闲标记 (miss; 跳过本帧命中格) ----
    for (const auto& w : world) {
        const double dx = w.first - ox, dy = w.second - oy;
        const double dist = std::hypot(dx, dy);
        if (dist <= 1e-6 || dist > rc.max_free_range_m) continue;
        const int steps = static_cast<int>(dist / rc.step_m);
        for (int s = 1; s < steps; ++s) {
            const double t = static_cast<double>(s) / steps;
            const double fx = ox + t * dx, fy = oy + t * dy;
            int fc, fr;
            if (!world_to_index(fx, fy, fc, fr)) continue;
            const int fidx = static_cast<int>(
                static_cast<size_t>(fr) * width_ + fc);
            if (std::binary_search(hit_set.begin(), hit_set.end(), fidx))
                continue;  // 命中格不打 miss (端点/邻命中保护)
            grid_[fidx] = static_cast<int8_t>(
                clamp_l_(grid_[fidx] - LFREE));
        }
    }

    // ---- 第三遍: 占据标记 (hit 统一应用) ----
    for (int i : hit_cells) {
        grid_[i] = static_cast<int8_t>(clamp_l_(grid_[i] + LOCC));
    }
}

void OccupancyGridMap::insert_cloud_filtered(const PointCloud& cloud_base,
                                             const Pose2D& robot_pose) {
    if (cloud_base.points.empty()) return;

    // FIX_PLAN #1: 地面分割 — 障碍/地面/负障碍三分离
    GroundSegParams gp{};
    GroundSegResult seg;
    segment_ground(cloud_base, gp, seg);

    // 障碍点 → 普通占据管线
    PointCloud obstacles;
    obstacles.frame_id = cloud_base.frame_id;
    obstacles.stamp = cloud_base.stamp;
    obstacles.points.reserve(seg.obstacle_indices.size());
    for (int i : seg.obstacle_indices)
        obstacles.points.push_back(cloud_base.points[i]);
    insert_cloud(obstacles, robot_pose);

    // FIX_PLAN #9: 负障碍点 → 独立标记层 (不参与占据/光线)
    for (const auto& np : seg.negative_points) {
        double wx, wy;
        rotate_2d(np.x, np.y, robot_pose.theta, wx, wy);
        wx += robot_pose.x;
        wy += robot_pose.y;
        int col, row;
        if (world_to_index(wx, wy, col, row))
            negative_[static_cast<size_t>(row) * width_ + col] = 1;
        else
            ++dropped_points_;
    }
    // 地面点: 丢弃 (消除假墙)
}

void OccupancyGridMap::inflate(double radius_m) {
    const double res = resolution_m_;
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
    char buf[200];
    std::snprintf(buf, sizeof(buf),
        "map %dx%d res=%.2fm  unknown=%d free=%d occ=%d neg=%ld dropped=%ld",
        width_, height_, resolution_m_,
        count_cells(-1), count_cells(0), count_cells(100),
        std::count(negative_.begin(), negative_.end(), 1),
        dropped_points_);
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

bool OccupancyGridMap::save_nav2_map(const std::string& base_path) const {
    const std::string pgm = base_path + ".pgm";
    const std::string yaml = base_path + ".yaml";

    // --- P5 二进制 PGM (与 save_pgm 同值映射/行序, 二进制写) ---
    std::FILE* f = std::fopen(pgm.c_str(), "wb");
    if (!f) return false;
    std::fprintf(f, "P5\n%d %d\n255\n", width_, height_);
    for (int row = height_ - 1; row >= 0; --row) {
        for (int col = 0; col < width_; ++col) {
            const int s = occ_state(col, row);
            const unsigned char v = (s == 100) ? 0u
                                 : (s == 0   ? 254u : 205u);
            std::fwrite(&v, 1, 1, f);
        }
    }
    std::fclose(f);

    // --- map.yaml (nav2 map_server 加载项) ---
    std::FILE* y = std::fopen(yaml.c_str(), "wb");
    if (!y) return false;
    std::fprintf(y,
        "image: %s\n"
        "resolution: %.6f\n"
        "origin: [%.6f, %.6f, 0.000000]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.20\n",
        pgm.c_str(), resolution_m_,
        -(width_ * resolution_m_) * 0.5,
        -(height_ * resolution_m_) * 0.5);
    std::fclose(y);
    return true;
}

} // namespace mechdog
