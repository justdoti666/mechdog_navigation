/**
 * mapping_plus.hpp — 建图库 · 单文件自包含增强版 (P4.1)
 *
 * 用途: 在你现有 mechdog_navigation 的 P4 建图 (mapping.h/.cpp) 基础上,
 *   补上让它真正可用的三块缺口, 并把已知坑修掉。全在【一个文件】里。
 *
 *   ① 地面过滤直连 —— insert_cloud 前先 segment_ground, 只投"障碍+负障碍"
 *      点, 消除"地面被投影成假墙" (你 README 已知边界里也点了 P1 未接入)。
 *   ② 真·P5 二进制 PGM + map.yaml 联合导出 —— nav2 map_server 加载要成对的
 *      image/resolution/origin/negate/thresholds, 你原实现只出 P2 PGM, 直接丢
 *      给 Nav2 会报"找不到 map.yaml"。
 *   ③ 单一入口 MappingPipeline —— depth→to_base→ground filter→log-odds→
 *      inflate→save(pgm+yaml) 一步到位, 零 ROS 依赖, 可离线单测。
 *
 * 兼容性: 与你的 mapping.h 可并存 (本文件用独立类名 OccupancyMap /
 *   MappingPose / MappingPipeline, 不重定义 Pose2D/OccupancyGridMap)。
 * 依赖: 仅你已有的点云/地面分割头 (point_cloud.h, ground_segmentation.h) +
 *   config.h + 标准库。不引 PCL/ROS。
 */
#pragma once

#include "config.h"
#include "point_cloud.h"
#include "ground_segmentation.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

namespace mechdog {

// ============================================================
// 2D 位姿 (独立类型, 避免与 mapping.h 的 Pose2D 重名)
// ============================================================
struct MappingPose {
    double x = 0.0, y = 0.0, theta = 0.0;  // odom 系 (米, 弧度)
};

// ============================================================
// 光线空闲标记参数 (防远距离深度噪声误标空闲)
// ============================================================
struct MappingRayConfig {
    double max_free_range_m = 5.0;           // 只在 5m 内做光线空闲标记
    double step_m           = MapConfig::grid_size_m * 0.5;  // 2.5cm
};

// ============================================================
// OccupancyMap — log-odds 2D 占据栅格 (与你的 P4 同量级, 自足实现)
//
//   值域 [-LMAX,+LMAX]; hit: +=LOCC, miss: -=LFREE;
//   判占 occ_state: >=L_OCC_TH -> 100, <=L_FREE_TH -> 0, 其余 -> -1(未知)。
//   参数量级取自 Cartographer 默认 (和你 mapping.h 注释一致)。
// ============================================================
class OccupancyMap {
public:
    explicit OccupancyMap(double robot_radius_m = 0.25)
        : robot_radius_m_(robot_radius_m) {
        const int dim = dim_cells_();
        width_ = dim; height_ = dim;
        grid_.assign(static_cast<size_t>(dim) * dim, 0);
        inflated_.assign(static_cast<size_t>(dim) * dim, 0);
    }

    // --------------------------------------------------------
    // 单帧: 输入【障碍-only 的 base 系点云】+ 位姿 → 投到 odom 系,
    // 光线空闲 + 端点占据 (log-odds)。
    // 空点云安全跳过 (不清图, 不崩溃)。
    // ※ 调用方应先用 segment_ground 过滤地面后再进来 (见 MappingPipeline)。
    //
    // 顺序保证 (fix: 光线自清除): miss 先全部应用, hit 最后统一应用。
    // 否则 ① 光线步进终点与命中端点同格 → hit(+6) 后 miss(-6) 抵消成未知;
    //      ② 同帧邻近障碍的光线横穿本命中格 → 密集墙面被打花。
    // --------------------------------------------------------
    void insert_cloud(const PointCloud& obstacles_base,
                      const MappingPose& robot_pose) {
        if (obstacles_base.points.empty()) return;

        const MappingRayConfig rc;
        double ox, oy; camera_origin(robot_pose, ox, oy);

        // ---- 第一遍: 世界坐标 + 本帧命中格集合 ----
        std::vector<std::pair<double, double>> world;  // 命中点世界坐标
        world.reserve(obstacles_base.points.size());
        std::vector<int> hit_cells;                    // 本帧命中格线性索引
        hit_cells.reserve(obstacles_base.points.size());
        for (const auto& p : obstacles_base.points) {
            const double wx = c_(p.x, p.y, robot_pose.theta) + robot_pose.x;
            const double wy = s_(p.x, p.y, robot_pose.theta) + robot_pose.y;
            world.emplace_back(wx, wy);
            int col, row;
            if (world_to_index(wx, wy, col, row))
                hit_cells.push_back(static_cast<int>(idx_(col, row)));
        }
        // 命中格集合 (排序去重, 供光线 miss 排除)
        std::vector<int> hit_set = hit_cells;
        std::sort(hit_set.begin(), hit_set.end());
        hit_set.erase(std::unique(hit_set.begin(), hit_set.end()), hit_set.end());

        // ---- 第二遍: 光线 miss (跳过本帧命中格) ----
        for (const auto& w : world) {
            const double dx = w.first - ox, dy = w.second - oy;
            const double dist = std::hypot(dx, dy);
            if (dist <= 1e-6 || dist > rc.max_free_range_m) continue;
            const int steps = static_cast<int>(dist / rc.step_m);
            for (int k = 1; k < steps; ++k) {
                const double t = static_cast<double>(k) / steps;
                int fc, fr;
                if (!world_to_index(ox + t * dx, oy + t * dy, fc, fr)) continue;
                const int fidx = static_cast<int>(idx_(fc, fr));
                // 命中格不打 miss (端点所在格/邻命中格保护)
                if (std::binary_search(hit_set.begin(), hit_set.end(), fidx)) continue;
                auto& g = grid_[fidx];
                g = static_cast<int8_t>(clamp_l_(g - LFREE));
            }
        }

        // ---- 第三遍: hit 统一应用 ----
        for (int i : hit_cells) {
            auto& g = grid_[static_cast<size_t>(i)];
            g = static_cast<int8_t>(clamp_l_(g + LOCC));
        }
    }

    // --------------------------------------------------------
    // 障碍膨胀 (圆形核, 默认 MapConfig::inflation_radius_m)
    // --------------------------------------------------------
    void inflate(double radius_m = MapConfig::inflation_radius_m) {
        const int r = static_cast<int>(std::lround(radius_m / MapConfig::grid_size_m));
        if (r <= 0) return;
        inflated_.assign(inflated_.size(), 0);
        for (int row = 0; row < height_; ++row)
            for (int col = 0; col < width_; ++col)
                if (is_occupied(col, row)) inflated_[idx_(col, row)] = 1;

        auto result = inflated_;
        for (int row = 0; row < height_; ++row) {
            for (int col = 0; col < width_; ++col) {
                if (!inflated_[idx_(col, row)]) continue;
                for (int dr = -r; dr <= r; ++dr) {
                    for (int dc = -r; dc <= r; ++dc) {
                        if (dr == 0 && dc == 0) continue;
                        if (dr * dr + dc * dc > r * r) continue;  // 圆核
                        const int nr = row + dr, nc = col + dc;
                        if (nr < 0 || nr >= height_ || nc < 0 || nc >= width_) continue;
                        result[idx_(nc, nr)] = 1;
                    }
                }
            }
        }
        inflated_ = std::move(result);
    }

    // --------------------------------------------------------
    // 导出: P5 二进制 PGM + 配对的 map.yaml (nav2 map_server 可加载)。
    // 返回 false = 文件不可写。prefix 可选; 不加扩展名自动补 .pgm/.yaml。
    // --------------------------------------------------------
    bool save_nav2_map(const std::string& base_path) const {
        const std::string pgm = base_path + ".pgm";
        const std::string yaml = base_path + ".yaml";
        return save_pgm_p5(pgm) && save_yaml(yaml, pgm);
    }

    bool save_pgm_p5(const std::string& path) const {
        std::FILE* f = std::fopen(path.c_str(), "wb");
        if (!f) return false;
        std::fprintf(f, "P5\n%d %d\n255\n", width_, height_);
        for (int row = height_ - 1; row >= 0; --row) {   // 行序 = 世界 +y 向上
            for (int col = 0; col < width_; ++col) {
                const int s = occ_state(col, row);
                const unsigned char v = (s == 100) ? 0u
                                     : (s == 0   ? 254u : 205u);  // 0占/254空/205未知
                std::fwrite(&v, 1, 1, f);
            }
        }
        std::fclose(f);
        return true;
    }

    // map.yaml: 地图原点取 odom 系原点 (建图起点), 即世界坐标 (-wm/2,-hm/2)
    bool save_yaml(const std::string& path, const std::string& image_rel) const {
        std::FILE* f = std::fopen(path.c_str(), "wb");
        if (!f) return false;
        const double res = MapConfig::grid_size_m;
        const double ox = -MapConfig::map_width_m  * 0.5;
        const double oy = -MapConfig::map_height_m * 0.5;
        std::fprintf(f,
            "image: %s\n"
            "resolution: %.6f\n"
            "origin: [%.6f, %.6f, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.20\n",
            image_rel.c_str(), res, ox, oy);
        std::fclose(f);
        return true;
    }

    // --------------------------------------------------------
    // 栅格访问/坐标换算/统计 (测试与规划器用)
    // --------------------------------------------------------
    int  width()  const { return width_; }
    int  height() const { return height_; }
    double resolution() const { return MapConfig::grid_size_m; }

    bool is_occupied(int col, int row) const { return occ_state(col, row) == 100; }
    int occ_state(int col, int row) const {
        if (col < 0 || col >= width_ || row < 0 || row >= height_) return -1;
        const int l = grid_[idx_(col, row)];
        if (l >= L_OCC_TH) return 100;
        if (l <= L_FREE_TH) return 0;
        return -1;
    }
    int  raw_logodds(int col, int row) const {
        if (col < 0 || col >= width_ || row < 0 || row >= height_) return 0;
        return grid_[idx_(col, row)];
    }

    bool world_to_index(double wx, double wy, int& col, int& row) const {
        col = world_to_col_row_(wx);
        row = world_to_col_row_(wy);
        return col >= 0 && col < width_ && row >= 0 && row < height_;
    }
    void index_to_world(int col, int row, double& wx, double& wy) const {
        wx = col_row_to_world_(col);
        wy = col_row_to_world_(row);
    }

    std::string stats() const {
        int unk = 0, free = 0, occ = 0;
        for (size_t i = 0; i < grid_.size(); ++i) {
            const int s = occ_state(static_cast<int>(i % width_),
                                    static_cast<int>(i / width_));
            if (s == -1) ++unk; else if (s == 0) ++free; else ++occ;
        }
        char buf[160];
        std::snprintf(buf, sizeof(buf),
            "map %dx%d res=%.2fm unknown=%d free=%d occ=%d",
            width_, height_, MapConfig::grid_size_m, unk, free, occ);
        return std::string(buf);
    }

private:
    static constexpr int LOCC   = 6;
    static constexpr int LFREE  = 6;
    static constexpr int LMIN   = -20;
    static constexpr int LMAX   = +20;
    static constexpr int L_OCC_TH  = 4;
    static constexpr int L_FREE_TH = -4;

    static double c_(double x, double y, double t) { const double c = std::cos(t), s = std::sin(t); return c * x - s * y; }
    static double s_(double x, double y, double t) { const double c = std::cos(t), s = std::sin(t); return s * x + c * y; }
    static void camera_origin(const MappingPose& p, double& ox, double& oy) {
        ox = c_(0.12, 0.0, p.theta) + p.x;   // 相机外参 x=0.12 同源
        oy = s_(0.12, 0.0, p.theta) + p.y;
    }
    size_t idx_(int col, int row) const { return static_cast<size_t>(row) * width_ + col; }
    int    dim_cells_() const { return static_cast<int>(std::lround(MapConfig::map_width_m / MapConfig::grid_size_m)); }
    int    world_to_col_row_(double v) const { return static_cast<int>(std::lround(v / MapConfig::grid_size_m)) + dim_cells_() / 2; }
    double col_row_to_world_(int i) const { return (i - dim_cells_() / 2) * MapConfig::grid_size_m; }
    int    clamp_l_(int v) const { return v < LMIN ? LMIN : (v > LMAX ? LMAX : v); }

    int width_ = 0, height_ = 0;
    std::vector<int8_t>  grid_;
    std::vector<uint8_t> inflated_;
    double robot_radius_m_ = 0.25;
};

// ============================================================
// MappingPipeline — 单入口: 深度(base 点云)+位姿 → 地图文件
//
//   输入: base_link 系点云 (调用方先 transform_to_base)
//   内部: segment_ground 过滤地面 -> OccupancyMap.insert_cloud(仅障碍+负障碍)
//         -> 可选 inflate -> save_nav2_map(pgm+yaml)
// ============================================================
struct MappingPipeline {
    OccupancyMap map{0.25};
    GroundSegParams ground_params{};   // 可覆盖 (手持实验放宽 prior_window)
    bool inflate_before_save = true;

    // 单帧推进: 过滤地面后只投障碍/负障碍点
    void insert(const PointCloud& cloud_base, const MappingPose& pose) {
        GroundSegResult res;
        segment_ground(cloud_base, ground_params, res);

        // 只保留障碍 + 负障碍 (坑/台阶) 点; 地面点丢弃 → 消除假墙
        PointCloud obs;
        obs.frame_id = cloud_base.frame_id;
        obs.points.reserve(res.obstacle_indices.size() + res.negative_points.size());
        for (int i : res.obstacle_indices)
            obs.points.push_back(cloud_base.points[i]);
        for (const auto& np : res.negative_points)
            obs.points.push_back(np);

        map.insert_cloud(obs, pose);
    }

    // 落盘: 可选膨胀, 再导出 pgm+yaml
    bool save(const std::string& base_path) {
        if (inflate_before_save) map.inflate();
        return map.save_nav2_map(base_path);
    }
};

} // namespace mechdog
