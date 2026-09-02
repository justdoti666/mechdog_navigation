/**
 * 建图模块 (P4) — 位姿驱动点云累积 → 占据栅格
 *
 * 设计文档: docs/POINT_CLOUD_DESIGN.md (§6.3 base→odom / §220 投影 / P4)
 *
 * 上游: point_cloud.h 的 base 系点云 (transform_to_base 输出)
 * 下游: DWA / global planner 消费 OccupancyGrid
 *
 * 本模块零依赖 (仅标准库), 不依赖 ROS; ROS 胶水层把 nav_msgs/Odometry
 * 的位姿查出来后以 Pose2D 喂入。frame 语义:
 *   - 输入点云 frame_id 应为 "base_link" (transform_to_base 输出)
 *   - 位姿 pose 为 odom系下 base_link 的位姿 (SLAM/里程计输出)
 *   - 输出地图坐标原点 = odom 系原点 (即建图开始时机器人所在位置)
 *
 * P4 范围: 单帧投影 + 光线空闲标记 + log-odds 更新 + 膨胀 + PGM 导出。
 * 回环校正 / map→odom 修正 / octomap 留待后续阶段。
 */
#pragma once

#include "config.h"
#include "point_cloud.h"
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace mechdog {

// ============================================================
// 2D 位姿 (平移+航向)。来自 ROS 层对 Odometry.pose 的提取,
// 或测试直接构造。
// ============================================================
struct Pose2D {
    double x = 0.0, y = 0.0, theta = 0.0;
};

// ============================================================
// 光线投射参数 (空闲区域标记)
// ============================================================
struct RaycastConfig {
    // 光线追踪最大距离 (米)。超过此距离的观测不做空闲标记,
    // 防止远距离深度噪声把真实占据误标为空闲 (fail-safe)。
    static constexpr double max_free_range_m = 5.0;
    // 沿光线步进步长 (米), ≈ grid_size 的一半保证穿栅格
    double step_m = MapConfig::grid_size_m * 0.5;
};

// ============================================================
// log-odds 占据栅格地图
//
// 值域 [-l_max, +l_max], 正=占据, 负=空闲, 0=未知。
// hit: l += l_hit;  miss: l -= l_miss;  阈值 l_occ/l_free 判定。
// 参数取 Cartographer 默认量级。
// ============================================================
class OccupancyGridMap {
public:
    // extrinsics_x_m: 相机原点在 base 前方的偏移 (与
    // CameraExtrinsics::x 同源, FIX_PLAN #7 — 不再硬编码 0.12)。
    // 传 CameraExtrinsics{}.x 即可保持与 transform_to_base 一致。
    explicit OccupancyGridMap(double robot_radius_m = 0.25,
                              double extrinsics_x_m = CameraExtrinsics{}.x);

    // --------------------------------------------------------
    // 单帧更新: base 系点云 + 机器人位姿 → 变换到 odom 系,
    // 光线标记空闲 + 端点标记占据 (log-odds)。
    // 空点云/空位姿安全跳过 (不崩溃, 不清图)。
    // --------------------------------------------------------
    void insert_cloud(const PointCloud& cloud_base,
                      const Pose2D& robot_pose);

    // --------------------------------------------------------
    // 障碍膨胀 (供规划器消费前的后处理): 对每个占据格,
    // 以 inflation_radius_m 为半径标记周围格为不可通行。
    // 本实现采用“占据格直接置 INSCRIBED, 邻域按距离衰减”的
    // 简化 costmap 语义; 输出保留 log-odds 原图 + 膨胀层。
    // --------------------------------------------------------
    void inflate(double radius_m = MapConfig::inflation_radius_m);

    // --------------------------------------------------------
    // 导出 PGM (P2 格式) + 世界坐标换算辅助。
    // 返回 false = 文件不可写。
    // 注: P2 为调试/人眼可读格式; nav2 map_server 请用 save_nav2_map。
    // --------------------------------------------------------
    bool save_pgm(const std::string& path) const;

    // --------------------------------------------------------
    // nav2 map_server 地图导出 (FIX_PLAN #3): P5 二进制 PGM + 配对
    // map.yaml, 一次调用出两个文件 (base_path 自动补 .pgm/.yaml)。
    // yaml: image/resolution/origin/negate/occupied_thresh/free_thresh,
    // origin = 地图左下角世界坐标 (原点居中 → -W/2, -H/2)。
    // 返回 false = 任一文件不可写。
    // --------------------------------------------------------
    bool save_nav2_map(const std::string& base_path) const;

    // --------------------------------------------------------
    // 栅格访问 (测试/规划器用)
    // idx = row * width + col;  col↔x, row↔y
    // --------------------------------------------------------
    int width()  const { return width_; }
    int height() const { return height_; }
    double resolution() const { return MapConfig::grid_size_m; }
    int cell_value(int idx) const { return grid_[idx]; } // 原始 log-odds(截断int)
    int occ_state(int idx) const;   // -1=未知 0=空闲 100=占据(归一化概率语义)
    int occ_state(int col, int row) const;

    // 坐标换算: 世界(odom 米) ↔ 栅格索引 (const, 无越界写)
    bool world_to_index(double wx, double wy, int& col, int& row) const;
    void index_to_world(int col, int row, double& wx, double& wy) const;

    // 地图统计 (诊断输出)
    int count_cells(int state) const; // state: -1/0/100
    std::string stats() const;

private:
    // log-odds 截断参数
    static constexpr int LOCC = 6;   // 单次 hit 增量上限
    static constexpr int LFREE = 6;  // 单次 miss 减量上限
    static constexpr int LMIN = -20; // log-odds 下限
    static constexpr int LMAX = +20; // log-oddss 上限
    static constexpr int L_OCC_TH = 4;   // 判占据阈值
    static constexpr int L_FREE_TH = -4; // 判空闲阈值

    int world_to_col_row_(double v) const {  // 米→格, 原点居中
        return static_cast<int>(std::lround(v / MapConfig::grid_size_m))
               + dim_cells_() / 2;
    }
    double col_row_to_world_(int i) const {
        return (i - dim_cells_() / 2) * MapConfig::grid_size_m;
    }
    int dim_cells_() const {
        return static_cast<int>(std::lround(
            MapConfig::map_width_m / MapConfig::grid_size_m));
    }
    int clamp_l_(int v) const {
        return v < LMIN ? LMIN : (v > LMAX ? LMAX : v);
    }

    // raycast 空闲标记: 相机原点(位姿+外参平移) → 命中点
    void mark_free_along_ray_(const Pose2D& origin_pose,
                              double hit_x, double hit_y);

    int width_ = 0, height_ = 0;
    std::vector<int8_t> grid_;   // log-odds
    std::vector<uint8_t> inflated_; // 膨胀层 (0/1), inflate() 后有效
    double robot_radius_m_ = 0.25;
    double extrinsics_x_m_ = 0.12; // 相机前偏 (与 CameraExtrinsics::x 同源, #7)
};

} // namespace mechdog
