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
#include "ground_segmentation.h"  // insert_cloud_filtered (FIX_PLAN #1)
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
    // ------------------------------------------------------------
    // 构造 (FIX_PLAN #6: 尺寸/分辨率运行时参数化, 不再编译期定死)。
    // cfg 为占位 (MapConfig 默认值供兼容), 实际用下方显式参数:
    //   width_m/height_m  地图物理尺寸 (默认 MapConfig 的 10x10m)
    //   resolution_m      栅格分辨率    (默认 MapConfig 的 5cm)
    // extrinsics_x_m: 相机原点 base 前偏 (CameraExtrinsics::x 同源, #7)
    // ------------------------------------------------------------
    explicit OccupancyGridMap(const MapConfig& cfg = MapConfig{},
                              double robot_radius_m = 0.25,
                              double extrinsics_x_m = CameraExtrinsics{}.x,
                              double width_m  = MapConfig::map_width_m,
                              double height_m = MapConfig::map_height_m,
                              double resolution_m = MapConfig::grid_size_m);

    // --------------------------------------------------------
    // 单帧更新: base 系点云 + 机器人位姿 → 变换到 odom 系,
    // 光线标记空闲 + 端点标记占据 (log-odds)。
    // 空点云/空位姿安全跳过 (不崩溃, 不清图)。
    // 越界点计入 dropped_points() (FIX_PLAN #6: 不再静默丢)。
    // --------------------------------------------------------
    void insert_cloud(const PointCloud& cloud_base,
                      const Pose2D& robot_pose);

    // --------------------------------------------------------
    // 地面过滤入口 (FIX_PLAN #1): 先 segment_ground 分离地面,
    // 障碍点走 insert_cloud (占据+光线), 负障碍点(坑/下行台阶)
    // 单独标记到 negative_ 层 (FIX_PLAN #9: 不参与光线打空,
    // 不算普通占据 — 坑是绕行目标, 语义独立)。
    // 地面点丢弃 → 消除"地面投影成假墙"。
    // --------------------------------------------------------
    void insert_cloud_filtered(const PointCloud& cloud_base,
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
    double resolution() const { return resolution_m_; }
    int cell_value(int idx) const { return grid_[idx]; } // 原始 log-odds(截断int)
    int occ_state(int idx) const;   // -1=未知 0=空闲 100=占据(归一化概率语义)
    int occ_state(int col, int row) const;

    // 负障碍标记层 (FIX_PLAN #9): 1=坑/下行台阶标记, 0=无
    int negative_state(int col, int row) const {
        if (col < 0 || col >= width_ || row < 0 || row >= height_) return 0;
        return negative_[static_cast<size_t>(row) * width_ + col];
    }

    // 越界丢弃点计数 (FIX_PLAN #6)
    long dropped_points() const { return dropped_points_; }

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
        return static_cast<int>(std::lround(v / resolution_m_))
               + width_ / 2;
    }
    double col_row_to_world_(int i) const {
        return (i - width_ / 2) * resolution_m_;
    }
    int clamp_l_(int v) const {
        return v < LMIN ? LMIN : (v > LMAX ? LMAX : v);
    }

    // raycast 空闲标记: 相机原点(位姿+外参平移) → 命中点
    void mark_free_along_ray_(const Pose2D& origin_pose,
                              double hit_x, double hit_y);

    int width_ = 0, height_ = 0;
    double resolution_m_ = MapConfig::grid_size_m;
    std::vector<int8_t> grid_;   // log-odds
    std::vector<uint8_t> inflated_; // 膨胀层 (0/1), inflate() 后有效
    std::vector<uint8_t> negative_; // 负障碍标记层 (FIX_PLAN #9)
    long dropped_points_ = 0;       // 越界丢弃计数 (FIX_PLAN #6)
    double robot_radius_m_ = 0.25;
    double extrinsics_x_m_ = 0.12; // 相机前偏 (与 CameraExtrinsics::x 同源, #7)
};

} // namespace mechdog
