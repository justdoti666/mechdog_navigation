/**
 * 2.5D 高程/可通行地形 (P1.5) — 近场越障判断
 *
 * 定位: 在 P1 地面分割(单平面)基础上, 把 base_link 系点云组织成一张
 * 「每格一个高度」的 2.5D 栅格, 并给出每格的可通行性标签, 供越障决策使用。
 * 与 P1 的分工: P1 只输出地面平面/负障碍点(判断"有没有坑"), 本模块补全
 * 「正障碍(上行台阶/凸起) + 每格高度 + 完整可通行标签 + 可视化喂给」。
 *
 * 输出语义(每格):
 *   Traversable  = 能走 (与参考地面高度差在容忍内)
 *   ObstacleUp   = 凸起障碍 (高于地面超过可上台阶阈值, 上不去)
 *   CliffDown    = 沟/下行台阶 (低于地面超过可过沟阈值, 会跌落)
 *   TooSteep     = 坡度过陡 (相邻格高度差/坡度超阈值, 禁行)
 *   Unknown      = 该格没被扫到 (无样本)
 *
 * 复用 P1: 不做自己的 RANSAC, 直接吃 GroundSegResult.plane (单平面)。
 * v1 边界: 单平面假设 (与 P1 一致); 不处理多层平台 (将来升级 elevation_mapping)。
 *
 * 零依赖: 仅标准库 + point_cloud.h / ground_segmentation.h 类型。
 */
#pragma once

#include "config.h"
#include "point_cloud.h"
#include "ground_segmentation.h"

#include <cstdint>
#include <string>
#include <vector>

namespace mechdog {

/** 2.5D 每格可通行性标签 */
enum class CellFlag : int8_t {
    Unknown   = 0,  // 没扫到
    Traversable = 1, // 能走
    ObstacleUp = 2,  // 凸起障碍(上行台阶/物体)
    CliffDown  = 3,  // 沟/下行台阶(会跌落)
    TooSteep   = 4,  // 坡度过陡
};

inline const char* cell_flag_name(CellFlag f) {
    switch (f) {
        case CellFlag::Unknown:     return "Unknown";
        case CellFlag::Traversable: return "Traversable";
        case CellFlag::ObstacleUp:  return "ObstacleUp";
        case CellFlag::CliffDown:   return "CliffDown";
        case CellFlag::TooSteep:    return "TooSteep";
    }
    return "?";
}

/** 2.5D 地形运行参数 (阈值默认取保守值, 可按机械狗实测调) */
struct HeightMap25Config {
    // 栅格: x(前)∈[min_x, max_x], y(左右)∈[-y_half, +y_half]
    double cell_size = GroundSegConfig::cell_size;   // 0.05
    double min_x_m   = GroundSegConfig::neg_near_m;  // 0.6 (近界)
    double max_x_m   = GroundSegConfig::neg_far_m;   // 3.0 (远界)
    double y_half_m  = 2.5;

    // 可通行阈值 (保守默认, 按机械越障能力调)
    double step_up_max_m    = 0.10; // 可上台阶最大高度 (超出 = ObstacleUp)
    double drop_down_max_m  = 0.15; // 可过沟/下行台阶最大深度 (超出 = CliffDown)
    double slope_max        = 20.0; // 最大可靠坡度(度) (超出 = TooSteep)
    double surface_noise_m  = 0.05; // 地面不平度>此值判障碍 (滤噪)

    // 与 P1 一致的地面分类阈值
    double plane_eps_m = GroundSegConfig::point_on_plane_eps; // 0.02 视为地面
};

/** 2.5D 栅格结果 */
struct HeightMap25Result {
    bool   valid = false;
    int    cols = 0, rows = 0;
    double cell_size = 0.05;
    double min_x_m = 0.0, y_half_m = 0.0;

    std::vector<float>  height;  // 每格表面高度(m, 相对相机/原位姿的 z); NaN=无样本
    std::vector<CellFlag> flag;  // 每格可通行标签

    // 统计 (诊断输出)
    int count_unknown = 0, count_traversable = 0,
        count_up = 0, count_down = 0, count_steep = 0;

    // 世界(base 系) ↔ 栅格索引
    bool world_to_index(double wx, double wy, int& col, int& row) const;
    void index_to_world(int col, int row, double& wx, double& wy) const;

    std::string stats() const;
};

/**
 * 构建 2.5D 地形。
 * @param cloud_base  base_link 系点云 (调用方先 transform_to_base)
 * @param seg         P1 地面分割结果 (复用 plane; 需为同一点云的输出)
 * @param cfg         运行参数
 * @param out         2.5D 栅格结果
 *
 * 若 seg.plane 无效 (P1 fail-closed) → out.valid=false, 不输出 (保守).
 */
void build_heightmap_25(const PointCloud& cloud_base,
                        const GroundSegResult& seg,
                        const HeightMap25Config& cfg,
                        HeightMap25Result& out);

} // namespace mechdog
