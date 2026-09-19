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
 *   TooSteep     = 地面整体过陡 (拟合平面倾角 > HeightMap25Config::slope_max, 默认 20°) → 已测到的可通行格一并标禁行 (保守)
 *   Unknown      = 该格没被扫到 (无样本)
 *
 * 复用 P1: 不做自己的 RANSAC, 直接吃 GroundSegResult.plane (单平面)。
 * v1 边界: 单平面假设 (与 P1 一致); 不处理多层平台 (将来升级 elevation_mapping)。
 *   - TooSteep 是「整体拟合平面」级判断, **没有**逐格/相邻格坡度计算 (见 .cpp 同处注释);
 *   - 上游 segment_ground 只接受倾角 ≤ GroundSegConfig::plane_max_tilt_deg (默认 15°) 的平面,
 *     故本标签在生产路径下**不可达** (仅当外部构造的 seg 倾角 >20° 时才触发, 见 T6 单测);
 *     两个阈值的联动关系、"放宽前必须同批改造"的约束, 见 config.h 同名字段注释。
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

    // v2.8 视场楔形 (IMPL_PLAN_GRID_WEDGE_2026-09-18.md)
    //   动机: 0.18m 高相机水平半视场 ≈ ±29°(tan≈0.561), 而网格铺到 y±2.5m
    //         ⇒ 实机 4848 格里只有 65 格有数据, 其余"未知"纯属网格超出视场。
    //   默认 false ⇒ 判据/计数与历史完全一致 (零风险); 打开后额外按 in-FOV 口径统计。
    bool   wedge_only     = false;
    double wedge_y_slope  = 0.561;   // ≈ tan(水平半视场): 320px / fx570.34
    double wedge_margin_m = 0.15;    // 近场/装配余量 (yaw 未标定时避免切掉真实视野)
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

    // v2.8 in-FOV 口径 (wedge_only=false 时 count_in_fov == cols*rows)
    int    count_in_fov = 0;       // 视场楔形内的格数
    int    count_fov_unknown = 0;  // 其中仍未知的格
    double wedge_y_slope = 0.561;  // 从 cfg 复制, 供 in_fov() 使用
    double wedge_margin_m = 0.15;
    bool   in_fov(int col, int row) const;   // 该格是否在视场内 (可单测)
    double fov_coverage() const;             // (in_fov - fov_unknown) / in_fov

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

// ============================================================
// v2.9.3 深度质量守门 (方案 §4.3)
// ============================================================
// 动机(实机事故): 深度全 0 帧 (USB 重枚举/朝向错) 会造出 down=22~27 的假坑,
// 顶层据此 13/13 全 STOP —— 坏数据比没有数据更危险。策略: 质量不达标时
// 2.5D 判"未就绪"(调用方 abstain, 不把地形注入决策), 决策回落距离阶梯/超声。
struct DepthQualityConfig {
    // 依据: 实机对地帧 valid≈74%, 事故帧 = 0%; 0.25 留足余量(遮挡/低纹理场景不误伤)
    static constexpr double min_valid_ratio = 0.25;
    // 依据: 实机 640x480 下采样后约 2.5 万点; 300 只拦"帧可用但点数塌缩"
    static constexpr int    min_points      = 300;
};

/** 深度质量未就绪的原因 (日志/诊断用) */
enum class DepthQualityIssue {
    Ok = 0,
    NoValidPixels,   // 有效像素占比过低 (深度全 0 / 大面积失效)
    TooFewPoints,    // 点云数过少 (帧可用但点数塌缩)
};

/** 深度质量守门: true = 可用于 2.5D; false = 判"未就绪", 调用方应 abstain */
bool depth_quality_ok(double valid_ratio, int point_count, DepthQualityIssue& issue);

// ============================================================
// 走廊扫描 (近场地形避障用 —— 见 config.h TerrainAvoidConfig)
// ============================================================
/** 一次走廊扫描的聚合结果 */
struct CorridorScan {
    bool   blocked     = false; // 走廊内命中禁行地形
    int    count       = 0;     // 命中格/点数
    double nearest_x_m = 0.0;   // 最近命中处的 x (m; 无命中 = 0)
    double mean_y_m    = 0.0;   // 命中处平均 y (>0 偏左, <0 偏右; 选让开方向用)
};

/** 扫描 2.5D 栅格的走廊 x∈[x_lo,x_hi], |y|<=y_half (命中 = CliffDown/ObstacleUp/TooSteep) */
CorridorScan scan_corridor(const HeightMap25Result& hm,
                           double x_lo, double x_hi, double y_half);

/** 扫描点云/点集的走廊 (用于 P1 负障碍点: 2.5D 稀疏或无平面时仍能捕捉坑) */
CorridorScan scan_corridor_points(const std::vector<Point3D>& points,
                                  double x_lo, double x_hi, double y_half);

/** 合并两次扫描 (blocked 取或, count 相加, nearest 取小, mean_y 按 count 加权) */
CorridorScan merge_corridor(const CorridorScan& a, const CorridorScan& b);

} // namespace mechdog
