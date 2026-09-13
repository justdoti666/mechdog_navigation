/**
 * 地面分割与负障碍检测 (P1)
 *
 * 定位: 近场 3D 感知的核心差异化能力 —— 2D 激光雷达扫不到负障碍(坑/下行台阶),
 * 底部 HC-SR04 (ALG-1) 只能在狗到达边缘时兜底; 本模块用前瞻点云提前 0.6~3.0m
 * 标出负障碍, 让规划层能"绕行"而不是"踩空瞬间急停".
 *
 * 三级管线 (全部在 base_link 系, X前 Y左 Z上):
 *   ① 受约束 RANSAC 拟合地面平面: 高度先验带 + 法向竖直约束,
 *      防止室内场景把墙/桌面拟合成"地面"(墙的可见点常比地板多).
 *   ② 逐点分类: 距平面 < point_on_plane_eps 为地面, 否则为障碍.
 *   ③ 2.5D 栅格按列(由近及远)扫描: 参考地面 → 无地面/下沉带 → 更低回波 判负障碍.
 *      门口/Free space 后方地面同高 → 不标 (试金石, tests T4).
 *
 * fail-closed: 平面拟合失败 (无地面/全看墙) → 不输出负障碍, 只输出障碍点,
 * 底部 HC-SR04 兜底不受影响. 上坡地形(地面点高于先验带)按障碍处理, 保守.
 *
 * v1 边界: 单平面(不处理多层平台); 假定机身大致水平(IMU 降姿态等遥测落地后,
 * 以 pitch/roll 入参形式扩展); 对角方向坑沿靠逐列扫描阶梯式捕获, 边缘量化 ±1 cell.
 */
#pragma once

#include "config.h"
#include "point_cloud.h"

#include <vector>

namespace mechdog {

/** 运行期参数 (默认取 GroundSegConfig; 手持实验放宽 prior_window ~1.0) */
struct GroundSegParams {
    double ground_prior_z     = GroundSegConfig::ground_prior_z;
    double prior_window       = GroundSegConfig::prior_window;
    double plane_max_tilt_deg = GroundSegConfig::plane_max_tilt_deg;
    double ransac_inlier_dist = GroundSegConfig::ransac_inlier_dist;
    int    ransac_max_iters   = GroundSegConfig::ransac_max_iters;
    double ransac_early_ratio = GroundSegConfig::ransac_early_ratio;
    double point_on_plane_eps = GroundSegConfig::point_on_plane_eps;
    double cell_size          = GroundSegConfig::cell_size;
    double cliff_drop_min     = GroundSegConfig::cliff_drop_min;
    double neg_near_m         = GroundSegConfig::neg_near_m;
    double neg_far_m          = GroundSegConfig::neg_far_m;
    int    min_gap_cells      = GroundSegConfig::min_gap_cells;
    unsigned seed             = 20260830u;  // 固定默认种子: 单测/复现确定性
    // v2.7: 地面提取方法开关。false(默认) = 受约束 RANSAC (行为与历史逐字节一致);
    //   true = 先用确定性"格最小拟合"(见 fit_ground_plane_cells), 失败才回退 RANSAC。
    //   动因: RANSAC 从 1.2m 高候选带盲抽三点却要求 h0 落在 ±prior_window ⇒ 命中靠运气
    //   (真机实测: step=8 能拟合、step=1 反而找不到; 窗口 0.10↔0.15 结果翻车)。
    bool     use_cell_min_fit  = false;
};

/** 拟合出的地面平面: nx*x + ny*y + nz*z + d = 0, 单位法向且 nz > 0 (指向天空) */
struct GroundPlane {
    bool   valid = false;
    double nx = 0.0, ny = 0.0, nz = 1.0, d = 0.0;
    int    inliers = 0;
    /** x=y=0 处的平面高度 (|nz| >= cos(max_tilt), 无除零风险) */
    double height_at_origin() const { return nz != 0.0 ? -d / nz : 0.0; }
};

/** 分割结果 (索引对应输入点云; 负障碍点位于地面平面高度) */
struct GroundSegResult {
    GroundPlane plane;
    std::vector<int> ground_indices;
    std::vector<int> obstacle_indices;
    std::vector<Point3D> negative_points;  // base_link 系, z = 该 cell 的平面高度
};

/**
 * 入口. 输入必须已是 base_link 系点云 (调用方先 transform_to_base).
 * 确定性: 固定 seed 下结果可复现; O(ransac_iters * n + cells).
 */
/**
 * v2.7: 确定性地面提取 —— "按 (x,y) 小格取最低表面 → 两轮稳健最小二乘拟合"。
 *
 * 为什么需要它 (真机实测 2026-09-13, 相机俯视地板, 帧6):
 *   受约束 RANSAC 从 1.2m 高的候选带里随机抽三点, 却要求 h0 落在 ±prior_window 内
 *   ⇒ 命中靠运气: 同帧 step=8 能拟合而 step=1 (更稠密) 反而"找不到平面"; 窗口
 *   0.10↔0.15 结果翻车; 拟合出的平面常偏高/偏斜 (tilt 7~15°), 于是地板点被全判
 *   "凸起"、2.5D traversable 恒 0。
 *
 * 本方法无随机数: 相机俯视时, 每个 (x,y) 小格里**最低的那层表面就是地板**
 * (桌上物体/椅子腿都在其上)。对"格最低值"做最小二乘 → 按残差剔除离群 → 再拟合一次。
 * 实测同一帧: 稳定得到 841 格、地板倾角 13.34°、原点高度 -0.702m (与"相机离地"吻合)。
 *
 * @param p   用 cell_size / plane_max_tilt_deg / ground_prior_z / prior_window /
 *            ransac_inlier_dist / point_on_plane_eps
 * @return true = 得到合格平面 (写入 out, nz>0, 已归一化); false = 样本不足 /
 *         倾角超限 / 高度先验不合格 ⇒ **调用方必须回退 RANSAC**
 */
bool fit_ground_plane_cells(const PointCloud& cloud, const GroundSegParams& p,
                            GroundPlane& out);

void segment_ground(const PointCloud& cloud_base,
                    const GroundSegParams& params,
                    GroundSegResult& out);

} // namespace mechdog
