/**
 * 点云模块 (P0) — 深度图反投影 + 坐标变换
 *
 * 设计文档: docs/POINT_CLOUD_DESIGN.md (§5 反投影 / §6 坐标变换 / §13 验证)
 *
 * 本模块独立于 SensorFusion, 不破坏现有三区域 fail-closed 融合语义 (§10 互补不替代)。
 * 零依赖: 仅标准库, 可模拟单测, 不引 PCL。
 *
 * 坐标约定 (与 ROS REP-103 一致):
 *   camera_optical: X右, Y下, Z前 (标准针孔光学系, Astra SDK 出帧系)
 *   camera_link:    X前, Y左, Z上 (与 optical 差固定旋转 Rz(-90°)·Rx(-90°), §6.1)
 *   base_link:      X前, Y左, Z上 (底盘几何中心)
 *
 * P0 范围: 类型 + depth_to_cloud + transform_optical_to_link + transform_to_base
 * 地面分割/聚类/占据栅格留待 P1/P2。
 */
#pragma once

#include "config.h"
#include <cstdint>
#include <string>
#include <vector>

namespace mechdog {

// ============================================================
// 3D 点 (光学系或 base 系, 取决于所属 PointCloud 的 frame_id)
// ============================================================
struct Point3D {
    double x = 0.0, y = 0.0, z = 0.0;
    uint8_t r = 0, g = 0, b = 0;  // 可选 RGB 上色 (真机模式, 默认 0)
};

// ============================================================
// 点云容器
// ============================================================
struct PointCloud {
    uint64_t seq = 0;
    double   stamp = 0.0;
    std::string frame_id = "camera_optical";
    std::vector<Point3D> points;
};

// ============================================================
// 相机内参 (针孔模型)
// fx/fy/cx/cy 默认值由 FOV 反推 (§3.1), 真机用 SDK intrinsics() 覆盖
// min/max_depth_m 引用 AstraProConfig, 保持单一真相源 (不硬编码 0.6/8.0)
// ============================================================
struct CameraIntrinsics {
    double fx = 572.7, fy = 572.3, cx = 320.0, cy = 240.0;  // FOV 反推初值 (§3.1)
    double min_depth_m = AstraProConfig::min_valid_mm / 1000.0;  // 0.6 (与 analyze_region 同口径)
    double max_depth_m = AstraProConfig::max_valid_mm / 1000.0;  // 8.0
};

// ============================================================
// 相机外参 (相对 base_link 的安装位姿)
//
// pitch 符号约定 (§4, 与 §2 link 系 {X前,Y左,Z上} + ZYX 内旋一致):
//   前向轴 [1,0,0] 经 R 后 z 分量 = -sin(pitch)。
//   光轴朝下 (俯视) 需 pitch > 0, 故前倾 15° 取 +15°。
// ============================================================
struct CameraExtrinsics {
    double x = 0.12;    // 相机相对底盘中心: 前 12cm (待量测)
    double y = 0.0;     // 左右居中
    double z = 0.18;    // 离地高 18cm (待量测)
    double roll  = 0.0;
    // 弧度: 15° × π/180。用常量避免 M_PI 依赖 (MSVC 不默认定义 M_PI)
    double pitch = +15.0 * 0.01745329251994329576;  // +15° 前俯 (待量测/手眼标定)
    double yaw   = 0.0;
};

// ============================================================
// 深度图(mm) → 点云(camera_optical 系, 米)
//
// 无效像素 (px==0 或 d<min_depth_m 或 d>max_depth_m) 丢弃,
// 与 analyze_region 同一有效性口径 (§5.2)。
// 空输入 (depth==nullptr 或 w<=0 或 h<=0) 返回空点云, 不崩溃。
// ============================================================
void depth_to_cloud(const uint16_t* depth, int w, int h,
                    const CameraIntrinsics& K, PointCloud& out);

// 全帧有效深度像素计数 (600~8000mm 口径与 depth_to_cloud 一致; 可视化空态诊断数据源)
size_t count_valid_pixels(const std::vector<uint16_t>& depth_map);

// 点云空态标签 (可视化状态栏): 0=无帧 1=深度全无有效像素 2=正常
const char* cloud_state_label(int state);

// ============================================================
// camera_optical → camera_link 固定旋转 (§6.1)
//
// 矩阵 [[0,0,1],[-1,0,0],[0,-1,0]] (R = Rz(-90°)·Rx(-90°)):
//   X_link =  Z_opt  (前 ← 前)
//   Y_link = -X_opt  (左 ← 右反)
//   Z_link = -Y_opt  (上 ← 下反)
//
// 此旋转是相机坐标系的固定属性, 与安装无关。
// ============================================================
void transform_optical_to_link(const PointCloud& in, PointCloud& out);

// ============================================================
// camera_optical → base_link 全链路 (§6.1 + §6.2)
//
// 先做 optical→link 固定旋转, 再叠加 CameraExtrinsics 的
// ZYX 欧拉旋转 (R = Rz(yaw)·Ry(pitch)·Rx(roll)) + 平移:
//   p_base = R_extrinsic · p_link + t_extrinsic
//
// 输入应为 depth_to_cloud 的输出 (frame_id="camera_optical")。
// ============================================================
void transform_to_base(const PointCloud& in, const CameraExtrinsics& E,
                       PointCloud& out);
} // namespace mechdog
