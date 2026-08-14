/**
 * 机械狗寻路系统 - 全局配置 (C++ 版)
 * 基于 Astra Pro 深度相机 + HC-SR04 超声波传感器融合
 */
#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace mechdog {

// ============================================================
// Astra Pro 深度相机参数 (从官方规格表识别)
// ============================================================
struct AstraProConfig {
    static constexpr const char* depth_tech   = "单目结构光（红外投影）";
    static constexpr double  min_distance_m   = 0.6;    // 最小探测距离 (盲区起点)
    static constexpr double  max_distance_m   = 8.0;    // 最大探测距离
    static constexpr double  depth_fov_h      = 58.4;   // 深度水平视场角 (度)
    static constexpr double  depth_fov_v      = 45.5;   // 深度垂直视场角 (度)
    static constexpr double  rgb_fov_h        = 66.1;   // RGB水平视场角 (度)
    static constexpr double  rgb_fov_v        = 40.2;   // RGB垂直视场角 (度)
    static constexpr double  depth_accuracy_at_1m_mm = 3.0; // 1m处精度 ±3mm
    static constexpr const char* environment  = "室内";  // 推荐使用环境
    static constexpr const char* laser_safety = "Class 1";
    static constexpr int32_t  depth_width     = 640;
    static constexpr int32_t  depth_height    = 480;
    static constexpr int32_t  depth_fps       = 30;
    static constexpr uint16_t min_valid_mm    = 600;    // 0.6m 最小有效距离(毫米)
    static constexpr uint16_t max_valid_mm    = 8000;   // 8m 最大有效距离(毫米)
};

// ============================================================
// HC-SR04 超声波传感器参数
// ============================================================
struct UltrasonicConfig {
    static constexpr const char* model          = "HC-SR04";
    static constexpr const char* type           = "超声波测距模块（干燥环境）";
    static constexpr double  min_distance_cm    = 2.0;   // 最小探测距离 2cm
    static constexpr double  max_distance_cm    = 400.0; // 最大探测距离 4.0m
    static constexpr double  accuracy_cm        = 0.3;   // 精度 ±3mm
    static constexpr double  beam_angle_deg     = 75.0;  // 波束角
    static constexpr int32_t working_freq_hz    = 40000; // 工作频率 40kHz
    static constexpr const char* interface      = "GPIO (Trig/Echo)";
    static constexpr double  voltage_v          = 5.0;
    static constexpr double  speed_of_sound     = 34300.0; // 声速 cm/s at 20°C
    static constexpr double  timeout_sec        = 0.025;   // 超时 ~4m
    static constexpr double  min_interval_sec   = 0.05;    // 最小测量间隔 50ms
};

// ============================================================
// 超声波传感器布置方案
// ============================================================
struct UltrasonicSensorEntry {
    int32_t id;
    std::string position;
    double    yaw_offset_deg;
    double    pitch_offset_deg;
    int32_t   trig_pin;
    int32_t   echo_pin;
    std::string description;
};

inline std::unordered_map<std::string, UltrasonicSensorEntry> get_ultrasonic_layout() {
    return {
        {"front_left",  {1, "左前",  -30.0, 0.0,  23, 24, "覆盖左前方盲区"}},
        {"front_center",{2, "正前",  0.0,   0.0,  17, 27, "检测正前方障碍物"}},
        {"front_right", {3, "右前",  30.0,  0.0,  5,  6,  "覆盖右前方盲区"}},
        {"bottom",      {4, "底部朝下",0.0, -90.0,13, 19, "检测地面悬崖/台阶（防跌落）"}},
    };
}

// ============================================================
// 传感器融合距离分层策略
// ============================================================
// 分层逻辑实现在 sensor_fusion.cpp::layer_fusion() (L0 超声盲区 / L1 保守取min / L2 加权 / L3 远距)。
// 原 config.h 中的 FusionLayer / FusionLayerConfig / get_fusion_layers() 死代码已删除 (FIX-9):
// 其 L1 权重(0.6/0.4)与实现"取min"策略矛盾, 且无任何调用方, 保留只会误导。

// ============================================================
// 环境自适应权重
// ============================================================
struct EnvWeights {
    double astra;
    double ultrasonic;
};

inline std::unordered_map<std::string, EnvWeights> get_environment_weights() {
    return {
        {"indoor",      {0.8, 0.2}},
        {"semi_indoor", {0.5, 0.5}},
        {"outdoor",     {0.1, 0.9}},
    };
}

// ============================================================
// 障碍物地图参数
// ============================================================
struct MapConfig {
    static constexpr double  grid_size_m        = 0.05; // 栅格分辨率 5cm
    static constexpr double  map_width_m        = 10.0; // 地图宽度
    static constexpr double  map_height_m       = 10.0; // 地图高度
    static constexpr double  inflation_radius_m = 0.15; // 障碍物膨胀半径
    static constexpr double  unknown_threshold   = 0.3; // 未知区域阈值
    static constexpr double  obstacle_threshold  = 0.7; // 障碍物阈值
};

// ============================================================
// 路径规划参数
// ============================================================
struct PlannerConfig {
    static constexpr const char* algorithm          = "DWA";
    // FIX-8: 与底盘实际能力一致 (Stm32ChassisBridge 限幅 ±0.20 m/s / ±0.60 rad/s,
    // 师兄 wheel_board_bridge_node 层2 同值)
    static constexpr double  max_linear_velocity    = 0.20; // 最大线速度 m/s
    static constexpr double  max_angular_velocity   = 0.60; // 最大角速度 rad/s
    static constexpr double  linear_accel           = 0.3;  // 线加速度 m/s²
    static constexpr double  angular_accel          = 0.5;  // 角加速度 rad/s²
    static constexpr double  goal_tolerance_m       = 0.1;  // 到达目标点容差
    static constexpr double  obstacle_safety_dist_m = 0.3;  // 障碍物安全距离
    static constexpr double  emergency_stop_dist_m  = 0.1;  // 紧急停止距离
};

// ============================================================
// 紧急避障阈值 (来自超声波)
// ============================================================
struct EmergencyConfig {
    static constexpr double  critical_dist_cm = 10.0; // 临界距离，强制停止
    static constexpr double  warning_dist_cm  = 25.0; // 警告距离，减速
    static constexpr double  safe_dist_cm     = 50.0; // 安全距离，正常行驶
};

// ============================================================
// 环境红外强度阈值 (来自 TSL2591, 归一化 0.0~1.0)
// ============================================================
// 说明: 以下为软件组预设的保守默认值, 未经过实测标定。
// 硬件组请按 docs/IR_CALIBRATION.md 标定后修改这两个数字, 代码逻辑无需改动。
// 判定规则: light <= ir_indoor_max   -> indoor      (Astra 权重高)
//           light >= ir_outdoor_min  -> outdoor     (超声波主导)
//           之间                       -> semi_indoor (两者均衡)
struct IrConfig {
    static constexpr double ir_indoor_max  = 0.30; // 归一化红外强度上限: 室内
    static constexpr double ir_outdoor_min = 0.70; // 归一化红外强度下限: 室外/强光
    static constexpr double ir_sim_min     = 0.05; // 模拟模式红外最小值
    static constexpr double ir_sim_max     = 0.90; // 模拟模式红外最大值
};

} // namespace mechdog
