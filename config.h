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
    static constexpr double  cliff_threshold_cm = 30.0;    // 底部悬崖判定阈值 (安装高度相关, 标定见 docs/IR_CALIBRATION.md 同类流程)
    // ALG-5 (v2.2): 超声失效哨兵常量。原代码散落 4.5 魔法数 (sensor_fusion.cpp 多处)。
    // 语义: > max_distance_cm(4.0m) 的失效兜底值, 表示"无有效读数"; L0/L1/L2/L3 分层据此判 ultra_valid。
    static constexpr double  kUltrasonicInvalidM = 4.5;
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

// ALG-10 (v2.2): 返回 const 引用 + 局部 static, 避免每次调用按值构造 unordered_map
inline const std::unordered_map<std::string, UltrasonicSensorEntry>& get_ultrasonic_layout() {
    static const std::unordered_map<std::string, UltrasonicSensorEntry> layout = {
        {"front_left",  {1, "左前",  -30.0, 0.0,  23, 24, "覆盖左前方盲区"}},
        {"front_center",{2, "正前",  0.0,   0.0,  17, 27, "检测正前方障碍物"}},
        {"front_right", {3, "右前",  30.0,  0.0,  5,  6,  "覆盖右前方盲区"}},
        {"bottom",      {4, "底部朝下",0.0, -90.0,13, 19, "检测地面悬崖/台阶（防跌落）"}},
    };
    return layout;
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

// ALG-10 (v2.2): 返回 const 引用 + 局部 static, 避免 fuse() 每次按值构造 unordered_map
inline const std::unordered_map<std::string, EnvWeights>& get_environment_weights() {
    static const std::unordered_map<std::string, EnvWeights> weights = {
        {"indoor",      {0.8, 0.2}},
        {"semi_indoor", {0.5, 0.5}},
        {"outdoor",     {0.1, 0.9}},
    };
    return weights;
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
};

// ============================================================
// 紧急避障阈值 (来自超声波)
// ============================================================
struct EmergencyConfig {
    static constexpr double  critical_dist_cm = 10.0; // 临界距离，强制停止
    static constexpr double  warning_dist_cm  = 25.0; // 警告距离，后退 (实现为 BACKWARD, 非减速)
    static constexpr double  safe_dist_cm     = 50.0; // 安全距离，正常行驶
};

// ============================================================
// 环境判定统一阈值 (ALG-3 v2.2: 深度代理与红外同源, 消除双轨)
// ============================================================
// TSL2591 已取消购买 (见 docs/FIX_PLAN.md F3 决策), 环境光强判定默认走
// estimate_ambient_light() 深度图代理; sensor_ir 仅作可选增强 (模拟模式可用)。
// 深度代理 (classify_environment) 与红外 (light_to_env) 共用此阈值, 避免双轨。
// 判定规则: level <= indoor_max  -> INDOOR      (Astra 权重高)
//           level >= outdoor_min -> OUTDOOR     (超声波主导)
//           之间                  -> SEMI_INDOOR (两者均衡)
struct EnvironmentThresholds {
    static constexpr double indoor_max  = 0.3;
    static constexpr double outdoor_min = 0.7;
};

// ============================================================
// 环境红外强度阈值 (TSL2591, 归一化 0.0~1.0) —— 已与深度代理同源
// ============================================================
// 说明: TSL2591 已取消购买, 此结构仅作可选增强 (未来接入真机 IR 时用)。
// 阈值已对齐 EnvironmentThresholds (0.3/0.7), 不再保留原 0.10/0.40 双轨;
// 若未来重启 TSL2591, 按 docs/IR_CALIBRATION.md 实测标定后改 EnvironmentThresholds 一处即可。
struct IrConfig {
    // ALG-3 (v2.2): 与 EnvironmentThresholds 同源 (TSL2591 取消后无需独立标定刻度)
    static constexpr double ir_indoor_max  = EnvironmentThresholds::indoor_max;   // 0.3
    static constexpr double ir_outdoor_min = EnvironmentThresholds::outdoor_min;  // 0.7
    static constexpr double ir_sim_min     = 0.05; // 模拟模式红外最小值 (参考, simulate_ir 实用 uniform(0,1))
    static constexpr double ir_sim_max     = 0.90; // 模拟模式红外最大值 (参考)
};

} // namespace mechdog
