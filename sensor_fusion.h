/**
 * 多传感器数据融合模块 (C++ 版)
 * 融合 Astra Pro 深度相机 + HC-SR04 超声波传感器阵列
 * 环境光强: 默认深度图代理 estimate_ambient_light() (TSL2591 已取消购买, 见 config.h F3 决策),
 *           sensor_ir 仅作可选增强 (模拟模式可用, 真机 I2C 分支待硬件)
 */
#pragma once

#include "config.h"
#include "sensor_ultrasonic.h"
#include "sensor_astra.h"
#include "sensor_ir.h"
#include "heightmap_2d5.h"   // 路1: HeightMap25Result / CellFlag / scan_corridor (近场地形避障)
#include <string>
#include <unordered_map>

namespace mechdog {

/** 障碍物危险等级 */
enum class ObstacleLevel {
    SAFE,
    WARNING,
    DANGER,
    CRITICAL
};

/** 导航动作指令 */
enum class NavigationAction {
    STOP,
    BACKWARD,
    TURN_LEFT,
    TURN_RIGHT,
    SLOW_FORWARD,
    FORWARD,
    REACHED_GOAL
};

/** 融合后的单个方向障碍物信息 */
struct FusedObstacle {
    std::string direction;
    double      distance_m          = 8.0;
    double      confidence          = 0.0;
    double      ultrasonic_dist_cm  = 400.0;
    double      astra_dist_m        = 8.0;
    std::string source;
    ObstacleLevel level            = ObstacleLevel::SAFE;
    // A1: 方向级失效标记。某个方向传感器双侧失效 (Astra 无有效像素 + 超声无效) 时置 false,
    // 该方向的 8.0m "假设无障碍" 兜底值不得参与方向决策 (否则会被当作最开阔盲区转向)。
    bool        valid              = true;
};

/** 融合结果 */
struct FusionResult {
    double timestamp                           = 0.0;
    std::unordered_map<std::string, FusedObstacle> obstacles;
    EnvironmentType environment                = EnvironmentType::UNKNOWN;
    bool astra_valid                           = false;
    bool cliff_detected                        = false;
    bool sensors_valid                         = true;   // M1: 任一传感器有有效数据 (全失效 -> false)
    double effective_astra_weight              = 0.0;
    double effective_ultrasonic_weight          = 0.0;
    NavigationAction recommended_action        = NavigationAction::FORWARD;
    double min_forward_distance_m              = 8.0;
    // S1 (N2): 深度降级期标记 (由胶水包按口径② streak 驱动; false = 接入前行为)
    bool   depth_degraded                      = false;
    // 路1: 近场地形避障 (P1/P1.5 → 决策)。未注入地形时三者恒为 false/false/0 ——
    // 与接入前逐位一致 (回归安全)。
    bool   terrain_block_near = false;  // 前方 0.4~1.2m 走廊内有坑/台阶 (STOP 级)
    bool   terrain_block_mid  = false;  // 前方 1.2~2.0m 有坑/台阶 (降速/让开级)
    double terrain_block_x_m  = 0.0;    // 最近命中处的 x (m; 0 = 无命中)
};

/**
 * 传感器融合核心类
 *
 * 使用方式:
 *   SensorFusion fusion(&astra_driver, &ultrasonic_driver, &ir_sensor);
 *   auto result = fusion.fuse();
 */
class SensorFusion {
public:
    SensorFusion(AstraProDriver* astra, UltrasonicArrayDriver* ultrasonic,
                 InfraRedSensor* ir);

    /** 执行一次传感器融合 */
    FusionResult fuse();

    /**
     * 路1: 注入近场地形 (P1 地面分割 + P1.5 2.5D, 需为同帧输出)。
     * 每帧调用一次; 不调用 (或调用 clear_local_terrain) 时决策完全不受影响 (= 接入前行为)。
     * @param hm  2.5D 结果 (hm.valid=false 时只看 seg.negative_points)
     * @param seg 地面分割结果 (用其 negative_points 作为坑的第二来源)
     */
    void set_local_terrain(const HeightMap25Result& hm, const GroundSegResult& seg);

    /** 路1: 清除近场地形 (退出点云分支/无地形数据时调用) */
    void clear_local_terrain();

    /**
     * v2.5: 显式启用/禁用**超声链路** (默认启用 = 接入前行为, 零行为变化)。
     *
     * 为什么需要它: 真机上若超声无硬件/无数据源, 驱动会回落**模拟随机数**并带
     * valid=true 混进融合 (模拟器还有"底部 5% 概率造悬崖"), 而 bottom 的 fail-closed
     * 又会让 is_fall_risk() 恒 true → 结果要么被随机数带偏, 要么永远 STOP。
     * 禁用后: 超声不参与融合, 也不作悬崖判定 —— 由上层负责打印醒目警告,
     * 并在硬件/话题恢复后重新启用。
     */
    void set_ultrasonic_enabled(bool on) { ultrasonic_enabled_ = on; }
    bool ultrasonic_enabled() const { return ultrasonic_enabled_; }

    /**
     * S1 (N2): 深度降级期开关 (由胶水包按口径② streak 状态驱动, 与超声开关同为安全链路开关)。
     * on=true: 决策侧三级反应线收紧 (DegradedPolicyConfig), planner 侧前进限速 ≤SLOW。
     * 恢复由调用方置回 false; 默认 false ⇒ 与接入前行为逐位一致 (回归安全)。
     */
    void set_depth_degraded(bool on) { depth_degraded_ = on; }
    bool depth_degraded() const { return depth_degraded_; }

private:
    // 单元测试访问 (tests/test_fusion.cpp 专用, R-3: 测试调用真函数而非复刻逻辑)
    friend class SensorFusionTestAccess;

    AstraProDriver* astra_;
    UltrasonicArrayDriver* ultrasonic_;
    InfraRedSensor* ir_;

    // 路1: 近场地形状态 (由 set_local_terrain 刷新, 供 determine_action 读取)
    bool   terrain_near_      = false; // 近场走廊 (0.4~1.2m) 命中禁行地形
    bool   terrain_mid_       = false; // 中距 (1.2~2.0m) 命中禁行地形
    // v2.5: 超声链路开关 (false = 不读超声/不作悬崖判定; 上层在无硬件或无数据源时置 false)
    bool   ultrasonic_enabled_ = true;
    // S1 (N2): 降级链开关 (由 set_depth_degraded 驱动)
    bool   depth_degraded_ = false;
    double terrain_x_         = 0.0;   // 最近命中处 x (诊断)
    // ---- v2.9 路1 细分 (按标签分级) ----
    // 证据: 实机 181 帧 STOP 124 / FORWARD 57 ⇒ "近场凸起一律 STOP" 被证伪 (见 config.h
    // TerrainAvoidConfig::bump_* 的说明)。分级口径:
    //   CliffDown / TooSteep (坑、过陡)      -> STOP            (任何距离)
    //   ObstacleUp (凸起) + 贴身 + 正中      -> STOP            (确认过不去)
    //   ObstacleUp 偏一侧                    -> 转向对侧让开
    //   ObstacleUp 正中但未贴身 / 仅中距命中 -> 至少降速
    bool   terrain_near_cliff_ = false; // 近场命中"坑/过陡" (STOP 级)
    bool   terrain_near_bump_  = false; // 近场命中"凸起" (降速/让开级)
    double terrain_near_y_     = 0.0;   // 近场凸起命中处平均 y (>0 偏左, <0 偏右)
    double terrain_near_bump_x_ = 0.0;  // 近场凸起最近命中 x (m)
    double terrain_mid_side_  = 0.0;   // 中距命中的平均 y (>0 偏左, <0 偏右)
    int    terrain_mid_count_ = 0;     // 中距命中数

    // v2.9 路1 细分策略 (纯函数 ⇒ 可单测)。语义:
    //   返回 FORWARD       = "路1 不表态", 调用方应继续走后面的距离阶梯
    //   返回其它动作        = 路1 直接裁定 (STOP / SLOW_FORWARD / TURN_*)
    static NavigationAction terrain_action(bool near_any, bool near_cliff, bool near_bump,
                                           double bump_y_m, double bump_x_m, bool mid);

    static constexpr double kCmToM = 0.01;

    EnvironmentType determine_environment(const AstraFrame& frame);
    // ALG-3 (v2.2): 归一化光强 -> 环境档位 (IR 与深度代理同源阈值, 见 EnvironmentThresholds)
    static EnvironmentType light_to_env(double light);
    std::pair<double, double> get_adaptive_weights(EnvironmentType env_type,
                                                    const AstraFrame& frame);
    std::string env_to_key(EnvironmentType env_type) const;

    FusedObstacle fuse_direction(const std::string& direction,
                                  const AstraFrame& astra_frame,
                                  const UltrasonicArrayData& ultra_data,
                                  double astra_w, double ultra_w);

    const UltrasonicReading* get_ultrasonic_reading(
        const UltrasonicArrayData& ultra_data, const std::string& direction);

    std::pair<double, std::string> layer_fusion(
        double ultra_m, double astra_m, bool astra_valid,
        double astra_w, double ultra_w);

    FusedObstacle build_bottom_obstacle(const UltrasonicReading& bottom,
                                        bool cliff_detected);

    double calc_confidence(double ultra_m, double astra_m, bool astra_valid);

    ObstacleLevel classify_obstacle_level(double distance_m);
    // R3: front_valid —— 前向 (left/center/right) 是否有任一有效方向。
    // 旧判定把 bottom 计入有效性: 底部超声有效 + 前向全盲时 sensors_valid=true,
    // min_forward=8.0 兜底 -> 直接 FORWARD (fail-open)。前向失明必须保守, bottom 无效力。
    NavigationAction determine_action(double min_forward_m,
                                       double min_ultrasonic_cm,
                                       bool cliff_detected,
                                       const std::unordered_map<std::string, FusedObstacle>& obstacles,
                                       bool sensors_valid,
                                       bool front_valid);
    NavigationAction choose_direction(
        const std::unordered_map<std::string, FusedObstacle>& obstacles);

    // M1: 全部传感器均无有效数据 (fail-closed 判定, 独立可测)
    static bool all_sensors_invalid(const AstraFrame& astra_frame,
                                    const UltrasonicArrayData& ultra_data);
};

} // namespace mechdog
