/**
 * 单元测试 (轻量断言, 无第三方框架)
 *
 * 覆盖:
 *   - get_cliff_detected() 无效读数必须判为有风险 (回归 F4)
 *   - get_min_forward_distance_cm() 过滤无效读数 (回归 F5)
 *   - layer_fusion() 分层边界
 *   - calc_confidence() 一致性
 *
 * 构建/运行:
 *   g++ -std=c++20 test_fusion.cpp sensor_ultrasonic.cpp sensor_astra.cpp sensor_ir.cpp sensor_fusion.cpp -lpthread -o test_fusion
 *   ./test_fusion
 */
#include "../sensor_fusion.h"
#include "../sensor_ultrasonic.h"
#include "../sensor_astra.h"
#include "../sensor_ir.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <set>
#include <thread>

using namespace mechdog;

static int g_passed = 0;
static int g_failed = 0;

#define CHECK(cond) do { \
    if (cond) { ++g_passed; } \
    else { ++g_failed; std::cerr << "FAIL: " << #cond << " @ " << __LINE__ << std::endl; } \
} while (0)

static void test_cliff_valid_check() {
    // F4: 底部读数无效(超时 -1.0 或任意值)必须判为有风险
    UltrasonicArrayData data;
    data.bottom.valid = false;
    data.bottom.distance_cm = -1.0;
    CHECK(data.get_cliff_detected() == true);   // 无效 -> 有风险

    data.bottom.valid = false;
    data.bottom.distance_cm = 5.0;
    CHECK(data.get_cliff_detected() == true);   // 无效 -> 有风险

    data.bottom.valid = true;
    data.bottom.distance_cm = 50.0;
    CHECK(data.get_cliff_detected() == true);   // 有效且 >30cm -> 有风险

    data.bottom.valid = true;
    data.bottom.distance_cm = 15.0;
    CHECK(data.get_cliff_detected() == false);  // 有效且 <=30cm -> 安全
}

static void test_min_forward_valid_filter() {
    // F5: 无效读数(超时 -1.0)不得参与 min, 避免假性急停
    UltrasonicArrayData data;
    data.front_left.valid  = true;  data.front_left.distance_cm  = 20.0;
    data.front_center.valid = false; data.front_center.distance_cm = -1.0;
    data.front_right.valid = true;  data.front_right.distance_cm = 30.0;
    CHECK(data.get_min_forward_distance_cm() == 20.0);  // 忽略 -1.0

    data.front_left.valid = false; data.front_left.distance_cm = -1.0;
    data.front_center.valid = false; data.front_center.distance_cm = -1.0;
    data.front_right.valid = false; data.front_right.distance_cm = -1.0;
    CHECK(data.get_min_forward_distance_cm() == 400.0);  // 全无效 -> 量程上限
}

static void test_layer_fusion_boundaries() {
    AstraProDriver astra(true);
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);

    astra.start();  // 启动采集线程, 产生有效模拟帧 (quality_score > 0)

    // FIX-3: ir 模拟值 0~1 全覆盖, 200 次迭代内必须出现全部三档环境
    // (原实现环境恒 INDOOR 时三选一断言永远通过, 属名不副实的测试)
    std::set<int> seen;
    for (int i = 0; i < 200; ++i) {
        auto result = fusion.fuse();
        seen.insert(static_cast<int>(result.environment));
        // 权重不变量: 两者之和为 1, 且均在 [0,1] 范围
        double wsum = result.effective_astra_weight + result.effective_ultrasonic_weight;
        CHECK(std::abs(wsum - 1.0) < 1e-6);
        CHECK(result.effective_astra_weight >= 0.0 && result.effective_astra_weight <= 1.0);
        CHECK(result.effective_ultrasonic_weight >= 0.0 && result.effective_ultrasonic_weight <= 1.0);
    }
    // 三档覆盖: 若模拟环境被写死为单一档位, 此断言失败 (守护 FIX-2 的回归)
    CHECK(seen.count(static_cast<int>(EnvironmentType::INDOOR)) &&
          seen.count(static_cast<int>(EnvironmentType::SEMI_INDOOR)) &&
          seen.count(static_cast<int>(EnvironmentType::OUTDOOR)));

    astra.stop();
}

// 测试访问器: 暴露 SensorFusion 私有方法, 让测试调用真函数 (R-3)
// 必须位于 mechdog 命名空间内 (friend 声明限定于该命名空间)
namespace mechdog {
class SensorFusionTestAccess {
public:
    static std::pair<double, std::string> layer_fusion(
        SensorFusion& f, double ultra_m, double astra_m, bool astra_valid,
        double astra_w, double ultra_w) {
        return f.layer_fusion(ultra_m, astra_m, astra_valid, astra_w, ultra_w);
    }
    static ObstacleLevel classify_obstacle_level(SensorFusion& f, double dist_m) {
        return f.classify_obstacle_level(dist_m);
    }
    // A1: 暴露 fuse_direction 以验证方向级失效标记
    static FusedObstacle fuse_direction(SensorFusion& f, const std::string& dir,
                                        const AstraFrame& frame,
                                        const UltrasonicArrayData& ultra,
                                        double astra_w, double ultra_w) {
        return f.fuse_direction(dir, frame, ultra, astra_w, ultra_w);
    }
    static FusedObstacle build_bottom_obstacle(SensorFusion& f,
                                               const UltrasonicReading& bottom,
                                               bool cliff_detected) {
        return f.build_bottom_obstacle(bottom, cliff_detected);
    }
    static double calc_confidence(SensorFusion& f, double ultra_m, double astra_m,
                                  bool astra_valid) {
        return f.calc_confidence(ultra_m, astra_m, astra_valid);
    }
    // M6: 方向决策基于当前帧障碍 (参数显式传入, 不再读 last_fusion_)
    static NavigationAction choose_direction(
        SensorFusion& f, const std::unordered_map<std::string, FusedObstacle>& obstacles) {
        return f.choose_direction(obstacles);
    }
    // M1: 动作决策带传感器有效性 (fail-closed); R3: 前向是否有有效方向
    static NavigationAction determine_action(
        SensorFusion& f, double min_forward_m, double min_ultrasonic_cm,
        bool cliff_detected,
        const std::unordered_map<std::string, FusedObstacle>& obstacles,
        bool sensors_valid, bool front_valid = true) {
        return f.determine_action(min_forward_m, min_ultrasonic_cm, cliff_detected,
                                  obstacles, sensors_valid, front_valid);
    }
    static bool all_sensors_invalid(SensorFusion& f, const AstraFrame& frame,
                                    const UltrasonicArrayData& ultra) {
        return f.all_sensors_invalid(frame, ultra);
    }
    // ALG-3 (v2.2): 暴露 determine_environment 验证单一链路 (真实 IR/深度代理/模拟 IR/室外)
    static EnvironmentType determine_environment(SensorFusion& f, const AstraFrame& frame) {
        return f.determine_environment(frame);
    }
};

// ALG-4 (v2.2): 测试访问器 —— 注入 hw_unavailable_/use_simulated_ 状态做回归
// (InfraRedSensor 声明了 friend InfraRedTestAccess)
class InfraRedTestAccess {
public:
    static void set_hw_unavailable(InfraRedSensor& ir, bool v) { ir.hw_unavailable_ = v; }
    static void set_use_simulated(InfraRedSensor& ir, bool v) { ir.use_simulated_ = v; }
};
} // namespace mechdog

static void test_bottom_invalid_no_negative_distance() {
    // R5-1/R5-2: bottom 无效读数(-1.0cm, valid=false) 不得产生负距离
    // 原实现直接乘 kCmToM 得到 -0.01m 写入融合结果
    AstraProDriver astra(true);
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);

    UltrasonicReading invalid;
    invalid.sensor_name = "bottom";
    invalid.distance_cm = -1.0;  // 真机超时
    invalid.valid = false;

    auto obs = SensorFusionTestAccess::build_bottom_obstacle(fusion, invalid, /*cliff=*/true);
    CHECK(obs.distance_m >= 0.0);           // 无效 -> 兜底 0.0, 不得为负
    CHECK(obs.distance_m == 0.0);
    CHECK(obs.confidence == 0.0);           // 无效 -> 0 置信度
    CHECK(obs.level == ObstacleLevel::CRITICAL);  // 悬崖判定仍生效 (get_cliff_detected 语义)

    // 对照: 有效读数正常换算
    UltrasonicReading valid_r;
    valid_r.sensor_name = "bottom";
    valid_r.distance_cm = 15.0;
    valid_r.valid = true;
    auto obs2 = SensorFusionTestAccess::build_bottom_obstacle(fusion, valid_r, /*cliff=*/false);
    CHECK(std::abs(obs2.distance_m - 0.15) < 1e-9);
    CHECK(obs2.confidence == 1.0);
}

static void test_confidence_ultra_invalid_keeps_astra() {
    // R-1: 超声无效(兜底 4.5)不得参与一致性计算——兜底值不是真实距离,
    // 原实现 |4.5-8.0|/2>1 → consistency=0 → confidence 被错误压到 0
    AstraProDriver astra(true);
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);

    // 超声无效 + Astra 8m: 修复后 = 仅 Astra 单独可信度 0.8
    double conf_invalid = SensorFusionTestAccess::calc_confidence(
        fusion, /*ultra_m=*/4.5, /*astra_m=*/8.0, /*astra_valid=*/true);
    CHECK(std::abs(conf_invalid - 0.8) < 1e-6);

    // 对照: 超声有效且与 Astra 接近 (2.0 vs 2.5): consistency = 1-0.5/2 = 0.75
    double conf_valid = SensorFusionTestAccess::calc_confidence(
        fusion, /*ultra_m=*/2.0, /*astra_m=*/2.5, /*astra_valid=*/true);
    CHECK(std::abs(conf_valid - 0.95 * 0.75) < 1e-6);
}

static void test_invalid_ultrasonic_no_false_critical() {
    // 回归 D1/D2: 真机超声超时 (-1.0cm) 必须被过滤, 不得产生负距离/CRITICAL
    // R-3: 直接调用真函数 layer_fusion()/classify_obstacle_level(), 不再 lambda 复刻
    //   (原复刻逻辑一旦真函数变更测试不会失败, 起不到回归保护)
    AstraProDriver astra(true);
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);

    const double kCmToM = 0.01;
    const double ultra_invalid = 4.5;  // fuse_direction 过滤后的兜底值

    // 场景: 超时 -1.0cm 的无效读数 (修复前: ultra_dist_m = -0.01)
    struct Reading { double distance_cm; bool valid; };
    Reading bad{-1.0, false};
    Reading good{20.0, true};

    auto fused_dist = [&](const Reading& r) -> double {
        double ultra_m = (r.valid) ? r.distance_cm * kCmToM : ultra_invalid;
        // layer_fusion 真函数: 无效读数(4.5)不满足 L0/L1, 走 L2/L3 仅Astra 分支 (astra_m=8.0)
        auto [dist, src] = SensorFusionTestAccess::layer_fusion(
            fusion, ultra_m, 8.0, /*astra_valid=*/true, 0.8, 0.2);
        return dist;
    };

    double d_bad  = fused_dist(bad);
    double d_good = fused_dist(good);
    CHECK(d_bad >= 0.02);   // 无效读数不得产生负距离
    CHECK(d_good == 0.20);  // 有效读数 20cm 正常进入盲区

    // classify_obstacle_level 真函数: 修复后无效读数应判为 SAFE(>50cm), 而非 CRITICAL
    auto level = [&](double dist_m) {
        return SensorFusionTestAccess::classify_obstacle_level(fusion, dist_m);
    };
    CHECK(level(d_bad) == ObstacleLevel::SAFE);    // 无效 -> SAFE(兜底 4.5m), 不再误报 CRITICAL
    CHECK(level(d_good) == ObstacleLevel::DANGER); // 20cm -> DANGER(合理)
}

// H1: L2/L3 保守取近 —— 近处超声障碍不得被远处 Astra 加权平均/丢弃
// 报告 probe(修复前): ① astra=8.0(无有效像素兜底)+ultra=1.0 -> 8.0 (L3 丢弃超声)
//                    ② astra=7.9+ultra=1.0 -> 6.52 (L2 加权平均)
//                    ③ astra=3.1+ultra=0.7 -> 2.62 (L2 加权平均)
static void test_layer_fusion_take_nearest() {
    AstraProDriver astra(true);
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);

    // L3: astra 8.0 + ultra 1.0 -> 必须取近 1.0 (修复前 8.0)
    auto [d1, s1] = SensorFusionTestAccess::layer_fusion(fusion, 1.0, 8.0, true, 0.8, 0.2);
    CHECK(d1 == 1.0);
    CHECK(s1.find("超声") != std::string::npos);  // source 标注取近来源

    // L2: astra 7.9 + ultra 1.0 -> 必须取近 1.0 (修复前加权 6.52)
    auto [d2, s2] = SensorFusionTestAccess::layer_fusion(fusion, 1.0, 7.9, true, 0.8, 0.2);
    CHECK(d2 == 1.0);

    // L2: astra 3.1 + ultra 0.7 -> 必须取近 0.7 (修复前加权 2.62)
    auto [d3, s3] = SensorFusionTestAccess::layer_fusion(fusion, 0.7, 3.1, true, 0.8, 0.2);
    CHECK(d3 == 0.7);

    // L2: ultra 无效(兜底 4.5, 非真实距离) -> 仍用 Astra, 不得被兜底值污染
    auto [d4, s4] = SensorFusionTestAccess::layer_fusion(fusion, 4.5, 7.0, true, 0.8, 0.2);
    CHECK(d4 == 7.0);

    // L2: ultra 远于 astra -> 取近不误伤, 保持加权 (3.5*0.8 + 4.4*0.2 = 3.68)
    auto [d5, s5] = SensorFusionTestAccess::layer_fusion(fusion, 4.4, 3.5, true, 0.8, 0.2);
    CHECK(std::abs(d5 - 3.68) < 1e-9);

    // L0 盲区补偿不变: ultra < 0.6 直接返回
    auto [d6, s6] = SensorFusionTestAccess::layer_fusion(fusion, 0.3, 5.0, true, 0.8, 0.2);
    CHECK(d6 == 0.3);

    // L1 保守取 min 不变: astra 2.0 + ultra 1.5 -> 1.5
    auto [d7, s7] = SensorFusionTestAccess::layer_fusion(fusion, 1.5, 2.0, true, 0.8, 0.2);
    CHECK(d7 == 1.5);

    // 仅超声分支不变: astra 无效 + ultra 有效 -> ultra
    auto [d8, s8] = SensorFusionTestAccess::layer_fusion(fusion, 2.0, 8.0, false, 0.8, 0.2);
    CHECK(d8 == 2.0);
}

// M6: 方向决策必须基于当前帧障碍, 而非上一帧 (last_fusion_)
// 旧实现: choose_direction() 无参读 last_fusion_ -> 决策滞后一帧 (8Hz 下 ~125ms)
static void test_choose_direction_uses_current_frame() {
    AstraProDriver astra(true);
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);

    std::unordered_map<std::string, FusedObstacle> obs;
    FusedObstacle center; center.direction = "center"; center.distance_m = 0.30;  // 近, 触发转向
    FusedObstacle left;   left.direction = "left";   left.distance_m = 5.0;       // 左开阔
    FusedObstacle right;  right.direction = "right"; right.distance_m = 0.30;     // 右堵
    obs["center"] = center; obs["left"] = left; obs["right"] = right;

    // 左开阔右堵 -> 往开阔侧转 TURN_LEFT
    CHECK(SensorFusionTestAccess::choose_direction(fusion, obs) == NavigationAction::TURN_LEFT);

    // 反置: 右开阔左堵 -> TURN_RIGHT
    obs["left"].distance_m = 0.30;
    obs["right"].distance_m = 5.0;
    CHECK(SensorFusionTestAccess::choose_direction(fusion, obs) == NavigationAction::TURN_RIGHT);

    // center 开阔 -> 直行
    obs["center"].distance_m = 3.0;
    CHECK(SensorFusionTestAccess::choose_direction(fusion, obs) == NavigationAction::SLOW_FORWARD);

    // 两侧都堵 (<= 0.25 warning 阈值) -> BACKWARD
    obs["center"].distance_m = 0.30;
    obs["left"].distance_m = 0.20;
    obs["right"].distance_m = 0.20;
    CHECK(SensorFusionTestAccess::choose_direction(fusion, obs) == NavigationAction::BACKWARD);

    // 空数据 -> SLOW_FORWARD (默认兜底)
    std::unordered_map<std::string, FusedObstacle> empty;
    CHECK(SensorFusionTestAccess::choose_direction(fusion, empty) == NavigationAction::SLOW_FORWARD);
}

// N1: quality_score 归一化分母必须为本区域像素数, 不得是半幅满帧。
// 修复前 center 区域 76800 像素 / 分母 153600 -> quality 数学上限 0.5,
// indoor 权重被腰斩成 0.8*0.5=0.4。修复后完美模拟帧 quality 应接近 1.0。
static void test_quality_score_not_halved() {
    AstraProDriver astra(true);
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);
    astra.start();  // 捕获线程
    std::this_thread::sleep_for(std::chrono::milliseconds(150));  // 等待首帧就绪

    // 模拟帧全填有效距离 -> quality 应能超过 0.5 (修复前恒 ==0.5)
    double max_quality = 0.0;
    for (int i = 0; i < 60; ++i) {
        auto f = astra.get_latest_frame();
        if (!f.valid || f.depth_map.empty()) continue;
        if (f.center_region.quality_score > max_quality)
            max_quality = f.center_region.quality_score;
    }
    CHECK(max_quality > 0.6);   // 修复前恒 0.5, 修复后应接近 1.0

    // 完美帧 + indoor -> indoor 权重应恢复接近 0.8 (不被腰斩)
    // 用 explore: indoor base 0.8, quality 接近 1 -> astra_w 接近 0.8
    double astra_w = 0.0;
    for (int i = 0; i < 60; ++i) {
        auto result = fusion.fuse();
        if (result.environment == EnvironmentType::INDOOR && result.astra_valid) {
            astra_w = result.effective_astra_weight;
            break;
        }
    }
    if (astra_w > 0.0) {
        CHECK(astra_w > 0.5);   // 修复前 indoor 理想时 0.4, 修复后应 >0.5
    }

    astra.stop();
}

// A1: 单方向双侧失效 (valid=false) 时, 该方向的 8.0m "假设无障碍" 兜底值
// 不得被当作最开阔盲区参与转向决策 —— 否则机器人会转向实际被遮挡的一侧。
static void test_choose_direction_ignores_invalid_direction() {
    AstraProDriver astra(true);
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);

    std::unordered_map<std::string, FusedObstacle> obs;
    FusedObstacle center; center.direction = "center"; center.distance_m = 0.30;  // 近, 需转向
    FusedObstacle left;   left.direction = "left";   left.distance_m = 0.30;      // 有效读数
    FusedObstacle right;  right.direction = "right"; right.distance_m = 8.0;
    right.valid = false;   // A1: 右侧双侧失效 —— 8.0m 是"假设无障碍"兜底, 非真实
    obs["center"] = center; obs["left"] = left; obs["right"] = right;

    // 修复前: right_dist=8.0 被当最开阔 -> TURN_RIGHT (转向盲区!)
    // 修复后: 排除右侧, 左侧为唯一有效读数且 > warning(0.25) -> TURN_LEFT (远离盲区)
    CHECK(SensorFusionTestAccess::choose_direction(fusion, obs) == NavigationAction::TURN_LEFT);

    // 轻证: 若左侧也堵(<=warning), 则唯一可行侧(右)失效 -> 保守 BACKWARD
    obs["left"].distance_m = 0.20;
    CHECK(SensorFusionTestAccess::choose_direction(fusion, obs) == NavigationAction::BACKWARD);

    // 单侧有效且开阔: 左 5.0 (有效), 右 8.0 (失效) -> 只能转向左侧 (远离盲区)
    obs["left"].distance_m = 5.0;
    CHECK(SensorFusionTestAccess::choose_direction(fusion, obs) == NavigationAction::TURN_LEFT);

    // 中心失效时不作为开阔依据: 中心 8.0 失效 + 左/右 0.20 (均堵, <=warning) ->
    // 唯一可能缓行的依据(center 开阔)已失效, 两侧又都不可转向 -> 保守 BACKWARD
    std::unordered_map<std::string, FusedObstacle> obs2;
    FusedObstacle c2; c2.direction = "center"; c2.distance_m = 8.0; c2.valid = false;
    FusedObstacle l2; l2.direction = "left";   l2.distance_m = 0.20;
    FusedObstacle r2; r2.direction = "right";  r2.distance_m = 0.20;
    obs2["center"] = c2; obs2["left"] = l2; obs2["right"] = r2;
    // center 失效不作为开阔依据(SLOW_FORWARD 需 center_ok); 两侧堵 -> BACKWARD
    CHECK(SensorFusionTestAccess::choose_direction(fusion, obs2) == NavigationAction::BACKWARD);

    // 三个前向方向全部失效 -> 无可靠方向信息 -> 保守 SLOW_FORWARD
    std::unordered_map<std::string, FusedObstacle> obs3;
    FusedObstacle c3; c3.direction = "center"; c3.distance_m = 8.0; c3.valid = false;
    FusedObstacle l3; l3.direction = "left";   l3.distance_m = 8.0; l3.valid = false;
    FusedObstacle r3; r3.direction = "right";  r3.distance_m = 8.0; r3.valid = false;
    obs3["center"] = c3; obs3["left"] = l3; obs3["right"] = r3;
    CHECK(SensorFusionTestAccess::choose_direction(fusion, obs3) == NavigationAction::SLOW_FORWARD);
}

// A1 (全链路): fuse_direction 对双侧失效方向标记 valid=false, fuse() 的
// min_forward_distance_m 排除该方向 —— 单方向失明不污染前方最小距离。
static void test_fuse_direction_marks_blind_direction_invalid() {
    AstraProDriver astra(true);
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);

    // 构造: 左侧 Astra 区域无有效像素 (valid_pixel_ratio=0) + 左侧超声无效
    AstraFrame frame; frame.valid = true;
    frame.center_region.valid_pixel_ratio = 0.9;  // 中央有效
    frame.left_region.valid_pixel_ratio = 0.0;    // 左侧无像素
    frame.right_region.valid_pixel_ratio = 0.9;   // 右侧有效

    UltrasonicArrayData ultra;
    ultra.front_center.valid = true;  ultra.front_center.distance_cm = 100.0;
    ultra.front_right.valid = true;   ultra.front_right.distance_cm = 100.0;
    ultra.front_left.valid = false;   // 左侧超声失效

    auto left_obs = SensorFusionTestAccess::fuse_direction(
        fusion, "left", frame, ultra, 0.8, 0.2);
    CHECK(left_obs.valid == false);            // A1: 双侧失效 -> 标记无效
    CHECK(left_obs.distance_m == 8.0);         // 兜底值仍在 (仅供可视化) 但不参与决策

    auto right_obs = SensorFusionTestAccess::fuse_direction(
        fusion, "right", frame, ultra, 0.8, 0.2);
    CHECK(right_obs.valid == true);            // 右侧 Astar+超声 均有效
}

// M1: 全传感器失效 -> fail-closed STOP, 不得"假设无障碍"继续前进
static void test_all_invalid_fail_closed() {
    AstraProDriver astra(true);
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);

    // all_sensors_invalid 判定 (与融合层同一口径: astra 有效 = 任一前向区域有有效深度像素)
    AstraFrame bad_frame; bad_frame.valid = false;
    UltrasonicArrayData bad_ultra;  // 默认全 invalid
    CHECK(SensorFusionTestAccess::all_sensors_invalid(fusion, bad_frame, bad_ultra) == true);

    UltrasonicArrayData ok_ultra = bad_ultra;
    ok_ultra.front_center.valid = true; ok_ultra.front_center.distance_cm = 100.0;
    CHECK(SensorFusionTestAccess::all_sensors_invalid(fusion, bad_frame, ok_ultra) == false);

    // M1 缺口: frame.valid=true 但所有区域无有效像素 (镜头被挡/全黑) ——
    // 融合层 (H1 后) 已按无效处理, all_sensors_invalid 必须同口径, 否则仍 FORWARD
    AstraFrame lens_covered; lens_covered.valid = true;  // regions 默认 valid_pixel_ratio=0
    CHECK(SensorFusionTestAccess::all_sensors_invalid(fusion, lens_covered, bad_ultra) == true);

    // 任一区域有有效像素 -> astra 有效
    AstraFrame ok_frame; ok_frame.valid = true;
    ok_frame.center_region.valid_pixel_ratio = 0.5;
    CHECK(SensorFusionTestAccess::all_sensors_invalid(fusion, ok_frame, bad_ultra) == false);

    // 镜头被挡 + 超声全失效 -> determine_action 必须 STOP (fail-closed 兜底)
    std::unordered_map<std::string, FusedObstacle> obs_lens;
    auto act_lens = SensorFusionTestAccess::determine_action(
        fusion, /*min_forward_m=*/8.0, /*min_ultrasonic_cm=*/400.0,
        /*cliff_detected=*/false, obs_lens, /*sensors_valid=*/false);
    CHECK(act_lens == NavigationAction::STOP);

    // 全失效兜底组合 (min_fwd=8.0 兜底, ultra=400 兜底, 无悬崖) -> 必须 STOP
    std::unordered_map<std::string, FusedObstacle> obs;
    auto act = SensorFusionTestAccess::determine_action(
        fusion, /*min_forward_m=*/8.0, /*min_ultrasonic_cm=*/400.0,
        /*cliff_detected=*/false, obs, /*sensors_valid=*/false);
    CHECK(act == NavigationAction::STOP);

    // 对照: 传感器有效时同输入 -> 正常 FORWARD (8m 开阔)
    auto act2 = SensorFusionTestAccess::determine_action(
        fusion, 8.0, 400.0, false, obs, /*sensors_valid=*/true);
    CHECK(act2 == NavigationAction::FORWARD);
}

// R3: 前向 (left/center/right) 全盲 + bottom 有效 -> 旧判定把 bottom 计入有效性
// 导致 sensors_valid=true, min_forward=8.0 兜底 -> FORWARD (fail-open)。
// 修复: front_valid 独立判定, 前向失明时必须保守 (SLOW_FORWARD), bottom 无效力。
static void test_front_blind_with_bottom_valid_is_conservative() {
    AstraProDriver astra(true);
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);

    std::unordered_map<std::string, FusedObstacle> obs;
    FusedObstacle left;   left.direction = "left";   left.distance_m = 8.0; left.valid = false;
    FusedObstacle center; center.direction = "center"; center.distance_m = 8.0; center.valid = false;
    FusedObstacle right;  right.direction = "right";  right.distance_m = 8.0; right.valid = false;
    obs["left"] = left; obs["center"] = center; obs["right"] = right;

    // 模拟 R3 场景: sensors_valid=true (bottom 有效), 但 front_valid=false
    // 修复前: 8.0m 开阔 -> FORWARD; 修复后: 前向失明 -> SLOW_FORWARD
    auto act = SensorFusionTestAccess::determine_action(
        fusion, /*min_forward_m=*/8.0, /*min_ultrasonic_cm=*/400.0,
        /*cliff_detected=*/false, obs, /*sensors_valid=*/true, /*front_valid=*/false);
    CHECK(act == NavigationAction::SLOW_FORWARD);

    // 对照: front_valid=true 且开阔 -> 正常 FORWARD (回归不误伤)
    auto act2 = SensorFusionTestAccess::determine_action(
        fusion, 8.0, 400.0, false, obs, /*sensors_valid=*/true, /*front_valid=*/true);
    CHECK(act2 == NavigationAction::FORWARD);

    // 对照: front_valid=false 但悬崖 -> STOP 仍然优先 (R3 不影响悬崖安全)
    auto act3 = SensorFusionTestAccess::determine_action(
        fusion, 8.0, 400.0, /*cliff_detected=*/true, obs, true, false);
    CHECK(act3 == NavigationAction::STOP);

    // R3 全链路: fuse() 里 bottom 有效 + 三前向全盲 -> SLOW_FORWARD
    // (all_sensors_invalid 返回 false 因为 bottom valid; 但 front_valid=false)
    AstraFrame blind_frame; blind_frame.valid = true;  // regions 默认 valid_pixel_ratio=0
    UltrasonicArrayData ultra;
    ultra.bottom.valid = true;   ultra.bottom.distance_cm = 15.0;  // 底部有效(防摔)
    // 前向全失效
    // 通过 fuse_direction 验证三个前向都是 invalid
    CHECK(SensorFusionTestAccess::fuse_direction(
        fusion, "left", blind_frame, ultra, 0.8, 0.2).valid == false);
    CHECK(SensorFusionTestAccess::fuse_direction(
        fusion, "center", blind_frame, ultra, 0.8, 0.2).valid == false);
    CHECK(SensorFusionTestAccess::fuse_direction(
        fusion, "right", blind_frame, ultra, 0.8, 0.2).valid == false);
    // sensors_valid=true (bottom 有效) 但 front_valid=false -> determine_action SLOW_FORWARD
    CHECK(SensorFusionTestAccess::all_sensors_invalid(fusion, blind_frame, ultra) == false);
}

// R4: build_bottom_obstacle 无效读数时 valid 必须同步置 false
// (旧实现 distance_m 兜底 0.0 但 valid 仍默认 true, 字段语义误导 —— 0.0 是"无数据"非贴地)
static void test_bottom_invalid_sets_valid_false() {
    AstraProDriver astra(true);
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);

    UltrasonicReading invalid;
    invalid.sensor_name = "bottom";
    invalid.distance_cm = -1.0;  // 真机超时
    invalid.valid = false;
    auto obs_bad = SensorFusionTestAccess::build_bottom_obstacle(fusion, invalid, false);
    CHECK(obs_bad.valid == false);           // R4: 无效 -> valid=false
    CHECK(obs_bad.distance_m == 0.0);
    CHECK(obs_bad.confidence == 0.0);

    UltrasonicReading ok;
    ok.sensor_name = "bottom";
    ok.distance_cm = 15.0;
    ok.valid = true;
    auto obs_ok = SensorFusionTestAccess::build_bottom_obstacle(fusion, ok, false);
    CHECK(obs_ok.valid == true);             // 有效 -> valid=true (不误伤)
    CHECK(obs_ok.distance_m == 0.15);
}

// ALG-1 (v2.2): 底部独立通路 fail-closed + 逻辑自洽
static void test_is_fall_risk_fail_closed() {
    // 1) layout 不含 "bottom": bottom_sensor_=nullptr, bottom_have_ 恒 false -> 必须判有风险
    auto no_bottom = get_ultrasonic_layout();
    no_bottom.erase("bottom");
    {
        UltrasonicArrayDriver driver(no_bottom);
        CHECK(driver.is_fall_risk() == true);   // 无 bottom 传感器 -> fail-closed
        UltrasonicReading br = driver.get_bottom_reading();
        CHECK(br.valid == false);               // 未就绪 -> 默认无效读数
    }

    // 2) 完整 layout: bottom 线程 20Hz 刷新, is_fall_risk 与 get_bottom_reading 自洽
    AstraProDriver astra(true);
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));  // 等 bottom 首读
    UltrasonicReading br = ultrasonic.get_bottom_reading();
    CHECK(br.valid == true);  // sim 底部读数恒 valid (12-28cm 或 200-350cm 均在 2-400 量程)
    // is_fall_risk 必须与缓存读数同源 (F4 fail-closed 公式)
    bool expected = !br.valid || br.distance_cm > UltrasonicConfig::cliff_threshold_cm;
    CHECK(ultrasonic.is_fall_risk() == expected);
}

// 方案A (REVIEW): is_fall_risk 必须**优先用注入的 bottom** —— 外部注入 (ROS /ultrasonic,
// 后续为 STM32 捕获上报) 是真实数据源; 内部 bottom 线程在 ROS 编译下是模拟/占位, 不注入
// 时才会回退。修复前: cliff 判定无视注入数据, 真机防跌落到模拟读数上 (fail-open 缺口)。
static void test_is_fall_risk_uses_injected_bottom() {
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());

    // 1) 注入底部安全距离 (15cm < 阈值) -> 无风险 (即使内部线程尚未产出首读)
    UltrasonicArrayData safe;
    safe.bottom.valid = true; safe.bottom.distance_cm = 15.0;
    ultrasonic.inject_external_data(safe);
    CHECK(ultrasonic.is_fall_risk() == false);

    // 2) 注入底部无效 -> fail-closed 有风险
    UltrasonicArrayData bad;
    bad.bottom.valid = false; bad.bottom.distance_cm = 400.0;
    ultrasonic.inject_external_data(bad);
    CHECK(ultrasonic.is_fall_risk() == true);

    // 3) 注入底部超阈值 (悬崖) -> 有风险
    UltrasonicArrayData cliff;
    cliff.bottom.valid = true;
    cliff.bottom.distance_cm = UltrasonicConfig::cliff_threshold_cm + 10.0;
    ultrasonic.inject_external_data(cliff);
    CHECK(ultrasonic.is_fall_risk() == true);

    // 4) 注入过期 (timeout=0) 后回退内部线程口径 (与 read_all 同语义)
    ultrasonic.set_inject_timeout_sec(0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    UltrasonicReading br = ultrasonic.get_bottom_reading();
    bool expected = !br.valid || br.distance_cm > UltrasonicConfig::cliff_threshold_cm;
    CHECK(ultrasonic.is_fall_risk() == expected);
}

// ALG-4 (v2.2): 真机硬件失败 (hw_unavailable_) 不返回随机值, 走 -1 故障值
static void test_hw_unavailable_no_random() {
    InfraRedSensor ir(true);  // 构造为模拟
    // 注入"真机 TSL2591 初始化失败"状态: 非模拟 + 硬件不可用
    InfraRedTestAccess::set_use_simulated(ir, false);
    InfraRedTestAccess::set_hw_unavailable(ir, true);
    CHECK(ir.is_simulated() == false);
    CHECK(ir.is_real_available() == false);     // 真机失败 -> 不可用
    double v = ir.read_normalized_light();
    CHECK(v == -1.0);                           // 不返回随机, 返回故障值

    // 对照: 模拟模式保留随机语义 (FIX-2, 不被本修复破坏)
    InfraRedSensor ir2(true);
    double s = ir2.read_normalized_light();
    CHECK(s >= 0.0 && s <= 1.0);                 // 模拟 -> [0,1] 随机
    CHECK(ir2.is_simulated() == true);
    CHECK(ir2.is_real_available() == false);    // 模拟即非真机可用
}

// ALG-3 (v2.2): determine_environment 单一链路 —— 真机 IR 失败时走深度代理, 不消费随机
static void test_determine_environment_depth_proxy_default() {
    AstraProDriver astra(true);
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);

    // 注入真机 IR 失败 (use_simulated=false, hw_unavailable=true)
    InfraRedTestAccess::set_use_simulated(ir, false);
    InfraRedTestAccess::set_hw_unavailable(ir, true);
    CHECK(ir.is_real_available() == false);

    // 帧有效 + environment 已由深度代理算好 -> 直接用, 不读随机 IR
    AstraFrame frame; frame.valid = true;
    frame.environment = EnvironmentType::INDOOR;
    CHECK(SensorFusionTestAccess::determine_environment(fusion, frame) == EnvironmentType::INDOOR);
    frame.environment = EnvironmentType::OUTDOOR;
    CHECK(SensorFusionTestAccess::determine_environment(fusion, frame) == EnvironmentType::OUTDOOR);

    // 帧无效 + 真机 IR 失败 (非模拟) -> 室外 (安全侧), 不消费随机
    AstraFrame bad_frame; bad_frame.valid = false;
    bad_frame.environment = EnvironmentType::UNKNOWN;
    CHECK(SensorFusionTestAccess::determine_environment(fusion, bad_frame) == EnvironmentType::OUTDOOR);

    // 对照: 模拟 IR (use_simulated=true) + UNKNOWN 帧 -> 消费模拟随机值覆盖三档 (FIX-2 保留)
    InfraRedTestAccess::set_use_simulated(ir, true);
    InfraRedTestAccess::set_hw_unavailable(ir, false);
    AstraFrame sim_frame; sim_frame.valid = true;
    sim_frame.environment = EnvironmentType::UNKNOWN;
    EnvironmentType e = SensorFusionTestAccess::determine_environment(fusion, sim_frame);
    CHECK(e == EnvironmentType::INDOOR || e == EnvironmentType::SEMI_INDOOR ||
          e == EnvironmentType::OUTDOOR);
}

// ============================================================
// FIX-01 回归: 深度帧"可按 (w,h) 索引 depth_map"的守卫谓词
//
// 失效帧 (capture_real 取帧失败 / 模拟前) 的 depth_map 为空, 但
// depth_width/height 仍是默认 640x480 (sensor_astra.h) —— 直接按
// y*hw+x 索引即为空 vector 越界读 (UB)。热力图分支 (main.cpp D 键)
// 曾缺这道守卫; 谓词抽到 sensor_astra.h 供主循环与单测共用同一口径。
// ============================================================
static void test_depth_frame_usable_guard() {
    // 1) 三态失效帧: valid=false + 空 depth_map + 默认 640x480 → 不可用
    AstraFrame dead;
    dead.valid = false;
    CHECK(dead.depth_width == 640 && dead.depth_height == 480);  // 默认值本身就是坑
    CHECK(dead.depth_map.empty());
    CHECK(!depth_frame_usable(dead));

    // 2) 正常帧: 640x480 全尺寸 → 可用
    AstraFrame good;
    good.valid = true;
    good.depth_map.assign(static_cast<size_t>(good.depth_width) * good.depth_height, 1500);
    CHECK(depth_frame_usable(good));

    // 3) valid=true 但图空 (取帧成功但未填充) → 不可用
    AstraFrame empty_ok;
    empty_ok.valid = true;
    CHECK(!depth_frame_usable(empty_ok));

    // 4) 尺寸不足以覆盖 w*h (部分填充) → 不可用
    AstraFrame short_map;
    short_map.valid = true;
    short_map.depth_map.assign(1024, 1500);
    CHECK(!depth_frame_usable(short_map));

    // 5) 真机 320x240 帧 (SDK 默认未设 mode) → 可用
    AstraFrame real320;
    real320.valid = true;
    real320.depth_width = 320;
    real320.depth_height = 240;
    real320.depth_map.assign(320u * 240u, 1500);
    CHECK(depth_frame_usable(real320));

    // 6) 非法宽高 (0/负) → 不可用
    AstraFrame bad_dim;
    bad_dim.valid = true;
    bad_dim.depth_width = 0;
    bad_dim.depth_map.assign(640u * 480u, 1500);
    CHECK(!depth_frame_usable(bad_dim));
}

// ============================================================
// 路1: 近场地形避障 (P1/P1.5 → 决策)
//   背景: P1 负障碍点 / P1.5 2.5D 禁行格此前**只喂可视化**
//   (main.cpp 的 g_neg_cloud / g_hm25_snapshot), determine_action 看不到 →
//   前方有坑只能靠底部超声踩到边沿兜底 (事后), 不能提前停/让 (事前)。
//   锁定: 近场走廊命中 → STOP; 中距命中 → 至少降速 (明显偏侧则让开);
//         走廊外/未注入 → 与接入前逐位一致 (回归)。
// ============================================================

// 造 2.5D 结果: 全 Unknown, 在指定位置放一个禁行格 (栅格覆盖 x∈[0,3.0], y∈±2.5)
static HeightMap25Result make_hm25_cell(double wx, double wy, CellFlag f) {
    HeightMap25Result hm;
    hm.valid = true;
    hm.cols = 61; hm.rows = 101;
    hm.cell_size = 0.05; hm.min_x_m = 0.0; hm.y_half_m = 2.5;
    const size_t n = static_cast<size_t>(hm.cols) * hm.rows;
    hm.flag.assign(n, CellFlag::Unknown);
    hm.height.assign(n, 0.0f);
    int c = 0, r = 0;
    if (hm.world_to_index(wx, wy, c, r)) hm.flag[static_cast<size_t>(r) * hm.cols + c] = f;
    return hm;
}

// 造一条沿 y 的禁行带 (对称 → 平均 y≈0)
static HeightMap25Result make_hm25_band(double wx, double y_lo, double y_hi, CellFlag f) {
    HeightMap25Result hm = make_hm25_cell(wx, y_lo, f);
    for (double y = y_lo; y <= y_hi + 1e-9; y += 0.05) {
        int c = 0, r = 0;
        if (hm.world_to_index(wx, y, c, r)) hm.flag[static_cast<size_t>(r) * hm.cols + c] = f;
    }
    return hm;
}

static void test_terrain_near_block_stops() {
    AstraProDriver astra(true);
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);

    const std::unordered_map<std::string, FusedObstacle> none;
    GroundSegResult no_neg;

    // 基线: 未注入地形, 前方 8m 开阔 → FORWARD (与接入前一致)
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 400.0, false, none, true, true)
          == NavigationAction::FORWARD);

    // 前方 0.8m 有坑 (CliffDown) → STOP
    fusion.set_local_terrain(make_hm25_cell(0.8, 0.0, CellFlag::CliffDown), no_neg);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 400.0, false, none, true, true)
          == NavigationAction::STOP);

    // 过陡 (TooSteep) 与坑同类 → 仍 STOP
    fusion.set_local_terrain(make_hm25_cell(0.8, 0.0, CellFlag::TooSteep), no_neg);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 400.0, false, none, true, true)
          == NavigationAction::STOP);

    // ---- v2.9 路1 细分: 凸起(ObstacleUp)不再一律 STOP ----
    // 证据: 实机 181 帧 STOP 124 / FORWARD 57 ⇒ 旧口径"近场任意标签都停"过保守。
    // 0.8m 正中凸起: 未贴身(>bump_stop_x 0.70) ⇒ 降速靠近, 不停车
    fusion.set_local_terrain(make_hm25_cell(0.8, 0.0, CellFlag::ObstacleUp), no_neg);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 400.0, false, none, true, true)
          == NavigationAction::SLOW_FORWARD);
    // 0.6m 正中凸起: 贴身(<=0.70) + 正中(|y|<=0.18) ⇒ STOP
    fusion.set_local_terrain(make_hm25_cell(0.6, 0.0, CellFlag::ObstacleUp), no_neg);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 400.0, false, none, true, true)
          == NavigationAction::STOP);
    // 偏左凸起 (y=+0.25: 在走廊半宽 0.30 内, 且超出正中阈值 0.18) ⇒ 向右让开
    fusion.set_local_terrain(make_hm25_cell(0.65, 0.25, CellFlag::ObstacleUp), no_neg);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 400.0, false, none, true, true)
          == NavigationAction::TURN_RIGHT);
    // 偏右凸起 (y=-0.25) ⇒ 向左让开
    fusion.set_local_terrain(make_hm25_cell(0.65, -0.25, CellFlag::ObstacleUp), no_neg);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 400.0, false, none, true, true)
          == NavigationAction::TURN_LEFT);
    // 近场凸起 + 前向失明 ⇒ 仍是让开/降速语义 (不被前向失明覆盖)
    fusion.set_local_terrain(make_hm25_cell(0.65, 0.25, CellFlag::ObstacleUp), no_neg);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 400.0, false, none, true, false)
          == NavigationAction::TURN_RIGHT);

    // 近场有坑 + 前向失明 (原行为 SLOW_FORWARD) → 仍 STOP (路1 优先级更高, 但低于悬崖)
    fusion.set_local_terrain(make_hm25_cell(0.8, 0.0, CellFlag::CliffDown), no_neg);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 400.0, false, none, true, false)
          == NavigationAction::STOP);
    // 悬崖优先级仍在路1之上 (cliff=true → STOP, 与是否注入地形无关)
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 400.0, true, none, true, true)
          == NavigationAction::STOP);

    // 清除地形 → 回到基线 (回归: 不注入 = 零影响)
    fusion.clear_local_terrain();
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 400.0, false, none, true, true)
          == NavigationAction::FORWARD);
}

// ============================================================
// 路1 边界对加固 (OFFLINE_TODO #4) —— 把"已跑通的行为"锁进单测
//   约定来源 config.h: near_x_lo_m=0.40 / near_x_hi_m=1.20 / corridor_y_half_m=0.30
//                     bump_stop_x_m=0.70 (盲区 0.6 + 机身余量) / 正中 |y|<=0.18
//   每个判据都测"界内 / 界外"一对, 任一侧回归都会被抓住。
// ============================================================
static NavigationAction act_with_cell(SensorFusion& fusion, const std::unordered_map<std::string, FusedObstacle>& none,
                                      double wx, double wy, CellFlag f, bool fwd_ok = true) {
    GroundSegResult no_neg;
    fusion.set_local_terrain(make_hm25_cell(wx, wy, f), no_neg);
    return SensorFusionTestAccess::determine_action(fusion, 8.0, 400.0, false, none, true, fwd_ok);
}

static void test_terrain_threshold_edges() {
    AstraProDriver astra(true);
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);
    const std::unordered_map<std::string, FusedObstacle> none;

    // ---- ① 贴身判据 bump_stop_x_m = 0.70 (凸起, 正中) ----
    CHECK(act_with_cell(fusion, none, 0.70, 0.0, CellFlag::ObstacleUp) == NavigationAction::STOP);          // 界内(含)
    CHECK(act_with_cell(fusion, none, 0.75, 0.0, CellFlag::ObstacleUp) == NavigationAction::SLOW_FORWARD);  // 界外 ⇒ 降速
    CHECK(act_with_cell(fusion, none, 0.65, 0.0, CellFlag::ObstacleUp) == NavigationAction::STOP);          // 更近 ⇒ 停

    // ---- ② 正中判据 |y| <= 0.18 (贴身凸起) ----
    CHECK(act_with_cell(fusion, none, 0.65,  0.18, CellFlag::ObstacleUp) == NavigationAction::STOP);          // 界内(含) ⇒ 正中
    CHECK(act_with_cell(fusion, none, 0.65,  0.20, CellFlag::ObstacleUp) == NavigationAction::TURN_RIGHT);    // 偏左 ⇒ 向右让
    CHECK(act_with_cell(fusion, none, 0.65, -0.20, CellFlag::ObstacleUp) == NavigationAction::TURN_LEFT);     // 偏右 ⇒ 向左让

    // ---- ③ 走廊近界 near_x_lo_m = 0.40 ----
    CHECK(act_with_cell(fusion, none, 0.40, 0.0, CellFlag::CliffDown) == NavigationAction::STOP);   // 界内 ⇒ 参与
    CHECK(act_with_cell(fusion, none, 0.35, 0.0, CellFlag::CliffDown) == NavigationAction::FORWARD); // 界外 ⇒ 不参与(路1 不表态)

    // ---- ④ 走廊远界 near_x_hi_m = 1.20 (近场 STOP / 中距 至少降速) ----
    CHECK(act_with_cell(fusion, none, 1.20, 0.0, CellFlag::CliffDown) == NavigationAction::STOP);
    CHECK(act_with_cell(fusion, none, 1.25, 0.0, CellFlag::CliffDown) == NavigationAction::SLOW_FORWARD);

    // ---- ⑤ 走廊半宽 corridor_y_half_m = 0.30 ----
    CHECK(act_with_cell(fusion, none, 0.80,  0.30, CellFlag::CliffDown) == NavigationAction::STOP);    // 界内(含)
    CHECK(act_with_cell(fusion, none, 0.80,  0.35, CellFlag::CliffDown) == NavigationAction::FORWARD);  // 界外 ⇒ 不参与

    // ---- ⑥ 回归: 清除地形后必须回到"零影响" ----
    fusion.clear_local_terrain();
    GroundSegResult no_neg;
    fusion.set_local_terrain(make_hm25_cell(0.5, 0.0, CellFlag::CliffDown), no_neg);
    fusion.clear_local_terrain();
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 400.0, false, none, true, true)
          == NavigationAction::FORWARD);
}

static void test_terrain_mid_slow_and_dodge() {
    AstraProDriver astra(true);
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);

    const std::unordered_map<std::string, FusedObstacle> none;
    GroundSegResult no_neg;

    // 中距 1.5m 对称坑带 (走廊内平均 y≈0) → 降速
    fusion.set_local_terrain(make_hm25_band(1.5, -0.3, 0.3, CellFlag::CliffDown), no_neg);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 400.0, false, none, true, true)
          == NavigationAction::SLOW_FORWARD);

    // 走廊内偏左 (y≈+0.275) → 向右让
    fusion.set_local_terrain(make_hm25_cell(1.5, 0.25, CellFlag::CliffDown), no_neg);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 400.0, false, none, true, true)
          == NavigationAction::TURN_RIGHT);

    // 走廊内偏右 (y≈-0.275) → 向左让
    fusion.set_local_terrain(make_hm25_cell(1.5, -0.25, CellFlag::CliffDown), no_neg);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 400.0, false, none, true, true)
          == NavigationAction::TURN_LEFT);

    // 只降速不升级: 前方 20cm 障碍本应 BACKWARD → 仍 BACKWARD (中距地形不得改写更保守的动作)
    fusion.set_local_terrain(make_hm25_band(1.5, -0.3, 0.3, CellFlag::CliffDown), no_neg);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 0.20, 400.0, false, none, true, true)
          == NavigationAction::BACKWARD);

    fusion.clear_local_terrain();
}

static void test_terrain_corridor_bounds_and_negative_points() {
    AstraProDriver astra(true);
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);

    const std::unordered_map<std::string, FusedObstacle> none;
    GroundSegResult no_neg;

    // 走廊外不干预: 横向 y=1.5m (机身侧外) → FORWARD
    fusion.set_local_terrain(make_hm25_cell(0.8, 1.5, CellFlag::CliffDown), no_neg);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 400.0, false, none, true, true)
          == NavigationAction::FORWARD);

    // 超出中距 (3.02m > 2.0m) → FORWARD
    fusion.set_local_terrain(make_hm25_cell(3.0, 0.0, CellFlag::CliffDown), no_neg);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 400.0, false, none, true, true)
          == NavigationAction::FORWARD);

    // 近界之外 (0.22m < 0.40m): 交给底部超声 (0.6m 盲区) → 本模块不干预
    fusion.set_local_terrain(make_hm25_cell(0.20, 0.0, CellFlag::CliffDown), no_neg);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 400.0, false, none, true, true)
          == NavigationAction::FORWARD);

    // P1 负障碍点 (2.5D 无平面 → hm.valid=false) → 仍 STOP (两路 OR)
    GroundSegResult seg;
    Point3D p; p.x = 0.8; p.y = 0.1; p.z = -0.5;
    seg.negative_points.push_back(p);
    HeightMap25Result invalid_hm;   // valid=false
    fusion.set_local_terrain(invalid_hm, seg);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 400.0, false, none, true, true)
          == NavigationAction::STOP);

    fusion.clear_local_terrain();
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 400.0, false, none, true, true)
          == NavigationAction::FORWARD);
}

// v2.5: 超声链路可被**显式禁用** —— 真机上无超声硬件/无数据源时, 不能把"模拟随机数"
// (带 valid=true, 且底部 5% 概率造悬崖) 或"bottom fail-closed 永久判悬崖"带进决策。
// 禁用后: 悬崖层停用、超声不参与融合, 深度照常工作; 且可逆 (恢复后可再启用)。
static void test_ultrasonic_disabled_removes_cliff_layer() {
    AstraProDriver astra(false);          // 不 start(): 用注入帧保证三次 fuse 看到同一帧
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);

    // 固定深度: 前方 1.5m 墙面 (v2.4 注入接口)
    std::vector<uint16_t> wall(640 * 480, 1500);
    CHECK(astra.inject_depth_frame(wall, 640, 480, 100.0) == true);

    // 超声: 前向 120cm, 底部 >30cm (有跌落风险)
    UltrasonicArrayData u;
    u.timestamp = 0.0;
    u.front_left.valid = u.front_center.valid = u.front_right.valid = true;
    u.front_left.distance_cm = u.front_center.distance_cm = u.front_right.distance_cm = 120.0;
    u.bottom.valid = true;
    u.bottom.distance_cm = UltrasonicConfig::cliff_threshold_cm + 20.0;
    ultrasonic.inject_external_data(u);

    auto r1 = fusion.fuse();
    CHECK(r1.cliff_detected == true);                        // 悬崖层生效
    CHECK(r1.recommended_action == NavigationAction::STOP);

    // 禁用超声 → 悬崖层停用 (不是"永久停"), 决策回到深度
    fusion.set_ultrasonic_enabled(false);
    CHECK(fusion.ultrasonic_enabled() == false);
    auto r2 = fusion.fuse();
    CHECK(r2.cliff_detected == false);
    CHECK(r2.sensors_valid == true);                         // 深度仍在 → 不算全失效
    CHECK(r2.min_forward_distance_m > 1.2 && r2.min_forward_distance_m < 1.8);  // 仅由深度决定
    CHECK(r2.recommended_action == NavigationAction::FORWARD);

    // 可逆: 重新启用 → 悬崖判定恢复
    fusion.set_ultrasonic_enabled(true);
    auto r3 = fusion.fuse();
    CHECK(r3.cliff_detected == true);
    CHECK(r3.recommended_action == NavigationAction::STOP);
}


// ===== S1 (N2): 退化期降级链 (2026-09-26) =====
// 降级期间三级反应线收紧: 10/25/50 -> 20/40/70cm (超声与融合两套阶梯同口径);
// 未降级 (默认) 与接入前逐位一致。先红后绿: 实现前"降级"一侧的断言全部失败。
static void test_degraded_ladder_tightened() {
    AstraProDriver astra(true);
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);
    std::unordered_map<std::string, FusedObstacle> none;
    // 超声 18cm: 基线 BACKWARD (<=25) vs 降级 STOP (<=20)
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 18.0, false, none, true, true)
          == NavigationAction::BACKWARD);
    fusion.set_depth_degraded(true);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 18.0, false, none, true, true)
          == NavigationAction::STOP);
    // 超声 30cm: 基线不触发 (30>25, 融合远) -> FORWARD; 降级 <=40 -> BACKWARD
    fusion.set_depth_degraded(false);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 30.0, false, none, true, true)
          == NavigationAction::FORWARD);
    fusion.set_depth_degraded(true);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 30.0, false, none, true, true)
          == NavigationAction::BACKWARD);
    // 复原: 置回 false 与基线一致 (自动解除由胶水包驱动; 此处验证状态可逆)
    fusion.set_depth_degraded(false);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 8.0, 18.0, false, none, true, true)
          == NavigationAction::BACKWARD);
}

static void test_degraded_fused_ladder_and_choose_direction() {
    AstraProDriver astra(true);
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);
    auto mk3 = [](double l, double c, double r) {
        std::unordered_map<std::string, FusedObstacle> m;
        for (auto kv : {std::make_pair("left", l), std::make_pair("center", c), std::make_pair("right", r)}) {
            FusedObstacle o; o.direction = kv.first; o.distance_m = kv.second; m[kv.first] = o;
        }
        return m;
    };
    std::unordered_map<std::string, FusedObstacle> none;
    // 融合 0.60m (基线 60>50 -> FORWARD); 降级 (<=70) -> 选向: 中心 0.60<0.70 -> 右比较 -> TURN_RIGHT
    CHECK(SensorFusionTestAccess::determine_action(fusion, 0.60, 400.0, false, none, true, true)
          == NavigationAction::FORWARD);
    fusion.set_depth_degraded(true);
    auto obs60 = mk3(0.60, 0.60, 0.60);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 0.60, 400.0, false, obs60, true, true)
          == NavigationAction::TURN_RIGHT);
    // 全 0.35m: 基线 -> TURN_RIGHT (右 >25cm); 降级 -> BACKWARD (右 <=40cm 不达转向线)
    fusion.set_depth_degraded(false);
    auto obs35 = mk3(0.35, 0.35, 0.35);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 0.35, 400.0, false, obs35, true, true)
          == NavigationAction::TURN_RIGHT);
    fusion.set_depth_degraded(true);
    CHECK(SensorFusionTestAccess::determine_action(fusion, 0.35, 400.0, false, obs35, true, true)
          == NavigationAction::BACKWARD);
}

int main() {
    test_cliff_valid_check();
    test_min_forward_valid_filter();
    test_layer_fusion_boundaries();
    test_bottom_invalid_no_negative_distance();
    test_confidence_ultra_invalid_keeps_astra();
    test_invalid_ultrasonic_no_false_critical();
    test_layer_fusion_take_nearest();
    test_choose_direction_uses_current_frame();
    test_quality_score_not_halved();
    test_choose_direction_ignores_invalid_direction();
    test_fuse_direction_marks_blind_direction_invalid();
    test_all_invalid_fail_closed();
    test_front_blind_with_bottom_valid_is_conservative();
    test_bottom_invalid_sets_valid_false();
    test_is_fall_risk_fail_closed();           // ALG-1
    test_is_fall_risk_uses_injected_bottom();  // 方案A: 注入优先口径
    test_hw_unavailable_no_random();           // ALG-4
    test_determine_environment_depth_proxy_default();  // ALG-3
    test_depth_frame_usable_guard();           // FIX-01
    test_terrain_near_block_stops();            // 路1: 近场地形 → STOP
    test_terrain_mid_slow_and_dodge();          // 路1: 中距地形 → 降速/让开
    test_terrain_corridor_bounds_and_negative_points();  // 路1: 走廊边界 + 负障碍点
    test_terrain_threshold_edges();             // 路1: 边界对加固(#4)
    test_ultrasonic_disabled_removes_cliff_layer();      // v2.5: 超声链路可禁用 (真机无硬件)
    test_degraded_ladder_tightened();                    // S1 (N2): 降级阈值收紧
    test_degraded_fused_ladder_and_choose_direction();   // S1 (N2): 融合阶梯 + 选向收紧

    std::cout << "passed=" << g_passed << " failed=" << g_failed << std::endl;
    return g_failed == 0 ? 0 : 1;
}
