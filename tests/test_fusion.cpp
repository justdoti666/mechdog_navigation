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
    // M1: 动作决策带传感器有效性 (fail-closed)
    static NavigationAction determine_action(
        SensorFusion& f, double min_forward_m, double min_ultrasonic_cm,
        bool cliff_detected,
        const std::unordered_map<std::string, FusedObstacle>& obstacles,
        bool sensors_valid) {
        return f.determine_action(min_forward_m, min_ultrasonic_cm, cliff_detected,
                                  obstacles, sensors_valid);
    }
    static bool all_sensors_invalid(SensorFusion& f, const AstraFrame& frame,
                                    const UltrasonicArrayData& ultra) {
        return f.all_sensors_invalid(frame, ultra);
    }
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

    std::cout << "passed=" << g_passed << " failed=" << g_failed << std::endl;
    return g_failed == 0 ? 0 : 1;
}
