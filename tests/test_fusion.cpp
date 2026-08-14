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
 *   g++ -std=c++17 test_fusion.cpp sensor_ultrasonic.cpp sensor_astra.cpp sensor_ir.cpp sensor_fusion.cpp -lpthread -o test_fusion
 *   ./test_fusion
 */
#include "../sensor_fusion.h"
#include "../sensor_ultrasonic.h"
#include "../sensor_astra.h"
#include "../sensor_ir.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <set>

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
    static double calc_confidence(SensorFusion& f, double ultra_m, double astra_m,
                                  bool astra_valid, double astra_w, double ultra_w) {
        return f.calc_confidence(ultra_m, astra_m, astra_valid, astra_w, ultra_w);
    }
};
} // namespace mechdog

static void test_confidence_ultra_invalid_keeps_astra() {
    // R-1: 超声无效(兜底 4.5)不得参与一致性计算——兜底值不是真实距离,
    // 原实现 |4.5-8.0|/2>1 → consistency=0 → confidence 被错误压到 0
    AstraProDriver astra(true);
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);

    // 超声无效 + Astra 8m: 修复后 = 仅 Astra 单独可信度 0.8
    double conf_invalid = SensorFusionTestAccess::calc_confidence(
        fusion, /*ultra_m=*/4.5, /*astra_m=*/8.0, /*astra_valid=*/true, 0.1, 0.9);
    CHECK(std::abs(conf_invalid - 0.8) < 1e-6);

    // 对照: 超声有效且与 Astra 接近 (2.0 vs 2.5): consistency = 1-0.5/2 = 0.75
    double conf_valid = SensorFusionTestAccess::calc_confidence(
        fusion, /*ultra_m=*/2.0, /*astra_m=*/2.5, /*astra_valid=*/true, 0.5, 0.5);
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

int main() {
    test_cliff_valid_check();
    test_min_forward_valid_filter();
    test_layer_fusion_boundaries();
    test_confidence_ultra_invalid_keeps_astra();
    test_invalid_ultrasonic_no_false_critical();

    std::cout << "passed=" << g_passed << " failed=" << g_failed << std::endl;
    return g_failed == 0 ? 0 : 1;
}
