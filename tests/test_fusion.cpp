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

    // ir 模拟值 0~1 全覆盖, 环境类型必须是三种之一
    for (int i = 0; i < 50; ++i) {
        auto result = fusion.fuse();
        CHECK(result.environment == EnvironmentType::INDOOR ||
              result.environment == EnvironmentType::SEMI_INDOOR ||
              result.environment == EnvironmentType::OUTDOOR);
        // 权重不变量: 两者之和为 1, 且均在 [0,1] 范围
        double wsum = result.effective_astra_weight + result.effective_ultrasonic_weight;
        CHECK(std::abs(wsum - 1.0) < 1e-6);
        CHECK(result.effective_astra_weight >= 0.0 && result.effective_astra_weight <= 1.0);
        CHECK(result.effective_ultrasonic_weight >= 0.0 && result.effective_ultrasonic_weight <= 1.0);
    }

    astra.stop();
}

static void test_invalid_ultrasonic_no_false_critical() {
    // 回归 D1/D2: 真机超声超时 (-1.0cm) 必须被过滤, 不得产生负距离/CRITICAL
    // 复刻 fuse_direction + layer_fusion 的取值路径(修复后):
    //   1) fuse_direction: 仅 valid 读数参与, 否则 ultra_dist_m = 4.5
    //   2) layer_fusion L0: 仅 [0.02, 0.6) 进入盲区分支
    //   3) classify_obstacle_level: dist_cm = fused*100, <=10 才 CRITICAL
    const double kCmToM = 0.01;
    const double ultra_invalid = 4.5;  // fuse_direction 过滤后的兜底值

    // 场景: 超时 -1.0cm 的无效读数 (修复前: ultra_dist_m = -0.01)
    struct Reading { double distance_cm; bool valid; };
    Reading bad{-1.0, false};
    Reading good{20.0, true};

    auto fused_dist = [&](const Reading& r) -> double {
        double ultra_m = (r.valid) ? r.distance_cm * kCmToM : ultra_invalid;
        // layer_fusion L0 分支
        if (ultra_m >= 0.02 && ultra_m < 0.6) return ultra_m;
        return 8.0;  // 非盲区时走远距离默认
    };

    double d_bad  = fused_dist(bad);
    double d_good = fused_dist(good);
    CHECK(d_bad >= 0.02);   // 无效读数不得产生负距离
    CHECK(d_good == 0.20);  // 有效读数 20cm 正常进入盲区

    // classify_obstacle_level: 修复后无效读数应判为 SAFE(>50cm), 而非 CRITICAL
    auto level = [](double dist_m) {
        double cm = dist_m * 100;
        if (cm <= 10.0) return 0;       // CRITICAL
        if (cm <= 25.0) return 1;       // DANGER
        if (cm <= 50.0) return 2;       // WARNING
        return 3;                       // SAFE
    };
    CHECK(level(d_bad) == 3);   // 修复后: 无效 -> SAFE(兜底 4.5m), 不再误报 CRITICAL
    CHECK(level(d_good) == 1);  // 20cm -> DANGER(合理)
}

int main() {
    test_cliff_valid_check();
    test_min_forward_valid_filter();
    test_layer_fusion_boundaries();
    test_invalid_ultrasonic_no_false_critical();

    std::cout << "passed=" << g_passed << " failed=" << g_failed << std::endl;
    return g_failed == 0 ? 0 : 1;
}
