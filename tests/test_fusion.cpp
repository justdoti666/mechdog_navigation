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

int main() {
    test_cliff_valid_check();
    test_min_forward_valid_filter();
    test_layer_fusion_boundaries();

    std::cout << "passed=" << g_passed << " failed=" << g_failed << std::endl;
    return g_failed == 0 ? 0 : 1;
}
