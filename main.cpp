/**
 * mechdog_navigation 主程序 (模拟模式演示入口)
 *
 * 默认以模拟模式运行: 无需硬件, 在 PC 上即可验证传感器融合与导航决策全链路。
 * 真机模式: 通过 CMake 选项 USE_WIRINGPI / USE_OPENNI2 编译, 并修改下方 use_simulated 参数。
 *
 * 构建:
 *   mkdir build && cd build
 *   cmake ..            # 模拟模式
 *   cmake --build .
 *   ./mechdog_navigation
 */
#include "sensor_astra.h"
#include "sensor_ultrasonic.h"
#include "sensor_ir.h"
#include "sensor_fusion.h"
#include "path_planner.h"

#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

using namespace mechdog;

static volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

int main() {
    std::signal(SIGINT, on_signal);

    // 模拟模式 (真机时: AstraProDriver astra(false); InfraRedSensor ir(false);)
    AstraProDriver astra(/*use_simulated=*/true);
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(/*use_simulated=*/true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);
    PathPlanner planner;

    astra.start();

    std::cout << "=== mechdog_navigation 模拟模式 ===" << std::endl;
    std::cout << "按 Ctrl+C 退出" << std::endl << std::endl;

    unsigned int tick = 0;
    while (!g_stop) {
        auto result = fusion.fuse();
        auto cmd = planner.plan(result);

        std::cout << "[" << std::fixed << std::setprecision(2)
                  << result.timestamp << "] tick=" << tick
                  << " env=" << static_cast<int>(result.environment)
                  << " astra_w=" << result.effective_astra_weight
                  << " ultra_w=" << result.effective_ultrasonic_weight
                  << " cliff=" << (result.cliff_detected ? "YES" : "no")
                  << " min_fwd=" << result.min_forward_distance_m << "m"
                  << " action=" << static_cast<int>(result.recommended_action)
                  << " vel=(" << cmd.linear << ", " << cmd.angular << ")"
                  << std::endl;

        for (const auto& kv : result.obstacles) {
            std::cout << "    " << kv.first
                      << ": " << std::setw(6) << kv.second.distance_m << "m"
                      << " conf=" << kv.second.confidence
                      << " lvl=" << static_cast<int>(kv.second.level)
                      << " [" << kv.second.source << "]" << std::endl;
        }
        std::cout << std::endl;

        ++tick;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    astra.stop();
    std::cout << "已退出" << std::endl;
    return 0;
}
