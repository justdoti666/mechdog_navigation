# mechdog_navigation 修复方案（FIX PLAN）

> 基于三份独立审查（Claude / Cline / 人工核查）合并去重后的完整修复方案。
> 前置文档：`docs/CODE_REVIEW_BASELINE.md`（问题清单与来源）
> 状态约定：每完成一项，将 `[ ]` 改为 `[x]` 并附提交号。

> **状态总览（2026-08）**：F1/F3/F4/F5/F6/F7/F8/F12/F13 及 D1/D2/D3 缺陷修复已落地并推送（commit a6fa4fb / a0a922e）；F9（底部独立线程）暂缓待硬件阶段；F2（OpenNI2 真机）/F10（中断测距）待硬件；F14（画面反馈）挂起待决策。TSL2591 已取消购买，环境光强判定默认走 `estimate_ambient_light()` 深度图代理，`sensor_ir` 保留为可选增强。

---

## 修复总览（按优先级排序）

| 顺序 | 编号 | 问题 | 级别 | 涉及文件 |
|------|------|------|------|----------|
| 1 | F4 | 悬崖检测漏判（`bottom.valid` 未检查） | 🔴 安全 | sensor_ultrasonic.cpp |
| 2 | F1 | 缺失 main.cpp / path_planner.cpp，无法编译 | 🔴 阻塞 | CMakeLists.txt + 新增 2 文件 |
| 3 | F6 | `get_environment_weights().at()` 悬垂引用（UB） | 🟠 正确性 | sensor_fusion.cpp |
| 4 | F7 | 超声波测量间隔时间单位错乱（UB） | 🟠 正确性 | sensor_ultrasonic.h/.cpp |
| 5 | F8 | `boost::optional` 隐藏依赖 → 换 `std::optional` | 🟠 依赖 | sensor_fusion.h/.cpp、README |
| 6 | F5 | `get_min_forward_distance_cm()` 未检查 valid | 🟠 安全 | sensor_ultrasonic.cpp |
| 7 | F3 | 环境自适应不可验证：光强输入链路缺失 | 🔴 功能 | ✅ `sensor_ir` 驱动已生成（E:\33\mechdog_navigation_fixed\），待接入 fusion + 硬件组标定 |
| 8 | F9 | 底部传感器无独立通路 | 🟠 设计 | **暂缓，未实施**（2026-08 标记；待硬件阶段） |
| 9 | F2 | OpenNI2 真机模式空壳 | 🔴 阻塞(硬件) | sensor_astra.cpp、CMakeLists |
| 10 | F10 | Echo 忙等轮询受调度抖动 | 🟠 精度 | sensor_ultrasonic.cpp |
| 11 | F11 | wiringPi 版本/来源未声明 | 🟡 部署 | README |
| 12 | F12 | 死代码清理 | 🟢 质量 | sensor_astra.cpp 等 |
| 13 | F13 | 无测试文件却宣称"测试通过" | 🟢 质量 | 新增 tests/ |
| 14 | F14 | 画面反馈功能待定（deco 已拆；候选 Astra RGB 顶替） | 🟠 待定 | 挂起，待用户决策 |

---

## F4 悬崖检测漏判 —— `bottom.valid` 未检查 【🔴 安全 · 第一优先】

**位置**：`sensor_ultrasonic.cpp` → `UltrasonicArrayData::get_cliff_detected()`

**现状**：
```cpp
bool UltrasonicArrayData::get_cliff_detected() const {
    // 底部传感器读数 > 30cm 认为有跌落风险
    return bottom.distance_cm > 30.0;   // ← 没看 bottom.valid
}
```

**问题**：真机模式下 `measure_distance()` 超时返回 `-1.0`，`-1.0 > 30.0` 为假 → 判为"安全"。而回波超时恰恰常意味着"量程内没有能反射的地面"（深坑/大台阶）——与"宁可误判也不漏判"原则正好相反。

**修复**（一行）：
```cpp
bool UltrasonicArrayData::get_cliff_detected() const {
    // 无效读数（超时/无回波）默认判定为有风险，宁可误判也不漏判
    return !bottom.valid || bottom.distance_cm > 30.0;
}
```

**验收**：底部 `valid=false` 时 `get_cliff_detected()` 返回 `true` → `determine_action()` 返回 `STOP`。

---

## F1 缺失源文件，项目无法编译 【🔴 阻塞】

**位置**：`CMakeLists.txt`（`SOURCES` 列表）、新增 `main.cpp` + `path_planner.cpp`

**问题**：`SOURCES` 引用了仓库中不存在的 `main.cpp` 与 `path_planner.cpp`，`cmake --build .` 直接失败；`config.h` 预留的 `PlannerConfig`（DWA）无实现；`sensor_fusion.cpp` 输出的 `NavigationAction` 无消费方。

**修复**：分两步落地。

### 1a. 先恢复可编译（最小改动）
从 `SOURCES` 中临时移除 `main.cpp`/`path_planner.cpp`，并把构建目标改为静态库（纯算法库，无入口）：
```cmake
add_library(${PROJECT_NAME}_core STATIC
    sensor_ultrasonic.cpp
    sensor_astra.cpp
    sensor_fusion.cpp
)
```
同时新增 `main.cpp`（模拟模式演示入口）以恢复可执行目标。`main.cpp` 骨架：
```cpp
#include "sensor_astra.h"
#include "sensor_ultrasonic.h"
#include "sensor_fusion.h"
#include <iostream>
#include <csignal>

using namespace mechdog;

static volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

int main() {
    std::signal(SIGINT, on_signal);

    AstraProDriver astra(/*use_simulated=*/true);
    auto layout = get_ultrasonic_layout();
    UltrasonicArrayDriver ultrasonic(layout);
    SensorFusion fusion(&astra, &ultrasonic);

    astra.start();

    while (!g_stop) {
        auto result = fusion.fuse();
        std::cout << "[" << result.timestamp << "] action="
                  << static_cast<int>(result.recommended_action)
                  << " min_forward=" << result.min_forward_distance_m << "m"
                  << " cliff=" << (result.cliff_detected ? "YES" : "no")
                  << " env=" << static_cast<int>(result.environment) << "\n";
        for (const auto& kv : result.obstacles) {
            std::cout << "  " << kv.first << ": " << kv.second.distance_m << "m"
                      << " (conf " << kv.second.confidence << ", " << kv.second.source << ")\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    astra.stop();
    return 0;
}
```

### 1b. 实现 `path_planner.cpp`（DWA 骨架，待验证后迭代）
- 输入：`FusionResult`（融合结果）
- 输出：线速度 / 角速度目标（`PlannerConfig` 中的 `max_linear_velocity`、`max_angular_velocity`、`obstacle_safety_dist_m`、`emergency_stop_dist_m`）
- 第一版可做反应式映射（依据 `recommended_action`），后续再迭代真实 DWA 采样。

**验收**：`cmake .. && cmake --build .` 在模拟模式下编译通过、可运行并持续输出决策日志。

---

## F6 `get_environment_weights().at()` 悬垂引用（UB） 【🟠 正确性】

**位置**：`sensor_fusion.cpp` → `SensorFusion::get_adaptive_weights()`

**现状**：
```cpp
const auto& base = get_environment_weights().at(env_to_key(env_type));
```

**问题**：`get_environment_weights()` 返回**临时** `unordered_map`，`.at()` 返回的引用指向临时对象，语句结束即悬挂 → **未定义行为**（换编译器/开优化可能读垃圾或崩溃）。

**修复**（改为拷贝）：
```cpp
auto base = get_environment_weights().at(env_to_key(env_type));  // 值拷贝，消除悬垂
double astra_w = base.astra;
double ultra_w = base.ultrasonic;
```

**验收**：开 `-O2` + ASan/UBSan 跑模拟循环无报告。

---

## F7 超声波测量间隔时间单位错乱（UB） 【🟠 正确性】

**位置**：`sensor_ultrasonic.h`（成员声明）、`sensor_ultrasonic.cpp`（`UltrasonicSensor::measure()`）

**现状**：`last_measure_time_` 是 `double`，存的是 `time_since_epoch().count()`（**纳秒**数，约 1.7e18），读取时却当作 `duration<double>`（**秒**）再 `duration_cast` 到纳秒 → 单位错乱，差值约 1.7e18 秒量级 → 巨长 sleep 或整数溢出（UB）。模拟模式"碰巧能跑"靠的是溢出回绕。

**修复**：
```cpp
// sensor_ultrasonic.h
#include <chrono>
...
private:
    std::chrono::steady_clock::time_point last_measure_{};

// sensor_ultrasonic.cpp —— measure()
UltrasonicReading UltrasonicSensor::measure() {
    auto now = std::chrono::steady_clock::now();
    if (last_measure_.time_since_epoch().count() != 0) {
        auto elapsed = now - last_measure_;
        if (elapsed < std::chrono::duration<double>(MIN_INTERVAL_SEC)) {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(MIN_INTERVAL_SEC) - elapsed);
        }
    }
    double timestamp = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    double distance_cm = measure_distance();
    last_measure_ = std::chrono::steady_clock::now();
    // ... 其余不变
}
```

**验收**：连续两次 `measure()` 的间隔被正确约束到 ≥50ms（用日志/计时验证）。

---

## F8 `boost::optional` 隐藏依赖 → 换 `std::optional` 【🟠 依赖】

**位置**：`sensor_fusion.h`、`sensor_fusion.cpp`

**问题**：代码用 `boost::optional`，但 CMakeLists 无 `find_package(Boost)`、README 未提及；项目已是 C++17，`std::optional` 可完全替代。

**修复**：
- `sensor_fusion.h`：`#include <boost/optional.hpp>` → `#include <optional>`；`boost::optional<FusionResult>` → `std::optional<FusionResult>`（共 2 处：`get_latest_result()` 返回值、`last_fusion_` 成员）。
- `sensor_fusion.cpp`：同名的 2 处替换。
- 同步更新 README 依赖列表（删除 Boost）。

**验收**：无 boost 头文件的环境下可编译。

---

## F5 `get_min_forward_distance_cm()` 未检查 valid 【🟠 安全】

**位置**：`sensor_ultrasonic.cpp` → `UltrasonicArrayData::get_min_forward_distance_cm()`

**问题**：真机下任一前向传感器超时返回 `-1.0`，参与 `std::min` 后必 ≤10cm → 触发 `STOP`。后果是**过度保守急停**（假阳性），而非漏检撞墙。

**修复**：过滤无效读数后取最小；全部无效时返回量程上限。
```cpp
double UltrasonicArrayData::get_min_forward_distance_cm() const {
    double best = 400.0;  // 量程上限
    bool any_valid = false;
    for (const auto* r : {&front_left, &front_center, &front_right}) {
        if (r->valid) {
            any_valid = true;
            best = std::min(best, r->distance_cm);
        }
    }
    return any_valid ? best : 400.0;
}
```

**验收**：模拟模式给某一路注入 `valid=false` 时，不再触发无意义的 `STOP`。

---

## F3 环境自适应不可验证：光强输入链路缺失 【🔴 功能失效 · 设计意图校正】

> **背景补充（2026-07/08，用户提供）**：环境自适应权重的设计意图是用底盘上的"红外传感器"测红外光强，防阳光/强红外导致 Astra Pro 结构光识别不准。**但 2026-08 核实：师兄（底盘固件作者）本人也不确定底盘上有没有红外传感器，仅确认有一个"红外摄像头"（deco，为遥控提供画面反馈）。** 因此"红外强度 → 权重"链路的硬件来源尚未坐实。实现本条目时，环境光强输入支持**多种数据源**，选可用的接入：
> - 红外摄像头图像亮度/红外通道平均强度（若为被动红外；若带主动补光 LED 则读数会被污染，不可直接用于测环境光）；
> - 独立光敏/环境光传感器（如光敏电阻、BH1750 之类）；
> - `estimate_ambient_light()`（深度图无效像素比例，代码里已有的廉价代理）；
> - 模拟值（开发调试用）。
>
> **决策更新（2026-08）：TSL2591 已取消购买**——师兄方案含激光雷达（不受强光影响），Astra Pro 失效时已有雷达兜底远距感知，"Astra 是否可信"判据不再必须；且 `estimate_ambient_light()`（深度图无效像素比）是零成本代理。**F3 光强判定默认走 `estimate_ambient_light()` 代理**；`sensor_ir` 驱动保留（模拟模式可用、真机 I2C 分支待硬件）作为可选增强，不阻塞。
>
> 本条目按真实设计意图重写。

**位置**：新增 `sensor_ir.h/.cpp`；改动 `sensor_fusion.cpp`（`get_adaptive_weights`）、`config.h`、`sensor_astra.cpp`（`simulate_frame`）

**问题**：
1. 仓库**没有红外传感器驱动**——"红外强度 → 深度相机/超声权重"这条核心数据链路完全不存在；
2. 代码里的环境判定走的是另一条路（`estimate_ambient_light()` 用深度图无效像素比猜光强），且该函数从未被调用，模拟帧硬编码 `environment = INDOOR` → 三档环境权重永远走 indoor 分支，功能无任何路径可验证。

**修复**（按真实硬件接入）：
1. **新增 `sensor_ir.h/.cpp`**：读取红外传感器强度并归一化到 `0.0~1.0`（`ambient_light_level`）。硬件接口待用户确认（GPIO 模拟量 / I2C / 串口），先实现模拟模式 + 接口占位：
```cpp
// sensor_ir.h（骨架）
namespace mechdog {
class InfraRedSensor {
public:
    explicit InfraRedSensor(bool use_simulated = true);
    // 归一化红外强度 0.0(暗) ~ 1.0(强光)，供融合权重使用
    double read_normalized_light();
private:
    bool use_simulated_;
    std::mt19937 rng_;
};
}
```
2. **`get_adaptive_weights()` 改用红外实测值**（替代/优先于帧内估算）：
```cpp
std::pair<double, double> SensorFusion::get_adaptive_weights(...) {
    double light = ir_sensor_->read_normalized_light();   // 0~1
    auto base = get_environment_weights().at(env_to_key(light_to_env(light)));
    // ... 其余逻辑不变
}
```
3. **`sensor_astra.cpp` 模拟帧**不再硬编码：`ambient_light_level` 由红外模拟值提供，`classify_environment()` 接通（若确定红外链路统一提供，则 `estimate_ambient_light()` 可删，见 F12）。
4. **config.h** 补充红外阈值（如 `ir_indoor_max` / `ir_outdoor_min`）。

**验收**：模拟循环中注入不同红外强度，`result.environment` 出现 `INDOOR`/`SEMI_INDOOR`/`OUTDOOR` 三档，且 `effective_astra_weight` 随之变化（室内高、室外低）。

---

## F14 画面反馈功能（原 deco 相机模组）待定 【🟠 待定 · 决策挂起】

> **背景补充（2026-08，用户提供）**：deco（小型相机模组，原为遥控提供画面反馈）**已被师兄拆除**，底盘上不再有它。遥控/远程画面反馈这一功能**是否还需要、用什么实现，用户待定**（候选：Astra Pro 的 RGB 流顶替 / 不提供画面反馈 / 另配相机）。本条目暂不实现，待用户决策。

**位置**：待定（视决策结果：可能新增驱动，也可能直接从需求中移除）

**问题**：deco 已拆，但"巡检/遥控是否需要画面反馈"尚未决策，因此无法确定是否需要任何实现。

**修复**：挂起。用户决策后再排期：
- 若**不需要**画面反馈 → 本条目关闭，无需代码；
- 若**用 Astra Pro RGB 顶替** → 在 `sensor_astra` 侧暴露 RGB 流接口（Astra Pro 本身输出 RGB + 深度），接入遥控/巡检回传；
- 若**另配相机** → 按新硬件再写驱动。

**验收**：待决策明确后补充。

---

## F9 底部传感器无独立通路 【🟠 设计 · ⏸️ 暂缓，未实施】

> **状态（2026-08）**：本条目**暂缓**，未在 `a6fa4fb` 修复提交中实施。当前代码仍为 `read_all()` 分时轮询架构（bottom 与前方三路绑在同一 ~120ms 周期）。F4（valid 检查）已修，但独立线程重构未做，待硬件阶段再排期。

**位置**：`sensor_ultrasonic.h` / `sensor_ultrasonic.cpp`

**问题**：bottom 仅在同一 `read_all()`、同一把锁中顺序排第一，与前方三路共享 ~120ms 周期，刷新率未单独保障，违背"底部独立、优先级最高"的设计意图。

**修复**（落地已敲定的新架构）：
- `bottom_loop()` 独立线程，`BOTTOM_POLL_HZ = 20`（50ms 周期），与 `front_loop()` 完全分开；
- 前方三路 `front_loop()` 单线程按 `kFrontOrder` 分时轮询（30ms 间隔，防串扰）；
- 暴露独立接口 `bool is_fall_risk() const`（内部即 F4 修复后的逻辑），融合模块优先查询它；
- `read_all()` 保留为兼容接口或移除（由调用方改为分别获取）。

**验收**：底部刷新率 ≈ 20Hz 独立于前方 ~8Hz；`is_fall_risk()` 触发后 `determine_action()` 无条件 `STOP`。

---

## F2 OpenNI2 真机模式空壳 【🔴 阻塞(硬件)】

**位置**：`sensor_astra.cpp` → `capture_frame()`、`CMakeLists.txt`

**问题**：非模拟分支仅返回 `valid=false` 空帧；无 `#include <openni2/OpenNI.h>`、无 `openni::Device`/深度流/转换代码；`USE_OPENNI2` 开关唯一实际作用是让 `find_package(OpenNI2 REQUIRED)` 失败。

**修复**（两条路线二选一）：
- **路线 A（推荐，先做）**：明确定位为"算法验证阶段"，README 与 CMake 注释标注"OpenNI2 硬件接口待实现"，`USE_OPENNI2` 选项默认保持 OFF，避免误导；
- **路线 B（有真机时）**：实现 OpenNI2 采集——初始化 `openni::Device`、启动深度流、`readFrame` 后转换为 `depth_map`（mm → `uint16_t`），接入 `capture_frame()` 的 else 分支，并保留 `estimate_ambient_light()` 做环境判定。

**验收（路线 A）**：README 不再宣称"完整硬件模式可用"。

---

## F10 Echo 忙等轮询受调度抖动 【🟠 精度】

**位置**：`sensor_ultrasonic.cpp` → `measure_distance()`（`#ifdef USE_WIRINGPI` 分支）

**问题**：Linux 非实时系统，`while (digitalRead(...))` 忙等的调度抖动直接变成测距误差（1ms 抖动 ≈ 17cm）。

**修复**：改为边沿中断 + 时间戳（需硬件/主控支持，当前为占位注释）：
- 在 `echo_pin_` 上注册上升沿/下降沿中断，用 `high_resolution_clock` 打时间戳；
- 两沿时间差 × 声速 / 2 = 距离；
- 待确定主控方案（是否加独立 MCU）后实现。

**验收**：真机多次测量同一障碍物，读数抖动 < 1cm。

---

## F11 wiringPi 版本/来源未声明 【🟡 部署】

**位置**：`README.md` 依赖节、`CMakeLists.txt`

**问题**：原版 wiringPi（Gordon Henderson）2019 年停止维护，Pi 4 官方支持停在 2.52；社区维护 fork 为 `github.com/WiringPi/WiringPi`。

**修复**：README 明确指定来源与版本，例如"WiringPi fork (github.com/WiringPi/WiringPi) 2.60+"，并给出安装命令。

---

## F12 死代码清理 【🟢 质量】

**位置**：`sensor_astra.cpp`（`estimate_ambient_light`/`classify_environment` 视 F3 落地情况保留或删除）、`sensor_ultrasonic.cpp`（`get_available_direction`）、`sensor_fusion.h`（`distance_history_`/`kHistoryMaxLen` 声明未用，已在 F8 版本中删除）、`sensor_astra.cpp`（`get_obstacle_distances` 无调用方）

**处理原则**：无调用方且无计划使用的删除；有计划使用的接通（如 F3 接通环境函数）。

---

## F13 无测试文件却宣称"测试通过" 【🟢 质量】

**位置**：提交 `7e15037` 信息（"5 项单元测试通过"）

**问题**：仓库无任何测试文件且无 main 入口，无法复现。

**修复**：恢复可编译（F1）后，为纯逻辑函数添加测试（可用 `tests/test_fusion.cpp` 单文件断言，或引入 Catch2/GoogleTest）：
- `layer_fusion()`：L0/L1/L2/L3 四段边界 + astra 无效 + 超声超量程
- `calc_confidence()`：一致性高低
- `determine_action()`：悬崖 / 临界 / 危险 / 安全
- `get_cliff_detected()`：`valid=false` 时必须判有风险（回归 F4）
- `get_min_forward_distance_cm()`：过滤无效读数（回归 F5）

**验收**：`ctest` 全绿后再在提交信息中声称"测试通过"。

---

## 落地顺序与验证命令

```bash
# 1. 恢复可编译（F1）
cmake .. && cmake --build .

# 2. 模拟模式运行（F1 + F3 验证）
./mechdog_navigation

# 3. 安全回归（F4/F5）
ctest   # 或运行 tests/test_fusion.cpp

# 4. UB 检测（F6/F7）
cmake .. -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -O2"
./mechdog_navigation
```

**建议一次只合一个 PR**，顺序：F4 ✅ → F1 ✅ → F6 ✅ → F7 ✅ → F8 ✅ → F5 ✅ → F3 ✅（含新增 `sensor_ir` 驱动，TSL2591）→ F9 ⏸️（暂缓）→ F2 → F10/F11 → F12 ✅ → F13 ✅；F14（画面反馈）挂起待决策，不阻塞其他项。
