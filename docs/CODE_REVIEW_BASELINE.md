# mechdog_navigation 代码审查基线

> 三份独立审查的合并去重清单（Claude / Cline / 人工核查）
> 审查对象：`main` 分支全部 9 个文件（`config.h`、`sensor_astra.h/.cpp`、`sensor_ultrasonic.h/.cpp`、`sensor_fusion.h/.cpp`、`CMakeLists.txt`、`README.md`）
> 状态约定：每项修复后请将 `[ ]` 改为 `[x]` 并附提交号

---

## 审查方法说明

- **Claude 审查**：对 `capture_frame()`/`get_latest_frame()` 生产者-消费者数据流做了逐行追踪；抓到安全 bug（cliff valid）。
- **Cline 审查（VSCode）**：覆盖面广；抓到"环境自适应不可验证"这一功能级失效。
- **人工核查**：对全部源码做数值级推理；抓到两处未定义行为（UB），并修正 Cline 对 D 项后果方向的误判。
- 三方独立结论中**重复出现的问题**可信度最高，已在下方标注来源。

---

## 问题清单（按严重度排序）

### 🔴 阻塞级（项目当前无法编译 / 核心功能不存在）

#### 1. CMakeLists.txt 引用不存在的源文件，无法编译
- **位置**：`CMakeLists.txt`（`SOURCES` 列表）
- **问题**：引用了 `main.cpp` 与 `path_planner.cpp`，但仓库中这两个文件**不存在**（已逐一验证 404）。任何 `cmake --build .` 直接失败。`config.h` 已预留 `PlannerConfig`（DWA 参数），但无对应实现；`sensor_fusion.cpp` 输出的 `NavigationAction` 目前也没有任何消费方。
- **来源**：Claude ✅ / Cline ✅ / 人工 ✅
- **修复**：补齐 `main.cpp`（模拟模式演示入口 + 简单日志）+ `path_planner.cpp`，或在实现前从 `SOURCES` 移除。

#### 2. OpenNI2 真机模式是空壳，`USE_OPENNI2=ON` 也拿不到真实深度数据
- **位置**：`sensor_astra.cpp` `capture_frame()`（非模拟分支）、`CMakeLists.txt`
- **问题**：非模拟分支仅返回 `valid=false` 的空帧；全文无 `#include <openni2/OpenNI.h>`、无 `openni::Device` / 深度流 / 深度图转换等任何硬件代码。`USE_OPENNI2` 开关的唯一实际作用是让 `find_package(OpenNI2 REQUIRED)` 失败。README 宣称的"完整硬件模式"无法兑现。
- **来源**：Claude ✅ / Cline ✅ / 人工 ✅
- **修复**：实现 OpenNI2 采集（或明确标注"硬件接口待实现"，避免误导）；同样需评估 `USE_WIRINGPI` 分支的 GPIO 代码是否已在真机验证。

#### 3. 模拟帧硬编码 `environment = INDOOR`，环境自适应功能在模拟中不可验证
- **位置**：`sensor_astra.cpp` `simulate_frame()`（第 125-126 行）
- **问题**：模拟帧硬编码 `environment = INDOOR`、`ambient_light_level = 0.1`；而 `estimate_ambient_light()` 与 `classify_environment()` **从未被调用**。后果：`get_environment_weights()` 的三套权重（室内 0.8/半室内 0.5/室外 0.1）在模拟模式下**永远走 indoor 分支**，该功能既无真机、也无模拟路径可验证。
- **来源**：Cline（唯一实质增量）
- **修复**：在模拟模式中随机/参数化 `ambient_light_level` 并调用 `classify_environment()`；或为 `AstraFrame.environment` 提供显式注入。

### 🔴 安全级（影响避障/防跌落决策的正确性）

#### 4. `get_cliff_detected()` 未检查读数有效性 —— 超时被误判为"安全"
- **位置**：`sensor_ultrasonic.cpp`（`UltrasonicArrayData::get_cliff_detected`）
- **问题**：`return bottom.distance_cm > 30.0;` 未检查 `bottom.valid`。真机模式下 `measure_distance()` 超时返回 `-1.0`，`-1.0 > 30.0` 为假 → **判定为无悬崖/安全**。而回波超时恰恰常意味着"量程内没有能反射的地面"（深坑/大台阶）。这与自定的"宁可误判也不漏判"原则**正好相反**，是全部问题中最危险的一个。
- **来源**：Claude（本清单最高价值发现）
- **修复**：
  ```cpp
  return !bottom.valid || bottom.distance_cm > 30.0;  // 无效读数默认有风险
  ```

#### 5. `get_min_forward_distance_cm()` 同样未检查有效性 —— 真机下过度保守急停
- **位置**：`sensor_ultrasonic.cpp`（`UltrasonicArrayData::get_min_forward_distance_cm`）
- **问题**：`std::min({front_left.distance_cm, ...})` 未检查 `.valid`。真机下任一传感器超时返回 `-1.0`，参与 `std::min` 后结果必 ≤ 10cm → 触发 `STOP`。**真实后果是假阳性（动不动急停），而非漏检撞墙**——Cline 指出该函数，但其原始论证方向（"误判导致撞上去"）有误，此处已修正。
- **来源**：Cline 指出函数 / 人工修正后果方向
- **修复**：过滤 `valid=false` 的读数后再取最小值；全部无效时返回量程上限。

### 🟠 正确性级（未定义行为 / 确定性错误）

#### 6. `get_environment_weights().at()` 返回悬垂引用（UB）
- **位置**：`sensor_fusion.cpp` `get_adaptive_weights()`
- **问题**：`const auto& base = get_environment_weights().at(env_to_key(env_type));` —— `get_environment_weights()` 返回**临时** `unordered_map`，`.at()` 返回的引用指向临时对象，语句结束后即悬挂。属**未定义行为**：换编译器或开优化可能读到垃圾权重甚至崩溃。"测试通过"恰恰是因为 UB 可以碰巧工作。
- **来源**：人工（Claude / Cline 均未抓到）
- **修复**：先拷贝到局部变量：
  ```cpp
  auto base = get_environment_weights().at(env_to_key(env_type));
  ```

#### 7. 超声波测量间隔计算时间单位错乱（UB）
- **位置**：`sensor_ultrasonic.cpp` `UltrasonicSensor::measure()`
- **问题**：`last_measure_time_` 存的是 `time_since_epoch().count()`（**纳秒**数，约 1.7e18），读取时却被当作 `duration<double>`（**秒**）再 `duration_cast` 到纳秒。第二次 `measure()` 起差值约 1.7e18 秒量级 → 要么 `sleep_for` 巨长（挂起），要么整数溢出（UB）。模拟模式"碰巧能跑"靠的是溢出回绕的巧合。
- **来源**：人工（Cline 复报为 B 项，但其"第一次调用 elapsed≈0"细节有误：实际 `last_measure_time_=0` → time_point 回到 epoch → elapsed≈系统运行时长，不 sleep）
- **修复**：直接保存 `steady_clock::time_point`：
  ```cpp
  std::chrono::steady_clock::time_point last_measure_{};
  // ...
  if (last_measure_.time_since_epoch().count() != 0) {
      auto elapsed = std::chrono::steady_clock::now() - last_measure_;
      if (elapsed < std::chrono::milliseconds(50))
          std::this_thread::sleep_for(std::chrono::milliseconds(50) - elapsed);
  }
  last_measure_ = std::chrono::steady_clock::now();
  ```

### 🟠 设计 / 依赖级

#### 8. `boost::optional` 是未声明的隐藏依赖
- **位置**：`sensor_fusion.h/.cpp`
- **问题**：使用 `boost::optional`，但 CMakeLists 无 `find_package(Boost)`，README 依赖列表也未提及。项目已要求 C++17，`std::optional` 完全可替代，无必要引入一整个 Boost 依赖（换机器直接编译不过）。
- **来源**：Claude ✅ / Cline ✅ / 人工 ✅
- **修复**：`#include <optional>` + `std::optional`，并同步 CMakeLists 与 README。

#### 9. 底部传感器无独立通路，"最高优先级"未落实
- **位置**：`sensor_ultrasonic.cpp` `read_all()`
- **问题**：bottom 仅在同一 `read_all()`、同一把锁中**顺序排第一**，与前方三路共享同一次 ~120ms 轮询周期，刷新率未单独保障。悬崖检测的响应延迟与前方避障同数量级，违背"底部独立、优先级最高"的设计意图。
- **来源**：Claude
- **修复**：落地已敲定的新架构——`bottom_loop()` 独立线程（`BOTTOM_POLL_HZ=20`）+ `front_loop()` 单线程分时轮询 + `is_fall_risk()` 独立接口（注意其保守设计：读数无效默认有风险，与问题 4 修复一致）。

#### 10. Echo 忙等轮询受调度抖动影响
- **位置**：`sensor_ultrasonic.cpp` `measure_distance()`（`#ifdef USE_WIRINGPI` 分支）
- **问题**：Linux 非实时系统，`while (digitalRead(...))` 忙等轮询的调度抖动直接变成测距误差（1ms 抖动 ≈ 17cm）。
- **来源**：Claude
- **修复**：改边沿中断 + 时间戳（当前为占位注释，待确定主控方案后实现）。

#### 11. wiringPi 版本/来源未声明
- **位置**：`CMakeLists.txt` / `README.md`
- **问题**：原版 wiringPi（Gordon Henderson）2019 年起停止维护，Pi 4 官方支持停在 2.52；社区维护 fork 为 `github.com/WiringPi/WiringPi`。部署到树莓派 4B+ 时可能"库装上了但符号对不上"。
- **来源**：Claude
- **修复**：README 明确指定依赖的 wiringPi 来源与版本。

### 🟢 质量级

#### 12. 死代码
- **位置**：`sensor_astra.cpp`（`estimate_ambient_light`/`classify_environment`）、`sensor_ultrasonic.cpp`（`get_available_direction`）、`sensor_fusion.h`（`distance_history_`/`kHistoryMaxLen` 声明未用）、`sensor_astra.cpp`（`get_obstacle_distances` 无调用方）
- **修复**：删除或接通。其中 `estimate_ambient_light`/`classify_environment` 的接通见问题 3。

#### 13. 无测试文件却宣称"测试通过"
- **位置**：提交 `7e15037` 信息（"5 项单元测试通过"）
- **问题**：仓库中没有任何测试文件，且无 `main` 入口，无法复现该结论。验证声明不可信会连带削弱后续所有"已验证"说法的可信度。
- **修复**：补齐 `main.cpp`（问题 1）后，为 `layer_fusion`/`calc_confidence`/`determine_action`/`get_cliff_detected` 等纯逻辑函数添加可运行的单元测试，再写"测试通过"。

#### 14. 可忽略项（记录备查，不处理）
- Cline C 项：模拟模式 4 个 `uniform_real_distribution` 实例——功能无误，仅风格，无需处理。
- Cline E 项中"CMake if/endif 分支重复"——逻辑正确，非问题；其中"README 宣称功能未实现"部分已并入问题 2。

---

## 修复优先级建议

| 顺序 | 条目 | 理由 |
|------|------|------|
| 1 | #4 悬崖 valid 检查 | 安全，一行改动，最高风险 |
| 2 | #1 缺失源文件 | 恢复可编译 |
| 3 | #6 #7 两处 UB | 正确性，换环境即炸 |
| 4 | #9 底部独立线程 | 落实已敲定设计，落地进仓库 |
| 5 | #8 std::optional | 顺手替换，消依赖 |
| 6 | #5 min_forward valid | 安全，随 #4 一并改 |
| 7 | #3 环境自适应可验证 | 功能级失效 |
| 8 | #2 OpenNI2 实现或标注 | 消除虚假宣称 |
| 9 | #10 #11 硬件侧 | 待主控方案确定 |
| 10 | #12 #13 质量项 | 打磨 |

---

## 审查结论（一句话）

**设计思路（分层融合 + 环境自适应 + 悬崖检测 + 模拟模式）方向正确、可读性好，但当前是"算法骨架"而非可运行项目：三处阻塞级问题使其无法编译、真机与模拟均无法兑现文档宣称的能力；两处 UB 与一处安全 bug 是正确性硬伤。修复顺序按上表，其中 #4 与 #6/#7 优先。**
