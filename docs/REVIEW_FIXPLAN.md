# mechdog_navigation 审查跟进修复方案（REVIEW FIX_PLAN）

> 审查对象：`justdoti666/mechdog_navigation` @ `e908429`
> （2026-08-21「分层融合 review 修复计划 v2.3 全量落地」），与 GitHub 远端完全同步。
> 前置文档：`docs/FIX_PLAN.md`（v2.3 及之前修复方案）、`docs/CODE_REVIEW_BASELINE.md`。
> 状态约定：每完成一项，将 `[ ]` 改为 `[x]` 并附提交号。

> **审查实测基线（证据）**：
> - 测试：WSL Ubuntu 24.04 / g++ 13.3.0 / `-std=c++20 -O1 -fsanitize=address,undefined`
>   构建并运行 `tests/test_fusion.cpp` → **`passed=677 failed=0`，ASan/UBSan 零报告**。
> - 完整可执行：`main.cpp`+`path_planner.cpp` 同选项编译通过。
> - `e908429` 声称新增的 3 个用例（`test_is_fall_risk_fail_closed` / `test_hw_unavailable_no_random` / `test_determine_environment_depth_proxy_default`）真实存在并计入 677。
> - 探针实测：`ultra_w = 1.0 - astra_w` 在 full-quality 下与基础权重互补一致（非 UB/非语义错）；启动期 `is_fall_risk()` 即刻返 `TRUE(STOP)`、~150ms 后转 `false`。

---

## 跟进状态（R1~R5 已执行, 本轮落地）

| 编号 | 状态 | 落地证据 |
|------|------|----------|
| R1 | ✅ 已修正 | c2617b6 `sensor_fusion.h` 头部注释改为"深度图代理默认 + 可选 IR 增强" |
| R2 | ✅ 已修正 | c2617b6 `sensor_ultrasonic.cpp` 前向刷新率注释按调用方节流标注（3.8Hz / ROS 10Hz） |
| R3 | ✅ 已修正 | c2617b6 `sensor_astra.cpp` "维持 30fps" → "取帧耗时约束 ≤20fps"（两处） |
| R4 | ✅ 按设计保留（文档注明） | 首帧急停=启动即验证 fail-closed；预热屏蔽反而使首个有效消息延迟至 ~435ms、逼近上游闸门 0.5s 超时（见下详述） |
| R5 | ✅ 已核对 + 集成测试 | b003845 safety_node 消费路径无自行 reinterpret；新增 `test/test_safety_fail_closed.cpp` 全链路注入断言 STOP/零速 |

---

## 一、已验证正确项（v2.3 全量落地，无需再修）

> 以下各项已在源码逐行核对 + 测试实测，结论为"提交声称与代码一致"，**不纳入待办**。

| 编号 | 提交声称 | 代码实锤 | 验证方式 |
|------|---------|---------|---------|
| ALG-1 | 底部悬崖独立 20Hz 线程 + `is_fall_risk()` fail-closed | `bottom_loop()` 独立线程（50ms）；`is_fall_risk()` 在 `!bottom_have_ \|\| !valid \|\| >thr` 返 true；`read_all()` 改用缓存 `get_bottom_reading()` | 源码 + `test_is_fall_risk_fail_closed` |
| ALG-3 | 环境判定单一链路 + `EnvironmentThresholds` 同源 | `determine_environment` 四档优先级；`light_to_env`/`classify_environment` 共用 `indoor_max/outdoor_min`；删 `ambient_light_level` 字段 | 源码 + `test_determine_environment_depth_proxy_default` |
| ALG-4 | 红外真机失败置 `hw_unavailable_` 不返随机值 | `read_normalized_light()` 在 `hw_unavailable_` 时返 -1；`is_real_available()` 区分"模拟/真机不可用" | 源码 + `test_hw_unavailable_no_random` |
| ALG-5 | 4.5 失效哨兵常量化 `kUltrasonicInvalidM` | `config.h::kUltrasonicInvalidM=4.5`，分层逻辑统一引用 | 源码 |
| ALG-6 | 速度 ramp（一阶限幅） | `path_planner.cpp::clamp_step`，`dt=0.2` 限幅 `±0.06m/s`/`±0.10rad/s` | 源码 |
| ALG-7 | Astra 采集节流按本帧耗时补偿 | `capture_loop` 用 `frame_period - elapsed` 动态 sleep | 源码 |
| ALG-8 | `AstraFrame` 默认 `valid=false` + `frame_seq` | 头文件默认 `false`/`0`；`capture_loop` 递增 | 源码 |
| ALG-9 | CMake GCC/Clang UTF-8 选项 | `GNUCXX/Clang` 加 `-finput-charset=UTF-8 -fexec-charset=UTF-8` | 源码 |
| ALG-10 | `get_environment_weights`/`get_ultrasonic_layout` 静态返引用 | 两函数均 `static const` 局部返回 | 源码 |
| F4/F5/F6/F7 | 悬崖 valid / min_forward valid / 悬垂引用值拷贝 / 时间间隔 time_point | 源码逐行核对一致 | 回归测试覆盖 |

**结论：v2.3 的全部 10 项 ALG 修复真实落地，无"声称未落地"项；提交信息可信任。**

---

## 二、待修复项（R1~R5）

> 均为非阻断项（文档/措辞/跨层一致性），无安全或正确性硬伤。建议合入下一轮 PR。

| 顺序 | 编号 | 问题 | 级别 | 涉及文件 | 状态 |
|------|------|------|------|----------|------|
| 1 | R1 | 过时注释仍写"TSL2591 环境红外"（F3 已取消购买） | 🟢 质量 | sensor_fusion.h | ✅ c2617b6 |
| 2 | R2 | 前向刷新率注释误导（称 16Hz，主循环 200ms 下实际 ~3.8Hz） | 🟢 质量 | sensor_ultrasonic.cpp | ✅ c2617b6 |
| 3 | R3 | ALG-7 注释"30fps"与真机实测 ≤20fps 不符 | 🟢 质量 | sensor_astra.cpp | ✅ c2617b6 |
| 4 | R4 | 启动期 fail-closed 首帧必 STOP ~150ms（ROS 接入时序建议） | 🟡 设计 | ROS 胶水包 / main.cpp | ✅ 按设计保留（文档注明） |
| 5 | R5 | ROS 胶水包落后 v2.3 4 提交，需核对 safety_node 是否按新 fail-closed 语义消费 `FusionResult` | 🟠 跨仓库一致性 | mechdog_navigation_ros | ✅ b003845 |

---

## R1 过时注释：sensor_fusion.h 仍写"TSL2591 环境红外" 【🟢 质量】

**位置**：`sensor_fusion.h:1-4`（文件头注释）

**问题**：注释写"TSL2591 环境红外"，但 F3 决策已取消 TSL2591 购买，环境光强判定默认走 `estimate_ambient_light()` 深度图代理，`sensor_ir` 仅为可选增强。文档与事实脱节，易让接手者误以为已硬件接入 TSL2591。

**修复**：文件头注释改为"深度图代理（默认）+ 可选 IR 增强（模拟模式可用，真机 I2C 分支待硬件）"，与 `config.h:144` 的 F3 决策说明对齐。

**落实**：✅ 已修正（c2617b6）—— 头部改为三行：融合对象 + "环境光强: 默认深度图代理（TSL2591 已取消购买, 见 config.h F3 决策）, sensor_ir 仅作可选增强"。

**验收**：`sensor_fusion.h` 头部不再出现"TSL2591 环境红外"误导性表述。纯静态改动，编译即可验证。✅ 编译通过（WSL g++ 13.3.0, 677 用例回归绿）。

---

## R2 前向刷新率注释误导 【🟢 质量】

**位置**：`sensor_ultrasonic.cpp:243`（`read_all()` 内注释）

**问题**：注释称"3 颗完整一轮 ~60ms → 更新率约 16Hz（底部已独立 20Hz）"。该 16Hz 是按"连续不停调用 `read_all()`"推导的；但主循环（`main.cpp` 循环体末尾 `sleep_for(200ms)`）下，前向实际刷新率 ≈ `1000/(60+200) ≈ 3.8Hz`，与 16Hz 差 4 倍，易误导性能评估。底部 20Hz 独立线程是真实的，不受影响。

**修复**：注释改为"单轮 ~60ms（连续调用时 ~16Hz；受主循环 200ms 节流实际前向 ~3.8Hz）；底部独立 20Hz 不受影响"。或在 `read_all()` 调用处明确标注主循环节奏。

**落实**：✅ 已修正（c2617b6）—— 按"调用方节流"分层标注：连续调用 ~16Hz；`main.cpp` 主循环 200ms → ~3.8Hz；ROS 融合线程 100ms 门控 → ~10Hz；底部 20Hz 不受影响。

**验收**：注释数值与真实调用节奏一致。纯静态改动。✅ 编译通过。

---

## R3 ALG-7 注释与真机实测不符 【🟢 质量 · 措辞级】

**位置**：`sensor_astra.cpp:66-67`、`sensor_astra.cpp:188-189` 一带（采集节流注释）

**问题**：注释称"固定 33ms 叠加会让真机跌破 30fps"，设计意图是补偿后维持 30fps。但真机 `capture_real()` 内 `10×5ms` 轮询取帧本身已耗时 ~50ms，超过帧周期 33ms，导致 `capture_loop` 中 `elapsed < frame_period` 实际常假、不 sleep，真机自然掉到 **≤20fps**。注释"30fps"是乐观值，名不副实。

**修复**：将"维持 30fps"改为"真机受 SDK 取帧耗时约束，实际 ≤20fps（节流仅在取帧 <33ms 时生效）"。属文档措辞修正，非缺陷。

**落实**：✅ 已修正（c2617b6）—— 两处：capture_loop 节流注释改为"取帧耗时 ~50ms 超帧周期 → 真机 ≤20fps, 补偿仅 <33ms 时生效, 真机下通常不 sleep"；`frame_period` 标"名义值"；80ms 等待注释补"真机取帧 ~50ms 亦有裕量"。

**验收**：注释准确反映实测帧率区间。纯静态改动。✅ 编译通过。

---

## R4 启动期 fail-closed 首帧必 STOP —— ROS 接入时序建议 【🟡 设计 · 知会】

**位置**：`sensor_ultrasonic.cpp:207-212`（`is_fall_risk()`）、`sensor_fusion.cpp:52`（`result.cliff_detected = ultrasonic_->is_fall_risk()`）

**问题**：`is_fall_risk()` 在 `bottom_have_==false`（底部线程尚未产出首帧）时返回 `true` → 首次 `fuse()` 必触发 `STOP`。探针实测：构造驱动后即刻查 `is_fall_risk()=TRUE`，~150ms（底部线程首帧产出）后才转 `false`。这是"宁可误判"的安全行为，**非缺陷**，但意味着机器人上电首帧/每次重启 fusion 都会误急停约 1 帧。

**修复建议（非代码强制，属集成规范）**：
- 在底盘尚未运动（或首次 `fusion.fuse()` 前）再启动 Astra/Ultrasonic 采集线程，使 `bottom_have_` 在首帧 `fuse()` 前已置位；
- 或在 ROS `safety_node` 层对启动后首个 ~200ms 的 `STOP` 做"预热屏蔽"（仅屏蔽首次，后续 fail-closed 仍生效）。
- 若认为首帧急停可接受（安全优先），本条标记为"按设计保留"，无需改代码。

**落实**：✅ 按设计保留（本轮决策, 2026-08）—— 理由：
1. 首帧急停是"启动即验证 fail-closed"：机器人启动时通常静止，上游闸门（`cmd_vel_safety_gate_node`）对首个 `cmd_vel` 也有 0.5s 超时兜底，一帧 STOP 无实害；
2. 若加"预热屏蔽"（屏蔽首个 200ms STOP）等价于在启动窗口内放松 fail-closed 语义，与本次 review 主题（杜绝 fail-open 重现）相悖；
3. 若采用"延迟首次 fuse()"（等 bottom 首帧）：真机首个 `fuse()` ≈ 300ms 预热 + ~135ms 取帧 ≈ 435ms 才产出首个结果，距闸门 0.5s 首消息超时仅 ~65ms 裕量，调度抖动即触发假性零速 —— 得不偿失；
4. 日后若真机多次观察到"重启急停造成困扰"，正确路径是感知层 warm-up（等 bottom 线程首帧再开始走，而非屏蔽 STOP），届时再评估。

**验收**：按设计保留，仅在文档注明。✅ 本文档即验收记录；不改代码、不动 fail-closed 语义。

---

## R5 跨仓库一致性：ROS 胶水包需核对新 fail-closed 语义 【🟠 跨仓库一致性】

**位置**：`mechdog_navigation_ros`（本地 `C:\Users\老w\Documents\dsh\mechdog_navigation_ros` 落后远端 4 提交，停在第 4 轮 `3b2b7a0`）

**问题**：算法库 v2.3 改动了多个**语义级**接口与决策：
1. `is_fall_risk()` 新增 fail-closed（任何无数据/无效 → STOP）；
2. `all_sensors_invalid()` 改用融合层口径（区域 `valid_pixel_ratio>0` 才算 Astra 有效）；
3. `determine_action()` 新增 `front_valid` 参数，前向全盲时 `SLOW_FORWARD`（不再 fail-open `FORWARD`）；
4. `FusionResult.min_forward_distance_m` 在全失效时返 8.0 兜底、且 `sensors_valid=false` 由调用方判 STOP。

ROS 侧 `safety_node` 若仍按旧逻辑消费 `FusionResult`（例如把 `min_forward_distance_m==8.0` 当作"前方开阔"直接 `FORWARD`，或忽略 `sensors_valid`/`cliff_detected`），会在新语义下重现 **fail-open**——而算法库侧的 fail-closed 努力会被跨层消费逻辑抵消。

**修复**：
- 单独审查 `mechdog_navigation_ros`，确认 `safety_node` 严格按 `recommended_action` 执行（不自行 reinterpret `min_forward_distance_m`）；
- 确认 `cliff_detected` / `sensors_valid` 在 ROS 层有对应急停闸门；
- 将 ROS 包 pull 到 v2.3 对应 HEAD 后，补一项集成级测试（注入"全传感器无效"帧，断言 ROS 输出为 STOP）。

**落实**：✅ 已核对 + 集成测试（b003845）——
1. **核对结论（safety_node 全量源码）**：`on_timer()` 消费路径 = 新鲜度看门狗（800ms，过期→本地零速）→ `planner_->plan(result)` → 发布；`min_forward_distance_m` 仅进 JSON/日志（可视化），**不参与**任何 cmd 判定；`cliff_detected`/`sensors_valid` 均在算法库 `determine_action()` 内消费（M1 全失效→STOP、R3 前向全盲→SLOW_FORWARD、悬崖优先 STOP），ROS 层不经手。→ **无 fail-open 重现路径**。
2. **前提确认**：ROS 包已与远端同步（含方案A: 订阅 `/ultrasonic` 注入 + `mechdog_ultrasonic` 依赖），算法库以 OBJECT 库 `mechdog_algo` 直引同版本源码（`MECHDOG_ALGO_DIR`，默认 `../mechdog_navigation`），无版本错配。
3. **新增集成测试** `test/test_safety_fail_closed.cpp`（ament_add_gtest）：
   - `AllInvalidInjected_stopsOnFullChain`：方案A 注入全无效超声 + 相机无有效帧 → `fuse()` 必 `STOP`（`sensors_valid=false`、`min_fwd=8.0` 兜底）→ `planner_->plan()` 输出 `(0.0, 0.0)`；
   - `FallbackEightMetersNeverBecomesForward`：`min_fwd=8.0 + STOP` 语义下 `plan()` 绝不产生非零速度（防"8.0=开阔"回归）。
   - WSL colcon 测试：✅ 全绿。

**验收**：ROS 包与算法库 v2.3 同步、且 `recommended_action=STOP` 时底盘确实停。✅ 已满足（测试 + 代码路径核对）。

---

## 三、可忽略项（记录备查，不处理）

- 权重 `ultra_w = 1.0 - astra_w`：实测 full-quality 下与基础权重（indoor 0.8/0.2、semi 0.5/0.5、outdoor 0.1/0.9）互补一致；quality<1 时 Astra 降权、超声补权，是"Astra 不可靠就信超声"的预期行为，**非 bug**。无需处理。
- `kUltrasonicInvalidM=4.5` 与 `max_distance_cm=4.0`：二者语义不同（4.5=失效哨兵、4.0=量程上限），当前分层逻辑只用 4.5，一致。若日后有人把超量程判据误改成 4.0 会漏判——已在 `config.h:52` ALG-5 注释点明，足够，无需改代码。

---

## 四、修复优先级建议

| 顺序 | 条目 | 理由 | 状态 |
|------|------|------|------|
| 1 | R1 / R2 / R3 | 纯注释修正，零风险，顺手合入 | ✅ 已合入 c2617b6 |
| 2 | R5 | 跨仓库一致性是**唯一可能让 v2.3 安全修复失效**的路径，优先级最高需排查 | ✅ 已排查 + 集成测试 b003845 |
| 3 | R4 | 设计知会，按安全优先可"按设计保留"，仅在 ROS 集成时处理 | ✅ 按设计保留（文档注明） |

---

## 审查结论（一句话）

**v2.3 的 10 项 ALG 修复真实落地、经得起实测（677 绿 + ASan/UBSan 零报告），提交信息可信；剩余 R1~R3 为文档/措辞级改进，R4 为设计知会，R5 为跨仓库一致性排查（唯一需警惕 fail-open 重现的路径）。无安全或正确性硬伤，可放心推进。**

---

## 本轮跟进结论（2026-08, R1~R5 全部闭环）

R1/R2/R3 纯注释修正零风险合入；R4 按设计保留（首帧急停=启动即验证 fail-closed，且"预热屏蔽/延迟首帧"均会引入上游 0.5s 超时风险）；R5 排查确认 safety_node 消费路径无 fail-open 重现，并以跨层集成测试固化（全失效注入 → STOP → 零速 + 8.0 兜底永不 FORWARD 回归护栏）。**v2.3 安全语义自算法库至 ROS 输出全链路闭合，可放心推进硬件阶段。**
