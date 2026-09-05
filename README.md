# mechdog_navigation

机械狗导航系统 —— 基于 Astra Pro 深度相机 + HC-SR04 超声波传感器阵列的多传感器融合寻路方案。

> **环境光强判定状态（2026-08）**：TSL2591 已取消购买（见 `docs/FIX_PLAN.md` F3 决策），环境判定默认走 `estimate_ambient_light()` 深度图代理；`sensor_ir` 驱动保留为可选增强（模拟模式可用，真机 I2C 分支待硬件）。

## 项目简介

本项目为四足机械狗提供实时障碍物检测与自主导航能力。系统融合**奥比中光 Astra Pro 深度相机**（单目结构光）与 **4 颗 HC-SR04 超声波传感器**，实现多距离层次、多环境自适应的传感器融合策略。

## 传感器布局

```
         Astra Pro 深度相机 (58.4 deg FOV)
         ├─ 0.6m ~ 8m 主力探测
         └─ 盲区: 0 ~ 0.6m

              ▲ HC-SR04 正前 (补盲区)
             / \
            /   \
   左前 -30 deg /     \ 右前 +30 deg
  HC-SR04      ◇     ◇    HC-SR04
  (扩展侧向覆盖)      (扩展侧向覆盖)

         ● HC-SR04 底部朝下 (防跌落, -90 deg)
```

| 传感器 | 位置 | 偏航角 | 俯仰角 | 功能 |
|--------|------|--------|--------|------|
| front_left | 左前 | -30 deg | 0 deg | 覆盖左前方盲区 |
| front_center | 正前 | 0 deg | 0 deg | 检测正前方障碍物，填补 Astra 盲区 |
| front_right | 右前 | +30 deg | 0 deg | 覆盖右前方盲区 |
| bottom | 底部朝下 | 0 deg | -90 deg | 检测地面悬崖/台阶（防跌落） |

## 系统架构

| 模块 | 文件 | 功能 |
|------|------|------|
| 全局配置 | `config.h` | 传感器参数、融合分层策略、地图参数、规划参数、紧急避障阈值 |
| 超声波驱动 | `sensor_ultrasonic.h/.cpp` | HC-SR04 驱动，4 颗分时轮询防串扰（含模拟模式） |
| 深度相机驱动 | `sensor_astra.h/.cpp` | Astra Pro 驱动，通过 Orbbec Astra SDK 获取深度图（含模拟模式；真机经 `USE_ASTRA_SDK` 编译） |
| 红外强度驱动 | `sensor_ir.h/.cpp` | TSL2591 环境红外检测，用于环境自适应权重（含模拟模式） |
| 传感器融合 | `sensor_fusion.h/.cpp` | 分层加权融合、环境自适应、障碍物分类、导航决策 |
| 点云模块 | `point_cloud.h/.cpp` | 深度图→3D 点云反投影、光学系→link 系坐标变换（P0，供规划/建图预留，独立于融合层） |
| 地面分割 | `ground_segmentation.h/.cpp` | 受约束 RANSAC 地面平面 + 2.5D 栅格**负障碍检测**（坑/下行台阶，P1；门口试金石单测锁定） |
| 建图模块 | `mapping.h/.cpp` | 位姿驱动点云累积 → log-odds 占据栅格 + 光线空闲 + 膨胀 + PGM 导出（P4） |
| 2.5D 近场地形 | `heightmap_2d5.h/.cpp` | 单平面 2.5D 高程/可通行地形 → 越障判断（能走/凸起/沟坑/太陡，P1.5；复用 P1 地面平面） |
| 日志系统 | `logger.h` | 分级日志 DEBUG/INFO/WARN/ERROR + 时间戳 + 可选写文件（header-only，零依赖） |
| 路径规划 | `path_planner.h/.cpp` | 导航动作 -> 速度指令映射（DWA 待实现） |
| 单元测试 | `tests/test_fusion.cpp` | F4/F5 回归 + 融合逻辑验证 (`ctest`) |
| 点云单测 | `tests/test_point_cloud.cpp` | 反投影/坐标变换/失效哨兵验证 (`ctest`，P0) |
| 地面分割单测 | `tests/test_ground_segmentation.cpp` | 平地/坑/台阶/门口试金石/倾斜/退化输入 (`ctest`，P1) |
| 建图单测 | `tests/test_mapping.cpp` | 坐标换算/占空判定/位姿变换/多帧累积/动态清障/膨胀/PGM (`ctest`，P4，40 断言) |
| 2.5D 单测 | `tests/test_heightmap_2d5.cpp` | 平地/凸起台阶/沟坑/fail-closed (`ctest`，P1.5) |
| 真机建图工具 | `tools/mapping_real_test.cpp` | Windows 真机验证工具：Astra 真深度→全管线→PGM（静止校验 + 旋转扫描模式） |

## 点云模块（P0）

为未来「路径规划 + 建图（SLAM）」预埋的**几何感知中间层**，详见 `docs/POINT_CLOUD_DESIGN.md`。当前阶段（v2.3）融合层只用 3 个标量区域做反应式避障，深度图里约 30 万像素的 3D 几何信息被压缩后丢弃；点云模块把深度图还原为 3D 点云，向上支撑精细避障、建图与路径规划。

### 坐标约定（与 ROS REP-103 一致）

| 坐标系 | 轴约定 | 说明 |
|--------|--------|------|
| `camera_optical` | X 右, Y 下, Z 前 | 标准针孔光学系（Astra SDK 出帧系） |
| `camera_link` | X 前, Y 左, Z 上 | 与 optical 差固定旋转 `Rz(-90°)·Rx(-90°)` |
| `base_link` | X 前, Y 左, Z 上 | 底盘几何中心 |

完整链路：`depth_to_cloud`（光学系反投影）→ `transform_optical_to_link`（固定旋转）→ `transform_to_base`（叠加 `CameraExtrinsics`）。P0 仅实现 `camera_optical → camera_link`，`base→odom→map` 留待 ROS `tf2` 集成。

### 核心接口

```cpp
// 深度图(mm) → 点云(camera_optical 系, 米); 无效像素与 analyze_region 同口径丢弃
void depth_to_cloud(const uint16_t* depth, int w, int h,
                    const CameraIntrinsics& K, PointCloud& out);

// camera_optical → camera_link 固定旋转 (§6.1 矩阵 [[0,0,1],[-1,0,0],[0,-1,0]])
void transform_optical_to_link(const PointCloud& in, PointCloud& out);

// camera_optical → base_link 全链路 (先 optical→link, 再叠加 ZYX 欧拉 + 平移外参)
void transform_to_base(const PointCloud& in, const CameraExtrinsics& E,
                       PointCloud& out);
```

### 实现要点

- **零依赖**：仅标准库（`<vector>`/`<cmath>`），不引 PCL，可模拟单测
- **内参初值**：由 FOV 反推（`fx≈572.7, fy≈572.3`），真机可后用 Astra SDK `intrinsics()` 直读
- **轴约定**：`camera_optical` 与 `base_link` 的 {X右,Y下,Z前} vs {X前,Y左,Z上} 符号相反，是最大坑；已固化为 §6.1 矩阵并在单测中断言
- **与融合层解耦**：点云模块独立于 `SensorFusion`，只"增强"不"否决"，不破坏现有三区域 fail-closed 安全语义

### 真机点云可视化

```bash
# Astra Pro 真机 + 点云可视化 (分屏: 左半彩色帧 + 右半点云)
./mechdog_navigation --real --cloud
```

窗口内按 `S` 键在**俯视图**（前=上、左=右）与**侧视图**（前距=右、高度=上）之间切换，用于验证反投影与轴约定。`--cloud` 需与 `--real` 配合（真机深度图）；模拟模式也可运行（`--cloud` 不带 `--real`），但模拟帧只有"正前方起伏墙"，主要验证链路。

## 建图模块（P4）

位姿驱动的点云累积建图，把 P0 的 base 系点云升级为**世界系占据栅格地图**（`docs/POINT_CLOUD_DESIGN.md` P4 设计）。核心回答一个问题：*"机器人在哪、看到了什么 → 这间屋子长什么样"*。

### 建图链路

```
深度图 (Astra 真机/合成)
  → depth_to_cloud        (P0 反投影, camera_optical 系)
  → transform_to_base     (P0 外参变换, base_link 系)
  → insert_cloud(云, 位姿) (P4: base→odom 旋转平移 + 光线空闲 + log-odds 占据)
  → OccupancyGrid 200x200 @ 5cm (10x10m, 复用 MapConfig)
  → save_pgm / nav_msgs/OccupancyGrid
```

### 核心机制

| 机制 | 实现 | 说明 |
|------|------|------|
| 占据更新 | log-odds（±6，钳位 ±20，阈值 ±4） | 多帧投票，单帧噪声不会翻转状态 |
| 空闲标记 | 相机原点→命中点光线步进（步长 2.5cm，限 5m） | 走廊打通 + **动态清障**（障碍移走后光线穿过即清除） |
| 障碍膨胀 | 圆形核（默认 0.15m，`MapConfig::inflation_radius_m`） | 供规划器消费的 costmap 语义 |
| 地图导出 | PGM (P2, ROS map_server 惯例: 0=占据/254=空闲/205=未知) | `nav2 map_server` 可直接加载 |

### 核心接口

```cpp
mechdog::OccupancyGridMap map(/*robot_radius=*/0.25);
map.insert_cloud(cloud_base, /*Pose2D*/ {x, y, theta});  // 位姿来自 odom/SLAM
map.inflate();          // 可选: 规划前膨胀
map.save_pgm("map.pgm");
```

`Pose2D` 由 ROS 层从 `nav_msgs/Odometry` 提取——算法库零 ROS 依赖，单测全部离线可跑（40 断言：坐标换算/空帧安全/占空判定/位姿变换/多帧累积/动态清障/膨胀/PGM）。

### 真机建图验证（Windows 工具）

```bash
# MSVC 真机构建 (USE_ASTRA_SDK=ON) 后:
tools/build_real.bat

# 静止模式: 100 帧室内扫描, 占据点距离分布自动校验 (0.55~5.5m > 95%)
build_real/Release/mapping_real_test.exe 100 map_real.pgm

# 旋转扫描模式: 5 秒倒计时后匀速向左转 360° (~17s)
#   航向为匀速假设 (无 IMU/odom), demo 精度非测量
build_real/Release/mapping_real_test.exe 150 map_sweep.pgm 360 5
```

真机实测（2026-09）：静止 100 帧 100% 有效 + 距离校验 PASS；手持 360° 扫描成图。对照实验（静止输入误标旋转 → 90° 周期鬼影环；真旋转 → 非均匀真实墙环）验证了**管线对位姿输入的忠实性**——位姿错，地图立刻伪。

### 已知边界（P4 范围）

- 单次累积无回环校正：cmd_vel 开环积分 odom 长距离会漂移（10m 场地内可接受）
- 地面点未过滤：P1 地面分割结果尚未接入建图入口（接入后可消除地面假占据）
- 10×10m 固定窗口（`MapConfig`），大场地需参数化

## 2.5D 近场地形（P1.5）

单平面 2.5D 高程/可通行地形，基于 P1 地面分割（复用其 `GroundSegResult.plane`），把 base 系点云组织成「每格一个高度 + 可通行标签」的栅格，用于**近场越障判断**（台阶/沟/坡能不能走）。与 P1 的分工：P1 只判"有没有坑"（负障碍），本模块补全**正障碍（凸起）+ 每格高度 + 完整可通行标签**。

> 定位：**2D 雷达扫不到地面高度变化**（只扫一个水平面），深度相机天然看 3D 体积 + 地表，是越障（台阶/沟/坡）的**核心传感器**。这也是工业巡检越障场景下 2.5D 的意义所在——2D 雷达只负责全局导航，深度相机负责近场越障地形。

### 输出语义（每格）

| 标签 | 含义 | 触发条件 |
|------|------|---------|
| `Traversable` | 能走 | 与参考地面高度差在容忍内 |
| `ObstacleUp` | 凸起障碍 | 高于地面 > `step_up_max`（默认 10cm） |
| `CliffDown` | 沟/下行台阶 | 低于地面 > `drop_down_max`（默认 15cm） |
| `TooSteep` | 坡过陡 | 平面坡度 > `slope_max`（默认 20°） |
| `Unknown` | 未扫到 | 该格无样本 |

阈值默认取保守值（`HeightMap25Config`），可按机械狗越障能力实测调。v1 边界：**单平面假设**（与 P1 一致），不处理多层平台（将来升级 `elevation_mapping`）。

### 核心接口

```cpp
mechdog::HeightMap25Result hm;
mechdog::HeightMap25Config cfg;   // step_up_max / drop_down_max / slope_max 等
build_heightmap_25(cloud_base, seg, cfg, hm);   // seg 为 P1 segment_ground 结果
// hm.flag[cell] : CellFlag 枚举; hm.height[cell] : 该格参考地面高度
```

### 真机建图/2.5D 可视化（Windows）

```bash
# 真机构建 (USE_ASTRA_SDK=ON) 后, 先把 SDK bin 加进 PATH:
#   $env:PATH = "<SDK根目录>\bin;" + $env:PATH

# 真机 + 建图可视化 (占据图+轨迹), 静止位姿, 采 25 帧后自动存 PGM
./mechdog_navigation.exe --real --map --frame-n 25 --cloud

# 真机 + 旋转扫描 360° (匀速假设, 非精确)
./mechdog_navigation.exe --real --map --sweep 360 --frame-n 80 --cloud

# 真机 + 2.5D 近场地形 (需要相机外参标定)
./mechdog_navigation.exe --real --hm25 --pitch <俯角> --height <离地高> --cloud
```

命令行参数（可视化）：

| 参数 | 含义 | 默认 |
|------|------|------|
| `--real` | 真机模式（Astra SDK） | 模拟 |
| `--map` | 建图可视化（占据图+轨迹） | 关 |
| `--hm25` | 2.5D 近场地形可视化 | 关 |
| `--cloud` | 点云可视化（`--map`/`--hm25` 会隐式开启） | 关 |
| `--frame-n <N>` | 采 N 帧后冻结地图并存 PGM（0=无限） | 无限 |
| `--sweep <deg>` | 原地旋转扫描（位姿按帧序匀速推进） | 0（静止） |
| `--static` | 显式静止位姿 | 开 |
| `--pitch <deg>` | 相机前俯角（2.5D 标定用） | 15 |
| `--height <m>` | 相机离地高（2.5D 标定用） | 0.18 |

### 可视化窗口（三栏：彩色帧 | 点云 | 右栏）

- **`--map`**：右栏 = 2.5D 时显示地形，否则显示 P4 占据图 + 轨迹（青=轨迹线，黄=机头）
- **`--hm25`**：右栏 = 2.5D 地形（**绿=能走 / 橙=凸起 / 红=沟坑 / 蓝=太陡 / 灰=未知**）

### 真机实测（2026-09）

Astra 相机深度流正常（30fps），真机深度 → 反投影 → 建图（P4）→ PGM 全链路跑通；`--sweep 360` 扫出真实环境 360° 环形占据图（occ=5860 / free=26786）。**2.5D 当前依赖相机外参标定** —— 相机尚未装到机械狗、外参用占位值（`z=0.18, pitch=+15°`）时点云坐标系偏移，2.5D 待相机装机后实测 `--pitch/--height` 标定验证。详见 `docs/REAL_MAPPING_GUIDE.md`。

## 传感器融合策略

### 距离分层

| 层级 | 距离范围 | 策略 | 说明 |
|------|---------|------|------|
| L0 | 0 - 0.6m | 仅超声波 | Astra Pro 盲区补偿 |
| L1 | 0.6 - 3m | 融合（保守） | 取两者中更近的值 |
| L2 | 3 - 8m | 融合（Astra 为主） | 自适应权重: 室内 0.8 / 半室内 0.5 / 室外 0.1 (随环境与深度质量调整); 超声更近时保守取近 |
| L3 | > 8m | 仅 Astra | 超出超声波量程 |

### 环境自适应权重

| 环境 | Astra 权重 | 超声波权重 | 适用场景 |
|------|-----------|-----------|---------|
| 室内 (indoor) | 0.8 | 0.2 | 无阳光干扰，结构光最佳 |
| 半室内 (semi_indoor) | 0.5 | 0.5 | 走廊/棚下/窗边，需超声波补充 |
| 室外 (outdoor) | 0.1 | 0.9 | 阳光干扰严重，超声波主导 |

环境判定默认走 **深度图无效像素比例代理**（`estimate_ambient_light()`，TSL2591 已取消购买）；真机若接入 TSL2591 则作为可选增强（归一化 0~1）。阈值统一见 `config.h` 的 `EnvironmentThresholds`（深度代理与红外同源 0.3/0.7）。

### 超声波分时轮询

4 颗 HC-SR04 同频 40kHz，同时触发会产生串扰。采用**分时轮询**策略：

- 读取顺序: `bottom -> front_center -> front_left -> front_right`
- 每颗间隔: 30ms（> 最大回波时间 25ms + 衰减余量）
- 完整一轮: ~120ms，更新率约 8Hz
- 底部优先: 朝下发射，与前向传感器不在同一平面，天然无串扰

## 紧急避障阈值

| 距离 | 等级 | 动作 |
|------|------|------|
| ≤ 10cm | 临界 (CRITICAL) | 强制停止 |
| ≤ 25cm | 危险 (DANGER) | 后退 (BACKWARD) |
| ≤ 50cm | 警告 (WARNING) | 融合距离段转向/直行, 超声段后退 |
| > 50cm | 安全 (SAFE) | 正常行驶 |

> ⚠️ **上表未覆盖的兜底分支（真机部署前必读）**：前向三方向**全部失效**（Astra 三区域均无有效深度像素 + 三颗前向超声全无效）时，动作是 `SLOW_FORWARD` 降速盲行（≈0.1 m/s）**而非 STOP**，仅靠 bottom 悬崖检测兜底 —— 详见下方「已知限制」#7。

## 依赖项

- **CMake** >= 3.16
- **C++20** 编译器 (GCC 10+ / Clang 12+ / MSVC 2022+)
- **Orbbec Astra SDK**（可选，Astra Pro 真机模式，经 `-DUSE_ASTRA_SDK=ON -DASTRA_SDK_ROOT=<sdk根目录>` 启用）
- **WiringPi**（可选，树莓派 GPIO 模式；注意原版已停止维护，建议使用社区 fork `github.com/WiringPi/WiringPi`）
- **Linux i2c-dev**（可选，TSL2591 真机模式）

## 单元测试

```bash
# 构建并运行测试（F4 悬崖安全 / F5 读数过滤 / 融合逻辑）
cmake --build . --target test_fusion test_point_cloud
ctest --output-on-failure
```

- `test_fusion` — 融合层回归（677 断言）
- `test_point_cloud` — 点云模块 P0（反投影几何/内参/坐标变换/失效哨兵/轴约定，12 测试函数，30 万+ 断言）

## 构建

```bash
git clone https://github.com/justdoti666/mechdog_navigation.git
cd mechdog_navigation
mkdir build && cd build

# PC 模拟模式（无需硬件）
cmake ..

# 树莓派 WiringPi 模式
cmake .. -DUSE_WIRINGPI=ON

# Astra Pro 真机模式 (Orbbec Astra SDK)
cmake .. -DUSE_ASTRA_SDK=ON -DASTRA_SDK_ROOT=<sdk根目录>

# 完整硬件模式
cmake .. -DUSE_WIRINGPI=ON -DUSE_ASTRA_SDK=ON -DASTRA_SDK_ROOT=<sdk根目录>

cmake --build .
```

## 模拟模式

当 `USE_WIRINGPI` 和 `USE_ASTRA_SDK` 均未启用时，系统进入模拟模式：

- 超声波传感器返回随机模拟数据 (85% 空旷 / 10% 中距 / 5% 近距，底部 5% 悬崖)
- Astra Pro 返回模拟深度帧
- 适合在 PC 上进行算法验证和开发调试

## 硬件要求

| 组件 | 型号 | 数量 |
|------|------|------|
| 深度相机 | 奥比中光 Astra Pro | 1 |
| 超声波传感器 | HC-SR04 | 4 |
| 环境光强 | TSL2591 模块 (I2C 0x29) — **已取消购买，默认深度图代理**（可选增强） | 0–1 |
| 主控 | Raspberry Pi 5B| 1 |

## 已知限制

1. **Astra SDK 真机深度已实现** — 经 `USE_ASTRA_SDK` 编译后 `capture_frame()` 走 FrameListener 真机分支（深度+彩色双流）；未编译时回退模拟模式
2. **环境红外阈值待标定** — `IrConfig` 为软件预设默认值，需硬件组按 `docs/IR_CALIBRATION.md` 实测后更新
3. **wiringPi 真机 GPIO 未验证** — `measure_distance()` 的轮询实现受 Linux 调度抖动影响（1ms ≈ 17cm），后续可改边沿中断+时间戳
4. **点云外参未标定（2.5D 依赖）** — P0 用 FOV 反推内参 + 外参占位初值（`x=0.12, y=0, z=0.18, pitch=+15°`）；狗未装机时按 §18.4 用零外参临时摆放联调，装机后需按 `docs/POINT_CLOUD_DESIGN.md` §12 量测 + 标定；**P1.5 的 2.5D 地形对外参尤其敏感**，外参不准时点云坐标系偏移（实测 y 偏到 -7.8m），2.5D 全 unknown。已提供 `--pitch/--height` 命令行实机调参，相机装机后需实测标定。
5. **2.5D 待相机标定验证** — `heightmap_2d5` 已实现（单平面）且单测全绿，但**真机 2.5D 依赖相机外参标定**；相机未装到机械狗前点云坐标偏移，2.5D 无法正确分类。相机装机实测 `--pitch/--height` 后即可验证。
6. **Astra Pro 水平 FOV 仅 58.4°** — 点云俯视图因此呈中心扇形，属硬件物理限制，非代码问题
7. **前向全盲时降速盲行而非停车（接真机前必须确认）** — Astra 三区域均无有效深度像素**且**三颗前向超声全部无效时，`determine_action`（`sensor_fusion.cpp`）返回 `SLOW_FORWARD`（0.5×v_max ≈ 0.1 m/s 盲目前进），仅靠 bottom 悬崖检测兜底；该行为有 `test_front_blind_with_bottom_valid_is_conservative` 测试锁定。**设计前提是全局层（师兄安全闸门 / Nav2）对该场景另有兜底** —— 接机械狗实机前务必与师兄确认闸门已覆盖；若本层是最后防线，应把该分支改为 `STOP`，避免镜头被挡 + 前向超声全坏时低速撞上静止障碍物。

## 注意事项

1. **仅限室内/半室外** — Astra Pro 结构光怕阳光直射，室外需切换为超声波主导模式
2. **超声波串扰** — 已通过分时轮询解决，若更换传感器布局需重新评估轮询间隔
3. **底部安装高度** — 底部传感器平坦地面读数约 12-28cm，安装高度决定悬崖检测阈值
4. **算力需求** — 推荐树莓派 4B 及以上，3B 在多线程传感器读取时可能吃力
5. **门/暗区在俯视图中显示为"缺口"（勿误判为镜像）** — 门洞、阴影、黑色/强反光表面（电视屏、幕布）无深度回波，点云在该方位天然缺失。实测两次被误判为"左右镜像"（2026-08）：真机 D 键热力叠加验证，深度与 RGB 对齐无镜像。左右快速自检：不对称物体放 RGB 画面左侧，俯视图中应出现在左侧
6. **黑色/强反光表面深度失效** — 结构光物理特性（吸光/镜面反射），距离再近也无回波；此场景由底部 HC-SR04（声学）兜底，互不重叠

## 可视化窗口快捷键

| 键 | 功能 |
|---|---|
| `S` | 点云俯视图 ↔ 侧视图切换 |
| `M` | 深度图水平镜像开关（诊断用；默认关 = 正确方向，开启后热力/点云会偏离实物） |
| `D` | 深度热力图叠加 RGB（近=红 远=蓝；左右对齐裁决工具，红点应落在实际近处物体上） |
| `--ground <h>` | 手持实验模式：外参归零（base==link）、地面先验 = -h、窗口收紧 0.12；装机后勿用 |

### 可视化命令行参数（建图/2.5D）

| 参数 | 功能 |
|---|---|
| `--map` | 建图可视化（占据图 + 轨迹，`--sweep` 旋转扫描） |
| `--hm25` | 2.5D 近场地形可视化（绿=能走/橙=凸起/红=沟坑/蓝=陡/灰=未知） |
| `--cloud` | 点云可视化（`--map`/`--hm25` 会自动开启） |
| `--frame-n <N>` | 采 N 帧后冻结地图并存 PGM |
| `--sweep <deg>` | 原地旋转扫描（位姿匀速假设） |
| `--pitch <deg>` / `--height <m>` | 2.5D 相机外参实机标定 |
| `--log-level <D/I/W/E>` | 日志级别（默认 INFO） |
| `--log-file <path>` | 把日志写入文件（差错用） |

## 日志系统

程序内置分级日志（`logger.h`，header-only，零依赖），替代零散的 `std::cout` 打印，方便差错。每条日志自动带前缀：`[时间戳][级别][文件名:行号]`。

### 功能

- **分级**：`DEBUG` / `INFO` / `WARN` / `ERROR`
- **时间戳**：`年-月-日 时:分:秒`
- **定位**：自动附 `文件名:行号`，出错直接定位代码行
- **输出**：控制台 + 可选写文件

### 用法

```bash
# 默认 INFO 级别，控制台输出
./mechdog_navigation.exe --real --map

# 调 DEBUG 级别 + 写日志文件（差错调试用，推荐）
./mechdog_navigation.exe --real --map --log-level D --log-file run.log

# 只看警告/错误（减少输出）
./mechdog_navigation.exe --real --log-level W
```

### 日志示例（实测）

```
[2026-09-05 15:02:27][INFO][main.cpp:1064] [P4] 建图可视化: 位姿=静止原点, 采集 8 帧后停
[2026-09-05 15:02:30][INFO][main.cpp:1253] [P4] 已采集 8 帧, 地图冻结. PGM 已保存: mechdog_map_live.pgm
[2026-09-05 15:02:30][INFO][main.cpp:1254] [P4] map 200x200 res=0.05m  unknown=... free=... occ=...
```

建图（P4）、2.5D（hm25）的关键结果都走 `LOG_*` 记录，以后排查问题直接看日志文件即可。

> 注：Windows 控制台可能显示中文乱码（GBK/UTF-8 编码差异），日志文件内是正确 UTF-8。若需日志文件带中文说明，建议用 `--log-file` 并查看文件（非控制台）。

## License

MIT License

