# mechdog_navigation 真机深度建图 — 操作说明

- 适用平台：**Windows**（本项目算法库在 Windows/MSVC 下编译运行，无 ROS2 依赖）
- 目标：用 **Astra Pro 深度相机**采集真实深度数据，跑通「深度 → 反投影 → 建图 → 出图」，在可视化窗口查看「点云 + 占据图 + 轨迹」。
- 本文档基于一次**完整跑通的实战排障**整理，包含所有已知坑的修复方法，可照此复现。

---

## 0. 前置条件

| 项 | 要求 |
|---|---|
| 深度相机 | Orbbec Astra Pro（含深度传感器 + 彩色相机），USB 连接电脑 |
| 相机驱动 | Orbbec `obdrv4.sys`（`C:\Program Files\Orbbec\ASTRA`），**已安装** |
| Astra SDK | `D:\orbbec ceram\AstraSDK-v2.1.3-94bca0f52e-20210608T034051Z-vs2015-win64` |
| 编译环境 | CMake + MSVC (VS 2022 BuildTools 18.x) |
| 算法库 | `mechdog_navigation`（含 mapping/point_cloud/sensor_astra） |

> ⚠️ 相机机身通常有指示灯。**如果插上不亮/不确定，先确认 USB 供电和插入的接口**（优先 USB 3.0 蓝色口、直插主板后面）。深度相机量程约 **0.6~8m**，太近或太远都会无有效深度。

---

## 1. 一次性配置：OpenNI2 驱动目录（关键！）

**这是本次排障的核心**。Astra 的 OpenNI2 底层需要**导出 `oniDriverCreate` 函数的驱动 dll** 才能采集。SDK 里真正的这个驱动是：

```
D:\orbbec ceram\AstraSDK-v2.1.3-...\bin\orbbec.dll
```

而 SDK 的 `bin\Plugins` 下的 `openni_sensor.dll`、`orbbec_xs.dll` **不含** `oniDriverCreate`，不是 OpenNI2 设备驱动。

### 修复步骤

1. 确保存在目录：`bin\OpenNI2\Drivers\`
2. 把 `bin\orbbec.dll` **复制**到 `bin\OpenNI2\Drivers\orbbec.dll`
3. **移除**（若之前误放）`bin\OpenNI2\Drivers` 下的 `openni_sensor.dll`、`orbbec_xs.dll`（它们会让 OpenNI2 加载失败）

最终 `bin\OpenNI2\Drivers` 里应只有：

```
bin\OpenNI2\Drivers\orbbec.dll
```

> 验证方法：`bin\OpenNI.ini` 里临时打开日志（`Verbosity=0`、`LogToFile=1`），跑一次 SDK 示例，看 `bin\Log\*.log` 是否出现 `[FPS] Depth: 30.xx`。正常排查完可以把日志关回注释。

---

## 2. 编译真机版

在 `mechdog_navigation` 目录：

```powershell
cd C:\Users\老w\Documents\dsh\mechdog_navigation
$sdk = "D:\orbbec ceram\AstraSDK-v2.1.3-94bca0f52e-20210608T034051Z-vs2015-win64"
cmake -B build_real_map -DUSE_ASTRA_SDK=ON "-DASTRA_SDK_ROOT=$sdk"
cmake --build build_real_map --config Release --target mechdog_navigation
```

产物：`build_real_map\Release\mechdog_navigation.exe`

> 若报 `USE_ASTRA_SDK` 相关错误，确认 `ASTRA_SDK_ROOT` 指向含 `include/astra/astra.hpp` 的 SDK 根目录。

---

## 3. 运行真机建图

**关键**：运行时要把 SDK 的 `bin` 目录加进 `PATH`，否则程序找不到 `astra_core.dll` / `orbbec.dll` 等，会在启动时静默退出。

```powershell
cd C:\Users\老w\Documents\dsh\mechdog_navigation
$sdk = "D:\orbbec ceram\AstraSDK-v2.1.3-94bca0f52e-20210608T034051Z-vs2015-win64"
$env:PATH = "$sdk\bin;" + $env:PATH

# 真机 + 建图可视化 + 静止位姿, 采 25 帧后自动保存 PGM
.\build_real_map\Release\mechdog_navigation.exe --real --map --frame-n 25 --cloud
```

### 命令行参数（本次新增的建图可视化）

| 参数 | 含义 | 默认 |
|---|---|---|
| `--real` / `-r` | 真机模式（Astra SDK） | 模拟模式 |
| `--map` | 启用建图可视化（占据图+轨迹） | 关 |
| `--cloud` | 启用点云可视化（建图会隐式开启） | 关 |
| `--frame-n <N>` | 采 N 帧后冻结地图并保存 PGM（0=无限） | 无限 |
| `--sweep <deg>` | 原地旋转扫描（位姿按帧序匀速推进） | 0（静止） |
| `--static` | 显式静止位姿（取消 sweep） | 开 |
| `--ground <h>` | 手持模式：外参归零+地面先验=-h | 0.8 |

### 示例：旋转扫描一圈

```powershell
.\build_real_map\Release\mechdog_navigation.exe --real --map --sweep 360 --frame-n 60 --cloud
```

> ⚠️ `--sweep` 的位姿是**匀速假设**（按帧序线性推进航向），不是真实测量。转快了/慢了/没转满会错位，只能用于先看全貌，不能当精确建图。

---

## 4. 可视化窗口里看什么

运行后弹出窗口（**三栏布局**）：

```
┌────────────┬────────────┬────────────┐
│ 彩色/深度帧 │  点云俯视图  │  占据图+轨迹 │
└────────────┴────────────┴────────────┘
   状态栏: env / min_fwd / action / map: occ=.. free=.. unknown=..
```

- **占据图**：橙色=占据，深灰=空闲（观察到的可行走区），背景=未知
- **轨迹**：青色折线 = 机器人一路位姿，黄点+箭头 = 当前机头
- **底部**：`map: occ=... free=... unknown=...` 统计

控制台还会打印 `cloud_pts=... state=... vpx=...`：
- `state=0` = 取帧失败（相机没出数据）
- `state=1` = 有帧但无有效像素（太近/太远/高反光）
- `state=2` = 正常，`vpx` 是有效像素数

---

## 5. 成功标志

控制台出现类似输出即成功：

```
[P4] 已采集 25 帧, 地图冻结. PGM 已保存: mechdog_map_live.pgm
[P4] map 200x200 res=0.05m  unknown=... free=... occ=...
```

生成 `mechdog_map_live.pgm`（200x200，P2 格式，ROS map_server 惯例 0=占/254=空/205=未知）。

---

## 6. 常见问题（FAQ）

### Q1：运行即退出，无任何输出
**原因**：SDK 的 dll 不在 PATH。
**解决**：运行前 `$env:PATH = "$sdk\bin;" + $env:PATH`（见第 3 节）。`astra_core.dll`、`orbbec.dll`、`OpenNI2.dll` 都在 `bin`。

### Q2：报 `0x80070005 拒绝访问`（Unable to activate device）
**原因**：OpenNI2 找不到有效驱动（`Found no valid drivers`）。
**解决**：执行第 1 节——确保 `bin\OpenNI2\Drivers\orbbec.dll` 存在，且里面**没有** `openni_sensor.dll`/`orbbec_xs.dll`。

### Q3：`cloud_pts=0 state=0`
相机没取到帧。检查：
- 相机是否插好、指示灯是否亮
- 是否有其他程序占用相机（微信视频、奥比中光查看器等）→ 关掉重跑
- 若刚插拔过，重插一次

### Q4：`cloud_pts=0 state=1`
有帧但深度全无效。相机前**没有** 0.6~8m 内的物体，或距离太远/太近。让相机对着有东西的方向（墙、人、物体）。

### Q5：地图是"一个锥形"、不是完整房间
**正常**。相机静止时只看得到前方一个扇形。要更完整：
- 用 `--sweep 360` 转一圈
- 或接入真实位姿（轮速 odom / IMU，见技术方案 M3）

### Q6：底部一排橙色假点（假墙）
**正常现象**。Astra 前倾 15° 会把地面投成占据格。当前建图**还没接地面滤波**。可先用 `--ground <h>` 手持模式，或后续把 `ground_segmentation` 接进 `insert_cloud` 前（见技术方案风险表）。

### Q7：`get_latest_frame` 报 `astra::Frame` 无默认构造
**如果你改代码**：`astra::Frame` 没有默认构造，要 `std::make_unique<astra::Frame>(reader.get_latest_frame(0))` 或用原始指针。`get_latest_frame` 默认 `autoCloseFrame=true`，析构时自动 `close_frame`。

---

## 7. 排障用日志（可选）

若相机仍不出数据，打开 OpenNI2 详细日志定位：

1. 编辑 `bin\OpenNI.ini`：
   ```
   [Log]
   Verbosity=0
   LogToConsole=1
   LogToFile=1
   ```
2. 重跑，看 `bin\Log\*.log`：
   - `Found no valid drivers` → OpenNI2 没加载到正确驱动（回第 1 节）
   - `Couldn't find function oniDriverCreate` → 驱动目录里放错 dll
   - `[FPS] Depth: 30.xx` → 相机正常出数据

3. 排查完把 `Verbosity/LogToConsole/LogToFile` 改回注释。

---

## 8. 关键文件索引

| 目标 | 路径 |
|---|---|
| 建图实现 | `mechdog_navigation\mapping.cpp/.h`（insert_cloud/occ_state） |
| 点云管线 | `mechdog_navigation\point_cloud.cpp/.h` |
| Astra 驱动 | `mechdog_navigation\sensor_astra.cpp/.h`（capture_real/get_latest_frame） |
| 主程序+可视化 | `mechdog_navigation\main.cpp`（draw_map_view/draw_trace/draw_scene_impl） |
| SDK 路径 | `D:\orbbec ceram\AstraSDK-v2.1.3-...` |
| SDK 取帧示例 | `sdk\samples\cpp-api\DepthReaderEventCPP\main.cpp`（回调模式） |
| OpenNI2 驱动 | `sdk\bin\OpenNI2\Drivers\orbbec.dll`（**已修复，关键**） |
| 技术方案 | `docs\TECH_PLAN_LOCAL_MAPPING_TEST.md` |

---

## 9. 当前已知边界（如实说明）

- **建图是「地图累积器」，不是 SLAM**：`mapping.cpp` 只做占据栅格累积，不做定位/回环。位姿需外部喂入。静止/匀速假设下只能验证管线，不是精确建图。
- **真实位姿未接**：要让地图不漂、能拼完整房间，得接轮速 odom / IMU（技术方案 M3 阶段）。
- **地面点未过滤**：Astra 前倾会把地面当障碍（底部假墙）。需接 `ground_segmentation`（M1/M2 风险项）。
- **`--sweep` 匀速假设**：转不匀会错位，仅作全貌查看。
