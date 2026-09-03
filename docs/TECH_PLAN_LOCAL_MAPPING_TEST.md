# 机械狗建图程序「电脑端测试 + 可视化」技术方案

- 版本：v0.1（草案）
- 范围：`mechdog_navigation`（纯算法库，Windows/MSVC，无 ROS2 依赖）
- 目标：在不把程序装到机械狗、不依赖真机运动的前提下，**在 Windows 电脑上完整测试建图（P4 / OccupancyGridMap），并输出「轨迹 + 点云 + 占据栅格图」叠加的完整可视化**。
- 前置结论（来自上一轮诊断）：建图模块 `mapping.cpp` 是「已知位姿的地图累积器」，不是 SLAM；它本身不做定位、不做回环，位姿必须由外部喂入。因此本方案核心是「**把一套可靠的位姿源 + 深度数据 + 建图 + 可视化**」串成一条可在电脑上运行的闭环。

---

## 一、背景与目标

用户当前正在进行机械狗（轮腿复合机器人）的建图程序开发。硬件为 **Astra Pro 深度相机（近场）+ RPLIDAR 2D 雷达（远场）+ 轮速里程计 + BMI088 IMU**，软件栈为 **ROS2 Jazzy + Ubuntu 24.04**（Pi 5 / VM）。但**当前阶段不应把程序直接部署到机械狗**，而应先在电脑（Windows）上验证建图算法与链路。

一句话目标：

> **在电脑上，用 Astra 深度相机（或回放数据）实时驱动 `OccupancyGridMap` 建图，并在可视化窗口里同时看到：深度点云俯视图、机器人轨迹、以及建图得到的 2D 占据栅格图。**

### 1.1 为什么先在电脑测（不做真机部署）

| 原因 | 说明 |
|---|---|
| 算法尚在开发 | 建图模块（P4）刚写完，需先验证「深度→反投影→外参→栅格累积」链路是否正确 |
| 真机运动难复现 | 机械狗上 ROS 栈未稳定，位姿/odom 未通；电脑上可先用手持/匀速假设测管线 |
| 便于调参收敛 | log-odds 增量、光线步长、膨胀半径等需反复试，电脑上改编译即可 |
| 避免损坏硬件 | PETG 结构强度有限，未经充分验证前真机运行风险高 |

### 1.2 非目标（本方案不做）

- 不做 SLAM 定位（扫描匹配 / 回环 / 位姿图）——这是 Cartographer / RTAB-Map 的职责，建图模块只做「地图累积」这一半。
- 不做真机部署。本方案产物是「电脑端测试工具 + main.cpp 可视化增强」，部署留待后续阶段。
- 不做 3D 地形（elevation_mapping / 3D 雷达）——本方案只涉及 2D 栅格图。

---

## 二、现状盘点

### 2.1 已有能力（已经跑通的）

| 模块 | 文件 | 能力 | 状态 |
|---|---|---|---|
| 点云反投影 | `point_cloud.h/.cpp` | `depth_to_cloud`：深度图(mm)→点云(camera_optical，米)；`transform_optical_to_link`；`transform_to_base`（含 CameraExtrinsics 外参） | ✅ 已实现+单测 |
| 建图核心 | `mapping.h/.cpp` | `OccupancyGridMap`：base 系点云 + Pose2D → odom 系变换 → 光线空闲标记 → log-odds 占据更新 → 膨胀 → PGM 导出 | ✅ 已实现+40 断言单测 |
| Astra 驱动 | `sensor_astra.h/.cpp` | `AstraProDriver`：真机 `capture_real()`（Astra SDK 轮询取帧）/ 模拟 `simulate_frame()`；`get_color_frame()` 彩色帧 | ✅ 真机验证过 |
| 可视化窗口 | `main.cpp` | Windows GDI+ 程序，`draw_cloud_view`（点云俯视/侧视）、`draw_color_frame`、双缓冲、`window_thread` | ✅ 已实现 |
| 真机建图工具 | `tools/mapping_real_test.cpp` | 真深度帧 → 建图 → PGM 落盘 + 数值统计 | ✅ 仅验证数值，不出图 |

### 2.2 缺失环节（本方案要补的）

1. **`main.cpp` 主循环没有把 base 系点云喂给建图。** 当前点云只喂给了地面分割（`segment_ground`），`OccupancyGridMap` 从未被实例化/更新。
2. **`main.cpp` 没有可视化 `OccupancyGridMap`。** 窗口只渲染点云投影，不渲染 2D 占据栅格图。
3. **没有轨迹可视化。** 建图需要 `Pose2D`，但 `main.cpp` 里**没有任何位姿源**（机器人静止假设）。
4. **深度数据「导入」方式未统一。** 用户关心的是「Astra 深度数据能否直接导入」（不先录文件）。答案是**能**，有实时 + 回放两条路，见 §3.2。

---

## 三、关键技术前提与选型

### 3.1 建图需要位姿（核心前提）

`OccupancyGridMap::insert_cloud(const PointCloud& cloud_base, const Pose2D& robot_pose)` 必须吃一个 `Pose2D{x,y,theta}`（odom 系下 base_link 的位姿）。位姿来源决定「轨迹」与「地图」的真假。有三种取值路径：

| 路径 | 位姿来源 | 轨迹真实性 | 适用 |
|---|---|---|---|
| A. 静止/匀速假设 | 恒定 `(0,0,0)` 或按帧序线性推进 `theta`（`mapping_real_test` 现有做法） | 假轨迹、开环必漂 | 先验证「深度→建图→出图」管线 |
| B. 实时位姿 | 轮速 odom / IMU 实时算 | 真轨迹 | 需要机器人运动 + odom 源，现阶段不现实 |
| C. 回放位姿+点云 | 从预存轨迹/点云文件读 | 可控、可复现 | 调参、看完整建图效果，不依赖相机/机器人 |

**本方案采用「A（先跑通）+ C（看效果）+ 预留 B（后续接）」。**

### 3.2 深度数据「直接导入」的两种方式

用户确认 Astra 相机**已连接本机**。因此支持两种导入，且**不需要先录成文件**：

- **方式 ①：实时采集（推荐，现在就能用）**
  Astra 连在 Windows 电脑 → `AstraProDriver(true=false)` → `init_hardware()` → `start()` → `capture_frame()`（内部走 Astra SDK 轮询 `capture_real()`）→ 得到 `AstraFrame.depth_map` → `depth_to_cloud` → 建图。全程实时，不落盘。

- **方式 ②：离线回放（备选，便于反复调参）**
  先用任意方式录下 `{timestamp, depth_map, pose}` 序列存入文件，工具读档后逐帧喂给建图。适合「同一段数据反复调参」「相机不在手边时看效果」。

> **结论**：所谓「深度数据直接导入」是可行的——Astra SDK 的 `capture_frame()` 每次直接返回深度帧，无需中间文件。你现有的 `main.cpp`（`--real`）与 `mapping_real_test` 都已走通这条路。

### 3.3 可视化现状与目标

`main.cpp` 窗口已能渲染点云俯视图（`draw_cloud_view`），但缺少：
- 2D 占据栅格图的渲染（把 `OccupancyGridMap` 的 `occ_state` 画成 0=占据/自由/未知的色块）；
- 机器人轨迹（把一路 `Pose2D` 连成折线）；
- 建图状态栏（占据/空闲/未知计数）。

目标布局见 §5.3。

---

## 四、总体方案

### 4.1 技术路线

在**不破坏现有融合/避障/点云可视化功能**的前提下，向 `main.cpp` 增量注入「建图 + 建图可视化」：

```
Astra capture_frame() ──► depth_to_cloud ──► transform_to_base
                                          │
                                          ├──► (现有) segment_ground ──► 点云可视化(不变)
                                          │
                                          ├──► (新增) OccupancyGridMap::insert_cloud(cloud_base, pose)
                                          │              │
                                          │              └──► 更新共享栅格数据 + 轨迹
                                          │
                                          └──► (窗口线程重绘) draw_map_view(占据图) + draw_trace(轨迹)
```

### 4.2 方案分两个产物

| 产物 | 内容 | 依赖 | 阶段性 |
|---|---|---|---|
| **产物 1：离线仿真建图可视化工具** | 合成房间 + 机器人轨迹 + 相机扫描 → 跑 `OccupancyGridMap` → 出「轨迹+点云+占据图」的整图 PNG | 仅标准库 + 现有 point_cloud/mapping，不依赖相机 | ✅ 先用它看效果（不接相机） |
| **产物 2：main.cpp 真机可视化增强** | 在现有 GDI+ 窗口里，把每一帧真深度建图结果渲染成占据图 + 轨迹叠加，实时看 | Astra SDK + USE_ASTRA_SDK | ⏳ 接相机后开 |

两者共一套**栅格渲染逻辑**：先写「离线的、可复用的 `render_map_occupancy()`」，再让 `main.cpp` 复用同一套渲染。

### 4.3 关键设计决策

| 决策 | 选择 | 理由 |
|---|---|---|
| 建图实例化位置 | 全局单例 `OccupancyGridMap g_mapper`（主循环写，窗口读，`g_viz_mutex` 保护） | 与现有 `g_latest_cloud`/`g_latest_result` 同模式 |
| 位姿 | 阶段 A：`Pose2D pose` 恒定或按帧推进（可配置 `--sweep <deg>`）；预留 B | 先跑通管线 |
| 栅格渲染 | 新增 `draw_map_view()`，读 `g_mapper` 的 `occ_state(col,row)` | 与点云视图同屏 |
| 轨迹渲染 | 新增 `std::vector<Pose2D> g_trace`，主循环每帧 append；窗口折线绘制 | 与点云/地图同屏 |
| 数据共享 | 新加 `g_viz_mutex` 保护的 `g_map_snapshot`（宽高+栅格拷贝） | 避免窗口线程直接读建图对象（竞态） |

---

## 五、详细设计

### 5.1 数据链路（主循环）

在 `main.cpp` 主循环（约 line 796 附近，已有 `if (show_cloud)` 点云段）内，新增一段「建图更新」：

```cpp
// (在得到 cloud_base 之后、加锁写共享之前)
if (g_enable_mapping) {
    // 位姿: 阶段 A 恒静止/匀速; 阶段 B 从 odom 读 (待接入)
    Pose2D pose = g_current_pose;   // 全局, 由 --static / --sweep 驱动

    // 建图: 把 base 系点云喂进去 (可传入降采样后的点云, 控制开销)
    g_mapper.insert_cloud(cloud_base, pose);

    // 记录轨迹
    g_trace.push_back(pose);

    // 生成栅格快照供窗口线程读 (拷贝 occ_state, 而非整个 grid, 控制拷贝成本)
    {
        std::lock_guard<std::mutex> lock(g_viz_mutex);
        g_map_snapshot.w = g_mapper.width();
        g_map_snapshot.h = g_mapper.height();
        g_map_snapshot.cells.assign(g_mapper.width() * g_mapper.height(), 0);
        for (int i = 0; i < g_mapper.width() * g_mapper.height(); ++i)
            g_map_snapshot.cells[i] = g_mapper.occ_state(i);
        g_map_snapshot.valid = true;
        g_trace_snapshot = g_trace;
    }
}
```

> 说明：`OccupancyGridMap::occ_state(idx)` 返回 -1/0/100（未知/空闲/占据），可直接用于渲染配色。为控制线程间拷贝成本，快照存 `int8_t` 的 `occ_state` 摘要而非整个 `grid_`。

### 5.2 渲染函数设计

新增两个渲染函数，复用现有 GDI+ 双缓冲 `Graphics g`：

```cpp
// 渲染 2D 占据栅格图 (俯视): 0=未知(深灰) 0空闲(浅灰) 100=占据(橙)
static void draw_map_view(Gdiplus::Graphics& g, int x, int y, int cw, int ch,
                          const std::vector<int8_t>& cells, int w, int h,
                          double resolution_m);

// 渲染机器人轨迹 (折线, 世界米 → 屏幕像素)
static void draw_trace(Gdiplus::Graphics& g, int x, int y, int cw, int ch,
                       const std::vector<Pose2D>& trace, double resolution_m);
```

坐标换算复用地图内部约定（`index_to_world`）：`col↔x, row↔y`；渲染时把世界系 `(wx,wy)` 映射到屏幕矩形 `(x,y,cw,ch)` 内即可。

### 5.3 可视化布局（`draw_scene_impl`）

新增一个「建图模式」分支（例如命令行 `--mapmap` 触发，或复用 `--cloud` 时在右侧再叠一块地图）。推荐布局：

```
┌───────────────────────────────┬──────────────────────────┐
│    深度彩色帧 / 点云俯视图       │   建图占据栅格图 (俯视)     │
│     (draw_color_frame /       │   (draw_map_view)         │
│      draw_cloud_view)         │   [橙=占据 浅灰=空闲 深灰=未知]│
│                               │                           │
│                               │   + 机器人轨迹折线        │
├───────────────────────────────┴──────────────────────────┤
│  状态栏: env / min_fwd / action / vel / map: occ=.. free=.. │
└─────────────────────────────────────────────────────────┘
```

- **左屏**：保留现有深度/点云视图。
- **右屏**：新增建图占据图 + 轨迹。
- **状态栏**：追加 `map_stats`（`OccupancyGridMap::stats()` 输出 unknown/free/occ 计数）。

### 5.4 命令行开关

| 参数 | 含义 | 默认 |
|---|---|---|
| `--map` | 启用建图可视化 | 关 |
| `--static` | 位姿恒静止（原点朝+x） | 开（与 `--map` 配套） |
| `--sweep <deg>` | 位姿按帧序线性推进航向（原地旋转扫描，匀速假设），`0`=静止 | 0 |
| `--frame-n <n>` | 透传给建图工具的真机帧数上限 | 循环 |

### 5.5 CMake 变更

`mapping.cpp` 已加入 `add_executable(${PROJECT_NAME} ${SOURCES} mapping.cpp)`，建图模块已在主程序中。**无需额外 target**，仅需确保 `main.cpp` include `mapping.h`、`point_cloud.h`。

---

## 六、实现步骤（里程碑）

按「先看效果、再上真机」的顺序：

### 里程碑 M1：离线仿真工具（不接相机，先看效果）

1. 新建 `tools/mapping_sim_vis.cpp`：合成一个房间（墙 + 若干障碍），生成一条圆形/往复机器轨迹，模拟相机沿轨迹逐帧扫描。
2. 每帧：`generate_synthetic_cloud(pose)` 产出 base 系点云 → `g_mapper.insert_cloud` → 记录 `Pose2D`。
3. 跑完后用一次性 PNG/SVG 渲染：`draw_map_view` + `draw_trace`，输出一张完整图。
4. 验收：能看到房间轮廓 + 轨迹 + 占据图；墙壁稳定、无大面积假墙（验证 `mapping.cpp` 逻辑 + 外参 + 光线标记）。

**产物**：`docs/mapping_sim_output.png` + `tools/mapping_sim_vis.cpp`。

### 里程碑 M2：main.cpp 真机可视化增强

1. 在 `main.cpp` 加全局 `g_mapper` / `g_trace` / `g_map_snapshot`。
2. 主循环加「建图更新」段（§5.1）。
3. 新增 `draw_map_view` / `draw_trace`（§5.2）。
4. `draw_scene_impl` 加「建图模式」分支（§5.3）。
5. `--map` 命令行开关（§5.4）。
6. 验收：接 Astra，静止/匀速旋转，窗口里看到点云 + 占据图 + 轨迹；地图随帧累积，占据格收敛、无异常假墙。

**产物**：改 `main.cpp`，跑通真机链路。

### 里程碑 M3：后续（预留）

- 接入真实位姿（轮速 odom / IMU → `Pose2D`），替换静止/匀速假设，得到真轨迹。
- 把建图结果接给 Nav2 / 局部安全层（在 ROS 侧，非本工程）。

---

## 七、风险与应对

| 风险 | 影响 | 应对 |
|---|---|---|
| **开环位姿漂移** | 静止/匀速假设下，轨迹漂、地图出现假墙（上轮已诊断，`mapping_real_test` 同样有此问题） | M1/M2 阶段如实标注轨迹为「demo 精度」；只做链路验证，不作精度测量；M3 再接真实 odom |
| **地面点未过滤 → 假占据环** | Astra 前倾 15°，地板被投成墙（最严重） | M1 可加 `ground_segmentation`，在 `insert_cloud` 前滤地面点；或 M2 复用现有 `segment_ground` 的 `negative_points` 排除 |
| **点云量大渲染慢** | 640×480 全量约 30 万点，GDI+ 画不动 | 复用现有 `screen_decimate` / 降采样；栅格图只拷贝 `occ_state` 摘要而非整图 |
| **线程竞态** | 主循环写栅格、窗口线程读 | 新增数据用 `g_viz_mutex` 保护（与现有 `g_latest_cloud`/`g_latest_result` 同模式） |
| **位姿非真实 → 误导用户** | 假轨迹可能被误认为实际建图精度 | 文档 + 状态栏明确标注「demo/静止假设」 |

---

## 八、验收标准

**M1（离线仿真）**：
- [ ] 合成房间/障碍能跑通 `OccupancyGridMap`，出图轮廓正确；
- [ ] 「轨迹 + 点云 + 占据图」三要素都在同一张图上；
- [ ] 墙壁占据稳定，无大面积假占据环。

**M2（main.cpp 真机）**：
- [ ] 接 Astra 后能实时建图，窗口显示三要素；
- [ ] `--map` 开关可切换；
- [ ] 地图随帧累积，占据格数收敛；
- [ ] 状态栏正确显示 `map: occ=.. free=.. unknown=..`；
- [ ] 不破坏现有融合/点云可视化功能。

---

## 附：关键源码索引

| 目标 | 位置 |
|---|---|
| 建图实现 | `mapping.cpp` / `mapping.h`（`insert_cloud`、`occ_state`、`save_pgm`） |
| 点云上游 | `point_cloud.cpp` / `point_cloud.h`（`depth_to_cloud`、`transform_to_base`） |
| 真机驱动 | `sensor_astra.cpp` / `sensor_astra.h`（`capture_real`、`get_color_frame`） |
| 主循环 | `main.cpp`（约 line 780-964，`while(!g_stop)`） |
| 可视化 | `main.cpp`（`draw_scene_impl` line 397、`draw_cloud_view` line 214、`window_thread` line 638） |
| 地图参数 | `config.h`（`MapConfig`：grid_size=0.05 / map=10m / inflation=0.15） |
| 单测 | `tests/test_mapping.cpp`（40 断言，全绿） |
|坦克掉头| `通过判断自身旋转半径来判断是否可以掉头（结构强度不高，通过降低轮速和弯曲腿部降低重心防止损坏结构）下降高度和轮速待定`|
