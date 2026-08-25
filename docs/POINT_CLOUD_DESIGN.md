# mechdog_navigation 点云模块技术方案（设计文档）

> **目标定位**：为未来「路径规划 + 建图（SLAM）」预埋一个**几何感知中间层**。
> 当前阶段（v2.3 已落地）只做**反应式前向三区域避障**，深度图（Astra 640×480）里约 30 万像素的 3D 几何信息被 `analyze_region` 压缩成 3 个标量（`min_distance_m` / `valid_pixel_ratio`）后丢弃。
> 本方案在点云层把深度图还原为 3D 点云，向上支撑：①精细避障（地面分割 + 障碍物聚类 + 占据栅格）②建图（点云→世界系累积→占据栅格/octomap）③路径规划（消费占据栅格）。
>
> **范围声明**：本文档是**技术方案**，给出类型设计、算法描述、接口骨架（声明级）、坐标约定、标定与验证方案。**不含完整 .cpp 实现**。落地时按「§14 落地路线」分阶段实施。
>
> **前置文档**：`docs/FIX_PLAN.md`（v2.3）、`docs/CODE_REVIEW_BASELINE.md`。
> **相关约束**：保持算法库**零依赖、可模拟单测**的现行风格（见 `CMakeLists.txt` 仅 `Threads` 依赖）。

---

## 1. 现有基础与缺口

### 1.1 已有的（直接可用）
- `AstraFrame.depth_map`：`std::vector<uint16_t>`，640×480，单位 **mm**（模拟 `simulate_frame` 与真机 `capture_real` 均填，无效=0）。
- FOV 常量：`config.h::AstraProConfig` 含 `depth_fov_h=58.4°`、`depth_fov_v=45.5°`、`depth_width=640`、`depth_height=480`。
- 真机 RGB：`ColorFrameData.rgb`（640×480×3），可给点云上色。
- 现有地图参数：`config.h::MapConfig`（`grid_size_m=0.05`、`map_width/height=10.0`、`inflation_radius_m=0.15`）——占据栅格可直接复用，正好对接未来 DWA/global planner。
- 融合层 fail-closed 语义：`all_sensors_invalid` / `determine_action` 已成熟，**点云模块不得破坏**它。

### 1.2 缺口（必须补）
| 缺口 | 现状 | 补什么 |
|------|------|--------|
| 相机**内参** | 只有 FOV，无 `fx/fy/cx/cy` | `CameraIntrinsics`（可 FOV 反推，待真机标定） |
| 相机**外参** | 安装高度/俯角未显式存 | `CameraExtrinsics`（平移+俯仰，待真机量测标定） |
| 3D 几何层 | 融合层只用 3 个标量区域 | 新增 `Point3D` / `PointCloud` + 反投影 + 处理流水线 |
| 坐标变换 | 无相机→底盘→世界系链路 | 新增 `transform_to_base` / `transform_to_odom` |
| 点云处理 | 无 | 地面分割 + 聚类 + 占据栅格（算法库纯几何；ROS 侧用 PCL） |

---

## 2. 坐标框架定义（TF 树）

统一约定（与 ROS `REP-103` 一致，避免轴混淆坑）：

| 坐标系 | 原点 | 轴约定（右/前/上 = X/Y/Z） | 来源 |
|--------|------|--------------------------|------|
| `camera_optical` | 相机光心 | **X 右, Y 下, Z 前**（标准针孔光学系） | Astra SDK / 反投影公式 |
| `camera_link` | 相机本体 | X 前, Y 左, Z 上（与 optical 差固定旋转 `Rz(-90°)·Rx(-90°)`，见 §6.1） | 安装定义 |
| `base_link` | 底盘几何中心（轮轴中点） | X 前, Y 左, Z 上 | 机器人约定 |
| `odom` | 里程计积分原点 | 同 base，但随漂移漂移 | wheel_odom / IMU（未来） |
| `map` | 全局地图原点 | 固定世界系 | SLAM 输出（未来） |

**算法库阶段**只实现 `camera_optical → base_link`（依赖 `CameraExtrinsics`）。`base→odom→map` 由 ROS `tf2` + 里程计/SLAM 提供（未来），算法库点云结构保留 `frame_id` 字段即可。

> ⚠️ **轴约定坑**：Astra 深度图按光学系（Z 前、Y 下）出。若不做 `camera_optical→camera_link` 的固定旋转转换，点云会出现"上下颠倒/前后反"。反投影公式必须基于 optical 系，再做固定旋转 `Rz(-90°)·Rx(-90°)` 转到 link 系（见 §6.1，**不是单次 roll**），再叠加安装外参。文档化此约定并在单测中固化。

---

## 3. 相机内参 `CameraIntrinsics`

### 3.1 从 FOV 反推（算法库初始值，零标定即可跑）
针孔模型：`fx = (W/2) / tan(hfov/2)`，`fy = (H/2) / tan(vfov/2)`。

代入 `W=640, H=480, hfov=58.4°, vfov=45.5°`：
```
fx = 320 / tan(29.2°) = 320 / 0.5588 ≈ 572.7
fy = 240 / tan(22.75°) = 240 / 0.4193 ≈ 572.3
cx = 320, cy = 240
```
（fx≈fy **并非巧合**：Astra Pro 像素为方形，hfov/vfov 在 tan 半角域必然满足 `tan(hfov/2)/tan(vfov/2) = W/H`。验算：tan(29.2°)/tan(22.75°) ≈ 0.5588/0.4193 ≈ 1.333 = 640/480，精确吻合，故 fx≈fy 是方形像素的预期结果；真机以标定值为准。）

### 3.2 真机标定（落地时）
- 优先用 Astra SDK 直接读内参（`StreamReader` 的 `intrinsics()`），无需手标。
- 无 SDK 时棋盘角点标定（`camera_calibration`）。
- `config.h` 增加：
```cpp
struct CameraIntrinsics {
    double fx = 572.7, fy = 572.3, cx = 320.0, cy = 240.0; // FOV 反推初值, 待真机标定
    double min_depth_m = 0.6;   // 与 AstraProConfig::min_valid_mm/1000 对齐
    double max_depth_m = 8.0;   // 与 max_valid_mm/1000 对齐
};
```

---

## 4. 相机外参 `CameraExtrinsics`

相机相对 `base_link` 的安装位姿。机械狗 Astra 通常**前装、略向下俯**以看近地障碍。

```cpp
struct CameraExtrinsics {
    double x = 0.12;   // 相机相对底盘中心: 前 12cm (待量测)
    double y = 0.0;    // 左右居中
    double z = 0.18;   // 离地高 18cm (待量测)
    double roll  = 0.0;
    double pitch = +15.0 * M_PI/180.0; // 前俯 15° (待量测/手眼标定)
    double yaw   = 0.0;
};
```
- 初值凭机械结构估计，落地时用卷尺 + 倾角仪量测，或手眼标定精修。
- **pitch 符号约定**（与 §2 link 系 {X前, Y左, Z上} + 标准 ZYX 内旋 `R=Rz(yaw)·Ry(pitch)·Rx(roll)` 一致）：相机前向轴 `[1,0,0]` 经 R 后在 base 系的 Z 分量 = `-sin(pitch)`。要让光轴朝下（Z 分量 < 0），需 `pitch > 0`，故前倾俯视取 **+15°**。若改用非标准 Euler 约定（如绕 -Y 轴、外旋 XYZ），须在此显式声明并在单测中用"前向轴 Z 分量为负"固化。

---

## 5. 深度图 → 点云反投影

### 5.1 公式（基于 optics 系，Z 前 Y 下）
像素 `(u,v)`、深度 `d`（米，非 0）：
```
X = (u - cx) * d / fx     // 右
Y = (v - cy) * d / fy     // 下
Z = d                     // 前
```
### 5.2 单位与无效处理（与现有口径一致）
- `depth_map` 单位 mm → 反投影前 `d = px / 1000.0`。
- 无效点（`px==0` 或 `d < min_depth_m` 或 `d > max_depth_m`）丢弃——与 `analyze_region` 同一有效性口径，保证点云与三区域感知对"有效"的定义一致。

### 5.3 接口骨架（声明级，非实现）
```cpp
// point_cloud.h
struct Point3D { double x, y, z; uint8_t r=0, g=0, b=0; }; // 颜色可选(真机 RGB 上色)
struct PointCloud {
    uint64_t seq = 0;
    double   stamp = 0.0;
    std::string frame_id = "camera_optical";
    std::vector<Point3D> points;
};

// 深度图(mm) -> 点云(optical 系, 米)
void depth_to_cloud(const uint16_t* depth, int w, int h,
                    const CameraIntrinsics& K, PointCloud& out);
```

---

## 6. 坐标变换

### 6.1 `camera_optical → camera_link`
optical 系 {X右, Y下, Z前} → link 系 {X前, Y左, Z上}，按 §2 轴定义逐轴推导：

| 输出 | = | 物理含义 |
|------|---|----------|
| `X_link` | `Z_opt`  | 前 ← 前 |
| `Y_link` | `-X_opt` | 左 ← 右的相反 |
| `Z_link` | `-Y_opt` | 上 ← 下的相反 |

写成矩阵：
```
[ X_link ]   [ 0  0  1 ] [ X_opt ]
[ Y_link ] = [-1  0  0 ] [ Y_opt ]   // optical -> link (ROS body 系)
[ Z_link ]   [ 0 -1  0 ] [ Z_opt ]
```
> ⚠️ **不要用单次 roll**：optical 的 Z(前) 要落到 link 的 X(前)，光 roll 只能得到 {X右, Y前, Z上}，Y/Z 仍错。正确分解是 **`R = Rz(-90°) · Rx(-90°)`**（先 -90° roll 把 Y下/Z前 转成 Y前/Z上，再 -90° yaw 把 X右/Y前 转成 X前/Y左）。此旋转是相机坐标系的**固定属性**，与安装无关，可直接硬编码上面的 3×3。
> 单测固化：正前方墙质心应落到 `X_link > 0, Y_link ≈ 0, Z_link ≈ 0`。

### 6.2 `camera_link → base_link`
应用 `CameraExtrinsics` 的平移 + 旋转（roll/pitch/yaw，ZYX 欧拉或四元数）：
```
p_base = R_extrinsic * p_link + t_extrinsic
```
手写旋转矩阵（绕 X/Y/Z 的 3×3 乘积）即可，零依赖；ROS 侧用 `tf2`/`Eigen`。

```cpp
// 声明级
void transform_to_base(const PointCloud& in, const CameraExtrinsics& E,
                       PointCloud& out); // out.frame_id = "base_link"
```

### 6.3 `base → odom → map`（未来）
算法库点云仅携带 `frame_id`，实际变换由 ROS `tf2` 在集成时完成（消费 wheel_odom/IMU/SLAM 位姿）。算法库**不实现**此层，避免重复造轮子。

---

## 7. 点云处理流水线（为避障 + 建图）

```
depth_map ──▶ depth_to_cloud ──▶ 滤波(去噪/下采样) ──▶ transform_to_base
                   │                                            │
                   ▼                                            ▼
             [可选 RGB 上色]                         地面分割 ──▶ 障碍点云
                                                          │
                                                          ▼
                                              欧式聚类 ──▶ 障碍簇(质心/包围盒)
                                                          │
                                                          ▼
                                              占据栅格(投影 x-y) ──▶ 规划层/建图层
```

### 7.1 滤波
- **Voxel 下采样**：按空间格子（如 2cm）取代表点，降数据量、去高频噪声。算法库可纯写一个 `voxel_downsample(cloud, leaf=0.02)`（哈希格子取首点/均值）。零依赖。
- **统计离群点移除（SOR）**：ROS 侧用 PCL `StatisticalOutlierRemoval`；算法库阶段若需要可写简化版（邻域距离阈值），但非必需。

### 7.2 地面分割（关键）
目的：从点云中剔除地面，余下为障碍（避障所需）；建图时地面可单独保留（占用栅格的"可行走"层）。

**算法库方案（纯几何 RANSAC 平面拟合）**：
1. 点已转到 `base_link` 系（Z 上）。
2. 用 RANSAC 拟合平面 `a·x + b·y + c·z + d = 0`：
   - 随机抽 3 点 → 解法向量 → 统计内点（|a·p+b·p+c·p+d| < `ground_thr`，如 0.03m）。
   - 迭代 N 次取内点最多者。
3. 判为地面需法向接近 (0,0,1)（`|c| > 0.9` 且 `a,b` 近 0），防把"一面墙"误判成地面。
4. 内点→`ground`；外点→`obstacles`。

```cpp
// 声明级
struct Plane { double a,b,c,d; };
void segment_ground(const PointCloud& in, Plane& ground,
                    PointCloud& obstacles, double thr=0.03);
```
> 简化近似：若相机俯角已知且场景平坦，可用"Z < (相机高 - margin) 且 局部平整"的启发式替代 RANSAC，但精度差，建议直接写最小 RANSAC（~30 行）。

**ROS 方案**：PCL `SACSegmentation`（`SAC_RANSAC` + `SACMODEL_PLANE`），精度更高、已调优。

### 7.3 障碍物欧式聚类
将障碍点按空间距离聚类，每簇给出质心 + 轴对齐包围盒（AABB），供决策"哪个方向有障碍、多大、多远"。

```cpp
// 声明级
struct Cluster { Point3D centroid; double min_x,max_x,min_y,max_y,min_z,max_z; size_t n; };
void euclidean_cluster(const PointCloud& obstacles, double tol,
                       std::vector<Cluster>& out);
```
- 算法库：用 voxel 栅格连通域（BFS）近似，零依赖；或简单欧式距离阈值 + 并查集。
- ROS 侧：PCL `EuclideanClusterExtraction`。

### 7.4 占据栅格（对接路径规划）
把 `obstacles` 投影到 `base_link` 的 x-y 水平面，落到 `MapConfig` 栅格（`grid_size_m=0.05`，`10×10m`），按 `inflation_radius_m=0.15` 膨胀。输出 `OccupancyGrid`——**这正是未来 DWA/global planner 直接消费的格式**。

```cpp
// 声明级 (复用 config.h::MapConfig 参数)
struct OccupancyGrid {
    int w, h; double resolution; double origin_x, origin_y;
    std::vector<int8_t> cells; // 0=free, 100=occupied, -1=unknown
};
void build_occupancy_grid(const PointCloud& obstacles, OccupancyGrid& grid);
```

---

## 8. 数据结构总览（算法库零依赖）
全部用 `std::vector` / 标量，不引 PCL：
```cpp
Point3D, PointCloud, CameraIntrinsics, CameraExtrinsics,
Plane, Cluster, OccupancyGrid
```
头文件 `point_cloud.h`，实现 `point_cloud.cpp`。**纯标准库，可模拟单测**。

---

## 9. 模块分层（混合方案，推荐）

### 9.1 算法库 `mechdog_navigation`（验证层）
- 新增 `point_cloud.h/.cpp`：类型 + 反投影 + 变换 + 地面分割 + 聚类 + 占据栅格。**零依赖、可单测**。
- `CMakeLists.txt`：把 `point_cloud.cpp` 加入 `test_fusion` 的源（保持模拟可测），不引入 PCL。
- 与融合层解耦：点云模块**独立**于 `SensorFusion`，由 `main`/未来 planner 调用；不破坏现有三区域 fail-closed。

### 9.2 ROS 胶水包 `mechdog_navigation_ros`（真机层）
- `point_cloud_bridge`：订阅 Astra `depth_image` → `sensor_msgs/PointCloud2` → PCL 节点（去地面、聚类）→ 发布 `obstacles` / `occupancy_grid` → `safety_node` / `planner_node`。
- 依赖：`pcl`, `pcl_conversions`, `sensor_msgs`, `tf2`, `tf2_ros`, `Eigen`。
- `safety_node` 消费点云障碍簇/占据栅格时，**必须仍尊重算法库 `recommended_action` 的 fail-closed**（见 §10、§R5 跨层一致性）。

---

## 10. 与现有融合层的关系（不破坏、互补）

| 层 | 输入 | 输出 | 角色 |
|----|------|------|------|
| 现有三区域融合 | 深度 3 区域 + 超声 | `recommended_action`（STOP/BACK/TURN…） | **反应式快速决策**（低延迟、fail-closed 成熟） |
| 新增点云层 | 整帧深度 | 障碍簇 + 占据栅格 | **几何精细感知**（供规划/建图，延迟容忍高） |

- **点云层不替代**三区域融合。三区域继续做硬急停；点云层做"更聪明"的绕障/规划。
- **fail-closed 不被点云削弱**：点云处理失败（异常/超时）时，fusion 的 `recommended_action` 仍是唯一决策源；点云只"增强"不"否决"。
- 点云的 `sensors_valid` 概念与 `all_sensors_invalid` 对齐：点云空（全地面/全无效）→ 不产出障碍，但不应触发比三区域更激进的 FORWARD。

---

## 11. 为路径规划 / SLAM 预留

| 未来能力 | 点云层需提供的 | 备注 |
|---------|--------------|------|
| 反应式绕障（已部分有） | 障碍簇方向/距离 | §7.3 输出 |
| DWA / 局部规划 | `OccupancyGrid` | §7.4 直接对接 `MapConfig` |
| 全局规划（A*/Dijkstra） | 同一 `OccupancyGrid`（更大地图） | 扩大 `MapConfig` 或滚动窗口 |
| 建图（占据栅格地图） | 点云→`odom/map`系累积 + voxel 下采样 | 依赖里程计位姿（未来） |
| SLAM | 点云 + 里程计 + IMU → RTAB-Map/Cartographer | ROS 生态，算法库不实现 |
| OctoMap（3D 占据） | 点云→octomap | ROS `octomap_server` |

> 算法库阶段只做到 §7（反投影→地面分割→聚类→占据栅格）。`base→map` 变换、octomap、SLAM 留待 ROS 集成期。

---

## 12. 标定方案（落地必做）

### 12.1 内参
- 首选 Astra SDK `intrinsics()` 直读。
- 次选棋盘标定。
- 算法库初值用 §3.1 FOV 反推（误差 <5%，避障够用）。

### 12.2 外参
- **平移**：卷尺量相机相对底盘中心的 x(前后)/y(左右)/z(离地高)。
- **俯仰 pitch**：倾角仪测相机前倾角；或手眼标定（已知棋盘 + 已知地形）精修。
- 写入 `CameraExtrinsics`，并在单测中固化一组"已知正确"的标定值做回归。

### 12.3 验证标定正确性
- 放一块平整墙在正前方 2.0m，点云反投影后该墙质心距应为 ≈2.0m。
- 地面在 `base_link` 系应近似 `z≈0` 平面。

---

## 13. 验证方案（算法库单测，模拟数据，零硬件）

**关键**：模拟模式 `simulate_frame` 只生成"正前方起伏墙"、无地面平面。单测需**自建带地面的测试深度图**（写一个测试用 `make_test_depth(w,h,scenario)`）。

| 测试场景 | 构造 | 期望断言 |
|---------|------|---------|
| 反投影几何 | 全像素同距 d=2.0m | 点云为**垂直于光轴的平面**，所有点 `Z≈2.0m`（注意：Astra 给的是沿光轴深度 Z，非径向距；边缘像素径向距 `sqrt(x²+y²+z²)` > 2.0，不应断言径向≈2.0。若要测径向距，需构造球面深度图按每像素径向 2.0 反算 Z） |
| 内参正确性 | 中心像素 (320,240)、d=2m | `X≈0, Y≈0, Z≈2.0` |
| 坐标变换 | 相机离地 0.18m、俯角 0 | 地面点（Z=0 世界）在相机**下方**，光学系 Y 下为正 → 相机系 `Y≈+0.18`；经 §6.1 矩阵转 link 后 `Z_link=-Y_opt≈-0.18`，再叠加外参平移 `z=0.18` → base 系 `z≈0` ✅ |
| 地面分割 | 构造 z=0 地面 + 前方 2m 一块障碍 | `ground` 点数 ≫ `obstacles`；障碍簇质心距 ≈2.0m |
| 聚类 | 前方左右各一块障碍（间隔 > tol） | 得 2 簇，质心分别在其位置 |
| 占据栅格 | 前方 1.5m 障碍点 | 对应栅格 `cells==100`，并按 `inflation_radius` 膨胀邻格 |
| 失效哨兵 | 全 0 深度图 | `depth_to_cloud` 输出空点云，不崩溃（与现有 `valid_pixel_ratio==0` 口径一致） |

- 用 `g++ -std=c++20 -fsanitize=address,undefined` 跑，零 UB（延续现行验证纪律）。
- 以上可全部在无硬件下全绿，符合本项目"模拟验证优先"工作流。

---

## 14. 依赖与构建影响

- **算法库**：零新增依赖（仅 `<vector>`/`<cmath>`/`<random>`）。`CMakeLists.txt` 把 `point_cloud.cpp` 加进 `test_fusion` 源列表即可；不影响现有模拟构建。
- **ROS 胶水包**：新增 `pcl`, `pcl_conversions`, `sensor_msgs`, `tf2`, `tf2_ros`, `Eigen`（已在 ROS 生态内，无新风险）。

---

## 15. 风险与坑

| 风险 | 后果 | 规避 |
|------|------|------|
| 光学系/link 系轴约定混淆 | 点云上下颠倒/前后反 | §2 固化约定 + §13 单测固化 |
| 深度图单位错（mm↔m） | 点云缩放 1000 倍 | 反投影前显式 `/1000`，单测断言量纲 |
| Astra 俯角不足看不到近地 | 近地处仍盲区 | 保留底部超声 fail-closed 防跌落（与现设计互补），点云不负责近地 |
| 点云频率过高拖垮 CPU | Astra 30fps 点云处理重 | ROS 侧降频（如 10Hz）+ voxel 下采样 |
| 外参未标定 | 点云在底盘系位置错 | §12 量测 + 单测固化标定值 |
| 点云失败被当"无障碍" | fail-open 重现 | §10 点云只增强不否决，fusion 仍为硬决策源 |

---

## 16. 落地路线建议（分阶段）

| 阶段 | 内容 | 交付 | 依赖 |
|------|------|------|------|
| **P0** | `point_cloud.h/.cpp`：类型 + `depth_to_cloud` + `transform_to_base` + `CameraIntrinsics/Extrinsics` | 反投影 + 坐标变换单测全绿 | 无（FOV 反推内参即可） |
| **P1** | 地面分割（RANSAC 平面）+ 欧式聚类（栅格连通域） | 障碍簇质心/包围盒单测 | P0 |
| **P2** | 占据栅格 `build_occupancy_grid`（复用 `MapConfig`） | 栅格单测；输出接未来 planner | P1 |
| **P3** | ROS 集成：`point_cloud_bridge` + PCL 节点 + `safety_node` 消费 | 真机点云避障 | 内参 SDK 直读 + 外参量测标定 |
| **P4** | 建图/SLAM：`base→map` 变换 + voxel 累积 + octomap/costmap | 占据地图 | 里程计/IMU（未来）、P3 |

> **建议**：先落地 P0~P2（纯算法库、可模拟验证、零依赖），把点云能力"预埋"好；P3/P4 等真机标定 + ROS 规划层就绪后再做。当前 v2.3 任务完成后，P0 是低成本高价值的下一步。

---

## 17. 结论

在现有深度图基础上加點云，**技术风险低、收益高**：内参可 FOV 反推零标定起步，反投影是标准针孔模型，处理流水线（地面分割/聚类/占据栅格）全部可纯 C++ 实现并模拟单测，且 `MapConfig` 已为占据栅格预留参数、可直接对接未来路径规划。

唯一硬性新增工作是**补内参/外参并标定**，以及**固化坐标轴约定**（最大坑）。点云层作为"几何精细感知"与现有三区域 fail-closed 融合**互补而非替代**，不破坏已验证的安全语义。

按 P0→P2 在算法库落地，即可为未来的路径规划与 SLAM 准备好几何中间层，且全程可模拟验证、符合本项目现行工程纪律。

---

## 18. 现实约束：深度相机已接、机械狗未接的开发边界

> 记录一个实际开发阶段的状态：**Astra Pro 真机已通过 USB 出深度帧（`USE_ASTRA_SDK=ON`，`capture_real()` 已验证双流 640×480 出帧），但机械狗底盘未接线/未装机**。
> 本节明确在此状态下「点云模块现在能写/能验什么、不能验什么」，避免把"外参未定"误当成"点云做不了"。

### 18.1 先决条件：数据源是否真通
点云一切的源头是 `AstraFrame.depth_map` 真有值。本状态下需先确认：
- 构建为 `USE_ASTRA_SDK=ON`，`init_hardware()` + `start()` 成功；
- 一次 `get_latest_frame()` 后 `frame.valid==true` 且 `depth_map` 非空、非全 0。
代码注释（D1 修复）称真机"深度+彩色双流 640×480 正常出帧"，但实际开发时仍应打印一帧 `valid` 与深度像素统计坐实。**这一点成立，点云数据源即通，与机械狗无关。**

### 18.2 现在就能做（狗不在场，100% 可行）

| 能力 | 说明 | 约束 |
|------|------|------|
| 反投影 `depth_to_cloud` | 标准针孔模型，FOV 反推内参即可跑 | 内参为典型值（误差<5%），非个体标定 |
| 轴约定固化 + 单测 | 光学系→link 系固定旋转 `Rz(-90°)·Rx(-90°)`，纯数学（见 §6.1） | 不受狗影响；**此刻正是验证轴约定的好时机**（见 §18.4） |
| 模拟单测 7 场景全绿 | §13 全部用自建测试深度图，零硬件 | 比有狗时更可控 |
| 真机点云形状联调 | 拿真 `depth_map` 反投影，看几何形状（墙=平整面、桌=水平面） | 仅验证"形状对"，非"机器人系位姿对" |
| 地面分割/聚类/占据栅格算法 | 输入是任意点云，与谁采集无关 | 算法正确性与狗无关 |
| 与融合层/fail-closed 解耦 | 点云独立模块，不碰三区域融合 | 不互相干扰 |

### 18.3 现在做不了（必须等相机装到狗上）

| 能力 | 卡点 |
|------|------|
| 外参精确定位 | 外参 = 相机相对底盘安装位姿；相机未装机，无"相对底盘"关系，只有"临时摆放"姿态 |
| 点云在 base_link 系真实位姿 | 依赖外参（§4/§6.2） |
| 点云与底盘运动的 `base→odom` 对齐 | 需要里程计，本状态无底盘 |
| 真机外参标定 | 需装机后卷尺+倾角仪量测，或手眼标定（§12.2） |
| 占据栅格"狗踩在地上"的真实尺度验证 | 需底盘实际位姿 |

### 18.4 临时摆放下的联调方法（手持/桌上相机）

本状态下相机姿态是临时的，但**正好适合验证轴约定与几何**，不浪费：

1. **设外参初值占位**：`CameraExtrinsics` 取 `x=y=z=0, pitch=roll=yaw=0`（相机在原点、无俯角）。此时点云在"相机 optical 系"下，形状正确即可看，不要求机器人系位置。
2. **轴约定验证（最该此刻做的）**：把相机正对一面墙、距离约 2.0m → 反投影后点云应满足：墙质心位于 **+Z 方向 ≈2.0m、X≈0、Y≈0**。若出现 `Z` 变负/`Y` 异常大/点云上下颠倒 → 立即暴露 §15 "光学系/link 系轴约定混淆" 坑，比装机后再调省力。
3. **几何形状验证**：对着桌面 → 点云应是一张水平平整面；对着一个盒子 → 应出现一块凸起的簇。验证 `depth_to_cloud` 的量纲（`mm→m`）与内参反推是否正确（盒子尺寸数量级应对）。
4. **不验证**：点云"在机器人坐标系下的绝对位置" —— 此刻无狗，验证无意义，等装机填外参后再做 §12.3 标定正确性验证。

### 18.5 结论边界（一句话）

**本状态可立即开写 P0~P2（反投影→地面分割→占据栅格）并让模拟单测全绿、用真机深度图做形状级联调；唯一代价是外参现在只能占位/设初值，等相机装到狗上再填一次实测值。** 而"装机后填外参"本是 §12 规划的落地必做标定步骤，并非额外负担——故机械狗未接**不构成点云开发的阻塞**。

> 注意 §3.2 的提醒：FOV 反推内参是 Astra Pro 的**典型值**，个体间有微小差异；真机精值优先用 SDK `intrinsics()` 直读（当前 `capture_real` 未调用，需补几行）。本状态联调若发现墙距偏差 >5%，优先怀疑内参而非轴约定。
