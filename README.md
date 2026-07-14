# mechdog_navigation

机械狗导航系统 —— 基于 Astra Pro 深度相机 + HC-SR04 超声波传感器阵列的多传感器融合寻路方案。

## 项目简介

本项目为四足机械狗提供实时障碍物检测与自主导航能力。系统融合**奥比中光 Astra Pro 深度相机**（单目结构光）与 **4 颗 HC-SR04 超声波传感器**，实现多距离层次、多环境自适应的传感器融合策略，最终输出导航动作指令（前进/转向/停止/后退）。

## 系统架构

| 模块 | 文件 | 功能 |
|------|------|------|
| 全局配置 | config.h | 传感器参数、融合分层策略、地图参数、规划参数、紧急避障阈值 |
| 超声波驱动 | sensor_ultrasonic.h/.cpp | HC-SR04 超声波传感器驱动，支持 4 颗并行读取（含模拟模式） |
| 深度相机驱动 | sensor_astra.h/.cpp | Astra Pro 深度相机驱动，通过 OpenNI2 获取深度图（含模拟模式） |
| 传感器融合 | sensor_fusion.h/.cpp | 多传感器融合核心：分层加权、环境自适应、障碍物分类、导航决策 |
| 构建脚本 | CMakeLists.txt | CMake 构建配置 |

## 传感器融合策略

### 距离分层

| 层级 | 距离范围 | 策略 | 说明 |
|------|---------|------|------|
| L0 | 0 – 0.6m | 仅超声波 | Astra Pro 盲区补偿 |
| L1 | 0.6 – 3m | 融合保守 | 取两者中更近的值 |
| L2 | 3 – 8m | Astra 为主 | Astra 权重 0.9 |
| L3 | > 8m | 仅 Astra | 超出超声波量程 |

### 环境自适应权重

| 环境 | Astra 权重 | 超声波权重 |
|------|-----------|-----------|
| 室内 indoor | 0.8 | 0.2 |
| 半室内 semi_indoor | 0.5 | 0.5 |
| 室外 outdoor | 0.1 | 0.9 |

## 超声波传感器布局

| 传感器 | 位置 | 偏航角 | 俯仰角 | 功能 |
|--------|------|--------|--------|------|
| front_left | 左前 | -30° | 0° | 覆盖左前方盲区 |
| front_center | 正前 | 0° | 0° | 检测正前方障碍物 |
| front_right | 右前 | +30° | 0° | 覆盖右前方盲区 |
| bottom | 底部朝下 | 0° | -90° | 检测地面悬崖/台阶（防跌落） |

## 紧急避障阈值

| 距离 | 等级 | 动作 |
|------|------|------|
| < 10cm | 临界 CRITICAL | 强制停止 |
| 10 – 25cm | 危险 DANGER | 减速 |
| 25 – 50cm | 警告 WARNING | 正常行驶 |
| > 50cm | 安全 SAFE | 正常行驶 |

## 依赖项

- **CMake** >= 3.16
- **C++17** 编译器
- **OpenNI2**（可选，Astra Pro 真实硬件模式）
- **WiringPi**（可选，树莓派 GPIO 模式）
- **Boost**（boost::optional）

## 构建

`ash
git clone https://github.com/justdoti666/mechdog_navigation.git
cd mechdog_navigation
mkdir build && cd build

# PC 模拟模式（无需硬件）
cmake ..

# 树莓派 WiringPi 模式
cmake .. -DUSE_WIRINGPI=ON

# Astra Pro OpenNI2 模式
cmake .. -DUSE_OPENNI2=ON

# 完整硬件模式
cmake .. -DUSE_WIRINGPI=ON -DUSE_OPENNI2=ON

cmake --build .
`

## 模拟模式

当 USE_WIRINGPI 和 USE_OPENNI2 均未启用时，系统进入模拟模式：
- 超声波传感器返回随机模拟数据（85% 空旷、10% 中距、5% 近距）
- Astra Pro 返回模拟深度帧
- 适合在 PC 上进行算法验证和开发调试

## 硬件要求

| 组件 | 型号 | 数量 |
|------|------|------|
| 深度相机 | 奥比中光 Astra Pro | 1 |
| 超声波传感器 | HC-SR04 | 4 |
| 主控 | Raspberry Pi 4B+ | 1 |

## License

MIT License
