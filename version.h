/**
 * version.h — mechdog_navigation 版本单点声明 (B17 跨仓版本锁定)
 *
 * 背景: ROS 包 (mechdog_navigation_ros) 直接编译本库源码 (MECHDOG_ALGO_DIR),
 *       旧行为只查"文件在不在"、不查版本 ⇒ "半批部署/新旧混合"会静默漂移
 *       (v2.8→v2.9.16 曾再发生: 缺件才报错, 语义漂移无声)。
 *
 * 机制 (双道):
 *   1) 构建期 (主): ROS 包 CMakeLists 解析本文件的 MECHDOG_CORE_VERSION_CODE,
 *      与 MECHDOG_CORE_VERSION_REQUIRED 不等 ⇒ configure 期直接 FATAL。
 *   2) 编译期 (兜底): ROS 侧注入 MECHDOG_ROS_REQ_CORE_VERSION 后,
 *      任何 include 本头的 TU 由下方 static_assert 再断一次。
 *
 * ★ 升级纪律 (两仓必须同批):
 *   1) 本文件 PATCH+1 (如 2.9.22 → 2.9.23), 并同步 STR 与 CODE;
 *   2) mechdog_navigation_ros/CMakeLists.txt 的 MECHDOG_CORE_VERSION_REQUIRED 同步;
 *   3) 两仓同时提交/推送。
 *   任一侧漏改 ⇒ ROS 侧构建失败 —— 这是设计行为, 不要绕过。
 */
#pragma once

#define MECHDOG_CORE_VERSION_MAJOR 2
#define MECHDOG_CORE_VERSION_MINOR 9
#define MECHDOG_CORE_VERSION_PATCH 23
#define MECHDOG_CORE_VERSION_STR   "2.9.23"
/* 机器可读整数版号 = MAJOR*10000 + MINOR*100 + PATCH (CMake 侧解析这一行) */
#define MECHDOG_CORE_VERSION_CODE  20923

/* 自洽检查: CODE 必须等于 MAJOR/MINOR/PATCH 组合 (改版本时防手误) */
static_assert(MECHDOG_CORE_VERSION_CODE ==
                  MECHDOG_CORE_VERSION_MAJOR * 10000 +
                      MECHDOG_CORE_VERSION_MINOR * 100 +
                          MECHDOG_CORE_VERSION_PATCH,
              "version.h 自身不一致: CODE 与 MAJOR/MINOR/PATCH 不匹配");

/* 跨仓锁定: ROS 构建注入 MECHDOG_ROS_REQ_CORE_VERSION 时启用 */
#ifdef MECHDOG_ROS_REQ_CORE_VERSION
static_assert(MECHDOG_CORE_VERSION_CODE == MECHDOG_ROS_REQ_CORE_VERSION,
              "跨仓版本不一致 (B17): 核心算法库版本 != ROS 包锁定版本 —— "
              "两仓需同批升级 (core/version.h 与 ros CMakeLists 的 MECHDOG_CORE_VERSION_REQUIRED)");
#endif
