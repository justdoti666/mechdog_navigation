/**
 * frame_fingerprint.h — 深度帧指纹 (R-8, 2026-10-12)
 *
 * 为什么需要它 (背景):
 *   safety_node 的「感知:」行与离线工装 (tools/wedge_recompute.cpp) 的输出是**逐字段比对**的,
 *   但两侧都不带"帧身份": 节点日志里只有点云数/平面/2.5D 统计, 而深度流 ~19Hz、日志 3000ms 节流
 *   ⇒ 10s 只有 4~9 行。于是"离线算的是哪一帧"只能靠"点云数在日志里唯一"**间接**论证
 *   (2026-10-10 复核报告 R-8)。本头文件给出**两侧共用**的帧指纹实现 (同源纪律: 只有一份实现),
 *   节点在既有单遍清洗里顺带累加, 工装对同一个 .raw/.npy 按同一清洗规则重算
 *   ⇒ **指纹相同即同帧**, 不需要人工对号。
 *
 * 口径 (必须与驱动清洗一致, core/sensor_astra.cpp inject_depth_frame 的单遍扫描):
 *   lo <= v <= hi 保留原值; 其余一律按 0 计。
 * 算法: FNV-1a 64 位, 逐像素按"低字节在前"两次异或乘法 —— **显式拆字节 ⇒ 与主机端序无关**
 *   (MSVC/Windows 与 gcc/Linux 得到同一指纹; 这点对"两侧在不同机器上算"是硬要求)。
 * 输出: depth_fingerprint_hex = 高 32 位折进低 32 位后的 **8 位小写 hex** (日志里短小可比)。
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>

namespace mechdog {

inline constexpr uint64_t kFingerprintOffsetBasis = 14695981039346656037ULL;
inline constexpr uint64_t kFingerprintPrime       = 1099511628211ULL;

/** 清洗: 量程内保留, 否则 0 (与驱动/工装同一规则) */
inline uint16_t depth_fingerprint_clean(uint16_t v, uint16_t lo, uint16_t hi) {
    return (v >= lo && v <= hi) ? v : static_cast<uint16_t>(0);
}

/** 单步累加 (供节点在单遍扫描里复用; 入参必须是**清洗后**的值) */
inline uint64_t depth_fingerprint_step(uint64_t h, uint16_t cleaned) {
    h ^= static_cast<uint64_t>(cleaned & 0xFFu);
    h *= kFingerprintPrime;
    h ^= static_cast<uint64_t>((cleaned >> 8) & 0xFFu);
    h *= kFingerprintPrime;
    return h;
}

/** 整缓冲指纹 (供工装/测试; 内部按同一规则清洗) */
inline uint64_t depth_fingerprint(const uint16_t* data, std::size_t n, uint16_t lo, uint16_t hi) {
    uint64_t h = kFingerprintOffsetBasis;
    for (std::size_t i = 0; i < n; ++i)
        h = depth_fingerprint_step(h, depth_fingerprint_clean(data[i], lo, hi));
    return h;
}

/** 8 位小写 hex (高 32 位折进低 32 位, 便于日志里短小可比) */
inline std::string depth_fingerprint_hex(uint64_t h) {
    char b[16];
    std::snprintf(b, sizeof(b), "%08x",
                  static_cast<unsigned>((h ^ (h >> 32)) & 0xFFFFFFFFULL));
    return std::string(b);
}

}  // namespace mechdog
