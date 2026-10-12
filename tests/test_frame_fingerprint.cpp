/**
 * test_frame_fingerprint.cpp — 帧指纹单测 (R-8, 2026-10-12)
 *
 * 测的是**两侧共用契约** (节点单遍累加 vs 工装整缓冲重算), 不是实现细节:
 *   T1 确定性 / hex 形状 (8 位小写 hex)
 *   T2 单像素敏感 (改一个量程内像素 ⇒ 指纹必变)
 *   T3 清洗等价 (量程外值 ≡ 0 ⇒ 同指纹; 边界 lo/hi 必须保留)
 *   T4 单步 == 整体 (depth_fingerprint_step 逐像素 == depth_fingerprint)  ← 跨侧一致的核心保证
 *   T5 顺序敏感 (交换两个不同值 ⇒ 指纹变)
 *   T6 空缓冲不崩 + 宽量程语义
 */
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "frame_fingerprint.h"

using namespace mechdog;

static int g_checks = 0, g_fail = 0;
#define CHECK(cond) do { ++g_checks; if (!(cond)) { ++g_fail; \
    std::cout << "  FAIL " << __func__ << ":" << __LINE__ << "  " << #cond << std::endl; } } while (0)

static const uint16_t kLo = 600, kHi = 8000;

static bool is_lower_hex8(const std::string& s) {
    if (s.size() != 8) return false;
    for (char c : s)
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
    return true;
}

// ---- T1 确定性 + hex 形状 ----
static void test_deterministic_and_hex_shape() {
    std::vector<uint16_t> a{600, 1000, 2500, 8000, 0, 30000, 1234};
    const uint64_t h1 = depth_fingerprint(a.data(), a.size(), kLo, kHi);
    const uint64_t h2 = depth_fingerprint(a.data(), a.size(), kLo, kHi);
    CHECK(h1 == h2);                                  // 同数据同指纹
    CHECK(is_lower_hex8(depth_fingerprint_hex(h1)));  // 8 位小写 hex
    std::vector<uint16_t> b = a;
    std::reverse(b.begin(), b.end());
    CHECK(depth_fingerprint(b.data(), b.size(), kLo, kHi) != h1);   // 与顺序有关
}

// ---- T2 单像素敏感 ----
static void test_single_pixel_change() {
    std::vector<uint16_t> base(64, 1500);
    const uint64_t h0 = depth_fingerprint(base.data(), base.size(), kLo, kHi);
    std::vector<uint16_t> v = base; v[7] = 1501;      // 量程内改 1mm
    CHECK(depth_fingerprint(v.data(), v.size(), kLo, kHi) != h0);
    std::vector<uint16_t> w = base; w[63] = 7999;     // 末像素也能感知
    CHECK(depth_fingerprint(w.data(), w.size(), kLo, kHi) != h0);
}

// ---- T3 清洗等价 (跨侧口径) ----
static void test_cleaning_equivalence() {
    std::vector<uint16_t> with_low(32, 1200), as_zero(32, 1200);
    with_low[5] = 500;      // 低于 lo (600) ⇒ 按 0
    as_zero[5]  = 0;
    CHECK(depth_fingerprint(with_low.data(), with_low.size(), kLo, kHi) ==
          depth_fingerprint(as_zero.data(), as_zero.size(), kLo, kHi));

    // 高值用例必须另起一个干净向量: 复用上一个 as_zero 会把它的 [5]=0 带进来,
    // 于是两向量在 index 5 上就不同了 (本测试自身曾踩过这个坑, 2026-10-12 修正)。
    std::vector<uint16_t> with_high(32, 1200), as_zero_high(32, 1200);
    with_high[9] = 9000;    // 高于 hi (8000) ⇒ 按 0
    as_zero_high[9] = 0;
    CHECK(depth_fingerprint(with_high.data(), with_high.size(), kLo, kHi) ==
          depth_fingerprint(as_zero_high.data(), as_zero_high.size(), kLo, kHi));

    // 边界必须保留: lo 与 hi 本身是有效值, 不能和 0 混同
    std::vector<uint16_t> keep_lo(8, 1200), zero_lo(8, 1200);
    keep_lo[3] = kLo;  zero_lo[3] = 0;
    CHECK(depth_fingerprint(keep_lo.data(), keep_lo.size(), kLo, kHi) !=
          depth_fingerprint(zero_lo.data(), zero_lo.size(), kLo, kHi));
    std::vector<uint16_t> keep_hi(8, 1200), zero_hi(8, 1200);
    keep_hi[4] = kHi;  zero_hi[4] = 0;
    CHECK(depth_fingerprint(keep_hi.data(), keep_hi.size(), kLo, kHi) !=
          depth_fingerprint(zero_hi.data(), zero_hi.size(), kLo, kHi));

    CHECK(depth_fingerprint_clean(599, kLo, kHi) == 0);
    CHECK(depth_fingerprint_clean(600, kLo, kHi) == 600);
    CHECK(depth_fingerprint_clean(8000, kLo, kHi) == 8000);
    CHECK(depth_fingerprint_clean(8001, kLo, kHi) == 0);
    CHECK(depth_fingerprint_clean(0, kLo, kHi) == 0);
}

// ---- T4 单步 == 整体 (节点单遍累加 == 工装整缓冲) ----
static void test_step_matches_whole_buffer() {
    std::vector<uint16_t> d{0, 600, 601, 7999, 8000, 8001, 65535, 1200, 1200, 30000};
    const uint64_t whole = depth_fingerprint(d.data(), d.size(), kLo, kHi);
    uint64_t manual = kFingerprintOffsetBasis;
    for (uint16_t v : d)
        manual = depth_fingerprint_step(manual, depth_fingerprint_clean(v, kLo, kHi));
    CHECK(manual == whole);                       // ← 跨侧一致的核心保证
}

// ---- T5/T6 空缓冲与宽量程 ----
static void test_empty_and_wide_range() {
    const uint64_t h_empty = depth_fingerprint(nullptr, 0, kLo, kHi);
    CHECK(is_lower_hex8(depth_fingerprint_hex(h_empty)));     // 不崩且形状正确
    CHECK(h_empty == kFingerprintOffsetBasis);                // 空缓冲 = 初值

    std::vector<uint16_t> d{0, 500, 600, 8000, 8001, 65535};
    const uint64_t narrow = depth_fingerprint(d.data(), d.size(), kLo, kHi);
    const uint64_t wide   = depth_fingerprint(d.data(), d.size(), 0, 65535);
    CHECK(narrow != wide);                        // 宽量程把量程外值也算进去 ⇒ 必不同
    // 宽量程下 clean 是恒等
    for (uint16_t v : d) CHECK(depth_fingerprint_clean(v, 0, 65535) == v);
}

int main() {
    std::cout << "=== frame fingerprint tests (R-8) ===" << std::endl;
    test_deterministic_and_hex_shape();
    test_single_pixel_change();
    test_cleaning_equivalence();
    test_step_matches_whole_buffer();
    test_empty_and_wide_range();
    std::cout << "=== " << g_checks << " checks, " << g_fail << " failed ===" << std::endl;
    return g_fail == 0 ? 0 : 1;
}
