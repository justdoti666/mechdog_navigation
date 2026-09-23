/**
 * 点云模块单元测试 (P0)
 *
 * 设计文档: docs/POINT_CLOUD_DESIGN.md §13 验证方案
 *
 * 覆盖 P0 场景:
 *   1. 反投影几何 — 全像素同距 → 垂直光轴平面, Z≈2.0
 *   2. 内参正确性 — 中心像素 → X≈0, Y≈0, Z≈2.0
 *   3. optical→link 旋转 — 已知点逐轴验证 (§6.1 矩阵)
 *   4. transform_to_base 地面点 — 相机高 0.18m → base 系 z≈0
 *   5. pitch 方向 — pitch=+15° → 前向轴 z 分量 < 0 (光轴朝下)
 *   6. 失效哨兵 — 全 0 深度图 → 空点云, 不崩溃
 *   7. 空指针防御 — nullptr → 空点云, 不崩溃
 *   8. 深度范围过滤 — d<min / d>max → 丢弃
 *
 * 构建/运行:
 *   g++ -std=c++20 -fsanitize=address,undefined \
 *     tests/test_point_cloud.cpp point_cloud.cpp -o test_point_cloud -lpthread
 *   ./test_point_cloud
 *
 * 或通过 CMake:
 *   cmake -B build && cmake --build build && ctest -R point_cloud
 */
#include "../point_cloud.h"

#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <vector>

using namespace mechdog;

static int g_passed = 0;
static int g_failed = 0;

// 与 test_fusion.cpp 同款轻量断言 (无第三方框架)
#define CHECK(cond) do { \
    if (cond) { ++g_passed; } \
    else { ++g_failed; std::cerr << "FAIL: " << #cond << " @ " << __LINE__ << std::endl; } \
} while (0)

// 浮点近似比较
static bool approx(double a, double b, double tol = 1e-6) {
    return std::abs(a - b) < tol;
}

// ============================================================
// 测试辅助: 构造测试深度图 (mm)
// ============================================================
static std::vector<uint16_t> make_test_depth(int w, int h,
                                             const std::string& scenario) {
    std::vector<uint16_t> depth(static_cast<size_t>(w) * h, 0);  // 默认全 0 (无效)

    if (scenario == "flat_wall_2m") {
        // 全像素 2.0m → 垂直光轴的平面
        std::fill(depth.begin(), depth.end(), 2000);

    } else if (scenario == "center_pixel_2m") {
        // 仅中心像素 (320,240) = 2.0m
        depth[240 * w + 320] = 2000;

    } else if (scenario == "ground_point") {
        // 地面点: 相机高 0.18m, pitch=0, 前方 2.0m 处地面
        // optical 系: X_opt=0, Y_opt=+0.18 (下为正), Z_opt=2.0
        // 像素: u = cx + X_opt*fx/Z_opt = 320
        //        v = cy + Y_opt*fy/Z_opt = 240 + 0.18*572.3/2.0 ≈ 291.5 → 292
        depth[292 * w + 320] = 2000;

    } else if (scenario == "near_pixel") {
        // d=0.5m (低于 min_depth_m=0.6) → 应被过滤
        depth[100 * w + 100] = 500;

    } else if (scenario == "far_pixel") {
        // d=9.0m (高于 max_depth_m=8.0) → 应被过滤
        depth[100 * w + 100] = 9000;

    } else if (scenario == "all_zero") {
        // 全 0 (无效), 不填任何值

    } else if (scenario == "mixed_valid_invalid") {
        // 中心有效 (2.0m) + 邻近无效 (0)
        depth[240 * w + 320] = 2000;
        depth[240 * w + 321] = 0;       // 无效
        depth[241 * w + 320] = 0;       // 无效
        depth[240 * w + 319] = 2000;    // 有效
    }
    // scenario == "all_zero" 或未知: 全 0

    return depth;
}

// ============================================================
// 测试 1: 反投影几何 — 全像素同距 → 垂直光轴平面
// §13: 全像素同距 d=2.0m → 点云为垂直于光轴的平面, 所有点 Z≈2.0m
// (Astra 给的是沿光轴深度 Z, 非径向距; 边缘像素径向距 > 2.0)
// ============================================================
static void test_backprojection_plane() {
    const int W = 640, H = 480;
    auto depth = make_test_depth(W, H, "flat_wall_2m");
    CameraIntrinsics K;
    PointCloud cloud;

    depth_to_cloud(depth.data(), W, H, K, cloud);

    CHECK(cloud.frame_id == "camera_optical");
    CHECK(!cloud.points.empty());

    // 所有点 Z ≈ 2.0m (垂直光轴平面)
    for (const auto& p : cloud.points) {
        CHECK(approx(p.z, 2.0, 1e-3));
    }

    // 边缘像素径向距 > 2.0 (不是球面, 是平面)
    // 角落像素 (0,0): X=(0-320)*2/572.7≈-1.118, Y=(0-240)*2/572.3≈-0.839
    // 径向距 = sqrt(1.118²+0.839²+2.0²) ≈ 2.476 > 2.0
    bool found_corner = false;
    for (const auto& p : cloud.points) {
        if (p.x < -1.0 && p.y < -0.7) {  // 左上角附近
            double radial = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
            CHECK(radial > 2.0);  // 径向距 > 深度 (平面特征)
            found_corner = true;
            break;
        }
    }
    CHECK(found_corner);
}

// ============================================================
// 测试 2: 内参正确性 — 中心像素 → X≈0, Y≈0, Z≈2.0
// §13: 中心像素 (320,240)、d=2m → X≈0, Y≈0, Z≈2.0
// ============================================================
static void test_intrinsics_center_pixel() {
    const int W = 640, H = 480;
    auto depth = make_test_depth(W, H, "center_pixel_2m");
    CameraIntrinsics K;
    PointCloud cloud;

    depth_to_cloud(depth.data(), W, H, K, cloud);

    CHECK(cloud.points.size() == 1);  // 仅中心像素有效

    const auto& p = cloud.points[0];
    CHECK(approx(p.x, 0.0));  // u=cx → X=0
    CHECK(approx(p.y, 0.0));  // v=cy → Y=0
    CHECK(approx(p.z, 2.0));  // Z=d=2.0
}

// ============================================================
// 测试 3: optical→link 旋转 — 已知点逐轴验证 (§6.1 矩阵)
// 构造 optical 系点 (X=0, Y=+0.18, Z=2.0) →
//   X_link =  Z_opt  =  2.0  (前)
//   Y_link = -X_opt  =  0.0  (左)
//   Z_link = -Y_opt  = -0.18 (上)
// ============================================================
static void test_optical_to_link_rotation() {
    PointCloud in;
    in.frame_id = "camera_optical";
    in.points.push_back({0.0, 0.18, 2.0, 0, 0, 0});   // 地面点 (前方2m, 下0.18m)
    in.points.push_back({1.0, 0.0, 0.0, 0, 0, 0});     // 右侧点 (X_opt=1)
    in.points.push_back({0.0, -1.0, 0.0, 0, 0, 0});    // 上方点 (Y_opt=-1)

    PointCloud out;
    transform_optical_to_link(in, out);

    CHECK(out.frame_id == "camera_link");
    CHECK(out.points.size() == 3);

    // 点 0: (0, 0.18, 2.0) → (2.0, 0, -0.18)
    CHECK(approx(out.points[0].x, 2.0));    //  X_link =  Z_opt
    CHECK(approx(out.points[0].y, 0.0));    //  Y_link = -X_opt
    CHECK(approx(out.points[0].z, -0.18));  //  Z_link = -Y_opt

    // 点 1: (1, 0, 0) → (0, -1, 0)  [右侧 → 后方? 不, X_opt=1(右) → Y_link=-1(右→左反)]
    //  X_link = Z_opt = 0, Y_link = -X_opt = -1, Z_link = -Y_opt = 0
    CHECK(approx(out.points[1].x, 0.0));
    CHECK(approx(out.points[1].y, -1.0));
    CHECK(approx(out.points[1].z, 0.0));

    // 点 2: (0, -1, 0) → (0, 0, 1)  [上方 → Z_link=1 (上)]
    //  X_link = Z_opt = 0, Y_link = -X_opt = 0, Z_link = -Y_opt = 1
    CHECK(approx(out.points[2].x, 0.0));
    CHECK(approx(out.points[2].y, 0.0));
    CHECK(approx(out.points[2].z, 1.0));

    // §6.1 单测固化: 正前方墙质心应落到 X_link > 0, Y_link ≈ 0, Z_link ≈ 0
    PointCloud wall;
    wall.points.push_back({0.0, 0.0, 3.0, 0, 0, 0});  // 正前方 3m
    PointCloud wall_link;
    transform_optical_to_link(wall, wall_link);
    CHECK(wall_link.points[0].x > 0.0);  // 前
    CHECK(approx(wall_link.points[0].y, 0.0));
    CHECK(approx(wall_link.points[0].z, 0.0));
}

// ============================================================
// 测试 4: transform_to_base 地面点 — 相机高 0.18m → base 系 z≈0
// §13 坐标变换: 相机离地 0.18m、俯角 0
//   地面点在相机下方 → optical 系 Y≈+0.18
//   经 §6.1 转 link: Z_link = -Y_opt ≈ -0.18
//   叠加外参 z=0.18 → base 系 z ≈ 0
// ============================================================
static void test_transform_to_base_ground() {
    const int W = 640, H = 480;
    auto depth = make_test_depth(W, H, "ground_point");
    CameraIntrinsics K;
    CameraExtrinsics E;
    // 临时摆放: 无偏移无俯角 (§18.4 联调方法)
    E.x = 0.0; E.y = 0.0; E.z = 0.18;
    E.roll = 0.0; E.pitch = 0.0; E.yaw = 0.0;

    PointCloud cloud_optical;
    depth_to_cloud(depth.data(), W, H, K, cloud_optical);

    // 验证反投影: 地面点在 optical 系 Y ≈ +0.18 (下为正)
    CHECK(!cloud_optical.points.empty());
    const auto& opt = cloud_optical.points[0];
    CHECK(approx(opt.x, 0.0, 0.01));    // 正前方, 无左右偏移
    CHECK(opt.y > 0.15 && opt.y < 0.21);  // Y ≈ +0.18 (下为正, 地面在下)
    CHECK(approx(opt.z, 2.0, 0.01));    // Z = 2.0m

    // 转 base
    PointCloud cloud_base;
    transform_to_base(cloud_optical, E, cloud_base);

    CHECK(cloud_base.frame_id == "base_link");
    CHECK(!cloud_base.points.empty());

    const auto& base = cloud_base.points[0];
    // 像素量化误差: v=292 而非精确 291.5, Y_opt 略偏, z_base 约有 ~2mm 误差
    CHECK(approx(base.x, 2.0, 0.01));   // 前方 2m
    CHECK(approx(base.y, 0.0, 0.01));   // 无左右偏移
    CHECK(approx(base.z, 0.0, 0.01));   // 地面 z≈0 (容差 1cm, 像素量化)
}

// ============================================================
// 测试 5: pitch 方向 — pitch=+15° → 前向轴 z 分量 < 0 (光轴朝下)
// §4: pitch > 0 表示光轴朝下 (ZYX 内旋, R20 = -sin(pitch))
// ============================================================
static void test_pitch_direction() {
    // 构造 link 系前向点 (1,0,0) — 相机正前方 1m
    PointCloud link_cloud;
    link_cloud.frame_id = "camera_link";
    link_cloud.points.push_back({1.0, 0.0, 0.0, 0, 0, 0});

    // 先转成 optical (逆 §6.1: X_opt=-Y_link, Y_opt=-Z_link, Z_opt=X_link)
    PointCloud opt_cloud;
    opt_cloud.frame_id = "camera_optical";
    for (const auto& lp : link_cloud.points) {
        opt_cloud.points.push_back({-lp.y, -lp.z, lp.x, lp.r, lp.g, lp.b});
    }

    // 外参: pitch=+15° (光轴朝下), 无平移
    CameraExtrinsics E;
    E.x = 0; E.y = 0; E.z = 0;
    E.roll = 0; E.yaw = 0;
    E.pitch = 15.0 * 0.01745329251994329576;  // +15°

    PointCloud base_cloud;
    transform_to_base(opt_cloud, E, base_cloud);

    // 前向轴 [1,0,0] 经 R 后 z 分量 = R20 = -sin(15°) ≈ -0.2588
    const auto& p = base_cloud.points[0];
    CHECK(p.z < 0.0);  // 光轴朝下 → z < 0
    CHECK(approx(p.z, -std::sin(15.0 * 0.01745329251994329576), 1e-6));  // ≈ -0.2588

    // 对照: pitch=0 → z = 0 (水平)
    E.pitch = 0.0;
    transform_to_base(opt_cloud, E, base_cloud);
    CHECK(approx(base_cloud.points[0].z, 0.0, 1e-6));

    // 对照: pitch=-15° → z > 0 (光轴朝上)
    E.pitch = -15.0 * 0.01745329251994329576;
    transform_to_base(opt_cloud, E, base_cloud);
    CHECK(base_cloud.points[0].z > 0.0);  // 朝上 → z > 0
}

// ============================================================
// 测试 6: 失效哨兵 — 全 0 深度图 → 空点云, 不崩溃
// §13: 与现有 valid_pixel_ratio==0 口径一致
// ============================================================
static void test_all_zero_empty_cloud() {
    const int W = 640, H = 480;
    auto depth = make_test_depth(W, H, "all_zero");
    CameraIntrinsics K;
    PointCloud cloud;

    depth_to_cloud(depth.data(), W, H, K, cloud);

    CHECK(cloud.points.empty());
    CHECK(cloud.frame_id == "camera_optical");
}

// ============================================================
// 测试 7: 空指针防御 — nullptr → 空点云, 不崩溃
// ============================================================
static void test_null_depth_safe() {
    CameraIntrinsics K;
    PointCloud cloud;

    depth_to_cloud(nullptr, 640, 480, K, cloud);
    CHECK(cloud.points.empty());

    // 零尺寸也安全
    std::vector<uint16_t> depth(100, 2000);
    depth_to_cloud(depth.data(), 0, 480, K, cloud);
    CHECK(cloud.points.empty());

    depth_to_cloud(depth.data(), 640, 0, K, cloud);
    CHECK(cloud.points.empty());

    depth_to_cloud(depth.data(), -1, 480, K, cloud);
    CHECK(cloud.points.empty());

    // FIX-06: 内参 fx/fy = 0 (或负) 必须运行期安全早退 ——
    // 旧实现只有 assert, Release/NDEBUG 下被编译掉 → 除零产生 inf/nan 点。
    CameraIntrinsics bad{};
    bad.fx = 0.0;
    depth_to_cloud(depth.data(), 640, 480, bad, cloud);
    CHECK(cloud.points.empty());

    bad = CameraIntrinsics{};
    bad.fy = 0.0;
    depth_to_cloud(depth.data(), 640, 480, bad, cloud);
    CHECK(cloud.points.empty());

    bad = CameraIntrinsics{};
    bad.fx = -1.0;
    depth_to_cloud(depth.data(), 640, 480, bad, cloud);
    CHECK(cloud.points.empty());
}

// ============================================================
// 测试 8: 深度范围过滤 — d<min / d>max → 丢弃
// 与 analyze_region 同口径: [600, 8000] mm
// ============================================================
static void test_depth_range_filter() {
    const int W = 640, H = 480;
    CameraIntrinsics K;
    PointCloud cloud;

    // d=0.5m (500mm < 600mm min) → 过滤
    auto depth_near = make_test_depth(W, H, "near_pixel");
    depth_to_cloud(depth_near.data(), W, H, K, cloud);
    CHECK(cloud.points.empty());  // 全部被过滤

    // d=9.0m (9000mm > 8000mm max) → 过滤
    auto depth_far = make_test_depth(W, H, "far_pixel");
    depth_to_cloud(depth_far.data(), W, H, K, cloud);
    CHECK(cloud.points.empty());

    // 边界值: d=0.6m (600mm) 和 d=8.0m (8000mm) 应保留
    std::vector<uint16_t> depth_boundary(static_cast<size_t>(W) * H, 0);
    depth_boundary[100 * W + 100] = 600;   // 恰好 min
    depth_boundary[200 * W + 200] = 8000;  // 恰好 max
    depth_to_cloud(depth_boundary.data(), W, H, K, cloud);
    CHECK(cloud.points.size() == 2);  // 两个边界值均有效
    CHECK(approx(cloud.points[0].z, 0.6, 1e-6));
    CHECK(approx(cloud.points[1].z, 8.0, 1e-6));
}

// ============================================================
// 测试 9: 混合有效/无效像素 — 仅保留有效像素
// ============================================================
static void test_mixed_valid_invalid() {
    const int W = 640, H = 480;
    auto depth = make_test_depth(W, H, "mixed_valid_invalid");
    CameraIntrinsics K;
    PointCloud cloud;

    depth_to_cloud(depth.data(), W, H, K, cloud);

    // 2 个有效 (2000mm) + 2 个无效 (0) → 仅 2 个点
    CHECK(cloud.points.size() == 2);
    for (const auto& p : cloud.points) {
        CHECK(approx(p.z, 2.0, 1e-3));
    }
}

// ============================================================
// 测试 10: 空点云变换安全
// ============================================================
static void test_empty_cloud_transform() {
    PointCloud empty;
    empty.frame_id = "camera_optical";

    PointCloud out_link;
    transform_optical_to_link(empty, out_link);
    CHECK(out_link.points.empty());
    CHECK(out_link.frame_id == "camera_link");

    CameraExtrinsics E;
    PointCloud out_base;
    transform_to_base(empty, E, out_base);
    CHECK(out_base.points.empty());
    CHECK(out_base.frame_id == "base_link");
}

// ============================================================
// 测试 11: RGB 颜色保留
// ============================================================
static void test_color_preserved() {
    PointCloud in;
    in.frame_id = "camera_optical";
    in.points.push_back({1.0, 2.0, 3.0, 255, 128, 64});

    PointCloud out_link;
    transform_optical_to_link(in, out_link);
    CHECK(out_link.points[0].r == 255);
    CHECK(out_link.points[0].g == 128);
    CHECK(out_link.points[0].b == 64);

    CameraExtrinsics E;
    PointCloud out_base;
    transform_to_base(in, E, out_base);
    CHECK(out_base.points[0].r == 255);
    CHECK(out_base.points[0].g == 128);
    CHECK(out_base.points[0].b == 64);
}

// ============================================================
// 测试 12: seq/stamp 传递
// ============================================================
static void test_metadata_preserved() {
    PointCloud in;
    in.seq = 42;
    in.stamp = 12345.678;
    in.frame_id = "camera_optical";
    in.points.push_back({0, 0, 1, 0, 0, 0});

    PointCloud out_base;
    CameraExtrinsics E;
    transform_to_base(in, E, out_base);

    CHECK(out_base.seq == 42);
    CHECK(approx(out_base.stamp, 12345.678));
}

// ============================================================
// 测试 13: count_valid_pixels 全帧有效像素统计 (TDD, 空态诊断数据源)
// ============================================================
static void test_count_valid_pixels() {
    // 全 0 (无效) → 0
    std::vector<uint16_t> zero(64, 0);
    CHECK(count_valid_pixels(zero) == 0);

    // 边界/混合: 有效 = 600, 8000, 7999, 601, 1000 → 5 个
    // (599 近界外, 8001 远界外, 0 无效)
    std::vector<uint16_t> mixed = {0, 600, 8000, 7999, 601, 599, 8001, 1000};
    CHECK(count_valid_pixels(mixed) == 5);

    // 空向量 → 0
    std::vector<uint16_t> empty;
    CHECK(count_valid_pixels(empty) == 0);
}

// ============================================================
// 测试 14: cloud_state_label 三态标签 (TDD, 状态栏空态诊断)
// ============================================================
static void test_cloud_state_label() {
    const char* l0 = cloud_state_label(0);
    const char* l1 = cloud_state_label(1);
    const char* l2 = cloud_state_label(2);
    CHECK(l0 != nullptr && l0[0] != '\0');
    CHECK(l1 != nullptr && l1[0] != '\0');
    CHECK(l2 != nullptr && l2[0] != '\0');
    CHECK(std::strcmp(l0, l1) != 0);   // 三态标签互不相同
    CHECK(std::strcmp(l1, l2) != 0);
    CHECK(std::strcmp(l0, l2) != 0);
}

// ============================================================
// 测试 15: screen_decimate 屏幕空间抽稀 (TDD, 绘制量降 4-8x)
// ============================================================
static void test_screen_decimate() {
    // 同格只保留 1 点: 3 点 y 差 0.001m, px_per_m=43 → 像素差 0.043px, 同一 2px 格
    {
        PointCloud in;
        for (double dy : {0.0, 0.001, 0.002}) {
            Point3D p; p.x = 1.0; p.y = 0.5 + dy; p.z = -0.8;
            in.points.push_back(p);
        }
        PointCloud out;
        screen_decimate(in, 43.0, out);
        CHECK(out.points.size() == 1);
        CHECK(out.seq == in.seq && out.frame_id == in.frame_id);  // 元数据保留
    }
    // 不同格保留: y 差 0.5m → 21.5px, 不同格
    {
        PointCloud in;
        Point3D a; a.x = 1.0; a.y = 0.0; a.z = -0.8; in.points.push_back(a);
        Point3D b; b.x = 1.0; b.y = 0.5; b.z = -0.8; in.points.push_back(b);
        PointCloud out;
        screen_decimate(in, 43.0, out);
        CHECK(out.points.size() == 2);
    }
    // 空输入
    {
        PointCloud in, out;
        screen_decimate(in, 43.0, out);
        CHECK(out.points.empty());
    }
}

// ============================================================
// FIX-10 回归: transform_to_base 的输入必须是 camera_optical 系
// (函数内部先做 optical→link, 见 point_cloud.h 契约注释)。
// 误传 camera_link 系的云 = 二次旋转 90°, 前方距离跑到 Y 轴负向 ——
// main.cpp 曾犯此错, 导致建图格栅/地面分割/2.5D 全部工作在错帧。
// ============================================================
static void test_transform_to_base_frame_contract() {
    CameraExtrinsics E0;   // 外参归零, 只看旋转链
    E0.x = 0.0; E0.y = 0.0; E0.z = 0.0; E0.roll = 0.0; E0.pitch = 0.0; E0.yaw = 0.0;

    // 正确: 传 optical 系 (前方 2.5m / 左侧 1m) → base 得到 x=2.5(前), y=+1(左)
    PointCloud opt; opt.frame_id = "camera_optical";
    Point3D q; q.x = -1.0; q.y = 0.0; q.z = 2.5;
    opt.points.push_back(q);
    PointCloud out;
    transform_to_base(opt, E0, out);
    CHECK(std::fabs(out.points[0].x - 2.5) < 1e-9);
    CHECK(std::fabs(out.points[0].y - 1.0) < 1e-9);
    CHECK(std::fabs(out.points[0].z - 0.0) < 1e-9);

    // 误用: 把 link 系 (X前=2.5, Y左=1) 当 optical 传 → 前方落到 Y 负向
    PointCloud link; link.frame_id = "camera_link";
    Point3D p; p.x = 2.5; p.y = 1.0; p.z = 0.0;
    link.points.push_back(p);
    transform_to_base(link, E0, out);
    CHECK(std::fabs(out.points[0].x) < 1e-9);          // 不再落在 X
    CHECK(std::fabs(out.points[0].y + 2.5) < 1e-9);    // 跑到 y=-2.5
    CHECK(std::fabs(out.points[0].z + 1.0) < 1e-9);    // z 也被翻

    // 两个函数的组合关系: link == optical→link 的结果
    PointCloud viaFn;
    transform_to_base(opt, E0, out);
    transform_optical_to_link(opt, viaFn);
    CHECK(std::fabs(out.points[0].x - viaFn.points[0].x) < 1e-9);
    CHECK(std::fabs(out.points[0].y - viaFn.points[0].y) < 1e-9);
    CHECK(std::fabs(out.points[0].z - viaFn.points[0].z) < 1e-9);
}

// ============================================================
// 用法A 单元: 重力对齐 align_to_gravity
// 约定: pitch>0 = 机头抬起; 补偿做的是 p_out = Rx(-roll)·Ry(-pitch)·p_in
// ============================================================
static void test_align_to_gravity_unit() {
    const double D2R = 0.01745329251994329576;

    PointCloud base;
    Point3D p; p.x = 1.0; p.y = 0.5; p.z = -0.18; base.points.push_back(p);
    AttitudeSample att;              // 默认 valid=false
    AttitudeGate   gate;

    // ① 无姿态 → 点云逐位不动 (A1 回归: 不接 IMU = 现行为)
    PointCloud c = base;
    align_to_gravity(c, att, 100.0, 0.30, 25.0, gate);
    CHECK(!gate.used);
    CHECK(std::string(gate.reason) == "invalid");
    CHECK(c.points[0].x == 1.0 && c.points[0].y == 0.5 && c.points[0].z == -0.18);

    // ② 过期 (age 1.0s > 0.30s) → 不补偿
    att.valid = true; att.pitch_deg = 10.0; att.stamp_s = 99.0;
    c = base;
    align_to_gravity(c, att, 100.0, 0.30, 25.0, gate);
    CHECK(!gate.used);
    CHECK(std::string(gate.reason) == "stale");
    CHECK(c.points[0].x == 1.0);

    // ③ 超门控 (|pitch|30° > 25°) → 不补偿
    att.stamp_s = 100.0; att.pitch_deg = 30.0;
    c = base;
    align_to_gravity(c, att, 100.0, 0.30, 25.0, gate);
    CHECK(!gate.used);
    CHECK(std::string(gate.reason) == "gate");

    // ④ 符号/轴向 (A7): 机体上方向量 (0,0,1) + 抬头 10° → 重力系 (-sin10°, 0, cos10°)
    PointCloud up;
    Point3D q; q.x = 0.0; q.y = 0.0; q.z = 1.0; up.points.push_back(q);
    att.pitch_deg = 10.0; att.roll_deg = 0.0;
    align_to_gravity(up, att, 100.0, 0.30, 25.0, gate);
    CHECK(gate.used);
    CHECK(std::string(gate.reason) == "ok");
    CHECK(std::fabs(up.points[0].x + std::sin(10.0 * D2R)) < 1e-9);
    CHECK(std::fabs(up.points[0].z - std::cos(10.0 * D2R)) < 1e-9);

    // ⑤ 单位性: 平地云 + 姿态 → 补偿是刚体变换 (点数/两两距离不变)
    PointCloud flat;
    for (double x = 0.5; x <= 2.0; x += 0.1) {
        Point3D r; r.x = x; r.y = 0.0; r.z = -0.18; flat.points.push_back(r);
    }
    const size_t n0 = flat.points.size();
    const double d0 = std::hypot(flat.points[0].x - flat.points[n0 - 1].x,
                                 flat.points[0].z - flat.points[n0 - 1].z);
    att.pitch_deg = 12.0;
    align_to_gravity(flat, att, 100.0, 0.30, 25.0, gate);
    const double d1 = std::hypot(flat.points[0].x - flat.points[n0 - 1].x,
                                 flat.points[0].z - flat.points[n0 - 1].z);
    CHECK(flat.points.size() == n0);
    CHECK(std::fabs(d1 - d0) < 1e-9);       // 刚体: 距离保持
}


// ============================================================
// 9. v2.9.9 带步长反投影 (提速) 的等价性保障
//    性质: strided 点集的每个点, 都与 depth_to_cloud 在同像素算出的点**逐位相同**(子集关系)
// ============================================================
static void test_strided_backprojection_subset() {
    const int w = 64, h = 48;
    std::vector<uint16_t> d(static_cast<size_t>(w) * h, 0);
    for (int v = 0; v < h; ++v) {
        for (int u = 0; u < w; ++u) {
            if (((u * 3 + v * 5) % 7) == 0) continue;       // 造孔洞(无效像素)
            d[static_cast<size_t>(v) * w + u] = static_cast<uint16_t>(800 + u * 5 + v * 3);
        }
    }
    CameraIntrinsics K;
    K.fx = 570.34; K.fy = 570.34;
    K.cx = (w - 1) * 0.5; K.cy = (h - 1) * 0.5;
    K.min_depth_m = 0.3; K.max_depth_m = 8.0;

    PointCloud full, st;
    depth_to_cloud(d.data(), w, h, K, full);
    depth_to_cloud_strided(d.data(), w, h, K, 8, st);
    CHECK(!full.points.empty());
    CHECK(!st.points.empty());

    bool all_in = true;                                     // ① 逐位子集
    for (size_t i = 0; i < st.points.size(); ++i) {
        bool found = false;
        for (size_t j = 0; j < full.points.size(); ++j) {
            const Point3D& p = st.points[i];
            const Point3D& q = full.points[j];
            if (p.x == q.x && p.y == q.y && p.z == q.z) { found = true; break; }
        }
        if (!found) { all_in = false; break; }
    }
    CHECK(all_in);
    CHECK(st.points.size() > full.points.size() / 12);       // ② 密度约 1/8
    CHECK(st.points.size() < full.points.size() / 4);

    PointCloud one;                                          // ③ step<=1 退化为完全一致
    depth_to_cloud_strided(d.data(), w, h, K, 1, one);
    CHECK(one.points.size() == full.points.size());
    CHECK(one.points[0].x == full.points[0].x);
    CHECK(one.points[one.points.size() - 1].z == full.points[full.points.size() - 1].z);
}

int main() {
    test_backprojection_plane();
    test_intrinsics_center_pixel();
    test_optical_to_link_rotation();
    test_transform_to_base_ground();
    test_pitch_direction();
    test_all_zero_empty_cloud();
    test_null_depth_safe();
    test_depth_range_filter();
    test_mixed_valid_invalid();
    test_empty_cloud_transform();
    test_color_preserved();
    test_metadata_preserved();
    test_count_valid_pixels();
    test_cloud_state_label();
    test_screen_decimate();
    test_transform_to_base_frame_contract();   // FIX-10
    test_align_to_gravity_unit();              // 用法A: 重力对齐单元测试

    test_strided_backprojection_subset();      // v2.9.9 带步长反投影(提速)等价性

    std::cout << "passed=" << g_passed << " failed=" << g_failed << std::endl;
    return g_failed == 0 ? 0 : 1;
}
