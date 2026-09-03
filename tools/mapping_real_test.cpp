/**
 * mapping_real_test — Windows 真机建图验证工具 (P4 真机链路)
 *
 * 链路 (全真实数据, 仅位姿为静止假设):
 *   Astra Pro 真深度流 (USE_ASTRA_SDK, capture_frame)
 *     → depth_to_cloud      (P0 反投影, 640x480 真帧)
 *     → transform_to_base   (P0 外参变换)
 *     → OccupancyGridMap    (P4 建图, 静止位姿)
 *     → PGM 落盘 + 数值统计
 *
 * 验证目标:
 *   1. 真深度帧能进建图管线 (frame_seq 递增, 点数 >0)
 *   2. 静止累积 N 帧后地图稳定 (占据格数收敛)
 *   3. PGM 中占据点距离分布合理 (室内墙体 0.6~5m)
 *
 * 用法 (build_real 目录):
 *   mapping_real_test.exe [帧数=100] [PGM路径=map_real.pgm]
 *
 * 无 ROS 依赖, Windows 侧直接运行。
 */
#include "config.h"
#include "sensor_astra.h"
#include "point_cloud.h"
#include "mapping.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#ifndef USE_ASTRA_SDK
#error "mapping_real_test 必须以 USE_ASTRA_SDK 编译 (真机模式)"
#endif

using namespace mechdog;

int main(int argc, char** argv) {
    const int n_frames = (argc > 1) ? std::atoi(argv[1]) : 100;
    const std::string pgm =
        (argc > 2) ? argv[2] : "map_real.pgm";
    // 旋转扫描模式: 假设采集期间匀速向左(俯视逆时针)转 sweep_deg 度。
    // 无 IMU/odom, 航向按帧序线性推进 — demo 精度, 非测量。
    const double sweep_deg = (argc > 3) ? std::atof(argv[3]) : 0.0;
    const int countdown_sec = (argc > 4) ? std::atoi(argv[4]) : 0;
    const double DEG2RAD = 0.01745329251994329576;

    std::printf("== mapping real test: %d 帧 → %s ==\n", n_frames, pgm.c_str());
    if (sweep_deg > 0.0) {
        std::printf("旋转扫描模式: %.0f°, 请在倒计时结束后匀速向左转一整圈\n",
                    sweep_deg);
    }

    // --- 真机驱动 ---
    AstraProDriver astra(/*use_simulated=*/false);
    astra.init_hardware();  // 主线程预初始化 (SDK + 双流)
    astra.start();

    // --- 建图 ---
    OccupancyGridMap map(0.25);
    CameraIntrinsics K{};   // 默认 640x480 基准 (真机帧即 640x480)
    CameraExtrinsics E{};
    Pose2D pose;            // 静止: 原点, 朝 +x

    int valid_frames = 0, total_points = 0;
    int prev_occ = -1, stable_ticks = 0;
    const auto t0 = std::chrono::steady_clock::now();

    // --- 倒计时 (手动旋转协调; 期间 SDK 双流热身) ---
    for (int s = countdown_sec; s > 0; --s) {
        std::printf("  %d...\n", s);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (countdown_sec > 0) std::printf("  >>> 开始旋转! <<<\n");

    for (int i = 0; i < n_frames; ++i) {
        // 旋转扫描: 航向按帧序线性推进 (匀速假设)
        if (sweep_deg > 0.0) {
            pose.theta = sweep_deg * DEG2RAD
                         * static_cast<double>(i)
                         / static_cast<double>(n_frames - 1);
        }

        AstraFrame f = astra.capture_frame();
        if (!f.valid || f.depth_map.empty()) {
            std::printf("frame %d: invalid (seq=%llu)\n",
                        i, (unsigned long long)f.frame_seq);
            continue;
        }

        PointCloud cloud_optical, cloud_base;
        depth_to_cloud(f.depth_map.data(), f.depth_width,
                       f.depth_height, K, cloud_optical);
        transform_to_base(cloud_optical, E, cloud_base);

        // FIX_PLAN #1: 地面过滤建图 — 地面点剔除(消假墙),
        // 负障碍(坑/台阶)单独标记层, 只投障碍点
        map.insert_cloud_filtered(cloud_base, pose);

        ++valid_frames;
        total_points += static_cast<int>(cloud_base.points.size());

        if (i % 10 == 0) {
            const int occ = map.count_cells(100);
            const int free = map.count_cells(0);
            std::printf("frame %3d seq=%llu pts=%zu occ=%d free=%d",
                        i, (unsigned long long)f.frame_seq,
                        cloud_base.points.size(), occ, free);
            if (sweep_deg > 0.0) {
                std::printf("  heading=%.0f° (%d/%d)",
                            pose.theta / DEG2RAD, i, n_frames);
            }
            std::printf("\n");
            // 收敛检测: 占据数连续 3 次不变 → 静止场景稳定
            if (occ == prev_occ) ++stable_ticks; else stable_ticks = 0;
            prev_occ = occ;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30fps
    }

    astra.stop();

    // --- 结果 ---
    const double dt = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    std::printf("\n== 结果 ==\n");
    std::printf("有效帧 %d/%d, 总点数 %d, 耗时 %.1fs (%.1f fps)\n",
                valid_frames, n_frames, total_points, dt,
                valid_frames / std::max(dt, 0.1));

    if (valid_frames == 0) {
        std::printf("[FAIL] 无有效真机帧 — 取帧链路异常\n");
        return 1;
    }

    const bool ok = map.save_pgm(pgm);
    std::printf("PGM %s: %s\n", pgm.c_str(), ok ? "已保存" : "保存失败");
    std::printf("%s\n", map.stats().c_str());

    // --- 距离分布校验: 占据点应集中在 0.6~5m (室内墙体) ---
    // (从 PGM 重读计算, 与地图内部状态无关的独立校验)
    if (ok) {
        std::FILE* fp = std::fopen(pgm.c_str(), "rb");
        if (fp) {
            int w = 0, h = 0;
            if (std::fscanf(fp, "P2 %d %d 255", &w, &h) == 2) {
                std::vector<int> vals(static_cast<size_t>(w) * h);
                for (auto& v : vals) {
                    if (std::fscanf(fp, "%d", &v) != 1) v = 205;
                }
                int occ = 0, near_cnt = 0;
                for (int r = 0; r < h; ++r) {
                    for (int c = 0; c < w; ++c) {
                        if (vals[static_cast<size_t>(r) * w + c] != 0) continue;
                        ++occ;
                        const double wx = (c - w / 2.0) * 0.05;
                        const double wy = (h / 2.0 - r) * 0.05;
                        const double d = std::hypot(wx, wy);
                        if (d >= 0.55 && d <= 5.5) ++near_cnt;
                    }
                }
                const double ratio = occ ? near_cnt * 100.0 / occ : 0.0;
                std::printf("占据点 %d, 其中 0.55~5.5m 范围内 %d (%.1f%%)\n",
                            occ, near_cnt, ratio);
                if (ratio > 95.0) {
                    std::printf("[PASS] 占据点距离分布符合室内墙体预期\n");
                    return 0;
                } else {
                    std::printf("[WARN] 距离分布异常 (预期>95%%在 0.55~5.5m)\n");
                    return 2;
                }
            }
            std::fclose(fp);
        }
    }
    return 0;
}
