/**
 * mechdog_navigation 主程序 - 带实时可视化窗口 (Windows GDI+)
 *
 * 默认以模拟模式运行: 无需硬件, 在 PC 上即可验证传感器融合与导航决策全链路。
 * 真机模式: 通过 CMake 选项 USE_WIRINGPI / USE_ASTRA_SDK 编译, 并修改下方 use_simulated 参数。
 * --ground <h>: 手持实验 —— 相机离地 h 米 (默认 0.8), 外参归零使 base==link 显示同系,
 *               地面先验 = -h、窗口收紧 0.12; 装机后勿用 (用默认外参+紧窗口)。
 * --free: 等价 --ground 0.8 (兼容已发布用法的别名)。
 *
 * 构建 (Windows, 需要 GDI+):
 *   cmake -B build_vs -DCMAKE_CXX_STANDARD=17
 *   cmake --build build_vs --config Release
 *   .\build_vs\Release\mechdog_navigation.exe
 *
 * 可视化: 弹出窗口显示俯视图 (机器人 + 四周障碍物, 颜色=危险等级) + 状态栏。
 *   - 关闭窗口 或 Ctrl+C 退出
 */
#include "sensor_astra.h"
#include "sensor_ultrasonic.h"
#include "sensor_ir.h"
#include "sensor_fusion.h"
#include "path_planner.h"
#include "point_cloud.h"
#include "ground_segmentation.h"

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <atomic>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>
#include <map>

#ifdef _WIN32
#include <windows.h>
#include <gdiplus.h>
#pragma comment(lib, "gdiplus.lib")
#endif

using namespace mechdog;

// 跨线程退出标志: 信号处理器/窗口线程写, 主循环读 (Low 清理: 原 sig_atomic_t
// 仅保证对信号处理器的原子性, 跨线程共享需 atomic; int 为 lock-free, 信号处理器可用)
static std::atomic<int> g_stop{0};
void on_signal(int) { g_stop.store(1); }

#ifdef _WIN32

// ---------------- GDI+ 可视化 ----------------
static Gdiplus::GdiplusStartupInput g_gdiplus_startup;
static ULONG_PTR g_gdiplus_token = 0;
static volatile std::sig_atomic_t g_window_open = 1;
static HWND g_hwnd = nullptr;

// 共享最新融合结果 (由主循环写入, 由窗口线程读取)
static FusionResult g_latest_result;
static bool g_have_result = false;

// 共享最新彩色帧 (真机 RGB 可视化; 由主循环写入, 窗口线程读取)
static ColorFrameData g_color_frame;
static bool g_have_color = false;

// 共享最新点云 (由主循环写入, 窗口线程读取; --cloud 模式启用)
static PointCloud g_latest_cloud;
static bool g_have_cloud = false;
// 点云可视化模式开关 (--cloud; 窗口线程读, true 时优先渲染点云而非彩色帧)
static bool g_view_cloud = false;
// 点云视图类型 (false=俯视图 TopView, true=侧视图 SideView; 按 S 切换)
static bool g_view_side = false;
// 共享负障碍点云 (P1, base_link 系地面平面高度处) + 平面锁定状态 (主循环写, 窗口线程读)
static PointCloud g_neg_cloud;
static bool g_plane_valid = false;
// 诊断开关: 水平镜像深度图 (Astra 深度与 RGB 的左右关系存疑, 按 M 实测裁决)
static std::atomic<bool> g_flip_depth{false};
// 诊断开关: 深度热力图叠加 RGB (近=红 远=蓝; 对齐则热力落在实物上, 镜像则左右互换)
static std::atomic<bool> g_heat_overlay{false};
// 点云空态诊断: 0=无帧(取帧失败, 旧点云保留) 1=有帧但有效像素=0(近界/无回波) 2=正常
static std::atomic<int>    g_cloud_state{2};
static std::atomic<size_t> g_cloud_valid_px{0};
static std::atomic<int>    g_frame_w{0}, g_frame_h{0};   // 最近一帧深度分辨率 (诊断打印)
// 帧率诊断: 主循环 tick 更新率 (EMA Hz) + 窗口一帧绘制耗时 (EMA ms), 状态栏显示
static std::atomic<double> g_tick_hz{0.0};
static std::atomic<double> g_draw_ms{0.0};
// 窗口节流: 点云 seq 未变且距上次绘制 <80ms → 跳过陈旧重绘 (10Hz 定时 vs 数据更新率空转抑制)
static uint64_t g_last_drawn_seq = 0;
static std::chrono::steady_clock::time_point g_last_draw_t{};
static std::vector<uint8_t> g_heat_bgr;   // BGR 混合图 (主循环写, 窗口线程读, g_viz_mutex)
static int g_heat_w = 0, g_heat_h = 0;
static bool g_heat_valid = false;

// 可视化共享数据互斥锁 (主循环写 / 窗口线程读, 防数据竞争崩溃)
static std::mutex g_viz_mutex;

// 窗口线程独立刷新用: 指向 astra 驱动 (真机模式下窗口线程直接读彩色帧, 画面不依赖主循环频率)
static AstraProDriver* g_astra_ptr = nullptr;

// 方向 -> 俯视图角度 (度, 0=正前, 顺时针为正; 右=+90, 左=-90)
static std::map<std::string, double> g_dir_angle = {
    {"front_center", 0.0}, {"front_left", -30.0}, {"front_right", 30.0},
    {"left", -90.0}, {"right", 90.0},
    {"bottom", -90.0}, // 底部悬崖, 画在正下
};

static COLORREF level_color(ObstacleLevel lvl) {
    switch (lvl) {
        case ObstacleLevel::SAFE:    return RGB(0, 200, 0);
        case ObstacleLevel::WARNING: return RGB(220, 220, 0);
        case ObstacleLevel::DANGER:  return RGB(255, 140, 0);
        case ObstacleLevel::CRITICAL:return RGB(230, 30, 30);
    }
    return RGB(200, 200, 200);
}

static const char* level_name(ObstacleLevel lvl) {
    switch (lvl) {
        case ObstacleLevel::SAFE:    return "SAFE";
        case ObstacleLevel::WARNING: return "WARN";
        case ObstacleLevel::DANGER:  return "DANGER";
        case ObstacleLevel::CRITICAL:return "CRITICAL";
    }
    return "?";
}

static const char* action_name(NavigationAction a) {
    switch (a) {
        case NavigationAction::STOP:         return "STOP";
        case NavigationAction::BACKWARD:     return "BACKWARD";
        case NavigationAction::TURN_LEFT:    return "TURN_LEFT";
        case NavigationAction::TURN_RIGHT:   return "TURN_RIGHT";
        case NavigationAction::SLOW_FORWARD: return "SLOW_FWD";
        case NavigationAction::FORWARD:      return "FORWARD";
        case NavigationAction::REACHED_GOAL: return "GOAL";
    }
    return "?";
}

// 窄字符串 -> 宽字符串 (用于 GDI+ 绘制)
static std::wstring widen(const char* s) {
    if (!s) return L"";
    int len = MultiByteToWideChar(CP_UTF8, 0, s, -1, nullptr, 0);
    if (len <= 0) return L"";
    std::wstring out(len - 1, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s, -1, &out[0], len);
    return out;
}

// 绘制彩色帧到指定区域 (真机 RGB 画面 + DIST/NEAR 叠加)
static void draw_color_frame(Gdiplus::Graphics& g, int x, int y, int cw, int ch,
                             const ColorFrameData& local_color) {
    using namespace Gdiplus;
    if (!local_color.valid || local_color.rgb.empty() || cw <= 0 || ch <= 0) return;

    // Astra SDK 返回 RGB 字节序; GDI+ PixelFormat24bppRGB 实际期望 BGR
    // (Windows 历史遗留) -> 需交换 R/B 通道, 否则画面红蓝互换偏色
    std::vector<BYTE> bgr_buf(local_color.rgb.size());
    const BYTE* src = local_color.rgb.data();
    for (size_t i = 0; i + 2 < bgr_buf.size(); i += 3) {
        bgr_buf[i]     = src[i + 2];  // B
        bgr_buf[i + 1] = src[i + 1];  // G
        bgr_buf[i + 2] = src[i];      // R
    }
    Bitmap bmp(local_color.width, local_color.height,
               local_color.width * 3, PixelFormat24bppRGB, bgr_buf.data());
    Rect dst(x, y, cw, ch);
    g.DrawImage(&bmp, dst, 0, 0, local_color.width, local_color.height, UnitPixel);

    // 左上角叠加: DIST (中央平均距离, 蓝) + NEAR (最近障碍, 青)
    Font font_big(L"Arial", 16, FontStyleBold);
    Font font_small(L"Arial", 13);
    SolidBrush dist_brush(Color(255, 60, 160, 255));
    SolidBrush near_brush(Color(255, 60, 255, 230));
    SolidBrush shadow(Color(160, 0, 0, 0));

    wchar_t buf[32];
    std::wstring dist_txt, near_txt;
    if (local_color.center_distance_m > 0) {
        swprintf(buf, 32, L"DIST %.2fm", local_color.center_distance_m);
        dist_txt = buf;
    } else dist_txt = L"DIST --";
    if (local_color.nearest_distance_m > 0) {
        swprintf(buf, 32, L"NEAR %.2fm", local_color.nearest_distance_m);
        near_txt = buf;
    } else near_txt = L"NEAR --";

    Gdiplus::PointF ds((REAL)(x + 13), (REAL)(y + 13)), dt((REAL)(x + 12), (REAL)(y + 12));
    Gdiplus::PointF ns((REAL)(x + 37), (REAL)(y + 37)), nt((REAL)(x + 36), (REAL)(y + 36));
    g.DrawString(dist_txt.c_str(), -1, &font_big, ds, &shadow);
    g.DrawString(dist_txt.c_str(), -1, &font_big, dt, &dist_brush);
    g.DrawString(near_txt.c_str(), -1, &font_small, ns, &shadow);
    g.DrawString(near_txt.c_str(), -1, &font_small, nt, &near_brush);

    // 右上角模式提示
    Font font(L"Arial", 12);
    SolidBrush dim(Color(220, 160, 160, 160));
    Gdiplus::PointF tip((REAL)(x + cw - 160), (REAL)(y + 10));
    g.DrawString(L"REAL (RGB)", -1, &font, tip, &dim);
}

// 右对齐绘制标签: 按字符串实测宽度定位 (原固定 220px 边距装不下中文长标签,
// "Cloud SideView (X前→右, Z高→上)" 实宽 ~330px, 会被窗口右缘裁剪)
static void draw_label_right_aligned(Gdiplus::Graphics& g, const wchar_t* text,
                                     float right_x, float y,
                                     const Gdiplus::Font& font,
                                     const Gdiplus::Brush& brush) {
    Gdiplus::RectF measured;
    g.MeasureString(text, -1, &font, Gdiplus::RectF(0, 0, 0, 0), &measured);
    g.DrawString(text, -1, &font, Gdiplus::PointF(right_x - measured.Width - 8.0f, y), &brush);
}

// 绘制点云到指定区域 (camera_link 系; side_view=false=俯视 X-Y, true=侧视 X-Z)
// local_neg: P1 负障碍标记点 (base_link 系, 橙色大点); plane_valid: 地面平面是否锁定
static void draw_cloud_view(Gdiplus::Graphics& g, int x, int y, int cw, int ch,
                            const PointCloud& local_cloud, bool side_view,
                            const PointCloud& local_neg, bool plane_valid,
                            bool depth_flipped) {
    using namespace Gdiplus;
    if (local_cloud.points.empty() || cw <= 0 || ch <= 0) return;

    Font font(L"Arial", 12);
    Font font_big(L"Arial", 16, FontStyleBold);
    SolidBrush white(Color(255, 240, 240, 240));
    SolidBrush dim(Color(220, 160, 160, 160));

    int cxm = x + cw / 2;
    int cym = y + ch / 2 + 20;
    int range_px = (int)(ch * 0.34);
    double max_range = 8.0;

    // 屏幕空间抽稀: 38400 点 → 2px 像素格去重 (~5000-9000 点), GDI+ 绘制量降 4-8x, 视觉等价
    PointCloud sparse;
    screen_decimate(local_cloud, range_px / max_range, sparse);
    const auto& draw_pts = sparse.points;

    if (!side_view) {
        // ===== 俯视图: 前 X=上, 左 Y=右 =====
        // 同心圆 (1m/2m/4m/8m)
        for (int m : {1, 2, 4, 8}) {
            int r = (int)(range_px * (m / max_range));
            Pen pen(Color(60, 255, 255, 255), 1);
            g.DrawEllipse(&pen, cxm - r, cym - r, r * 2, r * 2);
            Gdiplus::PointF tp((REAL)(cxm + r + 4), (REAL)(cym - 4));
            g.DrawString(std::to_wstring(m).c_str(), -1, &font, tp, &dim);
        }
        // 前方方向箭头
        Pen arrow(Color(120, 255, 255, 255), 2);
        g.DrawLine(&arrow, cxm, cym, cxm, cym - range_px - 30);
        // 0.6m 近界警示环 (结构光最小有效距离; 环内深度像素全部置 0 —— 贴墙断崖可视化)
        int r_near = (int)(range_px * (0.6 / max_range));
        Pen near_pen(Color(160, 255, 60, 60), 1);
        g.DrawEllipse(&near_pen, cxm - r_near, cym - r_near, r_near * 2, r_near * 2);
        Gdiplus::PointF np((REAL)(cxm + r_near + 3), (REAL)(cym - 12));
        g.DrawString(L"0.6m", -1, &font, np, &dim);
        // FOV 扇形边界 (水平 58.4°, 相机朝上)
        Pen fov_pen(Color(90, 100, 180, 255), 1);
        double fov_h = 58.4 * 3.14159265 / 180.0;
        int kx1 = cxm + (int)(std::sin(-fov_h/2) * range_px);
        int ky1 = cym - (int)(std::cos(-fov_h/2) * range_px);
        int kx2 = cxm + (int)(std::sin(fov_h/2) * range_px);
        int ky2 = cym - (int)(std::cos(fov_h/2) * range_px);
        g.DrawLine(&fov_pen, cxm, cym, kx1, ky1);
        g.DrawLine(&fov_pen, cxm, cym, kx2, ky2);
        // 机器人本体
        SolidBrush body(Color(255, 70, 130, 200));
        g.FillRectangle(&body, cxm - 18, cym - 18, 36, 36);

        // 点云 (地图视角: X前=屏幕上, Y左=屏幕左) — 近=红, 远=蓝
        // 师兄实测发现左右镜像: camera_link +Y 指向机器人左方, 屏幕应画向左侧
        for (const auto& p : draw_pts) {
            double dist = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
            if (dist > max_range || dist < 0.1) continue;
            int sx = cxm - (int)(p.y / max_range * range_px);
            int sy = cym - (int)(p.x / max_range * range_px);
            if (sx < x || sx >= x + cw || sy < y || sy >= y + ch) continue;
            double t = (std::min)(1.0, dist / max_range);
            int r_c = (int)(255 * (1.0 - t * 0.8));
            int g_c = (int)(200 * (1.0 - std::abs(t - 0.5) * 2.0));
            int b_c = (int)(255 * t);
            SolidBrush pb(Color(200, r_c, g_c, b_c));
            g.FillEllipse(&pb, sx - 1, sy - 1, 2, 2);
        }
        // 负障碍标记点 (base_link; 橙色 4px 大点 —— 坑/下行台阶边缘), 同样按地图视角修正左右
        for (const auto& p : local_neg.points) {
            if (p.x > max_range || p.x < 0.1) continue;
            int sx = cxm - (int)(p.y / max_range * range_px);
            int sy = cym - (int)(p.x / max_range * range_px);
            if (sx < x || sx >= x + cw || sy < y || sy >= y + ch) continue;
            SolidBrush nb(Color(255, 255, 120, 0));
            g.FillEllipse(&nb, sx - 2, sy - 2, 5, 5);
        }
        draw_label_right_aligned(g, L"Cloud TopView (link)", (REAL)(x + cw), (REAL)(y + 10), font, dim);
    } else {
        // ===== 侧视图: 前距离 X=右, 高度 Z=上 =====
        // 网格 (1m 距离线, 高度 ±2m)
        double z_max = 3.0;   // 显示高度 -3~3m
        for (int m : {1, 2, 4, 8}) {
            int rx = (int)(range_px * (m / max_range));
            Pen pen(Color(60, 255, 255, 255), 1);
            g.DrawLine(&pen, cxm + rx, y, cxm + rx, y + ch - 56);
            Gdiplus::PointF tp((REAL)(cxm + rx + 3), (REAL)(y + 8));
            g.DrawString(std::to_wstring(m).c_str(), -1, &font, tp, &dim);
        }
        // 地平线 (z=0)
        int ground_y = cym;   // z=0 → 屏幕 cy (link Z上为+)
        Pen ground(Color(90, 255, 255, 255), 1);
        g.DrawLine(&ground, x, ground_y, x + cw, ground_y);
        // 0.6m 近界警示线 (结构光最小有效距离; 距离<0.6m 深度像素全部置 0)
        {
            int nx = cxm + (int)(0.6 / max_range * range_px);
            Pen np2(Color(160, 255, 60, 60), 1);
            g.DrawLine(&np2, nx, y, nx, y + ch - 56);
            Gdiplus::PointF npt((REAL)(nx + 3), (REAL)(y + 24));
            g.DrawString(L"0.6m", -1, &font, npt, &dim);
        }
        // 相机位置标记
        SolidBrush cam(Color(255, 70, 130, 200));
        g.FillRectangle(&cam, cxm - 9, ground_y - 9, 18, 18);

        // 点云 (X前=右, Z高=上) — 近=红, 远=蓝
        for (const auto& p : draw_pts) {
            if (p.x > max_range || p.x < 0.05) continue;
            if (std::abs(p.y) > max_range) continue;
            double dist = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
            if (dist > max_range) continue;
            int sx = cxm + (int)(p.x / max_range * range_px);
            int sy = ground_y - (int)(p.z / z_max * (ch * 0.34));
            if (sx < x || sx >= x + cw || sy < y || sy >= y + ch) continue;
            double t = (std::min)(1.0, dist / max_range);
            int r_c = (int)(255 * (1.0 - t * 0.8));
            int g_c = (int)(200 * (1.0 - std::abs(t - 0.5) * 2.0));
            int b_c = (int)(255 * t);
            SolidBrush pb(Color(200, r_c, g_c, b_c));
            g.FillEllipse(&pb, sx - 1, sy - 1, 2, 2);
        }
        // 负障碍标记点 (侧视: 沿拟合地面分布的橙色点带 —— 坑/台阶沿)
        for (const auto& p : local_neg.points) {
            if (p.x > max_range || p.x < 0.05) continue;
            if (std::abs(p.y) > max_range) continue;
            int sx = cxm + (int)(p.x / max_range * range_px);
            int sy = ground_y - (int)(p.z / z_max * (ch * 0.34));
            if (sx < x || sx >= x + cw || sy < y || sy >= y + ch) continue;
            SolidBrush nb(Color(255, 255, 120, 0));
            g.FillEllipse(&nb, sx - 2, sy - 2, 5, 5);
        }
        // 高度标尺
        for (double zm : {-2.0, -1.0, 1.0, 2.0}) {
            int sy = ground_y - (int)(zm / z_max * (ch * 0.34));
            Pen p2(Color(50, 255, 255, 255), 1);
            g.DrawLine(&p2, x, sy, x + 8, sy);
            wchar_t zb[16];
            swprintf(zb, 16, L"%.0fm", zm);
            Gdiplus::PointF zp((REAL)(x + 10), (REAL)(sy - 6));
            g.DrawString(zb, -1, &font, zp, &dim);
        }
        draw_label_right_aligned(g, L"Cloud SideView (X前→右, Z高→上)", (REAL)(x + cw), (REAL)(y + 10), font, dim);
    }

    // 底部状态栏
    SolidBrush bar(Color(255, 20, 20, 28));
    g.FillRectangle(&bar, x, y + ch - 56, cw, 56);
    Pen line(Color(80, 255, 255, 255), 1);
    g.DrawLine(&line, x, y + ch - 56, x + cw, y + ch - 56);

    wchar_t buf[256];
    swprintf(buf, 256, L"POINT CLOUD  pts=%zu  frame=%s  seq=%llu  NEG=%zu%s%s",
             local_cloud.points.size(),
             widen(local_cloud.frame_id.c_str()).c_str(),
             (unsigned long long)local_cloud.seq,
             local_neg.points.size(),
             plane_valid ? L"" : L"  [地面未锁定]",
             depth_flipped ? L"  [深度已镜像M]" : L"");
    std::wstring status(buf);
    Gdiplus::PointF sp((REAL)(x + 12), (REAL)(y + ch - 54));
    g.DrawString(status.c_str(), -1, &font_big, sp, &white);

    // 第二行: 空态诊断 —— 有效像素数/比例 + 三态标签 (右缘截断修复: 状态信息不再
    // 挤在第一行被窗口右缘裁掉, 无帧/深度全0/近界断崖可直接读出)
    const int fw = g_frame_w.load(), fh = g_frame_h.load();
    const double denom = (fw > 0 && fh > 0) ? (double)((size_t)fw * fh) : 1.0;
    const size_t vpx = g_cloud_valid_px.load();
    wchar_t buf2[256];
    swprintf(buf2, 256, L"valid_px=%zu (%.0f%%)  %s  viz:%.1fms/tick:%.1fHz",
             vpx, 100.0 * vpx / denom,
             widen(cloud_state_label(g_cloud_state.load())).c_str(),
             g_draw_ms.load(), g_tick_hz.load());
    Gdiplus::PointF sp2((REAL)(x + 12), (REAL)(y + ch - 30));
    g.DrawString(buf2, -1, &font, sp2, &dim);
}

// 窗口绘制 (双缓冲: 先在内存 Bitmap 画完整帧, 再一次性上屏, 消除闪烁)
static void draw_scene_impl(HDC hdc, int w, int h) {
    using namespace Gdiplus;
    // 内存缓冲 (防止逐笔绘制导致闪烁)
    Bitmap mem_bmp(w, h, PixelFormat32bppARGB);
    Graphics g(&mem_bmp);
    g.SetSmoothingMode(SmoothingModeAntiAlias);
    g.Clear(Color(30, 30, 38));

    Font font(L"Arial", 12);
    Font font_big(L"Arial", 16, FontStyleBold);
    SolidBrush white(Color(255, 240, 240, 240));
    SolidBrush dim(Color(220, 160, 160, 160));

    // ===== 加锁拷贝共享数据到局部 (窗口线程读, 主循环写, 防竞态) =====
    ColorFrameData local_color;
    bool have_color = false;
    PointCloud local_cloud;
    bool have_cloud = false;
    PointCloud local_neg;
    bool plane_valid = false;
    const bool depth_flipped = g_flip_depth.load();
    std::vector<uint8_t> local_heat;
    int heat_w = 0, heat_h = 0;
    bool heat_valid = false;
    {
        std::lock_guard<std::mutex> lock(g_viz_mutex);
        local_color = g_color_frame;
        have_color = g_have_color;
        local_cloud = g_latest_cloud;
        have_cloud = g_have_cloud;
        local_neg = g_neg_cloud;
        plane_valid = g_plane_valid;
        local_heat = g_heat_bgr;
        heat_w = g_heat_w; heat_h = g_heat_h;
        heat_valid = g_heat_valid;
    }

    // ===== 节流: 点云数据未更新且距上次绘制 <80ms → 整帧跳过 (陈旧重绘抑制) =====
    if (g_view_cloud && have_cloud && local_cloud.seq == g_last_drawn_seq &&
        std::chrono::steady_clock::now() - g_last_draw_t < std::chrono::milliseconds(80)) {
        return;
    }

    // ===== D 热力诊断模式: 深度热力图叠加 RGB (对齐裁决) =====
    if (g_heat_overlay.load() && heat_valid && !local_heat.empty()) {
        Bitmap heat_bmp(heat_w, heat_h, heat_w * 3, PixelFormat24bppRGB,
                        local_heat.data());
        g.DrawImage(&heat_bmp, Rect(0, 0, w, h), 0, 0, heat_w, heat_h, UnitPixel);
        SolidBrush hb2(Color(255, 255, 220, 0));
        g.DrawString(L"[D 热力诊断] 近=红 远=蓝  红点应落在实际近处物体上  按 D 关闭",
                     -1, &font, PointF(10, 10), &hb2);
        Graphics screen(hdc);
        screen.DrawImage(&mem_bmp, 0, 0, w, h);
        return;
    }

    bool color_ok = have_color && local_color.valid && !local_color.rgb.empty();
    bool cloud_ok = have_cloud && !local_cloud.points.empty();

    // ===== 点云模式: 优先点云视图 (分屏或全屏) =====
    if (g_view_cloud) {
        if (color_ok && cloud_ok) {
            // 分屏: 左半彩色帧, 右半点云俯视图
            int half = w / 2;
            draw_color_frame(g, 0, 0, half, h, local_color);
            draw_cloud_view(g, half, 0, w - half, h, local_cloud, g_view_side,
                            local_neg, plane_valid, depth_flipped);
            Pen sep(Color(120, 255, 255, 255), 1);
            g.DrawLine(&sep, half, 0, half, h);
        } else if (cloud_ok) {
            // 仅有彩色帧时也要保证有点云可看 (否则退化全屏点云)
            draw_cloud_view(g, 0, 0, w, h, local_cloud, g_view_side,
                            local_neg, plane_valid, depth_flipped);
        } else if (color_ok) {
            // 点云尚未就绪, 暂退彩色帧
            draw_color_frame(g, 0, 0, w, h, local_color);
        } else {
            Gdiplus::PointF tip((REAL)(w - 200), 10);
            g.DrawString(L"waiting for cloud...", -1, &font, tip, &dim);
            Gdiplus::PointF sp(12, (REAL)(h - 52));
            g.DrawString(L"POINT CLOUD  (no data)", -1, &font_big, sp, &white);
        }

        // 一次性上屏 (双缓冲)
        Graphics screen(hdc);
        screen.DrawImage(&mem_bmp, 0, 0, w, h);
        g_last_drawn_seq = local_cloud.seq;               // 节流记录 (仅点云模式比较)
        g_last_draw_t = std::chrono::steady_clock::now();
        return;
    }

    // ===== 非点云模式: 彩色帧优先 =====
    if (color_ok) {
        draw_color_frame(g, 0, 0, w, h, local_color);
        Graphics screen(hdc);
        screen.DrawImage(&mem_bmp, 0, 0, w, h);
        return;
    }

    // 加锁拷贝共享融合结果到局部 (窗口线程读, 主循环写, 防竞态; 与彩色帧同法, FIX-1)
    FusionResult local_result;
    bool have_result = false;
    {
        std::lock_guard<std::mutex> lock(g_viz_mutex);
        local_result = g_latest_result;
        have_result = g_have_result;
    }

    int cx = w / 2;
    int cy = h / 2 + 20;
    int range_px = (int)(h * 0.34);   // 8m 对应像素
    double max_range = 8.0;           // 与 config.h 一致

    // 同心圆 (2m/4m/8m)
    for (int m : {2, 4, 8}) {
        int r = (int)(range_px * (m / max_range));
        Pen pen(Color(60, 255, 255, 255), 1);
        g.DrawEllipse(&pen, cx - r, cy - r, r * 2, r * 2);
        // 标注
        Gdiplus::PointF tp((REAL)(cx + r + 4), (REAL)(cy - 4));
        g.DrawString(std::to_wstring(m).c_str(), -1, &font, tp, &dim);
    }

    // 前方方向箭头
    Pen arrow(Color(120, 255, 255, 255), 2);
    g.DrawLine(&arrow, cx, cy, cx, cy - range_px - 30);

    // 机器人本体 (中心方块)
    SolidBrush body(Color(255, 70, 130, 200));
    g.FillRectangle(&body, cx - 22, cy - 22, 44, 44);
    Color body_ring_color(255, 200, 220, 255);
    Pen body_ring_pen(body_ring_color, 2);
    g.DrawRectangle(&body_ring_pen, cx - 22, cy - 22, 44, 44);

    // 障碍物点
    if (have_result) {
        for (const auto& kv : local_result.obstacles) {
            double d = kv.second.distance_m;
            if (d <= 0.0 || d > max_range) continue;

            // 方向角度 (底部特殊处理: 画在机器人正下方)
            auto it = g_dir_angle.find(kv.first);
            double ang_deg = (it != g_dir_angle.end()) ? it->second : 0.0;
            double rad = ang_deg * 3.14159265 / 180.0;

            // 俯视图: 角度顺时针, 距离向上
            double px, py;
            if (kv.first == "bottom") {
                px = 0.0; py = d;   // 正下方
            } else {
                px = sin(rad) * d;
                py = -cos(rad) * d;
            }
            int sx = cx + (int)(px / max_range * range_px);
            int sy = cy + (int)(py / max_range * range_px);

            COLORREF c = level_color(kv.second.level);
            SolidBrush b(Color(255, GetRValue(c), GetGValue(c), GetBValue(c)));
            int dot = (kv.second.level == ObstacleLevel::CRITICAL) ? 12 : 8;
            g.FillEllipse(&b, sx - dot / 2, sy - dot / 2, dot, dot);

            // 距离标签
            Gdiplus::PointF lp((REAL)(sx + 6), (REAL)(sy - 10));
            std::wstring label = std::to_wstring((int)(d * 100)) + L"cm";
            g.DrawString(label.c_str(), -1, &font, lp, &white);
        }
    }

    // 底部状态栏
    SolidBrush bar(Color(255, 20, 20, 28));
    g.FillRectangle(&bar, 0, h - 56, w, 56);
    Pen line(Color(80, 255, 255, 255), 1);
    g.DrawLine(&line, 0, h - 56, w, h - 56);

    std::wstring status;
    if (have_result) {
        auto& r = local_result;
        status = L"action=" + widen(action_name(r.recommended_action)) +
                 L"   env=" + std::to_wstring((int)r.environment) +
                 L"   cliff=" + (r.cliff_detected ? L"YES" : L"no") +
                 L"   min_fwd=" + std::to_wstring(r.min_forward_distance_m * 100) + L"cm";
    } else {
        status = L"waiting for data...";
    }
    Gdiplus::PointF sp(12, (REAL)(h - 52));
    g.DrawString(status.c_str(), -1, &font_big, sp, &white);

    // 右上角说明
    Gdiplus::PointF tip((REAL)(w - 220), 10);
    g.DrawString(L"\u5f00\u53d1\u89c6\u89d2: \u6a21\u62df\u6a21\u5f0f", -1, &font, tip, &dim);

    // 一次性上屏 (双缓冲)
    Graphics screen(hdc);
    screen.DrawImage(&mem_bmp, 0, 0, w, h);
}

// 计时包装: 记录单帧绘制耗时 (EMA → 状态栏 viz 行)
static void draw_scene(HDC hdc, int w, int h) {
    const auto d0 = std::chrono::steady_clock::now();
    draw_scene_impl(hdc, w, h);
    const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - d0).count();
    const double ema = g_draw_ms.load();
    g_draw_ms.store(ema <= 0.0 ? ms : 0.9 * ema + 0.1 * ms);
}

static LRESULT CALLBACK wnd_proc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
        case WM_PAINT: {
            PAINTSTRUCT ps;
            HDC hdc = BeginPaint(hwnd, &ps);
            RECT rc; GetClientRect(hwnd, &rc);
            draw_scene(hdc, rc.right - rc.left, rc.bottom - rc.top);
            EndPaint(hwnd, &ps);
            return 0;
        }
        case WM_ERASEBKGND:
            return 1;
        case WM_KEYDOWN:
            // S: 切换点云侧视图/俯视图 (仅点云模式有效)
            if (wp == 'S' || wp == 's') g_view_side = !g_view_side;
            // M: 深度图水平镜像开关 (左右方向诊断用)
            if (wp == 'M' || wp == 'm') g_flip_depth.store(!g_flip_depth.load());
            // D: 深度热力图叠加 RGB (深度/RGB 对齐裁决)
            if (wp == 'D' || wp == 'd') g_heat_overlay.store(!g_heat_overlay.load());
            return 0;
        case WM_CLOSE:
            g_window_open = 0;
            g_stop.store(1);
            DestroyWindow(hwnd);
            return 0;
        case WM_DESTROY:
            g_window_open = 0;
            g_stop.store(1);
            PostQuitMessage(0);
            return 0;
    }
    return DefWindowProc(hwnd, msg, wp, lp);
}

// 窗口线程: 创建窗口 + 消息循环, 收到 WM_TIMER 时刷新
static DWORD WINAPI window_thread(LPVOID) {
    using namespace Gdiplus;
    GdiplusStartup(&g_gdiplus_token, &g_gdiplus_startup, nullptr);

    WNDCLASSW wc = {};
    wc.lpfnWndProc = wnd_proc;
    wc.hInstance = GetModuleHandle(nullptr);
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
    wc.lpszClassName = L"MechdogViz";
    RegisterClassW(&wc);

    RECT wr = {0, 0, 820, 640};
    AdjustWindowRect(&wr, WS_OVERLAPPEDWINDOW, FALSE);
    g_hwnd = CreateWindowW(
        L"MechdogViz", L"mechdog_navigation - 实时可视化 (模拟模式)",
        WS_OVERLAPPEDWINDOW, 60, 40,
        wr.right - wr.left, wr.bottom - wr.top,
        nullptr, nullptr, wc.hInstance, nullptr);

    if (!g_hwnd) {
        g_window_open = 0;
        return 1;
    }
    ShowWindow(g_hwnd, SW_SHOW);
    SetTimer(g_hwnd, 1, 100, nullptr);   // 100ms 刷新 (10Hz, 流畅)

    MSG msg;
    while (g_window_open && GetMessage(&msg, nullptr, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
        if (msg.message == WM_TIMER) {
            // 窗口线程独立刷新彩色帧 (画面流畅, 不依赖主循环 4-5Hz)
            if (g_astra_ptr && g_astra_ptr->is_real()) {
                auto cf = g_astra_ptr->get_color_frame();
                std::lock_guard<std::mutex> lock(g_viz_mutex);
                g_color_frame = cf;
                g_have_color = cf.valid;
            }
            InvalidateRect(g_hwnd, nullptr, FALSE);
        }
    }
    KillTimer(g_hwnd, 1);
    GdiplusShutdown(g_gdiplus_token);
    return 0;
}

#endif // _WIN32

int main(int argc, char** argv) {
#ifdef _WIN32
    // Windows 控制台默认 GBK(936), 而源码/输出是 UTF-8 -> 中文乱码
    SetConsoleOutputCP(CP_UTF8);
#endif
    std::signal(SIGINT, on_signal);

    // 命令行参数: --real 使用真机 (Astra SDK + 真实红外), 默认模拟模式
    //             --cloud 点云可视化 (深度图反投影, 俯视图)
    bool use_real = false;
    // maybe_unused: no_viz 仅 Windows 可视化链路 (_WIN32 块) 消费, Linux/gcc 下未用
    [[maybe_unused]] bool no_viz = false;
    bool show_cloud = false;
    bool ground_free = false;
    double ground_h = 0.8;   // 手持相机离地高度 (米), --ground <h> 覆盖
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--real" || arg == "-r") use_real = true;
        if (arg == "--noviz") no_viz = true;
        if (arg == "--cloud" || arg == "-c") show_cloud = true;
        if (arg == "--free") ground_free = true;  // 等价 --ground 0.8
        if (arg == "--ground" && i + 1 < argc) {
            ground_free = true;
            ground_h = (std::max)(0.1, atof(argv[++i]));  // 括号包住: windows.h 的 max 宏冲突
        }
    }

#ifdef USE_ASTRA_SDK
    // 真机模式: use_simulated=false 走 capture_real() (Astra SDK 深度图)
    AstraProDriver astra(/*use_simulated=*/!use_real);
    InfraRedSensor ir(/*use_simulated=*/!use_real);
#else
    // 未编译 Astra SDK 时强制模拟, 避免误用真机参数
    if (use_real) {
        std::cout << "[警告] 当前未启用 USE_ASTRA_SDK, --real 无效, 使用模拟模式" << std::endl;
        use_real = false;
    }
    AstraProDriver astra(/*use_simulated=*/true);
    InfraRedSensor ir(/*use_simulated=*/true);
#endif
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    SensorFusion fusion(&astra, &ultrasonic, &ir);

    PathPlanner planner;

#ifdef _WIN32
    // 真机模式: 先在 main 线程预初始化 Astra SDK (深度+彩色双流 start),
    // 再启动窗口线程, 避免采集线程/窗口线程同时初始化相机导致 start 阻塞
    if (use_real) {
        astra.init_hardware();
    }
    // 窗口线程独立刷新彩色帧用
    g_astra_ptr = &astra;
    // 点云可视化模式: 提前设全局标志 (窗口线程 draw_scene 据此优先渲染点云)
    g_view_cloud = show_cloud;
    // 启动可视化窗口线程 (--noviz 可禁用, 用于定位崩溃)
    if (!no_viz) {
        CreateThread(nullptr, 0, window_thread, nullptr, 0, nullptr);
    }
#endif

    astra.start();

    std::cout << "=== mechdog_navigation "
              << (use_real ? "真机模式 (Astra SDK)" : "模拟模式")
              << (show_cloud ? " + 点云可视化" : "")
              << " (带可视化窗口) ===" << std::endl;
    std::cout << "关闭可视化窗口 或 Ctrl+C 退出" << std::endl << std::endl;

    CameraIntrinsics cloud_K;       // FOV 反推内参 (真机可后续补 SDK 直读)
    CameraExtrinsics cloud_E;       // 外参初值 (装机后量测)
    // 狗未接时用占位外参 (§18.4): 无偏移无俯角, 看光学系→link 系形状
    if (show_cloud && !use_real) {
        // 模拟模式点云: 无外参, 纯验证链路
        cloud_E.x = 0; cloud_E.y = 0; cloud_E.z = 0;
        cloud_E.roll = 0; cloud_E.pitch = 0; cloud_E.yaw = 0;
    }
    // P1 地面分割手持模式 (--ground <h> / --free): 外参归零使 base==link
    // (可视化标记与点云同系对齐), 高度先验 = -h, 窗口收紧 —— "地面在哪"由真实
    // 持机高度决定, 不让 RANSAC 在地板/床板/桌面之间跳 (宿舍实测踩过的坑)。
    // 装机后: 去掉 --ground/--free, 用默认外参 + 紧窗口 (prior_window=0.10)。
    GroundSegParams gseg_params;
    if (ground_free) {
        cloud_E.x = 0; cloud_E.y = 0; cloud_E.z = 0;
        cloud_E.roll = 0; cloud_E.pitch = 0; cloud_E.yaw = 0;
        gseg_params.ground_prior_z = -ground_h;
        gseg_params.prior_window = 0.12;
        std::cout << "[P1] 手持模式: 相机离地 " << ground_h
                  << "m, 地面先验 z=" << gseg_params.ground_prior_z
                  << " (base==link)" << std::endl;
    }

    unsigned int tick = 0;
    while (!g_stop.load()) {
        const auto t0 = std::chrono::steady_clock::now();
        auto result = fusion.fuse();
        const auto t1 = std::chrono::steady_clock::now();

        auto cmd = planner.plan(result);

#ifdef _WIN32
        // 更新可视化共享数据 (彩色帧由窗口线程独立刷新, 这里只更新融合结果)
        {
            std::lock_guard<std::mutex> lock(g_viz_mutex);
            g_latest_result = result;
            g_have_result = true;
            if (!use_real) g_have_color = false;
        }

        // 点云模式: 从最新深度帧反投影 → optical→link → 存共享
        if (show_cloud) {
            AstraFrame frame = astra.get_latest_frame();
            if (!frame.valid) {
                // 三态诊断 0: 取帧失败 → 旧点云保留 (状态栏示警, 避免逐帧闪烁)
                g_cloud_state.store(0);
            } else {
                // 帧分辨率变化时打印一次 (验证 640×480 内参假定; 若真机非该分辨率 → 内参错位)
                if (frame.depth_width != g_frame_w.load() ||
                    frame.depth_height != g_frame_h.load()) {
                    std::cout << "[viz] depth frame " << frame.depth_width << "x"
                              << frame.depth_height << std::endl;
                    g_frame_w.store(frame.depth_width);
                    g_frame_h.store(frame.depth_height);
                }
                g_cloud_valid_px.store(count_valid_pixels(frame.depth_map));
                g_cloud_state.store(g_cloud_valid_px.load() == 0 ? 1 : 2);
            }
            if (frame.valid && !frame.depth_map.empty()
                && frame.depth_width > 0 && frame.depth_height > 0) {
                // 诊断: M 键切换的深度图水平镜像 (只影响本可视化的点云, 不影响融合)
                if (g_flip_depth.load()) {
                    const int fw = frame.depth_width, fh = frame.depth_height;
                    for (int y = 0; y < fh; ++y) {
                        std::reverse(frame.depth_map.begin() + (size_t)y * fw,
                                     frame.depth_map.begin() + (size_t)(y + 1) * fw);
                    }
                }
                PointCloud cloud_opt;
                depth_to_cloud(frame.depth_map.data(),
                               frame.depth_width, frame.depth_height,
                               cloud_K, cloud_opt);

                // optical → link (固定旋转, §6.1)
                PointCloud cloud_link;
                transform_optical_to_link(cloud_opt, cloud_link);
                cloud_link.seq = frame.frame_seq;
                cloud_link.stamp = frame.timestamp;

                // 下采样: 每隔 step 个点取 1 个 (640×480 全量 ~30万点, 画不动)
                const int step = 8;
                PointCloud cloud_ds;
                cloud_ds.seq = cloud_link.seq;
                cloud_ds.stamp = cloud_link.stamp;
                cloud_ds.frame_id = cloud_link.frame_id;
                cloud_ds.points.reserve(cloud_link.points.size() / step + 1);
                for (size_t i = 0; i < cloud_link.points.size(); i += step) {
                    cloud_ds.points.push_back(cloud_link.points[i]);
                }

                // P1 负障碍: 降采样云 → base_link → 地面分割 (--free 手持放宽先验)
                PointCloud cloud_base;
                transform_to_base(cloud_ds, cloud_E, cloud_base);
                GroundSegResult seg;
                segment_ground(cloud_base, gseg_params, seg);

                std::lock_guard<std::mutex> lock(g_viz_mutex);
                g_latest_cloud = cloud_ds;
                g_have_cloud = true;
                g_neg_cloud.seq = cloud_ds.seq;
                g_neg_cloud.stamp = cloud_ds.stamp;
                g_neg_cloud.frame_id = "base_link";
                g_neg_cloud.points = std::move(seg.negative_points);
                g_plane_valid = seg.plane.valid;
            }

            // D 诊断: 深度热力图混合 RGB —— 深度像素按距离着色(近红远蓝)后
            // 半透明叠回彩色图。若热力红点落在实际近处物体上 → 深度与 RGB 对齐;
            // 若红点出现在远处物体上 → 深度图相对 RGB 左右镜像, 需驱动层翻转。
            if (g_heat_overlay.load()) {
                ColorFrameData cf = astra.get_color_frame();
                const int hw = frame.depth_width, hh = frame.depth_height;
                std::vector<uint8_t> bgr((size_t)hw * hh * 3, 0);
                const bool have_cf = cf.valid && cf.width == hw && cf.height == hh &&
                                     cf.rgb.size() == cf.width * cf.height * 3;
                for (int y = 0; y < hh; ++y) {
                    for (int x = 0; x < hw; ++x) {
                        const uint16_t d = frame.depth_map[(size_t)y * hw + x];
                        uint8_t pr = 0, pg = 0, pb = 0;
                        if (d > 0) {
                            double t = (d / 1000.0 - 0.6) / (8.0 - 0.6);
                            t = (std::min)(1.0, (std::max)(0.0, t));
                            pr = (uint8_t)(255 * (1 - t));   // 近=红
                            pg = 70;
                            pb = (uint8_t)(255 * t);          // 远=蓝
                        }
                        const size_t o = ((size_t)y * hw + x) * 3;
                        if (have_cf) {
                            bgr[o]     = (uint8_t)(0.45 * pb + 0.55 * cf.rgb[o + 2]);
                            bgr[o + 1] = (uint8_t)(0.45 * pg + 0.55 * cf.rgb[o + 1]);
                            bgr[o + 2] = (uint8_t)(0.45 * pr + 0.55 * cf.rgb[o]);
                        } else {
                            bgr[o] = pb; bgr[o + 1] = pg; bgr[o + 2] = pr;
                        }
                    }
                }
                std::lock_guard<std::mutex> lock(g_viz_mutex);
                g_heat_bgr = std::move(bgr);
                g_heat_w = hw; g_heat_h = hh;
                g_heat_valid = true;
            }
        }
#endif
        const auto t2 = std::chrono::steady_clock::now();   // 点云管线耗时 (仅 --cloud 时有增量)

        std::cout << "[" << std::fixed << std::setprecision(2)
                  << result.timestamp << "] tick=" << tick
                  << " env=" << static_cast<int>(result.environment)
                  << " astra_w=" << result.effective_astra_weight
                  << " ultra_w=" << result.effective_ultrasonic_weight
                  << " cliff=" << (result.cliff_detected ? "YES" : "no")
                  << " min_fwd=" << result.min_forward_distance_m << "m"
                  << " action=" << static_cast<int>(result.recommended_action)
                  << " vel=(" << cmd.linear << ", " << cmd.angular << ")"
#ifdef _WIN32
                  // P1: g_latest_cloud 仅在上方 _WIN32 块声明, 非 Windows 平台无点云可视化;
                  // 无条件引用曾导致 Linux/gcc 构建直接失败 (第五轮 review)
                  << (show_cloud ? " cloud_pts=" + std::to_string(g_latest_cloud.points.size()) : "")
#endif
                  << std::endl;

        // 帧率诊断: tick 实际周期 + 分段耗时 (EMA 输出到状态栏)
        {
            const double tick_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
            const double fuse_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            const double cloud_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
            const double hz = tick_ms > 0.0 ? 1000.0 / tick_ms : 0.0;
            const double ema = g_tick_hz.load();
            g_tick_hz.store(ema <= 0.0 ? hz : 0.9 * ema + 0.1 * hz);
            std::cout << "  [perf] fuse_ms=" << fuse_ms
                      << " cloud_ms=" << cloud_ms
                      << " tick_ms=" << tick_ms
                      << " hz=" << hz << std::endl;
        }

        for (const auto& kv : result.obstacles) {
            std::cout << "    " << kv.first
                      << ": " << std::setw(6) << kv.second.distance_m << "m"
                      << " conf=" << kv.second.confidence
                      << " lvl=" << static_cast<int>(kv.second.level)
                      << " [" << kv.second.source << "]"
                      << std::endl;
        }
        std::cout << std::endl;

        ++tick;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    astra.stop();
    std::cout << "已退出" << std::endl;
    return 0;
}
