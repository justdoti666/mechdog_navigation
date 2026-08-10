/**
 * mechdog_navigation 主程序 - 带实时可视化窗口 (Windows GDI+)
 *
 * 默认以模拟模式运行: 无需硬件, 在 PC 上即可验证传感器融合与导航决策全链路。
 * 真机模式: 通过 CMake 选项 USE_WIRINGPI / USE_OPENNI2 编译, 并修改下方 use_simulated 参数。
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

#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>
#include <map>

#ifdef _WIN32
#include <windows.h>
#include <gdiplus.h>
#pragma comment(lib, "gdiplus.lib")
#endif

using namespace mechdog;

static volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

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

// 窗口绘制
static void draw_scene(HDC hdc, int w, int h) {
    using namespace Gdiplus;
    Graphics g(hdc);
    g.SetSmoothingMode(SmoothingModeAntiAlias);
    g.Clear(Color(30, 30, 38));

    Font font(L"Arial", 12);
    Font font_big(L"Arial", 16, FontStyleBold);
    SolidBrush white(Color(255, 240, 240, 240));
    SolidBrush dim(Color(220, 160, 160, 160));

    // ===== 彩色模式: 真机 RGB 画面 + DIST/NEAR 叠加 =====
    if (g_have_color && g_color_frame.valid && !g_color_frame.rgb.empty()) {
        // 画彩色帧 (拉伸到窗口客户区)
        Bitmap bmp(g_color_frame.width, g_color_frame.height,
                   g_color_frame.width * 3, PixelFormat24bppRGB,
                   const_cast<BYTE*>(g_color_frame.rgb.data()));
        Rect dst(0, 0, w, h);
        g.DrawImage(&bmp, dst, 0, 0, g_color_frame.width, g_color_frame.height,
                    UnitPixel);

        // 左上角叠加: DIST (中央平均距离, 蓝) + NEAR (最近障碍, 青)
        SolidBrush dist_brush(Color(255, 60, 160, 255));
        SolidBrush near_brush(Color(255, 60, 255, 230));
        SolidBrush shadow(Color(160, 0, 0, 0));

        std::wstring dist_txt;
        if (g_color_frame.center_distance_m > 0) {
            wchar_t buf[32];
            swprintf(buf, 32, L"DIST %.2fm", g_color_frame.center_distance_m);
            dist_txt = buf;
        } else {
            dist_txt = L"DIST --";
        }
        std::wstring near_txt;
        if (g_color_frame.nearest_distance_m > 0) {
            wchar_t buf[32];
            swprintf(buf, 32, L"NEAR %.2fm", g_color_frame.nearest_distance_m);
            near_txt = buf;
        } else {
            near_txt = L"NEAR --";
        }

        // 阴影 + 文字 (字号 18/13, 与 rgb_stream 一致)
        Gdiplus::PointF ds(13, 13), dt(12, 12);
        Gdiplus::PointF ns(37, 37), nt(36, 36);
        g.DrawString(dist_txt.c_str(), -1, &font_big, ds, &shadow);
        g.DrawString(dist_txt.c_str(), -1, &font_big, dt, &dist_brush);
        Font font_small(L"Arial", 13);
        g.DrawString(near_txt.c_str(), -1, &font_small, ns, &shadow);
        g.DrawString(near_txt.c_str(), -1, &font_small, nt, &near_brush);

        // 右上角模式提示
        Gdiplus::PointF tip((REAL)(w - 160), 10);
        g.DrawString(L"REAL (RGB)", -1, &font, tip, &dim);
        return;
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
    if (g_have_result) {
        for (const auto& kv : g_latest_result.obstacles) {
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
    if (g_have_result) {
        auto& r = g_latest_result;
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
        case WM_CLOSE:
            g_window_open = 0;
            g_stop = 1;
            DestroyWindow(hwnd);
            return 0;
        case WM_DESTROY:
            g_window_open = 0;
            g_stop = 1;
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
    SetTimer(g_hwnd, 1, 200, nullptr);   // 200ms 刷新

    MSG msg;
    while (g_window_open && GetMessage(&msg, nullptr, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
        if (msg.message == WM_TIMER) {
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
    // 启动可视化窗口线程
    CreateThread(nullptr, 0, window_thread, nullptr, 0, nullptr);
#endif
    std::signal(SIGINT, on_signal);

    // 命令行参数: --real 使用真机 (Astra SDK + 真实红外), 默认模拟模式
    bool use_real = false;
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--real" || arg == "-r") use_real = true;
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

    astra.start();

    std::cout << "=== mechdog_navigation "
              << (use_real ? "真机模式 (Astra SDK)" : "模拟模式")
              << " (带可视化窗口) ===" << std::endl;
    std::cout << "关闭可视化窗口 或 Ctrl+C 退出" << std::endl << std::endl;

    unsigned int tick = 0;
    while (!g_stop) {
        auto result = fusion.fuse();

        auto cmd = planner.plan(result);

#ifdef _WIN32
        // 更新可视化共享数据
        g_latest_result = result;
        g_have_result = true;

        // 真机模式: 读彩色帧用于 RGB 可视化 (模拟模式返回无效, 窗口保持俯视图)
        if (use_real) {
            g_color_frame = astra.get_color_frame();
            g_have_color = g_color_frame.valid;
        } else {
            g_have_color = false;
        }
#endif

        std::cout << "[" << std::fixed << std::setprecision(2)
                  << result.timestamp << "] tick=" << tick
                  << " env=" << static_cast<int>(result.environment)
                  << " astra_w=" << result.effective_astra_weight
                  << " ultra_w=" << result.effective_ultrasonic_weight
                  << " cliff=" << (result.cliff_detected ? "YES" : "no")
                  << " min_fwd=" << result.min_forward_distance_m << "m"
                  << " action=" << static_cast<int>(result.recommended_action)
                  << " vel=(" << cmd.linear << ", " << cmd.angular << ")"
                  << std::endl;

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
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    astra.stop();
    std::cout << "已退出" << std::endl;
    return 0;
}
