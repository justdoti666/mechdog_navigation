/**
 * Astra Pro 深度相机驱动模块实现
 */
#include "sensor_astra.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

#ifdef USE_ASTRA_SDK
#include <astra/astra.hpp>
#endif

namespace mechdog {

#ifdef USE_ASTRA_SDK
// H2: 停止 SDK update 线程 (stop() 在其定义之前调用;
//     实现在文件中部 USE_ASTRA_SDK 块内, 同一翻译单元)
void shutdown_real_ctx();
#endif

AstraProDriver::AstraProDriver(bool use_simulated)
    : use_simulated_(use_simulated)
    , rng_(std::random_device{}()) {
    if (!use_simulated_) {
        std::cout << "[Astra] 真实硬件模式（Orbbec Astra SDK）" << std::endl;
    }
}

AstraProDriver::~AstraProDriver() {
    stop();
}

// 预初始化: 在 main 线程调用 ensure_real_streams (深度+彩色双流 start)
// 避免在 capture_loop 线程里首次 start 时与窗口线程竞争导致阻塞
// (实现在文件末尾 USE_ASTRA_SDK 块内, real_ctx/ensure_real_streams 已定义)
void AstraProDriver::start() {
    if (running_) return;
    running_ = true;
    capture_thread_ = std::make_unique<std::thread>(&AstraProDriver::capture_loop, this);
    std::cout << "[Astra] 采集线程已启动" << std::endl;
}

void AstraProDriver::stop() {
    running_ = false;
    if (capture_thread_ && capture_thread_->joinable()) {
        capture_thread_->join();
        capture_thread_.reset();
    }
#ifdef USE_ASTRA_SDK
    // H2: 停掉 SDK update 线程 (先 join capture 线程, 确保 capture_real 不再访问 ctx 后再停)
    shutdown_real_ctx();
#endif
    std::cout << "[Astra] 采集已停止" << std::endl;
}

void AstraProDriver::capture_loop() {
    while (running_) {
        auto frame = capture_frame();
        {
            std::lock_guard<std::mutex> guard(lock_);
            latest_frame_ = std::move(frame);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / DEPTH_FPS));
    }
}

AstraFrame AstraProDriver::capture_frame() {
    if (use_simulated_) {
        return simulate_frame();
    }
#ifdef USE_ASTRA_SDK
    return capture_real();
#else
    // 未编译 Astra SDK 支持时返回无效帧
    AstraFrame frame;
    frame.timestamp = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    frame.valid = false;
    return frame;
#endif
}

#ifdef USE_ASTRA_SDK
// ========== 真机模式 (Astra SDK) ==========
// 使用官方 FrameListener 回调模式 (与 SDK 示例 DepthReaderEventCPP 一致):
//   首次 capture_real 调用时初始化 + 启动双流, 然后独立线程跑 astra_update() 驱动回调。

namespace {

// 前向声明 (init_hardware 在匿名命名空间定义前使用)
struct RealAstraContext;
RealAstraContext& real_ctx();
void ensure_real_streams(RealAstraContext& ctx);

// 全局共享上下文: StreamSet/Reader + update 线程
struct RealAstraContext {
    bool inited = false;
    astra::StreamSet streamSet;
    astra::StreamReader reader;
    std::thread update_thread;
    std::atomic<bool> running{false};
    std::atomic<bool> streams_ready{false};

    // reader 保护锁 (capture 线程和 get_color_frame 窗口线程都会访问 reader)
    std::mutex reader_mutex;

    // 彩色帧缓存 (capture_real 轮询时顺带更新, get_color_frame 只读)
    std::mutex color_mutex;
    ColorFrameData color;
    bool have_color = false;

    RealAstraContext() {
        astra::initialize();
        reader = streamSet.create_reader();
        inited = true;
    }

    // H2 修复: 停止 update 线程并 join
    void shutdown() {
        running = false;
        if (update_thread.joinable()) update_thread.join();
    }

    ~RealAstraContext() { shutdown(); }
};

RealAstraContext& real_ctx() {
    static RealAstraContext ctx;
    return ctx;
}

// 启动双流 + update 线程 (仅一次)
void ensure_real_streams(RealAstraContext& ctx) {
    if (ctx.streams_ready.load()) return;

    std::cout << "[Astra] 启动深度流..." << std::endl;
    auto depthStream = ctx.reader.stream<astra::DepthStream>();
    depthStream.start();
    std::cout << "[Astra] 深度流已启动 (serial="
              << depthStream.serial_number() << ")" << std::endl;

    std::cout << "[Astra] 启动彩色流..." << std::endl;
    auto colorStream = ctx.reader.stream<astra::ColorStream>();
    colorStream.start();
    std::cout << "[Astra] 彩色流已启动" << std::endl;

    ctx.streams_ready = true;

    // 独立线程持续 astra_update() 驱动 SDK 内部状态 (轮询模式也需要)
    ctx.running = true;
    ctx.update_thread = std::thread([&ctx]() {
        while (ctx.running) {
            astra_update();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });
}

} // namespace (匿名: RealAstraContext/real_ctx/ensure_real_streams 内部链接)

// H2: 停止 SDK update 线程
// 定义必须在匿名命名空间外 (mechdog 直接作用域, 外部链接) 以匹配文件头部的声明 ——
// 原定义放匿名 namespace 内 (内部链接) 导致 MSVC USE_ASTRA_SDK 构建 LNK2019。
// real_ctx() 虽在匿名 namespace (内部链接), 但同一翻译单元内可正常调用。
void shutdown_real_ctx() {
    real_ctx().shutdown();
}

AstraFrame AstraProDriver::capture_real() {
    auto& ctx = real_ctx();
    ensure_real_streams(ctx);

    AstraFrame frame;
    frame.timestamp = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    frame.depth_width  = DEPTH_WIDTH;
    frame.depth_height = DEPTH_HEIGHT;

    // 真机修复: 轮询模式取帧 (SDK DepthReaderPoll 示例同法)。
    // 原实现用 FrameListener 回调模式, Astra Pro 在回调模式下深度值恒 0
    // (SDK 已知行为; DepthReaderPoll 轮询模式正常出值)。
    std::vector<int16_t> depth;
    int w = 0, h = 0;
    {
        std::lock_guard<std::mutex> guard(ctx.reader_mutex);

        // 短等待重试 astra_update() 直到 has_new_frame (同 rgb_stream read_frame)
        bool got = false;
        for (int attempt = 0; attempt < 5; ++attempt) {
            astra_update();
            if (ctx.reader.has_new_frame()) { got = true; break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        if (!got) {
            frame.valid = false;
            return frame;
        }

        astra::Frame aframe = ctx.reader.get_latest_frame(0);  // timeout=0 非阻塞
        auto depthFrame = aframe.get<astra::DepthFrame>();
        auto colorFrame = aframe.get<astra::ColorFrame>();

        if (!depthFrame.is_valid() || depthFrame.width() <= 0) {
            frame.valid = false;
            return frame;
        }

        w = depthFrame.width();
        h = depthFrame.height();
        depth.resize(w * h);
        depthFrame.copy_to(depth.data());   // int16_t 原生 (FIX-10)
        frame.valid = true;

        // 顺带更新彩色帧缓存 (窗口线程经 get_color_frame 读取)
        if (colorFrame.is_valid() && colorFrame.width() > 0) {
            std::lock_guard<std::mutex> clock(ctx.color_mutex);
            int cw = colorFrame.width(), ch = colorFrame.height();
            ctx.color.rgb.resize(cw * ch * 3);
            colorFrame.copy_to(reinterpret_cast<astra::RgbPixel*>(ctx.color.rgb.data()));
            ctx.color.width = cw;
            ctx.color.height = ch;
            ctx.color.valid = true;
            ctx.have_color = true;
        }
    }

    if (w != DEPTH_WIDTH || h != DEPTH_HEIGHT) {
        frame.depth_width = w;
        frame.depth_height = h;
    }

    // int16_t 深度 -> uint16_t depth_map (0=无效, 与模拟模式语义一致; FIX-10)
    std::vector<uint16_t> depth_map(depth.size());
    for (size_t i = 0; i < depth.size(); ++i) {
        int16_t v = depth[i];
        depth_map[i] = (v >= static_cast<int16_t>(MIN_VALID_DISTANCE_MM) &&
                        v <= static_cast<int16_t>(MAX_VALID_DISTANCE_MM))
                           ? static_cast<uint16_t>(v) : 0;
    }
    frame.depth_map = std::move(depth_map);

    // 区域分析
    frame.center_region = analyze_region(frame.depth_map, frame.depth_width,
                                         frame.depth_height, "center");
    frame.left_region   = analyze_region(frame.depth_map, frame.depth_width,
                                         frame.depth_height, "left");
    frame.right_region  = analyze_region(frame.depth_map, frame.depth_width,
                                         frame.depth_height, "right");

    // 环境判定: 深度图无效像素比例代理
    frame.ambient_light_level = estimate_ambient_light(frame.depth_map,
                                                       frame.depth_width,
                                                       frame.depth_height);
    frame.environment = classify_environment(frame.ambient_light_level);

    // 彩色帧缓存补充距离信息 (从深度帧中央区域算)
    {
        std::lock_guard<std::mutex> clock(ctx.color_mutex);
        if (frame.center_region.valid_pixel_ratio > 0) {
            ctx.color.center_distance_m = frame.center_region.center_distance_m;
            ctx.color.nearest_distance_m = frame.center_region.min_distance_m;
        }
    }

    return frame;
}

// 获取最新彩色帧 (RGB888) + 中央区域平均距离/最近障碍
ColorFrameData AstraProDriver::get_color_frame() {
    ColorFrameData out;
    if (use_simulated_) {
        return out;  // 模拟模式无彩色数据
    }
    auto& ctx = real_ctx();
    std::lock_guard<std::mutex> guard(ctx.color_mutex);
    if (!ctx.have_color) return out;
    out = ctx.color;
    return out;
}

// 预初始化硬件: 在 main 纺程调用 (窗口线程/采集线程启动前)
void AstraProDriver::init_hardware() {
    if (use_simulated_) return;
    auto& ctx = real_ctx();
    ensure_real_streams(ctx);
    std::cout << "[Astra] 硬件预初始化完成" << std::endl;
}
#endif // USE_ASTRA_SDK

#ifndef USE_ASTRA_SDK
// C1 修复: 非 SDK 构建 (Windows 模拟模式) 空实现, 消除 LNK2019
// (头文件无条件声明 + main.cpp:345/398 无条件调用; 模拟模式本就不产生真机数据)
void AstraProDriver::init_hardware() {}
ColorFrameData AstraProDriver::get_color_frame() { return {}; }
#endif

AstraFrame AstraProDriver::get_latest_frame() const {
    std::lock_guard<std::mutex> guard(lock_);
    return latest_frame_;
}

// ========== 模拟模式 ==========
AstraFrame AstraProDriver::simulate_frame() {
    AstraFrame frame;
    frame.timestamp = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    frame.depth_width  = DEPTH_WIDTH;
    frame.depth_height = DEPTH_HEIGHT;
    frame.valid = true;

    // 生成模拟深度图 (640x480): 前方中央 2.0~5.0m 障碍
    frame.depth_map.resize(DEPTH_WIDTH * DEPTH_HEIGHT, 0);
    for (int y = 0; y < DEPTH_HEIGHT; ++y) {
        for (int x = 0; x < DEPTH_WIDTH; ++x) {
            double dx = (x - DEPTH_WIDTH / 2.0) / (DEPTH_WIDTH / 2.0);
            double dy = (y - DEPTH_HEIGHT / 2.0) / (DEPTH_HEIGHT / 2.0);
            double dist = 2.5 + 2.5 * std::abs(std::sin(dx * 3.0 + dy * 2.0));
            frame.depth_map[y * DEPTH_WIDTH + x] = static_cast<uint16_t>(dist * 1000.0);
        }
    }

    // 环境: 置 UNKNOWN, 让 determine_environment() 走红外模拟值 fallback,
    // 使室内/半室内/室外三档在模拟中均可验证 (FIX-2)
    frame.environment = EnvironmentType::UNKNOWN;
    frame.ambient_light_level = 0.1;

    // 区域分析
    frame.center_region = analyze_region(frame.depth_map, DEPTH_WIDTH, DEPTH_HEIGHT, "center");
    frame.left_region   = analyze_region(frame.depth_map, DEPTH_WIDTH, DEPTH_HEIGHT, "left");
    frame.right_region  = analyze_region(frame.depth_map, DEPTH_WIDTH, DEPTH_HEIGHT, "right");

    return frame;
}

// ========== 深度区域分析 ==========
DepthRegion AstraProDriver::analyze_region(const std::vector<uint16_t>& depth_map,
                                           int width, int height,
                                           const std::string& region) {
    DepthRegion reg;
    if (depth_map.empty() || width <= 0 || height <= 0) return reg;

    int x0, x1, y0, y1;
    if (region == "center") {
        x0 = width / 4; x1 = width * 3 / 4;
        y0 = height / 4; y1 = height * 3 / 4;
    } else if (region == "left") {
        x0 = 0; x1 = width / 4;
        y0 = height / 4; y1 = height * 3 / 4;
    } else if (region == "right") {
        x0 = width * 3 / 4; x1 = width;
        y0 = height / 4; y1 = height * 3 / 4;
    } else {
        x0 = 0; x1 = width; y0 = 0; y1 = height;
    }

    std::vector<double> valid;
    double sum = 0.0;
    int valid_count = 0;
    double min_v = MAX_VALID_DISTANCE_MM, max_v = 0.0;
    int total = (x1 - x0) * (y1 - y0);

    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            uint16_t v = depth_map[y * width + x];
            if (v >= MIN_VALID_DISTANCE_MM && v <= MAX_VALID_DISTANCE_MM) {
                valid.push_back(v);
                sum += v;
                ++valid_count;
                if (v < min_v) min_v = v;
                if (v > max_v) max_v = v;
            }
        }
    }

    reg.valid_pixel_ratio = total > 0 ? (double)valid_count / total : 0.0;
    reg.center_distance_m = valid_count > 0 ? (sum / valid_count) / 1000.0 : 0.0;
    reg.min_distance_m    = valid_count > 0 ? min_v / 1000.0 : MAX_VALID_DISTANCE_MM / 1000.0;
    reg.max_distance_m    = valid_count > 0 ? max_v / 1000.0 : 0.0;
    reg.obstacle_count    = valid_count;
    reg.quality_score     = calc_quality(valid);

    return reg;
}

double AstraProDriver::calc_quality(const std::vector<double>& valid_values) {
    if (valid_values.empty()) return 0.0;
    // 质量 = 有效像素比例 (相对满帧)
    return std::min(1.0, (double)valid_values.size() / (DEPTH_WIDTH * DEPTH_HEIGHT * 0.5));
}

double AstraProDriver::estimate_ambient_light(const std::vector<uint16_t>& depth_map,
                                              int width, int height) {
    if (depth_map.empty() || width <= 0 || height <= 0) return 0.0;
    int valid = 0;
    for (size_t i = 0; i < depth_map.size(); ++i) {
        if (depth_map[i] >= MIN_VALID_DISTANCE_MM && depth_map[i] <= MAX_VALID_DISTANCE_MM) {
            ++valid;
        }
    }
    double ratio = (double)valid / depth_map.size();
    // 无效像素多 -> 光线强(室外) 或 太近; 有效像素多 -> 室内
    return std::min(1.0, std::max(0.0, 1.0 - ratio));
}

EnvironmentType AstraProDriver::classify_environment(double light_level) {
    if (light_level < 0.3) return EnvironmentType::INDOOR;
    if (light_level < 0.7) return EnvironmentType::SEMI_INDOOR;
    return EnvironmentType::OUTDOOR;
}

} // namespace mechdog
