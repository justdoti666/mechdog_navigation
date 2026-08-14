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

// 全局共享上下文: StreamSet/Reader + 回调监听
class SampleListener;  // 前向声明 (成员指针)
struct RealAstraContext {
    bool inited = false;
    astra::StreamSet streamSet;
    astra::StreamReader reader;
    std::thread update_thread;
    std::atomic<bool> running{false};
    std::atomic<bool> streams_ready{false};

    // 帧数据 (由回调线程写, 由 capture 线程读; 用各自锁)
    std::mutex frame_mutex;
    std::vector<int16_t> depth_buf;       // 最新深度 (int16 mm, Astra SDK 原生类型, FIX-10)
    int depth_w = 0, depth_h = 0;
    ColorFrameData color;                 // 最新彩色 + 距离
    bool have_depth = false, have_color = false;

    // FrameListener 长期存活 (SDK 持有其指针, 局部变量析构会导致悬垂崩溃)
    std::unique_ptr<SampleListener> listener;

    RealAstraContext() {
        astra::initialize();
        reader = streamSet.create_reader();
        inited = true;
    }
};

// FrameListener: 帧就绪回调 (SDK 内部线程调用)
class SampleListener : public astra::FrameListener {
public:
    explicit SampleListener(RealAstraContext& ctx) : ctx_(ctx) {}
    virtual void on_frame_ready(astra::StreamReader& reader, astra::Frame& frame) override {
        (void)reader;
        auto depth = frame.get<astra::DepthFrame>();
        auto color = frame.get<astra::ColorFrame>();

        std::lock_guard<std::mutex> guard(ctx_.frame_mutex);

        if (depth.is_valid() && depth.width() > 0) {
            int w = depth.width(), h = depth.height();
            ctx_.depth_buf.resize(w * h);
            depth.copy_to(ctx_.depth_buf.data());   // FIX-10: 类型已为 int16_t, 去 reinterpret_cast
            ctx_.depth_w = w;
            ctx_.depth_h = h;
            ctx_.have_depth = true;

            // 中央区域平均距离/最近障碍 (每帧回调算一次, 避免 get_color_frame 重复遍历)
            double min_mm = -1.0, sum = 0.0;
            int n = 0;
            int cx0 = w / 4, cx1 = w * 3 / 4;
            int cy0 = h / 4, cy1 = h * 3 / 4;
            for (int y = cy0; y < cy1; ++y) {
                for (int x = cx0; x < cx1; ++x) {
                    int16_t v = ctx_.depth_buf[y * w + x];  // FIX-10: int16_t 原生读取
                    if (v >= AstraProDriver::MIN_VALID_DISTANCE_MM &&
                        v <= AstraProDriver::MAX_VALID_DISTANCE_MM) {
                        sum += v; ++n;
                        if (min_mm < 0 || v < min_mm) min_mm = v;
                    }
                }
            }
            ctx_.color.center_distance_m = (n > 0) ? (sum / n) / 1000.0 : -1.0;
            ctx_.color.nearest_distance_m = (min_mm > 0) ? min_mm / 1000.0 : -1.0;
        }
        if (color.is_valid() && color.width() > 0) {
            int w = color.width(), h = color.height();
            ctx_.color.rgb.resize(w * h * 3);
            color.copy_to(reinterpret_cast<astra::RgbPixel*>(ctx_.color.rgb.data()));
            ctx_.color.width = w;
            ctx_.color.height = h;
            ctx_.color.valid = true;
            ctx_.have_color = true;
        }
    }

private:
    RealAstraContext& ctx_;
};

RealAstraContext& real_ctx() {
    static RealAstraContext ctx;
    return ctx;
}

// 启动双流 + update 线程 (仅一次)
void ensure_real_streams(RealAstraContext& ctx) {
    if (ctx.streams_ready.load()) return;

    // listener 必须长期存活 (SDK 持有指针, 局部对象会悬垂崩溃)
    ctx.listener = std::make_unique<SampleListener>(ctx);
    ctx.reader.add_listener(*ctx.listener);

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

    // 独立线程持续 astra_update() 驱动回调
    ctx.running = true;
    ctx.update_thread = std::thread([&ctx]() {
        while (ctx.running) {
            astra_update();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });
}

} // namespace

AstraFrame AstraProDriver::capture_real() {
    auto& ctx = real_ctx();
    ensure_real_streams(ctx);

    AstraFrame frame;
    frame.timestamp = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    frame.depth_width  = DEPTH_WIDTH;
    frame.depth_height = DEPTH_HEIGHT;

    // 从回调缓存取最新深度帧
    std::vector<int16_t> depth;   // FIX-10: 与 depth_buf 同类型
    int w = 0, h = 0;
    {
        std::lock_guard<std::mutex> guard(ctx.frame_mutex);
        if (!ctx.have_depth) {
            frame.valid = false;
            return frame;
        }
        depth = ctx.depth_buf;
        w = ctx.depth_w;
        h = ctx.depth_h;
        frame.valid = true;
    }
    if (w != DEPTH_WIDTH || h != DEPTH_HEIGHT) {
        // 实际分辨率可能与默认不同, 以实际为准
        frame.depth_width = w;
        frame.depth_height = h;
    }
    // int16_t 深度 (Astra SDK 原生) -> uint16_t depth_map (0=无效, 与模拟模式语义一致; FIX-10)
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

    // 环境判定: 深度图无效像素比例代理 (F3 决策: 默认 estimate_ambient_light)
    frame.ambient_light_level = estimate_ambient_light(frame.depth_map,
                                                       frame.depth_width,
                                                       frame.depth_height);
    frame.environment = classify_environment(frame.ambient_light_level);

    return frame;
}

// 获取最新彩色帧 (RGB888) + 中央区域平均距离/最近障碍 (从回调缓存)
ColorFrameData AstraProDriver::get_color_frame() {
    ColorFrameData out;
    if (use_simulated_) {
        return out;  // 模拟模式无彩色数据
    }
    auto& ctx = real_ctx();
    std::lock_guard<std::mutex> guard(ctx.frame_mutex);
    if (!ctx.have_color) return out;
    // 距离已在回调线程算好 (center_distance_m/nearest_distance_m), 直接浅拷贝返回
    out = ctx.color;
    return out;
}

// 预初始化硬件: 在 main 线程调用 (窗口线程/采集线程启动前)
// 确保深度+彩色双流在 main 线程 start, 避免采集线程里首次 start 阻塞
void AstraProDriver::init_hardware() {
    if (use_simulated_) return;
    auto& ctx = real_ctx();
    ensure_real_streams(ctx);
    std::cout << "[Astra] 硬件预初始化完成" << std::endl;
}
#endif // USE_ASTRA_SDK

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
