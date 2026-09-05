/**
 * logger.h — 简单分级日志系统 (header-only, 零依赖)
 *
 * 用途: 替代 main.cpp 里零散的 std::cout 打印, 提供
 *   分级 (DEBUG/INFO/WARN/ERROR) + 时间戳 + 可选写文件 + 控制台输出。
 * 风格: 与本项目一致 (header-only, 仅标准库, 无第三方, 多平台)。
 *
 * 用法:
 *   #include "logger.h"
 *   mechdog::log::set_level(mechdog::log::Level::Info);   // 可选, 默认 Info
 *   mechdog::log::set_log_file("run.log");                 // 可选, 写文件
 *
 *   LOG_DEBUG("dbg x=" << x);
 *   LOG_INFO("frame=" << seq << " occ=" << occ);
 *   LOG_WARN("sensor invalid");
 *   LOG_ERROR("fatal: " << err);
 *
 * 宏自动附加 [时间戳] [级别] [文件名:行号]。头文件内定义 static 单例,
 * 多翻译单元共用同一份状态 (C++17 inline, 保证单实例)。
 *
 * 依赖: 仅 <string>, <fstream>, <iostream>, <chrono>, <mutex>, <sstream>, <ctime>.
 */
#pragma once

#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

namespace mechdog {
namespace log {

// ============================================================
// 日志级别 (命名避开 Windows 宏: ERROR/WARN/DEBUG 是全局宏)
// ============================================================
enum class Level {
    Debug = 0,
    Info = 1,
    Warn = 2,
    Error = 3,
    None = 4,   // 关闭全部日志
};

inline const char* level_str(Level lv) {
    switch (lv) {
        case Level::Debug: return "DEBUG";
        case Level::Info:  return "INFO";
        case Level::Warn:  return "WARN";
        case Level::Error: return "ERROR";
        default:           return "NONE";
    }
}

// ============================================================
// Logger — header-only 单例
// ============================================================
class Logger {
public:
    static Logger& instance() {
        static Logger inst;   // C++11 线程安全局部静态
        return inst;
    }

    // 配置
    void set_level(Level lv) { std::lock_guard<std::mutex> lk(mtx_); level_ = lv; }
    Level level() const { return level_; }

    // 打开/关闭写文件; 返回 false = 文件打开失败
    bool set_log_file(const std::string& path) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (ofs_.is_open()) ofs_.close();
        ofs_.open(path, std::ios::out | std::ios::app);
        return ofs_.is_open();
    }
    void close_log_file() {
        std::lock_guard<std::mutex> lk(mtx_);
        if (ofs_.is_open()) ofs_.close();
    }
    bool file_enabled() const { return ofs_.is_open(); }

    // 核心: 输出一条带级别/时间戳/位置的日志
    void write(Level lv, const char* file, int line, const std::string& msg) {
        if (static_cast<int>(lv) < static_cast<int>(level_)) return;  // 低于级别不输出

        std::lock_guard<std::mutex> lk(mtx_);
        const std::string ts = timestamp();
        // 只取文件名不含路径
        const std::string fn = file ? basename(file) : "?";
        std::ostringstream oss;
        oss << "[" << ts << "]" << "[" << level_str(lv) << "]"
            << "[" << fn << ":" << line << "] " << msg;

        const std::string line_out = oss.str();
        std::cout << line_out << std::endl;
        if (ofs_.is_open()) ofs_ << line_out << std::endl;
    }

private:
    Logger() = default;
    ~Logger() { if (ofs_.is_open()) ofs_.close(); }
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    static std::string timestamp() {
        using namespace std::chrono;
        const auto now = system_clock::now();
        const std::time_t t = system_clock::to_time_t(now);
        char buf[64];
        std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
        return std::string(buf);
    }

    static std::string basename(const std::string& p) {
        const size_t pos = p.find_last_of("/\\");
        return pos == std::string::npos ? p : p.substr(pos + 1);
    }

    Level level_ = Level::Info;
    std::mutex mtx_;
    std::ofstream ofs_;
};

// ============================================================
// 便捷接口
// ============================================================
inline void set_level(Level lv) { Logger::instance().set_level(lv); }
inline bool set_log_file(const std::string& path) { return Logger::instance().set_log_file(path); }
inline void close_log_file() { Logger::instance().close_log_file(); }

} // namespace log
} // namespace mechdog

// ============================================================
// 日志宏 (自动带文件名:行号)
// ============================================================
#define LOG_DEBUG(msg) \
    do { std::ostringstream _oss; _oss << msg; \
         ::mechdog::log::Logger::instance().write(::mechdog::log::Level::Debug, __FILE__, __LINE__, _oss.str()); } while (0)
#define LOG_INFO(msg) \
    do { std::ostringstream _oss; _oss << msg; \
         ::mechdog::log::Logger::instance().write(::mechdog::log::Level::Info, __FILE__, __LINE__, _oss.str()); } while (0)
#define LOG_WARN(msg) \
    do { std::ostringstream _oss; _oss << msg; \
         ::mechdog::log::Logger::instance().write(::mechdog::log::Level::Warn, __FILE__, __LINE__, _oss.str()); } while (0)
#define LOG_ERROR(msg) \
    do { std::ostringstream _oss; _oss << msg; \
         ::mechdog::log::Logger::instance().write(::mechdog::log::Level::Error, __FILE__, __LINE__, _oss.str()); } while (0)
