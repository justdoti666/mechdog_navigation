/**
 * 环境红外强度传感器驱动模块实现 (TSL2591)
 *
 * 真机 I2C 部分仅在 Linux 编译 (__linux__), 模拟模式跨平台 (Windows PC 也可编译)。
 */
#include "sensor_ir.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <thread>

#ifdef __linux__
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#endif

namespace mechdog {

// ============================================================
// TSL2591 寄存器定义 (真机模式使用)
// ============================================================
#ifdef __linux__
namespace {
constexpr uint8_t TSL2591_CMD_BIT    = 0x80;  // 命令寄存器位(必须置1)
constexpr uint8_t TSL2591_REG_ENABLE = 0x00;  // 使能寄存器
constexpr uint8_t TSL2591_REG_CONTROL= 0x01;  // 增益/积分时间
constexpr uint8_t TSL2591_REG_CHAN1   = 0x16; // CHAN1 (infrared) 低字节
constexpr uint8_t TSL2591_ENABLE_PON  = 0x01; // Power ON
constexpr uint8_t TSL2591_ENABLE_AEN  = 0x02; // ALS Enable
// CONTROL: 增益 25x (bit5:4=01), 积分 100ms (bit1:0=00) — 起步配置, 标定后可按需调整
constexpr uint8_t TSL2591_CTRL_25X_100MS = 0x10;
constexpr int     TSL2591_READ_REPEAT    = 3; // 滑动平均次数
} // namespace
#endif

InfraRedSensor::InfraRedSensor(bool use_simulated)
    : use_simulated_(use_simulated)
    , rng_(std::random_device{}()) {
#ifdef __linux__
    if (!use_simulated_) {
        if (i2c_open() && tsl2591_init()) {
            std::cout << "[IR] TSL2591 就绪 (I2C " << I2C_DEV << ", addr 0x"
                      << std::hex << static_cast<int>(I2C_ADDR) << std::dec << ")"
                      << std::endl;
        } else {
            // ALG-4 (v2.2): 不再静默回退模拟模式 (那样 read 会返回随机值污染环境判定,
            // 见 sensor_fusion.cpp determine_environment 的 Astra 无效分支)。
            // 改置 hw_unavailable_=true, read_normalized_light 返回 -1 -> 融合走深度代理/室外。
            std::cerr << "[IR] TSL2591 初始化失败, 标记 hw_unavailable (真机路径将返回 -1, 融合回退深度代理)"
                      << std::endl;
            hw_unavailable_ = true;
        }
    }
#else
    // 非 Linux 平台强制模拟模式
    use_simulated_ = true;
#endif
}

InfraRedSensor::~InfraRedSensor() {
#ifdef __linux__
    i2c_close();
#endif
}

double InfraRedSensor::read_normalized_light() {
    // 有意模拟 (sim 模式 / PC): 保留随机语义 (FIX-2 三档覆盖 + test_layer_fusion_boundaries 依赖)
    if (use_simulated_) {
        return simulate_ir();
    }
    // ALG-4 (v2.2): 真机硬件初始化失败 -> 返回故障值, 不返回随机值
    // (避免 fusion 的 Astra 无效分支消费随机 IR 污染环境判定)
    if (hw_unavailable_) {
        return -1.0;
    }
#ifdef __linux__
    // 修复D3: 读取失败返回 -1.0, 调用方应回退到安全侧(室外/超声波主导),
    // 而不是返回 0.0 被当成"室内"给深度相机最高权重
    double raw = read_ir_raw();
    if (raw < 0) return -1.0;
    double norm = raw / static_cast<double>(IR_MAX_REF);
    return std::clamp(norm, 0.0, 1.0);
#else
    // 非 Linux 且非模拟: 构造已强制 use_simulated_=true, 走不到此分支; 防御返回故障值
    return -1.0;
#endif
}

double InfraRedSensor::simulate_ir() {
    // 模拟红外强度: 0.0 ~ 1.0, 覆盖 室内/半室内/室外 三档
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    return std::round(dist(rng_) * 1000.0) / 1000.0;
}

#ifdef __linux__
double InfraRedSensor::read_ir_raw() {
    // 连续读 TSL2591_READ_REPEAT 次取平均, 抗瞬时抖动
    double sum = 0.0;
    for (int i = 0; i < TSL2591_READ_REPEAT; ++i) {
        double v = tsl2591_read_ir();
        if (v < 0) return -1.0;  // 任一读取失败, 整次判失败
        sum += v;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return sum / static_cast<double>(TSL2591_READ_REPEAT);
}

// ============================================================
// 真机 I2C 底层 (Linux i2c-dev)
// ============================================================
bool InfraRedSensor::i2c_open() {
    i2c_fd_ = ::open(I2C_DEV, O_RDWR);
    if (i2c_fd_ < 0) {
        std::cerr << "[IR] 无法打开 " << I2C_DEV << ": " << std::strerror(errno)
                  << std::endl;
        return false;
    }
    if (ioctl(i2c_fd_, I2C_SLAVE, I2C_ADDR) < 0) {
        std::cerr << "[IR] ioctl I2C_SLAVE 失败: " << std::strerror(errno)
                  << std::endl;
        i2c_close();
        return false;
    }
    return true;
}

void InfraRedSensor::i2c_close() {
    if (i2c_fd_ >= 0) {
        ::close(i2c_fd_);
        i2c_fd_ = -1;
    }
}

bool InfraRedSensor::tsl2591_init() {
    // 1. Power ON (寄存器写: 命令字节 = CMD|寄存器地址, 随后数据字节)
    if (!i2c_write_reg(TSL2591_REG_ENABLE, TSL2591_ENABLE_PON)) {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // 2. 使能 ALS + 设置增益/积分时间
    if (!i2c_write_reg(TSL2591_REG_ENABLE, TSL2591_ENABLE_PON | TSL2591_ENABLE_AEN)) {
        return false;
    }
    if (!i2c_write_reg(TSL2591_REG_CONTROL, TSL2591_CTRL_25X_100MS)) {
        return false;
    }
    return true;
}

int InfraRedSensor::tsl2591_read_ir() {
    uint8_t buf[2] = {0, 0};
    if (!i2c_read_reg(TSL2591_REG_CHAN1, buf, 2)) {
        return -1;  // 读取失败
    }
    return static_cast<int>(buf[1] << 8 | buf[0]);
}

// 底层: 用裸 ioctl(I2C_SMBUS) 实现, 不依赖 i2c_smbus_* 内联 helper (部分内核头文件版本不导出)
bool InfraRedSensor::i2c_write_reg(uint8_t reg, uint8_t value) {
    union i2c_smbus_data data;
    data.byte = value;
    struct i2c_smbus_ioctl_data args = {
        .read_write = I2C_SMBUS_WRITE,
        .command    = static_cast<uint8_t>(TSL2591_CMD_BIT | reg),
        .size       = I2C_SMBUS_BYTE_DATA,
        .data       = &data,
    };
    return ioctl(i2c_fd_, I2C_SMBUS, &args) >= 0;
}

bool InfraRedSensor::i2c_read_reg(uint8_t reg, uint8_t* out, uint8_t len) {
    union i2c_smbus_data data;
    struct i2c_smbus_ioctl_data args = {
        .read_write = I2C_SMBUS_READ,
        .command    = static_cast<uint8_t>(TSL2591_CMD_BIT | reg),
        .size       = I2C_SMBUS_I2C_BLOCK_DATA,
        .data       = &data,
    };
    data.block[0] = len;  // 要读的字节数
    if (ioctl(i2c_fd_, I2C_SMBUS, &args) < 0) {
        return false;
    }
    for (uint8_t i = 0; i < len; ++i) {
        out[i] = data.block[i + 1];
    }
    return true;
}
#endif // __linux__

} // namespace mechdog
