/**
 * 环境红外强度传感器驱动模块 (TSL2591)
 *
 * 用途: 检测环境红外辐射强度, 用于调整 深度相机 vs 超声波 的融合权重。
 * 背景: Astra Pro 是单目结构光, 阳光/强红外会淹没其投射的红外图案,
 *       导致深度失效。TSL2591 的 infrared 通道值直接度量环境红外强度。
 *
 * 硬件: TSL2591 模块 (I2C 地址 0x29, 板载 3.3V 稳压)
 *   VIN -> 树莓派 Pin 1 (3.3V)
 *   GND -> Pin 6
 *   SDA -> Pin 3 (GPIO2)
 *   SCL -> Pin 5 (GPIO3)
 *
 * 两种模式:
 *   - 模拟模式 (use_simulated=true, 默认): 返回模拟红外值, 任意平台可编译, 用于融合逻辑验证
 *   - 真机模式 (use_simulated=false): 通过 /dev/i2c-1 读取 TSL2591 infrared 通道 (仅 Linux)
 */
#pragma once

#include <cstdint>
#include <random>

namespace mechdog {

/**
 * TSL2591 环境红外传感器驱动
 *
 * 使用方式:
 *   InfraRedSensor ir(true);              // 模拟模式
 *   double light = ir.read_normalized_light();   // 0.0(暗) ~ 1.0(强红外)
 */
class InfraRedSensor {
public:
    static constexpr uint8_t I2C_ADDR      = 0x29;
#ifdef __linux__
    static constexpr const char* I2C_DEV   = "/dev/i2c-1";
#endif

    // 红外通道读数归一化用的参考上限 (FIX-4);
    // 25x 增益下 TSL2591 红外通道实测量程: 室内 ~200-2000, 窗边 ~2000-8000, 太阳直射 >20000
    // 分母取 20000 与 config.h 阈值 (0.10/0.40) 同数量级: 室内≈0.01-0.1, 窗边≈0.1-0.4,
    // 太阳直射≈1.0, 三档边界不再错位 (R-2)
    // 最终值待硬件组按 docs/IR_CALIBRATION.md 标定后定。
    static constexpr uint16_t IR_MAX_REF   = 20000;

    explicit InfraRedSensor(bool use_simulated = true);
    ~InfraRedSensor();

    /** 读取归一化环境红外强度: 0.0(暗) ~ 1.0(强红外); 返回 -1.0 表示读取失败 */
    double read_normalized_light();

    /** 是否处于模拟模式 (真机时环境判定应跳过红外, 避免模拟随机值污染) */
    bool is_simulated() const { return use_simulated_; }

    // ALG-4 (v2.2): 真实硬件是否可用 —— 非模拟 且 未发生硬件初始化失败。
    // 区别于 is_simulated(): 真机 TSL2591 初始化失败时置 hw_unavailable_=true
    // (不再静默回退模拟), 使 read_normalized_light() 返回 -1 走深度代理/室外,
    // 而非返回模拟随机值污染环境判定。
    bool is_real_available() const { return !use_simulated_ && !hw_unavailable_; }

private:
    // 单元测试访问 (tests/test_fusion.cpp 专用, 注入 hw_unavailable_ 状态做回归)
    friend class InfraRedTestAccess;

    bool use_simulated_;
    // ALG-4: 真机硬件初始化失败标志 (仅 Linux 真机模式可置 true; 模拟模式恒 false)
    bool hw_unavailable_ = false;

    // 模拟模式 RNG
    std::mt19937 rng_;

    double read_ir_raw();       // 返回 0~65535 红外通道原始值 (仅真机)
    double simulate_ir();       // 模拟: 0.0 ~ 1.0

#ifdef __linux__
    int  i2c_fd_ = -1;
    bool i2c_open();
    void i2c_close();
    bool tsl2591_init();
    int tsl2591_read_ir();      // 读 CHAN1 (infrared) 通道; 返回 -1 表示失败
    bool i2c_write_reg(uint8_t reg, uint8_t value);
    bool i2c_read_reg(uint8_t reg, uint8_t* out, uint8_t len);
#endif
};

} // namespace mechdog
