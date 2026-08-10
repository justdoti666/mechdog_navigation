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

    // 红外通道读数 (0~65535) 归一化用的参考上限;
    // TSL2591 在 25x 增益下, 室内 ~200-2000, 窗边 ~2000-8000, 太阳直射 >20000
    // 具体阈值由硬件组按 docs/IR_CALIBRATION.md 标定后填入 config.h
    static constexpr uint16_t IR_MAX_REF   = 65535;

    explicit InfraRedSensor(bool use_simulated = true);
    ~InfraRedSensor();

    /** 读取归一化环境红外强度: 0.0(暗) ~ 1.0(强红外); 返回 -1.0 表示读取失败 */
    double read_normalized_light();

    /** 是否处于模拟模式 (真机时环境判定应跳过红外, 避免模拟随机值污染) */
    bool is_simulated() const { return use_simulated_; }

private:
    bool use_simulated_;

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
