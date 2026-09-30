// ============================================================================
// mpu9250.h —— MPU9250/MPU6500 驱动接口与寄存器/量程定义
//
// 职责:
//   提供寄存器地址宏、量程（FSR）选项宏，以及 SPI 读写 / 配置 / 数据读取等接口声明。
//   供 imu.cpp 调用。
//
// 关键逻辑:
//   寄存器地址均为 7 位；SPI 传输时最高位区分读写（MSB=1 读，MSB=0 写）。
//   量程宏直接对应 GYRO_CONFIG / ACCEL_CONFIG 的低位编码，灵敏度 = 32768 / 量程。
//
// 输入/输出:
//   mpu9250_read_data() 输出 gyro(rad/s) 与 acc(m/s²)；
//   其余接口返回寄存器值或执行状态（部分为空实现）。
//
// 边界情况与潜在风险:
//   - 灵敏度换算（在 .cpp 中）必须与 configure 时选用的 FSR 宏保持一一对应。
//   - 本项目实际芯片为 MPU6500（WHO_AM_I=0x70），与 MPU9250 寄存器兼容，
//     但无内置磁力计（AK8963），相关寄存器/宏仅为兼容保留。
//   - mpu9250_set_rotation() 当前为空实现，安装旋转未在此层生效。
// ============================================================================

#pragma once

#include "vector.h"
#include "quaternion.h"
#include "board_config.h"

// 寄存器地址（均为 7 位地址；SPI 传输时最高位用于区分读/写：MSB=1 读，MSB=0 写）
#define MPU_REG_WHO_AM_I       0x75 // 芯片身份寄存器：只读，MPU9250 典型值 0x71（部分 0x73）
#define MPU_REG_PWR_MGMT_1     0x6B // 电源管理 1：选择时钟源、进入/退出睡眠模式
#define MPU_REG_SMPLRT_DIV     0x19 // 采样率分频：采样率 = 内部 1kHz / (值 + 1)
#define MPU_REG_CONFIG         0x1A // 配置寄存器：设置 DLPF（数字低通滤波器）带宽
#define MPU_REG_GYRO_CONFIG    0x1B // 陀螺仪配置：满量程范围（FSR）等
#define MPU_REG_ACCEL_CONFIG   0x1C // 加速度计配置：满量程范围（FSR）等
#define MPU_REG_FIFO_EN        0x23 // FIFO 使能：选择哪些传感器数据写入 FIFO
#define MPU_REG_INT_PIN_CFG    0x37 // 中断引脚配置；也含旁路 AK8963（磁力计）I2C 的位
#define MPU_REG_USER_CTRL      0x6A // 用户控制：FIFO / I2C 主机 / SPI 模式使能
#define MPU_REG_FIFO_COUNT     0x72 // FIFO 计数（高字节）：当前 FIFO 中的有效字节数
#define MPU_REG_FIFO_R_L       0x74 // FIFO 读数据寄存器
#define MPU_REG_ACCEL_XOUT_H   0x3B // 加速度计数据起始寄存器（X 高字节）：连续 6 字节为 X/Y/Z
#define MPU_REG_GYRO_XOUT_H    0x43 // 陀螺仪数据起始寄存器（X 高字节）：连续 6 字节为 X/Y/Z

// 陀螺仪满量程范围（FSR）选项 —— 写入 MPU_REG_GYRO_CONFIG 的低 2 位
// 灵敏度 = 32768 / 量程，用于把原始计数换算成 deg/s
#define MPU9250_GYRO_FSR_250   0x00  // +/- 250 deg/s  -> 131.0 LSB/(deg/s)
#define MPU9250_GYRO_FSR_500   0x08  // +/- 500 deg/s  -> 65.5  LSB/(deg/s)
#define MPU9250_GYRO_FSR_1000  0x10  // +/- 1000 deg/s -> 32.8  LSB/(deg/s)
#define MPU9250_GYRO_FSR_2000  0x18  // +/- 2000 deg/s -> 16.4  LSB/(deg/s)

// 加速度计满量程范围（FSR）选项 —— 写入 MPU_REG_ACCEL_CONFIG 的低 2 位
// 灵敏度 = 32768 / 量程，用于把原始计数换算成 g
#define MPU9250_ACCEL_FSR_2G   0x00  // +/- 2g  -> 16384 LSB/g
#define MPU9250_ACCEL_FSR_4G   0x08  // +/- 4g  -> 8192  LSB/g
#define MPU9250_ACCEL_FSR_8G   0x10  // +/- 8g  -> 4096  LSB/g
#define MPU9250_ACCEL_FSR_16G  0x18  // +/- 16g -> 2048  LSB/g

void mpu9250_init();                             // 初始化 SPI 总线与设备（CS/SCK/MOSI/MISO）并完成默认配置
uint8_t mpu9250_read_reg(uint8_t reg);           // 读单个寄存器：reg 的低 7 位为地址，返回该寄存器值
void mpu9250_write_reg(uint8_t reg, uint8_t value); // 写单个寄存器：把 value 写入地址为 reg 的寄存器
void mpu9250_read_data(Vector *gyro, Vector *acc);  // 读 accel/gyro 并换算：acc 单位 m/s^2，gyro 单位 rad/s
void mpu9250_configure(int gyroFSR, int accelFSR);  // 配置采样率、量程、DLPF 并唤醒设备（参数取上面的 FSR 宏）
void mpu9250_set_rotation(Quaternion rot);          // 设置"传感器坐标系 -> 机体坐标系"的安装旋转（当前为空实现）
