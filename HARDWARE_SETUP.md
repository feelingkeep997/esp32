# ESP32 30P + MPU6500 硬件连接指南

## 硬件清单

- **开发板**: ESP32 30P (ESP32-S3 系列)
- **IMU 模块**: MPU6500 (MPU-6500)
- **电源管理**: ME6118A33PG (3.3V 稳压器)

## SPI 引脚连接

### ESP32-S3 (ESP32 30P) → MPU6500

| ESP32-S3 Pin | MPU6500 Pin | Function |
|--------------|-------------|----------|
| GPIO 12      | SCK         | SPI Clock |
| GPIO 13      | MISO        | Master In Slave Out |
| GPIO 11      | MOSI        | Master Out Slave In |
| GPIO 10      | CS          | Chip Select |
| 3.3V         | VCC         | Power Supply |
| GND          | GND         | Ground |

**注意**: ESP32-S3 的默认 FSPI 引脚：
- SCK: GPIO 12
- MISO: GPIO 13
- MOSI: GPIO 11
- SS/CS: GPIO 10

## 其他传感器连接

### RC 接收机 (SBUS/CRSF)

| ESP32-S3 Pin | RC Receiver | Function |
|--------------|-------------|----------|
| GPIO 8       | Signal      | RC Signal Input |
| 3.3V         | VCC         | Power (if needed) |
| GND          | GND         | Ground |

### 电池电压检测

| ESP32-S3 Pin | Battery Module | Function |
|--------------|----------------|----------|
| GPIO 1       | ADC Input      | Voltage Divider Output |

### LED 状态灯

| ESP32-S3 Pin | LED | Function |
|--------------|-----|----------|
| GPIO 2       | Anode (+) | Status LED |
| GND          | Cathode (-) | Ground |

## ME6118A33PG 电源管理芯片

ME6118A33PG 是一款 3.3V 低压差线性稳压器 (LDO)：

```
Input (VIN)     → 锂电池正极 (3.7V-4.2V)
Output (VOUT)   → ESP32-S3 3.3V 电源
GND             → 地
```

## 电机驱动连接

ESP32-S3 PWM 引脚（通过 LEDC 控制器）：

| Motor | ESP32-S3 GPIO | Function |
|-------|---------------|----------|
| Rear Left | GPIO 4 | PWM Control |
| Rear Right | GPIO 5 | PWM Control |
| Front Right | GPIO 6 | PWM Control |
| Front Left | GPIO 7 | PWM Control |

## I2C 扩展传感器 (可选)

如果需要连接扩展传感器（如 BMP388、VL53L1X 等）：

| ESP32-S3 Pin | I2C Function |
|--------------|--------------|
| GPIO 35      | SDA (Data) |
| GPIO 36      | SCL (Clock) |

## 完整接线示意图

```
┌─────────────────────┐
│   ESP32 30P Board   │
│                     │
│  ┌─────────────┐    │
│  │   ESP32-S3  │    │
│  │             │    │
│  │ GPIO12 ────┼────┼─→ MPU6500 SCK
│  │ GPIO13 ────┼────┼─→ MPU6500 MISO
│  │ GPIO11 ────┼────┼─→ MPU6500 MOSI
│  │ GPIO10 ────┼────┼─→ MPU6500 CS
│  │ GPIO 8 ────┼────┼─→ RC Receiver SIG
│  │ GPIO 1 ────┼────┼─→ Battery ADC
│  │ GPIO 2 ────┼────┼─→ LED
│  │ GPIO 4-7 ──┼────┼─→ Motor PWM
│  │ GPIO 35 ───┼────┼─→ I2C SDA
│  │ GPIO 36 ───┼────┼─→ I2C SCL
│  │             │    │
│  │ 3.3V ──────┼────┼─→ MPU6500 VCC
│  │ GND  ──────┼────┼─→ All GND
│  └─────────────┘    │
└─────────────────────┘
```

## 编译和烧录

### 1. 设置目标芯片

```bash
cd /D/esp32/esp32project/CF-Drone-main
idf.py set-target esp32s3
```

### 2. 清理构建缓存

```bash
idf.py fullclean
```

### 3. 重新编译

```bash
idf.py build
```

### 4. 烧录固件

```bash
idf.py -p /dev/ttyACM0 flash monitor
```

**注意**: ESP32-S3 通常使用 `/dev/ttyACM0` 而不是 `/dev/ttyUSB0`

## 故障排查

### 1. MPU6500 检测失败

如果串口输出显示 `MPU6500 not found! Expected 0x71 or 0x70, got 0xXX`：

- 检查 SPI 接线是否正确
- 确认电源电压为 3.3V
- 尝试降低 SPI 时钟频率（修改 mpu6500.cpp 中的 clock_speed_hz）
- 检查 CS 引脚电平（空闲时应为高）

### 2. WiFi 无法工作

ESP32-S3 需要正确的 PSRAM 配置：
- 确保 `sdkconfig.defaults` 中启用了 `CONFIG_SPIRAM=y`
- 确认开发板确实有 PSRAM

### 3. ADC 读数异常

电池电压检测可能不准确：
- 检查外部分压电阻比例
- 校准 ADC 偏移
- 增加采样次数进行平均

### 4. 编译错误

如果遇到组件找不到的错误：
```bash
# 清理构建目录
rm -rf build/

# 重新配置
idf.py reconfigure

# 重新编译
idf.py build
```

## 调试命令

上电后通过串口终端（115200 baud）连接：

```
imu           # 查看 IMU 数据
ca            # 校准加速度计
cr            # 校准遥控器
sys           # 系统信息
```

## 参考资料

- [ESP32-S3 数据手册](https://www.espressif.com.cn/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)
- [MPU6500 数据手册](https://invensense.tdk.com/wp-content/uploads/2024/01/PS-MPU-6500-datasheet1.pdf)
- [ESP-IDF SPI Master 文档](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripheral/spi_master.html)
