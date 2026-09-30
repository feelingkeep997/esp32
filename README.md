# CF-Drone - ESP-IDF Flight Controller

基于 ESP32 的穿越机飞控固件，使用 ESP-IDF 开发。

## 项目结构

```
CF-Drone-main/
├── main/                    # 主应用程序入口
│   ├── main.cpp            # app_main() 和主循环
│   ├── parameters.cpp      # NVS 参数存储
│   ├── time.cpp            # 时间管理
│   └── util.cpp            # 工具函数
├── components/
│   ├── cf_common/          # 通用头文件和数学库
│   │   ├── globals.h       # 全局变量声明
│   │   ├── vector.h        # 向量数学
│   │   ├── quaternion.h    # 四元数数学
│   │   ├── pid.h           # PID 控制器
│   │   ├── lpf.h           # 低通滤波器
│   │   └── board_config.h  # 板级配置
│   ├── flight/             # 飞行控制
│   │   ├── control.cpp     # 姿态和高度控制
│   │   ├── estimate.cpp    # 姿态估计
│   │   ├── motors.cpp      # 电机输出
│   │   ├── led.cpp         # LED 状态灯
│   │   └── safety.cpp      # 安全保护
│   ├── sensors/            # 传感器驱动
│   │   ├── imu.cpp         # MPU9250 IMU
│   │   ├── battery.cpp     # 电池电压检测
│   │   └── rc.cpp          # RC 接收机 (SBUS/CRSF)
│   ├── drivers/            # 硬件驱动
│   │   └── mpu9250.cpp     # MPU9250 SPI 驱动
│   └── comms/              # 通信模块
│       ├── cli.cpp         # 命令行接口
│       ├── wifi.cpp        # WiFi 网络
│       ├── mavlink.cpp     # MAVLink 协议
│       ├── web_rc.cpp      # Web 遥控器
│       └── log.cpp         # 数据日志
├── CMakeLists.txt          # 项目构建配置
├── sdkconfig.defaults      # ESP-IDF 默认配置
└── partitions.csv          # Flash 分区表
```

## 支持的硬件

- **ESP32** (经典双核) - 默认配置
- **ESP32-C3** - WiFi 禁用模式
- **ESP32-S3** - 全功能配置

## 主要功能

### 飞行模式
- RAW - 手动模式（无辅助）
- ACRO - 自稳模式
- STAB - 定高模式
- ALTHOLD - 自动定高（无气压计近似）

### 传感器
- MPU9250 IMU（陀螺仪 + 加速度计）
- SBUS/CRSF 遥控接收
- 电池电压监测

### 通信
- UART 命令行界面 (115200 baud)
- WiFi AP/STA 模式
- MAVLink 遥测
- Web 遥控器

## 编译环境

### ESP-IDF 要求
- ESP-IDF v4.4 或更高版本
- Xtensa toolchain (ESP32)
- RISC-V toolchain (ESP32-C3/S3)

### 安装 ESP-IDF

```bash
git clone --recursive https://github.com/espressif/esp-idf.git --branch v5.1
cd esp-idf
./install.sh esp32
source export.sh
```

### 编译项目

```bash
cd /D/esp32/esp32project/CF-Drone-main

# 对于 ESP32 (默认)
idf.py set-target esp32
idf.py build

# 对于 ESP32-C3
idf.py set-target esp32c3
idf.py build

# 对于 ESP32-S3
idf.py set-target esp32s3
idf.py build
```

### 烧录固件

```bash
# ESP32
idf.py -p /dev/ttyUSB0 flash monitor

# ESP32-C3
idf.py -p /dev/ttyACM0 flash monitor

# ESP32-S3
idf.py -p /dev/ttyACM0 flash monitor
```

## 命令行接口

上电后通过串口终端连接（115200 baud），可用命令：

```
help          - 显示帮助信息
p             - 显示所有参数
p <name>      - 显示指定参数
p <name> <v>  - 设置参数值
preset        - 重置所有参数
arm           - 解锁无人机
disarm        - 锁定无人机
raw/stab/acro/auto - 切换飞行模式
ca            - 校准加速度计
cr            - 校准遥控器
imu           - 显示 IMU 信息
log [dump]    - 查看日志
reboot        - 重启系统
sys           - 显示系统信息
```

## 引脚配置

### ESP32 (默认)
- **电机**: GPIO 12, 13, 15, 14
- **IMU SPI**: SCK=18, MISO=19, MOSI=23, CS=5
- **RC UART**: RX=GPIO 4, TX=-1
- **电池 ADC**: GPIO 36
- **LED**: GPIO 2

### ESP32-C3
- **电机**: GPIO 3, 10, 0, 1
- **IMU SPI**: SCK=4, MISO=5, MOSI=6, CS=7
- **RC UART**: RX=GPIO 9
- **电池 ADC**: GPIO 2
- **LED**: GPIO 8

### ESP32-S3
- **电机**: GPIO 4, 5, 6, 7
- **IMU SPI**: SCK=12, MISO=13, MOSI=11, CS=10
- **RC UART**: RX=GPIO 8
- **电池 ADC**: GPIO 1
- **LED**: GPIO 2

## 注意事项

1. **无气压计**：ALTHOLD 模式使用垂直速度积分近似定高，会随时间漂移
2. **WiFi 性能**：ESP32-C3 单核模式下 WiFi 会占用 CPU，建议关闭
3. **Flash 分区**：使用自定义分区表，预留空间存储参数和日志
4. **ADC 精度**：电池电压使用 ESP32 内部 ADC，需要外部分压电路

## 开发指南

### 添加新组件

```bash
mkdir components/new_component
touch components/new_component/CMakeLists.txt
touch components/new_component/new_component.cpp
```

在 `CMakeLists.txt` 中添加：

```cmake
idf_component_register(
    SRCS "new_component.cpp"
    INCLUDE_DIRS "."
    REQUIRES cf_common
)
```

### 调试技巧

1. 启用打印日志：修改 `board_config.h` 中的打印级别
2. 使用 JTAG 调试：配置 OpenOCD
3. 查看 FreeRTOS 任务：通过 CLI 命令 `sys`

## 许可证

本项目遵循 MIT 许可证。详见 LICENSE 文件。

## 致谢

- ESP32 by Espressif Systems
- Crazyflie firmware inspiration
- ESP-IDF development framework
