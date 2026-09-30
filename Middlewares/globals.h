// ============================================================================
// globals.h —— 全工程共享声明中心（跨模块变量与函数）
//
// 职责:
//   在 ESP-IDF 下每个 .cpp 都是独立编译单元（TU），模块间无法自动共享全局量。
//   本头文件集中声明所有跨模块使用的 extern 变量与函数原型，并包含公共基础头
//   （vector/quaternion/lpf/pid/board_config）。
//
// 关键逻辑:
//   按功能分组声明：时间与状态 / 控制 / 电机 / IMU / 姿态估计 / 遥控 / 电池 /
//   LED / 安全 / 日志 / 参数 / CLI / 时间 / 工具 / WiFi / MAVLink / Web RC / 遥测。
//   各分组的"定义"分散在对应 .cpp 中（如 controlRoll 定义在 main.cpp，
//   mode 定义在 control.cpp），此处仅做 extern 声明。
//
// 输入/输出:
//   纯声明，无运行时行为。包含顺序有讲究：必须在 board_config.h 之前使用其宏的
//   地方已在文件内处理（WIFI_ENABLED/WEB_RC_ENABLED 由 board_config.h 推导）。
//
// 边界情况与潜在风险:
//   - 这里是"全局可变状态"的集合，多任务（HTTP/ESP-NOW 回调）与主循环并发访问时
//     没有同步机制，属工程已知的设计取舍。
//   - 部分宏（如 ALTHOLD_HOVER_THRUST、VBAT_* 阈值）集中定义在此，改阈值时注意
//     是否与 safety.cpp / led.cpp 中的判断逻辑保持一致。
//   - extern 变量若无对应定义会导致链接错误；新增全局量须同时改本文件与定义处。
//   - WIFI_ENABLED / WEB_RC_ENABLED 未定义时回退到 board_config.h 的板级开关。
// ============================================================================

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "vector.h"       // Vector：三维向量
#include "quaternion.h"   // Quaternion：姿态四元数
#include "lpf.h"          // LowPassFilter / KalmanFilter：滤波器
#include "pid.h"          // PID：控制器
#include "board_config.h" // 板级引脚与功能开关配置

// 重力加速度常数（约 9.81 m/s²，定义在 util.cpp）
extern const float ONE_G;

// 悬停油门（0~1 推力标度）：失控保护 descend() 切入 AUTO 时的推力上限，
// 之后按 descendTime 线性递减到 0。原值 0.5f（50%），现改为 0.7f（70%）
#define ALTHOLD_HOVER_THRUST 0.7f

// ============================================================
// 功能开关（来自 board_config.h，编译期确定）
// ============================================================
#ifndef WIFI_ENABLED
#define WIFI_ENABLED BOARD_WIFI_ENABLED
#endif
#ifndef WEB_RC_ENABLED
#define WEB_RC_ENABLED BOARD_WEB_RC_ENABLED
#endif

// ============================================================
// 飞行模式常量（定义在 control.cpp）
// ============================================================
extern const int RAW, ACRO, STAB, ALTHOLD, AUTO;

// ============================================================
// 核心时间与状态（定义在 main.cpp，源自 CF-Drone.ino）
// ============================================================
extern float t;        // 当前步进时刻，单位秒（自启动以来的单调时间）
extern float dt;       // 距上一步的时间差，单位秒
extern float loopRate; // 主循环实际运行频率，单位 Hz

extern float controlRoll, controlPitch, controlYaw, controlThrottle; // 飞手输入，范围 [-1, 1]
extern float controlMode;         // 模式通道归一化值
extern Vector gyro;               // 陀螺仪角速度（rad/s，已校准）
extern Vector acc;                // 加速度计加速度（m/s²，已校准）
extern Vector rates;              // 低通滤波后的角速率，rad/s
extern Quaternion attitude;       // 估计出的姿态四元数
extern bool landed;               // 是否已落地且静止

// ============================================================
// 控制（定义在 control.cpp）
// ============================================================
extern int mode;                  // 当前飞行模式（RAW/ACRO/STAB/ALTHOLD/AUTO）
extern bool armed;                // 是否已解锁（true = 允许输出到电机）
extern int flightModes[3];        // 模式通道三档对应的飞行模式
extern PID rollRatePID, pitchRatePID, yawRatePID; // 角速率环 PID（内环）
extern PID rollPID, pitchPID, yawPID;             // 姿态环 PID（外环）
extern Vector maxRate;            // 各轴最大角速率，rad/s
extern float tiltMax;             // 自稳模式最大倾角，rad
extern Quaternion attitudeTarget; // 目标姿态
extern Vector ratesTarget;        // 目标角速率
extern Vector ratesExtra;         // 额外角速率指令（前馈项）
extern Vector torqueTarget;       // 目标力矩
extern float thrustTarget;        // 目标推力（0~1）
extern float hoverThrustEst; // ALTHOLD 学习到的悬停推力（NAN=未激活，定高时有效）
extern float altHoldP, altHoldI, altHoldRateMax, altHoldDeadband; // ALTHOLD 定高参数
extern float motThrMin;           // 电机最小推力对应的油门下限
extern float motThrMax;           // 电机最大推力对应的油门上限
extern float trimRoll;            // 横滚配平
extern float trimPitch;           // 俯仰配平

void control();               // 控制主入口（每个控制周期调用一次）
void interpretControls();     // 解析遥控输入 → 目标姿态/推力
void controlAltitude();       // 定高环（ALTHOLD）
void controlAttitude();       // 姿态环
void controlRates();          // 角速率环
void controlTorque();         // 力矩输出（PID → torqueTarget）
#if WEB_RC_ENABLED
void interpretWebRC();        // 解析网页遥控输入
#endif

const char* getModeName();    // 返回当前模式名称字符串

// ============================================================
// 电机（定义在 motors.cpp）
// ============================================================
extern const int MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT; // 四路电机索引
extern float motors[4];       // 四路电机输出（0~1）
extern int motorPins[4];      // 四路电机 GPIO 引脚
extern int pwmFrequency, pwmResolution, pwmStop, pwmMin, pwmMax; // PWM 频率/分辨率/停止值/最小/最大占空

void setupMotors();           // 初始化电机 PWM
void sendMotors();            // 把 motors[] 写入 PWM 输出
bool motorsActive();          // 是否正在输出（用于 landed 判定等）
void testMotor(int n);        // 测试单个电机（CLI 命令用）

// ============================================================
// IMU（定义在 imu.cpp）
// ============================================================
extern bool imuOK;            // IMU 是否初始化成功（false 时禁止解锁）
extern Vector imuRotation;    // IMU 安装旋转（传感器坐标系 → 机体坐标系）
extern Vector accBias;        // 加速度计零偏（m/s²）
extern Vector accScale;       // 加速度计比例因子
extern Vector gyroBias;       // 陀螺仪静态零偏（rad/s）
// 陀螺仪逐轴零偏估计滤波器（定义在 imu.cpp，在线跟踪慢变零偏）
extern KalmanFilter<float> gyroBiasFilterX;
extern KalmanFilter<float> gyroBiasFilterY;
extern KalmanFilter<float> gyroBiasFilterZ;

void setupIMU();              // 初始化 IMU（SPI + 配置 + 识别）
void configureIMU();          // 配置 IMU 并校验 WHO_AM_I
void readIMU();               // 读一帧 IMU 数据并校准，写入全局 gyro/acc
void calibrateGyroOnce();     // 陀螺仪一次性零偏标定
void calibrateAccel();        // 加速度计标定（阻塞约 3s，需水平静止）
void calibrateAccelOnce();    // 加速度计标定（不等待 3s，供上电自动流程）
void printIMUCalibration();   // 打印 IMU 标定值（零偏/比例）
void printIMUInfo();          // 打印 IMU 状态与一帧原始读数

// ============================================================
// 姿态估计（定义在 estimate.cpp）
// ============================================================
extern float accWeight;           // 加速度计对姿态的修正权重（很小）
extern float levelWeight;         // 水平修正权重（0 = 关闭）
extern float levelMaxTilt;        // 水平修正允许的最大倾角
extern float levelGateThreshold;  // 杆量门控阈值（杆量越大越不修正）
extern float levelBiasGain;       // 陀螺零偏在线学习增益（0 = 关闭）
extern Vector levelGyroBias;      // 在线学习的陀螺零偏
extern LowPassFilter<Vector> ratesFilter; // 角速率低通滤波器
extern float velZ; // 估计的垂直速度（m/s，向上为正，无高度参考的近似值）

void estimate();       // 估计主入口（每控制周期调用一次）
void applyGyro();      // 陀螺积分更新姿态
void applyAcc();       // 加速度计修正姿态（仅落地静止时）
void applyLevel();     // 水平修正（默认权重为 0，即关闭）
void estimateVelZ();   // 垂直速度估计

// ============================================================
// 遥控接收机（定义在 rc.cpp）
// ============================================================
extern int rcProtocol;        // 遥控协议（SBUS/CRSF）
extern int rcBaud;            // 串口波特率
extern int rcTxPin;           // 发送引脚
extern int rcRxPin;           // 接收引脚
extern uint16_t channels[16]; // 原始通道值
extern float controlTime;     // 最近一次收到遥控数据的时间（用于失控判断）
extern float channelZero[16]; // 各通道零点
extern float channelMax[16];  // 各通道满量程
extern float rollChannel, pitchChannel, throttleChannel, yawChannel, modeChannel; // 通道映射

void setupRC();               // 初始化遥控串口
bool readRC();                // 读取一帧遥控数据
void normalizeRC();           // 原始通道 → 归一化控制量
void calibrateRC();           // 遥控标定
void calibrateRCChannel(float *channel, uint16_t in[16], uint16_t out[16], const char *str);
void printRCCalibration();    // 打印遥控标定值

// ============================================================
// 电池（定义在 battery.cpp）
// ============================================================
extern float batteryVoltage;  // 当前电池电压（V）

void updateBatteryVoltage();  // 刷新电池电压
float readBatteryVoltage();   // 读取电池电压

// 电池阈值
#define VBAT_WARN_THRESHOLD     3.5f  // 电压告警阈值（V）
#define VBAT_LOW_THRESHOLD      3.3f  // 电压偏低阈值
#define VBAT_CRITICAL_THRESHOLD 3.0f  // 电压危险阈值
#define VBAT_ABSENT_THRESHOLD   0.5f  // 判定"未接电池"的阈值
#define BATTERY_FLYING_THRUST_MIN    0.15f // 判定"正在飞行"的最小推力
#define BATTERY_ACTION_DEBOUNCE_TIME  0.9f // 电池保护动作去抖时间（s）

// ============================================================
// LED（定义在 led.cpp）
// ============================================================
void setupLED();              // 初始化 LED
void setLED(bool on);         // 直接点亮/熄灭
void blinkLED();              // 闪烁
bool batteryAlertActive();    // 是否存在电池告警
bool ledAlertActive();        // 是否存在 LED 告警
void updateLED();             // 按系统状态刷新 LED

// ============================================================
// 安全 / 失控保护（定义在 safety.cpp）
// ============================================================
extern bool isInverted;       // 是否处于倒飞状态
extern float rcLossTimeout;   // 遥控失联超时时间（s）
extern float descendTime;     // 受控下降耗时（s）

void failsafe();              // 汇总触发各类失控保护
void rcLossFailsafe();        // 遥控丢失保护
void descend();               // 受控下降
void autoFailsafe();          // 自动模式失败保护
void invertedFailsafe();      // 倒飞保护
void batteryFailsafe();       // 电池保护
#if WEB_RC_ENABLED
void webRCLossFailsafe();     // 网页遥控丢失保护
#endif

// ============================================================
// 日志（定义在 log.cpp）
// ============================================================
extern Vector attitudeEuler;       // 姿态欧拉角（供日志）
extern Vector attitudeTargetEuler; // 目标姿态欧拉角（供日志）

void prepareLogData();  // 准备一帧日志数据
void logData();         // 记录日志
void printLogHeader();  // 打印日志表头
void printLogData();    // 打印日志数据

// ============================================================
// 参数系统（定义在 parameters.cpp）
// ============================================================
struct Parameter;       // 前向声明（完整定义在 parameters.cpp）

void setupParameters();  // 初始化参数系统
int parametersCount();   // 参数总数
const char* getParameterName(int index); // 按下标取参数名
float getParameter(int index);           // 按下标取值
float getParameter(const char* name);    // 按名字取值
bool setParameter(const char* name, const float value); // 按名字设值
void syncParameters();   // 同步参数到 NVS
void printParameters();  // 打印全部参数
void resetParameters();  // 恢复默认参数

// ============================================================
// 命令行（定义在 cli.cpp）
// ============================================================
void print(const char* format, ...);        // 格式化输出（同时送 CLI/MAVLink/Web）
void pause(float duration);                 // 暂停控制环指定时长（地面调试用）
void doCommand(const char* str, bool echo); // 执行一条命令
void handleInput();                         // 处理串口输入

// ============================================================
// 时间（定义在 time.cpp）
// ============================================================
void step();            // 推进时间基准（更新 t/dt）
void computeLoopRate(); // 计算主循环频率

// ============================================================
// 工具函数（定义在 util.cpp）
// ============================================================
float mapf(float x, float in_min, float in_max, float out_min, float out_max); // 线性区间映射
bool invalid(float x);   // 是否为无效值（NAN）
bool valid(float x);     // 是否为有效值
float wrapAngle(float angle); // 角度归一化到 [-π, π]
void disableBrownOut();  // 关闭欠压复位（Brownout）检测
void splitString(char* str, char* token0, char* token1, char* token2, int maxLen); // 按空格拆分字符串

// UART 读取用的哑元字节（IMU 校准用）
extern uint8_t dummyByte;

// 周期节拍器：每隔 rate 秒返回一次真（用于限频发送等）
class Rate {
public:
	float rate;   // 触发周期（s）
	float last;   // 上次触发时间
	Rate(float rate) : rate(rate), last(0) {}
	operator bool();  // 隐式转 bool：true 表示本周期到点
};

// 延时/去抖判定：持续满足 on 达 delay 秒后返回真
class Delay {
public:
	float delay;  // 需要的持续时间（s）
	float start;  // 开始计时时刻（NAN = 尚未开始）
	Delay(float delay) : delay(delay), start(NAN) {}
	bool update(bool on);  // 送入当前状态，返回是否达到延时
};

// ============================================================
// WiFi（定义在 wifi.cpp）
// ============================================================
#if WIFI_ENABLED
extern int wifiMode;        // WiFi 模式（AP/STA）
extern int udpLocalPort;    // 本地 UDP 端口
extern int udpRemotePort;   // 远端 UDP 端口

void setupWiFi();                                  // 初始化 WiFi
void sendWiFi(const uint8_t *buf, int len);        // 通过 WiFi 发送数据
int receiveWiFi(uint8_t *buf, int len);            // 通过 WiFi 接收数据
void printWiFiInfo();                              // 打印 WiFi 状态信息
void configWiFi(bool ap, const char *ssid, const char *password); // 配置 WiFi（AP/STA）
#endif

// ============================================================
// MAVLink（定义在 mavlink.cpp）
// ============================================================
#if WIFI_ENABLED
extern int mavlinkSysId;      // MAVLink 系统 ID
extern bool mavlinkConnected; // 是否已连接地面站

void processMavlink();        // 处理 MAVLink 收发
void mavlinkPrint(const char* str); // 通过 MAVLink 输出日志
#endif

// ============================================================
// 网页遥控（定义在 web_rc.cpp）
// ============================================================
#if WEB_RC_ENABLED
extern bool webRCEnabled;       // 网页遥控功能是否启用
extern bool useWebRC;           // 当前是否正在使用网页遥控
extern bool webRCUpdated;       // 网页遥控是否更新了数据
extern bool webConsoleEnabled;  // 网页控制台是否启用
extern char webRCWarnMsg[];     // 网页告警信息
extern uint16_t webRCButtons;   // 网页遥控按钮状态
extern unsigned long webRCLastUpdate; // 网页遥控最近一次更新时间

void setupWebRC();              // 初始化网页遥控
void readWebRC();               // 读取网页遥控数据
void processConsoleCommandQueue(); // 处理网页控制台命令队列
bool isWebRCEnabled();          // 网页遥控是否启用
bool isUsingWebRC();            // 是否正在使用网页遥控
void setWebRCWarn(const char* msg); // 设置网页告警信息
void webLog(const char* msg);   // 网页日志输出
#endif

// ============================================================
// 遥测发送频率（定义在 mavlink.cpp 或 main.cpp）
// ============================================================
#if WIFI_ENABLED
extern Rate telemetrySlow;  // 慢速遥测发送节拍
extern Rate telemetryFast;  // 快速遥测发送节拍
#endif
