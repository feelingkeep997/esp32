// ============================================================================
// imu.cpp —— IMU 上层驱动（初始化、读取、去偏、标定、调试绘图）
//
// 职责:
//   在 mpu9250.cpp（SPI 寄存器层）之上，提供飞控需要的"已校准、已对齐坐标系"的
//   角速度 gyro 与加速度 acc，并负责芯片识别、零偏标定与串口调试绘图。
//
// 关键逻辑:
//   - setupIMU()/configureIMU() 初始化 SPI 并按 WHO_AM_I 识别芯片
//     （0x71/0x73=MPU9250/9255，0x70=MPU6500；本项目实际使用 MPU6500，寄存器兼容）；
//   - readIMU() 每周期：读原始值 → 陀螺逐轴卡尔曼去偏 → 减 accBias → 乘 accScale →
//     若配置了 imuRotation 则把向量旋到机体坐标系 → 写入全局 gyro / acc；
//   - 陀螺去偏是"静态零偏 + 逐轴卡尔曼残余"两级：gyroBias 由标定得到，
//     gyroBiasFilter*（一维卡尔曼）持续跟踪缓慢漂移，两者相减后再做低通。
//
// 输入/输出:
//   输入：SPI 硬件（经 mpu9250_* 接口）、参数（IMU_ROT_* / IMU_ACC_BIAS_* / IMU_ACC_SCALE_*）。
//   输出：全局 gyro（rad/s）、acc（m/s²）、imuOK 标志。
//
// 重要参数:
//   imuRotation  传感器到机体的安装旋转（欧拉角，参数 IMU_ROT_ROLL/PITCH/YAW）
//   accBias/accScale 加速度计零偏与比例修正
//   IMU_DEBUG_PLOT / IMU_DEBUG_PLOT_HZ  调试绘图开关与输出频率
//
// 边界情况与潜在风险:
//   - imuOK 为 false 时 readIMU() 直接返回，gyro/acc 保持上一次的值（不会清零），
//     因此解锁自检必须依赖 imuOK，否则会拿着陈旧数据飞行。
//   - calibrateGyroOnce() 直接写全局 gyro/acc（采样期间会污染控制用数据），
//     必须在解锁前调用；calibrateAccel*() 会阻塞 3 秒+，同样仅限地面使用。
//   - calibrateAccel 假定"机体水平静止"，未做六面标定，故只能校正零偏不能校正比例。
//   - IMU_DEBUG_PLOT 打开时会以 100Hz 往 UART0 直写纯数值行（绕过 print），
//     会与 CLI/日志输出混在一起，仅调试时开启。
//   - 调试绘图输出用 invalid(hoverThrustEst) 判断，未激活时发 -1 作为占位。
// ============================================================================

#include "globals.h"
#include "mpu9250.h"
#include <cmath>
#include "esp_log.h"
#include "driver/uart.h"   // 调试绘图需直接写控制台串口
#include <cstdio>          // snprintf
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "imu";

// ============================================================
// FireWater 串口绘图（调试用）
// 数据格式（无标签形式）："ch0,ch1,...,chN\n"
// 必须以 '\n' 结尾，且每行只能有纯数值 + 逗号，FireWater 才会绘图
// ============================================================
#define IMU_DEBUG_PLOT     1    // 1 = 打开调试绘图，0 = 关闭
#define IMU_DEBUG_PLOT_HZ  100  // 输出频率（Hz），避免刷屏或压垮串口

#if IMU_DEBUG_PLOT
// 把一组原始 IMU 数据按 FireWater 格式写到控制台串口（UART0）
// 直接 uart_write_bytes 而不走 print()，避免同时灌到 MAVLink 和 Web 控制台
static void imuPlotRaw(const Vector& accel, const Vector& gyroRad) {
    static float lastTime = 0.0f;
    if (t - lastTime < 1.0f / IMU_DEBUG_PLOT_HZ) return; // 限速
    lastTime = t;

    char buf[192];
    // 通道顺序：acc xyz (m/s^2)、gyro xyz (deg/s)、姿态 roll/pitch/yaw (rad)、
    //           velZ (m/s)、thrustTarget (0~1)、hoverThrustEst (0~1，未激活发-1)、armed (0/1)
    // 注意：不加任何标签/前缀，行首即第一个数值
    Vector euler = attitude.toEuler();
    // 13 个通道，逗号分隔、行尾 '\n'，与上方"通道顺序"说明一一对应：
    int n = snprintf(buf, sizeof(buf), "%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.1f\n",
                     accel.x,                                       // ch0 : 加速度 X (m/s^2)
                     accel.y,                                       // ch1 : 加速度 Y (m/s^2)
                     accel.z,                                       // ch2 : 加速度 Z (m/s^2)
                     gyroRad.x * 180.0f / M_PI,                     // ch3 : 角速度 X (deg/s)
                     gyroRad.y * 180.0f / M_PI,                     // ch4 : 角速度 Y (deg/s)
                     gyroRad.z * 180.0f / M_PI,                     // ch5 : 角速度 Z (deg/s)
                     euler.x,                                       // ch6 : 姿态 roll  (rad，绕机体 X)
                     euler.y,                                       // ch7 : 姿态 pitch (rad，绕机体 Y)
                     euler.z,                                       // ch8 : 姿态 yaw   (rad，绕机体 Z)
                     velZ,                                          // ch9 : 垂直速度 (m/s，向上为正)
                     thrustTarget,                                  // ch10: 油门目标 (0~1，已限幅)
                     invalid(hoverThrustEst) ? -1.0f : hoverThrustEst, // ch11: 悬停油门估计 (0~1；未激活发 -1)
                     armed ? 1.0f : 0.0f);                          // ch12: 解锁标志 (1=已解锁, 0=未解锁)
    if (n > 0) uart_write_bytes(UART_NUM_0, buf, n);
}
#endif

// 陀螺仪逐轴零偏（漂移）估计器：每个轴各一个一维卡尔曼滤波器
// 作用：在静态陀螺零偏 gyroBias（由 ca 校准得到）之外，再在线跟踪"残余/慢变"零偏漂移。
//       readIMU() 里对每帧原始角速度做 filtered = filter.update(rawGyro.*)，随后用
//       correctedGyro = rawGyro - gyroBias - Vector(filteredX, filteredY, filteredZ) 一并扣除。
// 相比原一阶低通：卡尔曼用噪声模型（q=过程噪声 / r=测量噪声）实时计算增益 K，
//   起步/久未观测时 K 大（快速收敛到当前值），估计稳定后 K 小（强力抑噪、输出平滑）。
// 参数：初始 0.0f；q = 1e-6f（偏置极慢漂移）、r = 0.01f（测量含噪声），
//       稳态增益 K_ss ≈ 0.01，与原 alpha=0.01 低通平滑程度相当，但收敛更快。
// 注意：定义为全局变量，滤波状态在多次 readIMU() 调用之间持续累积，不会因函数返回而清零。
KalmanFilter<float> gyroBiasFilterX(0.0f, 1e-6f, 0.01f);
KalmanFilter<float> gyroBiasFilterY(0.0f, 1e-6f, 0.01f);
KalmanFilter<float> gyroBiasFilterZ(0.0f, 1e-6f, 0.01f);

bool imuOK = false;
Vector imuRotation(0, 0, 0);  // IMU 安装旋转（欧拉角 rad）：描述"传感器坐标系 -> 机体坐标系"的方向偏置；参数 IMU_ROT_ROLL/PITCH/YAW，默认 0 表示无旋转
Vector accBias(0, 0, 0);      // 加速度计零偏（m/s²）：每轴恒定偏移量，由 ca 校准写入（水平静止采样均值 - 重力）；参数 IMU_ACC_BIAS_*，默认 0
Vector accScale(1, 1, 1);     // 加速度计比例因子（无量纲）：每轴增益/灵敏度修正；本工程未做六面标定，故恒为 1；参数 IMU_ACC_SCALE_*
Vector gyroBias(0, 0, 0);     // 陀螺仪静态零偏（rad/s）：本应由校准写入，但 calibrateGyroOnce() 无调用者，实际永远为 0（真实零偏由 readIMU 内卡尔曼滤波器在线跟踪扣除）

// 初始化 IMU：底层 SPI 初始化 + 配置并识别芯片
void setupIMU() {
    print("Setting up MPU9250...\n");
    
    // 初始化 MPU9250（SPI 总线与设备）
    mpu9250_init();
    
    // 配置 IMU（量程、采样率、DLPF）并校验 WHO_AM_I
    configureIMU();
}

// 配置 IMU：设置陀螺 ±1000dps / 加速度 ±16g，识别 WHO_AM_I 并置 imuOK。
// 注意：识别失败时 imuOK=false，此后 readIMU() 不再更新数据。
void configureIMU() {
    print("Configuring MPU9250...\n");
    
    // 配置：陀螺 ±1000 deg/s，加速度 ±16g
    mpu9250_configure(MPU9250_GYRO_FSR_1000, MPU9250_ACCEL_FSR_16G);
    
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // 校验芯片身份
    // WHO_AM_I: 0x71/0x73 = MPU9250/MPU9255，0x70 = MPU6500
    // （本项目实际用的是 MPU6500，其加速度计/陀螺仪寄存器与 MPU9250 兼容）
    uint8_t whoami = mpu9250_read_reg(MPU_REG_WHO_AM_I);
    if (whoami == 0x71 || whoami == 0x73 || whoami == 0x70) {
        print("MPU9250/MPU6500 detected: WHO_AM_I=0x%02X\n", whoami);
        imuOK = true;
        
        // 应用安装旋转参数：把"传感器坐标系"旋到"机体坐标系"
        Quaternion rotQuat = Quaternion::fromEuler(imuRotation);
        mpu9250_set_rotation(rotQuat);
    } else {
        print("IMU not found! Expected 0x71/0x73 (MPU9250) or 0x70 (MPU6500), got 0x%02X\n", whoami);
        imuOK = false;
    }
}

// 读取一帧 IMU 数据并完成校准/对齐，写入全局 gyro / acc。
// 处理链：原始值 → 陀螺逐轴低通去偏 → 减静态零偏 → 乘比例因子 → 坐标系旋转。
// 注意：imuOK=false 时直接返回（不清零数据）。
void readIMU() {
    if (!imuOK) return;
    
    // 读取加速度计与陀螺仪原始数据
    Vector rawGyro, rawAcc;
    mpu9250_read_data(&rawGyro, &rawAcc);

#if IMU_DEBUG_PLOT
    imuPlotRaw(rawAcc, rawGyro); // FireWater 调试绘图：输出驱动返回的原始 acc / gyro
#endif
    
    // 逐轴低通跟踪残余零偏，再与静态零偏 gyroBias 一并扣除
    float filteredX = gyroBiasFilterX.update(rawGyro.x);
    float filteredY = gyroBiasFilterY.update(rawGyro.y);
    float filteredZ = gyroBiasFilterZ.update(rawGyro.z);
    Vector correctedGyro = rawGyro - gyroBias - Vector(filteredX, filteredY, filteredZ);
    Vector correctedAcc = rawAcc - accBias;
    
    // 应用加速度计比例修正
    correctedAcc.x *= accScale.x;
    correctedAcc.y *= accScale.y;
    correctedAcc.z *= accScale.z;
    
    // 若配置了安装旋转，则把两路向量旋到机体坐标系
    if (imuRotation.norm() > 0.001f) {
        Quaternion rotQuat = Quaternion::fromEuler(imuRotation);
        correctedGyro = rotQuat.conjugate(correctedGyro);
        correctedAcc = rotQuat.conjugate(correctedAcc);
    }
    
    gyro = correctedGyro;
    acc = correctedAcc;
}

// 陀螺仪一次性零偏标定：静止采样 200 次求均值作为 gyroBias。
// 注意：采样期间直接写全局 gyro/acc，必须在解锁前调用。
void calibrateGyroOnce() {
    if (!imuOK) return;
    
    print("Calibrating gyro once...\n");
    Vector sum(0, 0, 0);
    int samples = 200;
    
    for (int i = 0; i < samples; i++) {
        mpu9250_read_data(&gyro, &acc);
        sum += gyro;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    gyroBias = sum / samples;
    print("Gyro bias: %f %f %f\n", gyroBias.x, gyroBias.y, gyroBias.z);
}

// 加速度计标定：水平静止采样 500 次求均值，减去重力得到 accBias。
// 注意：会阻塞约 3s + 采样时间；仅限地面调用。假定机体水平，未做六面标定。
void calibrateAccel() {
    if (!imuOK) return;
    
    print("Calibrating accel...\n");
    print("Place drone on flat surface and keep still...\n");
    
    print("Starting calibration in 3 seconds...\n");
    vTaskDelay(pdMS_TO_TICKS(3000)); // 留出时间摆平并静置
    
    Vector sum(0, 0, 0);
    int samples = 500;
    
    for (int i = 0; i < samples; i++) {
        mpu9250_read_data(&gyro, &acc); // 注意：直接写全局数据
        sum += acc;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    
    accBias = sum / samples - Vector(0, 0, ONE_G); // 减去重力（Z 轴）
    print("Accel bias: %f %f %f\n", accBias.x, accBias.y, accBias.z);
}

// 与 calibrateAccel 相同的采样逻辑，但不等待 3 秒（供上电自动标定流程使用）
void calibrateAccelOnce() {
    if (!imuOK) return;
    
    print("Calibrating accel once...\n");
    Vector sum(0, 0, 0);
    int samples = 500;
    
    for (int i = 0; i < samples; i++) {
        mpu9250_read_data(&gyro, &acc);
        sum += acc;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    
    accBias = sum / samples - Vector(0, 0, ONE_G);
    print("Accel bias: %f %f %f\n", accBias.x, accBias.y, accBias.z);
}

// 打印当前 IMU 标定值（零偏与比例因子）
void printIMUCalibration() {
    print("Gyro bias: %f %f %f\n", gyroBias.x, gyroBias.y, gyroBias.z);
    print("Accel bias: %f %f %f\n", accBias.x, accBias.y, accBias.z);
    print("Accel scale: %f %f %f\n", accScale.x, accScale.y, accScale.z);
}

// 打印 IMU 状态：WHO_AM_I 与一帧原始读数（CLI 命令 imu）
void printIMUInfo() {
    if (!imuOK) {
        print("IMU not initialized\n");
        return;
    }
    
    uint8_t whoami = mpu9250_read_reg(MPU_REG_WHO_AM_I);
    print("MPU9250 WHO_AM_I: 0x%02X\n", whoami);
    
    Vector dummyGyro, dummyAcc;
    mpu9250_read_data(&dummyGyro, &dummyAcc);
    // 注意：Vector 分量是 float，必须用 %f（原来的 %d 属于未定义行为，会打印垃圾值）
    print("Raw gyro (rad/s): %f %f %f\n", dummyGyro.x, dummyGyro.y, dummyGyro.z);
    print("Raw accel (m/s^2): %f %f %f\n", dummyAcc.x, dummyAcc.y, dummyAcc.z);
}
