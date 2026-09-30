// ============================================================================
// main.cpp —— 程序入口与主循环（ESP-IDF 移植版）
//
// 职责:
//   1) 定义全局状态变量（t/dt、控制输入、姿态、电机相关等，原 CF-Drone.ino 中的全局量）；
//   2) 初始化控制台 UART（UART0，115200）；
//   3) 执行一次性初始化 setup()；
//   4) 运行 app_main() 主循环，按固定顺序调度各模块。
//
// 关键逻辑:
//   - app_main() 先初始化 NVS（参数系统与 WiFi 都依赖它），NVS 损坏时自动擦除重建；
//   - setup() 按"参数 → LED → 电机 → WiFi → Web RC → IMU → 遥控"的顺序初始化，
//     期间用 LED 常亮表示正在初始化；
//   - loop() 是控制主循环，执行顺序对控制正确性至关重要（见下方注释）。
//
// 输入/输出:
//   无参数。输入来自各外设（IMU/遥控/WiFi），输出为电机 PWM 与日志。
//
// 边界情况与潜在风险:
//   - 全局变量在此定义（而非在 globals.h），其余模块通过 extern 引用；
//     新增全局量时需同步在 globals.h 中声明。
//   - loop() 末尾的 vTaskDelay(1 tick) 是为了喂看门狗；若某模块耗时过长
//     （如 HTTP 处理、参数批量写 flash），仍可能触发看门狗复位。
//   - 所有模块都在同一个任务（主任务）里顺序执行，没有多任务并发保护；
//     HTTP/ESP-NOW 回调在其它任务里，通过队列或标志与主循环交互（见 web_rc.cpp/espnow.cpp）。
//   - 初始化顺序不能随意调换：例如 setupMotors() 必须在 sendMotors() 之前，
//     setupParameters() 必须在依赖参数的各模块之前。
//   - 若 IMU 初始化失败（imuOK=false），程序不会中止，但会禁止解锁（见 control.cpp）。
// ============================================================================

// CF-Drone main entry point - ESP-IDF port
// Entry point: app_main() + main loop

#include "globals.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

// ============================================================
// Global variable definitions (originally in CF-Drone.ino)
// ============================================================
float t = NAN;
float dt;
float controlRoll, controlPitch, controlYaw, controlThrottle;
float controlMode = NAN;
Vector gyro;
Vector acc;
Vector rates;
Quaternion attitude;
bool landed;

// Dummy byte for UART reading (used in IMU calibration)
uint8_t dummyByte = 0;

// ============================================================
// 控制台 UART 初始化（UART0，115200-8N1，无流控）
// 说明：只配置参数与引脚（沿用默认引脚），实际驱动安装后会由 cli.cpp 读写。
// ============================================================
static void initConsoleUART() {
	uart_config_t uart_config = {};
	uart_config.baud_rate = 115200;
	uart_config.data_bits = UART_DATA_8_BITS;
	uart_config.parity = UART_PARITY_DISABLE;
	uart_config.stop_bits = UART_STOP_BITS_1;
	uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
	uart_config.source_clk = UART_SCLK_APB;

	uart_driver_install(UART_NUM_0, 512, 512, 0, NULL, 0);
	uart_param_config(UART_NUM_0, &uart_config);
	uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
	             UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// ============================================================
// Setup：一次性初始化（顺序有依赖，勿随意调换）
// ============================================================
static void setup() {
	print("程序开始初始化！\n");
	disableBrownOut();   // 关闭欠压复位，避免电机启动瞬间掉压导致重启
	setupParameters();   // 最先初始化：后续模块的配置都来自参数
	setupLED();
	setupMotors();
	setLED(true);        // 初始化期间常亮
#if WIFI_ENABLED
	setupWiFi();         // 建立 UDP/ESP-NOW 链路
#endif
#if WEB_RC_ENABLED
	setupWebRC();        // 启动网页遥控 HTTP 服务
#endif
	setupIMU();          // 初始化 IMU（失败会置 imuOK=false，禁止解锁）
	setupRC();           // 初始化遥控串口
	setLED(false);
	print("程序初始化完成！\n");
	print("================================\n");
}

// ============================================================
// Main loop：控制主循环（执行顺序对控制正确性至关重要）
// ============================================================
static void loop() {
	readIMU();               // 1) 采样 IMU（拿到最新 gyro/acc）
	step();                  // 2) 推进时间基准（更新 t/dt，供后续滤波/PID 使用）
	readRC();                // 3) 采样遥控（更新控制输入与 controlTime）
#if WEB_RC_ENABLED
	readWebRC();             // 4) 刷新网页遥控在线标志
	processConsoleCommandQueue(); // 5) 执行网页控制台排队的命令
#endif
	estimate();              // 6) 姿态/速度估计（依赖 IMU 与时间）
	updateBatteryVoltage();  // 7) 刷新电池电压（供保护与显示）
	control();               // 8) 控制链（含 failsafe，输出 motors[]）
	sendMotors();            // 9) 下发电机 PWM
	handleInput();           // 10) 处理串口 CLI 输入
#if WIFI_ENABLED
	processMavlink();        // 11) MAVLink 遥测收发
#endif
	logData();               // 12) 飞行日志采样（仅解锁时）
	syncParameters();        // 13) 参数回写 NVS（1Hz、未飞行时）
	updateLED();             // 14) 刷新状态灯
}

// ============================================================
// ESP-IDF 入口
// ============================================================
extern "C" void app_main() {
	// 初始化 NVS（参数系统与 WiFi 都依赖）；损坏/版本不符时擦除后重建
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		nvs_flash_erase();
		nvs_flash_init();
	}

	// 初始化控制台 UART（UART0）
	initConsoleUART();

	// 一次性初始化
	setup();

	// 主循环（运行在主任务上）
	while (true) {
		loop();
		// 让出 1 个 tick，防止看门狗超时
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}
