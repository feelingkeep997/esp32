// ============================================================================
// cli.cpp —— 串口命令行（CLI）与控制台输出
//
// 职责:
//   在 UART0（115200）上提供一个简单命令行，用于地面调试：
//   查看/修改参数、标定 IMU/遥控、测试电机、切换飞行模式、解锁上锁、查看状态、导出日志等。
//   同时 print() 是全工程统一的输出入口，会同时把文本送到串口、MAVLink 和网页控制台。
//
// 关键逻辑:
//   - handleInput() 每周期非阻塞读取串口字节，按行（\n 或 \r）触发 doCommand()；
//   - doCommand() 用 splitString 切成"命令 + 参数0 + 参数1"后逐条匹配（命令名转小写）；
//   - print() 用 vsnprintf 格式化后写 UART0，并按编译开关转发给 MAVLink / 网页控制台；
//   - pause() 在等待期间继续 step()/handleInput()/processMavlink()/readWebRC()，
//     使"等待"期间时间与通讯不中断（但控制环不推进）。
//
// 输入/输出:
//   输入：UART0 上的文本命令；输出：文本响应（经 print）。
//
// 边界情况与潜在风险:
//   - print() 使用 1000 字节栈缓冲，单条日志过长会被截断。
//   - handleInput() 的输入行缓冲为 200 字节，超长输入会被丢弃多余字符（不会报错）。
//   - `preset`（重置参数）会擦除 NVS 并重启，注意会连带清除 WiFi 凭证。
//   - `arm` 命令只检查 imuOK，不检查电量/油门，属调试命令，飞行前慎用。
//   - pause() 期间不执行 estimate()/control()/sendMotors()，即控制环停摆；
//     因此 pause 只适合地面使用（如电机测试）。
//   - CLI 与网页控制台共用 doCommand()，两者命令集完全一致。
// ============================================================================

#include "globals.h"
#include "cf_math.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <ctype.h>

#define CLI_UART_NUM 0
#define CLI_BUF_SIZE 256
#define CLI_MAX_LINE 200

static char cliInputLine[CLI_MAX_LINE];
static int cliInputLen = 0;
static bool cliMotdShown = false;

static const char* motd =
"CLI命令菜单，输入相应命令，回车后执行:\n"
"help - 帮助\n"
"p - 显示所有参数\n"
"p <name> - 显示指定参数\n"
"p <name> <value> - 设置参数\n"
"preset - 重置参数存储\n"
"mfr, mfl, mrr, mrl - 测试马达\n"
"ca - 校准陀螺仪加速度计\n"
"ps - 显示姿态\n"
"cr - 校准RC遥控器\n"
"rc - 显示RC遥控数据\n"
"wifi - 显示WiFi信息\n"
"ap <ssid> <password> - 配置AP模式\n"
"sta <ssid> <password> - 配置STA模式\n"
"raw/stab/acro/auto - 飞行模式\n"
"arm - 解锁无人机\n"
"disarm - 锁定无人机\n"
"psq - 显示姿态四元数\n"
"imu - 显示IMU数据\n"
"time - 显示时间\n"
"mot - 显示motor输出\n"
"sys - 显示系统info\n"
"log [dump] - 打印日志\n"
"reboot - 重启\n"
"reset - 重置姿态\n";

// 统一输出入口：格式化后写 UART0，并按编译开关转发给 MAVLink 与网页控制台。
// 注意：栈缓冲固定 1000 字节，超长内容会被 vsnprintf 截断。
void print(const char* format, ...) {
	char buf[1000];
	va_list args;
	va_start(args, format);
	vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);

	// 输出到 UART0（控制台）
	uart_write_bytes((uart_port_t)CLI_UART_NUM, buf, strlen(buf));

#if WIFI_ENABLED
	mavlinkPrint(buf); // 转发到 MAVLink 串口控制台
#endif
#if WEB_RC_ENABLED
	if (webConsoleEnabled) webLog(buf); // 转发到网页控制台
#endif
}

// 阻塞等待指定时长（秒）。等待期间持续推进时间基准、处理 CLI/MAVLink/网页遥控，
// 但**不执行控制环**（estimate/control/sendMotors），故仅限地面使用。
void pause(float duration) {
	float start = t;
	while (t - start < duration) {
		step();
		handleInput();
#if WIFI_ENABLED
		processMavlink();
#endif
#if WEB_RC_ENABLED
		readWebRC();
#endif
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

void doCommand(const char* str, bool echo) {
	char buf[CLI_MAX_LINE];
	strncpy(buf, str, CLI_MAX_LINE - 1);
	buf[CLI_MAX_LINE - 1] = '\0';

	char command[64], arg0[64], arg1[128];
	splitString(buf, command, arg0, arg1, sizeof(command));
	if (command[0] == '\0') return;

	if (echo) {
		print("> %s\n", str);
	}

	// lowercase command
	for (char* p = command; *p; p++) *p = tolower(*p);

	if (strcmp(command, "help") == 0 || strcmp(command, "motd") == 0) {
		print("%s\n", motd);
	} else if (strcmp(command, "p") == 0 && arg0[0] == '\0') {
		printParameters();
	} else if (strcmp(command, "p") == 0 && arg0[0] != '\0' && arg1[0] == '\0') {
		print("%s = %g\n", arg0, getParameter(arg0));
	} else if (strcmp(command, "p") == 0) {
		bool success = setParameter(arg0, atof(arg1));
		if (success) {
			print("%s = %g\n", arg0, getParameter(arg0));
		} else {
			print("Parameter not found: %s\n", arg0);
		}
	} else if (strcmp(command, "preset") == 0) {
		resetParameters();
	} else if (strcmp(command, "time") == 0) {
		print("Time: %f\n", t);
		print("Loop rate: %.0f\n", loopRate);
		print("dt: %f\n", dt);
	} else if (strcmp(command, "ps") == 0) {
		Vector a = attitude.toEuler();
		print("roll: %f pitch: %f yaw: %f\n", degrees(a.x), degrees(a.y), degrees(a.z));
	} else if (strcmp(command, "psq") == 0) {
		print("qw: %f qx: %f qy: %f qz: %f\n", attitude.w, attitude.x, attitude.y, attitude.z);
	} else if (strcmp(command, "imu") == 0) {
		printIMUInfo();
		printIMUCalibration();
		print("landed: %d\n", landed);
	} else if (strcmp(command, "arm") == 0) {
		if (!imuOK) { print("IMU故障，禁止解锁！\n"); }
		else armed = true;
	} else if (strcmp(command, "disarm") == 0) {
		armed = false;
	} else if (strcmp(command, "raw") == 0) {
		mode = RAW;
	} else if (strcmp(command, "stab") == 0) {
		mode = STAB;
	} else if (strcmp(command, "acro") == 0) {
		mode = ACRO;
	} else if (strcmp(command, "auto") == 0) {
		mode = AUTO;
	} else if (strcmp(command, "rc") == 0) {
		print("channels: ");
		for (int i = 0; i < 16; i++) print("%u ", channels[i]);
		print("\nroll: %g pitch: %g yaw: %g throttle: %g mode: %g\n",
			controlRoll, controlPitch, controlYaw, controlThrottle, controlMode);
		print("time: %.1f\n", controlTime);
		print("mode: %s\n", getModeName());
		print("armed: %d\n", armed);
	} else if (strcmp(command, "wifi") == 0) {
#if WIFI_ENABLED
		printWiFiInfo();
#endif
	} else if (strcmp(command, "ap") == 0) {
#if WIFI_ENABLED
		configWiFi(true, arg0, arg1);
#endif
	} else if (strcmp(command, "sta") == 0) {
#if WIFI_ENABLED
		configWiFi(false, arg0, arg1);
#endif
	} else if (strcmp(command, "mot") == 0) {
		print("front-right %g front-left %g rear-right %g rear-left %g\n",
			motors[MOTOR_FRONT_RIGHT], motors[MOTOR_FRONT_LEFT], motors[MOTOR_REAR_RIGHT], motors[MOTOR_REAR_LEFT]);
	} else if (strcmp(command, "log") == 0) {
		printLogHeader();
		if (strcmp(arg0, "dump") == 0) printLogData();
	} else if (strcmp(command, "cr") == 0) {
		calibrateRC();
	} else if (strcmp(command, "ca") == 0) {
		calibrateAccel();
	} else if (strcmp(command, "mfr") == 0) {
		testMotor(MOTOR_FRONT_RIGHT);
	} else if (strcmp(command, "mfl") == 0) {
		testMotor(MOTOR_FRONT_LEFT);
	} else if (strcmp(command, "mrr") == 0) {
		testMotor(MOTOR_REAR_RIGHT);
	} else if (strcmp(command, "mrl") == 0) {
		testMotor(MOTOR_REAR_LEFT);
	} else if (strcmp(command, "sys") == 0) {
		print("Chip: ESP32\n");
		print("Free heap: %d\n", (int)esp_get_free_heap_size());
	} else if (strcmp(command, "reset") == 0) {
		attitude = Quaternion();
		// Reset gyro bias filters for all axes (KalmanFilter::reset 需要初值参数)
		gyroBiasFilterX.reset(0.0f);
		gyroBiasFilterY.reset(0.0f);
		gyroBiasFilterZ.reset(0.0f);
	} else if (strcmp(command, "reboot") == 0) {
		esp_restart();
	} else {
		print("Invalid command: %s\n", command);
	}
}

// 每周期非阻塞读取串口，累积到一行（\n 或 \r）后交给 doCommand 执行。
// 首次调用时打印一次命令菜单（motd）。输入行超过 CLI_MAX_LINE-1 的部分被丢弃。
void handleInput() {
	if (!cliMotdShown) {
		print("%s\n", motd);
		cliMotdShown = true;
	}

	uint8_t byte;
	while (uart_read_bytes((uart_port_t)CLI_UART_NUM, &byte, 1, 0) > 0) {
		if (byte == '\n' || byte == '\r') {
			if (byte == '\r') {
				// also consume following \n if present
				uart_read_bytes((uart_port_t)CLI_UART_NUM, &byte, 1, 0);
			}
			cliInputLine[cliInputLen] = '\0';
			doCommand(cliInputLine, false);
			cliInputLen = 0;
		} else if (cliInputLen < CLI_MAX_LINE - 1) {
			cliInputLine[cliInputLen++] = byte;
		}
	}
}
