// ============================================================================
// log.cpp —— 内存飞行日志（黑匣子）
//
// 职责:
//   在解锁飞行期间以固定频率把关键状态采样到 RAM 环形缓冲区，供事后通过
//   CLI 命令 `log` / `log dump` 导出 CSV，用于分析振动、跟踪误差、姿态漂移等。
//
// 关键逻辑:
//   - 采样列由 logEntries[] 声明（列名 + 变量指针），打印表头与数据都遍历它；
//   - 缓冲区是固定大小的二维数组 logBuffer[LOG_SIZE][14]，写满后回绕覆盖最旧数据；
//   - logData() 仅在上锁(armed)时记录，并按 LOG_RATE(100Hz) 限速；
//   - attitudeEuler/attitudeTargetEuler 是四元数转欧拉角的中间缓存，列里记录的是它们。
//
// 输入/输出:
//   无参数。输出为串口 CSV 文本（printLogHeader / printLogData）。
//
// 重要参数:
//   LOG_RATE     采样频率 Hz（100），与 LOG_DURATION 相乘决定缓冲容量。
//   LOG_DURATION 记录时长 s，来自板级宏 BOARD_LOG_DURATION（ESP32/S3 为 8s）。
//   LOG_SIZE     = LOG_DURATION * LOG_RATE，缓冲区行数。
//
// 边界情况与潜在风险:
//   - 缓冲区为静态分配：LOG_SIZE*14*4 字节，ESP32 默认 8s*100Hz=800 行约 44KB，
//     属较大的常驻 RAM 占用，调大 LOG_DURATION 需评估内存余量。
//   - 第 14 列上限是硬编码（[14]）：新增 logEntries 列数超过 14 会越界写。
//   - printLogData 以 logBuffer[i][0]==0（时间列为 0）判断"空槽"，若某次采样恰好
//     t 为 0 则该行被跳过（实际飞行中 t 远大于 0，不会触发）。
//   - 回绕后 dump 输出顺序不是时间序（从物理 0 行开始），分析时需自行按 t 排序。
// ============================================================================

#include "globals.h"

#define LOG_RATE 100                              // 采样频率 Hz
#define LOG_DURATION BOARD_LOG_DURATION           // 记录时长 s（板级宏）
#define LOG_SIZE (LOG_DURATION * LOG_RATE)        // 环形缓冲行数

Vector attitudeEuler;        // attitude 的欧拉角缓存（供日志记录）
Vector attitudeTargetEuler;  // attitudeTarget 的欧拉角缓存（供日志记录）

// 一条日志列：列名 + 指向被记录变量的指针
struct LogEntry {
	const char *name;
	float *value;
};

// 日志列定义：顺序即 CSV 列顺序（共 14 列，勿超 logBuffer 的第二维）
static LogEntry logEntries[] = {
	{"t", &t},
	{"rates.x", &rates.x},
	{"rates.y", &rates.y},
	{"rates.z", &rates.z},
	{"ratesTarget.x", &ratesTarget.x},
	{"ratesTarget.y", &ratesTarget.y},
	{"ratesTarget.z", &ratesTarget.z},
	{"attitude.x", &attitudeEuler.x},
	{"attitude.y", &attitudeEuler.y},
	{"attitude.z", &attitudeEuler.z},
	{"attitudeTarget.x", &attitudeTargetEuler.x},
	{"attitudeTarget.y", &attitudeTargetEuler.y},
	{"attitudeTarget.z", &attitudeTargetEuler.z},
	{"thrustTarget", &thrustTarget}
};

static const int logColumns = sizeof(logEntries) / sizeof(logEntries[0]);
static float logBuffer[LOG_SIZE][14]; // 14 列上限（硬编码，见文件头风险）

// 把四元数姿态转换为欧拉角缓存（打印日志前调用）
void prepareLogData() {
	attitudeEuler = attitude.toEuler();
	attitudeTargetEuler = attitudeTarget.toEuler();
}

// 按 LOG_RATE 限速采样一帧；仅解锁时记录
void logData() {
	if (!armed) return;
	static int logPointer = 0;
	static Rate period(LOG_RATE);
	if (!period) return;

	prepareLogData();

	for (int i = 0; i < logColumns; i++) {
		logBuffer[logPointer][i] = *logEntries[i].value;
	}

	logPointer++;
	if (logPointer >= LOG_SIZE) { // 环形回绕
		logPointer = 0;
	}
}

// 打印 CSV 表头（列名）
void printLogHeader() {
	for (int i = 0; i < logColumns; i++) {
		print("%s%s", logEntries[i].name, i < logColumns - 1 ? "," : "\n");
	}
}

// 打印缓冲区中所有已写入的日志行（跳过空槽）
void printLogData() {
	for (int i = 0; i < LOG_SIZE; i++) {
		if (logBuffer[i][0] == 0) continue; // 空槽判断（见文件头风险）
		for (int j = 0; j < logColumns; j++) {
			print("%g%s", logBuffer[i][j], j < logColumns - 1 ? "," : "\n");
		}
	}
}
