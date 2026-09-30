// ============================================================================
// parameters.cpp —— 参数系统（运行时调参与 NVS 掉电保存）
//
// 职责:
//   把飞控中可调的关键变量（PID 增益、限幅、IMU 标定、电机 PWM、遥控通道、WiFi 等）
//   注册成"命名参数"，从而支持：CLI 读写（p NAME VALUE）、MAVLink 参数协议读写、
//   以及掉电保存（存于 NVS 命名空间 "flix"）。
//
// 关键逻辑:
//   - 每个 Parameter 持有"名字 + 变量指针（float* 或 int*）+ 缓存 + 可选回调"；
//   - setupParameters() 启动时逐个从 NVS 读取：键不存在或损坏则把当前默认值写回 NVS；
//   - setParameter() 写入后立即调用回调（如 MOT_PIN_* / MOT_PWM_* 会触发 setupMotors()
//     重新初始化 PWM），实现"改参数即生效"；
//   - syncParameters() 在运行时（未飞行、1Hz）检测到变量被直接改动时回写 NVS，
//     这样 PID 自整定/MAVLink 改值也能持久化。
//
// 输入/输出:
//   输入：参数名（字符串，大小写不敏感）与值；输出：参数值 / 是否成功。
//
// 重要参数:
//   NVS 命名空间 "flix"；参数表 parameters[] 是唯一注册点（新增参数只需在此加一行）。
//
// 边界情况与潜在风险:
//   - Parameter 存储的是"变量地址"，因此被指向的变量必须是全局/静态生命周期；
//     若指向栈上临时对象会造成悬空指针。
//   - integer 型参数拒绝非有限值（NAN/INF），float 型不检查，写入 NAN 会污染该参数。
//   - resetParameters() 会 nvs_erase_all() 后 esp_restart()，清空的不只是本模块的键
//     （WiFi 凭证等同一命名空间下的键也会一并清除）。
//   - syncParameters() 在 motorsActive() 时跳过，避免飞行中频繁写 flash；
//     因此飞行中改的参数要到落地后才会持久化。
//   - 参数表引用 rollRatePID.p 等成员，依赖 control.cpp 的全局 PID 对象已构造；
//     静态初始化顺序若被改动可能导致悬空指针。
// ============================================================================

#include "globals.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include <string.h>

static nvs_handle_t nvs_storage; // NVS 句柄（命名空间 "flix"）

// 一个可调参数：名字 + 变量指针 + 缓存（用于检测运行时改动）+ 可选回调
struct Parameter {
	const char *name;                     // 参数名（大小写不敏感）
	bool integer;                         // true = 指向 int 变量，false = float
	union { float *f; int *i; };          // 指向实际变量的指针
	float cache;                          // 上次同步到 NVS 的值（检测改动用）
	void (*callback)();                   // 值改变后的回调（可空）
	Parameter(const char *name, float *variable, void (*callback)() = nullptr)
		: name(name), integer(false), f(variable), cache(0), callback(callback) {}
	Parameter(const char *name, int *variable, void (*callback)() = nullptr)
		: name(name), integer(true), i(variable), cache(0), callback(callback) {}
	float getValue() const { return integer ? (float)*i : *f; }        // 统一按 float 读取
	void setValue(const float value) { if (integer) *i = (int)value; else *f = value; } // 按类型写回
};

// 参数注册表：新增可调参数只需在此追加一行。
// 分组：控制增益 / 限幅 / 配平 / 定高 / 模式映射 / IMU 标定 / 估计 / 电机 / 遥控 / WiFi / 安全
static Parameter parameters[] = {
	{"CTL_R_RATE_P",  &rollRatePID.p},
	{"CTL_R_RATE_I",  &rollRatePID.i},
	{"CTL_R_RATE_D",  &rollRatePID.d},
	{"CTL_R_RATE_WU", &rollRatePID.windup},
	{"CTL_P_RATE_P",  &pitchRatePID.p},
	{"CTL_P_RATE_I",  &pitchRatePID.i},
	{"CTL_P_RATE_D",  &pitchRatePID.d},
	{"CTL_P_RATE_WU", &pitchRatePID.windup},
	{"CTL_Y_RATE_P",  &yawRatePID.p},
	{"CTL_Y_RATE_I",  &yawRatePID.i},
	{"CTL_Y_RATE_D",  &yawRatePID.d},

	{"CTL_R_P",  &rollPID.p},
	{"CTL_R_I",  &rollPID.i},
	{"CTL_R_D",  &rollPID.d},
	{"CTL_R_WU", &rollPID.windup},
	{"CTL_P_P",  &pitchPID.p},
	{"CTL_P_I",  &pitchPID.i},
	{"CTL_P_D",  &pitchPID.d},
	{"CTL_P_WU", &pitchPID.windup},
	{"CTL_Y_P",  &yawPID.p},

	{"CTL_P_RATE_MAX", &maxRate.y},
	{"CTL_R_RATE_MAX", &maxRate.x},
	{"CTL_Y_RATE_MAX", &maxRate.z},
	{"CTL_TILT_MAX",   &tiltMax},

	{"CTL_TRIM_ROLL",  &trimRoll},
	{"CTL_TRIM_PITCH", &trimPitch},

	// ===== ALTHOLD 近似定高（无气压计）=====
	// 油门中位死区内 = 松杆悬停；死区外 = 升降速率指令。
	// ALT_P 过小松杆后缓升缓降不停顿，过大则垂直方向振荡；
	// ALT_I 决定悬停油门学习速度，过大振荡、过小松杆后缓慢下沉/上飘。
	{"ALT_P",         &altHoldP},
	{"ALT_I",         &altHoldI},
	{"ALT_RATE_MAX",  &altHoldRateMax},
	{"ALT_DEADBAND",  &altHoldDeadband},

	{"CTL_FLT_MODE_0", &flightModes[0]},
	{"CTL_FLT_MODE_1", &flightModes[1]},
	{"CTL_FLT_MODE_2", &flightModes[2]},

	{"IMU_ROT_ROLL",  &imuRotation.x},
	{"IMU_ROT_PITCH", &imuRotation.y},
	{"IMU_ROT_YAW",   &imuRotation.z},

	// Note: gyro bias filter alpha is now handled per-axis (X, Y, Z)

	{"IMU_ACC_BIAS_X", &accBias.x},
	{"IMU_ACC_BIAS_Y", &accBias.y},
	{"IMU_ACC_BIAS_Z", &accBias.z},
	{"IMU_ACC_SCALE_X", &accScale.x},
	{"IMU_ACC_SCALE_Y", &accScale.y},
	{"IMU_ACC_SCALE_Z", &accScale.z},

	{"EST_ACC_WEIGHT",   &accWeight},
	{"EST_RATES_LPF_A",  &ratesFilter.alpha},
	{"EST_LVL_GATE_THR", &levelGateThreshold},
	{"EST_LVL_BIAS_GAIN",&levelBiasGain},

	{"MOT_PIN_FL", &motorPins[MOTOR_FRONT_LEFT],  setupMotors},
	{"MOT_PIN_FR", &motorPins[MOTOR_FRONT_RIGHT], setupMotors},
	{"MOT_PIN_RL", &motorPins[MOTOR_REAR_LEFT],   setupMotors},
	{"MOT_PIN_RR", &motorPins[MOTOR_REAR_RIGHT],  setupMotors},

	{"MOT_PWM_FREQ", &pwmFrequency, setupMotors},
	{"MOT_PWM_RES",  &pwmResolution, setupMotors},
	{"MOT_PWM_STOP", &pwmStop},
	{"MOT_PWM_MIN",  &pwmMin},
	{"MOT_PWM_MAX",  &pwmMax},

	{"MOT_THR_MIN", &motThrMin},
	{"MOT_THR_MAX", &motThrMax},

	{"RC_ZERO_0", &channelZero[0]},
	{"RC_ZERO_1", &channelZero[1]},
	{"RC_ZERO_2", &channelZero[2]},
	{"RC_ZERO_3", &channelZero[3]},
	{"RC_ZERO_4", &channelZero[4]},
	{"RC_ZERO_5", &channelZero[5]},
	{"RC_ZERO_6", &channelZero[6]},
	{"RC_ZERO_7", &channelZero[7]},
	{"RC_MAX_0",  &channelMax[0]},
	{"RC_MAX_1",  &channelMax[1]},
	{"RC_MAX_2",  &channelMax[2]},
	{"RC_MAX_3",  &channelMax[3]},
	{"RC_MAX_4",  &channelMax[4]},
	{"RC_MAX_5",  &channelMax[5]},
	{"RC_MAX_6",  &channelMax[6]},
	{"RC_MAX_7",  &channelMax[7]},

	{"RC_ROLL",     &rollChannel},
	{"RC_PITCH",    &pitchChannel},
	{"RC_THROTTLE", &throttleChannel},
	{"RC_YAW",      &yawChannel},
	{"RC_MODE",     &modeChannel},

	{"RC_RX_PIN",   &rcRxPin},
	{"RC_PROTOCOL", &rcProtocol},
	{"RC_TX_PIN",   &rcTxPin},
	{"RC_BAUD",     &rcBaud},

#if WIFI_ENABLED
	{"WIFI_MODE",     &wifiMode},
	{"WIFI_LOC_PORT", &udpLocalPort},
	{"WIFI_REM_PORT", &udpRemotePort},

	{"MAV_SYS_ID",    &mavlinkSysId},
	{"MAV_RATE_SLOW", &telemetrySlow.rate},
	{"MAV_RATE_FAST", &telemetryFast.rate},
#endif

	{"SF_RC_LOSS_TIME",  &rcLossTimeout},
	{"SF_DESCEND_TIME",  &descendTime},
};

// 启动时加载参数：打开 NVS，逐个读取；键缺失/损坏则把当前默认值写回。
// 需在其它模块使用参数前调用（见 main.cpp 的 setup()）。
void setupParameters() {
	print("Setup parameters\n");
	nvs_open("flix", NVS_READWRITE, &nvs_storage);

	for (auto &parameter : parameters) {
		float stored = parameter.getValue(); // 以当前默认值为初值
		size_t required_size = sizeof(float);
		esp_err_t err = nvs_get_blob(nvs_storage, parameter.name, &stored, &required_size);
		if (err != ESP_OK || required_size != sizeof(float)) {
			// 键不存在或长度不符：把默认值写入 NVS
			nvs_set_blob(nvs_storage, parameter.name, &stored, sizeof(float));
			nvs_commit(nvs_storage);
		}
		if (parameter.integer && !isfinite(stored)) {
			// 整型参数读到非有限值视为损坏：回退到默认值并重写
			stored = parameter.getValue();
			nvs_set_blob(nvs_storage, parameter.name, &stored, sizeof(float));
			nvs_commit(nvs_storage);
		}
		parameter.setValue(stored);
		parameter.cache = parameter.getValue();
	}
}

// 参数总数
int parametersCount() {
	return sizeof(parameters) / sizeof(parameters[0]);
}

// 按下标取参数名；越界返回空串
const char *getParameterName(int index) {
	if (index < 0 || index >= parametersCount()) return "";
	return parameters[index].name;
}

// 按下标取参数值；越界返回 NAN
float getParameter(int index) {
	if (index < 0 || index >= parametersCount()) return NAN;
	return parameters[index].getValue();
}

// 按名字取参数值（大小写不敏感）；未找到返回 NAN
float getParameter(const char *name) {
	for (auto &parameter : parameters) {
		if (strcasecmp(parameter.name, name) == 0) {
			return parameter.getValue();
		}
	}
	return NAN;
}

// 按名字设置参数值（大小写不敏感）；成功时触发回调。整型参数拒绝非有限值。
// 返回是否找到并写入成功。
bool setParameter(const char *name, const float value) {
	for (auto &parameter : parameters) {
		if (strcasecmp(parameter.name, name) == 0) {
			if (parameter.integer && !isfinite(value)) return false;
			parameter.setValue(value);
			if (parameter.callback) parameter.callback(); // 如 MOT_PWM_FREQ -> setupMotors()
			return true;
		}
	}
	return false;
}

// 把运行时被直接改动的参数回写 NVS（1Hz 限速；飞行中跳过，避免频繁写 flash）。
// 用于持久化 MAVLink/自整定等"绕过 setParameter"的修改。
void syncParameters() {
	static Rate rate(1);
	if (!rate) return;
	if (motorsActive()) return; // 飞行中不写 flash

	for (auto &parameter : parameters) {
		if (parameter.getValue() == parameter.cache) continue;
		if (isnan(parameter.getValue()) && isnan(parameter.cache)) continue;
		float val = parameter.getValue();
		nvs_set_blob(nvs_storage, parameter.name, &val, sizeof(float));
		nvs_commit(nvs_storage);
		parameter.cache = parameter.getValue();
	}
}

// 打印全部参数（CLI 命令 p）
void printParameters() {
	for (auto &parameter : parameters) {
		print("%s = %g\n", parameter.name, parameter.getValue());
	}
}

// 重置参数存储：清空 NVS 命名空间并重启（注意会连带清除同命名空间的 WiFi 凭证）
void resetParameters() {
	nvs_erase_all(nvs_storage);
	nvs_commit(nvs_storage);
	esp_restart();
}

// 从 NVS 读取字符串；键不存在时用 defaultVal 填充（供 wifi.cpp 存 SSID/密码）
// 注意：outLen 为缓冲区容量，读取失败时会按 outLen-1 截断并补 '\0'。
void nvsGetString(const char* key, char* out, size_t outLen, const char* defaultVal) {
	size_t required = outLen;
	esp_err_t err = nvs_get_str(nvs_storage, key, out, &required);
	if (err != ESP_OK) {
		strncpy(out, defaultVal, outLen - 1);
		out[outLen - 1] = '\0';
	}
}

// 写入字符串到 NVS 并提交（供 wifi.cpp 保存 SSID/密码）
void nvsPutString(const char* key, const char* val) {
	nvs_set_str(nvs_storage, key, val);
	nvs_commit(nvs_storage);
}
