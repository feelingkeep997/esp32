// ============================================================================
// safety.cpp —— 失控保护与安全策略
//
// 职责:
//   在飞行中出现异常时接管控制，尽量让飞行器安全落地或停机。由 control() 在
//   输入解释之后、姿态控制之前统一调用 failsafe()，依次执行：
//     rcLossFailsafe()    遥控信号丢失 → 下降
//     webRCLossFailsafe() 网页遥控失联   → 下降
//     autoFailsafe()      人工接管：一旦检测到杆量变化就退出 AUTO
//     invertedFailsafe()  机体倒置超时   → 直接停机
//     batteryFailsafe()   低电量分级处理（警告 / 自动上锁 / 强制下降）
//
// 关键逻辑:
//   - descend() 是统一的"受控下降"：切到 AUTO、保持当前偏航、姿态回平，
//     把推力按 descendTime 线性递减到 0，减到 0 即自动上锁；
//   - batteryFailsafe 区分"飞行中"与"未飞行"两套阈值：
//       飞行中 < VBAT_CRITICAL_THRESHOLD(3.0V) → 强制下降并锁定(l3Latched)；
//       未飞行 < VBAT_WARN_THRESHOLD(3.5V)     → 自动上锁；
//     所有动作都需持续 BATTERY_ACTION_DEBOUNCE_TIME(0.9s) 才生效，避免瞬时压降误判。
//
// 输入/输出:
//   输入：controlTime/controlRoll/Pitch/Yaw/Throttle、attitude、batteryVoltage、armed、mode、t；
//   输出：armed / mode / thrustTarget / attitudeTarget / ratesExtra / isInverted。
//
// 重要参数（可由参数系统改写）:
//   rcLossTimeout  遥控丢失判定时长 s（SF_RC_LOSS_TIME）
//   descendTime    下降过程时长 s（SF_DESCEND_TIME）
//   常量：WEB_RC_LOSS_TIMEOUT_MS(8000) / INVERTED_COS_THRESHOLD(-0.7) / INVERTED_TIMEOUT(1.5)
//
// 边界情况与潜在风险:
//   - descend() 内多次写 attitudeTarget 与 thrustTarget，且会重置所有 PID；
//     若与 MAVLink 的 SET_ATTITUDE_TARGET 同时作用可能互相覆盖（AUTO 模式下以此处为准）。
//   - autoFailsafe 用"杆量变化"退出 AUTO：遥控抖动/回中误差都可能提前退出自动下降，
//     故下降场景应配合遥控丢失判定一起使用。
//   - invertedFailsafe 判定倒置超过 1.5s 直接停机（不尝试翻转自救），属"保机不保飞"策略。
//   - batteryFailsafe 的 l3Latched 只在解锁状态下复位；重新解锁前若电压已恢复也需先上锁。
//   - batteryVoltage 为 NAN（未接电池检测）时直接返回，不触发任何保护。
// ============================================================================

#include "globals.h"
#include "cf_math.h"
#include "esp_timer.h"
#include <string.h>
#include <stdio.h>

bool isInverted = false;      // 当前是否处于倒置状态（供 LED 告警使用）
float rcLossTimeout = 1;      // 遥控丢失判定时长（s），参数 SF_RC_LOSS_TIME
float descendTime = 10;       // 受控下降时长（s），参数 SF_DESCEND_TIME
#define WEB_RC_LOSS_TIMEOUT_MS 8000UL // 网页遥控失联判定（ms）
#define INVERTED_COS_THRESHOLD -0.7f  // 机体 Z 轴与世界向上方向夹角余弦阈值（<-0.7 视为倒置）
#define INVERTED_TIMEOUT        1.5f  // 倒置持续多久后停机（s）

// 保护总入口：按顺序执行各类保护（每个控制周期调用一次）
void failsafe() {
	rcLossFailsafe();
#if WEB_RC_ENABLED
	webRCLossFailsafe();
#endif
	autoFailsafe();
	invertedFailsafe();
	batteryFailsafe();
}

// 遥控信号丢失保护：仅在已解锁、且当前不是网页遥控接管时才判定
// controlTime 为 0 表示从未收到过遥控数据（不上电就判丢失会误触发）
void rcLossFailsafe() {
	if (controlTime == 0) return;
	if (!armed) return;
#if WEB_RC_ENABLED
	if (isUsingWebRC()) return; // 网页遥控接管时由 webRCLossFailsafe 负责
#endif
	if (t - controlTime > rcLossTimeout) {
		descend();
	}
}

// 受控下降：切到 AUTO，保持当前偏航、姿态回平，推力线性递减到 0 后自动上锁。
// 首次进入时会重置所有 PID 并限制初始推力，避免切换瞬间的冲击。
void descend() {
	if (mode != AUTO) { // 首次进入下降
		float currentYaw = attitude.getYaw();
		attitudeTarget = Quaternion::fromEuler(Vector(0, 0, currentYaw)); // 姿态回平，保持机头朝向
		ratesExtra = Vector(0, 0, 0);
		rollRatePID.reset(); // 清空积分，防止切换冲击
		pitchRatePID.reset();
		yawRatePID.reset();
		rollPID.reset();
		pitchPID.reset();
		yawPID.reset();
		if (thrustTarget > ALTHOLD_HOVER_THRUST) thrustTarget = ALTHOLD_HOVER_THRUST; // 限制初始推力
		mode = AUTO;
	}
	float currentYaw = attitude.getYaw();
	attitudeTarget = Quaternion::fromEuler(Vector(0, 0, currentYaw)); // 每帧保持水平目标
	thrustTarget -= dt / descendTime; // 线性递减推力
	if (thrustTarget < 0) {
		thrustTarget = 0;
		armed = false; // 推力归零即上锁
	}
}

// 人工接管：一旦检测到任意杆量发生变化，就从 AUTO 退出到 STAB，交还控制权。
// 用 static 保存上一帧杆量，油门带 0.05 死区避免抖动误判。
void autoFailsafe() {
	static float roll, pitch, yaw, throttle;
	if (roll != controlRoll || pitch != controlPitch || yaw != controlYaw || std::fabs(throttle - controlThrottle) > 0.05) {
		if (mode == AUTO) mode = STAB;
	}
	roll = controlRoll;
	pitch = controlPitch;
	yaw = controlYaw;
	throttle = controlThrottle;
}

#if WEB_RC_ENABLED
// 网页遥控失联保护：超过 WEB_RC_LOSS_TIMEOUT_MS 未收到心跳则下降，
// 并关闭网页遥控（避免恢复后突然接管）。
void webRCLossFailsafe() {
	if (!webRCEnabled || !useWebRC) return;
	if (!armed) return;
	if ((unsigned long)(esp_timer_get_time() / 1000) - webRCLastUpdate > WEB_RC_LOSS_TIMEOUT_MS) {
		print("Web RC连接丢失，启动下降\n");
		descend();
		webRCEnabled = false;
		useWebRC = false;
	}
}
#endif

// 倒置保护：机体 Z 轴与世界向上方向的夹角余弦 < -0.7（即倾角 > ~135°）视为倒置；
// 持续超过 INVERTED_TIMEOUT 则直接停机（不尝试翻转自救）。
void invertedFailsafe() {
	if (!armed) {
		isInverted = false;
		return;
	}
	Vector worldUp = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	static float invertedStartTime = 0;
	if (worldUp.z < INVERTED_COS_THRESHOLD) {
		isInverted = true;
		if (invertedStartTime == 0) invertedStartTime = t;
		if (t - invertedStartTime > INVERTED_TIMEOUT) {
			armed = false;
			thrustTarget = 0.0f;
			invertedStartTime = 0;
			print("倒置保护：停机\n");
		}
	} else {
		isInverted = false;
		invertedStartTime = 0;
	}
}

// 低电量分级保护：
//   飞行中（thrustTarget >= 0.15）电压 < 3.0V → 强制下降并锁定(l3Latched，不再恢复)；
//   未飞行  电压 < 3.5V                        → 自动上锁；
//   所有动作需持续 BATTERY_ACTION_DEBOUNCE_TIME 才生效。
// 未接电池检测（电压 < 0.5V）时不参与任何保护。
void batteryFailsafe() {
	static bool l3Latched = false;     // 是否已进入"电量告急强制下降"状态（锁定）
	static float lowSince = 0.0f;      // 低电压起始时间（0 = 未计时）
	static float l3LastNotify = 0.0f;  // 上次告急提示时间（限速打印）

	if (batteryVoltage < VBAT_ABSENT_THRESHOLD) return; // 未接电池检测，跳过
	if (!armed) { // 上锁状态：清空所有保护状态
		l3Latched = false;
		lowSince = 0.0f;
		l3LastNotify = 0.0f;
		return;
	}

	if (l3Latched) { // 已进入强制下降：每 2s 提示一次，持续下降
		if (t - l3LastNotify >= 2.0f) {
			print("电池电量告急(%.2fV)，持续降落，推力%.0f%%\n",
			      batteryVoltage, thrustTarget * 100.0f);
#if WEB_RC_ENABLED
			char warnBuf[64];
			snprintf(warnBuf, sizeof(warnBuf),
			         "电池电量告急(%.2fV) 持续降落 推力%.0f%%",
			         batteryVoltage, thrustTarget * 100.0f);
			setWebRCWarn(warnBuf);
#endif
			l3LastNotify = t;
		}
		descend();
		return;
	}

	bool flying = thrustTarget >= BATTERY_FLYING_THRUST_MIN; // 是否处于飞行（有推力）
	bool actionCondition = false; // 是否需要采取动作
	bool criticalAction = false;  // 是否为"告急"级动作（强制下降）

	if (flying) {
		if (batteryVoltage < VBAT_CRITICAL_THRESHOLD) { // 飞行中 3.0V
			actionCondition = true;
			criticalAction = true;
		}
	} else {
		if (batteryVoltage < VBAT_WARN_THRESHOLD) { // 未飞行 3.5V
			actionCondition = true;
		}
	}

	if (!actionCondition) { // 恢复正常，清除计时
		lowSince = 0.0f;
		return;
	}

	// 去抖：条件需持续 BATTERY_ACTION_DEBOUNCE_TIME 才执行，避免瞬时压降误动作
	if (lowSince == 0.0f) lowSince = t;
	if (t - lowSince < BATTERY_ACTION_DEBOUNCE_TIME) return;

	lowSince = 0.0f;
	if (criticalAction) { // 告急：锁定并强制下降
		l3Latched = true;
		l3LastNotify = 0.0f;
		print("电池电量告急(%.2fV)，进入自动降落\n", batteryVoltage);
#if WEB_RC_ENABLED
		char warnBuf[64];
		snprintf(warnBuf, sizeof(warnBuf), "电池电量告急(%.2fV) 进入自动降落", batteryVoltage);
		setWebRCWarn(warnBuf);
#endif
		descend();
		return;
	}

	// 低电量：直接上锁停机
	armed = false;
	thrustTarget = 0.0f;
	print("电量低(%.2fV)，自动上锁\n", batteryVoltage);
#if WEB_RC_ENABLED
	char warnBuf[64];
	snprintf(warnBuf, sizeof(warnBuf), "电量低(%.2fV) 已自动上锁", batteryVoltage);
	setWebRCWarn(warnBuf);
#endif
}
