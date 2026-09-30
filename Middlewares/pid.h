// ============================================================================
// pid.h —— 单路 PID 控制器（带积分限幅与 D 项低通）
//
// 职责:
//   提供飞控中最基础的闭环控制单元。本项目共实例化 6 个 PID：
//     - 角速率环：rollRatePID / pitchRatePID / yawRatePID（误差 = 角速度 rad/s）
//     - 姿态环：  rollPID / pitchPID / yawPID（误差 = 姿态角/旋转矢量，rad）
//   全部在 control.cpp 中构造，增益可由参数系统（CTL_*_P/I/D）在线改写。
//
// 关键逻辑:
//   dt 由全局时间基准 t 与上一次调用时刻 prevTime 之差得到；
//   仅在 0 < dt < dtMax 时才做积分与微分（异常 dt 视为一次断点，清零 I/D 防爆），
//   输出 = p*error + clamp(i*integral, ±windup) + d*derivative。
//
// 输入/输出:
//   update(error) -> 控制量（float，单位随使用场景而定）。
//   内部维护 prevError / prevTime / integral / derivative / lpf 状态。
//
// 重要参数:
//   p, i, d   比例/积分/微分增益。
//   windup    积分项限幅（抗积分饱和）；为 0 时 I 项被完全钳死（等于关闭积分）。
//   dAlpha    D 项低通滤波系数（1 = 不过滤；越小越平滑但相位滞后越大）。
//   dtMax     单步允许的最大时间间隔（s），默认 0.1；超过则认为控制断流并复位 I/D。
//
// 边界情况与潜在风险:
//   - 依赖全局 t（外部时钟），必须保证每个控制周期都调用 update()；否则 dt 累积过大会
//     触发 dt <= 0 || dt >= dtMax 分支，导致积分器被清零（表现为"松杆回中"）。
//   - reset() 把 prevTime/prevError 置 NAN，下一帧 dt 为 NAN，会再次走复位分支，
//     属于刻意设计（保证复位后第一帧不产生 I/D 冲击）。
//   - 微分项对噪声敏感，故默认经 lpf 平滑；若把 dAlpha 设为 1 需自行确保输入干净。
//   - 未做输出限幅，饱和保护由调用方（controlTorque 的 desaturate/clamp）负责。
// ============================================================================

#pragma once

#include "lpf.h"
#include "cf_math.h"

// 全局时间基准（定义在 time.cpp，由 step() 每个控制周期推进）
extern float t;

class PID {
public:
	float p, i, d;      // 比例、积分、微分增益
	float windup;       // 积分项限幅（抗积分饱和）
	float dtMax;        // 允许的最大 dt（s），超过即判定控制断流

	float derivative = 0; // 最近一次计算出的微分项（已滤波）
	float integral = 0;   // 累积积分项

	LowPassFilter<float> lpf; // D 项低通滤波器

	PID(float p, float i, float d, float windup = 0, float dAlpha = 1, float dtMax = 0.1) :
		p(p), i(i), d(d), windup(windup), dtMax(dtMax), lpf(dAlpha) {}

	// 送入当前误差，返回 PID 输出（同时推进内部状态）
	float update(float error) {
		float dt = t - prevTime; // 与上次调用的时间差

		if (dt > 0 && dt < dtMax) {
			integral += error * dt;                                  // 累积积分
			derivative = lpf.update((error - prevError) / dt);       // 求微分并低通
		} else {
			// 首次调用（prevTime=NAN）或时间异常跳变：本帧不做 I/D，避免数值爆炸
			integral = 0;
			derivative = 0;
		}

		prevError = error;
		prevTime = t;

		return p * error + std::clamp(i * integral, -windup, windup) + d * derivative;
	}

	// 复位控制器状态（用于解锁/切换模式/失控保护等场景，避免残留积分造成突跳）
	void reset() {
		prevError = NAN;
		prevTime = NAN;
		integral = 0;
		derivative = 0;
		lpf.reset();
	}

private:
	float prevError = NAN; // 上一次误差（首次为 NAN，用于识别"首帧"）
	float prevTime = NAN;  // 上一次调用时间（首次为 NAN）
};
