// ============================================================================
// estimate.cpp —— 姿态与垂直速度估计（互补滤波 + 重力积分）
//
// 职责:
//   由 IMU 原始数据（gyro 角速度、acc 加速度）解算出机体的姿态四元数 attitude，
//   并额外估计一个"近似垂直速度" velZ 供无气压计的 ALTHOLD 使用。
//
// 关键逻辑（每个控制周期按顺序执行）:
//   applyGyro()   陀螺仪积分：rates = LPF(gyro - levelGyroBias)，attitude 按 rates*dt 旋转；
//   applyAcc()    加速度计修正：仅在"已着陆且静止"(landed) 时，用重力方向缓慢纠正姿态
//                 （accWeight 很小，属长期修正，避免机动时加速度干扰）；
//   applyLevel()  水平修正：飞行中按倾角/杆量加权，把姿态缓慢拉回水平，
//                 并可在线学习陀螺仪零偏（levelGyroBias，带限幅）；
//   estimateVelZ()垂直速度：把机体加速度旋到世界系取 Z 轴、去重力后积分，并加泄漏。
//
// 输入/输出:
//   输入：全局 gyro / acc（来自 imu.cpp）、controlRoll/Pitch（杆量门控）、motorsActive()；
//   输出：attitude（四元数）、rates（滤波后角速率）、landed、velZ、levelGyroBias。
//
// 重要参数:
//   accWeight          加速度计修正强度（默认 0.003，很小）
//   levelWeight/levelMaxTilt/levelGateThreshold/levelBiasGain  水平修正相关（默认多为 0，即关闭）
//   ratesFilter.alpha  角速率低通系数（参数 EST_RATES_LPF_A）
//   ALT_VEL_LEAK       垂直速度泄漏率；ALT_ACC_LPF_ALPHA 垂直加速度低通
//
// 边界情况与潜在风险:
//   - 纯陀螺积分必然漂移，靠 applyAcc（仅落地静止）与 applyLevel（默认关闭）抑制；
//     若两者都失效，长时间飞行会缓慢倾斜。
//   - landed 判据为"电机不转 且 |acc| 接近 1g"，起飞瞬间/自由落体时会误判，
//     进而误触发 applyAcc 修正——accWeight 很小正是为此留的余量。
//   - applyLevel 在 tilt < 0.1° 时直接返回，避免小误差反复微调；
//     levelGyroBias 有 ±3° 的限幅，防止错误修正累积成大幅偏置。
//   - estimateVelZ 的速度估计带泄漏（缓慢归零），长时间（>30s）必然漂移，
//     只能用于定高阻尼，不能当作真实高度/速度使用。
// ============================================================================

#include "globals.h"
#include "cf_math.h"

float accWeight = 0.003f;                // 加速度计对姿态的修正权重（很小，长期修正）
float levelWeight = 0;                   // 水平修正权重（0 = 关闭）
float levelMaxTilt = radians(30);        // 水平修正允许的最大倾角
float levelGateThreshold = 0.2f;         // 杆量门控阈值：杆量越大越不修正（机动时不干扰）
float levelBiasGain = 0;                 // 陀螺零偏在线学习增益（0 = 关闭）
Vector levelGyroBias(0, 0, 0);           // 在线学习的陀螺零偏
LowPassFilter<Vector> ratesFilter(0.2);  // 角速率低通滤波（抑制陀螺噪声）

// ============== 垂直速度估计（无气压计近似定高用）==============
// 把机体加速度旋转到世界系取 Z 轴积分得到垂直速度，再减去重力。
// 没有高度参考，纯积分必然漂移，因此加泄漏项（每秒衰减 ALT_VEL_LEAK 比例），
// 让速度估计缓慢回归零：短时间（几秒~几十秒）内足够准，供 ALTHOLD 阻尼使用。
float velZ = 0;
#define ALT_VEL_LEAK 0.15f    // 速度泄漏率（1/s），越大越快回归零但定高阻尼越弱
#define ALT_ACC_LPF_ALPHA 0.05 // 垂直加速度低通（约10Hz），抑制电机/机架振动
static LowPassFilter<float> accZFilter(ALT_ACC_LPF_ALPHA);

// 估计主入口：每个控制周期调用一次，顺序固定（先陀螺积分，再加速度/水平修正，最后速度）
void estimate() {
	applyGyro();
	applyAcc();
	applyLevel();
	estimateVelZ();
}

// 垂直速度估计：世界系 Z 轴加速度积分 + 泄漏
// 输入：attitude（用于把 acc 旋到世界系）、acc、landed、dt
// 输出：velZ（向上为正，m/s）
// 注意：无高度参考，纯积分必然漂移，靠泄漏项缓慢归零（见文件头说明）
void estimateVelZ() {
	if (landed) { // 已落地：速度归零并复位滤波器，避免地面噪声累积
		velZ = 0;
		accZFilter.reset();
		return;
	}

	Vector accWorld = attitude.conjugate(acc); // 机体系 → 世界系
	float az = accZFilter.update(accWorld.z - ONE_G); // 去掉重力，得到净垂直加速度（并低通）
	velZ += az * dt;                                   // 积分成速度
	velZ -= velZ * ALT_VEL_LEAK * dt;                  // 泄漏项：缓慢回归零
}

// 陀螺仪积分：把角速度积分到姿态上（姿态解算的主体）
// 输入：gyro（原始角速度）、levelGyroBias（在线零偏）、dt
// 输出：rates（滤波后角速率）、attitude（更新后的姿态）
void applyGyro() {
	rates = ratesFilter.update(gyro - levelGyroBias);              // 去零偏 + 低通
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(rates * dt)); // 增量旋转
}

// 加速度计修正：仅在"已着陆且静止"时，用重力方向缓慢纠正姿态漂移。
// landed 判据：电机不转 且 |acc| 接近 1g（在 ±10% 内）。
// 注意：飞行中/机动时加速度不等于重力，故此处不做修正（交给陀螺积分）。
void applyAcc() {
	float accNorm = acc.norm();
	landed = !motorsActive() && std::fabs(accNorm - ONE_G) < ONE_G * 0.1f;
	if (!landed) return;
	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude); // 机体 Z 轴在世界系方向
	Vector correction = Vector::rotationVectorBetween(acc, up) * accWeight; // 误差 * 小权重
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(correction));
}

// 水平修正：飞行中按"倾角 + 杆量门控"加权，把姿态缓慢拉回水平，并可学习陀螺零偏。
// 默认 levelWeight=0（关闭），故正常情况下本函数只做落地复位。
void applyLevel() {
	if (landed) { // 已落地：清空零偏学习
		levelGyroBias = Vector(0, 0, 0);
		return;
	}
	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	float tilt = acos(std::clamp(up.z, -1.0f, 1.0f)); // 当前倾角
	if (tilt < radians(0.1f)) return; // 已接近水平，无需修正
	// 倾角越大修正越弱（levelMaxTilt 外为 0），避免大角度时干扰控制
	float dynamicWeight = levelWeight * std::clamp(1.0f - tilt / levelMaxTilt, 0.0f, 1.0f);
	// 杆量门控：遥控打杆越大越不修正，避免与飞手意图对抗
	float stickDeflection = std::max(std::fabs(controlRoll), std::fabs(controlPitch));
	float stickGate = std::clamp(1.0f - stickDeflection / levelGateThreshold, 0.0f, 1.0f);
	dynamicWeight *= stickGate;
	Vector error = Vector::rotationVectorBetween(Vector(0, 0, 1), up); // 水平误差旋转矢量
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(error * dynamicWeight));
	levelGyroBias += error * (levelBiasGain * stickGate); // 在线学习零偏
	// 零偏限幅 ±3°，防止错误修正累积
	float biasNorm = levelGyroBias.norm();
	const float biasLimit = radians(3.0f);
	if (biasNorm > biasLimit) levelGyroBias = levelGyroBias * (biasLimit / biasNorm);
}
