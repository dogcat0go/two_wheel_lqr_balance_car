/*
 * @Author: LCOIT dogcat.let@gmail.com
 * @Date: 2026-08-03 13:59:58
 * @LastEditors: LCOIT dogcat.let@gmail.com
 * @LastEditTime: 2026-08-21 17:16:13
 * @FilePath: /fishbot_esp32_mt_example/include/config.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

// ============================================================
// 唯一改参处：硬件接线、方向校准、几何、控制周期、安全阈值
// 坐标系约定：车体坐标系，正 = 车体前进；pitch 正 = 车体前倾。
// 上层（PID/LQR/运动学/ROS）只认这个约定；
// 电机极性、编码器极性由 kWheelDir 在硬件边界层吸收。
// 内部一律 SI 单位（m、m/s、rad、rad/s），只在 ROS 消息边界换算。
// ============================================================

#include <stdint.h>

namespace cfg {

// ---- 轮序 ----
constexpr int kLeft  = 0;
constexpr int kRight = 1;

// ---- 编码器接线 (PCNT 单元号 = 轮序) ----
constexpr int kEncoderPinA[2] = {32, 26};
constexpr int kEncoderPinB[2] = {33, 25};

// ---- 电机接线 (MCPWM 通道号 = 轮序) ----
constexpr int kMotorPinA[2] = {23, 13};
constexpr int kMotorPinB[2] = {22, 12};

// ---- IMU (LSM6DS3/MPU6050 兼容库，I2C) ----
// 原理图：MPU_SDA=IO18, MPU_SCL=IO19（勿用默认 21/22，22 是电机脚）
constexpr int      kImuSda      = 18;
constexpr int      kImuScl      = 19;
constexpr uint32_t kImuI2cHz    = 400000;
// 平衡自由度绕 Y 轴：pitch 由 ax 算、角速度取 gyroY；前倾正负装反了翻这里
constexpr float    kPitchSign   = 1.0f;
// yaw 角速度取陀螺 Z；正 = 车体逆时针。装反或与轮式差速符号不一致时翻这里
constexpr float    kYawSign     = 1.0f;
// 互补滤波：越接近 1 越信陀螺（响应快、易漂），越小越信加速度计（抗漂、易被振动污染）
constexpr float    kPitchGyroCoef = 0.98f;

// ---- 方向校准（阶段0标定）----
// 判据：applyRawPwm(+duty) 时轮子朝“车前进”方向转，且 speed() > 0。
constexpr float kWheelDir[2] = {1.f, 1.f};

// ---- 几何/换算 ----
// kMPerTick 已按新电机/编码器实测重标（走直线 1m 反推，约 7450 ticks/m）。
constexpr float kMPerTick       = 0.134228e-3f; // m/tick（正值）
constexpr float kWheelRadiusM   = 0.0375f;      // 实测轮半径 3.75cm
constexpr float kWheelDistanceM = 0.16f;       // 轮距

// ---- LQR 物理参数（离线 tools/lqr_gain_design.py 读取；改 M/m/l/I/r/dt 后必须重算 K）----
// 称重：docs/stage5_physical_params_table.md（2026-08-16）。旧估算 l=0.012 已作废。
constexpr float kBodyMassKg       = 0.70f;   // M：去轮、留电机
constexpr float kWheelMassKg      = 0.135f;  // m：两轮合计，转子=0 含车轮得等效质量
constexpr float kComHeightM       = 0.0204f; // l：车体质心到轮轴
constexpr float kBodyPitchInertia = 5.63e-4f;    // I_cm；对照可改 5.63e-4
constexpr float kGravity          = 9.81f;
// 斜坡开环（S1）：串口 q 喂 α_inj（度），h 调 gain；两半共用 sin_eff。不写 KF。
constexpr float kTotalMassKg = kBodyMassKg + kWheelMassKg;
constexpr float kMgrNm = kTotalMassKg * kGravity * kWheelRadiusM; // τ_ff = MGR·sin_eff
constexpr float kThetaEqGain = kTotalMassKg * kWheelRadiusM
    / (kBodyMassKg * kComHeightM); // θ_eq = K_EQ·sin_eff ≈ 2.19
constexpr float kSlopeGain         = 0.72f;    // S1 初值；串口 h
constexpr float kSlopeSinDeadzone  = 0.015f;  // |sin_eff| 超此禁止 HOLD、冻 Trim
constexpr float kSlopeAlphaMaxDeg  = 12.0f;   // 串口 q 限幅

// m 3：LQR 增益默认值 (N·m/状态)，上电载入；串口 k/p/d/y/w 可改，断电丢失。
// tools/lqr_gain_design.py 打印「串口：k ...」可直接粘贴；--write 只改这里的默认。
constexpr float kLqrPitch     = 1.8511479f;
constexpr float kLqrPitchRate = 0.096101144f;
constexpr float kLqrPos       = 0.1472623f;
constexpr float kLqrVel       = 0.42034732f;

// ---- 测速 ----
constexpr int   kSpeedDiffWindow = 4;    // 差分窗口(采样数)，窗口时长 = N*控制周期
constexpr float kSpeedLpfAlpha   = 0.3f; // 速度低通系数 (0~1]，1 = 不滤波
// 遥测用：均速的慢直流分量 v_dc += α(v-v_dc)，α=dt/τ；τ≈2s 抹掉对摇、留下净爬行
constexpr float kVdcLpfAlpha     = 0.0025f; // 200Hz → τ=dt/α=2s（原 0.001→5s）

// ---- 电流传感（ACS712-05B ±5A；下标同轮序 kLeft/kRight；须用 ADC1 脚）----
// 灵敏度厂家标称 185 mV/A；零点上电各校一次。极性反了翻对应 kCurrentSign[i]。
// currentRaw = 本拍；current = 一阶低通（压 PWM 纹波/噪声）。
constexpr int   kCurrentAdcGpio[2]     = {34, 35}; // L=GPIO34, R=GPIO35
constexpr float kCurrentSensVPerA[2]   = {0.185f, 0.185f}; // V/A
constexpr float kCurrentSign[2]        = {-1.0f, 1.0f};    // +1 = 该轮前进为正电流
constexpr float kCurrentLpfAlpha[2]    = {0.15f, 0.15f};   // 低通；越小越稳、越滞后
constexpr int   kCurrentZeroSamples[2] = {100, 100};       // 上电零点平均次数
// 零点合理性带：ACS712 零点≈VCC/2=2.5V，标定结果出带=传感器/线故障，拒绝采纳
constexpr float kCurrentZeroMinV = 2.0f;
constexpr float kCurrentZeroMaxV = 3.0f;
// 在线零点跟踪（T1 实测：上电偏移约 -17mA，跑 2min 温漂 L-18/R+24mA 反向）。
// 门：PWM=0 且 |轮速|<eps 且归零后满 settle 拍（AT8236 PWM=0 为滑行，转轮无电流）。
constexpr float kCurrentZeroTrackAlpha  = 0.0025f; // τ=dt/α≈2s；0=关
constexpr int   kCurrentZeroSettleTicks = 40;      // PWM 归零后停等 0.2s
constexpr float kCurrentZeroSpeedEps    = 0.01f;   // m/s，轮速门
constexpr float kCurrentMaxA = 2.5f;   // 开环 |I_ref| 上限 (A)
constexpr float kCurrentFaultA = 6.0f; // 读数合理性上限 (A)：超 ACS712-05B ±5A 量程=测量失效，PI 冻结防追幻影
constexpr float kCurrentKp   = 80.0f;  // 电流 PI，%/A
constexpr float kCurrentKi   = 160.0f;  // 电流 PI，%/(A·s)
// 轮端有效 Kt（2026-08-14 挂重）。m 2：I_ref = τ / Kt
constexpr float kTorquePerAmp[2] = {0.23f, 0.23f}; // N·m/A
// 平衡仍输出 %，m 2 再换成力矩：堵转约 8% ↔ 0.3 A → τ≈0.066 → 0.008 N·m/%
constexpr float kPctToTorque = 0.003f; // N·m per %；仅 m 2
constexpr float kMaxTorque   = 0.40f;  // 单轮力矩饱和 (N·m)，m 2 / m 3
constexpr float kLqrMaxSlew  = 32.0f;   // m 3 斜率 (N·m/s)
constexpr float kLqrPosTermLimit = 0.12f; // m 3 位置项 anti-windup (N·m)

// ---- 执行器硬限幅 ----
constexpr float kMaxDuty = 100.0f; // PWM 占空比上限 (%)，硬件边界层

// ---- 死区前馈（测门槛时必须先关；开着平衡时左右务必相同，否则差速撕平衡）----
// 测法：kMotorDeadband={0,0} 烧录 → 架空 → el/er 从小往上找启动门槛 →
// 再填实测门槛的 60~70%，且左右先填同一个值验证平衡，再考虑分填。
// 见 docs/stage4_velocity_loop.md#deadband-bench
constexpr float kMotorDeadband[2] = {5.0*0.7f, 5.0*0.7f}; // %，默认关，先恢复可站

// applyRawPwm（m 0 / m 1）：|duty|<此值则 PWM=0，让静摩擦钉住
constexpr float kMotorCmdEps      = 3.0f;         // % 指令死区；补死区后需抬高打破平衡点极限环
// applyTorque 保险丝（分轮）；车辆进 HOLD 由 HoldPolicy 决定，stage2/5 执行器侧填 0
constexpr float kTorqueEps        = kMotorCmdEps * kPctToTorque; // N·m/轮，3%×0.003=0.009

// ---- 死区自标定（串口 b 触发；架空标定：轮子悬空，逐轮正反斜坡找起转门槛）----
// 架空只测负载无关的电机/驱动死区；地面摩擦差是变量，交偏航积分动态补，别固化进此处。
// 结果存 NVS(namespace motorcal, key dbL/dbR)，上电覆盖 kMotorDeadband；电压波动残差交电流环 I 项。
constexpr float   kCalibRampPctPerTick = 0.15f; // 每拍 duty 增量 (%)，200Hz 约 30%/s
constexpr float   kCalibDutyMaxPct     = 40.0f; // 爬升上限 (%)，到顶仍不转则记上限并报警
constexpr int32_t kCalibTicksThresh    = 6;     // 判起转的编码器增量 (ticks，约 0.8mm)；空载灵敏
constexpr int     kCalibSettleTicks    = 80;    // 阶段间停等拍数 (~0.4s) 让轮停稳
// 存库前欠补系数。0.7 实测欠补：地面起转 duty 8~10%、τ 攒到 3~6×eps 才蹬出（stage6 E4 log），
// 粘着占比 20% 的张弛极限环即由此来。0.85 折中；若出现高频抖动(过补)退 0.80，仍粘再试 0.90。
constexpr float   kCalibScale          = 0.85f;
// r 前探轮（DeadbandCalibrator::requestArm）。false=发 r 直接武装，不推轮。
constexpr bool    kPrearmEnable        = true;
constexpr float   kPrearmProbeA        = 0.30f; // 探轮电流 (A)，两轮同向
constexpr int     kPrearmHoldTicks     = 30;    // 最长 150ms
constexpr int     kPrearmSettleTicks   = 10;    // 起步停等 (~50ms)
constexpr float   kPrearmAbortPitchDeg = 12.0f; // 离 trim 超过此角不探 / 探中停
constexpr float   kPrearmAbortOmega    = 0.8f;  // rad/s，探中角速度过大则停

// ---- 控制周期（频带分离：控制环独占 Core1，通信在 Core0）----
constexpr uint32_t kCtrlHz         = 200;
constexpr uint32_t kCtrlPeriodMs   = 1000 / kCtrlHz; // 5ms
constexpr float    kCtrlDt         = 1.0f / kCtrlHz; // 固定 dt，LQR 离散化用同一值
constexpr int      kCtrlCore       = 1;
constexpr int      kCommCore       = 0;
constexpr uint32_t kTelemetryMs    = 200; // 遥测打印周期
// 上电模式：0 开环 / 1 平衡PWM / 2 平衡电流(手调%) / 3 平衡电流(LQR N·m)
// 未 armed 时平衡模式也不出力，须扶正后发 r。试 LQR 改成 3。
constexpr uint8_t  kBootMode       = 2;

// ---- 安全层（阶段2，必须先于控制算法测通）----
// 当前执行器接口是占空比(%)，阶段5 接 τ→PWM 后改成 N·m
constexpr float    kMaxEffort       = 100.0f;   // 输出饱和 (%)，平衡调参期先压低防炸机
// 斜率限幅 (%/s)。倒塌特征时间只有 33ms，而 3000%/s 走完 -60→+60 要 40ms，
// 限幅本身就成了主导滞后，故放到 12000（每个 5ms 拍可走满 60%），实际只剩饱和起作用
constexpr float    kMaxEffortSlew   = 12000.0f;
constexpr float    kFallAngleRad    = 0.52f;   // ~30°
constexpr uint32_t kFallHoldMs      = 20;      // 连续超角 20ms（约 4 拍@200Hz）才确认摔倒
constexpr uint32_t kImuTimeoutMs    = 100;     // IMU 数据过期
constexpr uint32_t kCmdTimeoutMs    = 500;     // 通信断链：清速度目标，不切电机（摔倒/IMU 才硬停）

// ---- 阶段3 平衡状态反馈（k 即 ∂u/∂x，已含符号；见 docs/stage34_gain_init_theory.md）----
// 设计点：ω_n = 2λ ≈ 57rad/s, ζ = 0.8 → T_d = k_θ̇/k_θ = 0.022s。
// T_d 由 l 决定，可信；k_θ 的绝对值要除以未实测的"占空比→加速度"标度 K_a，
// 按 K_a 的合理区间 k_θ 落在 150~730，故初值取偏保守的 200，靠 p 命令按 ×1.5 往上爬。
// 调参时 k_θ̇ 始终跟着算：k_θ̇ = 0.022 * k_θ。
constexpr float kPitchTrimDeg    = 2.6f;   // 机械平衡角(度)，与串口 t 同单位；内部再转 rad
constexpr float kGainPitch       = 500.0f; // k_θ,   %/rad     （前倾要正输出，>0）
constexpr float kGainPitchRate   = 10.0f;   // k_θ̇,   %/(rad/s)
constexpr float kGainIntegPitch  = 5.63e-4f;  // k_iθ,  %/(rad·s)
constexpr float kIntegTermLimit  = 15.0f; // 积分项限幅 (%)

// ---- 阶段4 速度环/位置环（同一状态反馈里多两项，非级联）----
// 初值/时标分离：docs/stage34_gain_init_theory.md ；调参步骤：docs/stage4_velocity_loop.md
// 实车：k_vel≈10 稳、20 开始晃；位置环先保持 0。
constexpr float kGainVel         = 20.0f;  // k_ṡ,   %/(m/s)
constexpr float kGainPos         = 10.0f;   // k_s,   %/m（当积分用，靠限幅防 windup）
constexpr float kPosTermLimit    = 25.0f;  // 位置项 anti-windup 限幅 (%)：饱和后停止累积 pos_ref
constexpr float kMaxLinearMps    = 0.5f;  // 速度目标限幅 (m/s)，防手滑给过大目标

// 速度→倾角前馈：θ_ref = trim + kff·v_cmd。参考生成，不是反馈、不是软停。
// 串口 g 输入 deg/(m/s)，内部转 rad；从小往上调，过大会给速瞬间猛冲。
constexpr float kGainVelToPitch  = 0.1745f; // k_ff, rad/(m/s)（=10°/(m/s)，实车标定值）
constexpr float kFfPitchLimitRad = 0.26f; // 前馈后 θ_ref 相对 trim 的限幅 (rad,≈15°)，防大 vref 顶到摔倒角
constexpr float kVelSlewMps2     = 1.0f;  // v_smooth 斜率 (m/s²)；软起/软停
constexpr float kYawSlewRps2     = 3.0f;  // ω_smooth 斜率 (rad/s²)

// ---- 偏航 / 走直线（差速项，不进 BalanceController；补左右轮摩擦不齐）----
// stage2：z/n 仍走 %。stage5：z 航向 P；n = 轮速差 P（k_sync），不再用陀螺 ω_z 做 D。
constexpr float kGainYaw         = 20.0f; // k_ψ,  %/rad（stage2；0=不锁航向）
constexpr float kGainYawRate     = 20.0f; // k_ψ̇, %/(rad/s)（stage2 的串口 n）
constexpr float kGainYawNm       = 0.03f; // stage5 的 z，N·m/rad；0=不锁航向
constexpr float kGainYawRateNm   = kGainYawRate * kPctToTorque; // 旧 stage5 陀螺 D，现不用
constexpr float kGainWheelSync   = 0.5f;  // stage5 的 n：k_sync，N·m/(m/s)；串口 n 改此项
constexpr float kGainYawIntegNm  = 0.02f; // stage5 航向积分 ki，N·m/(rad·s)；保守初值补左右摩擦差，串口 j 调
constexpr float kYawIntegTermLimit = 0.08f; // 航向积分项限幅 (N·m)，anti-windup
constexpr float kMaxAngularRps   = 1.5f;  // 角速度目标限幅 (rad/s)
constexpr float kYawCmdEps       = 1e-3f; // |az| 小于此视为松杆，进入 heading hold
constexpr float kYawErrLimitRad  = 20.0f * 0.01745329252f; // heading P：|wrap(ψ_ref-ψ)| 钳位，假航向不按 180° 拧

// ---- in-position（HoldPolicy：TRACK/CONFIRM/HOLD；docs/in_position_hold.md 第1步）----
// 位置门 pos_in<=0 关闭。enter_ticks<=0 整策略关闭。
constexpr float kHoldThetaInDeg  = 0.4f;
constexpr float kHoldThetaOutDeg = 0.8f;
constexpr float kHoldOmegaIn     = 0.12f; // rad/s
constexpr float kHoldOmegaOut    = 0.25f;
constexpr float kHoldVelIn       = 0.02f; // m/s
constexpr float kHoldVelOut      = 0.04f;
constexpr float kHoldPosIn       = 0.0f;  // m；0=本步关闭
constexpr float kHoldPosOut      = 0.06f;
constexpr float kHoldVCmdEps     = 0.01f; // m/s
constexpr int   kHoldEnterTicks  = 20;    // 100 ms @ 200 Hz
constexpr int   kHoldSettleTicks = 40;    // HOLD 进入后再观望 200ms；0=不用
// TRACK 死区前馈常开（粘着段也补）；关补偿只在 HOLD。kTorqueEps 仍给 HoldPolicy 力矩门。
// E3 v_dc trim 伺服：bias += −k·v_dc·dt，每净爬 1m 拧回 k rad。
// 收敛时标 τ = k_vel/(k_pitch·k) ≈ 0.42/(1.85·k)：k=0.015 → τ≈15s。串口 u 在线改，u 0 关。
constexpr float kTrimServoK        = 0.015f; // rad/m
constexpr float kTrimServoLimitDeg = 2.0f;   // bias 限幅；顶满说明另有故障

// ---- micro-ROS WiFi（与 src/main.cpp / 历史工程一致；只跑在 Core0）----
// true：遥测/应答经 /fishbot/log 转发，命令走 /fishbot/cmd（电脑端 tools/fishbot_wifi_bridge.py）
// false：不上电 WiFi/micro-ROS，只留 USB 串口（查编码器/电流干扰时用）
constexpr bool        kEnableWifi   = true;
// agent 电脑 IP:PORT 须与 micro_ros_agent 监听一致；SSID/密码可改成现场热点
constexpr const char* kWifiSsid     = "CHY";
constexpr const char* kWifiPassword = "13705558902";
constexpr const char* kAgentIp      = "192.168.5.62";
constexpr uint16_t    kAgentPort    = 8888;
constexpr const char* kRosNodeName  = "goudan_bot_balance";
constexpr const char* kCmdVelTopic  = "/cmd_vel";
// 串口文本协议 WiFi 透传（与 USB 串口同一套 m/p/d/...；电脑端 tools/fishbot_wifi_bridge.py）
constexpr const char* kSerialCmdTopic = "/fishbot/cmd"; // std_msgs/String 下行
constexpr const char* kSerialLogTopic = "/fishbot/log"; // std_msgs/String 上行遥测/应答

} // namespace cfg
