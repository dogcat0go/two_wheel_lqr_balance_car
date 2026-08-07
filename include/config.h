/*
 * @Author: LCOIT dogcat.let@gmail.com
 * @Date: 2026-08-03 13:59:58
 * @LastEditors: LCOIT dogcat.let@gmail.com
 * @LastEditTime: 2026-08-07 18:57:21
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
constexpr float kWheelDistanceM = 0.175f;       // 轮距

// ---- 整车质量分布（阶段3 增益推导用，见 docs/balance_gain_theory.md）----
// 电机+轮 0.515kg 基本位于轮轴高度，上层仅 0.26kg，故质心几乎贴着轮轴：
// 质心离地约 48mm，减去轮轴高 37.5mm，l 约 12mm（区间 6~16mm）。
// lambda = sqrt(g/l) ≈ 30 rad/s，倒塌特征时间仅 33ms —— 属于"矮而快"的摆。
constexpr float kComHeightM = 0.012f; // 质心到轮轴，估算值；抬高电池可显著变好控

// ---- 测速 ----
constexpr int   kSpeedDiffWindow = 4;    // 差分窗口(采样数)，窗口时长 = N*控制周期
constexpr float kSpeedLpfAlpha   = 0.3f; // 速度低通系数 (0~1]，1 = 不滤波

// ---- 执行器硬限幅 ----
constexpr float kMaxDuty = 100.0f; // PWM 占空比上限 (%)，硬件边界层

// ---- 死区前馈（测门槛时必须先关；开着平衡时左右务必相同，否则差速撕平衡）----
// 测法：kMotorDeadband={0,0} 烧录 → 架空 → el/er 从小往上找启动门槛 →
// 再填实测门槛的 60~70%，且左右先填同一个值验证平衡，再考虑分填。
// 见 docs/stage4_velocity_loop.md#deadband-bench
constexpr float kMotorDeadband[2] = {5.0*0.7f, 5.0*0.7f}; // %，默认关，先恢复可站
constexpr float kMotorCmdEps      = 3.0f;         // % 指令死区，以下不出力；补死区后需抬高打破平衡点极限环

// ---- 控制周期（频带分离：控制环独占 Core1，通信在 Core0）----
constexpr uint32_t kCtrlHz         = 200;
constexpr uint32_t kCtrlPeriodMs   = 1000 / kCtrlHz; // 5ms
constexpr float    kCtrlDt         = 1.0f / kCtrlHz; // 固定 dt，LQR 离散化用同一值
constexpr int      kCtrlCore       = 1;
constexpr int      kCommCore       = 0;
constexpr uint32_t kTelemetryMs    = 200; // 遥测打印周期

// ---- 安全层（阶段2，必须先于控制算法测通）----
// 当前执行器接口是占空比(%)，阶段5 接 τ→PWM 后改成 N·m
constexpr float    kMaxEffort       = 60.0f;   // 输出饱和 (%)，平衡调参期先压低防炸机
// 斜率限幅 (%/s)。倒塌特征时间只有 33ms，而 3000%/s 走完 -60→+60 要 40ms，
// 限幅本身就成了主导滞后，故放到 12000（每个 5ms 拍可走满 60%），实际只剩饱和起作用
constexpr float    kMaxEffortSlew   = 12000.0f;
constexpr float    kFallAngleRad    = 0.52f;   // ~30°
constexpr uint32_t kFallHoldMs      = 20;      // 连续超角 20ms（约 4 拍@200Hz）才确认摔倒
constexpr uint32_t kImuTimeoutMs    = 100;     // IMU 数据过期
constexpr uint32_t kCmdTimeoutMs    = 500;     // 通信断链自动停车

// ---- 阶段3 平衡状态反馈（k 即 ∂u/∂x，已含符号；见 docs/stage34_gain_init_theory.md）----
// 设计点：ω_n = 2λ ≈ 57rad/s, ζ = 0.8 → T_d = k_θ̇/k_θ = 0.022s。
// T_d 由 l 决定，可信；k_θ 的绝对值要除以未实测的"占空比→加速度"标度 K_a，
// 按 K_a 的合理区间 k_θ 落在 150~730，故初值取偏保守的 200，靠 p 命令按 ×1.5 往上爬。
// 调参时 k_θ̇ 始终跟着算：k_θ̇ = 0.022 * k_θ。
constexpr float kPitchTrimDeg    = 3.23f;   // 机械平衡角(度)，与串口 t 同单位；内部再转 rad
constexpr float kGainPitch       = 500.0f; // k_θ,   %/rad     （前倾要正输出，>0）
constexpr float kGainPitchRate   = 10.0f;   // k_θ̇,   %/(rad/s)
constexpr float kGainIntegPitch  = 0.0f;  // k_iθ,  %/(rad·s)
constexpr float kIntegTermLimit  = 15.0f; // 积分项限幅 (%)

// ---- 阶段4 速度环/位置环（同一状态反馈里多两项，非级联）----
// 初值/时标分离：docs/stage34_gain_init_theory.md ；调参步骤：docs/stage4_velocity_loop.md
// 实车：k_vel≈10 稳、20 开始晃；位置环先保持 0。
constexpr float kGainVel         = 20.0f;  // k_ṡ,   %/(m/s)
constexpr float kGainPos         = 10.0f;   // k_s,   %/m（当积分用，靠限幅防 windup）
constexpr float kPosTermLimit    = 25.0f;  // 位置项 anti-windup 限幅 (%)：饱和后停止累积 pos_ref
constexpr float kMaxLinearMps    = 0.5f;  // 速度目标限幅 (m/s)，防手滑给过大目标
// 速度目标斜率限幅 (m/s²)：软起步 + 软停。停止时 vref 斜坡衰减，车滑行减速，
// pos_ref 跟着走而非急锁，消除松杆瞬间位置误差突变引起的回拉振荡。越小越软、越肉。
constexpr float kVelSlewMps2     = 1.0f;

// 速度→倾角前馈：θ_ref = trim + kff·vref，主动前倾顶过死区（docs/stage4_vel_pitch_feedforward.md）
// 串口 g 输入 deg/(m/s)，内部转 rad；从小往上调，过大会给速瞬间猛冲。
constexpr float kGainVelToPitch  = 0.1745f; // k_ff, rad/(m/s)（=10°/(m/s)，实车标定值）
constexpr float kFfPitchLimitRad = 0.26f; // 前馈后 θ_ref 相对 trim 的限幅 (rad,≈15°)，防大 vref 顶到摔倒角

// ---- 偏航 / 走直线（平衡主量之外的差速项，不进 BalanceController）----
// az≈0 时 heading hold：锁住松开转向杆时的航向；az≠0 时只做角速度跟踪，松杆再锁新航向。
// 航向角来自轮式里程计（短时走直够用）；角速度用陀螺 Z 阻尼。
constexpr float kGainYaw         = 40.0f; // k_ψ,  %/rad     （串口 z）
constexpr float kGainYawRate     = 5.0f;  // k_ψ̇, %/(rad/s) （串口 n）
constexpr float kMaxAngularRps   = 1.5f;  // 角速度目标限幅 (rad/s)
constexpr float kYawCmdEps       = 1e-3f; // |az| 小于此视为松杆，进入 heading hold

// ---- micro-ROS WiFi（与 src/main.cpp / 历史工程一致；只跑在 Core0）----
// agent 电脑 IP:PORT 须与 micro_ros_agent 监听一致；SSID/密码可改成现场热点
constexpr const char* kWifiSsid     = "CHY";
constexpr const char* kWifiPassword = "13705558902";
constexpr const char* kAgentIp      = "192.168.5.62";
constexpr uint16_t    kAgentPort    = 8888;
constexpr const char* kRosNodeName  = "goudan_bot_balance";
constexpr const char* kCmdVelTopic  = "/cmd_vel";

} // namespace cfg
