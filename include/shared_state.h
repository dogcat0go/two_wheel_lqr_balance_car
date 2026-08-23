#pragma once

#include <stdint.h>

// ============================================================
// 频带分离的唯一数据边界。
// 控制环(Core1, 200Hz 硬实时) 与 通信环(Core0, WiFi/串口, 抖动不可控)
// 之间只交换下面两个 POD 快照，禁止互相直接调用对方的对象。
// 换掉通信层（串口 → micro-ROS）时，控制环代码零改动。
// 临界区只拷一个结构体，用 portMUX 自旋锁而非队列，避免阻塞控制环。
// ============================================================

// 控制环 → 通信环（状态上报）
struct ControlSnapshot {
    uint32_t stamp_ms;
    float    pitch_rad;
    float    pitch_rate_rps;
    float    pitch_ref_rad;  // 当前平衡角目标，调 trim 时要看
    float    pitch_acc_rad;  // 仅加速度计估角，判 AHRS 零点用
    float    acc_g[3];       // 标定后的 ax,ay,az (g)
    float    yaw_rad;        // 轮式里程计航向 (rad)
    float    yaw_ref_rad;    // heading hold 参考
    float    yaw_rate_rps;   // 陀螺 Z (rad/s)
    float    u_yaw;          // 差速项 (%)：right += u_yaw, left -= u_yaw
    float    yaw_integ_term;  // 航向积分项 (N·m)，调参观察
    float    wheel_pos_m[2];
    float    wheel_vel_mps[2];
    float    v_dc_mps;           // 两轮均速的慢直流分量（~2s 低通）；判是否真稳态爬行
    float    v_ref_mps;          // TwistRef 平滑后的线速度目标
    float    w_ref_rps;          // TwistRef 平滑后的角速度目标
    int32_t  wheel_ticks[2]; // 原始编码器计数，标 kMPerTick 用
    float    effort[2];         // 安全层之后真正下发的量（当前是占空比 %）
    float    tau_nm[2];         // 轮端力矩指令 (N·m)；m 2/3 有值，PWM 为 0
    float    current_a[2];      // 左右电流低通后 (A)，正 = 该轮前进
    float    current_raw_a[2];  // 本拍原始 (A)
    uint8_t  isense_fault;      // 电流通道量程门：bit0=L bit1=R，1=该路已降级开环
    float    i_ref_a[2];        // 开环电流目标 (A)；PWM 开环/平衡为 0
    float    terms[5];          // BalanceController::Term 各分量，判振源用
    uint32_t ctrl_hz;        // 实测控制频率，看有没有掉拍
    uint32_t overrun_count;  // 单周期超时次数
    uint8_t  fault;          // Safety::Fault 位掩码
    uint8_t  mode;           // 0 开环, 1 PWM平衡, 2 电流+手调%, 3 电流+LQR N·m
    bool     imu_ok;
    bool     armed;          // 平衡出力使能：上电/摔倒后需 r
    uint8_t  hold;           // 0 TRACK/未武装；1 HOLD（钳位）
    uint8_t  hold_n;         // CONFIRM 已连续拍数；HOLD 时 = enter_ticks
    float    alpha_inj_rad;  // 串口 q，开环坡角
    float    sin_eff;        // gain·sin(α_inj)
    float    theta_eq_rad;   // K_EQ·sin_eff，已叠进 pitch_ref
    float    tau_ff;         // MGR·sin_eff，叠在 LQR 之和上
};

// 通信环 → 控制环（指令下行）
struct CommandInput {
    uint32_t stamp_ms;     // 指令时间戳，控制环据此判断通信是否断链
    float    linear_x;     // m/s，阶段4 起作为速度目标
    float    angular_z;    // rad/s，正 = 逆时针（左慢右快）
    float    test_effort[2];   // 开环占空比(%)，mode 0 且该轮未走电流
    float    test_current[2];  // 开环 I_ref (A)
    uint8_t  use_current[2];   // 1 = 该轮电流 PI
    uint32_t current_zero_seq;
    uint8_t  mode;         // 0 开环, 1 PWM平衡, 2 电流+手调%, 3 电流+LQR N·m
    float    pitch_ref_rad;
    float    gains[5];     // m 1/2 手调 %；m 3 时与 lqr_gains 相同（兼容 stage2）
    float    lqr_gains[4]; // 始终下发：kθ kω ks kv (N·m/状态)；串口 k 只写这里
    float    k_yaw;        // stage2: %/rad；stage5: N·m/rad（0=不锁航向）
    float    k_yaw_rate;   // stage2: %/(rad/s)；stage5: k_sync N·m/(m/s)
    float    k_yaw_integ;  // stage5 航向积分 ki，N·m/(rad·s)（串口 j）
    float    k_vff;        // 速度→倾角前馈, rad/(m/s)：θ_ref = trim + k_vff·linear_x
    float    alpha_inj_rad; // 串口 q，开环坡角 (rad)；0=补偿关
    float    slope_gain;    // 串口 h，0~1；两半同乘
    uint32_t reset_seq;    // 递增即视为一次复位请求（清故障锁存 + 清积分）
    uint32_t calib_seq;    // 递增触发一次死区自标定（串口 b）
};

void publishSnapshot(const ControlSnapshot& s);
ControlSnapshot fetchSnapshot();

void publishCommand(const CommandInput& c);
CommandInput fetchCommand();
