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
    float    wheel_pos_m[2];
    float    wheel_vel_mps[2];
    int32_t  wheel_ticks[2]; // 原始编码器计数，标 kMPerTick 用
    float    effort[2];         // 安全层之后真正下发的量（当前是占空比 %）
    float    current_a[2];      // 左右电流低通后 (A)，正 = 该轮前进
    float    current_raw_a[2];  // 本拍原始 (A)
    float    terms[5];          // BalanceController::Term 各分量，判振源用
    uint32_t ctrl_hz;        // 实测控制频率，看有没有掉拍
    uint32_t overrun_count;  // 单周期超时次数
    uint8_t  fault;          // Safety::Fault 位掩码
    uint8_t  mode;           // 0 = 开环, 1 = 平衡
    bool     imu_ok;
    bool     armed;          // 平衡出力使能：上电/摔倒后需 r
};

// 通信环 → 控制环（指令下行）
struct CommandInput {
    uint32_t stamp_ms;     // 指令时间戳，控制环据此判断通信是否断链
    float    linear_x;     // m/s，阶段4 起作为速度目标
    float    angular_z;    // rad/s，正 = 逆时针（左慢右快）
    float    test_effort[2]; // 开环左右占空比(%)，仅 mode 0；e 同设，el/er 分设
    uint8_t  mode;         // 0 = 开环, 1 = 平衡
    float    pitch_ref_rad;
    float    gains[5];     // 对应 BalanceController::Gains 的五个字段
    float    k_yaw;        // 航向 P，%/rad
    float    k_yaw_rate;   // 航向 D / 角速度跟踪，%/(rad/s)
    float    k_vff;        // 速度→倾角前馈, rad/(m/s)：θ_ref = trim + k_vff·linear_x
    uint32_t reset_seq;    // 递增即视为一次复位请求（清故障锁存 + 清积分）
};

void publishSnapshot(const ControlSnapshot& s);
ControlSnapshot fetchSnapshot();

void publishCommand(const CommandInput& c);
CommandInput fetchCommand();
