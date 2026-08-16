#pragma once

#include <stdint.h>

// 平衡控制器：统一状态反馈
//   u = Σ k_i (x_i - x_ref_i) + ki_pitch ∫(pitch - pitch_ref) dt
//
// 阶段3(PID) / 阶段4(速度环) / 阶段5(LQR) 是同一个式子填不同增益，不是三套 backend：
//   阶段3: k = {kp, kd, 0, 0}
//   阶段4: 补 k_pos / k_vel（或由外层速度环写 ref.pitch 走级联）
//   阶段5 m 3: k = -K_lqr（N·m；LQR 解 u=-Kx 时填入取反）
//
// 符号约定：k 就是 ∂u/∂x，已含符号。倒立摆要"朝倒的方向追"，
// 所以前倾(pitch>0)时需要正输出(前进)，即 k_pitch > 0。
struct BalanceState {
    float pitch;      // rad，正 = 车体前倾
    float pitch_rate; // rad/s，取陀螺实测值，不要用 pitch 差分
    float pos;        // m，两轮位移均值
    float vel;        // m/s，两轮速度均值
};

class BalanceController {
public:
    struct Gains {
        float k_pitch;
        float k_pitch_rate;
        float k_pos;
        float k_vel;
        float ki_pitch; // 只对 pitch 误差积分，用于消 trim 残差
    };

    enum Term : int {
        kTermPitch = 0,
        kTermPitchRate,
        kTermPos,
        kTermVel,
        kTermInteg,
        kTermCount,
    };

    void setGains(const Gains& g) { gains_ = g; }
    void setRef(const BalanceState& ref) { ref_ = ref; }
    // out_abs: 输出饱和(与 u 同单位)，用于条件积分判据；integ_abs: 积分项本身的限幅
    void setLimits(float out_abs, float integ_abs);

    float update(const BalanceState& x, float dt_s);
    void  reset(); // 清积分与分量，故障期间/模式切换时调

    // 各分量，供遥测判断是哪一项在振荡
    const float* terms() const { return terms_; }

private:
    Gains        gains_{};
    BalanceState ref_{};
    float        out_abs_ = 100.0f;
    float        integ_abs_ = 20.0f;
    float        integ_ = 0.0f; // ∫(pitch - pitch_ref) dt，rad·s
    float        terms_[kTermCount] = {};
};
