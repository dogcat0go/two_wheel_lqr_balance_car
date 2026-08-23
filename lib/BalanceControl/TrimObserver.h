#pragma once

// 机械平衡角 θ* 慢观测：可选。关：只更新 v_dc，不改 ref、不切力矩。
// 开：τ=0 后倾倒看 Δpitch（前倾降、后倾升）。连续 HOLD 满 hold_snap_n 窗且站住才 ref=pitch。
// 倾倒每窗最多 ±step0；方向反了则 ×(-0.5)。
//
// E3 v_dc trim 伺服（servo_k>0 开，替代上面的倾倒观测）：bias += −k·v_dc·dt。
// 稳态爬速 v_dc ≈ (k_pitch/k_vel)·(ref−θ*)，积分把 ref 拧到 θ*，τ = k_vel/(k_pitch·k)。
class TrimObserver {
public:
    struct Params {
        bool  enable;
        float vdc_alpha;
        int   period_ticks;
        int   enter_ticks;
        float step0_rad;
        float step_min_rad;
        float fall_rad;      // 窗内 |Δpitch| 小于此视为站住
        int   hold_snap_n;   // 连续 HOLD 满此窗数且站住，才 ref=pitch
        float vel_max;
        float tau_max;
        float omega_max;
        float alpha_max;
        float alpha_lpf;
        float ctrl_hz;
        float v_cmd_eps;
        float limit_rad;
        float servo_k;         // E3 伺服增益 rad/m（每净爬 1m 拧回的角），0=关
        float servo_limit_rad; // E3 bias 限幅（±2°；超出说明另有故障）
    };

    struct Sample {
        bool  balancing;
        bool  holding;
        float vel;
        float pitch;
        float omega;
        float v_cmd;
        float pitch_cmd;
        float tau_half;
        bool  freeze;    // 上坡：冻 bias，仍更新 v_dc
    };

    void setParams(const Params& p) { p_ = p; }
    void setServoK(float k) { p_.servo_k = k > 0.0f ? k : 0.0f; } // 串口 u 在线改
    void reset();

    float update(const Sample& s);

    float vDc() const { return v_dc_; }
    float bias() const { return (p_.enable || p_.servo_k > 0.0f) ? bias_ : 0.0f; }
    bool  coast() const { return p_.enable && coast_; }

private:
    void resetWindow();
    void resetBisection();
    void applyWindow(bool holding, float pitch_cmd, float p_mean, float dp);

    Params p_{};
    float  v_dc_ = 0.0f;
    float  bias_ = 0.0f;
    float  last_tcmd_ = 0.0f;
    bool   tcmd_init_ = false;
    bool   coast_ = false;
    bool   was_holding_ = false;
    int    quiet_n_ = 0;
    int    hold_n_ = 0;     // 连续完成的 HOLD 窗数
    int    obs_n_ = 0;
    float  obs_psum_ = 0.0f;
    float  p_open_ = 0.0f;
    float  delta_ = 0.0f;
    float  last_omega_ = 0.0f;
    float  alpha_wh_ = 0.0f;
    bool   omega_init_ = false;
};
