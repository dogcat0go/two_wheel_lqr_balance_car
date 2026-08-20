#pragma once

// 偏航差速，不进 LQR。输入原始 ω_cmd（判转向/松杆）与平滑 ω_ref（轮速差目标）。
// 有转向：只同步轮速差。松杆且 k_yaw≠0：锁航向 P+I + 同步。
class YawMixer {
public:
    struct Params {
        float dt;
        float track;         // 轮距 (m)
        float w_eps;
        float yaw_err_lim;
        float integ_lim;     // 积分项限幅 (N·m)
        float tau_max;
    };

    struct Sample {
        float w_cmd;
        float w_ref;
        float yaw;
        float v_l;
        float v_r;
        float k_yaw;
        float k_sync;
        float k_integ;
        bool  active;
        bool  holding;
        bool  one_stuck;
    };

    struct Output {
        float u_yaw;
        float u_i;
        float yaw_ref;
        bool  turning;
    };

    void setParams(const Params& p) { p_ = p; }
    void reset(float yaw);

    Output update(const Sample& s);

    float yawRef() const { return yaw_ref_; }

private:
    Params p_{};
    float  yaw_ref_ = 0.0f;
    float  yaw_integ_ = 0.0f;
    bool   yaw_hold_ = true;
};
