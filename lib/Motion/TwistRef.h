#pragma once

// cmd_vel 参考成形。入口与 geometry_msgs/Twist 同口径：v (m/s)、ω (rad/s)。
// 来源（串口 v/a 或 ROS /cmd_vel）在 CommHost 已合成 CommandInput，本类不关心。
// 输出给 LQR：v_smooth、pos_ref、pitch_ff = sat(k_ff·v_smooth)。
// HOLD / Trim 的「有没有指令」仍看原始 twist；本类不代替 HoldPolicy。
class TwistRef {
public:
    struct Params {
        float dt;
        float v_slew;          // m/s²
        float w_slew;          // rad/s²
        float v_max;
        float w_max;
        float v_eps;           // |v_smooth| 小于此停止积分 pos_ref
        float ff_limit;        // |k_ff·v| 限幅 (rad)
        float pos_term_limit;  // 与 k_pos 同单位的位置项限幅，做 anti-windup
    };

    struct Sample {
        float v_cmd;
        float w_cmd;
        float pos;
        float k_pos;   // <=0 不钳 pos_ref
        float k_ff;    // rad/(m/s)，串口 g
        bool  active;
    };

    struct Output {
        float v;
        float w;
        float pos_ref;
        float pitch_ff;
    };

    void setParams(const Params& p) { p_ = p; }
    void reset(float pos);
    void rebasePos(float pos) { pos_ref_ = pos; }

    Output update(const Sample& s);

    float v() const { return v_; }
    float w() const { return w_; }
    float posRef() const { return pos_ref_; }

private:
    Params p_{};
    float  v_ = 0.0f;
    float  w_ = 0.0f;
    float  pos_ref_ = 0.0f;
};
