#pragma once

#include <math.h>

// E3 v_dc trim 伺服：消每次上电机械平衡角偏移导致的单向爬行。
// 稳态爬速 v_dc ≈ (k_pitch/k_vel)·(ref−θ*)，是 trim 误差的可测代理；
// bias += −k·v_dc·dt 把 ref 慢慢拧到 θ*。收敛时标 τ = k_vel/(k_pitch·k)。
// 门控：balancing（否则清零、下次武装重学）、|v_cmd|<eps、!freeze（坡上冻 bias 仍更新 v_dc）。
class TrimServo {
public:
    struct Params {
        float vdc_alpha; // v_dc 低通 α=dt/τ（τ≈2s 抹掉对摇、留下净爬行）
        float ctrl_hz;
        float v_cmd_eps; // m/s，有速度指令时 v_dc 不是误差，不伺服
        float k;         // rad/m（每净爬 1m 拧回的角）；0=关，串口 u 在线改
        float limit_rad; // bias 限幅；顶满说明另有故障
    };

    struct Sample {
        bool  balancing;
        float vel;       // m/s，两轮均速
        float v_cmd;
        float pitch_cmd; // 串口 t；变了则 bias 清零重学
        bool  freeze;    // 坡上
    };

    void setParams(const Params& p) { p_ = p; }
    void setK(float k) { p_.k = k > 0.0f ? k : 0.0f; }

    float update(const Sample& s)
    {
        if (s.pitch_cmd != last_tcmd_) {
            last_tcmd_ = s.pitch_cmd;
            bias_ = 0.0f;
        }
        if (!s.balancing) {
            v_dc_ = 0.0f;
            bias_ = 0.0f;
            return 0.0f;
        }
        v_dc_ += p_.vdc_alpha * (s.vel - v_dc_);
        if (p_.k > 0.0f && !s.freeze && fabsf(s.v_cmd) < p_.v_cmd_eps) {
            bias_ -= p_.k * v_dc_ / p_.ctrl_hz;
            if (bias_ > p_.limit_rad) {
                bias_ = p_.limit_rad;
            } else if (bias_ < -p_.limit_rad) {
                bias_ = -p_.limit_rad;
            }
        }
        return bias_;
    }

    float vDc() const { return v_dc_; }
    float bias() const { return bias_; }

private:
    Params p_{};
    float  v_dc_ = 0.0f;
    float  bias_ = 0.0f;
    float  last_tcmd_ = 0.0f;
};
