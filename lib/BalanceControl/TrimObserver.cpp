#include "TrimObserver.h"

#include <math.h>

namespace {
float clampf(float v, float lim)
{
    if (v > lim) {
        return lim;
    }
    if (v < -lim) {
        return -lim;
    }
    return v;
}
} // namespace

void TrimObserver::resetWindow()
{
    obs_n_ = 0;
    obs_psum_ = 0.0f;
}

void TrimObserver::resetBisection()
{
    delta_ = 0.0f;
}

void TrimObserver::reset()
{
    v_dc_ = 0.0f;
    bias_ = 0.0f;
    coast_ = false;
    was_holding_ = false;
    quiet_n_ = 0;
    hold_n_ = 0;
    alpha_wh_ = 0.0f;
    omega_init_ = false;
    resetWindow();
    resetBisection();
}

void TrimObserver::applyWindow(bool holding, float pitch_cmd, float p_mean, float dp)
{
    if (!holding) {
        hold_n_ = 0;
    } else if (hold_n_ < p_.hold_snap_n) {
        hold_n_++;
    }

    if (fabsf(dp) < p_.fall_rad) {
        if (holding && hold_n_ >= p_.hold_snap_n) {
            bias_ = clampf(p_mean - pitch_cmd, p_.limit_rad);
            resetBisection();
        }
        return;
    }
    // τ=0：前倾 Δpitch>0 → pitch>θ* → 降 ref；后倾 → 升 ref
    const float want = (dp > 0.0f) ? -1.0f : 1.0f;
    if (delta_ == 0.0f) {
        delta_ = want * p_.step0_rad;
    } else if ((delta_ > 0.0f) != (want > 0.0f)) {
        delta_ *= -0.5f;
        if (fabsf(delta_) < p_.step_min_rad) {
            delta_ = copysignf(p_.step_min_rad, delta_);
        }
    }
    if (delta_ > p_.step0_rad) {
        delta_ = p_.step0_rad;
    } else if (delta_ < -p_.step0_rad) {
        delta_ = -p_.step0_rad;
    }
    bias_ = clampf(bias_ + delta_, p_.limit_rad);
}

float TrimObserver::update(const Sample& s)
{
    if (!tcmd_init_) {
        last_tcmd_ = s.pitch_cmd;
        tcmd_init_ = true;
    } else if (s.pitch_cmd != last_tcmd_) {
        last_tcmd_ = s.pitch_cmd;
        bias_ = 0.0f;
        resetWindow();
        resetBisection();
    }

    if (!s.balancing) {
        reset();
        return 0.0f;
    }

    v_dc_ += p_.vdc_alpha * (s.vel - v_dc_);

    if (!omega_init_) {
        last_omega_ = s.omega;
        omega_init_ = true;
        alpha_wh_ = 0.0f;
    } else {
        const float a_inst = (s.omega - last_omega_) * p_.ctrl_hz;
        alpha_wh_ += p_.alpha_lpf * (a_inst - alpha_wh_);
        last_omega_ = s.omega;
    }

    if (!p_.enable) {
        coast_ = false;
        was_holding_ = s.holding;
        return 0.0f;
    }

    if (!s.holding) {
        hold_n_ = 0;
    }

    const bool hunt_quiet =
        !s.holding &&
        fabsf(s.v_cmd) < p_.v_cmd_eps &&
        fabsf(s.vel) < p_.vel_max &&
        fabsf(s.omega) < p_.omega_max &&
        fabsf(alpha_wh_) < p_.alpha_max &&
        s.tau_half < p_.tau_max;

    if (s.holding) {
        quiet_n_ = p_.enter_ticks;
    } else if (hunt_quiet) {
        if (quiet_n_ < p_.enter_ticks) {
            quiet_n_++;
        }
    } else if (quiet_n_ > 0) {
        quiet_n_--;
    }

    const bool was_coast = coast_;
    coast_ = s.holding || (quiet_n_ >= p_.enter_ticks);
    const bool hold_enter = s.holding && !was_holding_;
    was_holding_ = s.holding;
    if (hold_enter || (coast_ && !was_coast)) {
        resetWindow();
        resetBisection();
        return bias_;
    }
    if (!coast_) {
        resetWindow();
        return bias_;
    }

    if (obs_n_ == 0) {
        p_open_ = s.pitch;
    }
    obs_n_++;
    obs_psum_ += s.pitch;
    if (obs_n_ >= p_.period_ticks) {
        applyWindow(s.holding, s.pitch_cmd, obs_psum_ / (float)obs_n_,
                    s.pitch - p_open_);
        resetWindow();
    }
    return bias_;
}
