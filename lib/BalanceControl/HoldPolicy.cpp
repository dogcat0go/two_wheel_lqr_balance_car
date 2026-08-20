#include "HoldPolicy.h"

#include <math.h>

void HoldPolicy::reset()
{
    state_ = kTrack;
    confirm_ = 0;
    just_entered_ = false;
}

bool HoldPolicy::canEnter(const Sample& s) const
{
    if (!s.allow) {
        return false;
    }
    if (fabsf(s.v_cmd) >= lim_.v_cmd_eps || fabsf(s.w_cmd) >= lim_.w_cmd_eps) {
        return false;
    }
    if (fabsf(s.e_theta) >= lim_.theta_in_rad) {
        return false;
    }
    if (fabsf(s.omega) >= lim_.omega_in) {
        return false;
    }
    if (fabsf(s.vel) >= lim_.vel_in) {
        return false;
    }
    if (lim_.pos_in > 0.0f && fabsf(s.e_pos) >= lim_.pos_in) {
        return false;
    }
    if (lim_.tau_in > 0.0f && fabsf(s.tau_half) >= lim_.tau_in) {
        return false;
    }
    return true;
}

bool HoldPolicy::shouldExit(const Sample& s) const
{
    if (!s.allow) {
        return true;
    }
    if (fabsf(s.v_cmd) >= lim_.v_cmd_eps || fabsf(s.w_cmd) >= lim_.w_cmd_eps) {
        return true;
    }
    if (fabsf(s.e_theta) > lim_.theta_out_rad) {
        return true;
    }
    if (fabsf(s.omega) > lim_.omega_out) {
        return true;
    }
    if (fabsf(s.vel) > lim_.vel_out) {
        return true;
    }
    if (lim_.pos_in > 0.0f && fabsf(s.e_pos) > lim_.pos_out) {
        return true;
    }
    if (lim_.tau_in > 0.0f && fabsf(s.tau_half) > lim_.tau_out) {
        return true;
    }
    return false;
}

HoldPolicy::State HoldPolicy::update(const Sample& s)
{
    just_entered_ = false;
    if (lim_.enter_ticks <= 0) {
        reset();
        return state_;
    }

    if (state_ == kHold) {
        if (shouldExit(s)) {
            state_ = kTrack;
            if (confirm_ > 0) {
                confirm_--;
            }
        }
        return state_;
    }

    if (canEnter(s)) {
        state_ = kConfirm;
        if (confirm_ < 255) {
            confirm_++;
        }
        if (confirm_ >= (uint8_t)lim_.enter_ticks) {
            state_ = kHold;
            just_entered_ = true;
        }
        return state_;
    }

    if (confirm_ > 0) {
        confirm_--;
    }
    state_ = kTrack;
    return state_;
}
