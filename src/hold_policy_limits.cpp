#include "HoldPolicy.h"
#include "config.h"

HoldPolicy::Limits HoldPolicy::robotLimits(bool torque_gate)
{
    HoldPolicy::Limits lim{};
    lim.theta_in_rad = cfg::kHoldThetaInDeg * 0.0174532925f;
    lim.theta_out_rad = cfg::kHoldThetaOutDeg * 0.0174532925f;
    lim.omega_in = cfg::kHoldOmegaIn;
    lim.omega_out = cfg::kHoldOmegaOut;
    lim.vel_in = cfg::kHoldVelIn;
    lim.vel_out = cfg::kHoldVelOut;
    lim.pos_in = cfg::kHoldPosIn;
    lim.pos_out = cfg::kHoldPosOut;
    if (torque_gate) {
        lim.tau_in = cfg::kTorqueEps;
        lim.tau_out = cfg::kTorqueEps * 1.5f;
    }
    lim.v_cmd_eps = cfg::kHoldVCmdEps;
    lim.w_cmd_eps = cfg::kYawCmdEps;
    lim.enter_ticks = cfg::kHoldEnterTicks;
    lim.settle_ticks = cfg::kHoldSettleTicks;
    return lim;
}
