// foc_svpwm.c
// Field-oriented control loop for the 48 V / 20 kHz BLDC driver.
//
// Signal chain per control tick (50 us):
//   phase shunt ADC -> Clarke (a,b,c -> alpha,beta) -> Park (alpha,beta -> d,q)
//   -> PI(d), PI(q) -> inverse Park -> SVPWM duty -> TIM1 CCRx
//
// Regenerative braking reuses the same inverter legs. We force Iq negative,
// which pushes current back through the body diodes and raises the DC bus.
// A bus voltage governor then clamps Iq so we do not cook the 63 V caps.
//
// Units: currents in amps, voltages in volts, angles in electrical radians.
// All phase currents are measured low-side with 5 mOhm shunts and a x10 amp.

#include "foc_svpwm.h"

#include <math.h>
#include <stddef.h>

static foc_state_t g_foc;
static regen_state_t g_regen = REGEN_IDLE;
static float g_regen_iq_limit = 0.0f;
static float g_bus_soft_limit = 56.0f;   // V, start folding Iq back here
static float g_bus_hard_limit = 60.0f;   // V, cut regen entirely

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Clarke transform, amplitude invariant form (2/3 scaling).
// ia + ib + ic = 0 is assumed; we only sample two shunts and reconstruct.
void foc_clarke(float ia, float ib, float ic, float* alpha, float* beta) {
    (void)ic;
    *alpha = (2.0f / 3.0f) * (ia - 0.5f * ib - 0.5f * ic);
    *beta = (2.0f / 3.0f) * (0.8660254f * (ib - ic));
}

// Park transform. theta_e is the rotor electrical angle from the encoder,
// already offset by the calibration constant in g_foc.
void foc_park(float alpha, float beta, float theta_e, float* d, float* q) {
    const float c = cosf(theta_e);
    const float s = sinf(theta_e);
    *d = alpha * c + beta * s;
    *q = -alpha * s + beta * c;
}

// Inverse Park, used to turn the PI outputs back into the stationary frame.
void foc_inv_park(float d, float q, float theta_e, float* alpha, float* beta) {
    const float c = cosf(theta_e);
    const float s = sinf(theta_e);
    *alpha = d * c - q * s;
    *beta = d * s + q * c;
}

// PI controller with back-calculation anti-windup. The integrator is only
// allowed to run while the output is not saturated in the same direction.
static float pi_step(pi_ctrl_t* pi, float err, float dt) {
    const float p = pi->kp * err;
    float out = p + pi->integ;

    if (out > pi->out_max) {
        out = pi->out_max;
        if (err < 0.0f) pi->integ += pi->ki * err * dt;
    } else if (out < pi->out_min) {
        out = pi->out_min;
        if (err > 0.0f) pi->integ += pi->ki * err * dt;
    } else {
        pi->integ += pi->ki * err * dt;
        pi->integ = clampf(pi->integ, pi->out_min, pi->out_max);
    }
    return out;
}

// Space-vector PWM. Takes alpha/beta in the stationary frame, returns three
// duty cycles in [0,1] plus the active sector for telemetry.
void foc_svpwm(float alpha, float beta, float vbus, svpwm_out_t* out) {
    // Normalise to the hexagon. 1/sqrt(3) keeps us inside the linear region.
    const float inv_vbus = (vbus > 1.0f) ? (1.0f / vbus) : 0.0f;
    const float u1 = beta * inv_vbus;
    const float u2 = (0.8660254f * alpha - 0.5f * beta) * inv_vbus;
    const float u3 = (-0.8660254f * alpha - 0.5f * beta) * inv_vbus;

    // Sector determination by sign of the three offset references.
    int sector = 0;
    if (u1 > 0.0f) sector += 1;
    if (u2 > 0.0f) sector += 2;
    if (u3 > 0.0f) sector += 4;
    switch (sector) {
        case 1: sector = 2; break;
        case 2: sector = 6; break;
        case 3: sector = 1; break;
        case 4: sector = 4; break;
        case 5: sector = 3; break;
        case 6: sector = 5; break;
        default: sector = 0; break;
    }

    // Common-mode offset injection (min/max midpoint), equivalent to the
    // standard SVPWM dwell-time solution but cheaper on this MCU.
    const float vmax = fmaxf(u1, fmaxf(u2, u3));
    const float vmin = fminf(u1, fminf(u2, u3));
    const float vcom = -0.5f * (vmax + vmin);

    out->duty_a = clampf(0.5f + (u1 + vcom), 0.0f, 1.0f);
    out->duty_b = clampf(0.5f + (u2 + vcom), 0.0f, 1.0f);
    out->duty_c = clampf(0.5f + (u3 + vcom), 0.0f, 1.0f);
    out->sector = sector;
}

// Bus voltage governor for regen. Folds the commanded Iq back as the bus
// climbs toward the hard limit so the brake chopper never has to fire.
static float regen_governor(float iq_cmd, float vbus) {
    if (vbus >= g_bus_hard_limit) {
        return 0.0f;
    }
    if (vbus <= g_bus_soft_limit) {
        return iq_cmd;
    }
    const float span = g_bus_hard_limit - g_bus_soft_limit;
    const float fold = 1.0f - (vbus - g_bus_soft_limit) / span;
    return iq_cmd * clampf(fold, 0.0f, 1.0f);
}

// Regenerative braking state machine. Entered when the throttle is released
// above a rotor speed threshold; exits on low speed or bus overvoltage.
regen_state_t foc_regen_step(float rotor_rpm, float vbus, float throttle,
                             float* iq_cmd) {
    switch (g_regen) {
        case REGEN_IDLE:
            if (throttle < 0.02f && rotor_rpm > 400.0f && vbus < g_bus_soft_limit) {
                g_regen = REGEN_ARMED;
            }
            break;

        case REGEN_ARMED:
            // Ramp the negative Iq in so we do not slam the bus.
            g_regen_iq_limit -= 0.4f;
            if (g_regen_iq_limit < -18.0f) g_regen_iq_limit = -18.0f;
            *iq_cmd = g_regen_iq_limit;
            if (rotor_rpm < 300.0f) {
                g_regen = REGEN_IDLE;
                g_regen_iq_limit = 0.0f;
            } else if (vbus > g_bus_soft_limit) {
                g_regen = REGEN_FOLDING;
            } else {
                g_regen = REGEN_ACTIVE;
            }
            break;

        case REGEN_ACTIVE:
            g_regen_iq_limit -= 0.1f;
            if (g_regen_iq_limit < -24.0f) g_regen_iq_limit = -24.0f;
            *iq_cmd = regen_governor(g_regen_iq_limit, vbus);
            if (vbus > g_bus_soft_limit) {
                g_regen = REGEN_FOLDING;
            } else if (rotor_rpm < 300.0f) {
                g_regen = REGEN_IDLE;
                g_regen_iq_limit = 0.0f;
            }
            break;

        case REGEN_FOLDING:
            *iq_cmd = regen_governor(g_regen_iq_limit, vbus);
            if (vbus >= g_bus_hard_limit) {
                g_regen = REGEN_FAULT;
            } else if (vbus < g_bus_soft_limit - 1.0f) {
                g_regen = REGEN_ACTIVE;
            }
            break;

        case REGEN_FAULT:
            // Latched. Host has to clear it, we do not auto-retry into a hot bus.
            *iq_cmd = 0.0f;
            break;
    }
    return g_regen;
}

// One full control tick. Called from the TIM1 update ISR at 20 kHz.
void foc_tick(const foc_input_t* in, foc_output_t* out) {
    float alpha, beta, d, q;

    foc_clarke(in->ia, in->ib, in->ic, &alpha, &beta);
    foc_park(alpha, beta, in->theta_e, &d, &q);

    // D axis is held at zero; all torque comes from Q.
    float iq_cmd = in->iq_ref;
    if (in->regen_enable) {
        foc_regen_step(in->rotor_rpm, in->vbus, in->throttle, &iq_cmd);
    }

    const float vd = pi_step(&g_foc.pi_d, 0.0f - d, in->dt);
    const float vq = pi_step(&g_foc.pi_q, iq_cmd - q, in->dt);

    foc_inv_park(vd, vq, in->theta_e, &alpha, &beta);
    foc_svpwm(alpha, beta, in->vbus, &out->pwm);

    out->id_meas = d;
    out->iq_meas = q;
    out->regen = g_regen;
}

// TODO: the Clarke transform currently trusts ia+ib+ic=0. On the rev B board
// the third shunt is populated, so switch to the full three-sample form and
// add a phase-loss check before we ship regen to production.
