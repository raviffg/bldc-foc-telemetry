// foc_svpwm.h
// Public types and hardware register definitions for the FOC driver.
// Target: STM32G4 class MCU, TIM1 centre-aligned at 20 kHz.

#ifndef FOC_SVPWM_H
#define FOC_SVPWM_H

#include <stdint.h>

// ---- TIM1 register map (advanced control timer) ----
#define TIM1_BASE        0x40012C00UL
#define TIM1_CR1         (*(volatile uint32_t*)(TIM1_BASE + 0x00))
#define TIM1_CCMR1       (*(volatile uint32_t*)(TIM1_BASE + 0x18))
#define TIM1_CCER        (*(volatile uint32_t*)(TIM1_BASE + 0x20))
#define TIM1_CNT         (*(volatile uint32_t*)(TIM1_BASE + 0x24))
#define TIM1_ARR         (*(volatile uint32_t*)(TIM1_BASE + 0x2C))
#define TIM1_CCR1        (*(volatile uint32_t*)(TIM1_BASE + 0x34))
#define TIM1_CCR2        (*(volatile uint32_t*)(TIM1_BASE + 0x38))
#define TIM1_CCR3        (*(volatile uint32_t*)(TIM1_BASE + 0x3C))
#define TIM1_BDTR        (*(volatile uint32_t*)(TIM1_BASE + 0x44))

#define TIM1_ARR_20KHZ   4250U   // 170 MHz / (1 * 2 * 20000), centre aligned
#define TIM1_BDTR_MOE    (1U << 15)

// ---- current loop PI controller ----
typedef struct {
    float kp;
    float ki;
    float integ;
    float out_min;
    float out_max;
} pi_ctrl_t;

// ---- full FOC controller state ----
typedef struct {
    pi_ctrl_t pi_d;
    pi_ctrl_t pi_q;
    float theta_offset;   // encoder to electrical angle calibration, rad
    float pole_pairs;
    uint32_t fault_flags;
} foc_state_t;

// ---- SVPWM output ----
typedef struct {
    float duty_a;
    float duty_b;
    float duty_c;
    int sector;           // 1..6, 0 = zero vector
} svpwm_out_t;

// ---- regen braking states ----
typedef enum {
    REGEN_IDLE = 0,
    REGEN_ARMED = 1,
    REGEN_ACTIVE = 2,
    REGEN_FOLDING = 3,
    REGEN_FAULT = 4
} regen_state_t;

// ---- per-tick inputs from ADC / encoder / host ----
typedef struct {
    float ia;
    float ib;
    float ic;
    float theta_e;       // electrical angle, rad
    float vbus;
    float rotor_rpm;
    float throttle;      // 0..1
    float iq_ref;
    float dt;            // control period, s
    int regen_enable;
} foc_input_t;

// ---- per-tick outputs ----
typedef struct {
    svpwm_out_t pwm;
    float id_meas;
    float iq_meas;
    regen_state_t regen;
} foc_output_t;

void foc_clarke(float ia, float ib, float ic, float* alpha, float* beta);
void foc_park(float alpha, float beta, float theta_e, float* d, float* q);
void foc_inv_park(float d, float q, float theta_e, float* alpha, float* beta);
void foc_svpwm(float alpha, float beta, float vbus, svpwm_out_t* out);
regen_state_t foc_regen_step(float rotor_rpm, float vbus, float throttle, float* iq_cmd);
void foc_tick(const foc_input_t* in, foc_output_t* out);

#endif  // FOC_SVPWM_H
