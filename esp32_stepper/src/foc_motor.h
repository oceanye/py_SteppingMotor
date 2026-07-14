// esp32_stepper/src/foc_motor.h
// 闭环单轴电机 API。FOC 构建由 foc_motor.cpp 实现（BLDC + AS5600），
// GEAR 构建由 gear_motor.cpp 实现（DRV8871 + 编码器 + PID）。
#pragma once
#include <stdint.h>

enum FocState : uint8_t {
    FOC_STATE_DISABLED = 0,
    FOC_STATE_ALIGNING = 1,   // GEAR 模式不进入此状态
    FOC_STATE_RUNNING  = 2,
    FOC_STATE_FAULT    = 3,
};

// ── 共用 API（两种构建都实现）──
void foc_init();
bool foc_set_target_deg(int axis, float deg);
bool foc_request_enable(int axis, bool en);
bool foc_clear_fault(int axis);
void foc_latch_fault(int axis);
bool foc_set_voltage_limit(int axis, float v);   // GEAR: 重解释为 PWM duty cap (0-100%)
void foc_home(int axis);
bool foc_set_p_angle(int axis, float p);

FocState foc_get_state(int axis);
float    foc_get_current_deg(int axis);
float    foc_get_target_deg(int axis);
bool     foc_is_fault_latched(int axis);

// ── FOC-only API（仅 foc_motor.cpp 实现；GEAR 构建里不要调用）──
bool foc_set_pole_pairs_and_store(int axis, int n);
bool foc_set_p_velocity(int axis, float p);
bool foc_force_realign(int axis);

// ── GEAR-only API（仅 gear_motor.cpp 实现；FOC 构建里不要调用）──
bool foc_set_gear_ratio_and_store(int axis, float ratio);
bool foc_set_p_integral(int axis, float i);
bool foc_set_p_derivative(int axis, float d);
void foc_run_diagnostics(int axis);        // GEAR 模式 DIAG,<axis> 的实现
void foc_run_diagnostics_live(int axis);   // GEAR 模式 DIAG,<axis>,LIVE: 5 秒实时编码器状态
