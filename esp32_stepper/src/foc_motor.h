// esp32_stepper/src/foc_motor.h
#pragma once
#include <stdint.h>

enum FocState : uint8_t {
    FOC_STATE_DISABLED = 0,
    FOC_STATE_ALIGNING = 1,
    FOC_STATE_RUNNING  = 2,
    FOC_STATE_FAULT    = 3,
};

// 初始化所有轴：sensors / drivers / motors + 每轴 Core 0 任务。
void foc_init();

// 多轴 API：第一参数都是 axis（0=L, 1=R）。非法 axis 返回 false。
bool foc_set_target_deg(int axis, float deg);
bool foc_request_enable(int axis, bool en);
bool foc_clear_fault(int axis);
void foc_latch_fault(int axis);
bool foc_set_voltage_limit(int axis, float v);
void foc_home(int axis);
bool foc_set_pole_pairs_and_store(int axis, int n);
bool foc_set_p_angle(int axis, float p);
bool foc_set_p_velocity(int axis, float p);

// 状态查询（按轴）
FocState foc_get_state(int axis);
float    foc_get_current_deg(int axis);
float    foc_get_target_deg(int axis);
bool     foc_is_fault_latched(int axis);
