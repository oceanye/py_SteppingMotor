// esp32_stepper/src/foc_motor.h
#pragma once
#include <stdint.h>

// FSM state enumeration. Matches protocol FOC,S 第二字段数值。
enum FocState : uint8_t {
    FOC_STATE_DISABLED = 0,
    FOC_STATE_ALIGNING = 1,
    FOC_STATE_RUNNING  = 2,
    FOC_STATE_FAULT    = 3,
};

// 初始化 sensor/driver/motor；注册 Core 0 FOC 任务。
// 必须在 Serial.begin 之后调用。
void foc_init();

// ── Core 1 API（线程安全，从串口/主循环调用）──
// 设目标角度，单位：度。超范围返回 false。
bool foc_set_target_deg(float deg);
// 请求切换使能；若从 FAULT 请求 enable=true，返回 false（拒绝）。
bool foc_request_enable(bool en);
// 清除故障 latch。仅 FAULT 态返回 true。
bool foc_clear_fault();
// 由 Core 1 nFAULT 轮询调用，把状态机打到 FAULT
void foc_latch_fault();
// 在线设 voltage_limit。范围外返回 false。
bool foc_set_voltage_limit(float v);
// 把当前角度设为 0°。
void foc_home();
// 设极对数（1-50）并存 NVS，下次启动生效。
bool foc_set_pole_pairs_and_store(int n);

// ── 状态查询（从 Core 0 镜像读取）──
FocState foc_get_state();
float    foc_get_current_deg();
float    foc_get_target_deg();
bool     foc_is_fault_latched();
