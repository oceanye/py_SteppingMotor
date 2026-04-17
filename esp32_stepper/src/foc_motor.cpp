// esp32_stepper/src/foc_motor.cpp
#include "foc_motor.h"
#include "config.h"
#include <atomic>

// ── 跨核 atomic 镜像变量 ──
// ESP32-S3 32-bit aligned float 硬件读写原子，std::atomic 消除编译器缓存/重排
static std::atomic<float>   g_target_deg    {0.0f};
static std::atomic<float>   g_current_deg   {0.0f};
static std::atomic<bool>    g_enable_req    {false};
static std::atomic<bool>    g_fault_latched {false};
static std::atomic<float>   g_voltage_limit {FOC_INITIAL_V_LIMIT};
static std::atomic<uint8_t> g_state         {FOC_STATE_DISABLED};

// ── 初始化（Task B3 将填充 SimpleFOC 对象与 Core 0 任务）──
void foc_init() {
    // TODO(B3): Wire.begin, sensor.init, driver.init, motor setup, xTaskCreatePinnedToCore
}

// ── Core 1 API ──
bool foc_set_target_deg(float deg) {
    if (deg > FOC_MAX_ANGLE_ABS || deg < -FOC_MAX_ANGLE_ABS) return false;
    g_target_deg.store(deg);
    return true;
}

bool foc_request_enable(bool en) {
    if (en && g_state.load() == FOC_STATE_FAULT) return false;
    g_enable_req.store(en);
    return true;
}

bool foc_clear_fault() {
    if (g_state.load() != FOC_STATE_FAULT) return false;
    g_fault_latched.store(false);
    g_state.store(FOC_STATE_DISABLED);
    return true;
}

void foc_latch_fault() {
    g_fault_latched.store(true);
    g_state.store(FOC_STATE_FAULT);
    g_enable_req.store(false);
    // motor.disable() 在 Core 0 任务中执行（通过 g_state 触发）
}

bool foc_set_voltage_limit(float v) {
    if (v <= 0.0f || v > FOC_MAX_V_LIMIT) return false;
    g_voltage_limit.store(v);
    return true;
}

void foc_home() {
    // TODO(B3): 使用 motor.shaft_angle 设置 offset
}

bool foc_set_pole_pairs_and_store(int n) {
    if (n < 1 || n > 50) return false;
    // TODO(C3): 存 NVS
    return true;
}

// ── 状态查询 ──
FocState foc_get_state()        { return (FocState)g_state.load(); }
float    foc_get_current_deg()  { return g_current_deg.load(); }
float    foc_get_target_deg()   { return g_target_deg.load(); }
bool     foc_is_fault_latched() { return g_fault_latched.load(); }
