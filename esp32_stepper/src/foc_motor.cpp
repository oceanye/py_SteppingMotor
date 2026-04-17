// esp32_stepper/src/foc_motor.cpp
#include "foc_motor.h"
#include "config.h"
#include <Arduino.h>
#include <SimpleFOC.h>
#include <atomic>

// ── SimpleFOC 对象（文件级静态）──
static MagneticSensorI2C sensor(AS5600_I2C);
static BLDCMotor         motor(FOC_POLE_PAIRS_DEFAULT);
static BLDCDriver3PWM    driver(PIN_FOC_M1, PIN_FOC_M2, PIN_FOC_M3, PIN_FOC_EN);

static TaskHandle_t s_foc_task_handle = nullptr;
static float        s_home_offset_rad = 0.0f;

// ── 跨核 atomic 镜像变量 ──
static std::atomic<float>   g_target_deg    {0.0f};
static std::atomic<float>   g_current_deg   {0.0f};
static std::atomic<bool>    g_enable_req    {false};
static std::atomic<bool>    g_fault_latched {false};
static std::atomic<float>   g_voltage_limit {FOC_INITIAL_V_LIMIT};
static std::atomic<uint8_t> g_state         {FOC_STATE_DISABLED};

// 前置声明
static void foc_task(void* param);

// ── 初始化 ──
void foc_init() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  sensor.init();
  Serial.println("[FOC] AS5600 sensor init done");

  driver.voltage_power_supply = FOC_PSU_VOLTAGE;
  driver.voltage_limit        = FOC_PSU_VOLTAGE;
  driver.init();

  motor.linkSensor(&sensor);
  motor.linkDriver(&driver);
  motor.voltage_limit  = FOC_INITIAL_V_LIMIT;
  motor.velocity_limit = FOC_VELOCITY_LIMIT;
  motor.controller     = MotionControlType::angle;
  motor.init();
  // 不调 motor.initFOC()，等 foc_request_enable(true) 触发

  pinMode(PIN_FOC_NFAULT, INPUT_PULLUP);

  xTaskCreatePinnedToCore(foc_task, "FOC", FOC_TASK_STACK, nullptr,
                          FOC_TASK_PRIORITY, &s_foc_task_handle, FOC_TASK_CORE);
  Serial.println("[FOC] Core 0 task started");
}

// ── Core 0 FOC 任务：状态机 + 闭环 ──
static void foc_task(void* /*param*/) {
  static bool s_aligned_once = false;  // 防止重复 initFOC()

  for (;;) {
    // 1. 读镜像 → SimpleFOC
    motor.target        = g_target_deg.load() * PI / 180.0f;
    motor.voltage_limit = g_voltage_limit.load();

    // 2. 状态转换
    FocState state = (FocState)g_state.load();
    bool want_en   = g_enable_req.load();

    if (state == FOC_STATE_DISABLED && want_en) {
      g_state.store(FOC_STATE_ALIGNING);
      state = FOC_STATE_ALIGNING;
    }
    if (state == FOC_STATE_ALIGNING) {
      if (!s_aligned_once) {
        motor.initFOC();  // 首次才跑对齐（阻塞 1-3 秒，电机缓转）
        s_aligned_once = true;
      }
      motor.enable();
      g_state.store(FOC_STATE_RUNNING);
      state = FOC_STATE_RUNNING;
    }
    if ((state == FOC_STATE_RUNNING || state == FOC_STATE_ALIGNING) && !want_en) {
      motor.disable();
      g_state.store(FOC_STATE_DISABLED);
      state = FOC_STATE_DISABLED;
    }
    if (state == FOC_STATE_FAULT) {
      // latch 态由 foc_latch_fault() 设置，这里确认 motor 已 disable
      motor.disable();
    }

    // 3. sensor 永远 update（即使 DISABLED 也要让 GUI 读到实时角度）
    sensor.update();

    // 4. 闭环仅 RUNNING 态跑
    if (state == FOC_STATE_RUNNING && !g_fault_latched.load()) {
      motor.loopFOC();
      motor.move();
    }

    // 5. 更新输出镜像
    float shaft = motor.shaft_angle - s_home_offset_rad;
    g_current_deg.store(shaft * 180.0f / PI);

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
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
}

bool foc_set_voltage_limit(float v) {
  if (v <= 0.0f || v > FOC_MAX_V_LIMIT) return false;
  g_voltage_limit.store(v);
  return true;
}

void foc_home() {
  s_home_offset_rad = motor.shaft_angle;  // Core 1 读 motor 成员，32-bit float OK
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
