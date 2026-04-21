// esp32_stepper/src/foc_motor.cpp
#include "foc_motor.h"
#include "config.h"
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Preferences.h>
#include <atomic>

// ── SimpleFOC 对象（文件级静态）──
static MagneticSensorI2C sensor(AS5600_I2C);
static BLDCMotor         motor(FOC_POLE_PAIRS_DEFAULT);
static BLDCDriver3PWM    driver(PIN_FOC_M1, PIN_FOC_M2, PIN_FOC_M3, PIN_FOC_EN);

static TaskHandle_t s_foc_task_handle = nullptr;
static float        s_home_offset_rad = 0.0f;
static Preferences  s_prefs;
static bool         s_aligned_once    = false;  // 文件级，允许 foc_clear_fault() 复位

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
  // 从 NVS 读极对数覆盖默认值。motor.pole_pairs 是 public 成员，
  // 在 motor.init() 之前赋值即生效。
  s_prefs.begin("foc", /*readOnly=*/true);
  int pp = s_prefs.getInt("pp", FOC_POLE_PAIRS_DEFAULT);
  s_prefs.end();
  motor.pole_pairs = pp;
  Serial.print("[FOC] pole_pairs from NVS = "); Serial.println(pp);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  sensor.init();
  Serial.println("[FOC] AS5600 sensor init done");

  driver.voltage_power_supply = FOC_PSU_VOLTAGE;
  driver.voltage_limit        = FOC_PSU_VOLTAGE;
  driver.init();

  motor.linkSensor(&sensor);
  motor.linkDriver(&driver);
  motor.useMonitoring(Serial);  // 让 SimpleFOC 的 MOT: 诊断信息打印到串口
  motor.voltage_limit         = FOC_INITIAL_V_LIMIT;
  motor.voltage_sensor_align  = 6.0f;   // 24V PSU 下对齐需要更大扭矩，默认 3V 推不动
  motor.velocity_limit        = FOC_VELOCITY_LIMIT;
  motor.controller            = MotionControlType::angle;

  // 位置环 PID 调参（24V PSU 实测稳定值，2208 gimbal）
  motor.P_angle.P        = 25.0f;  // PSU 升到 24V 后 PA=40 会振荡，降到 25 稳
  motor.PID_velocity.P   = 0.2f;   // 速度环 P，PSU=24 下 0.4 过激进
  motor.PID_velocity.I   = 2.0f;
  motor.PID_velocity.D   = 0.0f;
  motor.PID_velocity.output_ramp = 1000.0f;
  motor.LPF_velocity.Tf  = 0.02f;

  motor.init();
  // 不调 motor.initFOC()，等 foc_request_enable(true) 触发

  pinMode(PIN_FOC_NFAULT, INPUT_PULLUP);

  xTaskCreatePinnedToCore(foc_task, "FOC", FOC_TASK_STACK, nullptr,
                          FOC_TASK_PRIORITY, &s_foc_task_handle, FOC_TASK_CORE);
  Serial.println("[FOC] Core 0 task started");
}

// ── Core 0 FOC 任务：状态机 + 闭环 ──
static void foc_task(void* /*param*/) {
  for (;;) {
    // 1. 读镜像 → SimpleFOC
    // target 用户坐标（以 home 为 0）转 sensor 原生坐标：加 home_offset
    motor.target        = g_target_deg.load() * PI / 180.0f + s_home_offset_rad;
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
        int ok = motor.initFOC();  // 返回 1 成功 / 0 失败
        if (!ok) {
          Serial.println("FOC,FAULT");
          g_fault_latched.store(true);
          g_state.store(FOC_STATE_FAULT);
          g_enable_req.store(false);
          state = FOC_STATE_FAULT;
          vTaskDelay(1 / portTICK_PERIOD_MS);
          continue;  // 下一轮循环处理 FAULT
        }
        s_aligned_once = true;
      }
      // 使能前把 target 设为当前角度（用户坐标），避免大角度瞬间跳跃
      sensor.update();
      float cur_user_rad = sensor.getAngle() - s_home_offset_rad;
      g_target_deg.store(cur_user_rad * 180.0f / PI);
      motor.target = sensor.getAngle();

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
    // 注意：motor.shaft_angle 只在 loopFOC() 调用时更新，所以 DISABLED 态不刷新。
    // 直接用 sensor.getAngle() 读 AS5600 连续累计角（rad），与使能态无关。
    float shaft = sensor.getAngle() - s_home_offset_rad;
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
  s_aligned_once = false;  // 故障可能让 motor 内部状态失效，强制下次 EN,1 重新对齐
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
  sensor.update();
  s_home_offset_rad = sensor.getAngle();  // 当前 sensor 位置即新的 0°
  g_target_deg.store(0.0f);               // 同时把目标归零，motor 保持原地不动
}

bool foc_set_pole_pairs_and_store(int n) {
  if (n < 1 || n > 50) return false;
  s_prefs.begin("foc", /*readOnly=*/false);
  s_prefs.putInt("pp", n);
  s_prefs.end();
  // 下次启动生效（foc_init 读 NVS 时写 motor.pole_pairs）
  return true;
}

bool foc_set_p_angle(float p) {
  if (p < 0.1f || p > 50.0f) return false;
  motor.P_angle.P = p;  // 立即生效，Core 0 的 motor.move() 下个循环就用
  return true;
}

bool foc_set_p_velocity(float p) {
  if (p < 0.01f || p > 2.0f) return false;
  motor.PID_velocity.P = p;  // 立即生效
  return true;
}

// ── 状态查询 ──
FocState foc_get_state()        { return (FocState)g_state.load(); }
float    foc_get_current_deg()  { return g_current_deg.load(); }
float    foc_get_target_deg()   { return g_target_deg.load(); }
bool     foc_is_fault_latched() { return g_fault_latched.load(); }
