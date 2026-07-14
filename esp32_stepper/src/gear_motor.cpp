// esp32_stepper/src/gear_motor.cpp
// GEAR 模式：DRV8871 (H 桥) + AB 增量编码器 + 位置 PID 闭环。
// 实现 foc_motor.h 里的"闭环单轴 API"，让 protocol.cpp 不用区分底层电机种类。
//
// 仅在 -DDRIVE_MODE_GEAR 构建下编译（platformio.ini: env:esp32s3_gear）。
//
// Phase 1 限制：
//   • NUM_AXES = 1（只接了一个 N20）
//   • PCNT 16-bit 硬件计数器（±32767）：默认齿轮比 1000 时 ≈ ±421°，
//     单方向连续转 > 1.17 圈会溢出，软件没接溢出回调；Phase 2 再补
//   • 无 nFAULT 输入（DRV8871 无此线）；fault latch 仅供协议对齐
#include "foc_motor.h"
#include "config.h"
#include <Arduino.h>
#include <Preferences.h>
#include <atomic>
#include <math.h>
#include "driver/pcnt.h"
#include "driver/gpio.h"

struct GearAxis {
  // 硬件资源（foc_init 里填）
  pcnt_unit_t pcnt_unit;
  int pin_in1, pin_in2, pin_enca, pin_encb;
  int ledc_ch_in1, ledc_ch_in2;

  // 持久化 / 可调参数
  float gear_ratio;
  float kp, ki, kd;
  float duty_cap_pct;   // 0-100

  // PID 内部状态（仅控制环任务访问）
  float integrator;
  float prev_error;
  int32_t home_offset_counts;

  // 跨核镜像（Core1 API 读 / Core0 任务读写）
  std::atomic<float>   target_deg;
  std::atomic<float>   current_deg;
  std::atomic<bool>    enable_req;
  std::atomic<bool>    fault_latched;
  std::atomic<uint8_t> state;
  std::atomic<int32_t> encoder_count;
};

static GearAxis axes[NUM_AXES];
static TaskHandle_t s_task[NUM_AXES] = { nullptr };
static Preferences s_prefs;

static bool valid_axis(int a) { return a >= 0 && a < NUM_AXES; }

static float counts_per_degree(const GearAxis& ax) {
  // counts/output_rev = base_PPR × quadrature × gear_ratio
  return (float)GEAR_ENC_PPR_BASE * GEAR_ENC_QUAD_MULT * ax.gear_ratio / 360.0f;
}

// PID 输出 → IN1/IN2 PWM（sign-magnitude 双 PWM 模式）
static void set_pwm(GearAxis& ax, float pid_output) {
  const float duty_cap_units = ax.duty_cap_pct * 0.01f * GEAR_PWM_MAX;
  float mag = fabsf(pid_output);
  if (mag > duty_cap_units) mag = duty_cap_units;
  int pwm = (int)(mag + 0.5f);
  if (pwm < 0) pwm = 0;
  if (pwm > GEAR_PWM_MAX) pwm = GEAR_PWM_MAX;

  if (pid_output > 0.5f) {            // 正转
    ledcWrite(ax.ledc_ch_in1, pwm);
    ledcWrite(ax.ledc_ch_in2, 0);
  } else if (pid_output < -0.5f) {    // 反转
    ledcWrite(ax.ledc_ch_in1, 0);
    ledcWrite(ax.ledc_ch_in2, pwm);
  } else {                            // 空转（coast）
    ledcWrite(ax.ledc_ch_in1, 0);
    ledcWrite(ax.ledc_ch_in2, 0);
  }
}

// Core 0 闭环任务，1 ms tick
static void gear_task(void* arg) {
  int axis = (int)(intptr_t)arg;
  GearAxis& ax = axes[axis];
  uint32_t last_ms = millis();

  for (;;) {
    uint32_t now_ms = millis();
    float dt_s = (now_ms - last_ms) * 1e-3f;
    if (dt_s < 1e-4f) dt_s = 1e-4f;
    last_ms = now_ms;

    // ── 读编码器 ──
    int16_t hw = 0;
    pcnt_get_counter_value(ax.pcnt_unit, &hw);
    int32_t total = (int32_t)hw - ax.home_offset_counts;
    ax.encoder_count.store(total);
    float cur_deg = (float)total / counts_per_degree(ax);
    ax.current_deg.store(cur_deg);

    FocState state = (FocState)ax.state.load();
    bool want_en   = ax.enable_req.load();

    // ── 状态机 ──
    if (state == FOC_STATE_DISABLED && want_en) {
      ax.integrator = 0.0f;
      ax.prev_error = 0.0f;
      ax.state.store(FOC_STATE_RUNNING);
      state = FOC_STATE_RUNNING;
    }
    if (state == FOC_STATE_RUNNING && !want_en) {
      set_pwm(ax, 0.0f);
      ax.state.store(FOC_STATE_DISABLED);
      state = FOC_STATE_DISABLED;
    }
    if (state == FOC_STATE_FAULT) {
      set_pwm(ax, 0.0f);
    }

    // ── PID 控制 ──
    if (state == FOC_STATE_RUNNING && !ax.fault_latched.load()) {
      float tgt = ax.target_deg.load();
      float err = tgt - cur_deg;

      ax.integrator += err * dt_s;
      if (ax.integrator >  GEAR_INTEGRATOR_CLAMP) ax.integrator =  GEAR_INTEGRATOR_CLAMP;
      if (ax.integrator < -GEAR_INTEGRATOR_CLAMP) ax.integrator = -GEAR_INTEGRATOR_CLAMP;

      float deriv = (err - ax.prev_error) / dt_s;
      ax.prev_error = err;

      float u = ax.kp * err + ax.ki * ax.integrator + ax.kd * deriv;
      set_pwm(ax, u);
    }

    vTaskDelay(GEAR_TASK_TICK_MS / portTICK_PERIOD_MS);
  }
}

// 单个 PCNT unit 配置成正交 X4 解码器，A/B 互为彼此的 ctrl
static void init_pcnt(pcnt_unit_t unit, int pin_a, int pin_b) {
  pcnt_config_t conf_a = {};
  conf_a.pulse_gpio_num = pin_a;
  conf_a.ctrl_gpio_num  = pin_b;
  conf_a.lctrl_mode     = PCNT_MODE_REVERSE;
  conf_a.hctrl_mode     = PCNT_MODE_KEEP;
  conf_a.pos_mode       = PCNT_COUNT_INC;
  conf_a.neg_mode       = PCNT_COUNT_DEC;
  conf_a.counter_h_lim  = 32767;
  conf_a.counter_l_lim  = -32768;
  conf_a.unit           = unit;
  conf_a.channel        = PCNT_CHANNEL_0;
  pcnt_unit_config(&conf_a);

  pcnt_config_t conf_b = {};
  conf_b.pulse_gpio_num = pin_b;
  conf_b.ctrl_gpio_num  = pin_a;
  conf_b.lctrl_mode     = PCNT_MODE_KEEP;
  conf_b.hctrl_mode     = PCNT_MODE_REVERSE;
  // 修复：两个通道在正转时都应该加（X4 解码）。之前 pos=DEC / neg=INC 把 B 通道
  // 配成了反向计数，结果 A 通道 +1 和 B 通道 -1 相互抵消，counter 一直在 ±1 抖。
  conf_b.pos_mode       = PCNT_COUNT_INC;
  conf_b.neg_mode       = PCNT_COUNT_DEC;
  conf_b.counter_h_lim  = 32767;
  conf_b.counter_l_lim  = -32768;
  conf_b.unit           = unit;
  conf_b.channel        = PCNT_CHANNEL_1;
  pcnt_unit_config(&conf_b);

  // 噪声过滤：忽略 < 100 APB cycle (~1.25 µs at 80 MHz) 的脉冲
  pcnt_set_filter_value(unit, 100);
  pcnt_filter_enable(unit);

  // v0 bench：用内置上拉（无外部 4.7kΩ）
  gpio_pullup_en((gpio_num_t)pin_a);
  gpio_pullup_en((gpio_num_t)pin_b);
  gpio_pulldown_dis((gpio_num_t)pin_a);
  gpio_pulldown_dis((gpio_num_t)pin_b);

  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

void foc_init() {
  // ── 填充硬件资源表 ──
  axes[0].pcnt_unit   = PCNT_UNIT_0;
  axes[0].pin_in1     = PIN_GEAR_IN1_0;
  axes[0].pin_in2     = PIN_GEAR_IN2_0;
  axes[0].pin_enca    = PIN_GEAR_ENCA_0;
  axes[0].pin_encb    = PIN_GEAR_ENCB_0;
  axes[0].ledc_ch_in1 = GEAR_LEDC_CH_IN1_0;
  axes[0].ledc_ch_in2 = GEAR_LEDC_CH_IN2_0;

#if NUM_AXES >= 2
  axes[1].pcnt_unit   = PCNT_UNIT_1;
  axes[1].pin_in1     = PIN_GEAR_IN1_1;
  axes[1].pin_in2     = PIN_GEAR_IN2_1;
  axes[1].pin_enca    = PIN_GEAR_ENCA_1;
  axes[1].pin_encb    = PIN_GEAR_ENCB_1;
  axes[1].ledc_ch_in1 = GEAR_LEDC_CH_IN1_1;
  axes[1].ledc_ch_in2 = GEAR_LEDC_CH_IN2_1;
#endif

  for (int a = 0; a < NUM_AXES; a++) {
    GearAxis& ax = axes[a];

    // NVS 读齿轮比（key = "gr<axis>"）
    s_prefs.begin("gear", /*readOnly=*/true);
    char key[8]; snprintf(key, sizeof(key), "gr%d", a);
    ax.gear_ratio = s_prefs.getFloat(key, GEAR_RATIO_DEFAULT);
    s_prefs.end();

    ax.kp = GEAR_DEFAULT_KP;
    ax.ki = GEAR_DEFAULT_KI;
    ax.kd = GEAR_DEFAULT_KD;
    ax.duty_cap_pct = (float)GEAR_PWM_DUTY_CAP_PCT;
    ax.integrator = 0.0f;
    ax.prev_error = 0.0f;
    ax.home_offset_counts = 0;

    ax.target_deg.store(0.0f);
    ax.current_deg.store(0.0f);
    ax.enable_req.store(false);
    ax.fault_latched.store(false);
    ax.state.store(FOC_STATE_DISABLED);
    ax.encoder_count.store(0);

    Serial.print("[GEAR "); Serial.print(a); Serial.print("] gear_ratio = ");
    Serial.print(ax.gear_ratio, 1);
    Serial.print("  (counts/° = ");
    Serial.print(counts_per_degree(ax), 2); Serial.println(")");

    // PCNT（正交编码器 + 内置上拉）
    init_pcnt(ax.pcnt_unit, ax.pin_enca, ax.pin_encb);
    Serial.print("[GEAR "); Serial.print(a); Serial.println("] encoder PCNT init done");

    // LEDC（PWM 双通道，共享 timer，sign-magnitude）
    ledcSetup(ax.ledc_ch_in1, GEAR_PWM_FREQ_HZ, GEAR_PWM_RES_BITS);
    ledcSetup(ax.ledc_ch_in2, GEAR_PWM_FREQ_HZ, GEAR_PWM_RES_BITS);
    ledcAttachPin(ax.pin_in1, ax.ledc_ch_in1);
    ledcAttachPin(ax.pin_in2, ax.ledc_ch_in2);
    ledcWrite(ax.ledc_ch_in1, 0);
    ledcWrite(ax.ledc_ch_in2, 0);
    Serial.print("[GEAR "); Serial.print(a); Serial.println("] LEDC PWM init done");

    // Core 0 控制环任务
    char tname[16]; snprintf(tname, sizeof(tname), "gear%d", a);
    xTaskCreatePinnedToCore(gear_task, tname, GEAR_TASK_STACK,
                            (void*)(intptr_t)a, GEAR_TASK_PRIORITY,
                            &s_task[a], GEAR_TASK_CORE);
    Serial.print("[GEAR "); Serial.print(a); Serial.println("] Core 0 task started");
  }
}

// ============ Common API（foc_motor.h）============
bool foc_set_target_deg(int axis, float deg) {
  if (!valid_axis(axis)) return false;
  if (deg > GEAR_MAX_ANGLE_ABS || deg < -GEAR_MAX_ANGLE_ABS) return false;
  axes[axis].target_deg.store(deg);
  return true;
}

bool foc_request_enable(int axis, bool en) {
  if (!valid_axis(axis)) return false;
  if (en && axes[axis].state.load() == FOC_STATE_FAULT) return false;
  axes[axis].enable_req.store(en);
  return true;
}

bool foc_clear_fault(int axis) {
  if (!valid_axis(axis)) return false;
  if (axes[axis].state.load() != FOC_STATE_FAULT) return false;
  axes[axis].fault_latched.store(false);
  axes[axis].state.store(FOC_STATE_DISABLED);
  return true;
}

void foc_latch_fault(int axis) {
  if (!valid_axis(axis)) return;
  axes[axis].fault_latched.store(true);
  axes[axis].state.store(FOC_STATE_FAULT);
  axes[axis].enable_req.store(false);
}

// GEAR 模式：参数 v 重解释为 PWM duty cap 百分比（0-100）
bool foc_set_voltage_limit(int axis, float v) {
  if (!valid_axis(axis)) return false;
  if (v <= 0.0f || v > 100.0f) return false;
  axes[axis].duty_cap_pct = v;
  return true;
}

void foc_home(int axis) {
  if (!valid_axis(axis)) return;
  int16_t hw = 0;
  pcnt_get_counter_value(axes[axis].pcnt_unit, &hw);
  axes[axis].home_offset_counts = hw;
  axes[axis].target_deg.store(0.0f);
}

bool foc_set_p_angle(int axis, float p) {
  if (!valid_axis(axis)) return false;
  if (p < 0.0f || p > 1000.0f) return false;
  axes[axis].kp = p;
  return true;
}

FocState foc_get_state(int axis) {
  if (!valid_axis(axis)) return FOC_STATE_DISABLED;
  return (FocState)axes[axis].state.load();
}
float foc_get_current_deg(int axis) {
  if (!valid_axis(axis)) return 0.0f;
  return axes[axis].current_deg.load();
}
float foc_get_target_deg(int axis) {
  if (!valid_axis(axis)) return 0.0f;
  return axes[axis].target_deg.load();
}
bool foc_is_fault_latched(int axis) {
  if (!valid_axis(axis)) return false;
  return axes[axis].fault_latched.load();
}

// ============ GEAR-only API ============
bool foc_set_gear_ratio_and_store(int axis, float ratio) {
  if (!valid_axis(axis)) return false;
  if (ratio < 1.0f || ratio > 10000.0f) return false;
  axes[axis].gear_ratio = ratio;
  s_prefs.begin("gear", /*readOnly=*/false);
  char key[8]; snprintf(key, sizeof(key), "gr%d", axis);
  s_prefs.putFloat(key, ratio);
  s_prefs.end();
  return true;
}

bool foc_set_p_integral(int axis, float i) {
  if (!valid_axis(axis)) return false;
  if (i < 0.0f || i > 1000.0f) return false;
  axes[axis].ki = i;
  axes[axis].integrator = 0.0f;   // 清积分避免突变
  return true;
}

bool foc_set_p_derivative(int axis, float d) {
  if (!valid_axis(axis)) return false;
  if (d < 0.0f || d > 100.0f) return false;
  axes[axis].kd = d;
  return true;
}

void foc_run_diagnostics(int axis) {
  if (!valid_axis(axis)) { Serial.println("ERR:bad axis"); return; }
  GearAxis& ax = axes[axis];
  int16_t hw = 0;
  pcnt_get_counter_value(ax.pcnt_unit, &hw);
  int raw_a = digitalRead(ax.pin_enca);
  int raw_b = digitalRead(ax.pin_encb);

  Serial.print("[DIAG "); Serial.print(axis); Serial.println("]");
  Serial.print("  pins: IN1="); Serial.print(ax.pin_in1);
  Serial.print(" IN2=");        Serial.print(ax.pin_in2);
  Serial.print(" ENC_A=");      Serial.print(ax.pin_enca);
  Serial.print(" ENC_B=");      Serial.println(ax.pin_encb);
  Serial.print("  raw_pins: A="); Serial.print(raw_a);
  Serial.print(" B=");            Serial.println(raw_b);
  Serial.print("  gear_ratio="); Serial.println(ax.gear_ratio, 1);
  Serial.print("  pid Kp=");    Serial.print(ax.kp, 3);
  Serial.print(" Ki=");          Serial.print(ax.ki, 3);
  Serial.print(" Kd=");          Serial.println(ax.kd, 3);
  Serial.print("  duty_cap=");   Serial.print(ax.duty_cap_pct, 1); Serial.println("%");
  Serial.print("  enc_raw=");    Serial.print(hw);
  Serial.print(" home_offset="); Serial.println(ax.home_offset_counts);
  Serial.print("  enc_total=");  Serial.print(ax.encoder_count.load());
  Serial.print(" current_deg="); Serial.println(ax.current_deg.load(), 2);
  Serial.print("  target_deg="); Serial.print(ax.target_deg.load(), 2);
  Serial.print(" state=");       Serial.print((int)ax.state.load());
  Serial.print(" fault=");       Serial.println(ax.fault_latched.load() ? 1 : 0);
  uint32_t pwm1 = ledcRead(ax.ledc_ch_in1);
  uint32_t pwm2 = ledcRead(ax.ledc_ch_in2);
  Serial.print("  pwm: IN1="); Serial.print(pwm1);
  Serial.print(" (");          Serial.print(pwm1 * 100.0f / GEAR_PWM_MAX, 1); Serial.print("%) ");
  Serial.print("IN2=");        Serial.print(pwm2);
  Serial.print(" (");          Serial.print(pwm2 * 100.0f / GEAR_PWM_MAX, 1); Serial.println("%)");
}

// 5 秒实时编码器状态打印（10 Hz）。手转电机轴边看边读：
//   • A/B 跳变 + cnt 变化  → 编码器正常、PCNT 工作
//   • A/B 跳变 + cnt 不变  → PCNT 配置问题
//   • A/B 一直 1（高电平）→ 编码器没在驱动（最可能：没供电 / 接线虚 / 轴没真在转）
//   • A/B 一直 0           → 信号线短路到 GND
void foc_run_diagnostics_live(int axis) {
  if (!valid_axis(axis)) { Serial.println("ERR:bad axis"); return; }
  GearAxis& ax = axes[axis];

  Serial.println("[LIVE] 5s @ 10Hz — turn the encoder shaft now");
  uint32_t end_ms = millis() + 5000;
  while ((int32_t)(millis() - end_ms) < 0) {
    int16_t hw = 0;
    pcnt_get_counter_value(ax.pcnt_unit, &hw);
    int raw_a = digitalRead(ax.pin_enca);
    int raw_b = digitalRead(ax.pin_encb);
    Serial.print("A="); Serial.print(raw_a);
    Serial.print(" B="); Serial.print(raw_b);
    Serial.print(" cnt="); Serial.println(hw);
    delay(100);
  }
  Serial.println("[LIVE] done");
}
