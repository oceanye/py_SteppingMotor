// GEAR mode: two independent DRV8871 + quadrature encoder position loops.
#include "foc_motor.h"
#include "config.h"
#include <Arduino.h>
#include <Preferences.h>
#include <atomic>
#include <cmath>
#include "driver/pcnt.h"
#include "driver/gpio.h"
#include "serial_tx.h"

struct GearAxis {
  pcnt_unit_t pcnt_unit;
  int pin_in1, pin_in2, pin_enca, pin_encb;
  int ledc_ch_in1, ledc_ch_in2;
  // Serial-task writes/control-task reads are atomic. The PID state and
  // position_counts are owned exclusively by the control task.
  std::atomic<float> gear_ratio, kp, ki, kd, duty_cap_pct;
  std::atomic<bool> reset_pid, home_requested;
  float integrator, prev_error;
  int64_t position_counts;
  int64_t published_counts;  // guarded by s_data_mux
  std::atomic<float> target_deg, current_deg;
  std::atomic<bool> enable_req, fault_latched;
  std::atomic<uint8_t> state;
};

static GearAxis axes[NUM_GEAR_AXES];
static TaskHandle_t s_task[NUM_GEAR_AXES] = {};
static Preferences s_prefs;
static portMUX_TYPE s_data_mux = portMUX_INITIALIZER_UNLOCKED;

static bool valid_axis(int axis) { return axis >= 0 && axis < NUM_GEAR_AXES; }
static float counts_per_degree(const GearAxis& axis) {
  return (float)GEAR_ENC_PPR_BASE * GEAR_ENC_QUAD_MULT * axis.gear_ratio.load() / 360.0f;
}
static int64_t get_published_counts(const GearAxis& axis) {
  int64_t value;
  portENTER_CRITICAL(&s_data_mux);
  value = axis.published_counts;
  portEXIT_CRITICAL(&s_data_mux);
  return value;
}
static void publish_counts(GearAxis& axis) {
  portENTER_CRITICAL(&s_data_mux);
  axis.published_counts = axis.position_counts;
  portEXIT_CRITICAL(&s_data_mux);
}
static void set_pwm(GearAxis& axis, float output) {
  const float cap = axis.duty_cap_pct.load() * 0.01f * GEAR_PWM_MAX;
  float magnitude = fabsf(output);
  if (magnitude > cap) magnitude = cap;
  int pwm = (int)(magnitude + 0.5f);
  if (pwm < 0) pwm = 0;
  if (pwm > GEAR_PWM_MAX) pwm = GEAR_PWM_MAX;
  if (output > 0.5f) {
    ledcWrite(axis.ledc_ch_in1, pwm); ledcWrite(axis.ledc_ch_in2, 0);
  } else if (output < -0.5f) {
    ledcWrite(axis.ledc_ch_in1, 0); ledcWrite(axis.ledc_ch_in2, pwm);
  } else {
    ledcWrite(axis.ledc_ch_in1, 0); ledcWrite(axis.ledc_ch_in2, 0);
  }
}

// Keeping each hardware delta small prevents the signed 16-bit PCNT counter
// from saturating. The short pause also makes read+clear one coherent sample.
static int16_t consume_pcnt_delta(pcnt_unit_t unit) {
  int16_t delta = 0;
  pcnt_counter_pause(unit);
  pcnt_get_counter_value(unit, &delta);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
  return delta;
}

static void gear_task(void* argument) {
  const int index = (int)(intptr_t)argument;
  GearAxis& axis = axes[index];
  uint32_t last_ms = millis();
  for (;;) {
    const uint32_t now_ms = millis();
    float dt_s = (now_ms - last_ms) * 1e-3f;
    if (dt_s < 1e-4f) dt_s = 1e-4f;
    last_ms = now_ms;

    axis.position_counts += consume_pcnt_delta(axis.pcnt_unit);
    if (axis.home_requested.exchange(false)) {
      axis.position_counts = 0;
      axis.integrator = 0.0f;
      axis.prev_error = 0.0f;
    }
    publish_counts(axis);
    const float current = (float)axis.position_counts / counts_per_degree(axis);
    axis.current_deg.store(current);
    if (axis.reset_pid.exchange(false)) {
      axis.integrator = 0.0f;
      axis.prev_error = 0.0f;
    }

    FocState state = (FocState)axis.state.load();
    const bool enable = axis.enable_req.load();
    if (state == FOC_STATE_DISABLED && enable && !axis.fault_latched.load()) {
      axis.integrator = 0.0f; axis.prev_error = 0.0f;
      axis.state.store(FOC_STATE_RUNNING); state = FOC_STATE_RUNNING;
    }
    if (state == FOC_STATE_RUNNING && !enable) {
      set_pwm(axis, 0.0f);
      axis.state.store(FOC_STATE_DISABLED); state = FOC_STATE_DISABLED;
    }
    if (state == FOC_STATE_FAULT) set_pwm(axis, 0.0f);
    if (state == FOC_STATE_RUNNING && !axis.fault_latched.load()) {
      const float error = axis.target_deg.load() - current;
      axis.integrator += error * dt_s;
      if (axis.integrator > GEAR_INTEGRATOR_CLAMP) axis.integrator = GEAR_INTEGRATOR_CLAMP;
      if (axis.integrator < -GEAR_INTEGRATOR_CLAMP) axis.integrator = -GEAR_INTEGRATOR_CLAMP;
      const float derivative = (error - axis.prev_error) / dt_s;
      axis.prev_error = error;
      set_pwm(axis, axis.kp.load() * error + axis.ki.load() * axis.integrator +
                    axis.kd.load() * derivative);
    }
    vTaskDelay(pdMS_TO_TICKS(GEAR_TASK_TICK_MS));
  }
}

static void init_pcnt(pcnt_unit_t unit, int pin_a, int pin_b) {
  pcnt_config_t a = {};
  a.pulse_gpio_num = pin_a; a.ctrl_gpio_num = pin_b;
  a.lctrl_mode = PCNT_MODE_REVERSE; a.hctrl_mode = PCNT_MODE_KEEP;
  a.pos_mode = PCNT_COUNT_INC; a.neg_mode = PCNT_COUNT_DEC;
  a.counter_h_lim = 32767; a.counter_l_lim = -32768;
  a.unit = unit; a.channel = PCNT_CHANNEL_0; pcnt_unit_config(&a);
  pcnt_config_t b = {};
  b.pulse_gpio_num = pin_b; b.ctrl_gpio_num = pin_a;
  b.lctrl_mode = PCNT_MODE_KEEP; b.hctrl_mode = PCNT_MODE_REVERSE;
  b.pos_mode = PCNT_COUNT_INC; b.neg_mode = PCNT_COUNT_DEC;
  b.counter_h_lim = 32767; b.counter_l_lim = -32768;
  b.unit = unit; b.channel = PCNT_CHANNEL_1; pcnt_unit_config(&b);
  pcnt_set_filter_value(unit, 100); pcnt_filter_enable(unit);
  gpio_pullup_en((gpio_num_t)pin_a); gpio_pullup_en((gpio_num_t)pin_b);
  gpio_pulldown_dis((gpio_num_t)pin_a); gpio_pulldown_dis((gpio_num_t)pin_b);
  pcnt_counter_pause(unit); pcnt_counter_clear(unit); pcnt_counter_resume(unit);
}

void foc_init() {
  axes[0].pcnt_unit = PCNT_UNIT_0;
  axes[0].pin_in1 = PIN_GEAR_IN1_0; axes[0].pin_in2 = PIN_GEAR_IN2_0;
  axes[0].pin_enca = PIN_GEAR_ENCA_0; axes[0].pin_encb = PIN_GEAR_ENCB_0;
  axes[0].ledc_ch_in1 = GEAR_LEDC_CH_IN1_0; axes[0].ledc_ch_in2 = GEAR_LEDC_CH_IN2_0;
#if NUM_GEAR_AXES >= 2
  axes[1].pcnt_unit = PCNT_UNIT_1;
  axes[1].pin_in1 = PIN_GEAR_IN1_1; axes[1].pin_in2 = PIN_GEAR_IN2_1;
  axes[1].pin_enca = PIN_GEAR_ENCA_1; axes[1].pin_encb = PIN_GEAR_ENCB_1;
  axes[1].ledc_ch_in1 = GEAR_LEDC_CH_IN1_1; axes[1].ledc_ch_in2 = GEAR_LEDC_CH_IN2_1;
#endif
  for (int index = 0; index < NUM_GEAR_AXES; ++index) {
    GearAxis& axis = axes[index];
    s_prefs.begin("gear", true);
    char key[8]; snprintf(key, sizeof(key), "gr%d", index);
    float ratio = s_prefs.getFloat(key, GEAR_RATIO_DEFAULT); s_prefs.end();
    if (!std::isfinite(ratio) || ratio < 1.0f || ratio > 10000.0f)
      ratio = GEAR_RATIO_DEFAULT;
    axis.gear_ratio.store(ratio); axis.kp.store(GEAR_DEFAULT_KP);
    axis.ki.store(GEAR_DEFAULT_KI); axis.kd.store(GEAR_DEFAULT_KD);
    axis.duty_cap_pct.store((float)GEAR_PWM_DUTY_CAP_PCT);
    axis.reset_pid.store(false); axis.home_requested.store(false);
    axis.integrator = 0.0f; axis.prev_error = 0.0f;
    axis.position_counts = 0; axis.published_counts = 0;
    axis.target_deg.store(0.0f); axis.current_deg.store(0.0f);
    axis.enable_req.store(false); axis.fault_latched.store(false);
    axis.state.store(FOC_STATE_DISABLED);
    serial_tx_printf("[GEAR %d] ratio=%.1f counts/deg=%.2f", index,
                     (double)ratio, (double)counts_per_degree(axis));
    init_pcnt(axis.pcnt_unit, axis.pin_enca, axis.pin_encb);
    ledcSetup(axis.ledc_ch_in1, GEAR_PWM_FREQ_HZ, GEAR_PWM_RES_BITS);
    ledcSetup(axis.ledc_ch_in2, GEAR_PWM_FREQ_HZ, GEAR_PWM_RES_BITS);
    ledcAttachPin(axis.pin_in1, axis.ledc_ch_in1);
    ledcAttachPin(axis.pin_in2, axis.ledc_ch_in2); set_pwm(axis, 0.0f);
    char name[16]; snprintf(name, sizeof(name), "gear%d", index);
    xTaskCreatePinnedToCore(gear_task, name, GEAR_TASK_STACK, (void*)(intptr_t)index,
                            GEAR_TASK_PRIORITY, &s_task[index], GEAR_TASK_CORE);
  }
}

bool foc_set_target_deg(int axis, float value) {
  if (!valid_axis(axis) || !std::isfinite(value) || value < -GEAR_MAX_ANGLE_ABS || value > GEAR_MAX_ANGLE_ABS) return false;
  axes[axis].target_deg.store(value); return true;
}
bool foc_request_enable(int axis, bool enabled) {
  if (!valid_axis(axis)) return false;
  if (enabled && (axes[axis].fault_latched.load() || axes[axis].state.load() == FOC_STATE_FAULT)) return false;
  axes[axis].enable_req.store(enabled); return true;
}
bool foc_clear_fault(int axis) {
  if (!valid_axis(axis) || axes[axis].state.load() != FOC_STATE_FAULT) return false;
  axes[axis].enable_req.store(false); axes[axis].fault_latched.store(false);
  axes[axis].state.store(FOC_STATE_DISABLED); axes[axis].reset_pid.store(true); return true;
}
void foc_latch_fault(int axis) {
  if (!valid_axis(axis)) return;
  axes[axis].fault_latched.store(true); axes[axis].enable_req.store(false);
  axes[axis].state.store(FOC_STATE_FAULT);
}
bool foc_set_voltage_limit(int axis, float value) {
  if (!valid_axis(axis) || !std::isfinite(value) || value <= 0.0f || value > 100.0f) return false;
  axes[axis].duty_cap_pct.store(value); return true;
}
void foc_home(int axis) {
  if (!valid_axis(axis)) return;
  axes[axis].target_deg.store(0.0f); axes[axis].home_requested.store(true);
}
bool foc_set_p_angle(int axis, float value) {
  if (!valid_axis(axis) || !std::isfinite(value) || value < 0.0f || value > 1000.0f) return false;
  axes[axis].kp.store(value); axes[axis].reset_pid.store(true); return true;
}
FocState foc_get_state(int axis) { return valid_axis(axis) ? (FocState)axes[axis].state.load() : FOC_STATE_DISABLED; }
float foc_get_current_deg(int axis) { return valid_axis(axis) ? axes[axis].current_deg.load() : 0.0f; }
float foc_get_target_deg(int axis) { return valid_axis(axis) ? axes[axis].target_deg.load() : 0.0f; }
bool foc_is_fault_latched(int axis) { return valid_axis(axis) && axes[axis].fault_latched.load(); }
bool foc_set_gear_ratio_and_store(int axis, float value) {
  if (!valid_axis(axis) || !std::isfinite(value) || value < 1.0f || value > 10000.0f) return false;
  axes[axis].gear_ratio.store(value); axes[axis].reset_pid.store(true); s_prefs.begin("gear", false);
  char key[8]; snprintf(key, sizeof(key), "gr%d", axis); s_prefs.putFloat(key, value); s_prefs.end(); return true;
}
bool foc_set_p_integral(int axis, float value) {
  if (!valid_axis(axis) || !std::isfinite(value) || value < 0.0f || value > 1000.0f) return false;
  axes[axis].ki.store(value); axes[axis].reset_pid.store(true); return true;
}
bool foc_set_p_derivative(int axis, float value) {
  if (!valid_axis(axis) || !std::isfinite(value) || value < 0.0f || value > 100.0f) return false;
  axes[axis].kd.store(value); axes[axis].reset_pid.store(true); return true;
}

void foc_run_diagnostics(int axis) {
  if (!valid_axis(axis)) { serial_tx_line("ERR:bad axis"); return; }
  GearAxis& item = axes[axis];
  serial_tx_printf("[DIAG %d]", axis);
  serial_tx_printf("  pins IN1/IN2/ENC_A/ENC_B=%d/%d/%d/%d",
                   item.pin_in1, item.pin_in2, item.pin_enca, item.pin_encb);
  serial_tx_printf("  raw A/B=%d/%d", digitalRead(item.pin_enca),
                   digitalRead(item.pin_encb));
  serial_tx_printf("  ratio=%.1f Kp/Ki/Kd=%.3f/%.3f/%.3f",
                   (double)item.gear_ratio.load(), (double)item.kp.load(),
                   (double)item.ki.load(), (double)item.kd.load());
  serial_tx_printf("  duty_cap=%.1f%%", (double)item.duty_cap_pct.load());
  serial_tx_printf("  encoder_total_int64=%lld",
                   (long long)get_published_counts(item));
  serial_tx_printf("  current/target=%.2f/%.2f",
                   (double)item.current_deg.load(),
                   (double)item.target_deg.load());
  serial_tx_printf("  state/fault=%d/%d", (int)item.state.load(),
                   item.fault_latched.load() ? 1 : 0);
  serial_tx_printf("  PWM IN1/IN2=%lu/%lu",
                   (unsigned long)ledcRead(item.ledc_ch_in1),
                   (unsigned long)ledcRead(item.ledc_ch_in2));
}
void foc_run_diagnostics_live(int axis) {
  if (!valid_axis(axis)) { serial_tx_line("ERR:bad axis"); return; }
  GearAxis& item = axes[axis];
  serial_tx_line("[LIVE] 5 seconds at 10 Hz; turn the encoder shaft now");
  const uint32_t end_ms = millis() + 5000;
  while ((int32_t)(millis() - end_ms) < 0) {
    serial_tx_printf("A=%d B=%d total=%lld", digitalRead(item.pin_enca),
                     digitalRead(item.pin_encb),
                     (long long)get_published_counts(item));
    delay(100);
  }
  serial_tx_line("[LIVE] done");
}
