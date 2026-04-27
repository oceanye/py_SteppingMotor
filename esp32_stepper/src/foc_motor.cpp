#include "foc_motor.h"
#include "config.h"
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Preferences.h>
#include <atomic>

// ── 每轴 SimpleFOC 对象（文件级静态数组）──
// 注意：sensors 必须在 foc_init 里用 init(&Wire) / init(&Wire1) 分别初始化不同总线。
static MagneticSensorI2C sensors[NUM_AXES] = {
  MagneticSensorI2C(AS5600_I2C),
  MagneticSensorI2C(AS5600_I2C),
};
static BLDCMotor motors[NUM_AXES] = {
  BLDCMotor(FOC_POLE_PAIRS_DEFAULT),
  BLDCMotor(FOC_POLE_PAIRS_DEFAULT),
};
static BLDCDriver3PWM drivers[NUM_AXES] = {
  BLDCDriver3PWM(PIN_FOC_M1_0, PIN_FOC_M2_0, PIN_FOC_M3_0, PIN_FOC_EN_0),
  BLDCDriver3PWM(PIN_FOC_M1_1, PIN_FOC_M2_1, PIN_FOC_M3_1, PIN_FOC_EN_1),
};

// 每轴 nFAULT 引脚
static const int PIN_NFAULT[NUM_AXES] = { PIN_FOC_NFAULT_0, PIN_FOC_NFAULT_1 };

// 每轴任务 + home offset + 对齐标记
static TaskHandle_t s_task[NUM_AXES]            = {nullptr, nullptr};
static float        s_home_offset_rad[NUM_AXES] = {0.0f, 0.0f};
static bool         s_aligned_once[NUM_AXES]    = {false, false};
static Preferences  s_prefs;

// 每轴 atomic 镜像变量
static std::atomic<float>   g_target_deg[NUM_AXES];
static std::atomic<float>   g_current_deg[NUM_AXES];
static std::atomic<bool>    g_enable_req[NUM_AXES];
static std::atomic<bool>    g_fault_latched[NUM_AXES];
static std::atomic<float>   g_voltage_limit[NUM_AXES];
static std::atomic<uint8_t> g_state[NUM_AXES];

static bool valid_axis(int axis) { return axis >= 0 && axis < NUM_AXES; }

// Core 0 FOC 任务（每轴一个，通过 arg 传 axis 索引）
static void foc_task(void* arg) {
  int axis = (int)(intptr_t)arg;
  MagneticSensorI2C& sensor = sensors[axis];
  BLDCMotor&         motor  = motors[axis];

  for (;;) {
    motor.target        = g_target_deg[axis].load() * PI / 180.0f + s_home_offset_rad[axis];
    motor.voltage_limit = g_voltage_limit[axis].load();

    FocState state = (FocState)g_state[axis].load();
    bool want_en   = g_enable_req[axis].load();

    if (state == FOC_STATE_DISABLED && want_en) {
      g_state[axis].store(FOC_STATE_ALIGNING);
      state = FOC_STATE_ALIGNING;
    }
    if (state == FOC_STATE_ALIGNING) {
      if (!s_aligned_once[axis]) {
        int ok = motor.initFOC();
        if (!ok) {
          Serial.print("FOC,"); Serial.print(axis); Serial.println(",FAULT");
          g_fault_latched[axis].store(true);
          g_state[axis].store(FOC_STATE_FAULT);
          g_enable_req[axis].store(false);
          state = FOC_STATE_FAULT;
          vTaskDelay(1 / portTICK_PERIOD_MS);
          continue;
        }
        s_aligned_once[axis] = true;
      }
      sensor.update();
      float cur_user_rad = sensor.getAngle() - s_home_offset_rad[axis];
      g_target_deg[axis].store(cur_user_rad * 180.0f / PI);
      motor.target = sensor.getAngle();
      motor.enable();
      g_state[axis].store(FOC_STATE_RUNNING);
      state = FOC_STATE_RUNNING;
    }
    if ((state == FOC_STATE_RUNNING || state == FOC_STATE_ALIGNING) && !want_en) {
      motor.disable();
      g_state[axis].store(FOC_STATE_DISABLED);
      state = FOC_STATE_DISABLED;
    }
    if (state == FOC_STATE_FAULT) {
      motor.disable();
    }

    sensor.update();
    if (state == FOC_STATE_RUNNING && !g_fault_latched[axis].load()) {
      motor.loopFOC();
      motor.move();
    }

    float shaft = sensor.getAngle() - s_home_offset_rad[axis];
    g_current_deg[axis].store(shaft * 180.0f / PI);

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void foc_init() {
  // 两条 I2C 总线
  Wire.begin(PIN_I2C_SDA_0, PIN_I2C_SCL_0);
  Wire1.begin(PIN_I2C_SDA_1, PIN_I2C_SCL_1);

  for (int axis = 0; axis < NUM_AXES; axis++) {
    // 镜像变量初值
    g_target_deg[axis].store(0.0f);
    g_current_deg[axis].store(0.0f);
    g_enable_req[axis].store(false);
    g_fault_latched[axis].store(false);
    g_voltage_limit[axis].store(FOC_INITIAL_V_LIMIT);
    g_state[axis].store(FOC_STATE_DISABLED);

    // NVS 读极对数（按轴 key "pp0"/"pp1"）
    s_prefs.begin("foc", /*readOnly=*/true);
    char key[8]; snprintf(key, sizeof(key), "pp%d", axis);
    int pp = s_prefs.getInt(key, FOC_POLE_PAIRS_DEFAULT);
    s_prefs.end();
    motors[axis].pole_pairs = pp;
    Serial.print("[FOC "); Serial.print(axis); Serial.print("] pole_pairs = "); Serial.println(pp);

    // 传感器到对应总线
    sensors[axis].init(axis == 0 ? &Wire : &Wire1);
    Serial.print("[FOC "); Serial.print(axis); Serial.println("] sensor init done");

    // 驱动
    drivers[axis].voltage_power_supply = FOC_PSU_VOLTAGE;
    drivers[axis].voltage_limit        = FOC_PSU_VOLTAGE;
    drivers[axis].init();

    // 电机
    motors[axis].linkSensor(&sensors[axis]);
    motors[axis].linkDriver(&drivers[axis]);
    motors[axis].useMonitoring(Serial);
    motors[axis].voltage_limit         = FOC_INITIAL_V_LIMIT;
    motors[axis].voltage_sensor_align  = 6.0f;
    motors[axis].velocity_limit        = FOC_VELOCITY_LIMIT;
    motors[axis].controller            = MotionControlType::angle;
    motors[axis].P_angle.P             = 25.0f;
    motors[axis].PID_velocity.P        = 0.2f;
    motors[axis].PID_velocity.I        = 2.0f;
    motors[axis].PID_velocity.D        = 0.0f;
    motors[axis].PID_velocity.output_ramp = 1000.0f;
    motors[axis].LPF_velocity.Tf       = 0.02f;
    motors[axis].init();

    pinMode(PIN_NFAULT[axis], INPUT_PULLUP);

    // 任务（每轴一个）都在 Core 0
    char tname[16]; snprintf(tname, sizeof(tname), "foc%d", axis);
    xTaskCreatePinnedToCore(foc_task, tname, FOC_TASK_STACK,
                            (void*)(intptr_t)axis, FOC_TASK_PRIORITY,
                            &s_task[axis], FOC_TASK_CORE);
    Serial.print("[FOC "); Serial.print(axis); Serial.println("] Core 0 task started");
  }
}

// ── Core 1 API ──
bool foc_set_target_deg(int axis, float deg) {
  if (!valid_axis(axis)) return false;
  if (deg > FOC_MAX_ANGLE_ABS || deg < -FOC_MAX_ANGLE_ABS) return false;
  g_target_deg[axis].store(deg);
  return true;
}

bool foc_request_enable(int axis, bool en) {
  if (!valid_axis(axis)) return false;
  if (en && g_state[axis].load() == FOC_STATE_FAULT) return false;
  g_enable_req[axis].store(en);
  return true;
}

bool foc_clear_fault(int axis) {
  if (!valid_axis(axis)) return false;
  if (g_state[axis].load() != FOC_STATE_FAULT) return false;
  g_fault_latched[axis].store(false);
  g_state[axis].store(FOC_STATE_DISABLED);
  s_aligned_once[axis] = false;
  return true;
}

void foc_latch_fault(int axis) {
  if (!valid_axis(axis)) return;
  g_fault_latched[axis].store(true);
  g_state[axis].store(FOC_STATE_FAULT);
  g_enable_req[axis].store(false);
}

bool foc_set_voltage_limit(int axis, float v) {
  if (!valid_axis(axis)) return false;
  if (v <= 0.0f || v > FOC_MAX_V_LIMIT) return false;
  g_voltage_limit[axis].store(v);
  return true;
}

void foc_home(int axis) {
  if (!valid_axis(axis)) return;
  sensors[axis].update();
  s_home_offset_rad[axis] = sensors[axis].getAngle();
  g_target_deg[axis].store(0.0f);
}

bool foc_set_pole_pairs_and_store(int axis, int n) {
  if (!valid_axis(axis)) return false;
  if (n < 1 || n > 50) return false;
  s_prefs.begin("foc", /*readOnly=*/false);
  char key[8]; snprintf(key, sizeof(key), "pp%d", axis);
  s_prefs.putInt(key, n);
  s_prefs.end();
  return true;
}

bool foc_set_p_angle(int axis, float p) {
  if (!valid_axis(axis)) return false;
  if (p < 0.1f || p > 50.0f) return false;
  motors[axis].P_angle.P = p;
  return true;
}

bool foc_set_p_velocity(int axis, float p) {
  if (!valid_axis(axis)) return false;
  if (p < 0.01f || p > 2.0f) return false;
  motors[axis].PID_velocity.P = p;
  return true;
}

bool foc_force_realign(int axis) {
  if (!valid_axis(axis)) return false;
  // 必须先 disable 才能安全修改对齐状态
  motors[axis].disable();
  s_aligned_once[axis] = false;
  g_state[axis].store(FOC_STATE_DISABLED);
  g_enable_req[axis].store(false);
  return true;
}

FocState foc_get_state(int axis) {
  if (!valid_axis(axis)) return FOC_STATE_DISABLED;
  return (FocState)g_state[axis].load();
}
float foc_get_current_deg(int axis) {
  if (!valid_axis(axis)) return 0.0f;
  return g_current_deg[axis].load();
}
float foc_get_target_deg(int axis) {
  if (!valid_axis(axis)) return 0.0f;
  return g_target_deg[axis].load();
}
bool foc_is_fault_latched(int axis) {
  if (!valid_axis(axis)) return false;
  return g_fault_latched[axis].load();
}
