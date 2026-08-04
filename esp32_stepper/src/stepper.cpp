#include <Arduino.h>
#include "config.h"
#include "stepper.h"
#include <atomic>
#include <math.h>

// 每轴引脚（编译期常量数组）
#if defined(DRIVE_MODE_FOC)
static const int PIN_PUL[NUM_AXES] = {PIN_STEP_PUL_0, PIN_STEP_PUL_1};
static const int PIN_DIR[NUM_AXES] = {PIN_STEP_DIR_0, PIN_STEP_DIR_1};
#elif defined(DRIVE_MODE_GEAR)
static const int PIN_PUL[NUM_AXES] = {
  PIN_STEP_PUL_0, PIN_STEP_PUL_1, PIN_STEP_PUL_2,
  PIN_STEP_PUL_3, PIN_STEP_PUL_4, PIN_STEP_PUL_5
};
static const int PIN_DIR[NUM_AXES] = {
  PIN_STEP_DIR_0, PIN_STEP_DIR_1, PIN_STEP_DIR_2,
  PIN_STEP_DIR_3, PIN_STEP_DIR_4, PIN_STEP_DIR_5
};
#endif

// 每轴异步任务状态
static TaskHandle_t      s_task[NUM_AXES]      = {};
static SemaphoreHandle_t s_start_sem[NUM_AXES] = {};
enum StepState : uint8_t {
  STEP_IDLE = 0,
  STEP_RUNNING = 1,
  STEP_ABORTING = 2,
  STEP_FINISHING = 3,
};
static std::atomic<uint8_t> s_state[NUM_AXES];
static int               s_steps[NUM_AXES], s_dir[NUM_AXES], s_delay_us[NUM_AXES];

static bool valid_axis(int axis) { return axis >= 0 && axis < NUM_AXES; }

// ── 加减速曲线参数 ──
#define STEP_RAMP_START_US    5000   // 启动延时(us)，安全慢速 ~60 RPM @200微步
#define STEP_RAMP_FACTOR      0.02f  // 每步延时缩减比例（越大加速越猛）

// 单脉冲输出：50us HIGH + 指定 LOW 延时，含周期性 yield 防饿死其他任务
static inline void emit_pulse(int pul, int delay_us, int* step_count, uint32_t* last_yield_ms,
                              int axis, int total_steps) {
  digitalWrite(pul, HIGH);
  delayMicroseconds(50);
  digitalWrite(pul, LOW);
  if (delay_us >= 2000) {
    vTaskDelay(delay_us / 1000 / portTICK_PERIOD_MS);
    int rem = delay_us % 1000;
    if (rem > 0) delayMicroseconds(rem);
  } else if (delay_us > 0) {
    delayMicroseconds(delay_us);
    if (millis() - *last_yield_ms >= 10) {
      vTaskDelay(1);
      *last_yield_ms = millis();
    }
  }
  (*step_count)++;
  if (total_steps > 500 && (*step_count % 500) == 0) {
    Serial.print("STEP,");
    Serial.print(axis);
    Serial.print(",P,");
    Serial.print(*step_count);
    Serial.print(",");
    Serial.println(total_steps);
  }
}

static void stepper_task(void* arg) {
  int axis = (int)(intptr_t)arg;
  for (;;) {
    xSemaphoreTake(s_start_sem[axis], portMAX_DELAY);
    int steps = s_steps[axis];
    int dir   = s_dir[axis];
    int dus   = s_delay_us[axis];
    int pul   = PIN_PUL[axis];
    int dpin  = PIN_DIR[axis];

    digitalWrite(dpin, dir);
    delayMicroseconds(100);
    // ── 梯形加减速曲线 ──
    float cruise_delay = (float)dus;
    float start_delay  = (float)STEP_RAMP_START_US;

    // 计算加速步数：从 start_delay 按比例缩减到 cruise_delay 需要多少步
    int ramp_steps = 0;
    if (cruise_delay < start_delay) {
      float ratio = cruise_delay / start_delay;
      ramp_steps = (int)(logf(ratio) / logf(1.0f - STEP_RAMP_FACTOR));
      if (ramp_steps < 1) ramp_steps = 1;
    }

    // 确定曲线类型：无加减速 / 三角形(短行程) / 梯形
    int accel_steps, cruise_steps, decel_steps;
    if (ramp_steps == 0) {
      accel_steps = 0; decel_steps = 0; cruise_steps = steps;
    } else if (2 * ramp_steps >= steps) {
      accel_steps  = steps / 2;
      decel_steps  = steps - accel_steps;
      cruise_steps = 0;
    } else {
      accel_steps  = ramp_steps;
      decel_steps  = ramp_steps;
      cruise_steps = steps - 2 * ramp_steps;
    }

    float current_delay = start_delay;
    int step_count = 0;
    uint32_t last_yield_ms = millis();
    bool aborted = s_state[axis].load() == STEP_ABORTING;

    // 加速阶段：逐步缩短延时
    for (int i = 0; i < accel_steps && !aborted; i++) {
      emit_pulse(pul, (int)current_delay, &step_count, &last_yield_ms, axis, steps);
      current_delay *= (1.0f - STEP_RAMP_FACTOR);
      if (current_delay < cruise_delay) current_delay = cruise_delay;
      if (s_state[axis].load() == STEP_ABORTING) aborted = true;
    }
    // 巡航阶段：恒速
    for (int i = 0; i < cruise_steps && !aborted; i++) {
      emit_pulse(pul, (int)cruise_delay, &step_count, &last_yield_ms, axis, steps);
      if (s_state[axis].load() == STEP_ABORTING) aborted = true;
    }
    // 减速阶段：逐步加长延时（镜像加速）
    for (int i = 0; i < decel_steps && !aborted; i++) {
      current_delay /= (1.0f - STEP_RAMP_FACTOR);
      if (current_delay > start_delay) current_delay = start_delay;
      emit_pulse(pul, (int)current_delay, &step_count, &last_yield_ms, axis, steps);
      if (s_state[axis].load() == STEP_ABORTING) aborted = true;
    }

    // Claim the terminal transition before printing. A concurrent STOP either
    // wins RUNNING->ABORTING, or observes FINISHING after all pulses are done.
    uint8_t expected = STEP_RUNNING;
    if (!s_state[axis].compare_exchange_strong(expected, STEP_FINISHING)) {
      if (expected == STEP_ABORTING) aborted = true;
      s_state[axis].store(STEP_FINISHING);
    }
    Serial.print("STEP,");
    Serial.print(axis);
    Serial.print(aborted ? ",ABORT," : ",DONE,");
    Serial.print(step_count);
    Serial.print(',');
    Serial.println(steps);
    // A new MOVE cannot be accepted until the previous terminal line is sent.
    s_state[axis].store(STEP_IDLE);
  }
}

void stepper_init() {
  for (int axis = 0; axis < NUM_AXES; axis++) {
    pinMode(PIN_PUL[axis], OUTPUT);
    pinMode(PIN_DIR[axis], OUTPUT);
    digitalWrite(PIN_PUL[axis], LOW);
    digitalWrite(PIN_DIR[axis], LOW);
    s_start_sem[axis] = xSemaphoreCreateBinary();
    s_state[axis].store(STEP_IDLE);
    char name[16]; snprintf(name, sizeof(name), "stepper%d", axis);
    xTaskCreatePinnedToCore(stepper_task, name, 4096,
                            (void*)(intptr_t)axis, 2, &s_task[axis], 1);
  }
}

void stepper_move(int axis, int steps, int direction, int delay_us) {
  if (!valid_axis(axis)) { Serial.print("ERR:bad axis "); Serial.println(axis); return; }
  int pul = PIN_PUL[axis], dpin = PIN_DIR[axis];
  digitalWrite(dpin, direction);
  delayMicroseconds(100);
  for (int i = 0; i < steps; i++) {
    digitalWrite(pul, HIGH);
    delayMicroseconds(50);
    digitalWrite(pul, LOW);
    if (delay_us >= 2000) delay(delay_us / 1000);
    else if (delay_us > 0) delayMicroseconds(delay_us);
  }
  Serial.print("OK,"); Serial.println(axis);
}

bool stepper_is_busy(int axis) {
  if (!valid_axis(axis)) return false;
  return s_state[axis].load() != STEP_IDLE;
}

bool stepper_abort(int axis) {
  if (!valid_axis(axis)) return false;
  uint8_t expected = STEP_RUNNING;
  if (s_state[axis].compare_exchange_strong(expected, STEP_ABORTING)) return true;
  return expected == STEP_ABORTING;
}

bool stepper_move_async(int axis, int steps, int direction, int delay_us) {
  if (!valid_axis(axis) || steps <= 0 || (direction != 0 && direction != 1) ||
      delay_us <= 0) return false;
  uint8_t expected = STEP_IDLE;
  if (!s_state[axis].compare_exchange_strong(expected, STEP_RUNNING)) return false;
  s_steps[axis]    = steps;
  s_dir[axis]      = direction;
  s_delay_us[axis] = delay_us;
  xSemaphoreGive(s_start_sem[axis]);
  return true;
}

void stepper_run_diagnostics(int axis) {
  if (!valid_axis(axis)) { Serial.println("ERR:bad axis"); return; }
  int pul = PIN_PUL[axis], dpin = PIN_DIR[axis];
  Serial.println();
  Serial.print("===== DIAG axis="); Serial.print(axis); Serial.println(" =====");
  Serial.print(">>> PUL(GPIO"); Serial.print(pul); Serial.println(") 将置 HIGH 5 秒");
  delay(1000);
  digitalWrite(pul, HIGH);
  delay(5000);
  digitalWrite(pul, LOW);
  delay(500);
  Serial.print(">>> DIR(GPIO"); Serial.print(dpin); Serial.println(") 将置 HIGH 3 秒");
  delay(500);
  digitalWrite(dpin, HIGH);
  delay(3000);
  digitalWrite(dpin, LOW);
  Serial.println(">>> 5 + 5 脉冲测试");
  digitalWrite(dpin, HIGH);
  delayMicroseconds(100);
  for (int i = 0; i < 5; i++) {
    digitalWrite(pul, HIGH); delay(20);
    digitalWrite(pul, LOW);  delay(80);
  }
  delay(500);
  digitalWrite(dpin, LOW);
  delayMicroseconds(100);
  for (int i = 0; i < 5; i++) {
    digitalWrite(pul, HIGH); delay(20);
    digitalWrite(pul, LOW);  delay(80);
  }
  Serial.println("===== DIAG 结束 =====");
}
