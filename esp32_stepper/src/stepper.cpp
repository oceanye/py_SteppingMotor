#include <Arduino.h>
#include "config.h"
#include "stepper.h"
#include "serial_tx.h"
#include <atomic>
#include <math.h>
#include <esp_timer.h>
#include "sync_math.h"

// 每轴引脚（编译期常量数组）
#if defined(DRIVE_MODE_FOC)
static const int PIN_PUL[NUM_AXES] = {PIN_STEP_PUL_0, PIN_STEP_PUL_1};
static const int PIN_DIR[NUM_AXES] = {PIN_STEP_DIR_0, PIN_STEP_DIR_1};
static const int PIN_ENA[NUM_AXES] = {-1, -1};  // FOC 板未接 ENA 线
#elif defined(DRIVE_MODE_GEAR)
static const int PIN_PUL[NUM_AXES] = {
  PIN_STEP_PUL_0, PIN_STEP_PUL_1, PIN_STEP_PUL_2,
  PIN_STEP_PUL_3, PIN_STEP_PUL_4, PIN_STEP_PUL_5
};
static const int PIN_DIR[NUM_AXES] = {
  PIN_STEP_DIR_0, PIN_STEP_DIR_1, PIN_STEP_DIR_2,
  PIN_STEP_DIR_3, PIN_STEP_DIR_4, PIN_STEP_DIR_5
};
static const int PIN_ENA[NUM_AXES] = {
  PIN_STEP_ENA_0, PIN_STEP_ENA_1, PIN_STEP_ENA_2,
  PIN_STEP_ENA_3, PIN_STEP_ENA_4, PIN_STEP_ENA_5
};
#endif

// ── ENA(使能/释放)控制：仅接了 ENA 线的轴(引脚 >= 0)生效 ──
bool stepper_ena_wire_present(int axis) {
  return axis >= 0 && axis < NUM_AXES && PIN_ENA[axis] >= 0;
}

bool stepper_ena_locked(int axis) {
  if (!stepper_ena_wire_present(axis)) return true;  // 未接线=驱动器恒使能
  return digitalRead(PIN_ENA[axis]) == STEPPER_ENA_LOCKED_LEVEL;
}

bool stepper_set_ena(int axis, bool locked) {
  if (!stepper_ena_wire_present(axis)) return false;
  if (!locked && stepper_is_busy(axis)) return false;  // 运动中禁止释放
  digitalWrite(PIN_ENA[axis],
               locked ? STEPPER_ENA_LOCKED_LEVEL : STEPPER_ENA_RELEASED_LEVEL);
  return true;
}

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
static SemaphoreHandle_t s_sync_sem = nullptr;
static std::atomic<int> s_sync_a{-1}, s_sync_b{-1};
static int s_sync_steps[2], s_sync_dirs[2], s_sync_duration;
static std::atomic<bool> s_sync_active{false};

static bool valid_axis(int axis) { return axis >= 0 && axis < NUM_AXES; }

static bool sync_aborting(int a, int b) {
  return s_state[a].load() == STEP_ABORTING || s_state[b].load() == STEP_ABORTING;
}

static void sync_task(void*) {
  for (;;) {
    xSemaphoreTake(s_sync_sem, portMAX_DELAY);
    const int a = s_sync_a.load(), b = s_sync_b.load();
    const int axes[2] = {a, b};
    const int master = max(s_sync_steps[0], s_sync_steps[1]);
    int done[2] = {0, 0};
    for (int j = 0; j < 2; ++j) digitalWrite(PIN_DIR[axes[j]], s_sync_dirs[j]);
    delayMicroseconds(100);
    const int64_t start = esp_timer_get_time();
    uint32_t last_progress = millis();
    uint32_t last_yield = last_progress;
    int64_t last_pulse = start - 200;
    bool aborted = false;
    for (int tick = 1; tick <= master; ++tick) {
      // Both axes follow this ONE quintic master path; separate motor ramps
      // would distort q_swing/q_support even if their end times matched.
      int64_t due = start + (int64_t)(s_sync_duration *
                          sync_inverse_progress((double)tick / master));
      due = max(due, last_pulse + 200);  // never burst to catch up after scheduling delay
      while (esp_timer_get_time() < due && !sync_aborting(a, b)) {
        int64_t remaining = due - esp_timer_get_time();
        if (remaining >= 2000) vTaskDelay(1);
        else if (remaining > 0) delayMicroseconds((uint32_t)min(remaining, (int64_t)100));
      }
      if (sync_aborting(a, b)) { aborted = true; break; }
      bool fire[2];
      for (int j = 0; j < 2; ++j) {
        fire[j] = sync_step_target(tick, s_sync_steps[j], master) > done[j];
        if (fire[j]) digitalWrite(PIN_PUL[axes[j]], HIGH);
      }
      last_pulse = esp_timer_get_time();
      delayMicroseconds(50);
      for (int j = 0; j < 2; ++j) if (fire[j]) {
        digitalWrite(PIN_PUL[axes[j]], LOW);
        ++done[j];
      }
      if ((uint32_t)(millis() - last_progress) >= 250 && tick < master) {
        for (int j = 0; j < 2; ++j)
          serial_tx_printf("STEP,%d,P,%d,%d", axes[j], done[j], s_sync_steps[j]);
        last_progress = millis();
      }
      if ((uint32_t)(millis() - last_yield) >= 10) {
        vTaskDelay(1);
        last_yield = millis();
      }
    }
    // Claim both terminals; a STOP winning either transition aborts the pair.
    for (int j = 0; j < 2; ++j) {
      uint8_t expected = STEP_RUNNING;
      if (!s_state[axes[j]].compare_exchange_strong(expected, STEP_FINISHING)) {
        aborted = true;
        s_state[axes[j]].store(STEP_FINISHING);
      }
    }
    for (int j = 0; j < 2; ++j)
      serial_tx_printf("STEP,%d,%s,%d,%d", axes[j], aborted ? "ABORT" : "DONE",
                       done[j], s_sync_steps[j]);
    // Clear the pair BEFORE making either axis reusable.
    s_sync_active.store(false);
    for (int j = 0; j < 2; ++j) s_state[axes[j]].store(STEP_IDLE);
  }
}

bool stepper_sync_async(int a, int signed_a, int b, int signed_b, int duration_us) {
  if (!valid_axis(a) || !valid_axis(b) || a == b || !s_sync_sem || s_sync_active.load()) return false;
  if (signed_a < -20000000 || signed_a > 20000000 || signed_b < -20000000 || signed_b > 20000000) return false;
  const int na = abs(signed_a), nb = abs(signed_b);
  if (!sync_timing_valid(max(na, nb), duration_us)
      || !stepper_ena_locked(a) || !stepper_ena_locked(b)) return false;
  uint8_t expected = STEP_IDLE;
  if (!s_state[a].compare_exchange_strong(expected, STEP_RUNNING)) return false;
  expected = STEP_IDLE;
  if (!s_state[b].compare_exchange_strong(expected, STEP_RUNNING)) {
    s_state[a].store(STEP_IDLE);
    return false;
  }
  s_sync_a.store(a); s_sync_b.store(b);
  s_sync_steps[0] = na; s_sync_steps[1] = nb;
  s_sync_dirs[0] = signed_a >= 0 ? HIGH : LOW;
  s_sync_dirs[1] = signed_b >= 0 ? HIGH : LOW;
  s_sync_duration = duration_us;
  s_sync_active.store(true);
  // ACK before waking the pulse task. Both STEP terminals retain existing format.
  serial_tx_printf("OK,SYNC,%d,%d", a, b);
  xSemaphoreGive(s_sync_sem);
  return true;
}

// ── 加减速曲线参数 ──
#define STEP_RAMP_START_US    5000   // 启动延时(us)，安全慢速 ~60 RPM @200微步
#define STEP_RAMP_FACTOR      0.02f  // 每步延时缩减比例（越大加速越猛）
#define STEP_PROGRESS_INTERVAL_MS 250  // 与 Pico 节点一致；限制多轴串口进度流量

// 单脉冲输出：50us HIGH + 指定 LOW 延时，含周期性 yield 防饿死其他任务
static inline void emit_pulse(int pul, int delay_us, int* step_count, uint32_t* last_yield_ms,
                              uint32_t* last_progress_ms, int axis, int total_steps) {
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
  uint32_t now_ms = millis();
  if (*step_count < total_steps &&
      (uint32_t)(now_ms - *last_progress_ms) >= STEP_PROGRESS_INTERVAL_MS) {
    serial_tx_printf("STEP,%d,P,%d,%d", axis, *step_count, total_steps);
    *last_progress_ms = now_ms;
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
    uint32_t last_progress_ms = last_yield_ms;
    bool aborted = s_state[axis].load() == STEP_ABORTING;

    // 加速阶段：逐步缩短延时
    for (int i = 0; i < accel_steps && !aborted; i++) {
      emit_pulse(pul, (int)current_delay, &step_count, &last_yield_ms,
                 &last_progress_ms, axis, steps);
      current_delay *= (1.0f - STEP_RAMP_FACTOR);
      if (current_delay < cruise_delay) current_delay = cruise_delay;
      if (s_state[axis].load() == STEP_ABORTING) aborted = true;
    }
    // 巡航阶段：恒速
    for (int i = 0; i < cruise_steps && !aborted; i++) {
      emit_pulse(pul, (int)cruise_delay, &step_count, &last_yield_ms,
                 &last_progress_ms, axis, steps);
      if (s_state[axis].load() == STEP_ABORTING) aborted = true;
    }
    // 减速阶段：逐步加长延时（镜像加速）
    for (int i = 0; i < decel_steps && !aborted; i++) {
      current_delay /= (1.0f - STEP_RAMP_FACTOR);
      if (current_delay > start_delay) current_delay = start_delay;
      emit_pulse(pul, (int)current_delay, &step_count, &last_yield_ms,
                 &last_progress_ms, axis, steps);
      if (s_state[axis].load() == STEP_ABORTING) aborted = true;
    }

    // Claim the terminal transition before printing. A concurrent STOP either
    // wins RUNNING->ABORTING, or observes FINISHING after all pulses are done.
    uint8_t expected = STEP_RUNNING;
    if (!s_state[axis].compare_exchange_strong(expected, STEP_FINISHING)) {
      if (expected == STEP_ABORTING) aborted = true;
      s_state[axis].store(STEP_FINISHING);
    }
    serial_tx_printf("STEP,%d,%s,%d,%d", axis,
                     aborted ? "ABORT" : "DONE", step_count, steps);
    // A new MOVE cannot be accepted until the previous terminal line is sent.
    s_state[axis].store(STEP_IDLE);
  }
}

void stepper_init() {
  s_sync_sem = xSemaphoreCreateBinary();
  if (s_sync_sem && xTaskCreatePinnedToCore(sync_task, "step_sync", 4096, nullptr, 2, nullptr, 1) != pdPASS) {
    vSemaphoreDelete(s_sync_sem);
    s_sync_sem = nullptr;
  }
  for (int axis = 0; axis < NUM_AXES; axis++) {
    pinMode(PIN_PUL[axis], OUTPUT);
    pinMode(PIN_DIR[axis], OUTPUT);
    digitalWrite(PIN_PUL[axis], LOW);
    digitalWrite(PIN_DIR[axis], LOW);
    if (PIN_ENA[axis] >= 0) {
      // 上电默认锁定(保持力矩在)；绝不带着释放态启动。
      pinMode(PIN_ENA[axis], OUTPUT);
      digitalWrite(PIN_ENA[axis], STEPPER_ENA_LOCKED_LEVEL);
    }
    s_start_sem[axis] = xSemaphoreCreateBinary();
    s_state[axis].store(STEP_IDLE);
    char name[16]; snprintf(name, sizeof(name), "stepper%d", axis);
    xTaskCreatePinnedToCore(stepper_task, name, 4096,
                            (void*)(intptr_t)axis, 2, &s_task[axis], 1);
  }
}

void stepper_move(int axis, int steps, int direction, int delay_us) {
  if (!valid_axis(axis)) { serial_tx_printf("ERR:bad axis %d", axis); return; }
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
  serial_tx_printf("OK,%d", axis);
}

bool stepper_is_busy(int axis) {
  if (!valid_axis(axis)) return false;
  return s_state[axis].load() != STEP_IDLE;
}

bool stepper_abort(int axis) {
  if (!valid_axis(axis)) return false;
  if (s_sync_active.load() && (axis == s_sync_a.load() || axis == s_sync_b.load())) {
    for (int peer : {s_sync_a.load(), s_sync_b.load()}) {
      uint8_t running = STEP_RUNNING;
      s_state[peer].compare_exchange_strong(running, STEP_ABORTING);
    }
  }
  uint8_t expected = STEP_RUNNING;
  if (s_state[axis].compare_exchange_strong(expected, STEP_ABORTING)) return true;
  return expected == STEP_ABORTING;
}

bool stepper_move_async(int axis, int steps, int direction, int delay_us) {
  if (!valid_axis(axis) || steps <= 0 || (direction != 0 && direction != 1) ||
      delay_us <= 0) return false;
  // 释放(失能)状态下发脉冲没有保持力矩，脉冲计数也会失去意义。
  if (!stepper_ena_locked(axis)) return false;
  uint8_t expected = STEP_IDLE;
  if (!s_state[axis].compare_exchange_strong(expected, STEP_RUNNING)) return false;
  s_steps[axis]    = steps;
  s_dir[axis]      = direction;
  s_delay_us[axis] = delay_us;
  xSemaphoreGive(s_start_sem[axis]);
  return true;
}

void stepper_run_diagnostics(int axis) {
  if (!valid_axis(axis)) { serial_tx_line("ERR:bad axis"); return; }
  int pul = PIN_PUL[axis], dpin = PIN_DIR[axis];
  serial_tx_line("");
  serial_tx_printf("===== DIAG axis=%d =====", axis);
  serial_tx_printf(">>> PUL(GPIO%d) 将置 HIGH 5 秒", pul);
  delay(1000);
  digitalWrite(pul, HIGH);
  delay(5000);
  digitalWrite(pul, LOW);
  delay(500);
  serial_tx_printf(">>> DIR(GPIO%d) 将置 HIGH 3 秒", dpin);
  delay(500);
  digitalWrite(dpin, HIGH);
  delay(3000);
  digitalWrite(dpin, LOW);
  serial_tx_line(">>> 5 + 5 脉冲测试");
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
  serial_tx_line("===== DIAG 结束 =====");
}
