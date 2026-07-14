#include <Arduino.h>
#include "config.h"
#include "stepper.h"
#include <atomic>

// 每轴引脚（编译期常量数组）
static const int PIN_PUL[NUM_AXES] = { PIN_STEP_PUL_0, PIN_STEP_PUL_1 };
static const int PIN_DIR[NUM_AXES] = { PIN_STEP_DIR_0, PIN_STEP_DIR_1 };

// 每轴异步任务状态
static TaskHandle_t      s_task[NUM_AXES]      = {nullptr, nullptr};
static SemaphoreHandle_t s_start_sem[NUM_AXES] = {nullptr, nullptr};
static std::atomic<bool> s_busy[NUM_AXES];
static int               s_steps[NUM_AXES], s_dir[NUM_AXES], s_delay_us[NUM_AXES];

static bool valid_axis(int axis) { return axis >= 0 && axis < NUM_AXES; }

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
    for (int i = 0; i < steps; i++) {
      digitalWrite(pul, HIGH);
      delayMicroseconds(50);
      digitalWrite(pul, LOW);
      if (dus >= 1000) {
        vTaskDelay(dus / 1000 / portTICK_PERIOD_MS);
      } else if (dus > 0) {
        delayMicroseconds(dus);
        if ((i % 50) == 49) vTaskDelay(1);
      }
    }
    s_busy[axis].store(false);
    Serial.print("STEP,");
    Serial.print(axis);
    Serial.println(",DONE");
  }
}

void stepper_init() {
  for (int axis = 0; axis < NUM_AXES; axis++) {
    pinMode(PIN_PUL[axis], OUTPUT);
    pinMode(PIN_DIR[axis], OUTPUT);
    digitalWrite(PIN_PUL[axis], LOW);
    digitalWrite(PIN_DIR[axis], LOW);
    s_start_sem[axis] = xSemaphoreCreateBinary();
    s_busy[axis].store(false);
    char name[16]; snprintf(name, sizeof(name), "stepper%d", axis);
    xTaskCreatePinnedToCore(stepper_task, name, 4096,
                            (void*)(intptr_t)axis, 1, &s_task[axis], 1);
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
  return s_busy[axis].load();
}

bool stepper_move_async(int axis, int steps, int direction, int delay_us) {
  if (!valid_axis(axis)) return false;
  if (s_busy[axis].load()) return false;
  s_steps[axis]    = steps;
  s_dir[axis]      = direction;
  s_delay_us[axis] = delay_us;
  s_busy[axis].store(true);
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
