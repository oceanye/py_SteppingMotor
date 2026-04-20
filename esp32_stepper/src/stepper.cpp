#include <Arduino.h>
#include "config.h"
#include "stepper.h"
#include <atomic>

// ── 异步步进任务（Core 1）──
static TaskHandle_t      s_task        = nullptr;
static SemaphoreHandle_t s_start_sem   = nullptr;
static std::atomic<bool> s_busy        {false};
static int               s_steps, s_dir, s_delay_ms;   // 任务参数，仅在 busy=false 时由 async 写

static void stepper_task(void* /*arg*/) {
  for (;;) {
    xSemaphoreTake(s_start_sem, portMAX_DELAY);
    int steps = s_steps;
    int dir   = s_dir;
    int dms   = s_delay_ms;

    digitalWrite(PIN_STEP_DIR, dir);
    delayMicroseconds(100);
    for (int i = 0; i < steps; i++) {
      digitalWrite(PIN_STEP_PUL, HIGH);
      delayMicroseconds(50);
      digitalWrite(PIN_STEP_PUL, LOW);
      // 用 vTaskDelay（而不是 delay）让其他任务尽早调度
      vTaskDelay(dms / portTICK_PERIOD_MS);
    }
    s_busy.store(false);
    Serial.println("STEP,DONE");
  }
}

void stepper_init() {
  pinMode(PIN_STEP_PUL, OUTPUT);
  pinMode(PIN_STEP_DIR, OUTPUT);
  digitalWrite(PIN_STEP_PUL, LOW);
  digitalWrite(PIN_STEP_DIR, LOW);

  s_start_sem = xSemaphoreCreateBinary();
  // Core 1，优先级 1 低于主 loop 但高于 idle
  xTaskCreatePinnedToCore(stepper_task, "stepper", 4096, nullptr, 1, &s_task, 1);
}

// 同步版本（保留，DIAG 可能直接调用）
void stepper_move(int steps, int direction, int delay_ms) {
  digitalWrite(PIN_STEP_DIR, direction);
  delayMicroseconds(100);
  for (int i = 0; i < steps; i++) {
    digitalWrite(PIN_STEP_PUL, HIGH);
    delayMicroseconds(50);
    digitalWrite(PIN_STEP_PUL, LOW);
    delay(delay_ms);
  }
  Serial.println("OK");
}

bool stepper_is_busy() {
  return s_busy.load();
}

bool stepper_move_async(int steps, int direction, int delay_ms) {
  // 调用方都在 Core 1 的 loop()（串口单线程），无竞态。
  if (s_busy.load()) return false;
  s_steps    = steps;
  s_dir      = direction;
  s_delay_ms = delay_ms;
  s_busy.store(true);
  xSemaphoreGive(s_start_sem);
  return true;
}

void stepper_run_diagnostics() {
  // ---- DIAG 1: 静态电平测试 ----
  Serial.println();
  Serial.println("===== DIAG 1: 静态电平测试 =====");
  Serial.println(">>> PUL+ 即将置 HIGH 持续 10 秒");
  Serial.println(">>> 请用万用表直流电压档 (20V) 黑笔夹 ESP32 GND，红笔依次测:");
  Serial.println("    a) ESP32 GPIO5 焊盘        期望 ~3.3V");
  Serial.println("    b) DM422 PUL+ 端子         期望 ~3.3V");
  Serial.println("    c) DM422 PUL+ 对 PUL-      期望 ~3.3V");
  delay(2000);
  digitalWrite(PIN_STEP_PUL, HIGH);
  Serial.println(">>> [PUL=HIGH] 开始测量 ...");
  delay(10000);

  Serial.println(">>> PUL+ 置 LOW 持续 3 秒 (应读到 ~0V)");
  digitalWrite(PIN_STEP_PUL, LOW);
  delay(3000);

  // ---- DIAG 2: DIR 引脚电平测试 ----
  Serial.println();
  Serial.println("===== DIAG 2: DIR 静态电平测试 =====");
  Serial.println(">>> DIR+ 即将置 HIGH 持续 5 秒，请测 DM422 DIR+ 对 GND 期望 ~3.3V");
  delay(1000);
  digitalWrite(PIN_STEP_DIR, HIGH);
  delay(5000);
  digitalWrite(PIN_STEP_DIR, LOW);

  // ---- DIAG 3: 超慢速大宽度脉冲（正反对称，净位移=0） ----
  Serial.println();
  Serial.println("===== DIAG 3: 慢速大脉冲测试 (正转5 + 反转5) =====");
  Serial.println(">>> 每脉冲 HIGH 20ms + LOW 80ms，应能听到咔声");
  delay(2000);

  Serial.println("  [正转 5 步]");
  digitalWrite(PIN_STEP_DIR, HIGH);
  delay(1);
  for (int i = 0; i < 5; i++) {
    digitalWrite(PIN_STEP_PUL, HIGH);
    delay(20);
    digitalWrite(PIN_STEP_PUL, LOW);
    delay(80);
  }

  delay(500);
  Serial.println("  [反转 5 步]");
  digitalWrite(PIN_STEP_DIR, LOW);
  delay(1);
  for (int i = 0; i < 5; i++) {
    digitalWrite(PIN_STEP_PUL, HIGH);
    delay(20);
    digitalWrite(PIN_STEP_PUL, LOW);
    delay(80);
  }

  // ---- DIAG 4: 中速脉冲（正反对称，幅度 1/10，净位移=0） ----
  Serial.println();
  Serial.println("===== DIAG 4: 中速脉冲测试 (正转500 + 反转500) =====");
  Serial.println(">>> 每步 2ms，单向总时长约 1 秒，观察转动是否明显");
  delay(1000);

  Serial.println("  [正转 500 步]");
  digitalWrite(PIN_STEP_DIR, HIGH);
  delay(1);
  for (int i = 0; i < 500; i++) {
    digitalWrite(PIN_STEP_PUL, HIGH);
    delayMicroseconds(50);
    digitalWrite(PIN_STEP_PUL, LOW);
    delay(2);
  }

  delay(800);
  Serial.println("  [反转 500 步，回到起点]");
  digitalWrite(PIN_STEP_DIR, LOW);
  delay(1);
  for (int i = 0; i < 500; i++) {
    digitalWrite(PIN_STEP_PUL, HIGH);
    delayMicroseconds(50);
    digitalWrite(PIN_STEP_PUL, LOW);
    delay(2);
  }

  Serial.println();
  Serial.println("===== 诊断结束，进入正常 MOVE 指令循环 =====");
  Serial.println("可发送: MOVE,steps,dir,delay_ms  例如 MOVE,400,1,20");
  Serial.println();
}
