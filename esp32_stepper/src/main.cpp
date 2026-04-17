#include <Arduino.h>
#include "config.h"
#include "stepper.h"
#include "protocol.h"
#include "foc_motor.h"

// ====================================================================
// 诊断固件 v1 —— 用于定位"指令OK但电机不转"的故障层
// 启动后先自动跑两段诊断，然后进入正常 Serial 指令循环
// ====================================================================

void setup() {
  Serial.begin(115200);
  stepper_init();

  delay(500);
  Serial.println();
  Serial.println("ESP32 Stepper Ready");
  Serial.print("PUL_PIN = GPIO"); Serial.println(PIN_STEP_PUL);
  Serial.print("DIR_PIN = GPIO"); Serial.println(PIN_STEP_DIR);
  Serial.println("发送 MOVE,steps,dir,delay_ms  或  DIAG");

  foc_init();
}

static uint32_t s_last_fault_check = 0;

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) protocol_handle_line(cmd);
  }

  // 周期性检查 DRV8313 nFAULT 引脚（INPUT_PULLUP，正常 HIGH）
  uint32_t now = millis();
  if (now - s_last_fault_check >= FAULT_POLL_MS) {
    s_last_fault_check = now;
    if (digitalRead(PIN_FOC_NFAULT) == LOW &&
        foc_get_state() != FOC_STATE_DISABLED &&
        foc_get_state() != FOC_STATE_FAULT) {
      foc_latch_fault();
      Serial.println("FOC,FAULT");
    }
  }
}
