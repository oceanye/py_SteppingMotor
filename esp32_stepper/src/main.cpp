#include <Arduino.h>
#include "config.h"
#include "stepper.h"
#include "protocol.h"

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
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) protocol_handle_line(cmd);
  }
}
