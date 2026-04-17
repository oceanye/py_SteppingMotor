#include <Arduino.h>
#include "config.h"
#include "stepper.h"

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

    // 格式: MOVE,steps,direction,delay_ms
    if (cmd.startsWith("MOVE,")) {
      int p1 = cmd.indexOf(',');
      int p2 = cmd.indexOf(',', p1 + 1);
      int p3 = cmd.indexOf(',', p2 + 1);

      if (p1 > 0 && p2 > 0 && p3 > 0) {
        int steps     = cmd.substring(p1 + 1, p2).toInt();
        int direction = cmd.substring(p2 + 1, p3).toInt();
        int delay_ms  = cmd.substring(p3 + 1).toInt();
        stepper_move(steps, direction, delay_ms);
      } else {
        Serial.println("ERR:bad format");
      }
    } else if (cmd == "DIAG") {
      stepper_run_diagnostics();
    } else if (cmd.length() > 0) {
      Serial.println("ERR:unknown command");
    }
  }
}
