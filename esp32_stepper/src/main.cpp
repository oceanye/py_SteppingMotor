#include <Arduino.h>
#include "config.h"
#include "protocol.h"
#include "foc_motor.h"
#include "stepper.h"     // 两个构建都需要步进
#if defined(DRIVE_MODE_FOC)
  static const int PIN_NFAULT[NUM_AXES] = { PIN_FOC_NFAULT_0, PIN_FOC_NFAULT_1 };
#endif

// 协议 v2.3+：全显式轴号。命令带 axis（0=L, 1=R）
//   FOC 构建：MOVE,<axis>,<steps>,<dir>,<delay_us> | FOC,<axis>,<sub>,<arg> | DIAG,<axis>
//   GEAR 构建：FOC,<axis>,<sub>,<arg> | DIAG,<axis>（无 MOVE）

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
#if defined(DRIVE_MODE_FOC)
  Serial.println("ESP32 Dual-Axis Motor Controller Ready (FOC mode)");
  Serial.print("NUM_AXES = "); Serial.println(NUM_AXES);
  for (int a = 0; a < NUM_AXES; a++) {
    Serial.print("  axis "); Serial.print(a); Serial.println(":");
    Serial.print("    stepper PUL="); Serial.print(a==0?PIN_STEP_PUL_0:PIN_STEP_PUL_1);
    Serial.print(" DIR=");            Serial.println(a==0?PIN_STEP_DIR_0:PIN_STEP_DIR_1);
    Serial.print("    FOC M1/2/3=");
    Serial.print(a==0?PIN_FOC_M1_0:PIN_FOC_M1_1); Serial.print("/");
    Serial.print(a==0?PIN_FOC_M2_0:PIN_FOC_M2_1); Serial.print("/");
    Serial.print(a==0?PIN_FOC_M3_0:PIN_FOC_M3_1);
    Serial.print(" EN="); Serial.print(a==0?PIN_FOC_EN_0:PIN_FOC_EN_1);
    Serial.print(" nFT="); Serial.println(a==0?PIN_FOC_NFAULT_0:PIN_FOC_NFAULT_1);
    Serial.print("    I2C SDA/SCL=");
    Serial.print(a==0?PIN_I2C_SDA_0:PIN_I2C_SDA_1); Serial.print("/");
    Serial.println(a==0?PIN_I2C_SCL_0:PIN_I2C_SCL_1);
  }
  stepper_init();
  foc_init();
  Serial.println("Protocol: MOVE,<axis>,s,d,us | FOC,<axis>,... | DIAG,<axis> | STDIAG,<axis> | MODE");
#elif defined(DRIVE_MODE_GEAR)
  Serial.println("ESP32 Controller Ready (GEAR mode, stepper + DC gear motor)");
  Serial.println("  fw: PCNT-quad-fix v2 + stepper");
  Serial.print("NUM_AXES = "); Serial.println(NUM_AXES);
  for (int a = 0; a < NUM_AXES; a++) {
    Serial.print("  axis "); Serial.print(a); Serial.println(":");
    Serial.print("    stepper PUL="); Serial.print(a==0?PIN_STEP_PUL_0:PIN_STEP_PUL_1);
    Serial.print(" DIR=");            Serial.println(a==0?PIN_STEP_DIR_0:PIN_STEP_DIR_1);
    Serial.print("    gear IN1=");    Serial.print(a==0?PIN_GEAR_IN1_0:PIN_GEAR_IN1_1);
    Serial.print(" IN2=");            Serial.print(a==0?PIN_GEAR_IN2_0:PIN_GEAR_IN2_1);
    Serial.print(" ENC_A=");          Serial.print(a==0?PIN_GEAR_ENCA_0:PIN_GEAR_ENCA_1);
    Serial.print(" ENC_B=");          Serial.println(a==0?PIN_GEAR_ENCB_0:PIN_GEAR_ENCB_1);
  }
  stepper_init();
  foc_init();   // GEAR 构建里这个符号由 gear_motor.cpp 提供
  Serial.println("Protocol: MOVE,<axis>,... | FOC,<axis>,... | DIAG,<axis> | STDIAG,<axis> | MODE");
#endif
}

#if defined(DRIVE_MODE_FOC)
static uint32_t s_last_fault_check = 0;
#endif

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) protocol_handle_line(cmd);
  }

#if defined(DRIVE_MODE_FOC)
  uint32_t now = millis();
  if (now - s_last_fault_check >= FAULT_POLL_MS) {
    s_last_fault_check = now;
    for (int a = 0; a < NUM_AXES; a++) {
      if (digitalRead(PIN_NFAULT[a]) == LOW &&
          foc_get_state(a) != FOC_STATE_DISABLED &&
          foc_get_state(a) != FOC_STATE_FAULT) {
        foc_latch_fault(a);
        Serial.print("FOC,"); Serial.print(a); Serial.println(",FAULT");
      }
    }
  }
#endif
}
