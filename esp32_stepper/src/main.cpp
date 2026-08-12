#include <Arduino.h>
#include "config.h"
#include "protocol.h"
#include "foc_motor.h"
#include "stepper.h"     // 两个构建都需要步进
#if defined(DRIVE_MODE_GEAR)
#include "hardware_estop.h"
#include "track_motor.h"
#include "stepper_encoders.h"
#include "remote_stepper.h"
#endif
#if defined(DRIVE_MODE_FOC)
  static const int PIN_NFAULT[NUM_AXES] = { PIN_FOC_NFAULT_0, PIN_FOC_NFAULT_1 };
#endif

// 协议 v2.3+：全显式轴号。
//   FOC 构建：MOVE,<axis>,<steps>,<dir>,<delay_us> | FOC,<axis>,<sub>,<arg> | DIAG,<axis>
//   GEAR 构建：六路 MOVE/STOP | 两路 FOC（闭环减速电机）| TRACK,D,... | DIAG

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
  Serial.println("Protocol: MOVE/STOP/ESTOP | FOC,<axis>,... | DIAG,<axis> | STDIAG,<axis> | MODE");
#elif defined(DRIVE_MODE_GEAR)
  hardware_estop_init();
  Serial.println("ESP32 Controller Ready (GEAR mode, stepper + DC gear motor)");
  Serial.println("  fw: accel-ramp + 6-axis stepper + PCNT-clear");
  Serial.print("NUM_AXES (stepper) = "); Serial.println(NUM_AXES);
  Serial.print("NUM_GEAR_AXES = "); Serial.println(NUM_GEAR_AXES);
  const int pul_pins[] = {PIN_STEP_PUL_0,PIN_STEP_PUL_1,PIN_STEP_PUL_2,PIN_STEP_PUL_3,PIN_STEP_PUL_4,PIN_STEP_PUL_5};
  const int dir_pins[] = {PIN_STEP_DIR_0,PIN_STEP_DIR_1,PIN_STEP_DIR_2,PIN_STEP_DIR_3,PIN_STEP_DIR_4,PIN_STEP_DIR_5};
  for (int a = 0; a < NUM_AXES; a++) {
    Serial.print("  stepper "); Serial.print(a);
    Serial.print(": PUL="); Serial.print(pul_pins[a]);
    Serial.print(" DIR="); Serial.println(dir_pins[a]);
  }
  const int gin1[] = {PIN_GEAR_IN1_0, PIN_GEAR_IN1_1};
  const int gin2[] = {PIN_GEAR_IN2_0, PIN_GEAR_IN2_1};
  const int gea[]  = {PIN_GEAR_ENCA_0, PIN_GEAR_ENCA_1};
  const int geb[]  = {PIN_GEAR_ENCB_0, PIN_GEAR_ENCB_1};
  for (int a = 0; a < NUM_GEAR_AXES; a++) {
    Serial.print("  gear "); Serial.print(a);
    Serial.print(": IN1="); Serial.print(gin1[a]);
    Serial.print(" IN2="); Serial.print(gin2[a]);
    Serial.print(" ENC_A="); Serial.print(gea[a]);
    Serial.print(" ENC_B="); Serial.println(geb[a]);
  }
  stepper_init();
  foc_init();   // GEAR 构建里这个符号由 gear_motor.cpp 提供
  track_motor_init();
  stepper_encoders_init();
  remote_stepper_init();
#if HW_ESTOP_ENABLED
  Serial.print("  hw estop: GPIO"); Serial.print(PIN_HW_ESTOP);
  Serial.println(HW_ESTOP_ACTIVE_HIGH ? " active-high(NC+pullup, open=fault)" : " active-low");
#else
  Serial.println("  hw estop: DISABLED (enable only after wiring NC loop)");
#endif
#if GEAR_NFAULT_ENABLED
  pinMode(PIN_GEAR_NFAULT_0, INPUT_PULLUP);
  pinMode(PIN_GEAR_NFAULT_1, INPUT_PULLUP);
  Serial.print("  gear nFAULT: GPIO"); Serial.print(PIN_GEAR_NFAULT_0);
  Serial.print("/GPIO"); Serial.println(PIN_GEAR_NFAULT_1);
#endif
  Serial.print("  track D: IN1="); Serial.print(PIN_TRACK_D_IN1);
  Serial.print(" IN2="); Serial.println(PIN_TRACK_D_IN2);
#if STEPPER_ENCODER_DIAGNOSTICS_ENABLED
  Serial.print("  optional step encoders: ENABLED TCA9548A@0x70 SDA/SCL=");
  Serial.print(PIN_STEPPER_ENCODER_I2C_SDA); Serial.print('/');
  Serial.println(PIN_STEPPER_ENCODER_I2C_SCL);
#else
  Serial.println("  optional step encoders: DISABLED/reserved (set diagnostics flag after hardware install)");
#endif
#if REMOTE_STEPPER_ENABLED
  Serial.print("  RS485 Pico nodes: ENABLED, global axes 6..");
  Serial.print(REMOTE_STEPPER_TOTAL_AXES - 1); Serial.print(" RX/TX/DE=");
  Serial.print(PIN_REMOTE_STEPPER_RX); Serial.print('/');
  Serial.print(PIN_REMOTE_STEPPER_TX); Serial.print('/');
  Serial.println(PIN_REMOTE_STEPPER_DE);
#else
  Serial.println("  RS485 Pico nodes: DISABLED (select a *_remote environment)");
#endif
  Serial.println("Protocol: MOVE/STOP/ESTOP | FOC | TRACK | ENC,<axis>,S | DIAG | STDIAG | MODE");
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
#elif defined(DRIVE_MODE_GEAR)
  remote_stepper_tick();
  static uint32_t s_last_gear_safety_check = 0;
  uint32_t now = millis();
  if (now - s_last_gear_safety_check >= FAULT_POLL_MS) {
    s_last_gear_safety_check = now;
#if HW_ESTOP_ENABLED
    {
      static bool s_hwestop_latched = false;
      const bool active = hardware_estop_is_active();
      if (active) {
        // Keep enforcing the stop while the NC loop is open.  The protocol
        // also rejects all motion-starting commands during this interval.
        system_estop();
        if (!s_hwestop_latched) {
          s_hwestop_latched = true;
          Serial.println("HWESTOP,TRIGGERED");
        }
      } else if (s_hwestop_latched) {
        s_hwestop_latched = false;
        Serial.println("HWESTOP,CLEARED");
      }
    }
#endif
#if GEAR_NFAULT_ENABLED
    for (int a = 0; a < NUM_GEAR_AXES; ++a) {
      static const int nfault_pin[NUM_GEAR_AXES] = { PIN_GEAR_NFAULT_0, PIN_GEAR_NFAULT_1 };
      if (digitalRead(nfault_pin[a]) == LOW &&
          foc_get_state(a) != FOC_STATE_DISABLED &&
          foc_get_state(a) != FOC_STATE_FAULT) {
        foc_latch_fault(a);
        Serial.print("FOC,"); Serial.print(a); Serial.println(",FAULT");
      }
    }
#endif
  }
#endif
}
