#include <Arduino.h>
#include "config.h"
#include "protocol.h"
#include "foc_motor.h"
#include "stepper.h"     // 两个构建都需要步进
#include "serial_tx.h"
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
  if (!serial_tx_init()) {
    // No worker task has started yet, so this direct fallback cannot interleave.
    // Refuse to run without the mutex rather than silently corrupting protocol
    // lines under concurrent motor events.
    Serial.println("FATAL:serial tx mutex init failed");
    for (;;) delay(1000);
  }
  delay(500);
  serial_tx_line("");
#if defined(DRIVE_MODE_FOC)
  serial_tx_line("ESP32 Dual-Axis Motor Controller Ready (FOC mode)");
  serial_tx_printf("NUM_AXES = %d", NUM_AXES);
  for (int a = 0; a < NUM_AXES; a++) {
    serial_tx_printf("  axis %d:", a);
    serial_tx_printf("    stepper PUL=%d DIR=%d",
                     a == 0 ? PIN_STEP_PUL_0 : PIN_STEP_PUL_1,
                     a == 0 ? PIN_STEP_DIR_0 : PIN_STEP_DIR_1);
    serial_tx_printf("    FOC M1/2/3=%d/%d/%d EN=%d nFT=%d",
                     a == 0 ? PIN_FOC_M1_0 : PIN_FOC_M1_1,
                     a == 0 ? PIN_FOC_M2_0 : PIN_FOC_M2_1,
                     a == 0 ? PIN_FOC_M3_0 : PIN_FOC_M3_1,
                     a == 0 ? PIN_FOC_EN_0 : PIN_FOC_EN_1,
                     a == 0 ? PIN_FOC_NFAULT_0 : PIN_FOC_NFAULT_1);
    serial_tx_printf("    I2C SDA/SCL=%d/%d",
                     a == 0 ? PIN_I2C_SDA_0 : PIN_I2C_SDA_1,
                     a == 0 ? PIN_I2C_SCL_0 : PIN_I2C_SCL_1);
  }
  stepper_init();
  foc_init();
  serial_tx_line("Protocol: MOVE/STOP/ESTOP | FOC,<axis>,... | DIAG,<axis> | STDIAG,<axis> | MODE");
#elif defined(DRIVE_MODE_GEAR)
  hardware_estop_init();
  serial_tx_line("ESP32 Controller Ready (GEAR mode, stepper + DC gear motor)");
  serial_tx_line("  fw: accel-ramp + 6-axis stepper + PCNT-clear");
  serial_tx_printf("NUM_AXES (stepper) = %d", NUM_AXES);
  serial_tx_printf("NUM_GEAR_AXES = %d", NUM_GEAR_AXES);
  const int pul_pins[] = {PIN_STEP_PUL_0,PIN_STEP_PUL_1,PIN_STEP_PUL_2,PIN_STEP_PUL_3,PIN_STEP_PUL_4,PIN_STEP_PUL_5};
  const int dir_pins[] = {PIN_STEP_DIR_0,PIN_STEP_DIR_1,PIN_STEP_DIR_2,PIN_STEP_DIR_3,PIN_STEP_DIR_4,PIN_STEP_DIR_5};
  for (int a = 0; a < NUM_AXES; a++) {
    serial_tx_printf("  stepper %d: PUL=%d DIR=%d", a, pul_pins[a],
                     dir_pins[a]);
  }
  const int gin1[] = {PIN_GEAR_IN1_0, PIN_GEAR_IN1_1};
  const int gin2[] = {PIN_GEAR_IN2_0, PIN_GEAR_IN2_1};
  const int gea[]  = {PIN_GEAR_ENCA_0, PIN_GEAR_ENCA_1};
  const int geb[]  = {PIN_GEAR_ENCB_0, PIN_GEAR_ENCB_1};
  for (int a = 0; a < NUM_GEAR_AXES; a++) {
    serial_tx_printf("  gear %d: IN1=%d IN2=%d ENC_A=%d ENC_B=%d", a,
                     gin1[a], gin2[a], gea[a], geb[a]);
  }
  stepper_init();
  foc_init();   // GEAR 构建里这个符号由 gear_motor.cpp 提供
  track_motor_init();
  stepper_encoders_init();
  remote_stepper_init();
#if HW_ESTOP_ENABLED
  serial_tx_printf("  hw estop: GPIO%d%s", PIN_HW_ESTOP,
                   HW_ESTOP_ACTIVE_HIGH
                       ? " active-high(NC+pullup, open=fault)"
                       : " active-low");
#else
  serial_tx_line("  hw estop: DISABLED (enable only after wiring NC loop)");
#endif
#if GEAR_NFAULT_ENABLED
  pinMode(PIN_GEAR_NFAULT_0, INPUT_PULLUP);
  pinMode(PIN_GEAR_NFAULT_1, INPUT_PULLUP);
  serial_tx_printf("  gear nFAULT: GPIO%d/GPIO%d", PIN_GEAR_NFAULT_0,
                   PIN_GEAR_NFAULT_1);
#endif
  serial_tx_printf("  track D: IN1=%d IN2=%d", PIN_TRACK_D_IN1,
                   PIN_TRACK_D_IN2);
#if STEPPER_ENCODER_DIAGNOSTICS_ENABLED
  serial_tx_printf("  optional step encoders: ENABLED TCA9548A@0x70 SDA/SCL=%d/%d",
                   PIN_STEPPER_ENCODER_I2C_SDA,
                   PIN_STEPPER_ENCODER_I2C_SCL);
#else
  serial_tx_line("  optional step encoders: DISABLED/reserved (set diagnostics flag after hardware install)");
#endif
#if REMOTE_STEPPER_ENABLED
  serial_tx_printf("  RS485 Pico nodes: ENABLED, global axes 6..%d RX/TX/DE=%d/%d/%d",
                   REMOTE_STEPPER_TOTAL_AXES - 1, PIN_REMOTE_STEPPER_RX,
                   PIN_REMOTE_STEPPER_TX, PIN_REMOTE_STEPPER_DE);
#else
  serial_tx_line("  RS485 Pico nodes: DISABLED (select a *_remote environment)");
#endif
  serial_tx_line("Protocol: MOVE/STOP/ESTOP | FOC | TRACK | ENC,<axis>,S | DIAG | STDIAG | MODE");
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
        serial_tx_printf("FOC,%d,FAULT", a);
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
          serial_tx_line("HWESTOP,TRIGGERED");
        }
      } else if (s_hwestop_latched) {
        s_hwestop_latched = false;
        serial_tx_line("HWESTOP,CLEARED");
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
        serial_tx_printf("FOC,%d,FAULT", a);
      }
    }
#endif
  }
#endif
}
