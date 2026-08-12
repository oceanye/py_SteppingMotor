#include "hardware_estop.h"

#include <Arduino.h>

#include "config.h"
#include "safety_input.h"

void hardware_estop_init() {
#if defined(DRIVE_MODE_GEAR) && HW_ESTOP_ENABLED
  pinMode(PIN_HW_ESTOP, INPUT_PULLUP);
#endif
}

bool hardware_estop_is_active() {
#if defined(DRIVE_MODE_GEAR) && HW_ESTOP_ENABLED
  const bool pin_high = digitalRead(PIN_HW_ESTOP) == HIGH;
  return safety_input_is_active(pin_high, HW_ESTOP_ACTIVE_HIGH != 0);
#else
  return false;
#endif
}
