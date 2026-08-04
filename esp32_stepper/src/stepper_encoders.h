#pragma once
#include <stdint.h>

// Diagnostic snapshot for one optional AS5600 behind TCA9548A channel axis.
// AS5600 is a single-turn absolute sensor. multi_turn_counts is only an
// in-RAM unwrap accumulated while this firmware remains powered and polling;
// it is lost at reset/power loss, so the machine must home again afterward.
struct StepperEncoderStatus {
  bool diagnostics_enabled;
  bool mux_online;
  bool sensor_online;
  bool magnet_detected;
  uint16_t raw_angle;
  int64_t multi_turn_counts;
  uint32_t age_ms;
  uint32_t error_count;
};

void stepper_encoders_init();
bool stepper_encoder_get_status(int axis, StepperEncoderStatus& status);
