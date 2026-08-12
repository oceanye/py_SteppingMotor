#pragma once

// Arduino-independent input polarity helper.  Keeping the electrical mapping
// here lets the native test suite verify the fail-safe NC wiring convention.
inline bool safety_input_is_active(bool pin_high, bool active_high) {
  return pin_high == active_high;
}
