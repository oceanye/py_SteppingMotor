#pragma once

// MOVE delay is an integer number of microseconds. Ten seconds covers the
// default rotary setup at 0.3 deg/s while remaining safely inside signed int.
constexpr int STEPPER_MAX_DELAY_US = 10000000;
constexpr int REMOTE_STEPPER_MIN_DELAY_US = 100;
