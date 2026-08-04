#pragma once
#include <stdint.h>

enum TrackDirection : int8_t {
  TRACK_STOPPED = 0,
  TRACK_FORWARD = 1,
  TRACK_REVERSE = -1,
};

struct TrackStatus {
  TrackDirection direction;
  uint8_t duty_pct;
  uint32_t remaining_ms;
};

void track_motor_init();
bool track_motor_drive(TrackDirection direction, uint8_t duty_pct, uint32_t lease_ms);
void track_motor_stop();
TrackStatus track_motor_get_status();
