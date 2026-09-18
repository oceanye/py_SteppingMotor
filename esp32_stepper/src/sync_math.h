#pragma once
#include <stdint.h>

// Pure common-clock trajectory math; also compiled by native tests.
inline double sync_progress(double s) {
  if (s < 0) s = 0;
  if (s > 1) s = 1;
  return s*s*s*(10 + s*(-15 + 6*s));
}

inline double sync_inverse_progress(double p) {
  double lo = 0, hi = 1;
  for (int i = 0; i < 32; ++i) {
    double mid = (lo + hi) / 2;
    if (sync_progress(mid) < p) lo = mid; else hi = mid;
  }
  return (lo + hi) / 2;
}

inline int sync_step_target(int tick, int steps, int master) {
  return (int)(((int64_t)tick * steps + master/2) / master);
}

inline bool sync_timing_valid(int master, int duration_us) {
  // Peak smoothstep rate = 1.875*N/T, capped at 5000 pps.
  return master > 0 && duration_us >= 1000 && duration_us <= 120000000
         && (int64_t)duration_us * 8 >= (int64_t)master * 3000;
}
