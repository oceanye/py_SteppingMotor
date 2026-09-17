#include "protocol.h"
#include "protocol_parse.h"
#include "foc_motor.h"
#include "config.h"
#include "stepper.h"
#include "protocol_limits.h"
#include "serial_tx.h"
#if defined(DRIVE_MODE_GEAR)
#include "hardware_estop.h"
#include "track_motor.h"
#include "stepper_encoders.h"
#include "remote_stepper.h"
#endif

static constexpr int MAX_TOKENS = 6;
static constexpr int MAX_MOVE_STEPS = 20000000;

static void reply_ok_axis(int axis) { serial_tx_printf("OK,%d", axis); }
static void reply_err(const char* why) { serial_tx_printf("ERR:%s", why); }

static bool reject_if_hardware_estop_active() {
#if defined(DRIVE_MODE_GEAR)
  if (hardware_estop_is_active()) {
    reply_err("hardware estop active");
    return true;
  }
#endif
  return false;
}

// Returns -1 when the line contains more fields than the caller can hold.
static int split_tokens(const String& line, String tokens[], int capacity) {
  int count = 0;
  int start = 0;
  for (;;) {
    if (count >= capacity) return -1;
    const int comma = line.indexOf(',', start);
    if (comma < 0) {
      tokens[count++] = line.substring(start);
      return count;
    }
    tokens[count++] = line.substring(start, comma);
    start = comma + 1;
  }
}

static bool parse_axis(const String& token, int axis_count, int& axis) {
  return protocol_parse_int(token.c_str(), 0, axis_count - 1, axis);
}

static constexpr int stepper_axis_count() {
#if defined(DRIVE_MODE_GEAR) && REMOTE_STEPPER_ENABLED
  return REMOTE_STEPPER_TOTAL_AXES;
#else
  return NUM_AXES;
#endif
}

static void handle_move(const String tok[], int n) {
  if (n != 5) { reply_err("bad format"); return; }
  if (reject_if_hardware_estop_active()) return;
  int steps = 0, direction = 0, delay_us = 0;
  if (!protocol_parse_int(tok[2].c_str(), 1, MAX_MOVE_STEPS, steps) ||
      !protocol_parse_int(tok[3].c_str(), 0, 1, direction) ||
      !protocol_parse_int(tok[4].c_str(), 1, STEPPER_MAX_DELAY_US, delay_us)) {
    reply_err("bad value"); return;
  }

  if (tok[1] == "*") {
#if defined(DRIVE_MODE_GEAR) && REMOTE_STEPPER_ENABLED
    // Wildcard spans local and remote axes. Validate the stricter RP2040
    // period before starting any local motor, avoiding a partial broadcast.
    if (delay_us < REMOTE_STEPPER_MIN_DELAY_US) {
      reply_err("remote delay below 100us"); return;
    }
#endif
    for (int axis = 0; axis < NUM_AXES; ++axis) {
      if (stepper_move_async(axis, steps, direction, delay_us)) {
        serial_tx_printf("ACK,%d", axis);
      } else {
        serial_tx_printf("ERR:busy %d", axis);
      }
    }
#if defined(DRIVE_MODE_GEAR) && REMOTE_STEPPER_ENABLED
    for (int axis = NUM_AXES; axis < REMOTE_STEPPER_TOTAL_AXES; ++axis) {
      if (remote_stepper_move(axis, steps, direction, delay_us)) {
        serial_tx_printf("ACK,%d", axis);
      } else {
        serial_tx_printf("ERR:%s %d", remote_stepper_last_error(), axis);
      }
    }
#endif
    return;
  }
  int axis = 0;
  if (!parse_axis(tok[1], stepper_axis_count(), axis)) { reply_err("bad axis"); return; }
#if defined(DRIVE_MODE_GEAR) && REMOTE_STEPPER_ENABLED
  if (axis >= NUM_AXES) {
    // RP2040 nodes guarantee pulse timing only at/above 100 us. Keep the
    // existing 1 us lower bound for the six directly wired ESP32 axes.
    if (delay_us < REMOTE_STEPPER_MIN_DELAY_US) {
      reply_err("remote delay below 100us"); return;
    }
    if (!remote_stepper_move(axis, steps, direction, delay_us)) {
      reply_err(remote_stepper_last_error()); return;
    }
    serial_tx_printf("ACK,%d", axis);
    return;
  }
#endif
  if (!stepper_ena_locked(axis)) { reply_err("axis released"); return; }
  if (!stepper_move_async(axis, steps, direction, delay_us)) {
    reply_err("busy"); return;
  }
  serial_tx_printf("ACK,%d", axis);
}

// ENA,<axis>,<0|1>  设置驱动器释放(0)/锁定(1)；ENA,<axis>,S 查询当前状态。
// 仅接了 ENA 线的本地轴支持(旋转轴)；释放正在运动的轴会被拒绝。
static void handle_ena(const String tok[], int n) {
  if (n != 3) { reply_err("bad format"); return; }
  int axis = 0;
  if (!parse_axis(tok[1], NUM_AXES, axis)) { reply_err("bad axis"); return; }
  if (tok[2] == "S") {
    serial_tx_printf("OK,ENA,%d,%d", axis, stepper_ena_locked(axis) ? 1 : 0);
    return;
  }
  int value = 0;
  if (!protocol_parse_int(tok[2].c_str(), 0, 1, value)) { reply_err("bad value"); return; }
  if (value == 0 && stepper_is_busy(axis)) { reply_err("busy"); return; }
  if (!stepper_set_ena(axis, value == 1)) { reply_err("unsupported"); return; }
  serial_tx_printf("OK,ENA,%d,%d", axis, value);
}

static void handle_stop(const String tok[], int n) {
  if (n != 2) { reply_err("bad format"); return; }
  if (tok[1] == "*") {
    for (int axis = 0; axis < NUM_AXES; ++axis) {
      stepper_abort(axis);
      reply_ok_axis(axis);
    }
#if defined(DRIVE_MODE_GEAR) && REMOTE_STEPPER_ENABLED
    remote_stepper_stop_all();
    for (int axis = NUM_AXES; axis < REMOTE_STEPPER_TOTAL_AXES; ++axis)
      reply_ok_axis(axis);
#endif
    return;
  }
  int axis = 0;
  if (!parse_axis(tok[1], stepper_axis_count(), axis)) { reply_err("bad axis"); return; }
#if defined(DRIVE_MODE_GEAR) && REMOTE_STEPPER_ENABLED
  if (axis >= NUM_AXES) {
    if (!remote_stepper_stop(axis)) { reply_err(remote_stepper_last_error()); return; }
    reply_ok_axis(axis);
    return;
  }
#endif
  stepper_abort(axis);  // STOP is idempotent; a valid idle axis is still OK.
  reply_ok_axis(axis);
}

void system_estop() {
  for (int axis = 0; axis < NUM_AXES; ++axis) stepper_abort(axis);
  // 急停后不留失能态：全部恢复锁定，机构不会长期处于可被外力扭动状态。
  for (int axis = 0; axis < NUM_AXES; ++axis) stepper_set_ena(axis, true);
#if defined(DRIVE_MODE_GEAR)
  #if REMOTE_STEPPER_ENABLED
  remote_stepper_estop();
  #endif
  track_motor_stop();
  constexpr int motor_axes = NUM_GEAR_AXES;
#else
  constexpr int motor_axes = NUM_AXES;
#endif
  for (int axis = 0; axis < motor_axes; ++axis) foc_request_enable(axis, false);
}

static void handle_estop(const String tok[], int n) {
  if (n != 1) { reply_err("bad format"); return; }
  system_estop();
  serial_tx_line("OK,ESTOP");
}

static void handle_stdiag(const String tok[], int n) {
  if (n != 2) { reply_err("bad format"); return; }
  if (reject_if_hardware_estop_active()) return;
  int axis = 0;
  if (!parse_axis(tok[1], stepper_axis_count(), axis)) { reply_err("bad axis"); return; }
#if defined(DRIVE_MODE_GEAR) && REMOTE_STEPPER_ENABLED
  if (axis >= NUM_AXES) {
    if (remote_stepper_is_busy(axis)) { reply_err("busy"); return; }
    if (!remote_stepper_run_diagnostics(axis)) reply_err(remote_stepper_last_error());
    else { serial_tx_printf("ACK,%d", axis); }
    return;
  }
#endif
  if (stepper_is_busy(axis)) { reply_err("busy"); return; }
  stepper_run_diagnostics(axis);
}

static void handle_diag(const String tok[], int n) {
#if defined(DRIVE_MODE_FOC)
  if (n != 2) { reply_err("bad format"); return; }
  int axis = 0;
  if (!parse_axis(tok[1], NUM_AXES, axis)) { reply_err("bad axis"); return; }
  if (stepper_is_busy(axis)) { reply_err("busy"); return; }
  stepper_run_diagnostics(axis);
#elif defined(DRIVE_MODE_GEAR)
  if (n != 2 && n != 3) { reply_err("bad format"); return; }
  if (n == 3 && tok[2] != "LIVE") { reply_err("bad format"); return; }
  int axis = 0;
  if (!parse_axis(tok[1], NUM_GEAR_AXES, axis)) { reply_err("bad axis"); return; }
  if (n == 3) foc_run_diagnostics_live(axis);
  else foc_run_diagnostics(axis);
#endif
}

static void dispatch_foc_single(int axis, const String& sub,
                                bool has_arg, const String& arg) {
  if (sub == "S" && !has_arg) {
    serial_tx_printf("FOC,%d,S,%d,%.1f,%.1f,%d", axis,
                     (int)foc_get_state(axis),
                     (double)foc_get_current_deg(axis),
                     (double)foc_get_target_deg(axis),
                     foc_is_fault_latched(axis) ? 1 : 0);
    return;
  }
  if (sub == "EN" && has_arg) {
    int value = 0;
    if (!protocol_parse_int(arg.c_str(), 0, 1, value)) { reply_err("bad value"); return; }
    if (value == 1 && reject_if_hardware_estop_active()) return;
    if (foc_request_enable(axis, value == 1)) reply_ok_axis(axis);
    else reply_err("fault latched");
    return;
  }
  if (sub == "A" && has_arg) {
    if (reject_if_hardware_estop_active()) return;
    float value = 0.0f;
#if defined(DRIVE_MODE_FOC)
    constexpr float limit = FOC_MAX_ANGLE_ABS;
#else
    constexpr float limit = GEAR_MAX_ANGLE_ABS;
#endif
    if (!protocol_parse_float(arg.c_str(), -limit, limit, value)) { reply_err("bad value"); return; }
    if (foc_set_target_deg(axis, value)) reply_ok_axis(axis); else reply_err("out of range");
    return;
  }
  if (sub == "H" && !has_arg) {
    if (reject_if_hardware_estop_active()) return;
    foc_home(axis); reply_ok_axis(axis); return;
  }
  if (sub == "CLR" && !has_arg) {
    if (foc_clear_fault(axis)) reply_ok_axis(axis); else reply_err("no fault");
    return;
  }
  if (sub == "V" && has_arg) {
    float value = 0.0f;
#if defined(DRIVE_MODE_FOC)
    constexpr float max_value = FOC_MAX_V_LIMIT;
#else
    constexpr float max_value = 100.0f;
#endif
    if (!protocol_parse_float(arg.c_str(), 0.0001f, max_value, value)) { reply_err("bad value"); return; }
    if (foc_set_voltage_limit(axis, value)) reply_ok_axis(axis); else reply_err("out of range");
    return;
  }
  if (sub == "PA" && has_arg) {
    float value = 0.0f;
#if defined(DRIVE_MODE_FOC)
    constexpr float min_value = 0.1f, max_value = 50.0f;
#else
    constexpr float min_value = 0.0f, max_value = 1000.0f;
#endif
    if (!protocol_parse_float(arg.c_str(), min_value, max_value, value)) { reply_err("bad value"); return; }
    if (foc_set_p_angle(axis, value)) reply_ok_axis(axis); else reply_err("out of range");
    return;
  }
#if defined(DRIVE_MODE_FOC)
  if (sub == "PP" && has_arg) {
    int value = 0;
    if (!protocol_parse_int(arg.c_str(), 1, 50, value)) { reply_err("bad value"); return; }
    if (foc_set_pole_pairs_and_store(axis, value)) reply_ok_axis(axis); else reply_err("out of range");
    return;
  }
  if (sub == "VP" && has_arg) {
    float value = 0.0f;
    if (!protocol_parse_float(arg.c_str(), 0.01f, 2.0f, value)) { reply_err("bad value"); return; }
    if (foc_set_p_velocity(axis, value)) reply_ok_axis(axis); else reply_err("out of range");
    return;
  }
  if (sub == "REALIGN" && !has_arg) {
    if (foc_force_realign(axis)) reply_ok_axis(axis); else reply_err("bad axis");
    return;
  }
#endif
#if defined(DRIVE_MODE_GEAR)
  if (sub == "GR" && has_arg) {
    float value = 0.0f;
    if (!protocol_parse_float(arg.c_str(), 1.0f, 10000.0f, value)) { reply_err("bad value"); return; }
    if (foc_set_gear_ratio_and_store(axis, value)) reply_ok_axis(axis); else reply_err("out of range");
    return;
  }
  if (sub == "PI" && has_arg) {
    float value = 0.0f;
    if (!protocol_parse_float(arg.c_str(), 0.0f, 1000.0f, value)) { reply_err("bad value"); return; }
    if (foc_set_p_integral(axis, value)) reply_ok_axis(axis); else reply_err("out of range");
    return;
  }
  if (sub == "PD" && has_arg) {
    float value = 0.0f;
    if (!protocol_parse_float(arg.c_str(), 0.0f, 100.0f, value)) { reply_err("bad value"); return; }
    if (foc_set_p_derivative(axis, value)) reply_ok_axis(axis); else reply_err("out of range");
    return;
  }
#endif
  reply_err("bad format");
}

static void handle_foc(const String tok[], int n) {
  if (n != 3 && n != 4) { reply_err("bad format"); return; }
#if defined(DRIVE_MODE_GEAR)
  constexpr int motor_axes = NUM_GEAR_AXES;
#else
  constexpr int motor_axes = NUM_AXES;
#endif
  const bool has_arg = n == 4;
  const String arg = has_arg ? tok[3] : String();
  if (tok[1] == "*") {
    for (int axis = 0; axis < motor_axes; ++axis)
      dispatch_foc_single(axis, tok[2], has_arg, arg);
    return;
  }
  int axis = 0;
  if (!parse_axis(tok[1], motor_axes, axis)) { reply_err("bad axis"); return; }
  dispatch_foc_single(axis, tok[2], has_arg, arg);
}

#if defined(DRIVE_MODE_GEAR)
static void reply_encoder_status(int axis) {
  StepperEncoderStatus status = {};
  if (!stepper_encoder_get_status(axis, status)) { reply_err("bad axis"); return; }
  const double single_deg = (double)status.raw_angle * 360.0 / 4096.0;
  const double multi_deg = (double)status.multi_turn_counts * 360.0 / 4096.0;
  // ENC,<axis>,S,<diag>,<mux>,<online>,<magnet>,<raw>,<single_deg>,
  //     <runtime_multi_deg>,<age_ms>,<errors>
  serial_tx_printf("ENC,%d,S,%d,%d,%d,%d,%u,%.2f,%.2f,%lu,%lu", axis,
                   status.diagnostics_enabled ? 1 : 0,
                   status.mux_online ? 1 : 0,
                   status.sensor_online ? 1 : 0,
                   status.magnet_detected ? 1 : 0,
                   (unsigned int)status.raw_angle, single_deg, multi_deg,
                   (unsigned long)status.age_ms,
                   (unsigned long)status.error_count);
}

static void handle_encoder(const String tok[], int n) {
  if (n != 3 || tok[2] != "S") { reply_err("bad format"); return; }
  int axis = 0;
  if (!parse_axis(tok[1], NUM_STEPPER_ENCODERS, axis)) { reply_err("bad axis"); return; }
  reply_encoder_status(axis);
}

static void handle_track(const String tok[], int n) {
  if (n < 3 || tok[1] != "D") { reply_err("bad format"); return; }
  if (tok[2] == "STOP" && n == 3) {
    track_motor_stop();
    serial_tx_line("OK,TRACK,D");
    return;
  }
  if (tok[2] == "S" && n == 3) {
    const TrackStatus status = track_motor_get_status();
    const char* state = status.direction == TRACK_FORWARD ? "FWD" :
                        status.direction == TRACK_REVERSE ? "REV" : "STOP";
    serial_tx_printf("TRACK,D,S,%s,%u,%lu", state,
                     (unsigned int)status.duty_pct,
                     (unsigned long)status.remaining_ms);
    return;
  }
  if ((tok[2] == "FWD" || tok[2] == "REV") && (n == 4 || n == 5)) {
    if (reject_if_hardware_estop_active()) return;
    int duty = 0, lease = TRACK_DEFAULT_LEASE_MS;
    if (!protocol_parse_int(tok[3].c_str(), 1, 100, duty) ||
        (n == 5 && !protocol_parse_int(tok[4].c_str(), TRACK_MIN_LEASE_MS,
                                       TRACK_MAX_LEASE_MS, lease))) {
      reply_err("bad value"); return;
    }
    const TrackDirection direction = tok[2] == "FWD" ? TRACK_FORWARD : TRACK_REVERSE;
    if (!track_motor_drive(direction, (uint8_t)duty, (uint32_t)lease)) {
      reply_err("bad value"); return;
    }
    serial_tx_line("OK,TRACK,D");
    return;
  }
  reply_err("bad format");
}

static void handle_node(const String tok[], int n) {
#if REMOTE_STEPPER_ENABLED
  if (n != 3 || tok[2] != "S") { reply_err("bad format"); return; }
  if (tok[1] == "*") { remote_stepper_print_status(0); return; }
  int node = 0;
  if (!protocol_parse_int(tok[1].c_str(), 1,
                          REMOTE_STEPPER_NODE_COUNT, node)) {
    reply_err("bad node"); return;
  }
  remote_stepper_print_status(node);
#else
  (void)tok; (void)n;
  reply_err("remote steppers disabled");
#endif
}
#endif

void protocol_handle_line(const String& command) {
  if (command.length() == 0) return;
  if (command.length() > 160) { reply_err("line too long"); return; }
  if (protocol_count_fields(command.c_str()) > MAX_TOKENS) {
    reply_err("too many fields"); return;
  }
  String tok[MAX_TOKENS];
  const int n = split_tokens(command, tok, MAX_TOKENS);
  if (n < 0) { reply_err("too many fields"); return; }

  if (tok[0] == "MODE") {
    if (n != 1) { reply_err("bad format"); return; }
#if defined(DRIVE_MODE_FOC)
    serial_tx_line("MODE,FOC");
#elif defined(DRIVE_MODE_GEAR)
    serial_tx_line("MODE,GEAR");
#endif
  } else if (tok[0] == "MOVE") handle_move(tok, n);
  else if (tok[0] == "STOP") handle_stop(tok, n);
  else if (tok[0] == "ESTOP") handle_estop(tok, n);
  else if (tok[0] == "ENA") handle_ena(tok, n);
  else if (tok[0] == "STDIAG") handle_stdiag(tok, n);
  else if (tok[0] == "DIAG") handle_diag(tok, n);
  else if (tok[0] == "FOC") handle_foc(tok, n);
#if defined(DRIVE_MODE_GEAR)
  else if (tok[0] == "TRACK") handle_track(tok, n);
  else if (tok[0] == "ENC") handle_encoder(tok, n);
  else if (tok[0] == "NODE") handle_node(tok, n);
#endif
  else reply_err("unknown command");
}
