// esp32_stepper/src/protocol.cpp
#include "protocol.h"
#include "stepper.h"
#include "foc_motor.h"

static void reply_ok()                 { Serial.println("OK"); }
static void reply_err(const char* why) { Serial.print("ERR:"); Serial.println(why); }

// ── MOVE,steps,dir,delay_ms ──
static void handle_move(const String& cmd) {
  int p1 = cmd.indexOf(',');
  int p2 = cmd.indexOf(',', p1 + 1);
  int p3 = cmd.indexOf(',', p2 + 1);
  if (p1 <= 0 || p2 <= 0 || p3 <= 0) { reply_err("bad format"); return; }

  int steps     = cmd.substring(p1 + 1, p2).toInt();
  int direction = cmd.substring(p2 + 1, p3).toInt();
  int delay_ms  = cmd.substring(p3 + 1).toInt();
  stepper_move(steps, direction, delay_ms);  // 末尾自带 println("OK")
}

// ── FOC,... ──
static void handle_foc(const String& cmd) {
  // cmd 格式: "FOC,<sub>[,<arg>]"
  int p1 = cmd.indexOf(',');                  // after "FOC"
  if (p1 <= 0) { reply_err("bad format"); return; }
  int p2 = cmd.indexOf(',', p1 + 1);          // may be -1 if no arg

  String sub = (p2 < 0) ? cmd.substring(p1 + 1) : cmd.substring(p1 + 1, p2);
  String arg = (p2 < 0) ? String("")          : cmd.substring(p2 + 1);

  if (sub == "S" && arg.length() == 0) {
    Serial.print("FOC,S,");
    Serial.print((int)foc_get_state()); Serial.print(',');
    Serial.print(foc_get_current_deg(), 1); Serial.print(',');
    Serial.print(foc_get_target_deg(), 1);   Serial.print(',');
    Serial.println(foc_is_fault_latched() ? 1 : 0);
    return;
  }
  if (sub == "EN" && arg.length() > 0) {
    int v = arg.toInt();
    if (v != 0 && v != 1) { reply_err("bad format"); return; }
    if (foc_request_enable(v == 1)) reply_ok();
    else reply_err("fault latched");
    return;
  }
  if (sub == "A" && arg.length() > 0) {
    float deg = arg.toFloat();
    if (foc_set_target_deg(deg)) reply_ok();
    else reply_err("out of range");
    return;
  }
  if (sub == "H" && arg.length() == 0) {
    foc_home(); reply_ok(); return;
  }
  if (sub == "CLR" && arg.length() == 0) {
    if (foc_clear_fault()) reply_ok();
    else reply_err("no fault");
    return;
  }
  if (sub == "V" && arg.length() > 0) {
    float v = arg.toFloat();
    if (foc_set_voltage_limit(v)) reply_ok();
    else reply_err("out of range");
    return;
  }
  if (sub == "PP" && arg.length() > 0) {
    int n = arg.toInt();
    if (foc_set_pole_pairs_and_store(n)) reply_ok();
    else reply_err("out of range");
    return;
  }
  if (sub == "PA" && arg.length() > 0) {
    float p = arg.toFloat();
    if (foc_set_p_angle(p)) reply_ok();
    else reply_err("out of range");
    return;
  }
  if (sub == "VP" && arg.length() > 0) {
    float p = arg.toFloat();
    if (foc_set_p_velocity(p)) reply_ok();
    else reply_err("out of range");
    return;
  }
  reply_err("bad format");
}

void protocol_handle_line(const String& cmd) {
  if (cmd.startsWith("MOVE,")) {
    handle_move(cmd);
  } else if (cmd == "DIAG") {
    stepper_run_diagnostics();
  } else if (cmd.startsWith("FOC,") || cmd == "FOC") {
    handle_foc(cmd);
  } else if (cmd.length() > 0) {
    Serial.println("ERR:unknown command");
  }
}
