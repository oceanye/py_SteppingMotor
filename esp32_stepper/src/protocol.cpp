// esp32_stepper/src/protocol.cpp
// 协议 v2.3+：全显式轴号。所有命令第一参数就是 axis（0=L, 1=R）。
//
//   MOVE,<axis>,<steps>,<dir>,<delay_us>   → ACK,<axis> / ERR:...
//                                            完成时异步 STEP,<axis>,DONE
//   DIAG,<axis>                            → 多行诊断输出
//   FOC,<axis>,EN,<0|1>                    → OK,<axis> / ERR:fault latched
//   FOC,<axis>,A,<deg>                     → OK,<axis>
//   FOC,<axis>,H                           → OK,<axis>
//   FOC,<axis>,CLR                         → OK,<axis> / ERR:no fault
//   FOC,<axis>,V,<volt>                    → OK,<axis>
//   FOC,<axis>,PA,<val>                    → OK,<axis>
//   FOC,<axis>,VP,<val>                    → OK,<axis>
//   FOC,<axis>,PP,<n>                      → OK,<axis> (NVS，需重启生效)
//   FOC,<axis>,S                           → FOC,<axis>,S,<state>,<cur>,<tgt>,<fault>
#include "protocol.h"
#include "stepper.h"
#include "foc_motor.h"

static void reply_ok_axis(int axis) { Serial.print("OK,"); Serial.println(axis); }
static void reply_err(const char* why) { Serial.print("ERR:"); Serial.println(why); }

// 把字符串分成以 ',' 分隔的 tokens，最多 6 个
static int split_tokens(const String& s, String tokens[], int max_n) {
  int n = 0, start = 0;
  while (start <= (int)s.length() && n < max_n) {
    int comma = s.indexOf(',', start);
    if (comma < 0) {
      tokens[n++] = s.substring(start);
      break;
    }
    tokens[n++] = s.substring(start, comma);
    start = comma + 1;
  }
  return n;
}

// ── MOVE,<axis>,<steps>,<dir>,<delay_us> ──
static void handle_move(const String tok[], int n) {
  if (n != 5) { reply_err("bad format"); return; }
  int axis     = tok[1].toInt();
  int steps    = tok[2].toInt();
  int dir      = tok[3].toInt();
  int delay_us = tok[4].toInt();
  if (stepper_move_async(axis, steps, dir, delay_us)) {
    Serial.print("ACK,"); Serial.println(axis);
  } else {
    reply_err("busy or bad axis");
  }
}

// ── DIAG,<axis> ──
static void handle_diag(const String tok[], int n) {
  if (n != 2) { reply_err("bad format"); return; }
  int axis = tok[1].toInt();
  stepper_run_diagnostics(axis);
}

// ── FOC,<axis>,<sub>[,<arg>] ──
static void handle_foc(const String tok[], int n) {
  if (n < 3) { reply_err("bad format"); return; }
  int axis = tok[1].toInt();
  const String& sub = tok[2];
  String arg = (n >= 4) ? tok[3] : String("");

  if (sub == "S" && arg.length() == 0) {
    Serial.print("FOC,"); Serial.print(axis); Serial.print(",S,");
    Serial.print((int)foc_get_state(axis)); Serial.print(',');
    Serial.print(foc_get_current_deg(axis), 1); Serial.print(',');
    Serial.print(foc_get_target_deg(axis), 1);   Serial.print(',');
    Serial.println(foc_is_fault_latched(axis) ? 1 : 0);
    return;
  }
  if (sub == "EN" && arg.length() > 0) {
    int v = arg.toInt();
    if (v != 0 && v != 1) { reply_err("bad format"); return; }
    if (foc_request_enable(axis, v == 1)) reply_ok_axis(axis);
    else reply_err("fault latched");
    return;
  }
  if (sub == "A" && arg.length() > 0) {
    if (foc_set_target_deg(axis, arg.toFloat())) reply_ok_axis(axis);
    else reply_err("out of range");
    return;
  }
  if (sub == "H" && arg.length() == 0) {
    foc_home(axis); reply_ok_axis(axis); return;
  }
  if (sub == "CLR" && arg.length() == 0) {
    if (foc_clear_fault(axis)) reply_ok_axis(axis);
    else reply_err("no fault");
    return;
  }
  if (sub == "V" && arg.length() > 0) {
    if (foc_set_voltage_limit(axis, arg.toFloat())) reply_ok_axis(axis);
    else reply_err("out of range");
    return;
  }
  if (sub == "PP" && arg.length() > 0) {
    if (foc_set_pole_pairs_and_store(axis, arg.toInt())) reply_ok_axis(axis);
    else reply_err("out of range");
    return;
  }
  if (sub == "PA" && arg.length() > 0) {
    if (foc_set_p_angle(axis, arg.toFloat())) reply_ok_axis(axis);
    else reply_err("out of range");
    return;
  }
  if (sub == "VP" && arg.length() > 0) {
    if (foc_set_p_velocity(axis, arg.toFloat())) reply_ok_axis(axis);
    else reply_err("out of range");
    return;
  }
  reply_err("bad format");
}

void protocol_handle_line(const String& cmd) {
  if (cmd.length() == 0) return;
  String tok[6];
  int n = split_tokens(cmd, tok, 6);
  if (tok[0] == "MOVE")      handle_move(tok, n);
  else if (tok[0] == "DIAG") handle_diag(tok, n);
  else if (tok[0] == "FOC")  handle_foc(tok, n);
  else Serial.println("ERR:unknown command");
}
