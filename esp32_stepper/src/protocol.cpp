// esp32_stepper/src/protocol.cpp
#include "protocol.h"
#include "stepper.h"

static void handle_move(const String& cmd) {
  int p1 = cmd.indexOf(',');
  int p2 = cmd.indexOf(',', p1 + 1);
  int p3 = cmd.indexOf(',', p2 + 1);

  if (p1 > 0 && p2 > 0 && p3 > 0) {
    int steps     = cmd.substring(p1 + 1, p2).toInt();
    int direction = cmd.substring(p2 + 1, p3).toInt();
    int delay_ms  = cmd.substring(p3 + 1).toInt();
    stepper_move(steps, direction, delay_ms);  // 末尾自带 Serial.println("OK")
  } else {
    Serial.println("ERR:bad format");
  }
}

void protocol_handle_line(const String& cmd) {
  if (cmd.startsWith("MOVE,")) {
    handle_move(cmd);
  } else if (cmd == "DIAG") {
    stepper_run_diagnostics();
  } else if (cmd.length() > 0) {
    Serial.println("ERR:unknown command");
  }
}
