#include "remote_stepper.h"

#include "config.h"
#include "protocol_parse.h"
#include "remote_stepper_map.h"

#if defined(DRIVE_MODE_GEAR) && REMOTE_STEPPER_ENABLED

namespace {

HardwareSerial s_rs485(1);
bool s_busy[REMOTE_STEPPER_NODE_COUNT][REMOTE_STEPPER_AXES_PER_NODE] = {};
bool s_online[REMOTE_STEPPER_NODE_COUNT] = {};
uint32_t s_last_seen[REMOTE_STEPPER_NODE_COUNT] = {};
uint32_t s_last_heartbeat = 0;
uint32_t s_last_poll = 0;
uint8_t s_poll_node = 1;
char s_last_error[40] = "unavailable";

void set_error(const char* error) {
  strncpy(s_last_error, error, sizeof(s_last_error) - 1);
  s_last_error[sizeof(s_last_error) - 1] = '\0';
}

bool map_global(int global_axis, int& node, int& local_axis) {
  return remote_stepper_global_to_node(global_axis, NUM_AXES,
                                       REMOTE_STEPPER_NODE_COUNT,
                                       REMOTE_STEPPER_AXES_PER_NODE,
                                       node, local_axis);
}

void set_transmit(bool enabled) {
  digitalWrite(PIN_REMOTE_STEPPER_DE, enabled ? HIGH : LOW);
  delayMicroseconds(10);
}

void send_line(const String& line) {
  // No node is allowed to speak unless it has just been addressed. Broadcasts
  // never receive a response, which makes switching DE deterministic.
  set_transmit(true);
  s_rs485.print(line);
  s_rs485.print('\n');
  s_rs485.flush();
  set_transmit(false);
}

bool read_line(String& line, uint32_t timeout_ms) {
  line = "";
  const uint32_t started = millis();
  while ((uint32_t)(millis() - started) < timeout_ms) {
    while (s_rs485.available()) {
      const char c = (char)s_rs485.read();
      if (c == '\n') {
        line.trim();
        return line.length() > 0;
      }
      if (c != '\r' && line.length() < 159) line += c;
    }
    delay(1);
  }
  return false;
}

int split(const String& line, String tokens[], int capacity) {
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

bool parse_addressed(const String& line, int expected_node,
                     String tokens[], int capacity, int& count) {
  count = split(line, tokens, capacity);
  int node = 0;
  if (count < 3 || tokens[0] != "N" ||
      !protocol_parse_int(tokens[1].c_str(), 1,
                          REMOTE_STEPPER_NODE_COUNT, node) ||
      node != expected_node) return false;
  s_online[node - 1] = true;
  s_last_seen[node - 1] = millis();
  return true;
}

bool transact(int node, const String& request, String tokens[],
              int capacity, int& count) {
  // Discard any incomplete/stale bytes. Under the strict master/slave contract
  // a valid node cannot start a new response until this request is sent.
  while (s_rs485.available()) (void)s_rs485.read();
  send_line(request);
  String response;
  if (!read_line(response, REMOTE_STEPPER_RESPONSE_TIMEOUT_MS)) {
    set_error("node timeout");
    return false;
  }
  if (!parse_addressed(response, node, tokens, capacity, count)) {
    set_error("bad node response");
    return false;
  }
  if (tokens[2] == "ERR") {
    set_error(count >= 4 ? tokens[3].c_str() : "node error");
    return false;
  }
  return true;
}

void mark_node_offline(int node) {
  const int index = node - 1;
  if (!s_online[index]) return;
  s_online[index] = false;
  for (int local = 0; local < REMOTE_STEPPER_AXES_PER_NODE; ++local) {
    if (!s_busy[index][local]) continue;
    s_busy[index][local] = false;
    int global = 0;
    remote_stepper_node_to_global(node, local, NUM_AXES,
                                  REMOTE_STEPPER_NODE_COUNT,
                                  REMOTE_STEPPER_AXES_PER_NODE, global);
    Serial.print("STEP,"); Serial.print(global); Serial.println(",ABORT,0,0");
  }
  Serial.print("NODE,"); Serial.print(node); Serial.println(",OFFLINE");
}

void handle_poll(int node) {
  String tokens[8];
  int count = 0;
  const String request = String("N,") + node + ",POLL";
  if (!transact(node, request, tokens, 8, count)) return;
  if (count == 3 && tokens[2] == "EMPTY") return;
  if (count != 8 || tokens[2] != "EVENT" || tokens[3] != "STEP") {
    set_error("bad poll response");
    return;
  }
  int local = 0, completed = 0, total = 0, global = 0;
  if (!protocol_parse_int(tokens[4].c_str(), 0,
                          REMOTE_STEPPER_AXES_PER_NODE - 1, local) ||
      (tokens[5] != "P" && tokens[5] != "DONE" && tokens[5] != "ABORT") ||
      !protocol_parse_int(tokens[6].c_str(), 0, 20000000, completed) ||
      !protocol_parse_int(tokens[7].c_str(), 0, 20000000, total) ||
      !remote_stepper_node_to_global(node, local, NUM_AXES,
                                     REMOTE_STEPPER_NODE_COUNT,
                                     REMOTE_STEPPER_AXES_PER_NODE, global)) {
    set_error("bad step event");
    return;
  }
  if (tokens[5] == "DONE" || tokens[5] == "ABORT")
    s_busy[node - 1][local] = false;
  Serial.print("STEP,"); Serial.print(global); Serial.print(',');
  Serial.print(tokens[5]); Serial.print(','); Serial.print(completed);
  Serial.print(','); Serial.println(total);
}

}  // namespace

void remote_stepper_init() {
  pinMode(PIN_REMOTE_STEPPER_DE, OUTPUT);
  digitalWrite(PIN_REMOTE_STEPPER_DE, LOW);
  s_rs485.begin(REMOTE_STEPPER_BAUD, SERIAL_8N1,
                PIN_REMOTE_STEPPER_RX, PIN_REMOTE_STEPPER_TX);
  s_last_heartbeat = millis();
  s_last_poll = millis();
}

void remote_stepper_tick() {
  const uint32_t now = millis();
  if ((uint32_t)(now - s_last_heartbeat) >= REMOTE_STEPPER_HEARTBEAT_MS) {
    send_line("N,*,HEARTBEAT");
    s_last_heartbeat = now;
  }
  if ((uint32_t)(now - s_last_poll) >= REMOTE_STEPPER_POLL_INTERVAL_MS) {
    const int node = s_poll_node;
    s_poll_node = s_poll_node == REMOTE_STEPPER_NODE_COUNT ? 1 : s_poll_node + 1;
    s_last_poll = now;
    handle_poll(node);
  }
  for (int node = 1; node <= REMOTE_STEPPER_NODE_COUNT; ++node) {
    if (s_online[node - 1] &&
        (uint32_t)(now - s_last_seen[node - 1]) >= REMOTE_STEPPER_NODE_TIMEOUT_MS)
      mark_node_offline(node);
  }
}

bool remote_stepper_move(int global_axis, int steps, int direction, int delay_us) {
  int node = 0, local = 0;
  if (!map_global(global_axis, node, local)) { set_error("bad axis"); return false; }
  if (delay_us < 100) { set_error("remote delay below 100us"); return false; }
  if (s_busy[node - 1][local]) { set_error("busy"); return false; }
  String tokens[6];
  int count = 0;
  const String request = String("N,") + node + ",MOVE," + local + ',' +
                         steps + ',' + direction + ',' + delay_us;
  if (!transact(node, request, tokens, 6, count)) return false;
  int response_axis = -1;
  if (count != 4 || tokens[2] != "ACK" ||
      !protocol_parse_int(tokens[3].c_str(), 0,
                          REMOTE_STEPPER_AXES_PER_NODE - 1, response_axis) ||
      response_axis != local) {
    set_error("bad move response");
    return false;
  }
  s_busy[node - 1][local] = true;
  return true;
}

bool remote_stepper_stop(int global_axis) {
  int node = 0, local = 0;
  if (!map_global(global_axis, node, local)) { set_error("bad axis"); return false; }
  String tokens[6];
  int count = 0;
  const String request = String("N,") + node + ",STOP," + local;
  if (!transact(node, request, tokens, 6, count)) return false;
  int response_axis = -1;
  if (count != 4 || tokens[2] != "OK" ||
      !protocol_parse_int(tokens[3].c_str(), 0,
                          REMOTE_STEPPER_AXES_PER_NODE - 1, response_axis) ||
      response_axis != local) {
    set_error("bad stop response");
    return false;
  }
  // Keep busy set until the queued ABORT event arrives, preserving command /
  // event ordering at the PC interface.
  return true;
}

void remote_stepper_stop_all() {
  for (int node = 1; node <= REMOTE_STEPPER_NODE_COUNT; ++node) {
    String tokens[6];
    int count = 0;
    const String request = String("N,") + node + ",STOP,*";
    (void)transact(node, request, tokens, 6, count);
  }
}

bool remote_stepper_is_busy(int global_axis) {
  int node = 0, local = 0;
  return map_global(global_axis, node, local) && s_busy[node - 1][local];
}

bool remote_stepper_run_diagnostics(int global_axis) {
  int node = 0, local = 0;
  if (!map_global(global_axis, node, local)) { set_error("bad axis"); return false; }
  String tokens[6];
  int count = 0;
  const String request = String("N,") + node + ",STDIAG," + local;
  if (!transact(node, request, tokens, 6, count)) return false;
  return count == 4 && tokens[2] == "ACK";
}

void remote_stepper_estop() {
  send_line("N,*,ESTOP");
  for (int node = 0; node < REMOTE_STEPPER_NODE_COUNT; ++node)
    for (int local = 0; local < REMOTE_STEPPER_AXES_PER_NODE; ++local)
      s_busy[node][local] = false;
}

void remote_stepper_print_status(int requested_node) {
  const int first = requested_node == 0 ? 1 : requested_node;
  const int last = requested_node == 0 ? REMOTE_STEPPER_NODE_COUNT : requested_node;
  for (int node = first; node <= last; ++node) {
    String tokens[5];
    int count = 0;
    const String request = String("N,") + node + ",PING";
    const bool pong = transact(node, request, tokens, 5, count) &&
                      count == 3 && tokens[2] == "PONG";
    int busy_mask = 0;
    for (int local = 0; local < REMOTE_STEPPER_AXES_PER_NODE; ++local)
      if (s_busy[node - 1][local]) busy_mask |= 1 << local;
    Serial.print("NODE,"); Serial.print(node); Serial.print(",S,");
    Serial.print(pong ? "ONLINE" : "OFFLINE"); Serial.print(',');
    Serial.print(busy_mask); Serial.print(',');
    Serial.println(s_online[node - 1] ? millis() - s_last_seen[node - 1] : 0);
  }
}

const char* remote_stepper_last_error() { return s_last_error; }

#else

void remote_stepper_init() {}
void remote_stepper_tick() {}
bool remote_stepper_move(int, int, int, int) { return false; }
bool remote_stepper_stop(int) { return false; }
void remote_stepper_stop_all() {}
bool remote_stepper_is_busy(int) { return false; }
bool remote_stepper_run_diagnostics(int) { return false; }
void remote_stepper_estop() {}
void remote_stepper_print_status(int) {}
const char* remote_stepper_last_error() { return "remote steppers disabled"; }

#endif
