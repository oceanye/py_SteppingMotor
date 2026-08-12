#include <Arduino.h>
#include <hardware/clocks.h>
#include <hardware/pio.h>
#include <hardware/pio_instructions.h>

#include "protocol.h"

#ifndef NODE_ID
#define NODE_ID 1
#endif

static_assert(NODE_ID >= 1 && NODE_ID <= 6, "NODE_ID must be 1..6");

namespace {

constexpr uint8_t kAxisCount = 4;
constexpr uint8_t kPulsePins[kAxisCount] = {2, 4, 6, 8};
constexpr uint8_t kDirectionPins[kAxisCount] = {3, 5, 7, 9};
constexpr uint8_t kRs485DirectionPin = 10;
constexpr uint32_t kBaud = 115200;
constexpr uint32_t kPioClockHz = 1000000;
constexpr uint32_t kPulseWidthUs = 50;
constexpr uint32_t kHeartbeatTimeoutMs = 1000;
constexpr uint32_t kProgressIntervalMs = 250;
constexpr size_t kLineCapacity = 128;
constexpr size_t kEventCapacity = 24;

struct AxisState {
  bool moving = false;
  uint32_t requested = 0;
  uint32_t executed = 0;
  uint32_t period_us = 0;
  uint32_t started_us = 0;
  uint32_t last_progress_ms = 0;
};

enum class EventKind : uint8_t { Progress, Done, Abort };

struct Event {
  EventKind kind;
  uint8_t axis;
  uint32_t executed;
  uint32_t requested;
};

AxisState axes[kAxisCount];
Event events[kEventCapacity];
size_t event_count = 0;
char receive_line[kLineCapacity];
size_t receive_length = 0;
bool discarding_line = false;
uint32_t last_heartbeat_ms = 0;
bool heartbeat_seen = false;
PIO step_pio = pio0;
uint step_program_offset = 0;

// One state machine per axis. At 1 MHz, SET+NOP holds PUL high for 50 us.
// ISR stores the programmable low-loop count; Y stores pulses minus one.
const uint16_t step_program_instructions[] = {
    pio_encode_pull(false, true),
    pio_encode_out(pio_isr, 32),
    pio_encode_pull(false, true),
    pio_encode_out(pio_y, 32),
    static_cast<uint16_t>(pio_encode_set(pio_pins, 1) | pio_encode_delay(31)),
    static_cast<uint16_t>(pio_encode_nop() | pio_encode_delay(17)),
    pio_encode_set(pio_pins, 0),
    pio_encode_mov(pio_x, pio_isr),
    pio_encode_jmp_x_dec(8),
    pio_encode_jmp_y_dec(4),
    pio_encode_irq_set(true, 0),
};

const pio_program step_program = {
    step_program_instructions,
    sizeof(step_program_instructions) / sizeof(step_program_instructions[0]),
    0,  // Branch targets above are absolute; reserve instructions 0..10.
};

uint32_t estimateExecuted(const AxisState &state, uint32_t now_us) {
  const uint32_t estimate = (now_us - state.started_us) / state.period_us;
  return estimate < state.requested ? estimate : state.requested;
}

void removeEvent(size_t index) {
  for (size_t i = index + 1; i < event_count; ++i) events[i - 1] = events[i];
  --event_count;
}

void enqueueEvent(const Event &event) {
  // Progress is best effort: keep at most one queued sample per axis.
  if (event.kind == EventKind::Progress) {
    for (size_t i = 0; i < event_count; ++i) {
      if (events[i].kind == EventKind::Progress && events[i].axis == event.axis) {
        events[i] = event;
        return;
      }
    }
    if (event_count == kEventCapacity) return;
  } else if (event_count == kEventCapacity) {
    // Terminal events displace oldest progress first. If the queue contains
    // terminals only, displace the oldest terminal to remain bounded.
    size_t victim = 0;
    for (size_t i = 0; i < event_count; ++i) {
      if (events[i].kind == EventKind::Progress) {
        victim = i;
        break;
      }
    }
    removeEvent(victim);
  }
  events[event_count++] = event;
}

void setReceiveMode() {
  digitalWrite(kRs485DirectionPin, LOW);
}

void sendLine(const char *line) {
  digitalWrite(kRs485DirectionPin, HIGH);
  delayMicroseconds(5);
  Serial1.println(line);
  Serial1.flush();
  delayMicroseconds(20);
  setReceiveMode();
}

void sendAxisReply(const char *kind, int axis) {
  char line[48];
  if (axis < 0) snprintf(line, sizeof(line), "N,%d,%s,*", NODE_ID, kind);
  else snprintf(line, sizeof(line), "N,%d,%s,%d", NODE_ID, kind, axis);
  sendLine(line);
}

void stopAxis(uint8_t axis, bool terminal_event) {
  AxisState &state = axes[axis];
  if (!state.moving) return;
  state.executed = estimateExecuted(state, micros());
  pio_sm_set_enabled(step_pio, axis, false);
  pio_sm_clear_fifos(step_pio, axis);
  pio_sm_restart(step_pio, axis);
  pio_sm_exec(step_pio, axis, pio_encode_jmp(step_program_offset));
  digitalWrite(kPulsePins[axis], LOW);
  state.moving = false;
  if (terminal_event) {
    enqueueEvent({EventKind::Abort, axis, state.executed, state.requested});
  }
}

void stopAll(bool terminal_event) {
  for (uint8_t axis = 0; axis < kAxisCount; ++axis) stopAxis(axis, terminal_event);
}

void runAxes() {
  const uint32_t now_us = micros();
  const uint32_t now_ms = millis();
  for (uint8_t axis = 0; axis < kAxisCount; ++axis) {
    AxisState &state = axes[axis];
    if (!state.moving) continue;
    if (pio_interrupt_get(step_pio, axis)) {
      pio_interrupt_clear(step_pio, axis);
      pio_sm_set_enabled(step_pio, axis, false);
      state.executed = state.requested;
      state.moving = false;
      enqueueEvent({EventKind::Done, axis, state.executed, state.requested});
    } else if (now_ms - state.last_progress_ms >= kProgressIntervalMs) {
      state.last_progress_ms = now_ms;
      state.executed = estimateExecuted(state, now_us);
      enqueueEvent({EventKind::Progress, axis, state.executed, state.requested});
    }
  }
}

void pollEvent() {
  if (event_count == 0) {
    char line[24];
    snprintf(line, sizeof(line), "N,%d,EMPTY", NODE_ID);
    sendLine(line);
    return;
  }
  const Event event = events[0];
  removeEvent(0);
  const char *kind = event.kind == EventKind::Progress ? "P" :
                     event.kind == EventKind::Done ? "DONE" : "ABORT";
  char line[80];
  snprintf(line, sizeof(line), "N,%d,EVENT,STEP,%u,%s,%lu,%lu", NODE_ID,
           event.axis, kind, static_cast<unsigned long>(event.executed),
           static_cast<unsigned long>(event.requested));
  sendLine(line);
}

void handleLine(char *line) {
  char local_prefix[12];
  snprintf(local_prefix, sizeof(local_prefix), "N,%d,", NODE_ID);
  const bool addressed_to_local =
      strncmp(line, local_prefix, strlen(local_prefix)) == 0;
  node_protocol::Command command;
  const node_protocol::ParseResult parsed =
      node_protocol::parseLine(line, NODE_ID, command);
  if (parsed == node_protocol::ParseResult::Ignore) return;
  if (parsed != node_protocol::ParseResult::Ok) {
    // Only an unambiguously local frame may produce an error response. An
    // invalid address or malformed broadcast must remain silent, otherwise
    // every node could drive the half-duplex bus simultaneously.
    if (addressed_to_local) {
      char reply[48];
      snprintf(reply, sizeof(reply), "N,%d,ERR,%s", NODE_ID,
               node_protocol::parseResultName(parsed));
      sendLine(reply);
    }
    return;
  }

  switch (command.type) {
    case node_protocol::CommandType::Heartbeat:
      heartbeat_seen = true;
      last_heartbeat_ms = millis();
      return;  // Broadcasts never respond.
    case node_protocol::CommandType::Estop:
      stopAll(true);
      return;  // Broadcasts never respond.
    case node_protocol::CommandType::Ping: {
      char reply[24];
      snprintf(reply, sizeof(reply), "N,%d,PONG", NODE_ID);
      sendLine(reply);
      return;
    }
    case node_protocol::CommandType::Poll:
      pollEvent();
      return;
    case node_protocol::CommandType::StepDiag:
      sendAxisReply("ACK", command.axis);
      return;
    case node_protocol::CommandType::Stop:
      if (command.axis < 0) stopAll(true);
      else stopAxis(static_cast<uint8_t>(command.axis), true);
      sendAxisReply("OK", command.axis);
      return;
    case node_protocol::CommandType::Move: {
      const uint32_t now_ms = millis();
      if (!heartbeat_seen || now_ms - last_heartbeat_ms > kHeartbeatTimeoutMs) {
        char reply[32];
        snprintf(reply, sizeof(reply), "N,%d,ERR,NO_HEARTBEAT", NODE_ID);
        sendLine(reply);
        return;
      }
      AxisState &state = axes[command.axis];
      if (state.moving) {
        char reply[32];
        snprintf(reply, sizeof(reply), "N,%d,ERR,AXIS_BUSY", NODE_ID);
        sendLine(reply);
        return;
      }
      digitalWrite(kDirectionPins[command.axis], command.direction ? HIGH : LOW);
      pio_sm_set_enabled(step_pio, command.axis, false);
      pio_sm_clear_fifos(step_pio, command.axis);
      pio_sm_restart(step_pio, command.axis);
      pio_sm_exec(step_pio, command.axis, pio_encode_jmp(step_program_offset));
      pio_interrupt_clear(step_pio, command.axis);
      state.moving = true;
      state.requested = command.steps;
      state.executed = 0;
      state.period_us = command.delay_us;
      state.started_us = micros();
      state.last_progress_ms = now_ms;
      // Program overhead is 54 cycles per period (50 high, 4 fixed low).
      pio_sm_put_blocking(step_pio, command.axis, command.delay_us - 54);
      pio_sm_put_blocking(step_pio, command.axis, command.steps - 1);
      pio_sm_set_enabled(step_pio, command.axis, true);
      sendAxisReply("ACK", command.axis);
      return;
    }
    default:
      return;
  }
}

void readBus() {
  while (Serial1.available() > 0) {
    const char byte = static_cast<char>(Serial1.read());
    if (byte == '\n') {
      if (!discarding_line && receive_length > 0) {
        receive_line[receive_length] = '\0';
        handleLine(receive_line);
      }
      receive_length = 0;
      discarding_line = false;
    } else if (!discarding_line && byte != '\r') {
      if (receive_length + 1 < kLineCapacity) receive_line[receive_length++] = byte;
      else discarding_line = true;
    }
  }
}

}  // namespace

void setup() {
  pinMode(kRs485DirectionPin, OUTPUT);
  setReceiveMode();
  step_program_offset = pio_add_program(step_pio, &step_program);
  for (uint8_t axis = 0; axis < kAxisCount; ++axis) {
    pinMode(kPulsePins[axis], OUTPUT);
    pinMode(kDirectionPins[axis], OUTPUT);
    digitalWrite(kPulsePins[axis], LOW);
    digitalWrite(kDirectionPins[axis], LOW);
    pio_gpio_init(step_pio, kPulsePins[axis]);
    pio_sm_set_consecutive_pindirs(step_pio, axis, kPulsePins[axis], 1, true);
    pio_sm_config config = pio_get_default_sm_config();
    sm_config_set_wrap(&config, step_program_offset,
                       step_program_offset + step_program.length - 1);
    sm_config_set_set_pins(&config, kPulsePins[axis], 1);
    sm_config_set_clkdiv(&config,
                         static_cast<float>(clock_get_hz(clk_sys)) / kPioClockHz);
    pio_sm_init(step_pio, axis, step_program_offset, &config);
    pio_sm_set_enabled(step_pio, axis, false);
  }
  Serial1.begin(kBaud);  // Pico Serial1: UART0 TX=GP0, RX=GP1.
  last_heartbeat_ms = millis();
}

void loop() {
  readBus();
  runAxes();
  if (heartbeat_seen && millis() - last_heartbeat_ms > kHeartbeatTimeoutMs) {
    stopAll(true);
    heartbeat_seen = false;
  }
}
