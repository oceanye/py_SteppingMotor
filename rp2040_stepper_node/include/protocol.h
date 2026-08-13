#pragma once

#include <stddef.h>
#include <stdint.h>

namespace node_protocol {

enum class CommandType : uint8_t {
  Invalid,
  Move,
  Stop,
  Ping,
  Poll,
  StepDiag,
  Estop,
  Heartbeat,
};

struct Command {
  CommandType type = CommandType::Invalid;
  bool broadcast = false;
  uint8_t node = 0;
  int8_t axis = -1;
  uint32_t steps = 0;
  bool direction = false;
  uint32_t delay_us = 0;
};

enum class ParseResult : uint8_t {
  Ok,
  Ignore,
  BadFrame,
  BadNode,
  BadCommand,
  BadAxis,
  BadNumber,
  BadArity,
  BadDelay,
};

// Parses one mutable, NUL-terminated line. Only frames addressed to local_node
// or '*' return Ok; frames for other nodes return Ignore.
ParseResult parseLine(char *line, uint8_t local_node, Command &out);
const char *parseResultName(ParseResult result);

}  // namespace node_protocol
