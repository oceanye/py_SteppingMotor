#include "protocol.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>

namespace node_protocol {
namespace {

constexpr size_t kMaxFields = 8;
constexpr uint32_t kMinDelayUs = 100;
constexpr uint32_t kMaxDelayUs = 2000000;

bool parseU32(const char *text, uint32_t &value) {
  if (text == nullptr || *text == '\0' || *text == '-') return false;
  errno = 0;
  char *end = nullptr;
  const unsigned long parsed = strtoul(text, &end, 10);
  if (errno == ERANGE || end == text || *end != '\0' ||
      parsed > UINT32_MAX) {
    return false;
  }
  value = static_cast<uint32_t>(parsed);
  return true;
}

size_t split(char *line, char **fields) {
  size_t count = 0;
  char *cursor = line;
  while (cursor != nullptr && count < kMaxFields) {
    fields[count++] = cursor;
    char *comma = strchr(cursor, ',');
    if (comma == nullptr) break;
    *comma = '\0';
    cursor = comma + 1;
  }
  // More than kMaxFields is always malformed.
  if (cursor != nullptr && count == kMaxFields && strchr(cursor, ',') != nullptr) {
    return kMaxFields + 1;
  }
  return count;
}

}  // namespace

ParseResult parseLine(char *line, uint8_t local_node, Command &out) {
  out = Command{};
  if (line == nullptr || local_node < 1 || local_node > 6) {
    return ParseResult::BadFrame;
  }

  const size_t length = strlen(line);
  while (length > 0 && (line[strlen(line) - 1] == '\r' || line[strlen(line) - 1] == '\n')) {
    line[strlen(line) - 1] = '\0';
  }

  char *fields[kMaxFields] = {};
  const size_t count = split(line, fields);
  if (count < 3 || count > kMaxFields || strcmp(fields[0], "N") != 0) {
    return ParseResult::BadFrame;
  }

  if (strcmp(fields[1], "*") == 0) {
    out.broadcast = true;
    out.node = local_node;
  } else {
    uint32_t node = 0;
    if (!parseU32(fields[1], node) || node < 1 || node > 6) {
      return ParseResult::BadNode;
    }
    if (node != local_node) return ParseResult::Ignore;
    out.node = static_cast<uint8_t>(node);
  }

  if (strcmp(fields[2], "MOVE") == 0) {
    if (out.broadcast) return ParseResult::BadCommand;
    if (count != 7) return ParseResult::BadArity;
    uint32_t axis = 0, steps = 0, direction = 0, delay = 0;
    if (!parseU32(fields[3], axis) || axis > 3) return ParseResult::BadAxis;
    if (!parseU32(fields[4], steps) || steps == 0) return ParseResult::BadNumber;
    if (!parseU32(fields[5], direction) || direction > 1) return ParseResult::BadNumber;
    if (!parseU32(fields[6], delay)) return ParseResult::BadNumber;
    if (delay < kMinDelayUs || delay > kMaxDelayUs) return ParseResult::BadDelay;
    out.type = CommandType::Move;
    out.axis = static_cast<int8_t>(axis);
    out.steps = steps;
    out.direction = direction != 0;
    out.delay_us = delay;
    return ParseResult::Ok;
  }

  if (strcmp(fields[2], "STOP") == 0) {
    if (out.broadcast) return ParseResult::BadCommand;
    if (count != 4) return ParseResult::BadArity;
    if (strcmp(fields[3], "*") == 0) {
      out.axis = -1;
    } else {
      uint32_t axis = 0;
      if (!parseU32(fields[3], axis) || axis > 3) return ParseResult::BadAxis;
      out.axis = static_cast<int8_t>(axis);
    }
    out.type = CommandType::Stop;
    return ParseResult::Ok;
  }

  if (strcmp(fields[2], "PING") == 0 || strcmp(fields[2], "POLL") == 0) {
    if (out.broadcast) return ParseResult::BadCommand;
    if (count != 3) return ParseResult::BadArity;
    out.type = strcmp(fields[2], "PING") == 0 ? CommandType::Ping : CommandType::Poll;
    return ParseResult::Ok;
  }

  if (strcmp(fields[2], "STDIAG") == 0) {
    if (out.broadcast) return ParseResult::BadCommand;
    if (count != 4) return ParseResult::BadArity;
    uint32_t axis = 0;
    if (!parseU32(fields[3], axis) || axis > 3) return ParseResult::BadAxis;
    out.axis = static_cast<int8_t>(axis);
    out.type = CommandType::StepDiag;
    return ParseResult::Ok;
  }

  if (strcmp(fields[2], "ESTOP") == 0 || strcmp(fields[2], "HEARTBEAT") == 0) {
    if (!out.broadcast) return ParseResult::BadCommand;
    if (count != 3) return ParseResult::BadArity;
    out.type = strcmp(fields[2], "ESTOP") == 0 ? CommandType::Estop : CommandType::Heartbeat;
    return ParseResult::Ok;
  }

  return ParseResult::BadCommand;
}

const char *parseResultName(ParseResult result) {
  switch (result) {
    case ParseResult::Ok: return "OK";
    case ParseResult::Ignore: return "IGNORE";
    case ParseResult::BadFrame: return "BAD_FRAME";
    case ParseResult::BadNode: return "BAD_NODE";
    case ParseResult::BadCommand: return "BAD_COMMAND";
    case ParseResult::BadAxis: return "BAD_AXIS";
    case ParseResult::BadNumber: return "BAD_NUMBER";
    case ParseResult::BadArity: return "BAD_ARITY";
    case ParseResult::BadDelay: return "BAD_DELAY";
  }
  return "BAD_FRAME";
}

}  // namespace node_protocol
