#include <unity.h>

#include <cstring>

#include "protocol.h"

using node_protocol::Command;
using node_protocol::CommandType;
using node_protocol::ParseResult;

static ParseResult parse(const char *input, Command &command, uint8_t node = 2) {
  char line[128];
  std::strncpy(line, input, sizeof(line));
  line[sizeof(line) - 1] = '\0';
  return node_protocol::parseLine(line, node, command);
}

void test_move() {
  Command command;
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseResult::Ok),
                        static_cast<int>(parse("N,2,MOVE,3,12000,1,80", command)));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandType::Move), static_cast<int>(command.type));
  TEST_ASSERT_EQUAL_INT(3, command.axis);
  TEST_ASSERT_EQUAL_UINT32(12000, command.steps);
  TEST_ASSERT_TRUE(command.direction);
  TEST_ASSERT_EQUAL_UINT32(80, command.delay_us);
}

void test_other_node_is_ignored() {
  Command command;
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseResult::Ignore),
                        static_cast<int>(parse("N,3,PING", command)));
}

void test_broadcast_safety_commands() {
  Command command;
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseResult::Ok),
                        static_cast<int>(parse("N,*,HEARTBEAT", command)));
  TEST_ASSERT_TRUE(command.broadcast);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandType::Heartbeat), static_cast<int>(command.type));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseResult::Ok),
                        static_cast<int>(parse("N,*,ESTOP\r\n", command)));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandType::Estop), static_cast<int>(command.type));
}

void test_poll_and_stop_all() {
  Command command;
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseResult::Ok),
                        static_cast<int>(parse("N,2,POLL", command)));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandType::Poll), static_cast<int>(command.type));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseResult::Ok),
                        static_cast<int>(parse("N,2,STOP,*", command)));
  TEST_ASSERT_EQUAL_INT(-1, command.axis);
}

void test_step_diagnostic() {
  Command command;
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseResult::Ok),
                        static_cast<int>(parse("N,2,STDIAG,1", command)));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandType::StepDiag), static_cast<int>(command.type));
  TEST_ASSERT_EQUAL_INT(1, command.axis);
}

void test_rejects_bad_values_and_broadcast_move() {
  Command command;
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseResult::BadAxis),
                        static_cast<int>(parse("N,2,MOVE,4,1,0,100", command)));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseResult::BadNumber),
                        static_cast<int>(parse("N,2,MOVE,0,0,0,100", command)));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseResult::BadDelay),
                        static_cast<int>(parse("N,2,MOVE,0,1,0,99", command)));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseResult::BadCommand),
                        static_cast<int>(parse("N,*,MOVE,0,1,0,100", command)));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseResult::BadCommand),
                        static_cast<int>(parse("N,2,HEARTBEAT", command)));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_move);
  RUN_TEST(test_other_node_is_ignored);
  RUN_TEST(test_broadcast_safety_commands);
  RUN_TEST(test_poll_and_stop_all);
  RUN_TEST(test_step_diagnostic);
  RUN_TEST(test_rejects_bad_values_and_broadcast_move);
  return UNITY_END();
}
