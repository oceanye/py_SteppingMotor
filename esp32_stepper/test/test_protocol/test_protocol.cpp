// esp32_stepper/test/test_protocol/test_protocol.cpp
#include <unity.h>
#include "../../src/protocol_parse.h"

void setUp(void) {}
void tearDown(void) {}

void test_integer_accepts_boundaries(void) {
    int value = -1;
    TEST_ASSERT_TRUE(protocol_parse_int("0", 0, 5, value));
    TEST_ASSERT_EQUAL_INT(0, value);
    TEST_ASSERT_TRUE(protocol_parse_int("5", 0, 5, value));
    TEST_ASSERT_EQUAL_INT(5, value);
}

void test_axis_rejects_invalid_tokens(void) {
    int value = 0;
    TEST_ASSERT_FALSE(protocol_parse_int("-1", 0, 5, value));
    TEST_ASSERT_FALSE(protocol_parse_int("6", 0, 5, value));
    TEST_ASSERT_FALSE(protocol_parse_int("axis0", 0, 5, value));
    TEST_ASSERT_FALSE(protocol_parse_int("0junk", 0, 5, value));
    TEST_ASSERT_FALSE(protocol_parse_int(" 0", 0, 5, value));
    TEST_ASSERT_FALSE(protocol_parse_int("", 0, 5, value));
}

void test_integer_rejects_overflow_and_decimal(void) {
    int value = 0;
    TEST_ASSERT_FALSE(protocol_parse_int("999999999999999999999", 0, 100, value));
    TEST_ASSERT_FALSE(protocol_parse_int("10.0", 0, 100, value));
}

void test_float_requires_finite_complete_value(void) {
    float value = 0.0f;
    TEST_ASSERT_TRUE(protocol_parse_float("-12.5", -20.0f, 20.0f, value));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -12.5f, value);
    TEST_ASSERT_FALSE(protocol_parse_float("nan", -20.0f, 20.0f, value));
    TEST_ASSERT_FALSE(protocol_parse_float("inf", -20.0f, 20.0f, value));
    TEST_ASSERT_FALSE(protocol_parse_float("1.2x", -20.0f, 20.0f, value));
    TEST_ASSERT_FALSE(protocol_parse_float("21", -20.0f, 20.0f, value));
}

void test_track_duty_and_lease_boundaries(void) {
    int value = 0;
    TEST_ASSERT_TRUE(protocol_parse_int("1", 1, 100, value));
    TEST_ASSERT_TRUE(protocol_parse_int("100", 1, 100, value));
    TEST_ASSERT_FALSE(protocol_parse_int("0", 1, 100, value));
    TEST_ASSERT_FALSE(protocol_parse_int("101", 1, 100, value));
    TEST_ASSERT_TRUE(protocol_parse_int("100", 100, 5000, value));
    TEST_ASSERT_TRUE(protocol_parse_int("5000", 100, 5000, value));
    TEST_ASSERT_FALSE(protocol_parse_int("99", 100, 5000, value));
    TEST_ASSERT_FALSE(protocol_parse_int("5001", 100, 5000, value));
}

void test_stop_and_track_field_counts(void) {
    TEST_ASSERT_EQUAL_INT(2, protocol_count_fields("STOP,0"));
    TEST_ASSERT_EQUAL_INT(1, protocol_count_fields("STOP"));
    TEST_ASSERT_EQUAL_INT(3, protocol_count_fields("STOP,0,unexpected"));
    TEST_ASSERT_EQUAL_INT(3, protocol_count_fields("TRACK,D,STOP"));
    TEST_ASSERT_EQUAL_INT(3, protocol_count_fields("TRACK,D,S"));
    TEST_ASSERT_EQUAL_INT(4, protocol_count_fields("TRACK,D,FWD,50"));
    TEST_ASSERT_EQUAL_INT(5, protocol_count_fields("TRACK,D,REV,50,1000"));
    TEST_ASSERT_EQUAL_INT(6, protocol_count_fields("TRACK,D,REV,50,1000,unexpected"));
}

int main(int /*argc*/, char ** /*argv*/) {
    UNITY_BEGIN();
    RUN_TEST(test_integer_accepts_boundaries);
    RUN_TEST(test_axis_rejects_invalid_tokens);
    RUN_TEST(test_integer_rejects_overflow_and_decimal);
    RUN_TEST(test_float_requires_finite_complete_value);
    RUN_TEST(test_track_duty_and_lease_boundaries);
    RUN_TEST(test_stop_and_track_field_counts);
    return UNITY_END();
}
