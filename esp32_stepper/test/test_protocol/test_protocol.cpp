// esp32_stepper/test/test_protocol/test_protocol.cpp
// Smoke test: verify unity native test env works.
// Task C1 将扩展本文件为完整的 protocol_parser 测试套件。
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

void test_smoke(void) {
    TEST_ASSERT_EQUAL_INT(2, 1 + 1);
}

int main(int /*argc*/, char ** /*argv*/) {
    UNITY_BEGIN();
    RUN_TEST(test_smoke);
    return UNITY_END();
}
