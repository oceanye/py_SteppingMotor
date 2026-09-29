// esp32_stepper/test/test_x42s_model/test_x42s_model.cpp
// X42S 遥测模型纯函数测试：新鲜度/掉线窗口、未知字段不回退 0、
// 故障标签映射（位定义待手册核对，测试锁定当前假设的确定性行为）。
#include <unity.h>

#include <string.h>

#include "../../src/x42s_model.h"

void setUp(void) {}
void tearDown(void) {}

void test_reset_clears_everything(void) {
    x42s::Telemetry t;
    x42s::telemetry_reset(t);
    t.command_position = 123.0f;  // 预置脏值再 reset，确认全清
    t.measured_valid = true;
    x42s::telemetry_reset(t);
    TEST_ASSERT_FALSE(t.online);
    TEST_ASSERT_FALSE(t.enabled);
    TEST_ASSERT_FALSE(t.command_valid);
    TEST_ASSERT_FALSE(t.measured_valid);
    TEST_ASSERT_FALSE(t.error_valid);
    TEST_ASSERT_FALSE(t.speed_valid);
    TEST_ASSERT_FALSE(t.current_valid);
    TEST_ASSERT_FALSE(t.temperature_valid);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.command_position);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.measured_position);
    TEST_ASSERT_EQUAL_INT((int)x42s::Fault::kNone, (int)t.fault);
    TEST_ASSERT_EQUAL_UINT32(0, t.last_seen_ms);
    TEST_ASSERT_EQUAL_UINT32(0, t.timeout_count);
}

void test_freshness_window_matches_pc_contract(void) {
    // 窗口值必须与 PC 端 actuator_model 一致（200/500ms）。
    TEST_ASSERT_EQUAL_UINT32(200, x42s::kTelemetryFreshMs);
    TEST_ASSERT_EQUAL_UINT32(500, x42s::kTelemetryOfflineMs);

    x42s::Telemetry t;
    x42s::telemetry_reset(t);
    x42s::telemetry_mark_seen(t, 1000);
    t.measured_valid = true;
    TEST_ASSERT_TRUE(x42s::telemetry_is_fresh(t, 1200));
    TEST_ASSERT_FALSE(x42s::telemetry_is_fresh(t, 1201));  // 超 200ms 即不新鲜
    // 只有 measured 没有其他字段：新鲜性不依赖那些字段
    t.speed_valid = false;
    TEST_ASSERT_TRUE(x42s::telemetry_is_fresh(t, 1100));
    // 无实测位置（*_valid=false）不得视为可信——未知不等于 0
    t.measured_valid = false;
    TEST_ASSERT_FALSE(x42s::telemetry_is_fresh(t, 1100));
}

void test_offline_detection_boundaries(void) {
    x42s::Telemetry t;
    x42s::telemetry_reset(t);
    // 从未应答：不触发（区分"没上线过"与"掉线"）
    TEST_ASSERT_FALSE(x42s::telemetry_should_mark_offline(t, 10000));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, x42s::telemetry_age_ms(t, 10000));
    x42s::telemetry_mark_seen(t, 1000);
    TEST_ASSERT_FALSE(x42s::telemetry_should_mark_offline(t, 1500));
    TEST_ASSERT_TRUE(x42s::telemetry_should_mark_offline(t, 1501));
    // 调用方清 online 后不再重复触发
    t.online = false;
    TEST_ASSERT_FALSE(x42s::telemetry_should_mark_offline(t, 9999));
}

void test_fault_priority_and_tags(void) {
    // 位映射当前假设：bit0 stall, bit1 over_current, bit2 over_temp,
    // bit3 under_voltage, bit4 follow_error；多位置位取安全优先。
    TEST_ASSERT_EQUAL_INT((int)x42s::Fault::kNone,
                          (int)x42s::fault_from_status_word(0));
    TEST_ASSERT_EQUAL_INT((int)x42s::Fault::kStall,
                          (int)x42s::fault_from_status_word(0x0001));
    TEST_ASSERT_EQUAL_INT((int)x42s::Fault::kStall,
                          (int)x42s::fault_from_status_word(0x001F));  // 全置位
    TEST_ASSERT_EQUAL_INT((int)x42s::Fault::kOverCurrent,
                          (int)x42s::fault_from_status_word(0x0002));
    TEST_ASSERT_EQUAL_INT((int)x42s::Fault::kOverTemp,
                          (int)x42s::fault_from_status_word(0x0004));
    TEST_ASSERT_EQUAL_INT((int)x42s::Fault::kUnderVoltage,
                          (int)x42s::fault_from_status_word(0x0008));
    TEST_ASSERT_EQUAL_INT((int)x42s::Fault::kFollowError,
                          (int)x42s::fault_from_status_word(0x0010));
    // 标签必须与 PC 端 KNOWN_X42S_FAULTS 集合一致（§12 共用状态模型）
    TEST_ASSERT_EQUAL_STRING("stall", x42s::fault_tag(x42s::Fault::kStall));
    TEST_ASSERT_EQUAL_STRING("over_current",
                             x42s::fault_tag(x42s::Fault::kOverCurrent));
    TEST_ASSERT_EQUAL_STRING("over_temp",
                             x42s::fault_tag(x42s::Fault::kOverTemp));
    TEST_ASSERT_EQUAL_STRING("under_voltage",
                             x42s::fault_tag(x42s::Fault::kUnderVoltage));
    TEST_ASSERT_EQUAL_STRING("follow_error",
                             x42s::fault_tag(x42s::Fault::kFollowError));
    TEST_ASSERT_EQUAL_STRING("comm_error",
                             x42s::fault_tag(x42s::Fault::kCommError));
    TEST_ASSERT_EQUAL_STRING("addr_conflict",
                             x42s::fault_tag(x42s::Fault::kAddrConflict));
    TEST_ASSERT_EQUAL_STRING("none", x42s::fault_tag(x42s::Fault::kNone));
}

int main(int /*argc*/, char ** /*argv*/) {
    UNITY_BEGIN();
    RUN_TEST(test_reset_clears_everything);
    RUN_TEST(test_freshness_window_matches_pc_contract);
    RUN_TEST(test_offline_detection_boundaries);
    RUN_TEST(test_fault_priority_and_tags);
    return UNITY_END();
}
