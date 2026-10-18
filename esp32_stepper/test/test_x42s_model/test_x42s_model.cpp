// esp32_stepper/test/test_x42s_model/test_x42s_model.cpp
// X42S 遥测模型纯函数测试：新鲜度/掉线窗口、未知字段不回退 0、
// 故障标签映射。状态字位定义已按手册 V1.0.5 核对（5.4.4/5.5.15，
// 测试值取自手册示例：3A 状态字 0x83 / 3B 回零状态字 0x03）。
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

void test_probe_does_not_refresh_old_optional_values(void) {
    x42s::Telemetry t;
    x42s::telemetry_reset(t);
    x42s::telemetry_mark_seen(t, 1000);
    t.enabled = true;
    t.reached = true;
    t.command_valid = t.measured_valid = t.error_valid = true;
    t.speed_valid = t.current_valid = t.temperature_valid = true;
    t.measured_position = 123.0f;
    t.timeout_count = 2;
    x42s::telemetry_invalidate_values(t);
    TEST_ASSERT_FALSE(x42s::telemetry_is_fresh(t, 1000));
    TEST_ASSERT_FALSE(t.command_valid);
    TEST_ASSERT_FALSE(t.measured_valid);
    TEST_ASSERT_FALSE(t.error_valid);
    TEST_ASSERT_FALSE(t.speed_valid);
    TEST_ASSERT_FALSE(t.current_valid);
    TEST_ASSERT_FALSE(t.temperature_valid);
    // Status and diagnostics survive; validity, not zero, expresses unknown.
    TEST_ASSERT_TRUE(t.online);
    TEST_ASSERT_TRUE(t.enabled);
    TEST_ASSERT_TRUE(t.reached);
    TEST_ASSERT_EQUAL_FLOAT(123.0f, t.measured_position);
    TEST_ASSERT_EQUAL_UINT32(2, t.timeout_count);
    // A successful 36 restores only this new position's validity.
    t.measured_position = 215.0f;
    t.measured_valid = true;
    x42s::telemetry_mark_seen(t, 1100);
    TEST_ASSERT_TRUE(x42s::telemetry_is_fresh(t, 1100));
    TEST_ASSERT_FALSE(t.speed_valid);
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
    // 手册位定义（3C 应答 [回零状态][电机状态]）：过流/过热在回零状态字
    // （bit5/bit4），堵转在电机状态字（bit3 保护 / bit2 标志）；多位置位
    // 取安全优先：过流 > 过热 > 堵转。限位/掉电/编码器等是信息位不映射。
    TEST_ASSERT_EQUAL_INT((int)x42s::Fault::kNone,
                          (int)x42s::fault_from_status_words(0x03, 0x83));
    TEST_ASSERT_EQUAL_INT((int)x42s::Fault::kOverCurrent,
                          (int)x42s::fault_from_status_words(
                              x42s::kOrgOverCurrent, 0));
    TEST_ASSERT_EQUAL_INT((int)x42s::Fault::kOverTemp,
                          (int)x42s::fault_from_status_words(
                              x42s::kOrgOverTemp, 0));
    TEST_ASSERT_EQUAL_INT((int)x42s::Fault::kStall,
                          (int)x42s::fault_from_status_words(
                              0, x42s::kStatusStallProt));
    TEST_ASSERT_EQUAL_INT((int)x42s::Fault::kStall,
                          (int)x42s::fault_from_status_words(
                              0, x42s::kStatusStallFlag));
    // 过流+堵转同时置位：过流优先（保护类先报）。
    TEST_ASSERT_EQUAL_INT((int)x42s::Fault::kOverCurrent,
                          (int)x42s::fault_from_status_words(
                              x42s::kOrgOverCurrent, x42s::kStatusStallProt));
    // 回零中/回零失败/限位/掉电是信息位，不产生故障标签。
    TEST_ASSERT_EQUAL_INT((int)x42s::Fault::kNone,
                          (int)x42s::fault_from_status_words(
                              x42s::kOrgInProgress | x42s::kOrgFailed,
                              x42s::kStatusLimitLeft | x42s::kStatusPowerLost));
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

void test_status_word_bit_constants_match_manual(void) {
    // 手册 5.5.15 示例：状态字 0x83 = Oac_TF + Prf_TF + Ens_TF。
    const uint8_t word = 0x83;
    TEST_ASSERT_TRUE(word & x42s::kStatusEnabled);
    TEST_ASSERT_TRUE(word & x42s::kStatusReached);
    TEST_ASSERT_FALSE(word & x42s::kStatusStallFlag);
    // 手册 5.4.4 示例：回零状态字 0x03 = Cal_Rdy + Enc_Rdy。
    const uint8_t org = 0x03;
    TEST_ASSERT_TRUE(org & x42s::kOrgCalibrated);
    TEST_ASSERT_TRUE(org & x42s::kOrgEncoderReady);
    TEST_ASSERT_FALSE(org & x42s::kOrgInProgress);
}

int main(int /*argc*/, char ** /*argv*/) {
    UNITY_BEGIN();
    RUN_TEST(test_reset_clears_everything);
    RUN_TEST(test_freshness_window_matches_pc_contract);
    RUN_TEST(test_probe_does_not_refresh_old_optional_values);
    RUN_TEST(test_offline_detection_boundaries);
    RUN_TEST(test_status_word_bit_constants_match_manual);
    RUN_TEST(test_fault_priority_and_tags);
    return UNITY_END();
}
