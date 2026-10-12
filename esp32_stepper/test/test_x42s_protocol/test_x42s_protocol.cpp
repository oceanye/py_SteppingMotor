// esp32_stepper/test/test_x42s_protocol/test_x42s_protocol.cpp
// X42S 帧层纯函数测试（§17.2）。帧结构/功能码/应答长度已按手册 V1.0.5
// 核对（2026-09-29）：命令帧=[Addr][FC][data][CHK 0x6B] 无帧头帧尾，
// 应答帧=[Addr][FC][返回数据][CHK]，触发类返回数据首字节=状态码，
// 读取类直接是数据。测试锁定的数值全部取自手册示例。
#include <unity.h>

#include <string.h>

#include "../../src/x42s_protocol.h"

namespace {

// 按协议规则手工拼一帧应答（默认校验 0x6B），可注入破坏用于负例。
size_t build_reply(uint8_t* buf, uint8_t id, uint8_t fc, const uint8_t* payload,
                   size_t n, bool break_checksum = false) {
    buf[0] = id;
    buf[1] = fc;
    if (n > 0) {
        memcpy(&buf[2], payload, n);
    }
    buf[2 + n] = break_checksum ? 0x00 : x42s::kChecksumFixed;
    return x42s::kFrameOverhead + n;
}

}  // namespace

void setUp(void) {}
void tearDown(void) {}

void test_command_frame_layout_manual_example(void) {
    // 手册 5.5.13：读实时位置命令 = 01 36 6B。
    uint8_t frame[x42s::kMaxFrameBytes] = {0};
    TEST_ASSERT_EQUAL_size_t(
        3, x42s::encode_unicast_command(0x01, x42s::kFcReadPosition, nullptr,
                                        0, frame, sizeof(frame)));
    TEST_ASSERT_EQUAL_UINT8(0x01, frame[0]);  // 地址
    TEST_ASSERT_EQUAL_UINT8(0x36, frame[1]);  // 功能码
    TEST_ASSERT_EQUAL_UINT8(0x6B, frame[2]);  // 校验（固定字节）
}

void test_command_frame_with_payload_manual_example(void) {
    // 手册 5.3.2：使能命令 = 01 F3 AB 01 00 6B。
    const uint8_t payload[] = {x42s::kAuxEnable, 0x01, 0x00};
    uint8_t frame[x42s::kMaxFrameBytes] = {0};
    const size_t len = x42s::encode_unicast_command(
        0x01, x42s::kFcEnable, payload, sizeof(payload), frame, sizeof(frame));
    TEST_ASSERT_EQUAL_size_t(6, len);
    const uint8_t expected[] = {0x01, 0xF3, 0xAB, 0x01, 0x00, 0x6B};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, frame, sizeof(expected));
}

void test_estop_best_effort_frame_bytes(void) {
    // 急停链 best-effort 帧（2026-10-12 修复配套）：逐台单播 FE98 停止 +
    // F3AB 失能、只发不等应答。字节按手册 5.3.15 / 5.3.2 锁定，常量漂移
    // （换固件/改辅助码）在此炸出，而不是到台架上才发现停不下来。
    const uint8_t stop_payload[] = {x42s::kAuxStop, 0x00};
    const uint8_t disable_payload[] = {x42s::kAuxEnable, 0x00, 0x00};
    uint8_t frame[x42s::kMaxFrameBytes] = {0};
    TEST_ASSERT_EQUAL_size_t(
        5, x42s::encode_unicast_command(0x01, x42s::kFcStop, stop_payload,
                                        sizeof(stop_payload), frame,
                                        sizeof(frame)));
    const uint8_t stop_frame[] = {0x01, 0xFE, 0x98, 0x00, 0x6B};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(stop_frame, frame, sizeof(stop_frame));
    TEST_ASSERT_EQUAL_size_t(
        6, x42s::encode_unicast_command(0x01, x42s::kFcEnable, disable_payload,
                                        sizeof(disable_payload), frame,
                                        sizeof(frame)));
    const uint8_t disable_frame[] = {0x01, 0xF3, 0xAB, 0x00, 0x00, 0x6B};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(disable_frame, frame, sizeof(disable_frame));
}

void test_broadcast_restricted_to_explicit_encode(void) {
    uint8_t frame[x42s::kMaxFrameBytes] = {0};
    // 广播帧允许显式编码（同步触发），单播入口必须拒绝 ID 0。
    const uint8_t payload[] = {x42s::kAuxSyncTrigger};
    TEST_ASSERT_EQUAL_size_t(
        4, x42s::encode_command(0x00, x42s::kFcSyncTrigger, payload,
                                sizeof(payload), frame, sizeof(frame)));
    TEST_ASSERT_EQUAL_UINT8(0x00, frame[0]);
    TEST_ASSERT_EQUAL_size_t(
        0, x42s::encode_unicast_command(0x00, x42s::kFcStop, nullptr, 0,
                                        frame, sizeof(frame)));
    // 单播正常路径与上限边界 ID。
    TEST_ASSERT_EQUAL_size_t(
        3, x42s::encode_unicast_command(0x01, x42s::kFcStop, nullptr, 0,
                                        frame, sizeof(frame)));
    TEST_ASSERT_EQUAL_size_t(
        3, x42s::encode_unicast_command(0xFF, x42s::kFcStop, nullptr, 0,
                                        frame, sizeof(frame)));
}

void test_encode_capacity_and_length_guards(void) {
    uint8_t frame[x42s::kMaxFrameBytes] = {0};
    const uint8_t payload[] = {1, 2, 3};
    TEST_ASSERT_EQUAL_size_t(
        0, x42s::encode_command(0x01, 0x36, payload, 1, frame, 3));  // 容量不足
    TEST_ASSERT_EQUAL_size_t(
        0, x42s::encode_command(0x01, 0x36, nullptr, 1,
                                frame, sizeof(frame)));  // 有长度无数据
    TEST_ASSERT_EQUAL_size_t(
        0, x42s::encode_command(0x01, 0x36, payload, x42s::kMaxPayloadBytes + 1,
                                frame, sizeof(frame)));  // 超长
}

void test_expected_reply_lengths_match_manual(void) {
    // 手册各命令字段表推得的应答帧总长（Addr+FC+data+CHK）。
    TEST_ASSERT_EQUAL_size_t(4, x42s::expected_reply_frame_len(x42s::kFcStop));
    TEST_ASSERT_EQUAL_size_t(4, x42s::expected_reply_frame_len(x42s::kFcEnable));
    TEST_ASSERT_EQUAL_size_t(4,
                             x42s::expected_reply_frame_len(x42s::kFcMoveTrap));
    // 01 3A 83 6B → 读取状态类 = 5 字节。
    TEST_ASSERT_EQUAL_size_t(5,
                             x42s::expected_reply_frame_len(x42s::kFcReadStatusPair));
    // 01 36 <符号><u32> 6B → 8 字节。
    TEST_ASSERT_EQUAL_size_t(8,
                             x42s::expected_reply_frame_len(x42s::kFcReadPosition));
    TEST_ASSERT_EQUAL_size_t(8,
                             x42s::expected_reply_frame_len(x42s::kFcReadTarget));
    TEST_ASSERT_EQUAL_size_t(8,
                             x42s::expected_reply_frame_len(x42s::kFcReadError));
    TEST_ASSERT_EQUAL_size_t(6,
                             x42s::expected_reply_frame_len(x42s::kFcReadSpeed));
    TEST_ASSERT_EQUAL_size_t(5,
                             x42s::expected_reply_frame_len(x42s::kFcReadTemp));
    TEST_ASSERT_EQUAL_size_t(5,
                             x42s::expected_reply_frame_len(x42s::kFcReadCurrent));
    // 01 1F <FW u16><HW u16> 6B → 7 字节（2026-10-10 实机修正：原 9 数错）。
    TEST_ASSERT_EQUAL_size_t(7,
                             x42s::expected_reply_frame_len(x42s::kFcReadVersion));
    TEST_ASSERT_EQUAL_size_t(4,
                             x42s::expected_reply_frame_len(x42s::kFcReadId));
    // 未知功能码：0 —— 无帧尾协议下无法断帧，禁止发起事务。
    TEST_ASSERT_EQUAL_size_t(0, x42s::expected_reply_frame_len(0x42));
}

void test_decode_accepts_valid_reply_manual_example(void) {
    // 手册 5.5.15：01 3A 83 6B（状态字 0x83）。
    const uint8_t frame[] = {0x01, 0x3A, 0x83, 0x6B};
    x42s::ReplyFrame reply{};
    TEST_ASSERT_EQUAL_INT(static_cast<int>(x42s::DecodeStatus::kOk),
                          static_cast<int>(x42s::decode_reply(
                              frame, sizeof(frame), &reply)));
    TEST_ASSERT_EQUAL_UINT8(0x01, reply.driver_id);
    TEST_ASSERT_EQUAL_UINT8(0x3A, reply.function);
    TEST_ASSERT_EQUAL_size_t(1, reply.payload_len);
    TEST_ASSERT_EQUAL_UINT8(0x83, reply.payload[0]);
}

void test_decode_rejects_each_corruption(void) {
    // 过短（不足 Addr+FC+CHK）。
    const uint8_t short_frame[] = {0x01, 0x36};
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(x42s::DecodeStatus::kTooShort),
        static_cast<int>(x42s::decode_reply(short_frame, sizeof(short_frame),
                                            nullptr)));
    // 校验字节非 0x6B（含驱动器被改设 XOR/CRC8 的情形）。
    uint8_t frame[x42s::kMaxFrameBytes] = {0};
    const uint8_t payload[] = {0x00, 0x00, 0x00, 0x10, 0x00};
    const size_t len = build_reply(frame, 0x03, x42s::kFcReadPosition, payload,
                                   sizeof(payload), /*break_checksum=*/true);
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(x42s::DecodeStatus::kBadChecksum),
        static_cast<int>(x42s::decode_reply(frame, len, nullptr)));
    // 应答地址为 0（广播地址不许出现在应答里）。
    uint8_t broadcast[x42s::kMaxFrameBytes] = {0};
    const size_t blen = build_reply(broadcast, 0x00, x42s::kFcReadStatusPair,
                                    nullptr, 0);
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(x42s::DecodeStatus::kBadDriverId),
        static_cast<int>(x42s::decode_reply(broadcast, blen, nullptr)));
}

void test_reply_matching_requires_id_and_function(void) {
    uint8_t frame[x42s::kMaxFrameBytes] = {0};
    const uint8_t payload[] = {0x02};
    build_reply(frame, 0x07, x42s::kFcStop, payload, sizeof(payload));
    x42s::ReplyFrame reply{};
    x42s::decode_reply(frame, 4, &reply);
    TEST_ASSERT_TRUE(x42s::reply_matches(reply, 0x07, x42s::kFcStop));
    TEST_ASSERT_FALSE(x42s::reply_matches(reply, 0x08, x42s::kFcStop));  // 错地址
    TEST_ASSERT_FALSE(x42s::reply_matches(reply, 0x07, x42s::kFcEnable));  // 错功能码
}

void test_unsolicited_and_ack_status_codes(void) {
    uint8_t frame[x42s::kMaxFrameBytes] = {0};
    x42s::ReplyFrame reply{};
    // 手册 4.2.2：到位主动上报 = Addr+FD+9F+CHK。
    const uint8_t reached[] = {x42s::kAckUnsolicited};
    build_reply(frame, 0x07, x42s::kFcMoveTrap, reached, sizeof(reached));
    x42s::decode_reply(frame, 4, &reply);
    TEST_ASSERT_TRUE(x42s::is_unsolicited(reply));
    TEST_ASSERT_FALSE(x42s::ack_accepted(reply));  // 9F 不是本次命令的确认
    // 02 = 命令正确。
    const uint8_t ok[] = {x42s::kAckOk};
    build_reply(frame, 0x07, x42s::kFcStop, ok, sizeof(ok));
    x42s::decode_reply(frame, 4, &reply);
    TEST_ASSERT_FALSE(x42s::is_unsolicited(reply));
    TEST_ASSERT_TRUE(x42s::ack_accepted(reply));
    // E2 = 参数错/保护触发；EE = 格式错 —— 都必须拒绝。
    const uint8_t rejected[] = {x42s::kAckRejected};
    build_reply(frame, 0x07, x42s::kFcEnable, rejected, sizeof(rejected));
    x42s::decode_reply(frame, 4, &reply);
    TEST_ASSERT_FALSE(x42s::ack_accepted(reply));
    const uint8_t bad_format[] = {x42s::kAckFormatErr};
    build_reply(frame, 0x07, x42s::kFcEnable, bad_format, sizeof(bad_format));
    x42s::decode_reply(frame, 4, &reply);
    TEST_ASSERT_FALSE(x42s::ack_accepted(reply));
    // 读取类应答无状态码前缀：首字节恰好是数据，不得当作主动上报误判——
    // 36 的符号字节 00/01 不是 9F，天然不冲突。
    const uint8_t position[] = {0x01, 0x00, 0x00, 0x0F, 0xA0};
    build_reply(frame, 0x07, x42s::kFcReadPosition, position, sizeof(position));
    x42s::decode_reply(frame, 8, &reply);
    TEST_ASSERT_FALSE(x42s::is_unsolicited(reply));
}

void test_read_parsers_follow_manual_layout(void) {
    // 36 实时位置：符号 01=负 + u32 大端。0x00000FA0=4000 → -400.0°。
    const uint8_t position[] = {0x01, 0x00, 0x00, 0x0F, 0xA0};
    int32_t value = 0;
    TEST_ASSERT_TRUE(x42s::parse_signed_u32(position, sizeof(position), &value));
    TEST_ASSERT_EQUAL_INT32(-4000, value);
    // 符号 00=正。
    const uint8_t positive[] = {0x00, 0x00, 0x00, 0x0F, 0xA0};
    TEST_ASSERT_TRUE(x42s::parse_signed_u32(positive, sizeof(positive), &value));
    TEST_ASSERT_EQUAL_INT32(4000, value);
    // 长度不足 → false（调用方保持 valid=false，不猜）。
    TEST_ASSERT_FALSE(x42s::parse_signed_u32(position, 4, &value));
    // 35 转速：符号 + u16。
    const uint8_t speed[] = {0x00, 0x01, 0x2C};
    TEST_ASSERT_TRUE(x42s::parse_signed_u16(speed, sizeof(speed), &value));
    TEST_ASSERT_EQUAL_INT32(300, value);  // 300 → 30.0RPM（调用方 /10）
    const uint8_t speed_neg[] = {0x01, 0x00, 0x64};
    TEST_ASSERT_TRUE(
        x42s::parse_signed_u16(speed_neg, sizeof(speed_neg), &value));
    TEST_ASSERT_EQUAL_INT32(-100, value);
    // 39 温度：符号 00=正 / 01=负——与其他命令一致（2026-10-10 实机修正，
    // 手册 5.5.12 的"相反"说法被实机证伪）。
    const uint8_t warm[] = {0x00, 0x2A};
    TEST_ASSERT_TRUE(x42s::parse_temp(warm, sizeof(warm), &value));
    TEST_ASSERT_EQUAL_INT32(42, value);
    const uint8_t cold[] = {0x01, 0x0A};
    TEST_ASSERT_TRUE(x42s::parse_temp(cold, sizeof(cold), &value));
    TEST_ASSERT_EQUAL_INT32(-10, value);
    // 26 总线电流：u16 mA。
    const uint8_t current[] = {0x03, 0x20};
    uint16_t ma = 0;
    TEST_ASSERT_TRUE(x42s::parse_u16(current, sizeof(current), &ma));
    TEST_ASSERT_EQUAL_UINT16(800, ma);  // 手册碰撞电流示例值
}

void test_pack_commands_concatenates_or_fails_whole(void) {
    const uint8_t p1[] = {0xAA};
    const uint8_t p2[] = {0xBB, 0xCC};
    const x42s::CommandSpec specs[] = {
        {0x01, x42s::kFcEnable, p1, sizeof(p1)},
        {0x02, x42s::kFcEnable, p2, sizeof(p2)},
    };
    uint8_t buffer[16] = {0};
    const size_t total = x42s::pack_commands(specs, 2, buffer, sizeof(buffer));
    // 每帧 Addr+FC+data+CHK：4 与 5 字节。
    TEST_ASSERT_EQUAL_size_t(9, total);
    TEST_ASSERT_EQUAL_UINT8(0x01, buffer[0]);
    TEST_ASSERT_EQUAL_UINT8(x42s::kFcEnable, buffer[1]);
    TEST_ASSERT_EQUAL_UINT8(0xAA, buffer[2]);
    TEST_ASSERT_EQUAL_UINT8(0x6B, buffer[3]);
    TEST_ASSERT_EQUAL_UINT8(0x02, buffer[4]);
    TEST_ASSERT_EQUAL_UINT8(x42s::kFcEnable, buffer[5]);
    TEST_ASSERT_EQUAL_UINT8(0xBB, buffer[6]);
    TEST_ASSERT_EQUAL_UINT8(0xCC, buffer[7]);
    TEST_ASSERT_EQUAL_UINT8(0x6B, buffer[8]);
    // 容量不足 → 整体失败，不留半包。
    TEST_ASSERT_EQUAL_size_t(0, x42s::pack_commands(specs, 2, buffer, 8));
}

void test_comm_stats_classify_failures_and_timeouts(void) {
    x42s::CommStats stats{};
    x42s::note_decode_failure(&stats, x42s::DecodeStatus::kBadChecksum);
    x42s::note_decode_failure(&stats, x42s::DecodeStatus::kBadChecksum);
    x42s::note_decode_failure(&stats, x42s::DecodeStatus::kTooShort);
    x42s::note_decode_failure(&stats, x42s::DecodeStatus::kBadDriverId);
    x42s::note_decode_failure(&stats, x42s::DecodeStatus::kOk);  // 不计数
    x42s::note_timeout(&stats);
    TEST_ASSERT_EQUAL_UINT32(2, stats.checksum_errors);
    TEST_ASSERT_EQUAL_UINT32(2, stats.mismatched_replies);
    TEST_ASSERT_EQUAL_UINT32(1, stats.timeouts);
    TEST_ASSERT_EQUAL_UINT32(0, stats.replies_matched);
}

void test_estimate_move_ms_matches_bench_reality(void) {
    // 运动挂起探测窗预估（2026-10-12 修复配套）。台架实测：输出轴 90°
    // = 17200 脉冲 @1600pps 实跑 10.8s；预估 10750 匀速 + 120 加减速。
    TEST_ASSERT_EQUAL_UINT32(
        10870, x42s::estimate_move_ms(17200, 1600, 3200, 500));
    // 首次运动验证：360° 整圈 3200 脉冲 @1600pps ≈ 2s。
    TEST_ASSERT_EQUAL_UINT32(
        2120, x42s::estimate_move_ms(3200, 1600, 3200, 500));
    // M1 验收步距：320 脉冲 @1600pps = 200ms 匀速 + 120 加减速。
    TEST_ASSERT_EQUAL_UINT32(
        320, x42s::estimate_move_ms(320, 1600, 3200, 500));
    // 非法参数与溢出保护 → 1h 上限（防探测窗 uint32 溢出）。
    TEST_ASSERT_EQUAL_UINT32(
        x42s::kMaxMoveEstimateMs, x42s::estimate_move_ms(100, 0, 3200, 500));
    TEST_ASSERT_EQUAL_UINT32(
        x42s::kMaxMoveEstimateMs,
        x42s::estimate_move_ms(0xFFFFFFFFu, 1, 3200, 500));
}

int main(int /*argc*/, char** /*argv*/) {
    UNITY_BEGIN();
    RUN_TEST(test_command_frame_layout_manual_example);
    RUN_TEST(test_command_frame_with_payload_manual_example);
    RUN_TEST(test_estop_best_effort_frame_bytes);
    RUN_TEST(test_broadcast_restricted_to_explicit_encode);
    RUN_TEST(test_encode_capacity_and_length_guards);
    RUN_TEST(test_expected_reply_lengths_match_manual);
    RUN_TEST(test_decode_accepts_valid_reply_manual_example);
    RUN_TEST(test_decode_rejects_each_corruption);
    RUN_TEST(test_reply_matching_requires_id_and_function);
    RUN_TEST(test_unsolicited_and_ack_status_codes);
    RUN_TEST(test_read_parsers_follow_manual_layout);
    RUN_TEST(test_pack_commands_concatenates_or_fails_whole);
    RUN_TEST(test_comm_stats_classify_failures_and_timeouts);
    RUN_TEST(test_estimate_move_ms_matches_bench_reality);
    return UNITY_END();
}
