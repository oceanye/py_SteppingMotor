// esp32_stepper/test/test_x42s_protocol/test_x42s_protocol.cpp
// X42S 帧层纯函数测试（§17.2）：帧编解码 / 校验和 / 错地址 / 错功能码 /
// 超时统计 / 主动上报 / 请求-应答匹配 / 多机打包。功能码具体数值不在
// 测试范围（待手册核对，见 x42s_protocol.h 核对点 2）。
#include <unity.h>

#include <string.h>

#include "../../src/x42s_protocol.h"

namespace {

// 按协议规则手工拼一帧应答，可注入校验破坏用于负例。
size_t build_reply(uint8_t* buf, uint8_t id, uint8_t fc, const uint8_t* payload,
                   size_t n, uint8_t checksum_bias = 0) {
    buf[0] = x42s::kFrameHeadReply;
    buf[1] = id;
    buf[2] = fc;
    if (n > 0) {
        memcpy(&buf[3], payload, n);
    }
    buf[3 + n] = static_cast<uint8_t>(
        x42s::checksum8(&buf[1], 2 + n) + checksum_bias);
    buf[4 + n] = x42s::kFrameTail;
    return x42s::kFrameOverhead + n;
}

}  // namespace

void setUp(void) {}
void tearDown(void) {}

void test_checksum8_accumulates_low_byte(void) {
    const uint8_t bytes[] = {0x02, 0x01, 0x1F};
    TEST_ASSERT_EQUAL_UINT8(0x22, x42s::checksum8(bytes, 3));
    TEST_ASSERT_EQUAL_UINT8(0, x42s::checksum8(bytes, 0));
    const uint8_t wrap[] = {0xFF, 0x03};
    TEST_ASSERT_EQUAL_UINT8(0x02, x42s::checksum8(wrap, 2));  // 回卷取低 8 位
}

void test_command_frame_layout(void) {
    const uint8_t payload[] = {0x11};
    uint8_t frame[x42s::kMaxFrameBytes] = {0};
    const size_t len = x42s::encode_unicast_command(
        0x02, 0x1F, payload, sizeof(payload), frame, sizeof(frame));
    TEST_ASSERT_EQUAL_size_t(6, len);
    TEST_ASSERT_EQUAL_UINT8(0x01, frame[0]);  // 命令帧头
    TEST_ASSERT_EQUAL_UINT8(0x02, frame[1]);  // 目标 ID
    TEST_ASSERT_EQUAL_UINT8(0x1F, frame[2]);  // 功能码
    TEST_ASSERT_EQUAL_UINT8(0x11, frame[3]);  // 数据
    TEST_ASSERT_EQUAL_UINT8(0x32, frame[4]);  // 02+1F+11
    TEST_ASSERT_EQUAL_UINT8(0x6B, frame[5]);  // 帧尾
}

void test_broadcast_restricted_to_explicit_encode(void) {
    uint8_t frame[x42s::kMaxFrameBytes] = {0};
    // 广播帧允许显式编码（全局触发/急停），单播入口必须拒绝 ID 0。
    TEST_ASSERT_EQUAL_size_t(
        5, x42s::encode_command(0x00, 0xF7, nullptr, 0, frame, sizeof(frame)));
    TEST_ASSERT_EQUAL_UINT8(0x00, frame[1]);
    TEST_ASSERT_EQUAL_size_t(
        0, x42s::encode_unicast_command(0x00, 0xF7, nullptr, 0, frame,
                                        sizeof(frame)));
    // 单播正常路径与上限边界 ID。
    TEST_ASSERT_EQUAL_size_t(
        5, x42s::encode_unicast_command(0x01, 0xF7, nullptr, 0, frame,
                                        sizeof(frame)));
    TEST_ASSERT_EQUAL_size_t(
        5, x42s::encode_unicast_command(0xFF, 0xF7, nullptr, 0, frame,
                                        sizeof(frame)));
}

void test_encode_capacity_and_length_guards(void) {
    uint8_t frame[x42s::kMaxFrameBytes] = {0};
    const uint8_t payload[] = {1, 2, 3};
    TEST_ASSERT_EQUAL_size_t(
        0, x42s::encode_command(0x01, 0x02, payload, 1, frame, 5));  // 容量不足
    TEST_ASSERT_EQUAL_size_t(
        0, x42s::encode_command(0x01, 0x02, nullptr, 1,
                                frame, sizeof(frame)));  // 有长度无数据
    TEST_ASSERT_EQUAL_size_t(
        0, x42s::encode_command(0x01, 0x02, payload, x42s::kMaxPayloadBytes + 1,
                                frame, sizeof(frame)));  // 超长
}

void test_decode_accepts_valid_reply(void) {
    const uint8_t payload[] = {0x0A, 0x0B};
    uint8_t frame[x42s::kMaxFrameBytes] = {0};
    const size_t len =
        build_reply(frame, 0x05, 0x36, payload, sizeof(payload));
    x42s::ReplyFrame reply{};
    TEST_ASSERT_EQUAL_INT(static_cast<int>(x42s::DecodeStatus::kOk),
                          static_cast<int>(x42s::decode_reply(frame, len, &reply)));
    TEST_ASSERT_EQUAL_UINT8(0x05, reply.driver_id);
    TEST_ASSERT_EQUAL_UINT8(0x36, reply.function);
    TEST_ASSERT_EQUAL_size_t(2, reply.payload_len);
    TEST_ASSERT_EQUAL_UINT8(0x0A, reply.payload[0]);
    TEST_ASSERT_EQUAL_UINT8(0x0B, reply.payload[1]);
}

void test_decode_rejects_each_corruption(void) {
    const uint8_t payload[] = {0x00};
    uint8_t frame[x42s::kMaxFrameBytes] = {0};
    size_t len = build_reply(frame, 0x03, 0x36, payload, sizeof(payload));

    // 过短
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(x42s::DecodeStatus::kTooShort),
        static_cast<int>(x42s::decode_reply(frame, 4, nullptr)));
    // 错帧头（命令帧头出现在应答流里 = 未知帧）
    frame[0] = x42s::kFrameHeadCommand;
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(x42s::DecodeStatus::kBadHead),
        static_cast<int>(x42s::decode_reply(frame, len, nullptr)));
    frame[0] = x42s::kFrameHeadReply;
    // 错帧尾
    frame[len - 1] = 0x00;
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(x42s::DecodeStatus::kBadTail),
        static_cast<int>(x42s::decode_reply(frame, len, nullptr)));
    frame[len - 1] = x42s::kFrameTail;
    // 完好帧此时应通过（负例前的健全性检查）
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(x42s::DecodeStatus::kOk),
        static_cast<int>(x42s::decode_reply(frame, len, nullptr)));
    // 校验和不符
    uint8_t broken[x42s::kMaxFrameBytes] = {0};
    build_reply(broken, 0x03, 0x36, payload, sizeof(payload), /*bias=*/1);
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(x42s::DecodeStatus::kBadChecksum),
        static_cast<int>(x42s::decode_reply(broken, len, nullptr)));
    // 应答地址为 0（广播地址不许出现在应答里）
    uint8_t broadcast[x42s::kMaxFrameBytes] = {0};
    size_t blen = build_reply(broadcast, 0x00, 0x36, payload, sizeof(payload));
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(x42s::DecodeStatus::kBadDriverId),
        static_cast<int>(x42s::decode_reply(broadcast, blen, nullptr)));
}

void test_reply_matching_requires_id_and_function(void) {
    const uint8_t payload[] = {0x01};
    uint8_t frame[x42s::kMaxFrameBytes] = {0};
    build_reply(frame, 0x07, 0x36, payload, sizeof(payload));
    x42s::ReplyFrame reply{};
    x42s::decode_reply(frame, x42s::kFrameOverhead + 1, &reply);
    TEST_ASSERT_TRUE(x42s::reply_matches(reply, 0x07, 0x36));
    TEST_ASSERT_FALSE(x42s::reply_matches(reply, 0x08, 0x36));  // 错地址
    TEST_ASSERT_FALSE(x42s::reply_matches(reply, 0x07, 0x32));  // 错功能码
}

void test_unsolicited_reached_frame_detection(void) {
    uint8_t frame[x42s::kMaxFrameBytes] = {0};
    x42s::ReplyFrame reply{};
    build_reply(frame, 0x07, x42s::kFcReachedNotify, nullptr, 0);
    x42s::decode_reply(frame, x42s::kFrameOverhead, &reply);
    TEST_ASSERT_TRUE(x42s::is_unsolicited(reply));
    build_reply(frame, 0x07, x42s::kFcReadPosition, nullptr, 0);
    x42s::decode_reply(frame, x42s::kFrameOverhead, &reply);
    TEST_ASSERT_FALSE(x42s::is_unsolicited(reply));
}

void test_pack_commands_concatenates_or_fails_whole(void) {
    const uint8_t p1[] = {0xAA};
    const uint8_t p2[] = {0xBB, 0xCC};
    const x42s::CommandSpec specs[] = {
        {0x01, x42s::kFcSyncCache, p1, sizeof(p1)},
        {0x02, x42s::kFcSyncCache, p2, sizeof(p2)},
    };
    uint8_t buffer[64] = {0};
    const size_t total = x42s::pack_commands(specs, 2, buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL_size_t(6 + 7, total);
    // 逐帧解回：第一帧 6 字节、第二帧 7 字节。
    x42s::ReplyFrame as_reply{};
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(x42s::DecodeStatus::kBadHead),
        static_cast<int>(x42s::decode_reply(buffer, 6, &as_reply)));
    // （命令帧头不会过应答解码——这里只验证打包边界与内容）
    TEST_ASSERT_EQUAL_UINT8(0x01, buffer[1]);
    TEST_ASSERT_EQUAL_UINT8(x42s::kFcSyncCache, buffer[2]);
    TEST_ASSERT_EQUAL_UINT8(0xAA, buffer[3]);
    // 第二帧自 buffer[6] 起：head, id, fc, BB, CC, chk, tail
    TEST_ASSERT_EQUAL_UINT8(0x01, buffer[6]);
    TEST_ASSERT_EQUAL_UINT8(0x02, buffer[7]);
    TEST_ASSERT_EQUAL_UINT8(x42s::kFcSyncCache, buffer[8]);
    TEST_ASSERT_EQUAL_UINT8(0xBB, buffer[9]);
    TEST_ASSERT_EQUAL_UINT8(0xCC, buffer[10]);
    // 容量不足 → 整体失败，不留半包
    TEST_ASSERT_EQUAL_size_t(0, x42s::pack_commands(specs, 2, buffer, 10));
}

void test_comm_stats_classify_failures_and_timeouts(void) {
    x42s::CommStats stats{};
    x42s::note_decode_failure(&stats, x42s::DecodeStatus::kBadChecksum);
    x42s::note_decode_failure(&stats, x42s::DecodeStatus::kBadChecksum);
    x42s::note_decode_failure(&stats, x42s::DecodeStatus::kBadHead);
    x42s::note_decode_failure(&stats, x42s::DecodeStatus::kTooShort);
    x42s::note_decode_failure(&stats, x42s::DecodeStatus::kBadDriverId);
    x42s::note_decode_failure(&stats, x42s::DecodeStatus::kOk);  // 不计数
    x42s::note_timeout(&stats);
    TEST_ASSERT_EQUAL_UINT32(2, stats.checksum_errors);
    TEST_ASSERT_EQUAL_UINT32(3, stats.mismatched_replies);
    TEST_ASSERT_EQUAL_UINT32(1, stats.timeouts);
    TEST_ASSERT_EQUAL_UINT32(0, stats.replies_matched);
}

int main(int /*argc*/, char** /*argv*/) {
    UNITY_BEGIN();
    RUN_TEST(test_checksum8_accumulates_low_byte);
    RUN_TEST(test_command_frame_layout);
    RUN_TEST(test_broadcast_restricted_to_explicit_encode);
    RUN_TEST(test_encode_capacity_and_length_guards);
    RUN_TEST(test_decode_accepts_valid_reply);
    RUN_TEST(test_decode_rejects_each_corruption);
    RUN_TEST(test_reply_matching_requires_id_and_function);
    RUN_TEST(test_unsolicited_reached_frame_detection);
    RUN_TEST(test_pack_commands_concatenates_or_fails_whole);
    RUN_TEST(test_comm_stats_classify_failures_and_timeouts);
    return UNITY_END();
}
