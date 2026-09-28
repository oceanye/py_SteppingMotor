// X42S TTL-UART 协议纯函数层（feature/x42s-uart-actuator-binding，2026-09-28）.
//
// 对应《REQUIREMENTS_2026-09-28_X42S_UART_ACTUATOR_BINDING》§8.3：本层只做
// 帧编解码、校验和、请求-应答匹配与多机打包，不持有串口、队列或定时器
// （那些归 x42s_bus / x42s_motion）。全部函数无动态分配、无 Arduino 依赖，
// header-only 纯函数——与 protocol_parse.h 等现有层一致，ESP32 固件与
// 主机 native 测试（pio test -e native_test）共用同一份实现。
//
// ┌──────────────────────────────────────────────────────────────────────┐
// │ ⚠️ 手册核对点（台架前必须完成）                                      │
// │ 1. 帧结构假设：命令帧 01 ID FC data... CHK 0x6B；                    │
// │    应答帧 02 ID FC data... CHK 0x6B；CHK = (ID+FC+data 逐字节累加    │
// │    和) & 0xFF。若与《ZDT X42S 用户手册 V1.0.5》不符，只需改本文件    │
// │    常量与 encode/decode 两处实现。                                   │
// │ 2. FunctionCode 枚举数值取自张大头/ZDT 系列公开资料，逐项标注        │
// │    "待核对"；帧结构与校验逻辑不依赖具体数值，核对后只改枚举表。      │
// │ 3. kMaxFrameBytes 为占位上限，按手册最长帧调整。                     │
// └──────────────────────────────────────────────────────────────────────┘
#pragma once

#include <stddef.h>
#include <stdint.h>

#include <string.h>

namespace x42s {

// ---- 帧结构常量（核对点 1 / 3）----
constexpr uint8_t kFrameHeadCommand = 0x01;  // 主机命令帧头
constexpr uint8_t kFrameHeadReply = 0x02;    // 驱动器应答帧头
constexpr uint8_t kFrameTail = 0x6B;         // 帧尾
constexpr uint8_t kBroadcastId = 0x00;       // 仅限广播（全局触发/急停）
constexpr uint8_t kDriverIdMin = 0x01;
constexpr uint8_t kDriverIdMax = 0xFF;
constexpr size_t kFrameOverhead = 5;  // head + id + fc + chk + tail
constexpr size_t kMaxFrameBytes = 32;  // 占位上限（核对点 3）
constexpr size_t kMaxPayloadBytes = kMaxFrameBytes - kFrameOverhead;

// ---- 功能码表（核对点 2：数值待手册逐项核对）----
enum FunctionCode : uint8_t {
  kFcPing = 0x1F,           // 探测在线 / 固件版本（待核对）
  kFcEnable = 0xF3,         // 使能 / 释放（待核对）
  kFcStop = 0xF7,           // 立即停机（待核对）
  kFcSetZero = 0x93,        // 设置零点（待核对）
  kFcMoveRelative = 0xFD,   // 相对位置运动（待核对）
  kFcReadPosition = 0x36,   // 读取实测位置（待核对）
  kFcReachedNotify = 0x42,  // 驱动器主动上报到位（待核对）
  kFcSyncCache = 0x20,      // 多机同步：缓存命令（待核对）
  kFcSyncTrigger = 0x21,    // 多机同步：广播触发（待核对）
};

// 解码结果分类。除 kOk 外一律不得当作应答消费（§8.3 严格校验）。
enum class DecodeStatus : uint8_t {
  kOk = 0,
  kTooShort,     // 不足最短帧长
  kBadHead,      // 帧头不是应答帧头（未知帧不当作回复）
  kBadTail,      // 帧尾不是 0x6B
  kBadChecksum,  // 累加和校验不符
  kBadDriverId,  // 应答地址为 0（驱动器不得用广播地址应答）
};

// 应答帧视图。payload 指向调用方输入缓冲区（零拷贝），仅在输入缓冲区
// 存活期间有效。
struct ReplyFrame {
  uint8_t driver_id;
  uint8_t function;
  const uint8_t* payload;
  size_t payload_len;
};

// 累加和校验：ID + 功能码 + 数据逐字节求和取低 8 位。
inline uint8_t checksum8(const uint8_t* bytes, size_t count) {
  uint8_t sum = 0;
  for (size_t i = 0; i < count; ++i) {
    sum = static_cast<uint8_t>(sum + bytes[i]);
  }
  return sum;
}

// 编码命令帧（允许广播地址——用于全局触发/急停）。容量不足或超长返回 0。
inline size_t encode_command(uint8_t driver_id, uint8_t function,
                             const uint8_t* payload, size_t payload_len,
                             uint8_t* out, size_t capacity) {
  if (out == nullptr || payload_len > kMaxPayloadBytes) {
    return 0;
  }
  if (payload_len > 0 && payload == nullptr) {
    return 0;
  }
  const size_t total = kFrameOverhead + payload_len;
  if (capacity < total) {
    return 0;
  }
  out[0] = kFrameHeadCommand;
  out[1] = driver_id;
  out[2] = function;
  if (payload_len > 0) {
    memcpy(&out[3], payload, payload_len);
  }
  // 校验覆盖：ID + 功能码 + 数据（与应答帧同一规则）。
  out[3 + payload_len] = checksum8(&out[1], 2 + payload_len);
  out[4 + payload_len] = kFrameTail;
  return total;
}

// 编码单播命令帧：driver_id==0 直接拒绝（§10 广播禁用于单轴动作）。
inline size_t encode_unicast_command(uint8_t driver_id, uint8_t function,
                                     const uint8_t* payload, size_t payload_len,
                                     uint8_t* out, size_t capacity) {
  if (driver_id == kBroadcastId || driver_id < kDriverIdMin ||
      driver_id > kDriverIdMax) {
    return 0;
  }
  return encode_command(driver_id, function, payload, payload_len, out,
                        capacity);
}

// 严格解码应答帧：帧头/帧尾/校验和/地址全查，任何一项不符返回错误分类。
inline DecodeStatus decode_reply(const uint8_t* frame, size_t len,
                                 ReplyFrame* out) {
  if (frame == nullptr || len < kFrameOverhead) {
    return DecodeStatus::kTooShort;
  }
  if (frame[0] != kFrameHeadReply) {
    return DecodeStatus::kBadHead;
  }
  if (frame[len - 1] != kFrameTail) {
    return DecodeStatus::kBadTail;
  }
  const size_t payload_len = len - kFrameOverhead;
  if (frame[1] == kBroadcastId) {
    return DecodeStatus::kBadDriverId;
  }
  const uint8_t expected = checksum8(&frame[1], 2 + payload_len);
  if (expected != frame[len - 2]) {
    return DecodeStatus::kBadChecksum;
  }
  if (out != nullptr) {
    out->driver_id = frame[1];
    out->function = frame[2];
    out->payload = (payload_len > 0) ? &frame[3] : nullptr;
    out->payload_len = payload_len;
  }
  return DecodeStatus::kOk;
}

// 请求-应答严格匹配：地址与功能码都一致才算本次请求的回复。
inline bool reply_matches(const ReplyFrame& reply, uint8_t expected_id,
                          uint8_t expected_function) {
  return reply.driver_id == expected_id && reply.function == expected_function;
}

// 是否驱动器主动上报帧（如到位通知）——不属于任何等待中的请求。
inline bool is_unsolicited(const ReplyFrame& reply) {
  return reply.function == kFcReachedNotify;
}

// 多机打包：若干命令帧连续填入同一缓冲，总线单 owner 一次写完。
// 任一帧编码失败则整体失败（返回 0，不留半包）。
struct CommandSpec {
  uint8_t driver_id;
  uint8_t function;
  const uint8_t* payload;
  size_t payload_len;
};
inline size_t pack_commands(const CommandSpec* specs, size_t count, uint8_t* out,
                            size_t capacity) {
  if (specs == nullptr || out == nullptr) {
    return 0;
  }
  size_t used = 0;
  for (size_t i = 0; i < count; ++i) {
    const size_t wrote = encode_command(
        specs[i].driver_id, specs[i].function, specs[i].payload,
        specs[i].payload_len, out + used, capacity - used);
    if (wrote == 0) {
      return 0;  // 整体失败，不留半包
    }
    used += wrote;
  }
  return used;
}

// 通讯统计（§8.3 通讯错误计数）。由总线层在超时/错配/校验失败各事件点
// 调用对应 note_* 归类累加；M2 扫描用它定位掉线台。
struct CommStats {
  uint32_t frames_sent;
  uint32_t replies_matched;
  uint32_t checksum_errors;
  uint32_t mismatched_replies;  // 错地址/错功能码——不得当作回复消费
  uint32_t unsolicited_frames;  // 主动上报（如到位），单独通道处理
  uint32_t timeouts;
};

inline void note_decode_failure(CommStats* stats, DecodeStatus status) {
  if (stats == nullptr) {
    return;
  }
  switch (status) {
    case DecodeStatus::kBadChecksum:
      ++stats->checksum_errors;
      return;
    case DecodeStatus::kOk:
      return;
    case DecodeStatus::kBadHead:
    case DecodeStatus::kBadTail:
    case DecodeStatus::kTooShort:
    case DecodeStatus::kBadDriverId:
    default:
      ++stats->mismatched_replies;
      return;
  }
}

inline void note_timeout(CommStats* stats) {
  if (stats != nullptr) {
    ++stats->timeouts;
  }
}

}  // namespace x42s
