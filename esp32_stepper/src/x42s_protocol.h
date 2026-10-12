// X42S TTL-UART 协议纯函数层（feature/x42s-uart-actuator-binding，2026-09-28）.
//
// 对应《REQUIREMENTS_2026-09-28_X42S_UART_ACTUATOR_BINDING》§8.3：本层只做
// 帧编解码、校验、请求-应答匹配与多机打包，不持有串口、队列或定时器
// （那些归 x42s_uart / 主循环）。全部函数无动态分配、无 Arduino 依赖，
// header-only 纯函数——与 protocol_parse.h 等现有层一致，ESP32 固件与
// 主机 native 测试（pio test -e native_test）共用同一份实现。
//
// ✅ 已按《ZDT X42S第二代闭环步进电机用户手册 V1.0.5_260527》核对（2026-09-29）：
//  - 命令帧 = [Addr][功能码][命令数据][校验码]，无帧头/帧尾（手册 4.1/5.x 示例）；
//  - 应答帧 = [Addr][功能码][返回数据][校验码]；
//  - 校验码出厂默认 = 固定字节 0x6B（手册 7.1 附录；驱动器若改设 XOR/CRC8/
//    Modbus 则本层校验全部失败，需扩展 checksum 计算后才能用）；
//  - 触发类命令的返回数据首字节为应答状态码：02=正确，E2=参数错误或
//    低压/堵转/过流过热保护触发，EE=格式错误（手册 4.2.2，UART 同语义）；
//    12/22 仅回零命令 9A：已在零点/限位不动；
//  - 读取类命令的返回数据直接是数据，无状态码前缀（如 01 3A 83 6B）；
//  - 9F = 动作完成主动上报（到位=Addr+FD+9F+CHK；回零完成=Addr+9A+9F+CHK），
//    受驱动器“控制命令应答”参数控制，可能被关闭——不得依赖，仅作加速信号；
//  - 无帧尾标记 → 接收侧按“期望应答长度”断帧（expected_reply_frame_len）。
#pragma once

#include <stddef.h>
#include <stdint.h>

#include <string.h>

namespace x42s {

// ---- 帧结构常量（手册 4.1 / 7.1 已核对）----
constexpr uint8_t kChecksumFixed = 0x6B;  // 校验码出厂默认：固定字节
constexpr uint8_t kBroadcastId = 0x00;    // 仅限广播（同步触发/急停，§10）
constexpr uint8_t kDriverIdMin = 0x01;
constexpr uint8_t kDriverIdMax = 0xFF;
constexpr size_t kFrameOverhead = 3;      // Addr + FC + CHK
constexpr size_t kMinFrameBytes = kFrameOverhead;          // 无数据应答
constexpr size_t kMaxFrameBytes = 32;
constexpr size_t kMaxPayloadBytes = kMaxFrameBytes - kFrameOverhead;

// ---- 功能码表（手册第 5 章已核对，X42S= X 固件）----
enum FunctionCode : uint8_t {
  kFcReadId = 0x15,        // 5.6.30 广播读取ID地址（发 00 15 6B，应答报真实ID）
  kFcReadVersion = 0x1F,   // 5.5.2 读固件/硬件版本（兼作 ping 在线探测）
  kFcReadCurrent = 0x26,   // 5.5.5 读总线电流 mA（X42S）
  kFcReadSpeed = 0x35,     // 5.5.11 读实时转速 0.1RPM
  kFcReadPosition = 0x36,  // 5.5.13 读实时位置 0.1°（X 固件，符号+u32）
  kFcReadTarget = 0x33,    // 5.5.9 读目标位置 0.1°（X 固件；0x32 是输入脉冲数）
  kFcReadError = 0x37,     // 5.5.14 读位置误差 0.01°（X 固件）
  kFcReadTemp = 0x39,      // 5.5.12 读驱动温度 ℃（X42S）
  kFcReadStatusPair = 0x3C,  // 5.5.16 读回零+电机状态两字节（X42S）
  kFcEnable = 0xF3,        // 5.3.2 使能控制（辅助码 AB）
  kFcMoveTrap = 0xFD,      // 5.3.10 X 固件梯形位置模式（角度制）
  kFcStop = 0xFE,          // 5.3.15 立即停止（辅助码 98）
  kFcSyncTrigger = 0xFF,   // 5.3.16 多机同步触发（辅助码 66，广播发送）
  kFcSetZero = 0x93,       // 5.4.1 设单圈回零零点（辅助码 88）
};

// ---- 辅助码 / 应答状态码（手册 5.x / 4.2.2）----
constexpr uint8_t kAuxEnable = 0xAB;      // F3 使能命令辅助码
constexpr uint8_t kAuxStop = 0x98;        // FE 停止命令辅助码
constexpr uint8_t kAuxSetZero = 0x88;     // 93 设零点命令辅助码
constexpr uint8_t kAuxSyncTrigger = 0x66; // FF 同步触发命令辅助码
constexpr uint8_t kAckOk = 0x02;          // 触发类应答：命令正确
constexpr uint8_t kAckHomingAtZero = 0x12;  // 仅 9A：已在零点不动
constexpr uint8_t kAckHomingAtLimit = 0x22; // 仅 9A：已在限位不动
constexpr uint8_t kAckRejected = 0xE2;    // 参数错误 / 保护触发（不可执行）
constexpr uint8_t kAckFormatErr = 0xEE;   // 格式错误
constexpr uint8_t kAckUnsolicited = 0x9F; // 动作完成主动上报

// 各功能码的应答帧总长（Addr+FC+返回数据+CHK）。未知功能码返回 0——
// 调用方不得对未知长度功能码发起事务（无帧尾协议下无法断帧）。
inline size_t expected_reply_frame_len(uint8_t function) {
  switch (function) {
    // 触发类：返回数据只有 1 字节状态码。
    case kFcReadId:          // 5.6.30 应答 [Addr][15][Addr]（ID 报两遍）
    case kFcEnable:
    case kFcMoveTrap:
    case kFcStop:
    case kFcSyncTrigger:
    case kFcSetZero:
      return 4;
    // 读取类：返回数据长度按手册字段表。
    case kFcReadVersion:      // 5.5.2 FW_Ver u16 + HW u16(Series/Type/Ver)
      return 3 + 4;           // 2026-10-10 实机：原 3+6 数错，应答 7 字节
    case kFcReadCurrent:      // u16 mA
    case kFcReadTemp:         // 温度符号 + u8 ℃
    case kFcReadStatusPair:   // 回零状态 + 电机状态
      return 3 + 2;
    case kFcReadSpeed:        // 符号 + u16（0.1RPM）
      return 3 + 3;
    case kFcReadPosition:     // 符号 + u32（0.1°）
    case kFcReadTarget:       // 符号 + u32（0.1°）
    case kFcReadError:        // 符号 + u32（0.01°）
      return 3 + 5;
    default:
      return 0;
  }
}

// 解码结果分类。除 kOk 外一律不得当作应答消费（§8.3 严格校验）。
enum class DecodeStatus : uint8_t {
  kOk = 0,
  kTooShort,     // 不足最短帧长
  kBadChecksum,  // 校验字节不是 0x6B（含驱动器被改设 XOR/CRC8 的情形）
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

// 编码命令帧（允许广播地址——用于同步触发/急停）。容量不足或超长返回 0。
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
  out[0] = driver_id;
  out[1] = function;
  if (payload_len > 0) {
    memcpy(&out[2], payload, payload_len);
  }
  out[2 + payload_len] = kChecksumFixed;
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

// 严格解码应答帧：长度/校验字节/地址全查，任何一项不符返回错误分类。
inline DecodeStatus decode_reply(const uint8_t* frame, size_t len,
                                 ReplyFrame* out) {
  if (frame == nullptr || len < kMinFrameBytes) {
    return DecodeStatus::kTooShort;
  }
  if (frame[len - 1] != kChecksumFixed) {
    return DecodeStatus::kBadChecksum;
  }
  if (frame[0] == kBroadcastId) {
    return DecodeStatus::kBadDriverId;
  }
  if (out != nullptr) {
    out->driver_id = frame[0];
    out->function = frame[1];
    const size_t payload_len = len - kFrameOverhead;
    out->payload = (payload_len > 0) ? &frame[2] : nullptr;
    out->payload_len = payload_len;
  }
  return DecodeStatus::kOk;
}

// 请求-应答严格匹配：地址与功能码都一致才算本次请求的回复。
inline bool reply_matches(const ReplyFrame& reply, uint8_t expected_id,
                          uint8_t expected_function) {
  return reply.driver_id == expected_id && reply.function == expected_function;
}

// 是否驱动器主动上报帧（动作完成 9F 族）——不属于任何等待中的请求。
// 手册 4.2.2：返回数据首字节 9F = 到位（FD）/回零完成（9A）/夹爪夹紧（F5）。
inline bool is_unsolicited(const ReplyFrame& reply) {
  return reply.payload_len >= 1 && reply.payload[0] == kAckUnsolicited;
}

// 触发类命令的应答状态码是否表示“命令已被驱动器接受/完成”。
// 仅 02 = 正确。E2（参数错/保护触发）与 EE（格式错）当失败；
// 12/22 仅出现在回零命令 9A（已在零点/限位）——本层未使用 9A，
// 保守起见同样不算接受；将来扩展回零时再放开。
inline bool ack_accepted(const ReplyFrame& reply) {
  return reply.payload_len >= 1 && reply.payload[0] == kAckOk;
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

// ---- 读取类应答的数据解析（手册 5.5.x 字段表，大端）----

// [符号 u8][数值 u32]：00=正 / 01=负（5.5.9/11/13/14 通用）。
inline bool parse_signed_u32(const uint8_t* payload, size_t len,
                             int32_t* out) {
  if (payload == nullptr || len < 5 || out == nullptr) {
    return false;
  }
  uint32_t raw = 0;
  for (size_t i = 1; i < 5; ++i) {
    raw = (raw << 8) | payload[i];
  }
  *out = (payload[0] == 0x01) ? -(int32_t)raw : (int32_t)raw;
  return true;
}

// [符号 u8][数值 u16]：00=正 / 01=负（5.5.11 转速）。
inline bool parse_signed_u16(const uint8_t* payload, size_t len,
                             int32_t* out) {
  if (payload == nullptr || len < 3 || out == nullptr) {
    return false;
  }
  const uint16_t raw = (uint16_t)((payload[1] << 8) | payload[2]);
  *out = (payload[0] == 0x01) ? -(int32_t)raw : (int32_t)raw;
  return true;
}

// [温度符号 u8][温度 u8]：00=正 / 01=负。手册 5.5.12 标称与其他命令相反
// （00=负），2026-10-10 实机 X 固件证伪：室温下应答 00 1D（=29℃），
// 符号位与其余命令一致，00=正、01=负。
inline bool parse_temp(const uint8_t* payload, size_t len, int32_t* out) {
  if (payload == nullptr || len < 2 || out == nullptr) {
    return false;
  }
  *out = (payload[0] == 0x01) ? -(int32_t)payload[1] : (int32_t)payload[1];
  return true;
}

// 无符号 u16（5.5.5 总线电流 mA）。
inline bool parse_u16(const uint8_t* payload, size_t len, uint16_t* out) {
  if (payload == nullptr || len < 2 || out == nullptr) {
    return false;
  }
  *out = (uint16_t)((payload[0] << 8) | payload[1]);
  return true;
}

// ---- 运动时长预估（运动挂起探测窗用）----
// 梯形相对运动预估：匀速段 steps/speed + 加减速段 2×(速度RPM/加速度)。
// 只用于 x42s_uart 的挂起探测截止时刻（防预估失准死锁），不参与到位
// 判定——到位的权威来源是 3C 状态字的 Prf_TF 位。上限 1h 防溢出。
constexpr uint32_t kMaxMoveEstimateMs = 3600000;

inline uint32_t estimate_move_ms(uint32_t abs_steps, uint32_t speed_steps_s,
                                 uint32_t pulse_per_rev, uint32_t acc_rpm_ps) {
  if (speed_steps_s == 0 || pulse_per_rev == 0 || acc_rpm_ps == 0) {
    return kMaxMoveEstimateMs;
  }
  const uint64_t cruise =
      ((uint64_t)abs_steps * 1000 + speed_steps_s - 1) / speed_steps_s;
  const uint64_t rpm =
      ((uint64_t)speed_steps_s * 60 + pulse_per_rev - 1) / pulse_per_rev;
  const uint64_t accel_ms = (rpm * 2000 + acc_rpm_ps - 1) / acc_rpm_ps;
  const uint64_t total = cruise + accel_ms;
  return total >= kMaxMoveEstimateMs ? kMaxMoveEstimateMs : (uint32_t)total;
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
