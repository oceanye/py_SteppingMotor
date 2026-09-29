// esp32_stepper/src/x42s_uart.cpp
// X42S 总线驱动实现。帧层纯函数见 x42s_protocol.h；遥测语义见
// x42s_model.h。收发范式沿用 remote_stepper.cpp 的同步 transact：
// 丢弃残留 → 发帧 → 限时等待首帧头 0x02 → 按 0x6B 断帧 → 严格解码。
//
// ⚠⚠ 全文所有 TODO(手册) 段在台架前必须按《ZDT X42S 用户手册 V1.0.5》
// 逐条核对：功能码、数据区字节序与长度、应答数据布局、状态字位。
// 帧结构 / 校验 / 匹配逻辑本身在 x42s_protocol.h，已单测覆盖。
#include "x42s_uart.h"

#if defined(DRIVE_MODE_GEAR) && X42S_UART_ENABLED

#include <Arduino.h>

#include "serial_tx.h"

namespace {

HardwareSerial s_x42s(1);

x42s::Telemetry s_telemetry[256];  // 按 driver_id 直接索引（0 号槽弃用）
x42s::CommStats s_stats{};
uint32_t s_last_poll = 0;
uint8_t s_poll_index = 0;          // 轮询游标：kDriverIds[] 下标
char s_last_error[48] = "unavailable";

// 首装建议台号（需求 §4）；X42S,SCAN / 轮询只覆盖这四台。
constexpr uint8_t kDriverIds[X42S_DRIVER_COUNT] = {1, 2, 3, 4};

void set_error(const char* error) {
  strncpy(s_last_error, error, sizeof(s_last_error) - 1);
  s_last_error[sizeof(s_last_error) - 1] = '\0';
}

bool id_valid(uint8_t id) {
  if (id < x42s::kDriverIdMin || id > x42s::kDriverIdMax) {
    set_error("bad driver id");
    return false;
  }
  return true;
}

void write_frame(const uint8_t* frame, size_t len) {
  s_x42s.write(frame, len);
  s_x42s.flush();
  ++s_stats.frames_sent;
}

// 读取一帧应答：等到超时窗口关闭为止；期间主动上报帧（如到位）写缓存
// 并继续等，未知/校验失败帧丢弃继续扫（0x6B 可能出现在数据区）。
bool transact(uint8_t id, uint8_t function, const uint8_t* payload,
              size_t payload_len, x42s::ReplyFrame* out) {
  while (s_x42s.available()) (void)s_x42s.read();  // 清残留，防错配消费
  uint8_t frame[x42s::kMaxFrameBytes];
  const size_t len = x42s::encode_unicast_command(
      id, function, payload, payload_len, frame, sizeof(frame));
  if (len == 0) {
    set_error("encode failed");
    return false;
  }
  write_frame(frame, len);

  const uint32_t deadline = millis() + X42S_RESPONSE_TIMEOUT_MS;
  uint8_t buf[x42s::kMaxFrameBytes];
  size_t fill = 0;
  x42s::ReplyFrame reply;
  while ((int32_t)(millis() - deadline) < 0) {
    while (s_x42s.available()) {
      const uint8_t byte = (uint8_t)s_x42s.read();
      if (fill == 0) {
        if (byte != x42s::kFrameHeadReply) continue;  // 重同步到帧头
        buf[fill++] = byte;
        continue;
      }
      buf[fill++] = byte;
      if (byte == x42s::kFrameTail && fill >= x42s::kFrameOverhead) {
        const x42s::DecodeStatus status = x42s::decode_reply(buf, fill, &reply);
        fill = 0;
        if (status == x42s::DecodeStatus::kOk) {
          if (x42s::reply_matches(reply, id, function)) {
            ++s_stats.replies_matched;
            if (out != nullptr) *out = reply;
            return true;
          }
          if (x42s::is_unsolicited(reply)) {
            ++s_stats.unsolicited_frames;
            s_telemetry[id].reached = true;  // 到位主动帧（§8.3）
            continue;
          }
          ++s_stats.mismatched_replies;  // 他台/他功能的帧，不得消费
          continue;
        }
        x42s::note_decode_failure(&s_stats, status);
        continue;  // 0x6B 出现在数据区：继续扫真正的帧尾
      }
      if (fill == sizeof(buf)) fill = 0;  // 溢出保护，重新找头
    }
    delay(1);
  }
  x42s::note_timeout(&s_stats);
  ++s_telemetry[id].timeout_count;
  set_error("driver timeout");
  return false;
}

// TODO(手册 V1.0.5)：以下数据区布局为张大头公开资料假设，逐条核对。
bool parse_i32(const uint8_t* payload, size_t len, size_t offset, int32_t* out) {
  if (payload == nullptr || len < offset + 4) return false;
  uint32_t raw = 0;
  for (size_t i = 0; i < 4; ++i) {
    raw = (raw << 8) | payload[offset + i];  // 大端假设，待核对
  }
  *out = (int32_t)raw;
  return true;
}

// 读状态：TODO(手册) 应答数据布局假设为
//   [status:u16][command:i32][measured:i32][speed:i16]…（核对后修正）
bool fetch_telemetry(uint8_t id) {
  // TODO(手册)：一次读全部还是分功能码多次读，按手册能力决定；
  // 这里按"读编码器位置 + 读目标位置 + 读状态字"三次事务实现。
  int32_t measured = 0, command = 0;
  uint16_t status_bits = 0;
  {
    x42s::ReplyFrame reply;
    if (!transact(id, x42s::kFcReadPosition, nullptr, 0, &reply)) return false;
    if (!parse_i32(reply.payload, reply.payload_len, 0, &measured)) {
      set_error("bad position reply");
      return false;
    }
  }
  {
    x42s::ReplyFrame reply;
    // TODO(手册)：目标位置功能码与 kFcReadPosition 是否同一功能码加子命令。
    if (!transact(id, x42s::kFcReadTarget, nullptr, 0, &reply)) return false;
    if (!parse_i32(reply.payload, reply.payload_len, 0, &command)) {
      set_error("bad target reply");
      return false;
    }
  }
  {
    x42s::ReplyFrame reply;
    // TODO(手册)：状态字功能码与位定义（fault_from_status_word 同步改）。
    if (!transact(id, x42s::kFcReadStatus, nullptr, 0, &reply)) return false;
    if (reply.payload_len >= 2) {
      status_bits = (uint16_t)((reply.payload[0] << 8) | reply.payload[1]);
    }
  }
  x42s::Telemetry& t = s_telemetry[id];
  const uint32_t now = millis();
  x42s::telemetry_mark_seen(t, now);
  t.command_position = (float)command;
  t.command_valid = true;
  t.measured_position = (float)measured;
  t.measured_valid = true;
  t.position_error = (float)(measured - command);
  t.error_valid = true;
  t.fault = x42s::fault_from_status_word(status_bits);
  return true;
}

}  // namespace

void x42s_uart_init() {
  for (auto& t : s_telemetry) x42s::telemetry_reset(t);
  s_x42s.begin(X42S_BAUD, SERIAL_8N1, PIN_X42S_RX, PIN_X42S_TX);
  s_last_poll = millis();
}

void x42s_uart_tick() {
  const uint32_t now = millis();
  // 掉线降沿：带事件行，PC 可直接定位是哪一台（§10）。
  for (uint8_t i = 0; i < X42S_DRIVER_COUNT; ++i) {
    const uint8_t id = kDriverIds[i];
    x42s::Telemetry& t = s_telemetry[id];
    if (x42s::telemetry_should_mark_offline(t, now)) {
      t.online = false;
      serial_tx_printf("X42S,%u,OFFLINE", (unsigned)id);
    }
  }
  if ((uint32_t)(now - s_last_poll) < X42S_POLL_INTERVAL_MS) return;
  s_last_poll = now;
  const uint8_t id = kDriverIds[s_poll_index];
  s_poll_index = (s_poll_index + 1) % X42S_DRIVER_COUNT;
  (void)fetch_telemetry(id);  // 失败只累计超时计数，不打断轮转
}

bool x42s_ping(uint8_t id) {
  if (!id_valid(id)) return false;
  // TODO(手册)：ping 应答可能带固件版本；此处只验证"应答且匹配"。
  if (!transact(id, x42s::kFcPing, nullptr, 0, nullptr)) return false;
  x42s::telemetry_mark_seen(s_telemetry[id], millis());
  return true;
}

bool x42s_enable(uint8_t id, bool enable) {
  if (!id_valid(id)) return false;
  // TODO(手册)：使能数据区假设 1 字节 01=使能 / 00=释放。
  const uint8_t payload[1] = {static_cast<uint8_t>(enable ? 0x01 : 0x00)};
  if (!transact(id, x42s::kFcEnable, payload, sizeof(payload), nullptr))
    return false;
  s_telemetry[id].enabled = enable;
  x42s::telemetry_mark_seen(s_telemetry[id], millis());
  return true;
}

bool x42s_stop(uint8_t id) {
  if (!id_valid(id)) return false;
  // TODO(手册)：立即停机数据区假设 1 字节 00=立即停。
  const uint8_t payload[1] = {0x00};
  if (!transact(id, x42s::kFcStop, payload, sizeof(payload), nullptr))
    return false;
  x42s::telemetry_mark_seen(s_telemetry[id], millis());
  return true;
}

void x42s_stop_all() {
  // §10：不得用广播执行普通单轴动作；全局停止也逐台单播，确认每台应答。
  for (uint8_t i = 0; i < X42S_DRIVER_COUNT; ++i) {
    (void)x42s_stop(kDriverIds[i]);
  }
}

void x42s_estop() {
  x42s_stop_all();
  // 失能全部：旧动作的迟到回复不得把状态重新置为运动（§14.1-4），
  // 这里直接清 reached 并失能；恢复必须显式 x42s_enable。
  for (uint8_t i = 0; i < X42S_DRIVER_COUNT; ++i) {
    const uint8_t id = kDriverIds[i];
    (void)x42s_enable(id, false);
    s_telemetry[id].reached = false;
  }
}

bool x42s_zero_position(uint8_t id) {
  if (!id_valid(id)) return false;
  return transact(id, x42s::kFcSetZero, nullptr, 0, nullptr);
}

bool x42s_move_relative(uint8_t id, int32_t steps, uint32_t speed_steps_s) {
  if (!id_valid(id)) return false;
  if (steps == 0) {
    set_error("zero steps");
    return false;
  }
  if (speed_steps_s == 0) {
    set_error("zero speed");
    return false;
  }
  // TODO(手册)：位置模式数据区假设
  //   [speed:i32 大端][pulses:i32 大端][abs=0x00 相对][sync_cache=0x00 立即]
  uint8_t payload[10];
  const uint32_t speed = speed_steps_s;
  payload[0] = (uint8_t)(speed >> 24);
  payload[1] = (uint8_t)(speed >> 16);
  payload[2] = (uint8_t)(speed >> 8);
  payload[3] = (uint8_t)(speed);
  payload[4] = (uint8_t)((uint32_t)steps >> 24);
  payload[5] = (uint8_t)((uint32_t)steps >> 16);
  payload[6] = (uint8_t)((uint32_t)steps >> 8);
  payload[7] = (uint8_t)((uint32_t)steps);
  payload[8] = 0x00;  // 相对位置
  payload[9] = 0x00;  // 不缓存，立即执行（多机同步走 sync_trigger 路径）
  if (!transact(id, x42s::kFcMoveRelative, payload, sizeof(payload), nullptr))
    return false;
  x42s::Telemetry& t = s_telemetry[id];
  t.reached = false;
  t.command_position += (float)steps;
  t.command_valid = true;
  x42s::telemetry_mark_seen(t, millis());
  return true;
}

bool x42s_sync_trigger() {
  // 广播触发是 §10 明确允许的广播用途；广播无应答，不做超时判定。
  uint8_t frame[x42s::kMaxFrameBytes];
  const size_t len = x42s::encode_command(
      x42s::kBroadcastId, x42s::kFcSyncTrigger, nullptr, 0, frame,
      sizeof(frame));
  if (len == 0) {
    set_error("encode failed");
    return false;
  }
  write_frame(frame, len);
  delay(2);  // 总线静默窗口（保守值，待台架标定）
  return true;
}

bool x42s_read_status(uint8_t id) {
  if (!id_valid(id)) return false;
  return fetch_telemetry(id);
}

const x42s::Telemetry& x42s_telemetry(uint8_t id) { return s_telemetry[id]; }
const x42s::CommStats& x42s_comm_stats() { return s_stats; }

void x42s_print_status_line(uint8_t id) {
  if (!id_valid(id)) return;
  const x42s::Telemetry& t = s_telemetry[id];
  const uint32_t now = millis();
  const uint32_t age = x42s::telemetry_age_ms(t, now);
  // valid 位掩码（§12 未知不冒充 0）：bit0=cmd bit1=meas bit2=err
  // bit3=speed bit4=current bit5=temp；PC 端按位还原为 None。
  const unsigned valid_mask =
      (t.command_valid ? 0x01u : 0u) | (t.measured_valid ? 0x02u : 0u) |
      (t.error_valid ? 0x04u : 0u) | (t.speed_valid ? 0x08u : 0u) |
      (t.current_valid ? 0x10u : 0u) | (t.temperature_valid ? 0x20u : 0u);
  // X42S,<id>,S,<online>,<enabled>,<reached>,<fault>,<cmd>,<meas>,<err>,
  //            <speed>,<current>,<temp>,<age_ms>,<timeouts>,<valid_mask>
  serial_tx_printf(
      "X42S,%u,S,%d,%d,%d,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%lu,%lu,%u",
      (unsigned)id, t.online ? 1 : 0, t.enabled ? 1 : 0, t.reached ? 1 : 0,
      x42s::fault_tag(t.fault), (double)t.command_position,
      (double)t.measured_position, (double)t.position_error, (double)t.speed,
      (double)t.current_a, (double)t.temperature_c,
      (unsigned long)(age == UINT32_MAX ? 0 : age),
      (unsigned long)t.timeout_count, valid_mask);
}

void x42s_print_scan() {
  // §10：启动扫描 1..4 + 唯一性由"每台单独 ping 应答"天然保证——
  // 两台误拨同号时，第二次 ping 也会得到该号应答，但总线上只有
  // 一个号在答：无法从协议层直接区分，必须在 X42S 上机初始化（§7）
  // 逐台设置唯一 ID 时避免。扫描结果逐台报行。
  for (uint8_t i = 0; i < X42S_DRIVER_COUNT; ++i) {
    const uint8_t id = kDriverIds[i];
    const bool online = x42s_ping(id);
    serial_tx_printf("X42S,%u,SCAN,%s", (unsigned)id,
                     online ? "ONLINE" : "OFFLINE");
  }
}

void x42s_print_diag(uint8_t id) {
  if (!id_valid(id)) return;
  const bool online = x42s_ping(id);
  if (online) (void)x42s_read_status(id);
  x42s_print_status_line(id);
  const x42s::CommStats& s = x42s_comm_stats();
  // X42S,<id>,DIAG,<tx>,<rx_matched>,<cksum_err>,<mismatch>,<unsolicited>,<timeout>
  serial_tx_printf("X42S,%u,DIAG,%lu,%lu,%lu,%lu,%lu,%lu", (unsigned)id,
                   (unsigned long)s.frames_sent,
                   (unsigned long)s.replies_matched,
                   (unsigned long)s.checksum_errors,
                   (unsigned long)s.mismatched_replies,
                   (unsigned long)s.unsolicited_frames,
                   (unsigned long)s.timeouts);
}

const char* x42s_last_error() { return s_last_error; }

#endif  // DRIVE_MODE_GEAR && X42S_UART_ENABLED
