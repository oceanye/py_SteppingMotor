// esp32_stepper/src/x42s_uart.cpp
// X42S 总线驱动实现。帧层纯函数见 x42s_protocol.h；遥测语义见
// x42s_model.h。收发范式沿用 remote_stepper.cpp 的同步 transact：
// 丢弃残留 → 发帧 → 按期望应答长度收帧（协议无帧尾，0x6B 是校验
// 字节可能出现在数据区，不能当帧尾扫）→ 严格解码匹配。
//
// ✅ 功能码/数据布局/状态字已按《ZDT X42S 手册 V1.0.5》核对（2026-09-29）：
//   使能 F3 AB、停止 FE 98、设零 93 88、同步触发 FF 66（广播）；
//   位置模式 FD（X 固件梯形，角度制 0.1°）；遥测轮询 3C/36/33/37/35/39/26。
//   单位换算依赖 config.h 的 X42S_PULSE_PER_REV（驱动器细分）。
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

// 运动挂起（2026-10-12 长运动 UART 卡死修复）：MOVE 期间该台驱动器
// 执行插补，高频查询风暴（全量轮询 ~70 事务/s）可致其 UART 服务卡死
// （10.8s 运动实测：连广播读 ID 都不应答，断电才恢复；2s 短运动无恙）。
// 挂起 = 跳过全量轮询，仅按 X42S_MOVE_PROBE_MS 节拍发 3C+36 两帧探测；
// 见到位（Prf_TF）或保护故障即恢复全量轮询，探测窗超限强制退出防死锁。
bool s_moving[256] = {};
uint32_t s_next_probe[256] = {};     // 下次探测时刻（含起步盲区偏移）
uint32_t s_probe_deadline[256] = {};  // 挂起强制退出时刻（预估时长×2+2s）

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

void note_unsolicited(const x42s::ReplyFrame& reply) {
  ++s_stats.unsolicited_frames;
  // 手册 4.2.2：FD+9F=到位，9A+9F=回零完成。仅在位状态作加速信号；
  // 该上报受驱动器“控制命令应答”设置控制（可能关闭），reached 的
  // 权威来源是轮询 3C 的 Prf_TF 位。
  x42s::Telemetry& t = s_telemetry[reply.driver_id];
  t.reached = true;
}

// 读取一帧应答：按期望长度收满再严格解码。无帧尾协议下错对齐时丢首
// 字节滑窗重同步；主动上报帧（9F）写缓存继续等；触发类应答的状态码
// E2/EE 表示驱动器拒绝（参数错/保护触发/格式错），按失败处理。
bool transact(uint8_t id, uint8_t function, const uint8_t* payload,
              size_t payload_len, x42s::ReplyFrame* out) {
  const size_t expect_len = x42s::expected_reply_frame_len(function);
  if (expect_len < x42s::kMinFrameBytes) {
    set_error("unknown function code");
    return false;
  }
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
      buf[fill++] = (uint8_t)s_x42s.read();
      if (fill < expect_len) continue;
      const x42s::DecodeStatus status = x42s::decode_reply(buf, expect_len, &reply);
      if (status != x42s::DecodeStatus::kOk) {
        x42s::note_decode_failure(&s_stats, status);
      } else if (x42s::reply_matches(reply, id, function)) {
        if (x42s::is_unsolicited(reply)) {
          note_unsolicited(reply);  // 9F 与应答同长（4 字节）时先到先记
        } else {
          ++s_stats.replies_matched;
          if (expect_len == 4 && !x42s::ack_accepted(reply)) {
            // 触发类应答：02=接受；E2/EE=驱动器拒绝（参数错/保护触发/格式错）。
            set_error("driver rejected (E2/EE)");
            return false;
          }
          if (out != nullptr) *out = reply;
          return true;
        }
      } else if (x42s::is_unsolicited(reply)) {
        note_unsolicited(reply);
      } else {
        ++s_stats.mismatched_replies;  // 他台/他功能的帧，不得消费
      }
      // 失败/旁路帧：丢首字节滑窗，重新对齐（0x6B 可能在数据区出现）。
      memmove(buf, buf + 1, fill - 1);
      --fill;
    }
    delay(1);
  }
  x42s::note_timeout(&s_stats);
  ++s_telemetry[id].timeout_count;
  set_error("driver timeout");
  return false;
}

// 急停链专用（2026-10-12）：只发帧不等应答，但帧后必须留静默间隔。
// 急停的安全语义是最快把停令砸上总线（在线台收到即执行）；离线台不会
// 应答，逐台等 2×X42S_RESPONSE_TIMEOUT_MS 超时纯属烧时间——3 台离线时
// estop 总耗时 ~360ms+，叠上命令排队（tick 正对离线台做 7×60ms 全量
// 轮询）就把 PC 侧 0.8s 的 ESTOP 握手窗拖爆（台架实测 5 次"串口 I/O
// 异常"断开）。台架另证（同日）：背靠背连发 8 帧驱动器整批丢帧（运动中
// ESTOP 未拦停），而确认式单帧 7.8ms 即被接受——驱动器需要帧间总线
// 静默消化，间隔取 X42S_CMD_GAP_MS。迟到的应答帧由下个事务开头的清
// 残留丢弃，不会错配消费。
static void send_frame_no_wait(uint8_t id, uint8_t function,
                               const uint8_t* payload, size_t payload_len) {
  uint8_t frame[x42s::kMaxFrameBytes];
  const size_t len = x42s::encode_unicast_command(
      id, function, payload, payload_len, frame, sizeof(frame));
  if (len == 0) {
    set_error("encode failed");
    return;
  }
  write_frame(frame, len);
  delay(X42S_CMD_GAP_MS);
}

// 遥测轮询（手册 5.5.x）：3C 状态对 + 36 实测位置 + 33 目标位置 + 37 跟随
// 误差 + 35 转速 + 39 温度 + 26 总线电流，共 7 次事务。3C/36 失败即视为
// 本次轮询失败（在线判定依赖它们）；其余字段失败只降级为无该字段
// （valid=false，严禁回退 0）。单位换算按 X 固件：位置 0.1°、误差 0.01°、
// 转速 0.1RPM、温度 ℃、电流 mA。
bool fetch_telemetry(uint8_t id) {
  x42s::Telemetry& t = s_telemetry[id];
  x42s::telemetry_invalidate_values(t);
  uint8_t org_bits = 0, status_bits = 0;
  {
    x42s::ReplyFrame reply;
    if (!transact(id, x42s::kFcReadStatusPair, nullptr, 0, &reply)) {
      return false;
    }
    if (reply.payload_len < 2) {
      set_error("bad status pair reply");
      return false;
    }
    org_bits = reply.payload[0];
    status_bits = reply.payload[1];
  }
  int32_t measured = 0;
  {
    x42s::ReplyFrame reply;
    if (!transact(id, x42s::kFcReadPosition, nullptr, 0, &reply)) {
      return false;
    }
    if (!x42s::parse_signed_u32(reply.payload, reply.payload_len, &measured)) {
      set_error("bad position reply");
      return false;
    }
  }
  const uint32_t now = millis();
  x42s::telemetry_mark_seen(t, now);
  t.enabled = (status_bits & x42s::kStatusEnabled) != 0;
  t.reached = (status_bits & x42s::kStatusReached) != 0;
  t.fault = x42s::fault_from_status_words(org_bits, status_bits);
  t.measured_position = (float)measured / 10.0f;  // 0.1° → °
  t.measured_valid = true;

  int32_t target = 0;
  x42s::ReplyFrame reply;
  if (transact(id, x42s::kFcReadTarget, nullptr, 0, &reply) &&
      x42s::parse_signed_u32(reply.payload, reply.payload_len, &target)) {
    t.command_position = (float)target / 10.0f;
    t.command_valid = true;
  } else {
    t.command_valid = false;
  }
  int32_t error = 0;
  if (transact(id, x42s::kFcReadError, nullptr, 0, &reply) &&
      x42s::parse_signed_u32(reply.payload, reply.payload_len, &error)) {
    t.position_error = (float)error / 100.0f;  // 0.01° → °
    t.error_valid = true;
  } else {
    t.error_valid = false;
  }
  int32_t speed = 0;
  if (transact(id, x42s::kFcReadSpeed, nullptr, 0, &reply) &&
      x42s::parse_signed_u16(reply.payload, reply.payload_len, &speed)) {
    t.speed = (float)speed / 10.0f;  // 0.1RPM → RPM
    t.speed_valid = true;
  } else {
    t.speed_valid = false;
  }
  int32_t temp = 0;
  if (transact(id, x42s::kFcReadTemp, nullptr, 0, &reply) &&
      x42s::parse_temp(reply.payload, reply.payload_len, &temp)) {
    t.temperature_c = (float)temp;
    t.temperature_valid = true;
  } else {
    t.temperature_valid = false;
  }
  uint16_t current_ma = 0;
  if (transact(id, x42s::kFcReadCurrent, nullptr, 0, &reply) &&
      x42s::parse_u16(reply.payload, reply.payload_len, &current_ma)) {
    t.current_a = (float)current_ma / 1000.0f;
    t.current_valid = true;
  } else {
    t.current_valid = false;
  }
  return true;
}

// 挂起期轻探测（运动挂起修复）：仅 3C 状态对 + 36 实测位置两帧，刷
// enabled/reached/fault/在线时刻与 GUI 位置。返回 true = 仍在运动
// （未到位且无保护故障），false = 可恢复全量轮询。超时/坏帧保守视为
// 仍在运动——探测窗超限（tick 里判定）兜底退出，不会死锁。
bool probe_move(uint8_t id) {
  x42s::Telemetry& t = s_telemetry[id];
  x42s::telemetry_invalidate_values(t);
  x42s::ReplyFrame reply;
  if (!transact(id, x42s::kFcReadStatusPair, nullptr, 0, &reply) ||
      reply.payload_len < 2) {
    return true;
  }
  x42s::telemetry_mark_seen(t, millis());
  const uint8_t org_bits = reply.payload[0];
  const uint8_t status_bits = reply.payload[1];
  t.enabled = (status_bits & x42s::kStatusEnabled) != 0;
  t.reached = (status_bits & x42s::kStatusReached) != 0;
  t.fault = x42s::fault_from_status_words(org_bits, status_bits);
  const bool done = t.reached || t.fault != x42s::Fault::kNone;
  int32_t measured = 0;
  // Even the first reached=1 response needs a new measured position. Skipping
  // 36 here returned the previous moving sample as a freshly reached endpoint,
  // which inflated the PC's next (cumulative target - measured) segment.
  // This remains a two-frame probe; failure leaves measured_valid=false.
  if (transact(id, x42s::kFcReadPosition, nullptr, 0, &reply) &&
      x42s::parse_signed_u32(reply.payload, reply.payload_len, &measured)) {
    t.measured_position = (float)measured / 10.0f;  // 0.1° → °
    t.measured_valid = true;
  }
  return !done;
}
// 地址位与数据位各报一遍真实 ID。不依赖任何预置 ID，台架排查零应答时
// 用它一步定位"驱动器在不在答、答的是几号"。应答地址是电机的真实 ID
// （非发送的广播 0），与 transact 的严格匹配不符，故独立收帧循环；
// 多台同时挂网会撞应答，手册本条即按"单独接线该电机"设计。
int broadcast_read_id() {
  while (s_x42s.available()) (void)s_x42s.read();  // 清残留
  uint8_t frame[x42s::kMaxFrameBytes];
  const size_t len = x42s::encode_command(
      x42s::kBroadcastId, x42s::kFcReadId, nullptr, 0, frame, sizeof(frame));
  if (len == 0) {
    set_error("encode failed");
    return -1;
  }
  write_frame(frame, len);
  // 应答窗口比单次事务放宽（3×）：SCAN 是一次性阻塞命令，多等换命中。
  const uint32_t deadline = millis() + 3 * X42S_RESPONSE_TIMEOUT_MS;
  uint8_t buf[x42s::kMaxFrameBytes];
  size_t fill = 0;
  x42s::ReplyFrame reply;
  while ((int32_t)(millis() - deadline) < 0) {
    while (s_x42s.available()) {
      buf[fill++] = (uint8_t)s_x42s.read();
      if (fill < 4) continue;  // 应答定长 4 字节（见 expected_reply_frame_len）
      const x42s::DecodeStatus status = x42s::decode_reply(buf, 4, &reply);
      if (status == x42s::DecodeStatus::kOk &&
          reply.function == x42s::kFcReadId && reply.payload_len >= 1 &&
          reply.payload[0] == reply.driver_id) {
        ++s_stats.replies_matched;
        return (int)reply.driver_id;
      }
      if (status != x42s::DecodeStatus::kOk) {
        x42s::note_decode_failure(&s_stats, status);
      } else {
        ++s_stats.mismatched_replies;
      }
      memmove(buf, buf + 1, fill - 1);  // 丢首字节滑窗重同步
      --fill;
    }
    delay(1);
  }
  return -1;
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
  // 挂起台跳过：MOVE 刚被接受显然在线，且探测节拍(300ms)本就低于
  // 掉线窗(500ms)——盲区瞬间超窗属预期，不得误报 OFFLINE。
  for (uint8_t i = 0; i < X42S_DRIVER_COUNT; ++i) {
    const uint8_t id = kDriverIds[i];
    if (s_moving[id]) continue;
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
  if (s_moving[id]) {
    // 运动挂起：不做 7 帧全量轮询。探测窗超限强制退出（预估失准/
    // 驱动器异常时恢复常规轮询，靠超时计数暴露问题）；否则按节拍
    // 两帧轻探测，到位/保护故障即恢复。
    if ((int32_t)(now - s_probe_deadline[id]) >= 0) {
      s_moving[id] = false;
      (void)fetch_telemetry(id);
      return;
    }
    if ((int32_t)(now - s_next_probe[id]) >= 0) {
      s_next_probe[id] = now + X42S_MOVE_PROBE_MS;
      if (!probe_move(id)) s_moving[id] = false;
    }
    return;
  }
  (void)fetch_telemetry(id);  // 失败只累计超时计数，不打断轮转
}

bool x42s_ping(uint8_t id) {
  if (!id_valid(id)) return false;
  // 5.5.2 读固件/硬件版本：应答即在线证明（payload 含版本号，暂不解析）。
  if (!transact(id, x42s::kFcReadVersion, nullptr, 0, nullptr)) return false;
  x42s::telemetry_mark_seen(s_telemetry[id], millis());
  return true;
}

bool x42s_enable(uint8_t id, bool enable) {
  if (!id_valid(id)) return false;
  // 5.3.2：F3 AB <使能 01/00> <同步 00=立即>。
  const uint8_t payload[3] = {x42s::kAuxEnable,
                              static_cast<uint8_t>(enable ? 0x01 : 0x00),
                              0x00};
  if (!transact(id, x42s::kFcEnable, payload, sizeof(payload), nullptr))
    return false;
  if (!enable) s_moving[id] = false;  // 失能即停：清挂起态恢复全量轮询
  s_telemetry[id].enabled = enable;
  x42s::telemetry_mark_seen(s_telemetry[id], millis());
  return true;
}

bool x42s_stop(uint8_t id) {
  if (!id_valid(id)) return false;
  // 5.3.15：FE 98 <同步 00=立即>。
  const uint8_t payload[2] = {x42s::kAuxStop, 0x00};
  if (!transact(id, x42s::kFcStop, payload, sizeof(payload), nullptr))
    return false;
  s_moving[id] = false;  // 立即停止：无运动在途，恢复全量轮询
  x42s::telemetry_mark_seen(s_telemetry[id], millis());
  return true;
}

void x42s_estop() {
  // 逐台单播 FE98 停止 + F3AB 失能，但 best-effort 只发不等（见
  // send_frame_no_wait 注释：等离线台超时会拖爆 PC 的 0.8s ESTOP 握手）。
  // §10 仍满足——不用广播，逐台单播。
  // 5.3.15：FE 98 <同步 00=立即>；5.3.2：F3 AB <失能 00> <00=立即>。
  const uint8_t stop_payload[2] = {x42s::kAuxStop, 0x00};
  const uint8_t disable_payload[3] = {x42s::kAuxEnable, 0x00, 0x00};
  for (uint8_t i = 0; i < X42S_DRIVER_COUNT; ++i) {
    const uint8_t id = kDriverIds[i];
    send_frame_no_wait(id, x42s::kFcStop, stop_payload, sizeof(stop_payload));
    send_frame_no_wait(id, x42s::kFcEnable, disable_payload,
                       sizeof(disable_payload));
    // 失能不确认就不谎报 enabled（§12 精神）：真实状态由后续常规轮询的
    // 3C 刷新。reached 直接清（§14.1-4）：迟到的 9F 上报不会重新点亮
    // 运动；恢复必须显式 x42s_enable。
    s_telemetry[id].reached = false;
    s_moving[id] = false;  // 急停即弃挂起：恢复常规轮询暴露真实状态
  }
}

bool x42s_zero_position(uint8_t id) {
  if (!id_valid(id)) return false;
  // 5.4.1：93 88 <是否存储 01=掉电不丢>。
  const uint8_t payload[2] = {x42s::kAuxSetZero, 0x01};
  return transact(id, x42s::kFcSetZero, payload, sizeof(payload), nullptr);
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
  // 5.3.10（X 固件 FD 梯形位置模式，角度制）：
  //   [方向 00=CW/01=CCW][加速 u16 RPM/S][减速 u16][速度 u16 0.1RPM]
  //   [位置 u32 0.1°][运动模式 02=相对当前实时位置][同步 00=立即]
  // 步数→角度经 X42S_PULSE_PER_REV（驱动器细分，config.h）换算；
  // 方向位承载符号（正=00，负=01），位置恒为绝对量。
  const int32_t angle_01deg =
      (int32_t)(((int64_t)steps * 3600) / X42S_PULSE_PER_REV);
  const uint32_t speed_01rpm = (uint32_t)(
      ((uint64_t)speed_steps_s * 600) / X42S_PULSE_PER_REV);
  if (angle_01deg == 0 || speed_01rpm == 0) {
    set_error("below driver resolution");
    return false;
  }
  const uint32_t pos = (uint32_t)(
      steps > 0 ? angle_01deg : -angle_01deg);
  const uint16_t acc = X42S_TRAP_ACC_RPM_PS > 0xFFFF ? 0xFFFF
                                                     : X42S_TRAP_ACC_RPM_PS;
  const uint8_t payload[13] = {
      static_cast<uint8_t>(steps > 0 ? 0x00 : 0x01),
      (uint8_t)(acc >> 8), (uint8_t)acc,
      (uint8_t)(acc >> 8), (uint8_t)acc,
      (uint8_t)(speed_01rpm >> 8), (uint8_t)speed_01rpm,
      (uint8_t)(pos >> 24), (uint8_t)(pos >> 16), (uint8_t)(pos >> 8),
      (uint8_t)pos,
      0x02,  // 相对当前实时位置
      0x00,  // 不缓存，立即执行（多机同步走 sync_trigger 路径）
  };
  if (!transact(id, x42s::kFcMoveTrap, payload, sizeof(payload), nullptr))
    return false;
  x42s::Telemetry& t = s_telemetry[id];
  t.reached = false;
  // 预估目标仅作无遥测时的占位显示；下次 33 轮询会覆盖为驱动器真值。
  t.command_position +=
      (float)steps * 360.0f / (float)X42S_PULSE_PER_REV;
  t.command_valid = true;
  x42s::telemetry_mark_seen(t, millis());
  // 运动挂起：起步盲区跨过"旧 reached=1 未清"前沿，之后低频探测；
  // 探测窗超限（预估×2+2s）兜底退出，见 tick 的挂起分支。
  const uint32_t now = millis();
  s_moving[id] = true;
  s_next_probe[id] = now + X42S_MOVE_START_BLIND_MS;
  s_probe_deadline[id] =
      now + 2 * x42s::estimate_move_ms(
                  (uint32_t)(steps > 0 ? steps : -steps), speed_steps_s,
                  X42S_PULSE_PER_REV, X42S_TRAP_ACC_RPM_PS) +
      2000;
  return true;
}

bool x42s_sync_trigger() {
  // 5.3.16：广播 00 FF 66 触发已缓存（同步标志=01）命令同步执行。
  // §10 明确允许的广播用途。手册：地址 1 电机会回确认帧 01 FF 02 6B，
  // 此处不消费它（下个事务开头的清残留会丢弃）。
  uint8_t frame[x42s::kMaxFrameBytes];
  const uint8_t payload[1] = {x42s::kAuxSyncTrigger};
  const size_t len = x42s::encode_command(
      x42s::kBroadcastId, x42s::kFcSyncTrigger, payload, sizeof(payload),
      frame, sizeof(frame));
  if (len == 0) {
    set_error("encode failed");
    return false;
  }
  write_frame(frame, len);
  delay(2);  // 总线静默窗口（保守值，待台架标定）
  while (s_x42s.available()) (void)s_x42s.read();  // 清地址1确认帧残留
  return true;
}

bool x42s_read_status(uint8_t id) {
  if (!id_valid(id)) return false;
  // 运动挂起：7 帧全量读会复发轮询风暴（2026-10-10 台架实证），降级为
  // 两帧轻探测（DIAG 人工命令也不例外）；探测失败退出挂起恢复常规轮询。
  if (s_moving[id]) {
    if (!probe_move(id)) s_moving[id] = false;
    return true;
  }
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
  // 2026-10-10 台架排查零应答，扫描改两级（用户要求 0-9）：
  // ① 广播读 ID（5.6.30）——id 0 是广播地址不可单播 ping，其探测语义
  //    由这一步覆盖：不问"0 号在不在"，问"你是几号"。
  // ② 逐个 ping 1..9（读固件版本 1F，应答即在线）。唯一性由"每台单独
  //    ping 应答"天然保证（§10 原注释）；两台同号须在上机初始化（§7）
  //    逐台设置时避免。扫描结果逐台报行。
  const int found = broadcast_read_id();
  serial_tx_printf("X42S,SCAN,BCAST_ID,%s",
                   found > 0 ? String(found).c_str() : "NONE");
  for (uint8_t id = 1; id <= 9; ++id) {
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

void x42s_raw_probe(const uint8_t* frame, size_t len) {
  while (s_x42s.available()) (void)s_x42s.read();  // 清残留
  write_frame(frame, len);
  // 固定 300ms 采集窗：比单次事务宽，迟到应答也照收；期间不解读，
  // 收到的每个字节原样上报——诊断语义与 transact 的严格匹配相反。
  const uint32_t deadline = millis() + 300;
  uint8_t buf[64];
  size_t fill = 0;
  while ((int32_t)(millis() - deadline) < 0) {
    while (s_x42s.available() && fill < sizeof(buf)) {
      buf[fill++] = (uint8_t)s_x42s.read();
    }
    delay(1);
  }
  char hex[2 * sizeof(buf) + 1];
  for (size_t i = 0; i < fill; ++i) {
    static const char kHex[] = "0123456789abcdef";
    hex[2 * i] = kHex[buf[i] >> 4];
    hex[2 * i + 1] = kHex[buf[i] & 0x0F];
  }
  hex[2 * fill] = '\0';
  serial_tx_printf("X42S,RAW,RX,%u,%s", (unsigned)fill, hex);
}

const char* x42s_last_error() { return s_last_error; }

#endif  // DRIVE_MODE_GEAR && X42S_UART_ENABLED
