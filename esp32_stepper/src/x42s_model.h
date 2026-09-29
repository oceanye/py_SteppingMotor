// X42S 执行器遥测模型（纯数据层，feature/x42s-uart-actuator-binding）.
//
// 与 PC 端 motor_control/actuator_model.py 保持同一语义：
// - command / measured / error 三值分开，未知必须用 *_valid=false 表达，
//   严禁把未知回退成 0；
// - 新鲜度窗口 200ms（10~20Hz 轮询）、掉线窗口 500ms；
// - 故障标签取自 §14 安全集。
// 本文件 header-only 纯函数：ESP32 固件与主机 native 测试共用。
// ⚠ 状态字 → 故障标签的位映射为假设，待《ZDT X42S 用户手册 V1.0.5》核对。
#pragma once

#include <stdint.h>

namespace x42s {

constexpr uint32_t kTelemetryFreshMs = 200;   // 与 PC 端 FRESH_MAX_AGE 一致
constexpr uint32_t kTelemetryOfflineMs = 500; // 与 PC 端 STALE_AGE 一致

// 故障标签（§14）。顺序即状态位检查优先级（安全优先）。
enum class Fault : uint8_t {
  kNone = 0,
  kStall,
  kOverCurrent,
  kOverTemp,
  kUnderVoltage,
  kFollowError,
  kCommError,
  kAddrConflict,
};

inline const char* fault_tag(Fault fault) {
  switch (fault) {
    case Fault::kStall: return "stall";
    case Fault::kOverCurrent: return "over_current";
    case Fault::kOverTemp: return "over_temp";
    case Fault::kUnderVoltage: return "under_voltage";
    case Fault::kFollowError: return "follow_error";
    case Fault::kCommError: return "comm_error";
    case Fault::kAddrConflict: return "addr_conflict";
    case Fault::kNone: break;
  }
  return "none";
}

// 状态字 → 故障标签。TODO(手册 V1.0.5): 位定义待核对；当前假设
// bit0=堵转 bit1=过流 bit2=过热 bit3=低压 bit4=跟随误差。
inline Fault fault_from_status_word(uint16_t status_bits) {
  if (status_bits & 0x0001) return Fault::kStall;
  if (status_bits & 0x0002) return Fault::kOverCurrent;
  if (status_bits & 0x0004) return Fault::kOverTemp;
  if (status_bits & 0x0008) return Fault::kUnderVoltage;
  if (status_bits & 0x0010) return Fault::kFollowError;
  return Fault::kNone;
}

// 单台驱动器遥测缓存。全部字段无锁访问：只由总线单写者更新，
// 协议层只读快照（读取时可能有撕裂，但状态查询非安全链路）。
struct Telemetry {
  bool online;
  bool enabled;
  bool reached;
  bool command_valid;
  bool measured_valid;
  bool error_valid;
  bool speed_valid;
  bool current_valid;
  bool temperature_valid;
  float command_position;
  float measured_position;
  float position_error;
  float speed;
  float current_a;
  float temperature_c;
  Fault fault;
  uint32_t last_seen_ms;   // 0 = 从未应答
  uint32_t timeout_count;  // 该台应答超时累计（掉线定位用，§10）
};

inline void telemetry_reset(Telemetry& t) {
  t.online = false;
  t.enabled = false;
  t.reached = false;
  t.command_valid = false;
  t.measured_valid = false;
  t.error_valid = false;
  t.speed_valid = false;
  t.current_valid = false;
  t.temperature_valid = false;
  t.command_position = 0.0f;
  t.measured_position = 0.0f;
  t.position_error = 0.0f;
  t.speed = 0.0f;
  t.current_a = 0.0f;
  t.temperature_c = 0.0f;
  t.fault = Fault::kNone;
  t.last_seen_ms = 0;
  t.timeout_count = 0;
}

inline void telemetry_mark_seen(Telemetry& t, uint32_t now_ms) {
  t.online = true;
  t.last_seen_ms = now_ms;
}

// 距上次应答的毫秒数；从未应答返回 UINT32_MAX。
inline uint32_t telemetry_age_ms(const Telemetry& t, uint32_t now_ms) {
  if (t.last_seen_ms == 0) return UINT32_MAX;
  return now_ms - t.last_seen_ms;
}

inline bool telemetry_is_fresh(const Telemetry& t, uint32_t now_ms) {
  return t.online && t.measured_valid &&
         telemetry_age_ms(t, now_ms) <= kTelemetryFreshMs;
}

// 超过掉线窗口应判离线（调用方执行降沿通知）。
inline bool telemetry_should_mark_offline(const Telemetry& t, uint32_t now_ms) {
  if (!t.online || t.last_seen_ms == 0) return false;
  return telemetry_age_ms(t, now_ms) > kTelemetryOfflineMs;
}

}  // namespace x42s
