// X42S 执行器遥测模型（纯数据层，feature/x42s-uart-actuator-binding）.
//
// 与 PC 端 motor_control/actuator_model.py 保持同一语义：
// - command / measured / error 三值分开，未知必须用 *_valid=false 表达，
//   严禁把未知回退成 0；
// - 新鲜度窗口 200ms（10~20Hz 轮询）、掉线窗口 500ms；
// - 故障标签取自 §14 安全集。
// 本文件 header-only 纯函数：ESP32 固件与主机 native 测试共用。
// ✅ 状态字位定义已按手册 V1.0.5 核对（5.4.4 回零状态 / 5.5.15 电机状态）。
#pragma once

#include <stdint.h>

namespace x42s {

constexpr uint32_t kTelemetryFreshMs = 200;   // 与 PC 端 FRESH_MAX_AGE 一致
constexpr uint32_t kTelemetryOfflineMs = 500; // 与 PC 端 STALE_AGE 一致

// 电机状态字（5.5.15，3C 应答第 2 字节）。
constexpr uint8_t kStatusEnabled = 0x01;      // Ens_TF 使能状态
constexpr uint8_t kStatusReached = 0x02;      // Prf_TF 位置到达标志
constexpr uint8_t kStatusStallFlag = 0x04;    // Cgi_TF 堵转标志（未触发保护）
constexpr uint8_t kStatusStallProt = 0x08;    // Cgp_TF 堵转保护已触发
constexpr uint8_t kStatusLimitLeft = 0x10;    // Esi_LF 左限位电平
constexpr uint8_t kStatusLimitRight = 0x20;   // Esi_RF 右限位电平
constexpr uint8_t kStatusPowerLost = 0x80;    // Oac_TF 掉电标志（信息位）

// 回零状态字（5.4.4，3C 应答第 1 字节）。
constexpr uint8_t kOrgEncoderReady = 0x01;    // Enc_Rdy 编码器就绪
constexpr uint8_t kOrgCalibrated = 0x02;      // Cal_Rdy 校准表就绪
constexpr uint8_t kOrgInProgress = 0x04;      // Org_SF 正在回零
constexpr uint8_t kOrgFailed = 0x08;          // Org_CF 回零失败
constexpr uint8_t kOrgOverTemp = 0x10;        // Otp_TF 过热保护已触发
constexpr uint8_t kOrgOverCurrent = 0x20;     // Ocp_TF 过流保护已触发

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

// 双状态字 → 故障标签（手册 3C：[回零状态][电机状态]）。
// 过流/过热保护位在回零状态字（5.4.4），堵转在电机状态字（5.5.15）；
// 低压保护无独立状态位——触发低压警告/保护时驱动器以 E2 拒绝命令，
// 由总线层按"命令被拒"失败处理（不猜具体故障，状态位说话）；
// 限位电平与掉电标志是信息位，不映射故障。
inline Fault fault_from_status_words(uint8_t org_bits, uint8_t status_bits) {
  if (org_bits & kOrgOverCurrent) return Fault::kOverCurrent;
  if (org_bits & kOrgOverTemp) return Fault::kOverTemp;
  if (status_bits & (kStatusStallProt | kStatusStallFlag)) {
    return Fault::kStall;
  }
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
