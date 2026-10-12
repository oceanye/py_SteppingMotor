// esp32_stepper/src/x42s_uart.h
// X42S TTL-UART 总线驱动（§8.1 最低 API / §8.2 串口单 owner）。
//
// 串口所有权：所有 X42S 收发都发生在 Arduino 主任务上下文
// （loop() 的 x42s_uart_tick() 与 protocol_handle_line() 的命令分发），
// 同一执行流串行化 = 唯一写者，天然满足"两个 task 不得同时写同一
// UART / 回复不错配"（§8.2 的硬性要求）。不引入第二个 FreeRTOS task。
//
// ✅ 功能码/数据布局/状态字已按《ZDT X42S 手册 V1.0.5》核对
// （2026-09-29）。步数→角度换算依赖 config.h 的 X42S_PULSE_PER_REV，
// 须与驱动器细分设置一致。M5 台架验证通过前步态仍拒绝该后端（§19）。
#pragma once

#include <stdint.h>

#include "config.h"
#include "x42s_model.h"
#include "x42s_protocol.h"

#if defined(DRIVE_MODE_GEAR) && X42S_UART_ENABLED

// 初始化 UART（GPIO42 TX / GPIO47 RX）。不自动扫描——用 X42S,SCAN 探测。
void x42s_uart_init();

// 主循环轮询：配置的台号轮转读状态（X42S_POLL_INTERVAL_MS 节拍，
// 四台 ≈ 10Hz/台），并把超窗未应答的台降为 offline（带降沿事件行）。
void x42s_uart_tick();

// ---- §8.1 最低 API（返回 false 时查 x42s_last_error）----
bool x42s_ping(uint8_t id);
bool x42s_enable(uint8_t id, bool enable);
bool x42s_stop(uint8_t id);
// 急停：逐台单播停止+失能但只发不等应答（离线台等超时会拖爆 PC 0.8s
// ESTOP 握手，2026-10-12 台架）；此后必须显式重新使能（§14.1）。
void x42s_estop();
bool x42s_zero_position(uint8_t id);
bool x42s_move_relative(uint8_t id, int32_t steps, uint32_t speed_steps_s);
bool x42s_sync_trigger();
bool x42s_read_status(uint8_t id);  // 立即拉一次遥测并写缓存

// ---- 状态查询 / PC 协议（X42S,S / X42S,SCAN / X42S,DIAG）----
const x42s::Telemetry& x42s_telemetry(uint8_t id);
const x42s::CommStats& x42s_comm_stats();
void x42s_print_status_line(uint8_t id);  // X42S,<id>,S,...
void x42s_print_scan();                   // 探测 ID1..X42S_DRIVER_COUNT
void x42s_print_diag(uint8_t id);         // ping + 全量读 + 通讯统计

// 台架排障直通（X42S,RAW,<hex>）：原样发送任意帧，固定窗口收全部原始
// 回包并以 hex 打印（X42S,RAW,RX,<n>,<hex>）。绕过帧层解码——迟到的/
// 不匹配的应答在这里不会丢，专查"电机到底回了什么"。诊断用途，不走
// §15 业务闸；运动类帧经此发送时同样绕过使能检查，仅台架用。
void x42s_raw_probe(const uint8_t* frame, size_t len);
const char* x42s_last_error();

#else  // X42S 未启用：main.cpp / system_estop 的无条件调用点给空实现

inline void x42s_uart_init() {}
inline void x42s_uart_tick() {}
inline void x42s_estop() {}

#endif  // DRIVE_MODE_GEAR && X42S_UART_ENABLED
