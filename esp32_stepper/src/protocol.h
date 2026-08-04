// esp32_stepper/src/protocol.h
#pragma once
#include <Arduino.h>

// 处理一行串口输入（不含换行），内部分发到 stepper 或后续 foc 模块。
// 调用方确保已 trim。响应文本（OK / ERR:... / 数据行）通过 Serial 输出。
void protocol_handle_line(const String& line);

// 原子停止全部步进/轨道并失能所有闭环轴（ESTOP 命令与硬件急停共用同一停止链）。
void system_estop();
