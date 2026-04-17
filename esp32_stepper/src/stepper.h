// esp32_stepper/src/stepper.h
#pragma once

void stepper_init();
// steps>0, direction 0 或 1, delay_ms>0
// 阻塞执行完整脉冲串后返回
void stepper_move(int steps, int direction, int delay_ms);
// 完整自检序列（DIAG 指令）
void stepper_run_diagnostics();
