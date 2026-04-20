// esp32_stepper/src/stepper.h
#pragma once

void stepper_init();

// 同步版本：阻塞执行脉冲串，末尾 Serial.println("OK")。DIAG 和单元测试用。
void stepper_move(int steps, int direction, int delay_ms);

// 异步版本（协议分发用）：立即返回；后台任务执行脉冲，完成时 Serial.println("STEP,DONE")。
// 返回 true = 已接受，false = 有任务未完成（busy）
bool stepper_move_async(int steps, int direction, int delay_ms);
// 查询后台任务是否在执行
bool stepper_is_busy();

// 完整自检序列（DIAG 指令）
void stepper_run_diagnostics();
