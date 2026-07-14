// esp32_stepper/src/stepper.h
#pragma once

// 多轴：所有函数都按 axis 索引（0=L, 1=R），axis 超界返回 false / 忽略
void stepper_init();

// 同步版本：阻塞执行脉冲串，末尾 Serial.println("OK,<axis>")。DIAG 内部用。
void stepper_move(int axis, int steps, int direction, int delay_us);

// 异步版本（协议分发用）：立即返回；后台任务执行脉冲。
// 完成时发 "STEP,<axis>,DONE"。返回 true = 已接受，false = 该轴 busy 或 axis 非法。
bool stepper_move_async(int axis, int steps, int direction, int delay_us);

// 某轴后台任务是否在执行
bool stepper_is_busy(int axis);

// 完整自检序列（DIAG,<axis> 指令）
void stepper_run_diagnostics(int axis);
