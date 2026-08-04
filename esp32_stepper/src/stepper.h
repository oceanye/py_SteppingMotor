// esp32_stepper/src/stepper.h
#pragma once

// 多轴：所有函数都按 axis 索引（0=L, 1=R），axis 超界返回 false / 忽略
void stepper_init();

// 同步版本：阻塞执行脉冲串，末尾 Serial.println("OK,<axis>")。DIAG 内部用。
void stepper_move(int axis, int steps, int direction, int delay_us);

// 异步版本（协议分发用）：立即返回；后台任务执行脉冲。
// 完成时发 "STEP,<axis>,DONE,<executed>,<requested>"；中止则发 ABORT 和实际步数。
// 返回 true = 已接受，false = 参数非法或该轴 busy。
bool stepper_move_async(int axis, int steps, int direction, int delay_us);

// 某轴后台任务是否在执行
bool stepper_is_busy(int axis);

// 中止某轴正在执行的脉冲运动（紧急停止用）
// 返回 true 表示确实截获了一个尚未结束的运动；false 表示轴非法或已空闲。
bool stepper_abort(int axis);

// 完整自检序列（DIAG,<axis> 指令）
void stepper_run_diagnostics(int axis);
