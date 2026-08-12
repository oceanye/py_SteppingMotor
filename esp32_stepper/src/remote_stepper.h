#pragma once

#include <Arduino.h>

void remote_stepper_init();
void remote_stepper_tick();
bool remote_stepper_move(int global_axis, int steps, int direction, int delay_us);
bool remote_stepper_stop(int global_axis);
void remote_stepper_stop_all();
bool remote_stepper_is_busy(int global_axis);
bool remote_stepper_run_diagnostics(int global_axis);
void remote_stepper_estop();
void remote_stepper_print_status(int node);  // node 1..6, or 0 for all
const char* remote_stepper_last_error();
