#pragma once

// Configure the optional GPIO hardware-estop input.  These functions are
// harmless no-ops in builds where the input is not available or is disabled.
void hardware_estop_init();
bool hardware_estop_is_active();
