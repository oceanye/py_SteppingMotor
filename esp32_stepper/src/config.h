// esp32_stepper/src/config.h
#pragma once

// ── 轴数（0=左腿 L，1=右腿 R）──
#define NUM_AXES 2

// ── 步进（PUL / DIR 每轴一对）──
#define PIN_STEP_PUL_0     5   // 左腿步进 PUL+
#define PIN_STEP_DIR_0     6   // 左腿步进 DIR+
#define PIN_STEP_PUL_1     7   // 右腿步进 PUL+
#define PIN_STEP_DIR_1    15   // 右腿步进 DIR+

// ── FOC 无刷（M1/M2/M3/EN/nFAULT 每轴一组）──
#define PIN_FOC_M1_0      11
#define PIN_FOC_M2_0      12
#define PIN_FOC_M3_0      13
#define PIN_FOC_EN_0      14
#define PIN_FOC_NFAULT_0  10

#define PIN_FOC_M1_1      16
#define PIN_FOC_M2_1      17
#define PIN_FOC_M3_1      18
#define PIN_FOC_EN_1      21
#define PIN_FOC_NFAULT_1   4

// ── I2C (AS5600 地址固定 0x36，两个 encoder 必须用两条总线) ──
#define PIN_I2C_SDA_0      8   // Wire（默认总线）
#define PIN_I2C_SCL_0      9
#define PIN_I2C_SDA_1     42   // Wire1（第二硬件总线）
#define PIN_I2C_SCL_1     41

// ── FOC 常数（所有轴共用初值）──
#define FOC_POLE_PAIRS_DEFAULT   7      // 2208 典型
#define FOC_PSU_VOLTAGE          24.0f  // 与 DM422 共用 24V 电源
#define FOC_INITIAL_V_LIMIT      10.0f
#define FOC_MAX_V_LIMIT          24.0f  // PSU=24V 满载（注意：>12V 长时间会烧 2208 电机）
#define FOC_MAX_ANGLE_ABS        3600.0f
#define FOC_VELOCITY_LIMIT       5.0f
#define FOC_TASK_STACK           4096
#define FOC_TASK_PRIORITY        2
#define FOC_TASK_CORE            0
#define FAULT_POLL_MS            100
