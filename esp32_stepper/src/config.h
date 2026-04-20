// esp32_stepper/src/config.h
#pragma once

// ── 步进（现有，不变）──
#define PIN_STEP_PUL      5
#define PIN_STEP_DIR      6

// ── FOC 无刷（新增，阶段 B 使用）──
#define PIN_FOC_M1        11
#define PIN_FOC_M2        12
#define PIN_FOC_M3        13
#define PIN_FOC_EN        14
#define PIN_FOC_NFAULT    10
#define PIN_I2C_SDA       8
#define PIN_I2C_SCL       9

// ── FOC 常数 ──
#define FOC_POLE_PAIRS_DEFAULT   7      // 2208 典型
#define FOC_PSU_VOLTAGE          24.0f   // 与 DM422 共用单 24V 电源
#define FOC_INITIAL_V_LIMIT      10.0f   // 启动默认，喂给 2208 电机的最大电压
#define FOC_MAX_V_LIMIT          12.0f   // ⚠️ 电机额定上限 12V，不能超（即使 PSU=24V）
#define FOC_MAX_ANGLE_ABS        3600.0f
#define FOC_VELOCITY_LIMIT       5.0f   // rad/s，配 V=10V+VP=0.4 防过冲
#define FOC_TASK_STACK           4096
#define FOC_TASK_PRIORITY        2
#define FOC_TASK_CORE            0
#define FAULT_POLL_MS            100
