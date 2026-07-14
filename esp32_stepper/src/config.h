// esp32_stepper/src/config.h
#pragma once

// 编译模式由 platformio.ini 的 build_flags 提供：
//   -DDRIVE_MODE_FOC   → 默认，步进 + FOC 双轴
//   -DDRIVE_MODE_GEAR  → Phase 1 新板，DRV8871 + N20 单轴（无步进、无 FOC）
#if !defined(DRIVE_MODE_FOC) && !defined(DRIVE_MODE_GEAR)
  #error "Build flag missing: define DRIVE_MODE_FOC or DRIVE_MODE_GEAR in platformio.ini"
#endif

// ============================================================
// 共用常量
// ============================================================
#define FAULT_POLL_MS            100

// ============================================================
// FOC 模式（原有双轴：步进 + BLDC + AS5600）
// ============================================================
#if defined(DRIVE_MODE_FOC)

#define NUM_AXES 2

// ── 步进（PUL / DIR 每轴一对）──
#define PIN_STEP_PUL_0     5
#define PIN_STEP_DIR_0     6
#define PIN_STEP_PUL_1     7
#define PIN_STEP_DIR_1    15

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
#define PIN_I2C_SDA_0      8
#define PIN_I2C_SCL_0      9
#define PIN_I2C_SDA_1     42
#define PIN_I2C_SCL_1     41

// ── FOC 常数 ──
#define FOC_POLE_PAIRS_DEFAULT   7
#define FOC_PSU_VOLTAGE          24.0f
#define FOC_INITIAL_V_LIMIT      10.0f
#define FOC_MAX_V_LIMIT          24.0f
#define FOC_MAX_ANGLE_ABS        3600.0f
#define FOC_VELOCITY_LIMIT       5.0f
#define FOC_TASK_STACK           4096
#define FOC_TASK_PRIORITY        2
#define FOC_TASK_CORE            0

#endif  // DRIVE_MODE_FOC

// ============================================================
// GEAR 模式（Phase 1：DRV8871 + N20 减速电机，单轴起步）
// ============================================================
#if defined(DRIVE_MODE_GEAR)

#define NUM_AXES 2   // 步进 L/R 都用；GEAR 电机当前只接了 axis 0，axis 1 留占位 pin

// ── 步进（PUL/DIR 每轴一对，和 FOC 模式同 pin） ──
#define PIN_STEP_PUL_0     5
#define PIN_STEP_DIR_0     6
#define PIN_STEP_PUL_1     7
#define PIN_STEP_DIR_1    15

// ── 减速电机 axis 0（已接好，2026-05-22 调通） ──
// 物理上 ESP32 GPIO 11 → DRV8871 IN1，GPIO 16 → DRV8871 IN2。
// 在固件里把 IN1/IN2 调换，PID 命令"正转"时其实驱动物理 IN2 → 电机反向，
// 使得 +PID 输出 ↔ 编码器 +counts 一致（避免闭环 runaway）。
#define PIN_GEAR_IN1_0    16   // ← 物理 IN2 当作 PID 的 IN1（反向）
#define PIN_GEAR_IN2_0    11   // ← 物理 IN1 当作 PID 的 IN2
// 编码器 A/B 在固件里互换：因为物理 C1 在 GPIO 18，C2 在 GPIO 21，
// 但 PCNT 解出的方向和 PID 期望相反 → 这里反过来声明，让正转 = +deg
#define PIN_GEAR_ENCA_0   21   // ← 物理 C2 当作 A
#define PIN_GEAR_ENCB_0   18   // ← 物理 C1 当作 B

// ── 减速电机 axis 1（占位，未接硬件；将来装 R 侧 N20 时用） ──
// 选 ESP32-S3 上确认空闲的 GPIO，不和步进 / axis 0 GEAR / USB / strapping 冲突。
#define PIN_GEAR_IN1_1    17   // 备选
#define PIN_GEAR_IN2_1    12
#define PIN_GEAR_ENCA_1   14
#define PIN_GEAR_ENCB_1   13

// ── PWM 配置（LEDC）──
#define GEAR_PWM_FREQ_HZ        20000   // 20 kHz，超声不刺耳
#define GEAR_PWM_RES_BITS       8       // 0–255
#define GEAR_PWM_MAX            ((1 << GEAR_PWM_RES_BITS) - 1)
#define GEAR_PWM_DUTY_CAP_PCT   100     // 默认 100%（已用独立 12V 源；若 VM=24V，改回 50）
#define GEAR_LEDC_CH_IN1_0      0
#define GEAR_LEDC_CH_IN2_0      1
#define GEAR_LEDC_CH_IN1_1      2
#define GEAR_LEDC_CH_IN2_1      3

// ── 编码器换算 ──
#define GEAR_RATIO_DEFAULT      1000.0f                  // N20 1000:1
#define GEAR_ENC_PPR_BASE       7                        // 电机轴端 PPR
#define GEAR_ENC_QUAD_MULT      4                        // 正交 X4 解码
// 每输出轴圈计数 = 7 × 4 × 1000 = 28000；1° ≈ 77.78 counts（默认齿轮比）

// ── PID 默认值（一般需要调）──
#define GEAR_DEFAULT_KP         1.0f
#define GEAR_DEFAULT_KI         0.0f
#define GEAR_DEFAULT_KD         0.05f
#define GEAR_INTEGRATOR_CLAMP   100.0f
#define GEAR_MAX_ANGLE_ABS      3600.0f

// ── 控制环任务参数 ──
#define GEAR_TASK_STACK         4096
#define GEAR_TASK_PRIORITY      2
#define GEAR_TASK_CORE          0
#define GEAR_TASK_TICK_MS       1       // 1 kHz 闭环

#endif  // DRIVE_MODE_GEAR
