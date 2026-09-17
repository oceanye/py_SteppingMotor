// esp32_stepper/src/config.h
#pragma once

// 编译模式由 platformio.ini 的 build_flags 提供：
//   -DDRIVE_MODE_FOC   → 旧板：双路步进 + 双路 FOC
//   -DDRIVE_MODE_GEAR  → 默认新板：六路步进 + 双路闭环减速电机 + 轨道 D
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
// GEAR 模式（六路 DM422/DM442 + 两路闭环 DRV8871 + 一路轨道 D）
// ============================================================
#if defined(DRIVE_MODE_GEAR)

#define NUM_AXES 6          // 步进轴数（6 × DM422）
#define NUM_GEAR_AXES 2     // GEAR 减速电机轴数（DRV8871，不变）

// ── 步进（PUL/DIR 每轴一对）──
#define PIN_STEP_PUL_0     5
#define PIN_STEP_DIR_0     6
#define PIN_STEP_PUL_1     7
#define PIN_STEP_DIR_1    15
#define PIN_STEP_PUL_2     1
#define PIN_STEP_DIR_2     2
#define PIN_STEP_PUL_3     4
#define PIN_STEP_DIR_3     8
#define PIN_STEP_PUL_4     9
#define PIN_STEP_DIR_4    10
#define PIN_STEP_PUL_5    38
#define PIN_STEP_DIR_5    39

// ── 步进驱动器 ENA(使能/释放)控制 ──
// 2026-09-17 新增：仅两个旋转轴(2=左侧转, 3=右侧转)的 DM442 接了 ENA 线，
// 供"悬空腿落地时释放旋转电机纠偏"使用。接法与 PUL/DIR 相同：
//   ENA+ → GPIO，ENA- → GND。DM442：ENA 光耦导通 = 线圈断电(释放)，
//   即 高电平=释放、低电平=锁定(保持力矩在)。
// 引脚选择：WROOM-1 模组不引出 GPIO22-34，右转不能用 33；空闲脚只剩
// 预留位(35=急停/42,47,48=编码器I2C或RS485/36,37=nFAULT)。选 36/37 中
// 的 36：DRV8871 成品模块未引出 nFAULT，该预留保留价值最低。
// ⚠ 若模组丝印为 N16R8(Octal PSRAM)，GPIO35-37 被 PSRAM 占用，
//   须换 42/47/48 并放弃对应预留功能。
// 其余轴未接线：引脚表填 -1，ENA 命令对它们回复 unsupported。
#define STEPPER_ENA_LOCKED_LEVEL    LOW
#define STEPPER_ENA_RELEASED_LEVEL  HIGH
#define PIN_STEP_ENA_0  -1
#define PIN_STEP_ENA_1  -1
#define PIN_STEP_ENA_2   3   // 左侧转 DM442 ENA+
#define PIN_STEP_ENA_3  36   // 右转 DM442 ENA+（占用 nFAULT_1 预留）
#define PIN_STEP_ENA_4  -1
#define PIN_STEP_ENA_5  -1

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
// 载板实体接线为 IN1=GPIO12、IN2=GPIO17、C1/A=GPIO13、C2/B=GPIO14；
// 和 axis 0 一样，这里的逻辑 IN1/IN2、ENCA/ENCB 为统一 PID 正方向而交换。
#define PIN_GEAR_IN1_1    17   // → 实体 IN2
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

// ── 轨道 D 开环直流电机（第三块 DRV8871，仅 GEAR 构建）──
// 这是独立的租约式开环输出，不占用两路闭环 GEAR 轴。
#define PIN_TRACK_D_IN1         40
#define PIN_TRACK_D_IN2         41
#define TRACK_D_LEDC_CH_IN1      4
#define TRACK_D_LEDC_CH_IN2      5
#define TRACK_PWM_FREQ_HZ       20000
#define TRACK_PWM_RES_BITS       8
#define TRACK_PWM_MAX           ((1 << TRACK_PWM_RES_BITS) - 1)
#define TRACK_DEFAULT_LEASE_MS  1000
#define TRACK_MIN_LEASE_MS       100
#define TRACK_MAX_LEASE_MS      5000
#define TRACK_SAFETY_TICK_MS      10

// ── 可选六路步进轴绝对编码器诊断总线（不参与步进控制）──
// GPIO42/47 与载板 AUX UART/RS485 共用，使用本功能时不得同时安装/启用 UART、
// RS485 或其他占用这两个 GPIO 的模块。长线 I2C 易受电机噪声干扰，应使用短线、
// 合理上拉、共地，必要时降低时钟或采用差分 I2C 延长器。
#define STEPPER_ENCODER_DIAGNOSTICS_ENABLED  0  // 编码器尚未安装；安装后显式改为 1
#define STEPPER_ENCODER_CLOSED_LOOP_ENABLED  0  // 预留；当前 DM442 始终原生 PUL/DIR 开环
#define NUM_STEPPER_ENCODERS                  6
#define PIN_STEPPER_ENCODER_I2C_SDA          42
#define PIN_STEPPER_ENCODER_I2C_SCL          47
#define STEPPER_ENCODER_I2C_HZ           100000
#define TCA9548A_I2C_ADDR                   0x70
#define AS5600_I2C_ADDR                     0x36
#define AS5600_RAW_ANGLE_REG                0x0C
#define AS5600_STATUS_REG                   0x0B
#define STEPPER_ENCODER_POLL_MS               20
#define STEPPER_ENCODER_OFFLINE_RETRY_MS    1000
#define STEPPER_ENCODER_TASK_STACK          3072
#define STEPPER_ENCODER_TASK_PRIORITY          1
#define STEPPER_ENCODER_TASK_CORE              0

// PCF8575 仅预留地址和未来诊断入口；当前 DIR/PUL 路径绝不经过 PCF8575。
#define PCF8575_DIAGNOSTICS_ENABLED           0
#define PCF8575_I2C_ADDR                    0x20
#if STEPPER_ENCODER_CLOSED_LOOP_ENABLED
  #error "Closed-loop DM442 control is not implemented; keep PUL/DIR open-loop"
#endif

// Optional RS485 master for six Raspberry Pi Pico (RP2040) stepper nodes.
// Local axes remain 0..5; nodes 1..6 add global axes 6..29 (four per node).
// GPIO42/47 are shared with the optional diagnostic I2C bus above, so the two
// features are intentionally compile-time exclusive.
#ifndef REMOTE_STEPPER_ENABLED
  #define REMOTE_STEPPER_ENABLED              0
#endif
#define REMOTE_STEPPER_NODE_COUNT              6
#define REMOTE_STEPPER_AXES_PER_NODE            4
#define REMOTE_STEPPER_FIRST_AXIS               NUM_AXES
#define REMOTE_STEPPER_TOTAL_AXES              (NUM_AXES + REMOTE_STEPPER_NODE_COUNT * REMOTE_STEPPER_AXES_PER_NODE)
#define PIN_REMOTE_STEPPER_RX                   47
#define PIN_REMOTE_STEPPER_TX                   42
#define PIN_REMOTE_STEPPER_DE                   48
#define REMOTE_STEPPER_BAUD                 115200
#define REMOTE_STEPPER_HEARTBEAT_MS            250
#define REMOTE_STEPPER_NODE_TIMEOUT_MS        1000
#define REMOTE_STEPPER_RESPONSE_TIMEOUT_MS      35
#define REMOTE_STEPPER_POLL_INTERVAL_MS          8
#if REMOTE_STEPPER_ENABLED && STEPPER_ENCODER_DIAGNOSTICS_ENABLED
  #error "RS485 remote steppers and GPIO42/47 diagnostic I2C cannot be enabled together"
#endif

// ── 可选硬件急停输入（常闭 NC 接点：GPIO35 ↔ GND + 内部上拉）──
// NC 接点在安全态导通，因此安全态=低；按下或断线后由内部上拉变高=触发。
// 未安装 NC 回路时必须保持 0，否则悬空/未接线会按失效安全原则触发急停。
#ifndef HW_ESTOP_ENABLED
  #define HW_ESTOP_ENABLED      0
#endif
#define PIN_HW_ESTOP            35
#define HW_ESTOP_ACTIVE_HIGH    1   // NC+PULLUP：1=高电平有效（按下或断线）
// ── 可选 GEAR DRV8871 nFAULT 故障输入（active-low 开漏）──
// DRV8871 成品模块是否引出 nFAULT 未确认，默认 0 不占用 GPIO36/37；确认模块有 nFAULT
// 引脚并接线后改为 1，固件在 loop() 轮询并 foc_latch_fault()。
#define GEAR_NFAULT_ENABLED     0
#define PIN_GEAR_NFAULT_0       36
#define PIN_GEAR_NFAULT_1       37

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
