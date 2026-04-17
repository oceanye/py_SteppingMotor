# FOC 无刷电机集成设计

**日期**：2026-04-17
**状态**：已通过用户批准，待进入实施计划阶段
**作者**：brainstorm session (Claude Code, 用户 bimpub5)

---

## 1. 概述

将一台带 AS5600 磁编码器的 2208 无刷电机通过 SimpleFOC mini v1.0 驱动板集成到已有的 `py_SteppingMotor` 项目。使用**同一块 ESP32-S3** 同时控制现有的 DM422 步进电机和新增的 FOC 无刷电机。目标：**位置（角度）闭环**。

## 2. 目标与非目标

### 目标
- 位置闭环：指令给定目标角度（度），电机转到并保持
- 实时角度反馈：GUI 每 100ms 显示 AS5600 实测角度
- 电气安全：voltage_limit 软限幅、nFAULT 硬件故障监测
- 与现有步进系统共存：串口协议向后兼容，GUI 标签页并列
- 最小侵入：步进部分稳定代码**不动**；FOC 作为独立模块

### 非目标
- 速度环/力矩环独立控制（位置环内已含级联速度环，非目标是**不提供独立运行**）
- 电流环闭环（SimpleFOC mini v1.0 无电流采样，不做）
- 多轴扩展（只做单轴 FOC）
- 硬件限位开关（后续扩展，非本次范围）
- 梯形加减速规划（SimpleFOC 自带 velocity_limit 够用）

## 3. 系统架构

### 3.1 框图

```
                ┌────────────── ESP32-S3 (单板) ─────────────┐
  PC (GUI)      │  ┌─────────────────┐   ┌────────────────┐  │
  pc_gui.py ◀──USB─┤  Core 1: 串口/  │   │ Core 0: FOC    │  │
  (Notebook:    │  │  Stepper        │◀─▶│ 1kHz 控制环路   │  │
   Stepper/     │  │  (MOVE指令)     │   │ (SimpleFOC)    │  │
   FOC)         │  └────────┬────────┘   └──┬──────────┬──┘  │
                │           │               │          │     │
                │       PUL/DIR         I2C(SDA/SCL)  PWM×3  │
                └───────────┼───────────────┼──────────┼─────┘
                            ▼               ▼          ▼
                      ┌─────────┐     ┌──────────┐ ┌─────────────┐
                      │  DM422  │     │  AS5600  │ │ SimpleFOC   │
                      │ 步进驱动 │     │ 磁编码器 │ │ mini v1.0   │
                      └────┬────┘     └────┬─────┘ └──────┬──────┘
                           ▼               ▼              ▼ 3-phase
                      [24V→步进电机]  [装在2208轴端]   [12V→2208电机]
```

### 3.2 双核任务分工

- **Core 0**：FOC 闭环 FreeRTOS task（优先级 2）
  - `motor.loopFOC()` 内环 ~10kHz
  - `motor.move()` 外环位置 PID ~1kHz
  - `vTaskDelay(1)` 每周期
- **Core 1**：默认 `loop()`
  - 串口命令解析与分发
  - 步进 MOVE 阻塞脉冲（最多几秒）
  - nFAULT 轮询（100ms 周期）

**隔离原因**：FOC 闭环不能被阻塞；步进需要精确 `delayMicroseconds(50)`。双核物理隔离避免相互影响。

### 3.3 电源拓扑

```
USB(5V)  ───→ ESP32-S3 (含 3V3 LDO 输出)
                   │
                   ├─→ AS5600 VCC (3.3V)
                   └─→ SimpleFOC mini 3V3 (逻辑侧)

12V 独立电源 ─→ SimpleFOC mini VM (电机供电)
24V 独立电源 ─→ DM422 VCC (步进供电)

GND（强制共地）:  ESP32 ── AS5600 ── SimpleFOC ── DM422 ── 12V- ── 24V-
```

**三套电源完全独立**（USB / 12V / 24V），仅共享地线。

## 4. 硬件引脚分配

### 4.0 前置模组确认（本项目使用 ESP32-S3-WROOM-1 N16R8）

ESP32-S3 不同模组占用的 GPIO 不同，部署前必须确认：

| 模组 | 占用 GPIO | 11-14 是否可用 |
|---|---|---|
| WROOM-1 N4/N8 | 26-32 (Quad Flash) | ✅ 可用 |
| WROOM-1 N8R2 | 26-32 (Quad Flash + Quad PSRAM) | ✅ 可用 |
| **WROOM-1 N16R8（本项目）** | 26-32 (Flash) + 33-37 (Octal PSRAM) | ✅ **可用** |
| WROOM-2 N8R8V | 26-37 (Octal Flash + Octal PSRAM) | ⚠️ **不可用**，需换 15-18、21 |

若后期换模组，必须重检引脚表。


| 功能 | ESP32-S3 GPIO | 连接到 | 备注 |
|---|---|---|---|
| 步进 PUL          | 5   | DM422 PUL+ | 现有不动 |
| 步进 DIR          | 6   | DM422 DIR+ | 现有不动 |
| FOC PWM M1        | 11  | SimpleFOC mini M1 | 新增 |
| FOC PWM M2        | 12  | SimpleFOC mini M2 | 新增 |
| FOC PWM M3        | 13  | SimpleFOC mini M3 | 新增 |
| FOC EN            | 14  | SimpleFOC mini EN | 新增 |
| FOC nFAULT        | 10  | SimpleFOC mini nFT | 新增，输入+上拉 |
| I2C SDA           | 8   | AS5600 SDA | 新增 |
| I2C SCL           | 9   | AS5600 SCL | 新增 |
| 3V3               | —   | AS5600 VCC + SimpleFOC 3V3 | |
| GND               | —   | 共地（4 处短接）| |

**避开的危险引脚**：strapping（0/3/45/46）、USB（19/20）、Flash/PSRAM（26-32）。

**SimpleFOC mini 右侧针脚处理**：
- `nSP`（nSLEEP）悬空（板载上拉）
- `nRT`（nRESET）悬空（板载上拉）
- `3V3` 必须接到 ESP32 3V3
- `nFT` 接 GPIO 10 用于故障监控

## 5. 固件结构

### 5.1 文件组织

```
esp32_stepper/
├── platformio.ini          ← 增加 askuric/Simple FOC 依赖
└── src/
    ├── main.cpp            ← setup() + loop() + 任务创建
    ├── stepper.h / .cpp    ← 从现有 main.cpp 拆出的步进模块
    ├── foc_motor.h / .cpp  ← FOC 初始化/闭环/状态访问
    └── protocol.h / .cpp   ← 串口指令解析与分发
```

**拆分原则**：
- `stepper` 只暴露 `stepper_init()`, `stepper_move()`, `stepper_diag()` 三个函数
- `foc_motor` 封装 SimpleFOC 对象，暴露 `foc_enable()`, `foc_set_target()`, `foc_get_status()`, `foc_set_voltage_limit()`, `foc_home()` 等函数
- `protocol` 负责字符串解析；识别前缀分发到 `stepper` 或 `foc_motor`，完全不依赖硬件细节
- `main.cpp` 只做初始化和任务创建，不包含业务逻辑

### 5.2 SimpleFOC 初始化参数

```cpp
// foc_motor.cpp
MagneticSensorI2C sensor(AS5600_I2C);
BLDCMotor   motor(7);                           // 2208 的极对数假定 7
BLDCDriver3PWM driver(11, 12, 13, 14);          // M1, M2, M3, EN

void foc_init() {
  Wire.begin(8, 9);                             // SDA, SCL
  sensor.init();

  driver.voltage_power_supply = 12;             // 实际 PSU 电压
  driver.voltage_limit        = 12;             // 硬件侧不限幅
  driver.init();

  motor.linkSensor(&sensor);
  motor.linkDriver(&driver);

  motor.voltage_limit    = 3;                   // 软件限幅起步 3V → 0.3A
  motor.velocity_limit   = 20;                  // rad/s 防失控
  motor.controller       = MotionControlType::angle;
  motor.init();
  // motor.initFOC() 在 foc_enable(true) 时调用，上电默认不对齐
}
```

### 5.3 关键常量

| 常量 | 初值 | 备注 |
|---|---|---|
| `POLE_PAIRS` | 7 | 2208 典型值，若不对走校准回退 |
| `PSU_VOLTAGE` | 12 | V，与 §4 电源匹配 |
| `INITIAL_V_LIMIT` | 3 | V，启动安全值 |
| `MAX_V_LIMIT` | 10 | V，`FOC,V` 命令硬上限 |
| `MAX_ANGLE_ABS` | 3600 | °，`FOC,A` 目标绝对值上限 |
| `FAULT_POLL_MS` | 100 | Core 1 nFAULT 检查周期 |
| `FOC_TASK_STACK` | 4096 | FreeRTOS 任务栈 |
| `FOC_TASK_PRI` | 2 | FreeRTOS 优先级 |

### 5.4 共享状态线程安全（跨核通讯）

⚠️ **关键**：虽然 ESP32-S3 硬件对对齐 32-bit 读写是原子的，但 C++ 编译器可能缓存到寄存器/重排，导致读侧永远看不到新值。**不能裸访问 SimpleFOC 对象的成员**（如 `motor.target`、`motor.shaft_angle`）跨核。

正确做法：**镜像变量 + std::atomic**。Core 0 每个控制周期做"输入镜像 → SimpleFOC / SimpleFOC → 输出镜像"的拷贝。

```cpp
// foc_motor.cpp 文件级变量
static std::atomic<float>   g_target_deg    {0.0f};   // Core 1 写，Core 0 读
static std::atomic<float>   g_current_deg   {0.0f};   // Core 0 写，Core 1 读
static std::atomic<bool>    g_enable_req    {false};  // Core 1 写，Core 0 读
static std::atomic<bool>    g_fault_latched {false};  // 由 Core 1 (nFAULT 轮询) 置位；FOC,CLR 清零
static std::atomic<float>   g_voltage_limit {3.0f};   // Core 1 写，Core 0 读
static std::atomic<uint8_t> g_state         {STATE_DISABLED};  // FSM 当前态
```

Core 0 任务循环：

```cpp
void foc_task(void* param) {
  for (;;) {
    // 1. 输入镜像 → SimpleFOC
    motor.target         = g_target_deg.load()    * PI / 180.0f;
    motor.voltage_limit  = g_voltage_limit.load();

    // 2. 处理使能转换（见 §6.x 状态机）
    handle_state_transitions();

    // 3. 运行 FOC（RUNNING 态才跑闭环）
    sensor.update();                      // ⚠️ 无论使能与否都 update，保证 Core 1 能读到实时角度
    if (g_state.load() == STATE_RUNNING) {
      motor.loopFOC();
      motor.move();
    }

    // 4. 输出镜像
    g_current_deg.store(motor.shaft_angle * 180.0f / PI);

    vTaskDelay(1);  // 1ms 节拍
  }
}
```

Core 1 API（`foc_motor.h`）：

```cpp
void  foc_set_target_deg(float deg);   // → g_target_deg
float foc_get_current_deg();           // ← g_current_deg
void  foc_request_enable(bool en);     // → g_enable_req
bool  foc_is_fault_latched();          // ← g_fault_latched
void  foc_clear_fault();               // g_fault_latched = false
void  foc_set_voltage_limit(float v);  // → g_voltage_limit
uint8_t foc_get_state();               // ← g_state
```

**所有跨核访问都通过这组 atomic 镜像**，不直接读写 SimpleFOC 对象。

## 6. 串口协议

### 6.1 设计原则
- 现有 `MOVE` / `DIAG` **零修改**
- 新指令统一 `FOC,` 前缀
- 行分隔：`\n`，字段分隔：`,`
- 响应：单行 `OK` / `ERR:<reason>` / 数据行

### 6.2 命令表

| 命令 | 说明 | 响应 |
|---|---|---|
| `MOVE,<steps>,<dir>,<delay_ms>` | 步进脉冲（现有） | `OK` / `ERR:bad format` |
| `DIAG` | 步进自检（现有） | 多行文本 |
| `FOC,EN,1` | FOC 使能（首次：触发 `initFOC()` 对齐；非首次：直接 enable）| `OK` / `ERR:fault latched` |
| `FOC,EN,0` | FOC 失能（`motor.disable()`）| `OK` |
| `FOC,A,<deg>` | 设位置目标（度，float，范围 ±3600）| `OK` / `ERR:out of range` |
| `FOC,H` | 将当前角度定义为 0°（通过 sensor 偏移）| `OK` |
| `FOC,V,<volt>` | 在线修改 `motor.voltage_limit`（0 < v ≤ 10）| `OK` / `ERR:out of range` |
| `FOC,S` | 查询状态 | `FOC,S,<state>,<cur>,<tgt>,<fault>` |
| `FOC,CLR` | 清故障 latch（仅在 fault=1 时有效；执行后状态回到 `DISABLED`）| `OK` / `ERR:no fault` |
| `FOC,PP,<n>` | 设极对数并存入 NVS（`n` 范围 1-50）。下次启动生效 | `OK` / `ERR:out of range` |

### 6.2.1 FOC 状态机

```
              ┌──────────────────────────────────┐
              │                                  │
              ▼                                  │
  ┌──────────────────┐                          │
  │    DISABLED      │ ◀─── FOC,EN,0            │
  │ motor.disable()  │ ◀─── FOC,CLR (from FAULT)│
  │ 空闲，无电流      │                          │
  └────────┬─────────┘                          │
           │ FOC,EN,1（首次）                    │
           ▼                                     │
  ┌──────────────────┐                          │
  │    ALIGNING      │──── initFOC() 失败 ─────▶│ FAULT
  │ motor.initFOC()  │                          │
  │ 电机转 1-2 圈     │                          │
  └────────┬─────────┘                          │
           │ initFOC() 成功                      │
           ▼                                     │
  ┌──────────────────┐                          │
  │    RUNNING       │                          │
  │ motor.move() 循环│                          │
  │ 位置闭环          │──── nFAULT 拉低 ────────▶│
  └────────┬─────────┘                          │
           │ FOC,EN,0                            │
           ▼                                     ▼
           (回 DISABLED)                  ┌──────────────┐
                                          │    FAULT     │
                                          │ motor.disable│
                                          │ 锁存，拒绝 EN │
                                          └──────┬───────┘
                                                 │ FOC,CLR
                                                 ▼
                                            (回 DISABLED)
```

**各状态下命令行为**：

| 状态 | `EN,1` | `EN,0` | `CLR` | `A,...` | `V,...` | `H` | `S` |
|---|---|---|---|---|---|---|---|
| `DISABLED` | → `ALIGNING`（或跳 `RUNNING` 若已对齐）| OK（noop）| `ERR:no fault` | OK（缓存，使能后生效）| OK | OK | OK |
| `ALIGNING` | OK（已在转）| → `DISABLED`，打断 align | `ERR:no fault` | OK（缓存）| OK | OK | OK |
| `RUNNING` | OK（noop）| → `DISABLED` | `ERR:no fault` | OK | OK | OK | OK |
| `FAULT` | `ERR:fault latched` | OK（noop）| → `DISABLED` | OK（缓存）| OK | OK | OK |

**状态标识**（`FOC,S` 响应里的 `<state>` 字段）：
- `0` = DISABLED
- `1` = ALIGNING
- `2` = RUNNING
- `3` = FAULT

### 6.3 状态回报格式

```
FOC,S,<state>,<cur_deg>,<tgt_deg>,<fault>
```

| 字段 | 类型 | 说明 |
|---|---|---|
| `state` | `0`/`1`/`2`/`3` | FSM 状态（见 §6.2.1）：0=DISABLED / 1=ALIGNING / 2=RUNNING / 3=FAULT |
| `cur_deg` | float（1 位小数）| SimpleFOC `shaft_angle` 换算的连续角度（含累计圈数，可超 ±360°）|
| `tgt_deg` | float（1 位小数）| 当前目标 |
| `fault` | `0`/`1` | nFAULT 锁存位，状态=3 时为 1；`FOC,CLR` 可清 |

**示例**：`FOC,S,2,45.3,90.0,0` 表示运行中、当前 45.3°、目标 90.0°、无故障。

### 6.4 错误类型

| 错误码 | 场景 |
|---|---|
| `ERR:bad format` | 参数数量/类型不对 |
| `ERR:unknown command` | 无法识别的指令 |
| `ERR:out of range` | 角度 > ±3600°、电压 > 10V、极对数 ∉ [1,50] |
| `ERR:fault latched` | 故障态下 `FOC,EN,1` 被拒（必须先 `FOC,CLR`）|
| `ERR:no fault` | `FOC,CLR` 在非故障态调用 |

## 7. GUI 设计

### 7.1 窗口结构

```
┌─ 串口连接（共用置顶）──────────────────────┐
│  串口 COM4 ▼  波特率 115200 ▼  [连接]      │
└───────────────────────────────────────────┘
┌─ Notebook: [🔩 步进] [🧲 FOC 无刷]  ──────┐
│                                            │
│     (当前选中 Tab 的控件区)                 │
│                                            │
└───────────────────────────────────────────┘
┌─ 日志（共用置底）──────────────────────────┐
│  ...                                        │
└───────────────────────────────────────────┘
```

使用 `ttk.Notebook` 实现标签页切换。顶部连接与底部日志跨 Tab 共享。

### 7.2 步进 Tab

把当前 `pc_gui.py` 中的"运动参数 + 控制 + 位置与原点"三个 LabelFrame 原样迁移到 Tab 内。逻辑代码不动。

### 7.3 FOC Tab

```
┌─ 状态区 ─────────────────────────┐
│  使能: ○ 失能 / ● 使能           │
│  故障: ● 正常 / ❌ 报警           │
│  当前角度: 123.4° (100ms 刷新)   │
│  连接状态: 🟢在线 / ⏸步进中      │
└───────────────────────────────────┘
┌─ 目标控制 ────────────────────────┐
│  目标角度 (°): [   90.0] [前往]   │
│  快捷: [0°][45°][90°][180°][270°] │
│  增量: [+10°][+1°][-1°][-10°]     │
└───────────────────────────────────┘
┌─ 原点 ────────────────────────────┐
│  [⌂ 把当前位置设为 0°]            │
└───────────────────────────────────┘
┌─ 使能 / 调试 ─────────────────────┐
│  [使能 FOC]  (切换按钮)            │
│  电压限幅: [==|========] 3.0 V    │
│  极对数: [  7] [保存到NVS]         │
│  [🧹 清除故障]  (仅故障态可点)     │
└───────────────────────────────────┘
```

### 7.4 控件映射

| 控件 | 协议指令 |
|---|---|
| `[使能 FOC]` 切换 | `FOC,EN,1` / `FOC,EN,0` |
| `[前往]` | `FOC,A,<deg>` |
| 快捷按钮 | `FOC,A,<fixed>` |
| 增量按钮 | `FOC,A,<cur_target + delta>` |
| `[⌂ 设为 0°]` | `FOC,H` |
| 电压限幅滑条 | `FOC,V,<float>` |
| `[保存到NVS]`（极对数输入框配套）| `FOC,PP,<n>` |
| `[🧹 清除故障]` | `FOC,CLR` |
| 后台 100ms 轮询 | `FOC,S` |

### 7.5 状态门控

| 状态 | 启用的控件 |
|---|---|
| 未连接 | 只 `[连接]` |
| `DISABLED` | `[使能 FOC]` + 归零 + 电压限幅滑条 + 实时显示 |
| `ALIGNING` | 只显示 "对齐中…"，全部按钮灰 |
| `RUNNING` | 全部可用 |
| `FAULT` | **仅 `[清除故障]` 按钮可用**（专用按钮，单独发 `FOC,CLR`）；其他全灰；红色故障提示 |

**关键**：故障态必须保留至少一个可点击控件（清故障），否则 GUI 会死锁。清故障后状态回 `DISABLED`，恢复常规门控。

### 7.6 并发安全

PC 侧所有 `serial.Serial` 访问加单一锁：

```python
self.serial_lock = threading.Lock()
def _send_and_read(self, cmd) -> str:
    with self.serial_lock:
        self.ser.write(cmd.encode())
        return self.ser.readline().decode().strip()
```

FOC 轮询、MOVE、FOC 控制命令全部走此入口，避免交叉。

### 7.7 阻塞行为

步进 MOVE 期间（数秒），`loop()` 阻塞，FOC 状态查询排队但 Core 0 的 FOC 闭环照常运行。GUI 显示"⏸步进中"标记，MOVE 完成后自动恢复轮询。

## 8. 通电校准流程

分 4 道关卡，每关通过再进下一关。

### 关卡 1：仅 USB 供电 + 固件启动

只接 ESP32 USB，不给 SimpleFOC mini 上 12V。
- 烧录固件、打开串口
- 期望输出：`AS5600 detected at 0x36`，sensor angle 手动转轴时变化
- **电机不会动**，绝对安全
- 失败排查：SDA/SCL 接线、3V3 供电、磁铁位置

### 关卡 2：上 12V，不使能

- 发 `FOC,S` → 应回 `FOC,S,0,<cur>,0,0`（state=0=DISABLED）
- 手动转轴，`cur` 应变化

> **实现要求**：Core 0 的 FOC 任务即便在 `DISABLED` 态也必须调用 `sensor.update()`，否则 `shaft_angle` 不会刷新，`FOC,S` 将始终回固定值 —— 这不是关卡 2 通过的条件。

- `fault=0`
- 失败排查：电源干扰导致 ESP32 复位、M1/2/3 短路、sensor 任务未调度

### 关卡 3：首次使能 + initFOC 对齐

- 发 `FOC,EN,1`
- 期望：SimpleFOC 自动跑 `initFOC()`，电机缓慢转 1-2 圈完成电气零点对齐
- 串口打印 `PP check: PASS` + `Zero electric angle = X rad`
- 再试 `FOC,A,10` → 应平滑到位
- 失败与对策：

| 症状 | 对策 |
|---|---|
| `PP check: FAIL` | 走关卡 3B 极对数校准 |
| 疯转 / 尖叫 | 断电，交换 M1↔M2 任意两相 |
| 只抖不转 | 断电，对调 M1↔M2 或翻转磁铁 |
| Align 时发烫 | 降 `motor.voltage_limit` 至 2，重烧 |

### 关卡 3B：极对数校准兜底

临时用 SimpleFOC 自带的 `find_pole_pair_number.ino` 示例单独跑一次，读出真实极对数，改 `BLDCMotor motor(N)` 后重烧主固件，回关卡 3。

### 关卡 4：参数精修

在线通过 `FOC,V,<float>` 调 voltage_limit：

| 现象 | 调节 |
|---|---|
| 持续抖动/啸叫 | 降 `motor.P_angle.P` 从 20 → 15 |
| 到位慢/静差 | 升 P 到 25 或加 `LPF_velocity.Tf = 0.01` |
| 大角度急停 | 设 `motor.velocity_limit = 15` rad/s |
| 扭矩不足 | 逐档上调 voltage_limit 3→4→5→6，观察温度 |

## 9. 安全机制

| 机制 | 实现 |
|---|---|
| voltage_limit 上限 | `FOC,V` 拒绝 > 10V |
| nFAULT 硬件监测 | Core 1 每 100ms 读 GPIO10，拉低则 `motor.disable()` + 串口报警 |
| AS5600 掉线 | Core 0 检测 sensor 500ms 无变化且 motor enabled → disable |
| 故障锁存 | 一旦 fault=1，必须 `FOC,EN,1` 才能清除 |
| 软件看门狗 | Core 0 任务每圈喂狗，卡死自动重启 |
| 目标角度上限 | `FOC,A` 拒绝 `|deg| > 3600` |

## 10. 依赖与环境

### 10.1 PlatformIO 新增依赖

```ini
lib_deps =
  askuric/Simple FOC @ ^2.3.4
```

`Wire` 和 `MagneticSensorI2C`（含 AS5600 预设）都在 SimpleFOC 库内。

### 10.2 PC 端

现有 Python 环境（pyserial + tkinter）已够用，无新增依赖。

## 11. 采购清单

- [x] ESP32-S3 DevKitC-1（已有）
- [x] DM422 驱动 + 步进电机（已有）
- [x] SimpleFOC mini v1.0（已有）
- [x] 2208 无刷电机 + AS5600（已有）
- [ ] **12V / 2A 直流电源**（桶头或端子接线，待购）
- [x] 杜邦线若干
- [x] 共地用导线

## 12. 预期工作量

| 阶段 | 估计时间 |
|---|---|
| 固件模块化重构 | 1.5 h |
| FOC 模块实现 | 2 h |
| 协议解析扩展 | 1 h |
| GUI 标签页改造 | 2 h |
| 首次通电调试（含校准）| 1-3 h |
| 参数精修 | 0.5-1 h |
| **总计** | **8-10 小时** |

## 13. 风险与后续扩展

### 风险
- SimpleFOC mini v1.0 的具体引脚标注可能与假设不完全一致（需实物确认）
- 2208 极对数可能不是 7（已准备回退流程）
- AS5600 磁铁距离/极性安装不当会导致读数跳变（初次上电需观察）

### 后续扩展（不在本次范围）
- 硬件限位开关 / 真绝对原点
- 位置持久化到 ESP32 NVS（掉电不丢）
- 多轴（重复 PWM 引脚 + 扩展指令 `FOC,<axis>,A,<deg>`）
- 图形化角度曲线示波器

### 明确**不做**
- **双电机联动**（步进线性位置与 FOC 角度的协调运动）—— 用户明确声明不需要，两个电机作为独立子系统，在同一 GUI 里各自操作即可

## 14. 变更记录

| 日期 | 变更 |
|---|---|
| 2026-04-17 | 初版，brainstorm 完成，用户批准 |
| 2026-04-17 | 经独立审阅修订：§4 增加模组确认表（N16R8）；§5.4 改用 `std::atomic` 镜像变量跨核；§6.2 拆分 `FOC,EN` 语义 + 新增 `FOC,CLR`/`FOC,PP` + 状态机图；§6.3 `<state>` 字段取代 `<en>`；§7.5 故障态保留清故障按钮；§8 明示 Core 0 `sensor.update()` 在 DISABLED 态也必须运行 |
