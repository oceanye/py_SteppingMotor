# FOC 无刷电机集成 —— 实施计划

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在现有 ESP32-S3 上集成 SimpleFOC mini v1.0 + 2208 无刷电机 + AS5600 编码器，实现位置闭环控制，与现有 DM422 步进系统并存。

**Architecture:** 固件模块化拆分（stepper / foc_motor / protocol），双核任务分工（Core 0 跑 FOC 闭环，Core 1 跑串口+步进），跨核 atomic 镜像变量。GUI 改为 Notebook 标签页，FOC Tab 提供角度控制与实时状态轮询。

**Tech Stack:** PlatformIO, Arduino framework, SimpleFOC 2.3.4, FreeRTOS, unity (单元测试), Python 3.12, Tkinter ttk.Notebook, pyserial.

**Spec reference:** `docs/superpowers/specs/2026-04-17-foc-motor-integration-design.md`

---

## 阶段概览

| 阶段 | 内容 | 是否需要硬件 |
|---|---|---|
| A | 固件模块拆分（行为不变）| 软件 + 步进硬件回归 |
| B | 引入 SimpleFOC + FOC 模块骨架 | 不需要 |
| C | FOC 协议与状态机 | 不需要 |
| D | 硬件通电（4 关卡）| **需要** FOC 硬件 |
| E | GUI 改造（Notebook + FOC Tab） | 步进硬件回归 |
| F | 整合测试 + 文档 | 完整系统 |

---

# 阶段 A：固件模块拆分（行为不变）

现有 `main.cpp` 118 行，包含步进、DIAG、协议解析三块混合代码。先拆分为独立模块，**不改任何行为**，每步都通过硬件回归验证步进依然工作。

## Task A1：新建 `config.h` 集中常量

**Files:**
- Create: `esp32_stepper/src/config.h`

- [ ] **Step 1：新建文件，写入 GPIO 与常量**

```cpp
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
#define FOC_PSU_VOLTAGE          12.0f
#define FOC_INITIAL_V_LIMIT      3.0f
#define FOC_MAX_V_LIMIT          10.0f
#define FOC_MAX_ANGLE_ABS        3600.0f
#define FOC_VELOCITY_LIMIT       20.0f  // rad/s
#define FOC_TASK_STACK           4096
#define FOC_TASK_PRIORITY        2
#define FOC_TASK_CORE            0
#define FAULT_POLL_MS            100
```

- [ ] **Step 2：提交**

```bash
cd C:/Users/bimpub5/PycharmProjects/py_SteppingMotor
git add esp32_stepper/src/config.h
git commit -m "feat(esp32): add config.h with pin and FOC constants"
```

## Task A2：提取 `stepper` 模块

**Files:**
- Create: `esp32_stepper/src/stepper.h`
- Create: `esp32_stepper/src/stepper.cpp`
- Modify: `esp32_stepper/src/main.cpp`

- [ ] **Step 1：写 `stepper.h` 对外 API**

```cpp
// esp32_stepper/src/stepper.h
#pragma once

void stepper_init();
// steps>0, direction 0 或 1, delay_ms>0
// 阻塞执行完整脉冲串后返回
void stepper_move(int steps, int direction, int delay_ms);
// 完整自检序列（DIAG 指令）
void stepper_run_diagnostics();
```

- [ ] **Step 2：写 `stepper.cpp`，从 main.cpp 复制 moveMotor/runDiagnostics 实现**

将现有 `main.cpp` 中的 `moveMotor()` 改名为 `stepper_move()`，`runDiagnostics()` 改名为 `stepper_run_diagnostics()`；`stepper_init()` 负责 `pinMode` + `digitalWrite(LOW, LOW)`。引用 `config.h` 的引脚宏。

⚠️ **契约**：`stepper_move()` 完成后必须打印 `Serial.println("OK");`（保留现有行为；Task C2 的 protocol 分发**依赖**此契约，不再额外打印 OK）。

- [ ] **Step 3：修改 `main.cpp`，删除步进实现，调用 `stepper_*`**

在 `setup()` 调 `stepper_init()`。在 `loop()` 的 `MOVE,...` 分支调 `stepper_move(...)`；`DIAG` 分支调 `stepper_run_diagnostics()`。

- [ ] **Step 4：编译验证**

```bash
cd C:/Users/bimpub5/PycharmProjects/py_SteppingMotor/esp32_stepper
pio run
```

Expected: Build succeeds, no warnings about missing symbols.

- [ ] **Step 5：烧录并回归验证步进**

```bash
pio run -t upload
pio device monitor --port COM4 --baud 115200
```

在串口里发送：`MOVE,100,0,20`，应看到 `OK` 且电机走 2mm。发送 `DIAG`，应看到完整自检输出。

- [ ] **Step 6：提交**

```bash
git add esp32_stepper/src/stepper.h esp32_stepper/src/stepper.cpp esp32_stepper/src/main.cpp
git commit -m "refactor(esp32): extract stepper logic into stepper.{h,cpp}"
```

## Task A3：提取 `protocol` 模块

**Files:**
- Create: `esp32_stepper/src/protocol.h`
- Create: `esp32_stepper/src/protocol.cpp`
- Modify: `esp32_stepper/src/main.cpp`

- [ ] **Step 1：写 `protocol.h`**

```cpp
// esp32_stepper/src/protocol.h
#pragma once
#include <Arduino.h>

// 处理一行串口输入（不含换行），内部分发到 stepper 或后续 foc 模块。
// 调用方确保已 trim。响应文本（OK / ERR:... / 数据行）通过 Serial 输出。
void protocol_handle_line(const String& line);
```

- [ ] **Step 2：写 `protocol.cpp`，把现有 `loop()` 里的解析逻辑搬过来**

内部函数：
```cpp
static void handle_move(const String& line);  // MOVE,s,d,ms
static void handle_diag();                    // DIAG
```

- [ ] **Step 3：修改 `main.cpp` loop()，只负责读行并调用 `protocol_handle_line`**

```cpp
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) protocol_handle_line(cmd);
  }
}
```

- [ ] **Step 4：编译 + 烧录 + 步进回归（同 A2 Step 4-5）**

- [ ] **Step 5：提交**

```bash
git add esp32_stepper/src/protocol.h esp32_stepper/src/protocol.cpp esp32_stepper/src/main.cpp
git commit -m "refactor(esp32): extract serial protocol dispatching into protocol.{h,cpp}"
```

## Task A4：为 protocol 添加 unity 单元测试框架

**Files:**
- Create: `esp32_stepper/test/test_protocol/test_protocol.cpp`
- Modify: `esp32_stepper/platformio.ini`

**目的**：协议解析是纯字符串逻辑，可以在 PC 上用 unity 跑 native 测试（不需要硬件）。后续阶段 C 扩展 FOC 命令时能 TDD。

- [ ] **Step 1：在 `platformio.ini` 末尾追加 native 测试环境**

⚠️ **注意**：此处的 `build_src_filter` 行将在 Task C1 被**替换**（不是追加）。写入时先用空白 filter，后续 C1 再改。

```ini
[env:native_test]
platform = native
test_framework = unity
build_flags = -std=gnu++17
; C1 会把下面这行替换为 +<protocol_parser.cpp>
build_src_filter = -<*>
```

- [ ] **Step 2：写 `test_protocol.cpp` 的骨架 + 一个烟雾测试**

```cpp
// esp32_stepper/test/test_protocol/test_protocol.cpp
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

void test_smoke(void) {
    TEST_ASSERT_EQUAL_INT(2, 1 + 1);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_smoke);
    return UNITY_END();
}
```

**说明**：此时 protocol.cpp 依赖 Arduino.h，无法在 native 下编译。后续阶段 C 会拆出一个不依赖 Arduino 的 `protocol_parser.cpp` 做真正单元测试。此步仅验证 native 环境能跑。

- [ ] **Step 3：运行 native 测试**

```bash
cd C:/Users/bimpub5/PycharmProjects/py_SteppingMotor/esp32_stepper
pio test -e native_test
```

Expected: `1 Tests 0 Failures 0 Ignored`

- [ ] **Step 4：提交**

```bash
git add esp32_stepper/platformio.ini esp32_stepper/test/
git commit -m "test(esp32): bootstrap unity native test env for protocol module"
```

---

# 阶段 B：SimpleFOC 集成骨架（无硬件）

## Task B1：加入 SimpleFOC 依赖

**Files:**
- Modify: `esp32_stepper/platformio.ini`

- [ ] **Step 1：在 `[env:esp32s3]` 内加 `lib_deps`**

```ini
lib_deps =
  askuric/Simple FOC @ ^2.3.4
```

- [ ] **Step 2：编译验证（PlatformIO 会自动下载库）**

```bash
cd C:/Users/bimpub5/PycharmProjects/py_SteppingMotor/esp32_stepper
pio run
```

Expected: 下载 SimpleFOC 依赖，编译成功。警告 "large code size" 可忽略。

- [ ] **Step 3：提交**

```bash
git add esp32_stepper/platformio.ini
git commit -m "build(esp32): add SimpleFOC 2.3.4 dependency"
```

## Task B2：创建 `foc_motor` 模块骨架

**Files:**
- Create: `esp32_stepper/src/foc_motor.h`
- Create: `esp32_stepper/src/foc_motor.cpp`

- [ ] **Step 1：写 `foc_motor.h`**

```cpp
// esp32_stepper/src/foc_motor.h
#pragma once
#include <stdint.h>

// FSM state enumeration. Matches protocol FOC,S 第二字段。
enum FocState : uint8_t {
    FOC_STATE_DISABLED = 0,
    FOC_STATE_ALIGNING = 1,
    FOC_STATE_RUNNING  = 2,
    FOC_STATE_FAULT    = 3,
};

// 初始化 sensor/driver/motor；注册 Core 0 FOC 任务。
// 必须在 Serial.begin 之后调用。
void foc_init();

// ── Core 1 API（线程安全，从串口/主循环调用）──
// 设目标角度，单位：度。超范围返回 false。
bool foc_set_target_deg(float deg);
// 请求切换使能；若从 FAULT 调 true，返回 false（拒绝）。
bool foc_request_enable(bool en);
// 清除故障 latch。仅 FAULT 态返回 true。
bool foc_clear_fault();
// 在线设 voltage_limit。范围外返回 false。
bool foc_set_voltage_limit(float v);
// 把当前角度设为 0°。
void foc_home();
// 设极对数（1-50）并存 NVS，下次启动生效。
bool foc_set_pole_pairs_and_store(int n);

// ── 状态查询（从 Core 0 镜像读取）──
FocState foc_get_state();
float foc_get_current_deg();
float foc_get_target_deg();
bool foc_is_fault_latched();
```

- [ ] **Step 2：写 `foc_motor.cpp` 骨架（所有函数暂返回默认值）**

```cpp
// esp32_stepper/src/foc_motor.cpp
#include "foc_motor.h"
#include "config.h"
#include <atomic>

static std::atomic<float>    g_target_deg    {0.0f};
static std::atomic<float>    g_current_deg   {0.0f};
static std::atomic<bool>     g_enable_req    {false};
static std::atomic<bool>     g_fault_latched {false};
static std::atomic<float>    g_voltage_limit {FOC_INITIAL_V_LIMIT};
static std::atomic<uint8_t>  g_state         {FOC_STATE_DISABLED};

void foc_init() {
    // TODO B3: 初始化 SimpleFOC 对象 + 启动 Core 0 任务
}

bool foc_set_target_deg(float deg) {
    if (deg > FOC_MAX_ANGLE_ABS || deg < -FOC_MAX_ANGLE_ABS) return false;
    g_target_deg.store(deg);
    return true;
}
bool foc_request_enable(bool en) {
    if (en && g_state.load() == FOC_STATE_FAULT) return false;
    g_enable_req.store(en);
    return true;
}
bool foc_clear_fault() {
    if (g_state.load() != FOC_STATE_FAULT) return false;
    g_fault_latched.store(false);
    g_state.store(FOC_STATE_DISABLED);
    return true;
}
bool foc_set_voltage_limit(float v) {
    if (v <= 0 || v > FOC_MAX_V_LIMIT) return false;
    g_voltage_limit.store(v);
    return true;
}
void foc_home() {
    // TODO B3: sensor.update() + sensor.getAngle() 作为 offset
}
bool foc_set_pole_pairs_and_store(int n) {
    if (n < 1 || n > 50) return false;
    // TODO C4: 存 NVS
    return true;
}

FocState foc_get_state()       { return (FocState)g_state.load(); }
float    foc_get_current_deg() { return g_current_deg.load(); }
float    foc_get_target_deg()  { return g_target_deg.load(); }
bool     foc_is_fault_latched(){ return g_fault_latched.load(); }
```

- [ ] **Step 3：在 `main.cpp` setup() 末尾调 `foc_init()` 并编译**

```cpp
#include "foc_motor.h"
// ... setup() 末尾
foc_init();
```

- [ ] **Step 4：编译验证**

```bash
pio run
```

Expected: Build 成功。此时 foc_init 是 no-op，不影响步进行为。

- [ ] **Step 5：步进回归（同 A2 Step 5）**

确保步进依然工作。

- [ ] **Step 6：提交**

```bash
git add esp32_stepper/src/foc_motor.h esp32_stepper/src/foc_motor.cpp esp32_stepper/src/main.cpp
git commit -m "feat(foc): scaffold foc_motor module with atomic mirrors"
```

## Task B3：实现 SimpleFOC 初始化 + Core 0 任务

**Files:**
- Modify: `esp32_stepper/src/foc_motor.cpp`

- [ ] **Step 1：在 `foc_motor.cpp` 顶部加 SimpleFOC 对象**

```cpp
#include <Arduino.h>
#include <SimpleFOC.h>

static MagneticSensorI2C sensor(AS5600_I2C);
static BLDCMotor         motor(FOC_POLE_PAIRS_DEFAULT);
static BLDCDriver3PWM    driver(PIN_FOC_M1, PIN_FOC_M2, PIN_FOC_M3, PIN_FOC_EN);

static TaskHandle_t s_foc_task_handle = nullptr;
static float        s_home_offset_rad = 0.0f;
```

- [ ] **Step 2：实现 `foc_init()`**

```cpp
void foc_init() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    sensor.init();

    driver.voltage_power_supply = FOC_PSU_VOLTAGE;
    driver.voltage_limit        = FOC_PSU_VOLTAGE;
    driver.init();

    motor.linkSensor(&sensor);
    motor.linkDriver(&driver);
    motor.voltage_limit  = FOC_INITIAL_V_LIMIT;
    motor.velocity_limit = FOC_VELOCITY_LIMIT;
    motor.controller     = MotionControlType::angle;
    motor.init();
    // 不在这里调 initFOC()，等 foc_request_enable(true) 触发

    pinMode(PIN_FOC_NFAULT, INPUT_PULLUP);

    xTaskCreatePinnedToCore(foc_task, "FOC", FOC_TASK_STACK, nullptr,
                            FOC_TASK_PRIORITY, &s_foc_task_handle, FOC_TASK_CORE);
}
```

- [ ] **Step 3：实现 `foc_task()`（Core 0 主循环）**

```cpp
static void foc_task(void* /*param*/) {
    static bool s_aligned_once = false;   // 防止重复 initFOC()

    for (;;) {
        // 1. 读镜像 → SimpleFOC
        motor.target        = g_target_deg.load() * PI / 180.0f;
        motor.voltage_limit = g_voltage_limit.load();

        // 2. 状态转换
        FocState state = (FocState)g_state.load();
        bool want_en   = g_enable_req.load();

        if (state == FOC_STATE_DISABLED && want_en) {
            g_state.store(FOC_STATE_ALIGNING);
            state = FOC_STATE_ALIGNING;
        }
        if (state == FOC_STATE_ALIGNING) {
            if (!s_aligned_once) {
                motor.initFOC();      // 首次才跑对齐（阻塞 1-3 秒，电机缓转）
                s_aligned_once = true;
            }
            motor.enable();
            g_state.store(FOC_STATE_RUNNING);
            state = FOC_STATE_RUNNING;
        }
        if ((state == FOC_STATE_RUNNING || state == FOC_STATE_ALIGNING) && !want_en) {
            motor.disable();
            g_state.store(FOC_STATE_DISABLED);
            state = FOC_STATE_DISABLED;
        }

        // 3. 闭环（仅 RUNNING 跑）
        sensor.update();  // 任何态都更新，保证角度镜像有效
        if (state == FOC_STATE_RUNNING && !g_fault_latched.load()) {
            motor.loopFOC();
            motor.move();
        }

        // 4. 更新输出镜像
        float shaft = motor.shaft_angle - s_home_offset_rad;
        g_current_deg.store(shaft * 180.0f / PI);

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}
```

需要在 foc_task 之前声明（或定义到使用之前）。

- [ ] **Step 4：实现 `foc_home()`**

```cpp
void foc_home() {
    s_home_offset_rad = motor.shaft_angle;  // Core 1 读，原子读 float OK
}
```

- [ ] **Step 5：编译验证**

```bash
pio run
```

Expected: Build 成功，lib deps 下载完整，无链接错误。

- [ ] **Step 6：提交（不烧录，硬件阶段再测）**

```bash
git add esp32_stepper/src/foc_motor.cpp
git commit -m "feat(foc): implement SimpleFOC init and Core 0 FSM task"
```

---

# 阶段 C：FOC 协议与状态机

## Task C1：抽出纯解析器（不依赖 Arduino）

**Files:**
- Create: `esp32_stepper/src/protocol_parser.h`
- Create: `esp32_stepper/src/protocol_parser.cpp`

**目的**：把字符串解析从 Arduino String 操作里剥离，用 `std::string` + `std::string_view`，以便 native 单元测试。

- [ ] **Step 1：写 `protocol_parser.h`**

```cpp
// esp32_stepper/src/protocol_parser.h
#pragma once
#include <string>
#include <optional>

enum CmdType {
    CMD_UNKNOWN, CMD_MOVE, CMD_DIAG,
    CMD_FOC_EN, CMD_FOC_ANGLE, CMD_FOC_HOME,
    CMD_FOC_VOLTAGE, CMD_FOC_STATUS,
    CMD_FOC_CLEAR, CMD_FOC_POLEPAIRS,
};

struct ParsedCmd {
    CmdType type = CMD_UNKNOWN;
    // 对应字段（仅相关字段有效）
    int    move_steps = 0, move_dir = 0, move_delay = 0;
    int    foc_en = 0;           // 0/1
    float  foc_angle = 0.0f;
    float  foc_voltage = 0.0f;
    int    foc_pole_pairs = 0;
    // 错误：返回 type=CMD_UNKNOWN + reason
    std::string error_reason;
};

ParsedCmd parse_command(const std::string& line);
```

- [ ] **Step 2：写测试**（TDD，先写失败的测试）

```cpp
// esp32_stepper/test/test_protocol/test_protocol.cpp (覆盖之前的)
#include <unity.h>
#include "../../src/protocol_parser.h"

void test_move_valid(void) {
    ParsedCmd c = parse_command("MOVE,100,0,20");
    TEST_ASSERT_EQUAL(CMD_MOVE, c.type);
    TEST_ASSERT_EQUAL_INT(100, c.move_steps);
    TEST_ASSERT_EQUAL_INT(0, c.move_dir);
    TEST_ASSERT_EQUAL_INT(20, c.move_delay);
}
void test_move_bad_format(void) {
    ParsedCmd c = parse_command("MOVE,100,0");
    TEST_ASSERT_EQUAL(CMD_UNKNOWN, c.type);
    TEST_ASSERT_EQUAL_STRING("bad format", c.error_reason.c_str());
}
void test_diag(void) {
    TEST_ASSERT_EQUAL(CMD_DIAG, parse_command("DIAG").type);
}
void test_foc_en_1(void) {
    ParsedCmd c = parse_command("FOC,EN,1");
    TEST_ASSERT_EQUAL(CMD_FOC_EN, c.type);
    TEST_ASSERT_EQUAL_INT(1, c.foc_en);
}
void test_foc_en_0(void) {
    ParsedCmd c = parse_command("FOC,EN,0");
    TEST_ASSERT_EQUAL(CMD_FOC_EN, c.type);
    TEST_ASSERT_EQUAL_INT(0, c.foc_en);
}
void test_foc_angle(void) {
    ParsedCmd c = parse_command("FOC,A,90.5");
    TEST_ASSERT_EQUAL(CMD_FOC_ANGLE, c.type);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.5f, c.foc_angle);
}
void test_foc_home(void) {
    TEST_ASSERT_EQUAL(CMD_FOC_HOME, parse_command("FOC,H").type);
}
void test_foc_clear(void) {
    TEST_ASSERT_EQUAL(CMD_FOC_CLEAR, parse_command("FOC,CLR").type);
}
void test_foc_voltage(void) {
    ParsedCmd c = parse_command("FOC,V,6.0");
    TEST_ASSERT_EQUAL(CMD_FOC_VOLTAGE, c.type);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 6.0f, c.foc_voltage);
}
void test_foc_status(void) {
    TEST_ASSERT_EQUAL(CMD_FOC_STATUS, parse_command("FOC,S").type);
}
void test_foc_pp(void) {
    ParsedCmd c = parse_command("FOC,PP,11");
    TEST_ASSERT_EQUAL(CMD_FOC_POLEPAIRS, c.type);
    TEST_ASSERT_EQUAL_INT(11, c.foc_pole_pairs);
}
void test_unknown(void) {
    TEST_ASSERT_EQUAL(CMD_UNKNOWN, parse_command("GARBAGE").type);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_move_valid);
    RUN_TEST(test_move_bad_format);
    RUN_TEST(test_diag);
    RUN_TEST(test_foc_en_1);
    RUN_TEST(test_foc_en_0);
    RUN_TEST(test_foc_angle);
    RUN_TEST(test_foc_home);
    RUN_TEST(test_foc_clear);
    RUN_TEST(test_foc_voltage);
    RUN_TEST(test_foc_status);
    RUN_TEST(test_foc_pp);
    RUN_TEST(test_unknown);
    return UNITY_END();
}
```

- [ ] **Step 3：修改 `platformio.ini` 的 `[env:native_test]` 区段**

将 A4 Step 1 写入的 `build_src_filter = -<*>` 行**替换**为：
```ini
build_src_filter = +<protocol_parser.cpp>
```
（只改这一行，其他行保留不动。用 Edit 工具定位旧 `-<*>` 做替换。）

- [ ] **Step 4：运行测试确认全部失败（因为 parser 还没写）**

```bash
pio test -e native_test
```

Expected: `12 Tests 12 Failures`（或编译错误，说明 protocol_parser.cpp 还没创建）。

- [ ] **Step 5：实现 `protocol_parser.cpp`**

```cpp
// esp32_stepper/src/protocol_parser.cpp
#include "protocol_parser.h"
#include <vector>
#include <cstdlib>

static std::vector<std::string> split(const std::string& s, char sep) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
        if (c == sep) { out.push_back(cur); cur.clear(); }
        else cur += c;
    }
    out.push_back(cur);
    return out;
}

ParsedCmd parse_command(const std::string& line) {
    ParsedCmd cmd;
    auto parts = split(line, ',');
    if (parts.empty() || parts[0].empty()) {
        cmd.error_reason = "empty"; return cmd;
    }

    if (parts[0] == "MOVE") {
        if (parts.size() != 4) { cmd.error_reason = "bad format"; return cmd; }
        cmd.type = CMD_MOVE;
        cmd.move_steps = std::atoi(parts[1].c_str());
        cmd.move_dir   = std::atoi(parts[2].c_str());
        cmd.move_delay = std::atoi(parts[3].c_str());
        return cmd;
    }
    if (parts[0] == "DIAG" && parts.size() == 1) {
        cmd.type = CMD_DIAG; return cmd;
    }
    if (parts[0] == "FOC") {
        if (parts.size() < 2) { cmd.error_reason = "bad format"; return cmd; }
        const std::string& sub = parts[1];
        if (sub == "EN" && parts.size() == 3) {
            cmd.type = CMD_FOC_EN; cmd.foc_en = std::atoi(parts[2].c_str()); return cmd;
        }
        if (sub == "A" && parts.size() == 3) {
            cmd.type = CMD_FOC_ANGLE; cmd.foc_angle = std::atof(parts[2].c_str()); return cmd;
        }
        if (sub == "H" && parts.size() == 2) { cmd.type = CMD_FOC_HOME; return cmd; }
        if (sub == "CLR" && parts.size() == 2) { cmd.type = CMD_FOC_CLEAR; return cmd; }
        if (sub == "V" && parts.size() == 3) {
            cmd.type = CMD_FOC_VOLTAGE; cmd.foc_voltage = std::atof(parts[2].c_str()); return cmd;
        }
        if (sub == "S" && parts.size() == 2) { cmd.type = CMD_FOC_STATUS; return cmd; }
        if (sub == "PP" && parts.size() == 3) {
            cmd.type = CMD_FOC_POLEPAIRS; cmd.foc_pole_pairs = std::atoi(parts[2].c_str()); return cmd;
        }
        cmd.error_reason = "bad format"; return cmd;
    }
    cmd.error_reason = "unknown command";
    return cmd;
}
```

- [ ] **Step 6：运行测试确认全部通过**

```bash
pio test -e native_test
```

Expected: `12 Tests 0 Failures`.

- [ ] **Step 7：提交**

```bash
git add esp32_stepper/src/protocol_parser.* esp32_stepper/test/ esp32_stepper/platformio.ini
git commit -m "feat(protocol): add pure command parser with unity tests (TDD)"
```

## Task C2：改写 `protocol.cpp` 使用新解析器 + FOC 分发

**Files:**
- Modify: `esp32_stepper/src/protocol.cpp`

- [ ] **Step 1：重写 `protocol_handle_line` 基于 `parse_command`**

```cpp
// esp32_stepper/src/protocol.cpp
#include "protocol.h"
#include "protocol_parser.h"
#include "stepper.h"
#include "foc_motor.h"
#include <Arduino.h>

static void reply_ok()                  { Serial.println("OK"); }
static void reply_err(const char* why)  { Serial.print("ERR:"); Serial.println(why); }

void protocol_handle_line(const String& line) {
    std::string s = line.c_str();
    ParsedCmd c = parse_command(s);

    switch (c.type) {
        case CMD_MOVE:
            stepper_move(c.move_steps, c.move_dir, c.move_delay);
            // stepper_move 已在末尾 println("OK")，不再重复
            return;

        case CMD_DIAG:
            stepper_run_diagnostics();
            return;

        case CMD_FOC_EN:
            if (foc_request_enable(c.foc_en == 1)) reply_ok();
            else reply_err("fault latched");
            return;

        case CMD_FOC_ANGLE:
            if (foc_set_target_deg(c.foc_angle)) reply_ok();
            else reply_err("out of range");
            return;

        case CMD_FOC_HOME:
            foc_home(); reply_ok(); return;

        case CMD_FOC_CLEAR:
            if (foc_clear_fault()) reply_ok();
            else reply_err("no fault");
            return;

        case CMD_FOC_VOLTAGE:
            if (foc_set_voltage_limit(c.foc_voltage)) reply_ok();
            else reply_err("out of range");
            return;

        case CMD_FOC_STATUS: {
            Serial.print("FOC,S,");
            Serial.print((int)foc_get_state()); Serial.print(',');
            Serial.print(foc_get_current_deg(), 1); Serial.print(',');
            Serial.print(foc_get_target_deg(), 1); Serial.print(',');
            Serial.println(foc_is_fault_latched() ? 1 : 0);
            return;
        }

        case CMD_FOC_POLEPAIRS:
            if (foc_set_pole_pairs_and_store(c.foc_pole_pairs)) reply_ok();
            else reply_err("out of range");
            return;

        case CMD_UNKNOWN:
        default:
            reply_err(c.error_reason.empty() ? "unknown command" : c.error_reason.c_str());
            return;
    }
}
```

- [ ] **Step 2：改 `platformio.ini` 让主 env 也编译 protocol_parser.cpp**

（默认 `build_src_filter = +<**>` 会带上，无需改动。确认一下。）

- [ ] **Step 3：编译 + 烧录**

```bash
cd esp32_stepper
pio run -t upload
```

- [ ] **Step 4：步进回归**

串口发 `MOVE,100,0,20`，应 `OK` + 电机动。发 `DIAG`，应自检输出。

- [ ] **Step 5：FOC 协议烟雾测试（无硬件也能验）**

串口依次发送：
- `FOC,S` → 应回 `FOC,S,0,0.0,0.0,0`（disabled / 角度 0 / 无故障）
- `FOC,A,50` → 应 `OK`
- `FOC,S` → 应回 `FOC,S,0,0.0,50.0,0`（target 已更新）
- `FOC,A,9999` → 应 `ERR:out of range`
- `FOC,V,100` → 应 `ERR:out of range`
- `FOC,CLR` → 应 `ERR:no fault`
- `FOC,PP,7` → 应 `OK`（但 NVS 还没实现，仅镜像接受）
- `FOC,EN,1` → **不要发**，此时硬件还没上 12V

- [ ] **Step 6：提交**

```bash
git add esp32_stepper/src/protocol.cpp
git commit -m "feat(protocol): wire FOC commands to foc_motor dispatch"
```

## Task C3：NVS 持久化极对数

**Files:**
- Modify: `esp32_stepper/src/foc_motor.cpp`

⚠️ **关键**：`BLDCMotor motor(N)` 是文件级静态对象，构造时已锁定 N=7。但 SimpleFOC 的 `BLDCMotor::pole_pairs` 是 **public int 成员**，在 `motor.init()` 之前赋值即可生效（已验证）。因此步骤如下：

- [ ] **Step 1：在 `foc_motor.cpp` 顶部加 Preferences**

```cpp
#include <Preferences.h>
static Preferences s_prefs;
```

- [ ] **Step 2：在 `foc_init()` **最开头**（在 Wire.begin 之前、任何 SimpleFOC 调用之前）插入：**

```cpp
// 从 NVS 读极对数覆盖默认值
s_prefs.begin("foc", /*readOnly=*/true);
int pp = s_prefs.getInt("pp", FOC_POLE_PAIRS_DEFAULT);
s_prefs.end();
motor.pole_pairs = pp;                // public 成员，init() 前赋值即生效
Serial.print("[FOC] pole_pairs from NVS = "); Serial.println(pp);
```

保留 `BLDCMotor motor(FOC_POLE_PAIRS_DEFAULT);` 不变（文件级）。

- [ ] **Step 3：实现 `foc_set_pole_pairs_and_store`（替换 Task B2 写的 TODO stub）**

```cpp
bool foc_set_pole_pairs_and_store(int n) {
    if (n < 1 || n > 50) return false;
    s_prefs.begin("foc", /*readOnly=*/false);
    s_prefs.putInt("pp", n);
    s_prefs.end();
    // 下次启动生效（motor.pole_pairs 在 foc_init 读 NVS 时设置）
    return true;
}
```

- [ ] **Step 4：编译 + 烧录**

```bash
cd esp32_stepper && pio run -t upload
pio device monitor --port COM4 --baud 115200
```

- [ ] **Step 5：验证启动读 NVS**

串口应看到 `[FOC] pole_pairs from NVS = 7`（或之前存的值）。

- [ ] **Step 6：验证写 NVS + 重启生效**

发 `FOC,PP,9` → 按复位键 → 应看到 `[FOC] pole_pairs from NVS = 9`。
发 `FOC,PP,7` → 复位 → 回到 7。

- [ ] **Step 7：提交**

```bash
git add esp32_stepper/src/foc_motor.cpp
git commit -m "feat(foc): persist pole pairs to NVS via FOC,PP command"
```

## Task C4：nFAULT 监测

**Files:**
- Modify: `esp32_stepper/src/foc_motor.h`
- Modify: `esp32_stepper/src/foc_motor.cpp`
- Modify: `esp32_stepper/src/main.cpp`

- [ ] **Step 1：在 `foc_motor.h` 加 `foc_latch_fault()` 声明**

在其他 API 声明旁加一行：
```cpp
// 由 Core 1 的 nFAULT 轮询调用，把状态机打到 FAULT
void foc_latch_fault();
```

- [ ] **Step 2：在 `foc_motor.cpp` 实现**

```cpp
void foc_latch_fault() {
    g_fault_latched.store(true);
    g_state.store(FOC_STATE_FAULT);
    g_enable_req.store(false);
    motor.disable();
}
```

- [ ] **Step 3：在 main.cpp loop() 里加周期性 FAULT 轮询**

在 `main.cpp` 顶部确保已 `#include "foc_motor.h"` 和 `#include "config.h"`。然后：

```cpp
static uint32_t s_last_fault_check = 0;

void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() > 0) protocol_handle_line(cmd);
    }

    uint32_t now = millis();
    if (now - s_last_fault_check >= FAULT_POLL_MS) {
        s_last_fault_check = now;
        if (digitalRead(PIN_FOC_NFAULT) == LOW && foc_get_state() != FOC_STATE_DISABLED) {
            foc_latch_fault();            // 通过正规 API，不用 extern
            Serial.println("FOC,FAULT");
        }
    }
}
```

（不再使用局部 `extern` 声明；完全依赖 foc_motor.h 的公开 API。）

- [ ] **Step 3：编译，硬件还没接所以 nFAULT 悬空读到 HIGH（因 INPUT_PULLUP），不会触发**

```bash
pio run
```

- [ ] **Step 4：提交**

```bash
git add esp32_stepper/src/main.cpp esp32_stepper/src/foc_motor.*
git commit -m "feat(foc): add nFAULT latched detection on Core 1"
```

---

# 阶段 D：硬件通电（4 关卡）

⚠️ **每关卡必须全部通过才能进下一关。失败回退，不堆叠。**
参见 spec `§8`。

## Task D1：关卡 1 — USB only，验证 I2C + AS5600

**前提**：SimpleFOC mini **不接 12V**。

- [ ] **Step 1：按 spec §4 完整接线**

双方 GND 共地；AS5600 接 SDA=8 / SCL=9 / 3V3；SimpleFOC 接 M1-3=11,12,13 / EN=14 / nFT=10 / 3V3。电机相线 A/B/C 先接 SimpleFOC OUT1/2/3（任意顺序）。

- [ ] **Step 2：烧录固件**

```bash
cd esp32_stepper && pio run -t upload
pio device monitor --port COM4 --baud 115200
```

- [ ] **Step 3：串口应看到**

```
ESP32 Stepper Ready
[FOC] pole_pairs from NVS = 7
```

- [ ] **Step 4：发 `FOC,S`**

预期响应：`FOC,S,0,<X>,0.0,0`（X 为 AS5600 当前读数，度）。

- [ ] **Step 5：手动旋转电机轴**

再发 `FOC,S`，`X` 应变化。证明 AS5600 I2C 通 + sensor.update() 在 DISABLED 态仍运行。

- [ ] **Step 6：失败排查**

| 症状 | 排查 |
|---|---|
| sensor.init 卡死 | SDA/SCL 接线反 / 磁铁未装 |
| X 恒为 0 | 磁铁距离过远（应 0.5-3mm）/ 磁铁极性装反 |
| 无打印 | 上一阶段烧录失败，重烧 |

- [ ] **Step 7：通过后记录**

```bash
git commit --allow-empty -m "milestone: 关卡 1 通过 (I2C+AS5600 OK)"
```

## Task D2：关卡 2 — 上 12V，验证电源与 nFAULT

**前提**：关卡 1 通过。

- [ ] **Step 1：接入 12V 电源到 SimpleFOC mini VM**

注意极性！红正黑负。

- [ ] **Step 2：观察 SimpleFOC mini 板上 LED（若有）**

典型：电源指示灯亮。

- [ ] **Step 3：串口发 `FOC,S`**

预期：`FOC,S,0,<X>,0.0,0`（fault=0）。

- [ ] **Step 4：手动转轴，X 仍应变化**

- [ ] **Step 5：万用表量 SimpleFOC mini 3V3 针脚**

预期 3.3V（证明 ESP32 3V3 线仍供电）。

- [ ] **Step 6：失败排查**

| 症状 | 排查 |
|---|---|
| ESP32 重启 / 串口断开 | 12V 电源干扰 USB，12V 供电加磁珠 / 换独立 USB 电源 |
| fault=1 | 断电，查 M1/M2/M3 有无短路 / 相线短接 |
| X 停止变化 | Core 0 任务挂了，检查栈大小 FOC_TASK_STACK |

- [ ] **Step 7：通过后记录**

```bash
git commit --allow-empty -m "milestone: 关卡 2 通过 (12V+nFAULT OK)"
```

## Task D3：关卡 3 — 首次使能 + initFOC 对齐

⚠️ 此步**电机会自行转动 1-2 圈**完成对齐。预先固定好，防飞出。

- [ ] **Step 1：确认 `motor.voltage_limit = 3.0` 起步低压**

`grep "INITIAL_V_LIMIT" esp32_stepper/src/config.h` 确认 3.0。

- [ ] **Step 2：发 `FOC,EN,1`**

串口预期输出：
```
[FOC] Align sensor ...
[FOC] Sensor direction = CW   (或 CCW)
[FOC] PP check: pp=7 → PASS
[FOC] Zero electric angle = 2.34 rad
OK
```

- [ ] **Step 3：发 `FOC,A,10`**

电机应平滑从当前角度走到 10°。用 `FOC,S` 观察 cur 接近 10.0。

- [ ] **Step 4：发 `FOC,A,0`**

电机应回到 0°。

- [ ] **Step 5：用手施加反向外力**

电机应"弹性回弹"，不滑动。证明位置闭环有效。

- [ ] **Step 6：失败处理**

| 症状 | 对策 |
|---|---|
| `PP check: FAIL` | 跳到 Task D4（极对数校准）|
| 电机疯转 / 高频尖叫 | 立刻 `FOC,EN,0`，交换电机相线 M1↔M2 任两相，重试 |
| 只抖不转 | 断电，对调 M1↔M2 或重装 AS5600 磁铁翻面 |
| Align 时发烫 | 临时在 `config.h` 把 `FOC_INITIAL_V_LIMIT` 改 2.0 重烧 |

- [ ] **Step 7：通过后记录**

```bash
git commit --allow-empty -m "milestone: 关卡 3 通过 (FOC 闭环工作)"
```

## Task D4：关卡 3B（仅当 D3 PP 校验失败）

**Files:**
- Create: `esp32_stepper/src_find_pp/find_pp.cpp`
- Modify: `esp32_stepper/platformio.ini`（新增独立 env）

**策略**：使用**独立的 PlatformIO env**，不污染主 env 的 src_filter。

- [ ] **Step 1：创建 `esp32_stepper/src_find_pp/find_pp.cpp`**

从 SimpleFOC 库的 `examples/utils/calibration/find_pole_pair_number/find_pole_pair_number.ino` 复制内容，修改引脚：

```cpp
#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCDriver3PWM driver = BLDCDriver3PWM(11, 12, 13, 14);
BLDCMotor motor = BLDCMotor(11);  // 起始猜测，脚本会改

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);
  sensor.init();
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 3;
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_sensor_align = 3;
  motor.linkSensor(&sensor);
  motor.init();
  // 其余按 SimpleFOC 示例原样
}

void loop() {
  // 示例的极对数搜索循环（原样复制）
}
```

- [ ] **Step 2：在 `platformio.ini` 追加新 env**

```ini
[env:esp32s3_find_pp]
extends = env:esp32s3
src_dir = src_find_pp
```

（`extends` 继承主 env 的所有配置，只覆盖 `src_dir`；主 env 不受影响。）

- [ ] **Step 3：烧录校准固件**

```bash
cd esp32_stepper
pio run -e esp32s3_find_pp -t upload
pio device monitor --port COM4 --baud 115200
```

- [ ] **Step 4：按提示交互操作**

脚本会提示 "Press any key to start" 等。跑完记下真实极对数 `N`（例如 11 或 14）。

- [ ] **Step 5：烧回主固件并写 NVS**

```bash
pio run -t upload       # 默认 env:esp32s3
```

连接后串口发 `FOC,PP,<N>`，然后按 ESP32 复位键，应看到 `[FOC] pole_pairs from NVS = <N>`。

- [ ] **Step 6：回关卡 3（Task D3）重试**

- [ ] **Step 7：（可选）删除或保留 `src_find_pp/`**

保留可作后续调试工具；删除可精简仓库。

## Task D5：关卡 4 — 参数精修

- [ ] **Step 1：逐档上调 voltage_limit**

`FOC,V,4` → 观察扭矩。`FOC,V,5` → `FOC,V,6`。每档跑 `FOC,A,90` / `FOC,A,-90` 看响应速度与温度。

**判停**：电机摸着不烫（<50°C），响应 1 秒内到位。

- [ ] **Step 2：若出现振荡/啸叫，降 P**

在 `foc_motor.cpp` 的 `foc_init()` 加：
```cpp
motor.P_angle.P = 15;  // 默认 20
```
重烧。

- [ ] **Step 3：若扭矩不足（被推动后不能回正）**

逐档上调 voltage_limit 至 8V 上限。

- [ ] **Step 4：记录最终参数到 spec §13（变更记录）**

编辑 `docs/superpowers/specs/2026-04-17-foc-motor-integration-design.md` 增加实测数据。

- [ ] **Step 5：提交**

```bash
git add docs/ esp32_stepper/src/foc_motor.cpp
git commit -m "tune(foc): final voltage_limit and P_angle values from bench"
```

---

# 阶段 E：GUI 改造（Notebook + FOC Tab）

## Task E1：引入 Notebook，迁移步进 UI 到 Tab

**Files:**
- Modify: `pc_gui.py`

- [ ] **Step 1：把现有 `_build_ui` 中的"运动参数 / 控制 / 位置与原点"三个 LabelFrame 抽到新方法 `_build_stepper_tab(parent)`**

改动：把 `grid(..., row=1, ...)` 的 parent 从 `self.root` 改为传入的 `parent`。

- [ ] **Step 2：在 `_build_ui` 用 `ttk.Notebook` 包裹**

```python
self.notebook = ttk.Notebook(self.root)
self.notebook.grid(row=1, column=0, columnspan=2, sticky="nsew", padx=10, pady=5)

self.stepper_tab = ttk.Frame(self.notebook)
self.notebook.add(self.stepper_tab, text="🔩 步进")
self._build_stepper_tab(self.stepper_tab)

self.foc_tab = ttk.Frame(self.notebook)
self.notebook.add(self.foc_tab, text="🧲 FOC 无刷")
self._build_foc_tab(self.foc_tab)  # 下 Task 实现
```

- [ ] **Step 3：暂时让 `_build_foc_tab` 只放一个占位 Label**

```python
def _build_foc_tab(self, parent):
    ttk.Label(parent, text="（FOC 控件将在下一任务加上）").pack(padx=20, pady=20)
```

- [ ] **Step 4：启动 GUI 验证步进 Tab 可用**

```bash
cd C:/Users/bimpub5/PycharmProjects/py_SteppingMotor
start_gui.bat
```

连接 COM4，点"向外 1mm ▶"，步进应正常工作。切到 FOC Tab 显示占位文字。

- [ ] **Step 5：提交**

```bash
git add pc_gui.py
git commit -m "refactor(gui): migrate to ttk.Notebook with stepper tab"
```

## Task E2：加 serial_lock 保护所有串口访问

**Files:**
- Modify: `pc_gui.py`

- [ ] **Step 1：在 `__init__` 加 `self.serial_lock = threading.Lock()`**

- [ ] **Step 2：把 `_send_pulses` 里的 `self.ser.write` + `readline` 包到 `with self.serial_lock:`**

```python
def _send_pulses(self, steps, direction, delay_ms):
    ...
    with self.serial_lock:
        try:
            self.ser.write(cmd.encode())
            resp = self.ser.readline().decode().strip()
        except Exception as e:
            ...
```

- [ ] **Step 3：回归测试**

启动 GUI，发连续点动，不应有异常。

- [ ] **Step 4：提交**

```bash
git add pc_gui.py
git commit -m "refactor(gui): add serial_lock around all serial I/O"
```

## Task E3：FOC Tab 静态控件

**Files:**
- Modify: `pc_gui.py`

- [ ] **Step 1：实现 `_build_foc_tab(parent)`**

按 spec §7.3 布局。变量：
```python
self.foc_target_var   = tk.DoubleVar(value=0.0)
self.foc_state_var    = tk.StringVar(value="未连接")
self.foc_current_var  = tk.StringVar(value="--")
self.foc_fault_var    = tk.StringVar(value="--")
self.foc_vlimit_var   = tk.DoubleVar(value=3.0)
self.foc_pp_var       = tk.IntVar(value=7)
```

控件：
- 状态区：`self.foc_state_label`, `self.foc_current_label`, `self.foc_fault_label`
- 目标控制：Spinbox(foc_target_var) + 前往按钮 + 5 快捷 + 4 增量
- 原点：设为 0° 按钮
- 使能 / 调试：使能切换按钮、voltage slider、极对数 Spinbox + 保存按钮、清除故障按钮

- [ ] **Step 2：所有按钮 `command=` 先连到空方法（下一任务实现）**

```python
def _foc_goto(self): pass
def _foc_quick(self, deg): pass
def _foc_increment(self, delta): pass
def _foc_home(self): pass
def _foc_toggle_enable(self): pass
def _foc_set_vlimit(self, _): pass
def _foc_save_pp(self): pass
def _foc_clear_fault(self): pass
```

- [ ] **Step 3：启动 GUI 目测布局**

切到 FOC Tab，验证控件可见、可点击但不报错。

- [ ] **Step 4：提交**

```bash
git add pc_gui.py
git commit -m "feat(gui): add FOC tab static layout"
```

## Task E4：FOC 命令绑定

**Files:**
- Modify: `pc_gui.py`

- [ ] **Step 1：新增高层发送方法**

```python
def _send_foc_cmd(self, cmd: str) -> str:
    """发 FOC 命令，返回响应。带 lock。"""
    with self.serial_lock:
        try:
            self.ser.write((cmd + "\n").encode())
            resp = self.ser.readline().decode().strip()
        except Exception as e:
            self.log(f"串口异常: {e}"); return ""
    self.log(f"发送 {cmd} → {resp}")
    return resp
```

- [ ] **Step 2：实现每个按钮回调**

```python
def _foc_goto(self):
    threading.Thread(target=lambda: self._send_foc_cmd(f"FOC,A,{self.foc_target_var.get():.1f}"),
                     daemon=True).start()

def _foc_quick(self, deg):
    self.foc_target_var.set(deg)
    self._foc_goto()

def _foc_increment(self, delta):
    self.foc_target_var.set(self.foc_target_var.get() + delta)
    self._foc_goto()

def _foc_home(self):
    threading.Thread(target=lambda: self._send_foc_cmd("FOC,H"), daemon=True).start()

def _foc_toggle_enable(self):
    self._foc_enabled = not getattr(self, '_foc_enabled', False)
    val = 1 if self._foc_enabled else 0
    threading.Thread(target=lambda: self._send_foc_cmd(f"FOC,EN,{val}"), daemon=True).start()

def _foc_set_vlimit(self, _):
    v = self.foc_vlimit_var.get()
    threading.Thread(target=lambda: self._send_foc_cmd(f"FOC,V,{v:.1f}"), daemon=True).start()

def _foc_save_pp(self):
    n = self.foc_pp_var.get()
    threading.Thread(target=lambda: self._send_foc_cmd(f"FOC,PP,{n}"), daemon=True).start()

def _foc_clear_fault(self):
    threading.Thread(target=lambda: self._send_foc_cmd("FOC,CLR"), daemon=True).start()
```

- [ ] **Step 3：GUI 测试（无硬件也能发）**

- 连 COM4
- FOC Tab 输入角度 50、点前往 → 日志应出现 `FOC,A,50.0 → OK`
- 点快捷 90° → 值变为 90，自动前往
- 滑动电压限幅 → 日志出现 `FOC,V,...`

- [ ] **Step 4：提交**

```bash
git add pc_gui.py
git commit -m "feat(gui): bind FOC tab controls to serial commands"
```

## Task E5：FOC 状态轮询线程

**Files:**
- Modify: `pc_gui.py`

- [ ] **Step 1：在 `__init__` 加 `self.foc_poll_running = False`**

- [ ] **Step 2：连接成功后启动轮询**

```python
def toggle_connect(self):
    ...  # 现有连接逻辑
    if self.ser and self.ser.is_open:
        self.ser.timeout = 0.2          # ← 关键：避免 readline 无限阻塞轮询线程
        self.foc_poll_running = True
        threading.Thread(target=self._foc_poll_loop, daemon=True).start()
```

断开时 `self.foc_poll_running = False`。

- [ ] **Step 3：写 `_foc_poll_loop`**

```python
def _foc_poll_loop(self):
    STATE_NAMES = {"0": "失能", "1": "对齐中", "2": "运行", "3": "故障"}
    while self.foc_poll_running and self.ser and self.ser.is_open:
        with self.serial_lock:
            try:
                self.ser.write(b"FOC,S\n")
                resp = self.ser.readline().decode().strip()
            except Exception:
                break
        # 解析 "FOC,S,<state>,<cur>,<tgt>,<fault>"
        parts = resp.split(",")
        if len(parts) == 6 and parts[0] == "FOC" and parts[1] == "S":
            state, cur, tgt, fault = parts[2], parts[3], parts[4], parts[5]
            self.root.after(0, lambda: self._update_foc_display(state, cur, tgt, fault))
        time.sleep(0.1)

def _update_foc_display(self, state, cur, tgt, fault):
    STATE_NAMES = {"0": "失能", "1": "对齐中", "2": "运行", "3": "故障"}
    self.foc_state_var.set(STATE_NAMES.get(state, "?"))
    self.foc_current_var.set(f"{float(cur):.1f}°")
    self.foc_fault_var.set("报警" if fault == "1" else "正常")
    # 更新门控
    self._update_foc_button_states(state, fault)
```

- [ ] **Step 4：写 `_update_foc_button_states` 实现 spec §7.5 门控**

```python
def _update_foc_button_states(self, state, fault):
    is_fault = fault == "1"
    is_running = state == "2"
    is_disabled_or_running = state in ("0", "2")

    # DISABLED/RUNNING 才能操作运动按钮
    motion_state = "normal" if is_running else "disabled"
    for b in self._foc_motion_btns:
        b.config(state=motion_state)

    # 使能按钮：DISABLED/RUNNING 时可切换；FAULT/ALIGNING 时灰
    self.foc_enable_btn.config(state="normal" if is_disabled_or_running else "disabled")

    # 清除故障：仅 FAULT 态可点
    self.foc_clear_btn.config(state="normal" if is_fault else "disabled")

    # 归零、电压限幅、极对数：DISABLED/RUNNING 可用
    for b in self._foc_cfg_widgets:
        b.config(state=motion_state)
```

（需在 `_build_foc_tab` 时把相关控件加入 `self._foc_motion_btns` / `self._foc_cfg_widgets` 列表。）

- [ ] **Step 5：硬件联机测试**

- 连接 ESP32
- 切到 FOC Tab → 当前角度应实时刷新
- 手动转电机轴 → 角度同步变化
- 点使能 FOC（若硬件 ready）→ 状态变"对齐中"→"运行"；相关按钮灰度联动

- [ ] **Step 6：提交**

```bash
git add pc_gui.py
git commit -m "feat(gui): add 100ms FOC status polling thread and state gating"
```

## Task E6：阻塞期间显示"步进中"

**Files:**
- Modify: `pc_gui.py`

- [ ] **Step 1：在 `_send_pulses` 包裹 `self.stepper_in_progress = True/False`**

- [ ] **Step 2：`_update_foc_display` 基于原始值设置，避免图标累积**

```python
def _update_foc_display(self, state, cur, tgt, fault):
    ...
    cur_text = f"{float(cur):.1f}°"
    if getattr(self, 'stepper_in_progress', False):
        cur_text += " ⏸"          # 仅在本帧加，下帧重新生成不会累积
    self.foc_current_var.set(cur_text)
    ...
```

- [ ] **Step 3：测试**

运行一次大步进（100mm），切 FOC Tab 应看到暂停标记；步进完成后标记消失。

- [ ] **Step 4：提交**

```bash
git add pc_gui.py
git commit -m "feat(gui): indicate 'stepper-busy' pause state in FOC panel"
```

---

# 阶段 F：整合测试与文档

## Task F1：整合测试

- [ ] **Step 1：同时给两个电机下指令**

GUI：
1. 步进 Tab 发 100mm 向外（约 10 秒）
2. 切到 FOC Tab，快速发 `FOC,A,90` 三次
3. 验证步进跑完后 FOC 稳定在 90°

- [ ] **Step 2：故障恢复流程**

1. 模拟故障（暂时短接 M1/M2 一瞬间，或物理 hold 住电机不让转）
2. 观察串口 `FOC,FAULT` 打印
3. GUI 切到 FOC Tab → 状态显示"故障"，所有按钮灰只剩清除故障
4. 点清除故障 → 状态回"失能"，其他按钮恢复

- [ ] **Step 3：断电重连**

1. 拔 ESP32 USB → GUI 日志应出现异常 / 轮询停止
2. 重新插 USB → 重连 → 轮询恢复

- [ ] **Step 4：记录**

```bash
git commit --allow-empty -m "milestone: 阶段 F 整合测试通过"
```

## Task F2：更新 `MD422_20K-2M.md` 增加 FOC 内容

**Files:**
- Modify: `MD422_20K-2M.md`

- [ ] **Step 1：先 Read 现有文件确认章节结构**

```bash
# 先通读 MD422_20K-2M.md 确认 §7 文件结构段、§6 故障排查段的位置和格式
```

- [ ] **Step 2：在文档末尾追加"SimpleFOC 无刷电机"章节**

包含：接线表（复用 spec §4）、FOC 协议命令列表（复用 spec §6.2）、4 关卡简述（复用 spec §8 摘要）、故障排查小表。保持与现有段落相同的 markdown 风格（###、表格格式、代码块语言标记）。

- [ ] **Step 3：更新文件结构树（§7），增加 foc_motor.* / protocol_parser.* / config.h / test/**

- [ ] **Step 3：提交**

```bash
git add MD422_20K-2M.md
git commit -m "docs: add FOC motor section to MD422_20K-2M readme"
```

## Task F3：最终验证与关闭

- [ ] **Step 1：回归测试所有原有功能**

- 步进点动、连续、距离、原点、回到原点 全部可用
- DIAG 命令可用
- FOC 全部命令可用

- [ ] **Step 2：更新 spec 的 §12 工作量实际耗时**

- [ ] **Step 3：最终 commit**

```bash
git add docs/
git commit -m "docs(spec): record actual bringup time in §12"
```

- [ ] **Step 4：标签**

```bash
git tag v1.0-foc-integrated
```

---

## 回滚策略

每个阶段通过后有独立 commit，可随时回滚：

| 故障 | 回滚到 |
|---|---|
| 阶段 A 破坏步进 | `git reset --hard HEAD~N`（N = A 内 commit 数）|
| FOC 不能闭环 | 保留 C 代码但不使能 FOC，单独用步进（GUI 切 Tab 即可）|
| 整体失败 | 执行前先 `git log --oneline -1` 记下当前 HEAD；出问题 `git checkout <那个 hash>`。或直接丢弃：`git reset --hard origin/master` |

## 开发环境检查清单

确认以下就绪：

- [ ] PlatformIO CLI 可用 (`pio --version`)
- [ ] ESP32-S3 在 COM4
- [ ] Python venv 就绪 (`.venv/Scripts/python.exe`)
- [ ] SimpleFOC mini + 2208 电机 + AS5600 上磁已装
- [ ] 12V 2A 电源就位

---

## 预估时长

| 阶段 | 估计 |
|---|---|
| A（重构） | 1.5 h |
| B（SimpleFOC 骨架） | 1 h |
| C（协议+NVS+fault） | 1.5 h |
| D（硬件 4 关卡） | 1-3 h |
| E（GUI） | 2 h |
| F（整合 + 文档） | 0.5 h |
| **总** | **7.5-10.5 h** |
