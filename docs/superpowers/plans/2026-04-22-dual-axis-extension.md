# 双轴扩展（左/右腿）实施计划

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development or executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把现有单轴系统（1× DM422 + 1× SimpleFOC mini + 2208 + AS5600）扩展为双轴，对应一台两腿机器人的左右旋转关节。左轴（L=axis 0）硬件不动，新增右轴（R=axis 1）。

**Architecture:** ESP32-S3 双核已有分工不变（Core 0 跑 FOC 闭环，Core 1 跑串口+步进+nFAULT 轮询）。新增右轴走第二条 I2C 总线（Wire1），其他 PWM/脉冲信号占新 GPIO。24V 电源共用，仅共地。固件用每轴数组 + 每轴 FreeRTOS 任务实现 4 电机真并行；GUI 用 4 个 Tab。协议升级到 v2.3（全显式轴号）。

**Tech Stack:** PlatformIO / Arduino ESP32 core 3.20017 / SimpleFOC 2.3.3 / FreeRTOS / Python Tkinter / pyserial

**Spec reference:** 此计划对应的设计已经分布在下列 commit 里（无独立 spec 文档）：
- `e1d248d` 固件多轴重构
- `e8227ac` GUI 4-Tab 重写

**当前 branch:** `feat/foc-integration`。合并到 master 前需完成本计划。

---

## 前置状态（已完成）

软件端已经完成、编译通过：

- ✅ 固件 `src/config.h`：`NUM_AXES=2` + 每轴引脚宏（axis 1 用 GPIO 7/15/16/17/18/21/4/42/41）
- ✅ 固件 `src/stepper.cpp`：按轴数组化；每轴独立 FreeRTOS 任务（Core 1 优先级 1）
- ✅ 固件 `src/foc_motor.cpp`：按轴 SimpleFOC 对象数组；Wire+Wire1 双 I2C；每轴任务（Core 0 优先级 2）
- ✅ 固件 `src/protocol.cpp`：协议 v2.3 全显式轴号，新事件 `STEP,<axis>,DONE` / `FOC,<axis>,FAULT`
- ✅ GUI `pc_gui.py`：Notebook 4 Tab（🔩步进 L/R + 🧲FOC L/R）+ 每轴状态数组 + 轮询按轴分发
- ✅ 校准持久化 `.stepper_calib.json` 已改为按轴结构

剩余全部是**硬件就位验证** + **文档同步** + **可选增强**。

---

## Phase A：右轴硬件就位 & 基础验证（必做）

**前提**：物理接好右腿（参考 §"右腿扩展接线表"）：
- DM422 右: GPIO 7→PUL+, 15→DIR+, 地共地, 24V 并联
- SimpleFOC mini 右: GPIO 16/17/18→M1/2/3, 21→EN, 4→nFT, 3V3/GND, 24V→VM
- AS5600 右: GPIO 42→SDA, 41→SCL, 3V3/GND
- 两台电机各自相线接对应驱动板

### Task A1：烧录新固件，冷启动验证

**Files:** 无代码改动，只是烧录已入库的 `e1d248d`。

- [ ] **Step 1：关闭所有占用 COM4 的程序**（PlatformIO Monitor / GUI）
- [ ] **Step 2：确认 24V 电源未加电**（只 USB 供电）
- [ ] **Step 3：烧录**

```bash
cd C:/Users/bimpub5/PycharmProjects/py_SteppingMotor/esp32_stepper
pio run -e esp32s3 -t upload
```

**Expected**: `SUCCESS ... Hard resetting via RTS pin`

- [ ] **Step 4：打开串口监视器观察启动 banner**

```bash
pio device monitor --port COM4 --baud 115200
```

**Expected** 看到两轴引脚都正常打印：

```
ESP32 Dual-Axis Motor Controller Ready
NUM_AXES = 2
  axis 0:
    stepper PUL=5 DIR=6
    FOC M1/2/3=11/12/13 EN=14 nFT=10
    I2C SDA/SCL=8/9
  axis 1:
    stepper PUL=7 DIR=15
    FOC M1/2/3=16/17/18 EN=21 nFT=4
    I2C SDA/SCL=42/41
[FOC 0] pole_pairs = 7
[FOC 0] sensor init done
[FOC 0] Core 0 task started
[FOC 1] pole_pairs = 7
[FOC 1] sensor init done    ← 关键：右腿 AS5600 在 Wire1 上被识别
[FOC 1] Core 0 task started
Protocol: MOVE,<axis>,s,d,us | FOC,<axis>,... | DIAG,<axis>
```

- [ ] **Step 5：若看到 `[FOC 1] sensor init done` 则 Wire1 + 右 AS5600 OK**

如果只有 axis 0 的打印、axis 1 卡住：
- 查 GPIO 41/42 接线
- 查右 AS5600 的 3V3 供电是否到位
- 查磁铁位置

### Task A2：右轴步进独立验证（关卡 1-2 合并版）

**前提**：A1 通过。**24V 仍未加电**。

- [ ] **Step 1：发测试指令到 axis 0 + axis 1，验证两轴独立**

在 Monitor 里依次发（每条后等 `ACK,x`）：
```
MOVE,0,10,0,20000
MOVE,1,10,0,20000
```

**Expected**: 两条都返回 `ACK,<axis>`；因为 24V 未接，电机不动，但 firmware 接受命令 + 打印 `STEP,<axis>,DONE` 说明脉冲输出正常。

- [ ] **Step 2：接入 24V 电源**

物理接好，开电。

- [ ] **Step 3：右腿步进小幅试走**

```
MOVE,1,50,0,20000      # 右腿走 1mm 向外
```

**Expected**: 右腿电机走 1mm。如果不动 → 检查共地线、DM422 拨码、相线。

- [ ] **Step 4：验证左腿未受影响**

```
MOVE,0,50,0,20000
```

**Expected**: 左腿如常，右腿不动。

- [ ] **Step 5：**`git commit --allow-empty -m "milestone: 右轴步进基础就位"`

### Task A3：右轴 FOC 对齐（关卡 3）

**前提**：A2 通过。

⚠️ **电机会自动转 1-2 圈完成对齐**，预先固定好右腿电机。

- [ ] **Step 1：发右轴使能**（Monitor）

```
FOC,1,EN,1
```

**Expected**: 看到多行 `MOT:` 诊断：
```
MOT: Align sensor.
MOT: sensor_direction==<CW或CCW>
MOT: PP check: OK!                    ← 关键
MOT: Zero elec. angle: X.XX
MOT: Ready.
OK,1
```

- [ ] **Step 2：小幅位置控制测试**

```
FOC,1,H
FOC,1,A,30
FOC,1,S
FOC,1,A,0
FOC,1,EN,0
```

**Expected**: 电机平滑转到 30° 再回 0°，`FOC,S` 查询返回的 cur 跟踪 target。

- [ ] **Step 3：失败处理**

| 现象 | 对策 |
|---|---|
| `PP check: FAIL` | 跑 `find_pole_pair_number` 示例找真实 pp，发 `FOC,1,PP,<n>` 存 NVS，重启 |
| 电机疯转/尖叫 | 发 `FOC,1,EN,0`，断 24V，**交换右电机的 OUT1/OUT2 相线**后重试 |
| 只抖不转 | 同上，交换两相或检查磁铁极性 |
| target=30 → cur 不动 | motor.initFOC 看上去成功但不跟踪 → 参考左腿 PSU=24V 调参的历史，检查 `voltage_sensor_align` |

- [ ] **Step 4：**`git commit --allow-empty -m "milestone: 右轴 FOC 对齐 + 位置闭环 OK"`

### Task A4：右轴 FOC 自动 PID 调参（关卡 4）

**前提**：A3 通过。

- [ ] **Step 1：关 Monitor，启动 GUI**

```bash
start_gui.bat
```

- [ ] **Step 2：GUI 连 COM4 → 切到 🧲 FOC R Tab**
- [ ] **Step 3：点"使能 FOC"** → 等状态变"运行"
- [ ] **Step 4：点"⌂ 把当前位置设为 0°"**
- [ ] **Step 5：点"🤖 自动优化 PID"按钮** → 确认对话框 → 等约 2 分钟

**Expected**: 日志里依次看到 PA 扫描 + VP 扫描 + 最终推荐值；滑条自动同步到最佳 PA/VP。

- [ ] **Step 6：手动验证**：用手推右腿电机轴，应感到刚度和阻尼（和左腿类似手感）

- [ ] **Step 7：**`git commit --allow-empty -m "milestone: 右轴 PID 自动调参通过"`

---

## Phase B：集成 / 并行验证

### Task B1：并行发命令四电机同跑

- [ ] **Step 1：GUI 4 个 Tab 分别设置：**
  - 步进 L Tab：距离 10mm，方向向外，速度 2ms
  - 步进 R Tab：距离 10mm，方向向外，速度 2ms
  - FOC L Tab：使能（若未使能）
  - FOC R Tab：使能

- [ ] **Step 2：快速依次点击：**
  1. 步进 L → "执行运动"
  2. 步进 R → "执行运动"
  3. FOC L → 输入 45° → 前往
  4. FOC R → 输入 -45° → 前往

**Expected**: 四个动作几乎同时发生：两台步进同时走 10mm，两台 FOC 同时转到各自角度。GUI 日志里每条命令立即返回 `ACK,<axis>` 或 `OK,<axis>`，无阻塞。

- [ ] **Step 3：验证 GUI 示波器正常刷新两轴（互不干扰）**

- [ ] **Step 4：**`git commit --allow-empty -m "milestone: 四电机并行动作验证通过"`

### Task B2：故障恢复测试（单轴 latch 不影响另一轴）

- [ ] **Step 1：左轴 FOC 使能 + 保持位置**
- [ ] **Step 2：人为让右轴进故障**：短暂短接右 SimpleFOC OUT1/OUT2，应触发 `FOC,1,FAULT`
- [ ] **Step 3：观察**：
  - 右轴 GUI 状态变"故障"
  - 左轴 GUI 状态保持"运行"不受影响
  - 日志 `⚠️ 轴R: FOC,1,FAULT`
- [ ] **Step 4：点右轴"🧹 清除故障"** → 重新使能 → 对齐成功
- [ ] **Step 5：**`git commit --allow-empty -m "milestone: 故障隔离验证通过"`

### Task B3：行程校准独立验证

- [ ] **Step 1：分别校准左右步进的 min/max**
- [ ] **Step 2：关 GUI 重启**
- [ ] **Step 3：验证 `.stepper_calib.json` 下两轴的 min/max 都正确加载**
- [ ] **Step 4：试超限命令**：GUI 给左轴发超出 max 的 MOVE → 应被软件拒绝

---

## Phase C：文档与收尾

### Task C1：更新 `MD422_20K-2M.md`

**Files:** `MD422_20K-2M.md`

- [ ] **Step 1：先 Read 现有文件**

- [ ] **Step 2：新增 §10 "双轴扩展"，内容：**
  - 两轴引脚映射表（L/R 对比）
  - 电源共用拓扑（24V 并联到两路 VM）
  - 双 I2C 总线（Wire/Wire1）
  - 协议 v2.3 全显式轴号示例
  - 并行运行的能力说明

- [ ] **Step 3：更新 §3 "通信协议"**：原命令加 axis 字段

- [ ] **Step 4：更新 §7 "文件结构"**：标注每文件多轴相关改动

### Task C2：更新设计文档

**Files:** 新建 `docs/superpowers/specs/2026-04-22-dual-axis-addendum.md`

- [ ] **Step 1：写一份短的"附录"spec**，引用原设计文档，说明：
  - 多轴引脚表
  - 多轴 I2C 总线拆分
  - 协议变化
  - 每轴独立 FreeRTOS 任务的决定

### Task C3：打 tag + 合并

- [ ] **Step 1：在 feat/foc-integration 上打 tag**

```bash
git tag -a v3.0-dual-axis -m "Dual-axis: 4 motors parallel, 4-tab GUI, v2.3 protocol"
```

- [ ] **Step 2：合并到 master（非 fast-forward）**

```bash
git checkout master
git merge --no-ff feat/foc-integration
```

- [ ] **Step 3：**（可选）推到远端

---

## Phase D：可选增强（视需求）

这些是未实现的后续需求，按用户需要选做。

### Task D1：广播命令

**新语法**：`MOVE,*,500,0,20000` / `FOC,*,A,90` —— 对所有轴并发

- 协议解析器识别 `*`，对 `[0..NUM_AXES)` 分别派发
- GUI 增加"同步控制"面板

**工时**：约 2 小时。

### Task D2：硬件限位开关

- 每轴加 2 限位（min/max），共 4 个数字输入 GPIO
- 固件中断 or 轮询读状态，触发时立即失能
- GUI 显示限位状态 LED

**工时**：约 3 小时 + 硬件采购。

### Task D3：位置/PID 全持久化

- `position_mm`、`P_angle.P`、`PID_velocity.P`、`voltage_limit` 全部进 NVS
- 重启后恢复（带用户确认对话框）

**工时**：约 1.5 小时。

### Task D4：加减速曲线

- 步进加梯形/S 曲线，高速启停无冲击
- FOC 加 velocity trajectory planner

**工时**：约 3-4 小时。

---

## 回滚策略

本计划所有改动都在 `feat/foc-integration` 分支。如果双轴扩展失败：

```bash
# 回到合并前的 master（只有单轴 FOC）
git checkout master
pio run -t upload
```

或回到本 plan 开始前的单轴稳定点：
```bash
git checkout e8227ac^     # 或任何之前的提交
```

如果在 Phase A 物理验证阶段发现硬件问题：
- 只需暂停验证，不用回退代码（固件自动按 `NUM_AXES=2` 工作，即使 axis 1 硬件未接只会 sensor.init 失败但不 crash）

---

## 预估总工时

| Phase | 任务 | 估计 |
|---|---|---|
| A | 硬件就位 + 4 关卡（右轴）| 1-2 h |
| B | 集成 / 并行验证 | 0.5 h |
| C | 文档 + tag + merge | 1 h |
| D | 可选增强（全做完）| 10+ h |
| **总（A+B+C）** | **最低必须做的** | **2.5-3.5 h** |
