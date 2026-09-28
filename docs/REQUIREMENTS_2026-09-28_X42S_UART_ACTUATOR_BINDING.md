# X42S 四轴 UART 闭环执行层改造需求

日期：2026-09-28  
目标分支：`feature/x42s-uart-actuator-binding`  
基线：`main@f73f5dab9bcdf0598fcb3c7294ee85a2ad821f47`

## 1. 背景

当前项目以 ESP32-S3 直接生成 PUL/DIR 脉冲，驱动 DM422/DM442。PC 端已经形成：

- 四个机构逻辑角色：`Mup1 / Mr1 / Mup2 / Mr2`；
- “逻辑电机绑定”机制；
- `AxisProfile / AxisRuntime`、直线/旋转模式、单位换算、软件限位、校准；
- 单轴 MOVE；
- 双轴原子 SYNC；
- S4 步态中的统一五次主进度；
- `gait_planner / gait_avoidance / gait_executor` 非线性机构轨迹。

现有绑定 v1 的本质是：

```text
LogicalRole -> physical stepper axis
```

建议映射为：

| 逻辑角色 | 现有建议步进轴 | 模式 |
|---|---:|---|
| Mup1 | 0 | linear |
| Mr1 | 2 | rotary |
| Mup2 | 1 | linear |
| Mr2 | 3 | rotary |

计划将实际执行器改为 4 台 ZDT X42S 第二代闭环步进电机，并优先评估直接使用 X42S 的 TTL UART 多机通讯，不再把 STEP/DIR 作为最终硬件接口。

X42S 用户手册明确支持：

- TTL/RS485/CAN 多机通讯；
- 电机地址 `ID_Addr`，地址 1..255，0 为广播地址；
- 多机同步运动；
- 多电机命令；
- 位置/速度/使能/停止等运动命令；
- 实时位置、目标位置、位置误差、速度、电流、温度及状态等读取；
- 驱动器内部编码器闭环。

本需求以仓库现有代码和《ZDT_X42S第二代闭环步进电机用户手册 V1.0.5 260527》为依据。

---

## 2. 总体目标

建立新的 **X42S UART 执行层**，并将 X42S 电机地址整合进现有“逻辑电机绑定”功能，使最终控制链形成：

```text
Mup1 / Mr1 / Mup2 / Mr2
          ↓
统一逻辑电机绑定
          ↓
X42S Address / Driver ID
          ↓
X42S UART Motion Backend
          ↓
4 × X42S internal closed loop
```

最终用户不再需要把“逻辑角色 -> GPIO 步进轴”和“GPIO 步进轴 -> X42S 编号”分成两套配置。

**X42S 编号必须成为现有绑定功能的一部分，而不是新增一套独立映射页面。**

---

## 3. 实施原则

### 3.1 保留上层机构逻辑

以下模块的机构语义不得因为更换驱动方式而重写：

- `motor_control/gait_planner.py`
- `motor_control/gait_avoidance.py`
- `motor_control/gait_twin.py`
- S0..S7 阶段语义
- `Mup1 / Mr1 / Mup2 / Mr2`
- 120° 三重对称和现有两模态避障逻辑
- 现有零位、方向、几何参数的含义

UART 改造属于“执行层替换”，不能把机构规划重新退化为简单的四个终点角度。

### 3.2 旧 STEP/DIR backend 暂时保留

开发期间保留旧 `Pulse/Stepper backend` 作为回退路径。

建议形成：

```text
MotionBackend
├── StepDirBackend      # 现有，可回退
└── X42SUartBackend     # 新增
```

不得在第一轮 UART 可运行前删除现有 `stepper.cpp / SYNC` 路径。

### 3.3 X42S 内部闭环，不在 ESP32 上再做位置 PID

X42S 的编码器闭环由驱动器内部完成。

ESP32/PC 负责：

- 机构轨迹；
- 目标运动指令；
- 多轴同步调度；
- 状态读取；
- 安全策略；
- 跟随误差判断。

不得在第一阶段用 ESP32 再叠加一层高频位置 PID 去“追”X42S 编码器。

---

## 4. 四台电机编号

首版固定规划：

| 逻辑角色 | X42S ID | 说明 |
|---|---:|---|
| Mup1 | 1 | 左侧升降 |
| Mr1 | 2 | 左侧旋转 |
| Mup2 | 3 | 右侧升降 |
| Mr2 | 4 | 右侧旋转 |

该表是首装建议值，但软件必须允许后续换绑。

### 4.1 编号规则

- 合法 X42S 地址：1..255。
- 地址 0 只允许作为广播地址使用，禁止绑定逻辑角色。
- 四个逻辑角色不能绑定同一个 X42S 地址。
- UI 显示必须明确写成 `X42S ID N`，不能只显示裸数字。
- 所有日志、错误、状态事件必须同时包含逻辑角色和 X42S ID，例如：
  - `Mr1 / X42S ID2 offline`
  - `Mup2 / X42S ID3 position error = ...`

---

## 5. 现有绑定功能升级

### 5.1 单一绑定入口

继续使用现有“电机绑定与状态”页作为唯一配置入口。

不得新增：

- “X42S 地址映射”独立页面；
- 第二份 role -> driver 映射 JSON；
- 需要用户先绑 axis 再绑 X42S 的双层配置流程。

### 5.2 Binding schema v2

现有 v1：

```json
{
  "schema_version": 1,
  "revision": 1,
  "bindings": {
    "Mup1": {"kind": "stepper", "axis": 0},
    "Mr1":  {"kind": "stepper", "axis": 2},
    "Mup2": {"kind": "stepper", "axis": 1},
    "Mr2":  {"kind": "stepper", "axis": 3}
  }
}
```

新 schema 建议：

```json
{
  "schema_version": 2,
  "revision": 1,
  "backend": "x42s_uart",
  "bindings": {
    "Mup1": {"kind": "x42s", "driver_id": 1},
    "Mr1":  {"kind": "x42s", "driver_id": 2},
    "Mup2": {"kind": "x42s", "driver_id": 3},
    "Mr2":  {"kind": "x42s", "driver_id": 4}
  }
}
```

最终字段名称可由实现者微调，但必须满足：

1. X42S ID 是绑定模型中的一等字段；
2. 不要求用户配置 GPIO axis；
3. 角色唯一性和 driver ID 唯一性严格校验；
4. 保存仍采用原有原子替换策略；
5. 旧 v1 文件必须明确迁移或明确拒绝，不能静默误解释。

### 5.3 v1 -> v2 迁移

当前建议映射：

```text
Mup1 axis0 -> X42S ID1
Mr1  axis2 -> X42S ID2
Mup2 axis1 -> X42S ID3
Mr2  axis3 -> X42S ID4
```

只允许在用户显式确认“按建议迁移”时生成该映射。

不能仅凭旧轴号自动假设现场接线一定符合建议。

### 5.4 AxisProfile 处理

当前很多参数仍依赖 `AxisProfile`：

- linear / rotary；
- pulse_per_rev；
- gear_ratio；
- lead_mm；
- 软件限位；
- 零位；
- 单位显示。

UART 改造第一阶段允许保留这些参数模型，但必须逐步去除“物理 GPIO axis = 执行器身份”的假设。

建议演进：

```text
LogicalRole
   ↓
Role/Actuator profile
   ↓
X42S driver_id
```

实现期间可保留兼容 adapter，但 UI 不应继续要求用户理解 ESP32 PUL/DIR 轴号。

---

## 6. 硬件要求

## 6.1 总体拓扑

目标硬件：

```text
ESP32-S3
   │
   │ TTL UART
   │
   ▼
X42S UART 转接板
   ├── X42S ID1
   ├── X42S ID2
   ├── X42S ID3
   └── X42S ID4
```

ESP32 到转接板只保留一组通讯链路。

当前项目可优先复用：

- GPIO42：UART TX
- GPIO47：UART RX
- GND

GPIO48 当前作为 RS485 DE 预留；TTL UART 方案原则上不需要 DE，但不得擅自改作其他关键功能，直到硬件版本最终确认。

## 6.2 转接板边界

首版转接板只承担：

- ESP32 UART 信号分配；
- 4 台 X42S 通讯接口连接；
- 公共通讯 GND；
- 必要的接插件；
- 调试测试点。

不在转接板上实现：

- 电机功率驱动；
- X42S 主电源分配（除非另行完成额定电流/保险设计）；
- RS485；
- 额外 MCU；
- 复杂协议转换。

### 6.3 必须现场确认

在定 PCB 前必须根据实际采购的 X42S 版本和手册 2.2.1“串口 TTL 多机通讯接线”确认：

- X42S 通讯端子的具体引脚定义；
- TTL 电平；
- 多机 TX/RX 的官方接线拓扑；
- 是否允许四台直接挂接；
- 是否需要串联电阻/上拉/特殊方向控制；
- 接口是否与 RS232/RS485/CAN 版本共用但电气实现不同；
- ESP32-S3 DevKit GPIO42/47 是否实际引出且无板级冲突。

不得只根据端子字母自行推断后直接制板。

## 6.4 通讯线要求

TTL UART 仅用于同一设备内部、短距离连接。

如果实际布线距离、噪声或可靠性测试表明 TTL 不满足要求，则保留未来迁移 RS485 的可能，但本分支不实施 RS485。

---

## 7. X42S 初始化要求

每台 X42S 上机前完成：

1. 安装磁铁并检查机械间隙；
2. 执行编码器/电机校准；
3. 设置闭环控制模式；
4. 设置唯一 `ID_Addr`；
5. 四台统一设置 UART baud；
6. 四台统一设置校验方式；
7. 明确 `Response` 模式；
8. 设置最大电流；
9. 设置最大速度；
10. 校验正方向；
11. 单轴低速运行确认；
12. 保存配置并形成现场记录。

首版建议：

```text
ID = 1,2,3,4
baud = 115200
checksum = 0x6B
```

具体最终值以实机验证为准。

---

## 8. ESP32 UART 驱动层

新增建议：

```text
esp32_stepper/src/x42s_uart.h
esp32_stepper/src/x42s_uart.cpp
```

也可采用更合适的命名，但不得把 X42S 协议大量塞进 `protocol.cpp` 或 `main.cpp`。

### 8.1 最低 API

至少应具备：

```text
x42s_init()
x42s_ping(id)
x42s_enable(id, enable)
x42s_stop(id)
x42s_stop_all()
x42s_zero_position(id)
x42s_move_position(...)
x42s_read_status(id)
x42s_read_position(id)
x42s_read_target(id)
x42s_read_position_error(id)
x42s_read_speed(id)
x42s_read_current(id)
x42s_read_temperature(id)
x42s_sync_trigger()
```

是否拆分为协议编码层和 bus 层由实现者决定。

### 8.2 串口所有权

UART 必须有唯一 owner。

不得出现：

- 两个 FreeRTOS task 同时直接写同一 UART；
- 查询回复被另一个请求消费；
- 多机主动返回与同步请求错配。

建议：

```text
motion/control tasks
      ↓
X42S request queue
      ↓
single X42S bus task
      ↓
HardwareSerial
```

### 8.3 帧解析

必须：

- 严格校验地址；
- 严格校验功能码；
- 严格校验校验字节；
- 有超时；
- 不把未知帧当作当前请求回复；
- 支持 X42S 主动到位帧；
- 记录通讯错误计数。

---

## 9. 第一阶段：单轴 UART 闭环最小闭环切换

第一里程碑只验证 1 台 X42S：

```text
PC
→ ESP32
→ UART
→ X42S ID1
→ position command
→ X42S internal closed loop
→ actual position feedback
```

必须完成：

- online/ping；
- enable/disable；
- 正反向；
- 相对位置；
- 绝对位置（若选定使用）；
- stop；
- emergency stop；
- 实时位置读取；
- 位置误差读取；
- 错误/保护状态读取。

此阶段不接 gait 自动动作。

验收：连续执行 100 次低速小角度/小位移命令，不发生地址错乱、回复错配、不可恢复通讯超时。

---

## 10. 第二阶段：四台总线

接入 ID1~4。

必须实现：

- 启动扫描 1~4；
- 唯一 ID 检查；
- missing motor 检查；
- 重复/错误地址能够明确诊断；
- 单台掉线不误认为另一台的回复；
- 四台依次读取状态；
- 单台 STOP；
- 全局 STOP；
- 全局 ESTOP。

不得使用广播地址执行普通单轴动作。

---

## 11. 第三阶段：绑定功能切换

“电机绑定与状态”页改为：

| 角色 | X42S ID | 类型 | 模式 | 在线 | Command | Actual | Error | State |
|---|---:|---|---|---|---:|---:|---:|---|
| Mup1 | 1 | X42S | linear | ... | ... | ... | ... | ... |
| Mr1 | 2 | X42S | rotary | ... | ... | ... | ... | ... |
| Mup2 | 3 | X42S | linear | ... | ... | ... | ... | ... |
| Mr2 | 4 | X42S | rotary | ... | ... | ... | ... | ... |

必须保留：

- 不重复绑定；
- Mup 只能 linear；
- Mr 只能 rotary；
- 运动中禁止换绑；
- 绑定保存不触发电机运动；
- revision；
- 原子持久化；
- 桌面/Web 使用同一状态快照。

---

## 12. 第四阶段：反馈信息

旧系统：

```text
position_source = host_pulse_accounting
measured = false
```

X42S UART 启用后必须区分：

```text
command_position
measured_position
position_error
```

不得把命令目标称为实际位置。

状态快照至少包含：

- logical_role；
- driver_id；
- online；
- enabled；
- command_position；
- measured_position；
- position_error；
- speed；
- current；
- temperature；
- reached；
- fault/protection；
- last_update_age_ms；
- telemetry_trusted。

GUI 和 Web 共用同一状态模型。

### 12.1 反馈刷新

第一版状态轮询可从 10~20 Hz 起步。

不要在未实测 UART 总线负载前默认 100 Hz 或更高。

运动控制命令优先级必须高于普通 telemetry 轮询。

---

## 13. 第五阶段：多轴同步运动

这是本项目最关键的验证阶段。

X42S 手册支持：

1. 广播相同命令；
2. 多电机命令；
3. 各电机缓存命令后使用同步触发。

但“同步开始”不自动证明与当前 ESP32 `SYNC` 的**全过程路径同步**等价。

当前 S4 使用统一五次进度：

```text
p(t) = 10s^3 - 15s^4 + 6s^5
q_swing(t)   = f(p)
q_support(t) = g(p)
```

two_mode 还会将非线性避障轨迹拆成多个角度段。

因此 UART 版必须分两步验证。

### 13.1 阶段 A：X42S 原生同步

先验证两个电机：

- 不同位置目标；
- 不同速度/加速度；
- 缓存；
- 同步触发；
- 实测启动时间差；
- 实测终点；
- 实测中间路径。

### 13.2 阶段 B：与现有 gait path 比较

离线/实机记录：

```text
t
q_swing_command
q_support_command
q_swing_actual
q_support_actual
```

与当前 StepDir SYNC 的目标路径比较。

必须定义最大允许：

- joint path error；
- ratio/path deviation；
- start skew；
- endpoint error；
- following error。

若 X42S 原生一次位置命令无法保持当前路径关系，则不得为了“全 UART”而降低 gait 安全性。

可选补救：

- 使用现有 two_mode segment 做更细的 UART 分段；
- 使用位置命令打断/更新能力做离散目标流；
- 暂时保留 StepDirBackend 执行 S4。

最终选择必须以实测轨迹误差为依据。

---

## 14. 安全与故障处理

必须定义：

- UART 超时；
- 单机离线；
- 地址冲突；
- X42S 堵转；
- 过流；
- 过热；
- 低压；
- 跟随误差过大；
- 非法位置；
- 超软件限位；
- ESTOP；
- PC 断线；
- ESP32 重启。

### 14.1 ESTOP

ESTOP 必须：

1. 停止正在排队的 X42S 动作；
2. 向相关 X42S 发停止/失能；
3. 废弃旧 motion generation；
4. 禁止旧 UART 回复重新把状态变为 RUNNING；
5. 不自动恢复动作。

软件停止不能替代物理断电急停。

### 14.2 跟随误差

新增可配置阈值：

```text
warning_error
abort_error
abort_hold_time
```

第一版只做监测和日志也可以，但字段和状态模型应预留。

---

## 15. PC 协议

PC -> ESP32 的现有串口接口应尽量保持兼容。

不要让 PC 直接控制 X42S UART 总线。

推荐保持：

```text
PC
  ↓ USB Serial
ESP32
  ↓ X42S UART
motors
```

PC 只面对 ESP32 的抽象协议。

可新增：

```text
X42S,S,<id>
X42S,SCAN
X42S,DIAG,<id>
```

但最终业务动作应优先通过逻辑角色/统一 motion API，而不是让 GUI 到处直接拼 X42S 原始帧。

---

## 16. 建议代码结构

PC：

```text
motor_control/
├── coordinated_control.py      # binding v2
├── actuator_model.py           # 可新增：执行器身份/状态
├── x42s_model.py               # 如有必要
├── gait_planner.py             # 机构数学保持
├── gait_executor.py            # 调用 backend
└── ...
```

ESP32：

```text
esp32_stepper/src/
├── x42s_bus.cpp/.h
├── x42s_protocol.cpp/.h
├── x42s_motion.cpp/.h
├── stepper.cpp                 # 旧 backend 保留
└── protocol.cpp                # 仅分发，不承载驱动实现
```

实际文件名可调整，但模块职责必须分离。

---

## 17. 测试要求

### 17.1 Python 单元测试

至少新增：

- binding schema v2；
- duplicate X42S ID；
- ID 0 拒绝；
- ID >255 拒绝；
- role mode 校验；
- v1 迁移；
- binding round trip；
- snapshot telemetry；
- offline/fault 状态；
- gait executor backend mock；
- legacy StepDirBackend 不回归。

### 17.2 ESP32 native/unit tests

协议纯函数尽量可 native 编译。

覆盖：

- frame encode；
- frame decode；
- checksum；
- wrong address；
- wrong function；
- timeout；
- unsolicited reached；
- request/reply matching；
- multi-motor command packing。

### 17.3 台架测试

顺序：

1. 1 台，无负载；
2. 1 台，机械负载；
3. 2 台总线；
4. 4 台总线；
5. 两台同步；
6. 4 台状态轮询；
7. gait 干跑；
8. 实机低速；
9. 故障注入；
10. 长时间运行。

---

## 18. 验收标准

### Milestone 1 — UART 单机

- ID1 稳定通讯；
- enable/stop/position/status 可用；
- X42S 内部闭环正常；
- 真实位置可读；
- 100 次动作无协议错配。

### Milestone 2 — 四机总线

- ID1~4 全部可发现；
- 地址唯一；
- 单机动作不会误驱其他电机；
- 单机掉线可精确定位；
- telemetry 连续稳定。

### Milestone 3 — Binding v2

- 现有绑定页改为 X42S ID；
- 无第二套绑定；
- v1 可控迁移；
- GUI/Web 一致；
- 保存/重启恢复。

### Milestone 4 — Feedback

- 显示 Command / Actual / Error；
- 不再把 host pulse accounting 当 X42S 实测；
- speed/current/temp/fault 可查询；
- stale data 明确标识。

### Milestone 5 — Synchronized gait

- UART 同步动作通过台架验证；
- S4 路径与旧 SYNC 基准做数据对比；
- 达到项目设定的路径误差阈值后才允许替换旧 StepDir S4；
- 未达到阈值时保留 StepDirBackend 回退，不强行上线。

---

## 19. 非目标

本分支暂不实施：

- RS485；
- CAN；
- 电机功率 PCB 重设计；
- 删除所有 STEP/DIR 代码；
- 用 UART 改写机构几何；
- 自动重新推导 gait；
- 直接合并未经验证的四轴全自动运行；
- 无物理测试情况下宣称 UART 原生位置模式与当前 quintic SYNC 完全等价。

---

## 20. 交付物

本分支实现完成时至少应包含：

1. binding schema v2；
2. v1 migration；
3. X42S UART ESP32 driver；
4. 四机 ID 支持；
5. 单轴闭环 UART 控制；
6. telemetry；
7. PC/ESP32 协议扩展；
8. GUI binding/status 更新；
9. Web 状态更新；
10. 多轴同步实验代码；
11. 单元测试；
12. 台架验证记录；
13. 硬件转接板接线说明；
14. 最终 handoff 文档。

---

## 21. 实施顺序

严格按以下顺序实施，不建议一次性“大爆炸”替换：

```text
A. X42S UART 协议纯函数
↓
B. 单台 ID1 台架
↓
C. 4 台地址总线
↓
D. telemetry
↓
E. binding schema v2 + UI
↓
F. X42SUartBackend 单轴
↓
G. 双轴 X42S 原生同步实验
↓
H. 与当前 quintic/S4 路径对比
↓
I. gait executor 接入
↓
J. 实机低速验证
↓
K. 决定是否停用 STEP/DIR
```

---

## 22. 关键设计决策总结

本项目此次改造不是简单“DM422 换 X42S”。

目标是：

> 用 4 台带唯一地址的 X42S 作为统一执行器，通过一组 ESP32-S3 TTL UART 总线完成闭环控制与状态反馈；X42S ID 直接整合进入现有逻辑电机绑定功能，并保持现有机构规划、非线性轨迹和安全逻辑的上层语义稳定。

同时保留一条原则：

> “通讯同步”不等于“机构路径等价”。在实测证明 X42S UART 同步轨迹满足当前 gait 路径要求之前，旧 STEP/DIR SYNC backend 必须保留为可验证基准和回退路径。
