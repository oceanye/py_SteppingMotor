# 四逻辑电机绑定与位置可视化实施计划

状态：已按计划实现并完成本机离线验证；待远端负责人合并与实机确认

基线：`24a438c`（`gitea/main`，2026-09-06 已同步）

规划分支：`codex/coordinated-step-control-2026-09-06`

输入：`docs/handoff control method.txt`、现有桌面 GUI、Web 控制、ESP32/Pico 步进协议

## 1. 本轮目标与明确边界

目标是为现有系统增加一层“机构逻辑电机”配置和监控能力：

1. 固定定义四个逻辑角色：`Mup1`、`Mr1`、`Mup2`、`Mr2`。
2. 允许现场人员把每个逻辑角色绑定到一个实际的步进轴编号。
3. 校验两路升降角色只能绑定直线轴，两路旋转角色只能绑定旋转轴。
4. 在桌面 GUI 中实时显示绑定后的角色、物理轴、软件位置、目标、协议进度、运行状态和可信度。
5. 在 Web 页面中只读同步显示同一份逻辑角色状态。
6. 保存配置后重启可恢复；损坏配置安全降级为未绑定。

本阶段明确不做：

- 不实现 A→C、B→A 或 S0–S7 自动换位。
- 不增加“一键自动运行”按钮。
- 不用 PC 顺序发送四条单轴命令冒充同步轨迹。
- 不把开环脉冲累计称为编码器实测位置。
- 不实现接触、支撑力、碰撞包络、限矩或过流联锁的虚假占位逻辑。
- 保存绑定时不自动改变物理轴模式、PPR、减速比、导程、位置或软限位。
- 本阶段不改变 DM422/TMC2209 等驱动器选型；绑定模型只依赖 PUL/DIR 步进轴抽象。

## 2. 已确认事实与推荐映射

项目中存在两个编号会重叠的电机命名空间：

- 步进轴：全局编号 `0..29`，其中 `0..5` 为 ESP32 本地轴，`6..29` 为 6 个 Pico 节点各 4 轴。
- FOC/GEAR 闭环轴：独立编号 `0..1`。

四个逻辑角色必须绑定“步进轴”，不能绑定到 FOC/GEAR 编号。任何 UI、日志和 API 都必须写成“步进轴 N”，不能只显示裸数字。

根据 `docs/HANDOFF_2026-08-24.md`，当前硬件的建议映射为：

| 逻辑角色 | 固定用途 | 必须模式 | 建议物理轴 | 当前物理标签 | 接线/路由 |
|---|---|---|---:|---|---|
| `Mup1` | 左侧升降 | `linear` | 步进轴 0 | 左侧直 | ESP32，PUL GPIO5 / DIR GPIO6 |
| `Mr1` | 左侧旋转/换位旋转 | `rotary` | 步进轴 2 | 左侧转 | ESP32，PUL GPIO1 / DIR GPIO2 |
| `Mup2` | 右侧升降 | `linear` | 步进轴 1 | 右侧直 | ESP32，PUL GPIO7 / DIR GPIO15 |
| `Mr2` | 右侧旋转/支撑切换 | `rotary` | 步进轴 3 | 右侧转 | ESP32，PUL GPIO4 / DIR GPIO8 |

安全决策：这只是“建议映射”，不是自动生效的默认值。首次没有配置文件时，四个角色均为未绑定；操作员点击“填入建议映射”后仍需现场确认并显式保存。

以下信息必须由现场确认，代码不能猜测：

- 四根电机线与步进轴 0/1/2/3 是否仍按上表接线。
- `Mup1/Mup2` 的轴正方向是否等于机械向上。
- `Mr1/Mr2` 的轴正方向是否等于方法文档中的 `q1+ / q2+`。
- 四轴的机械零位、合法行程/角度范围、微步设置和实际减速比。
- `Mr1` 自转/公转参考系与横梁角 `beta` 的来源。

方向、零偏和机构几何标定不是 v1 绑定字段。未完成这些标定时可以显示物理轴软件坐标，但不得计算或宣称可信的机构 `h/q` 坐标，也不得启用自动换位。

## 3. 位置数据的真实含义

现有 DM422/DM442 步进通道是开环 PUL/DIR。`AxisRuntime.position_steps` 是 PC 根据已确认执行的脉冲建立的软件账本，不是传感器实测位置。

现状限制：

- `STEP,...,P,...` 进度帧目前只更新单轴 Tk 进度条。
- 规范位置只在 `DONE/ABORT` 后结算。
- ESP32 本地轴目前每 500 脉冲才上报一次，短行程可能直接收到 `DONE`。
- Pico 固件已有约 250 ms 的进度上报节拍。
- 六路 AS5600 诊断默认关闭，而且即使安装也只是诊断，不参与闭环控制。
- 进程首次启动且没有校准文件时，当前代码会把位置初始化为可信；实施时必须修正为“不可信”。

所有页面统一采用以下语义：

- `committed_position`：最近一次 `DONE/ABORT` 结算后的软件位置。
- `projected_position`：仅根据实际收到的 `STEP,P` 帧推算的运动中软件位置。
- `position_source = "host_pulse_accounting"`。
- `measured = false`。
- 中文固定显示“软件脉冲估算”，不能显示“实际位置”或“编码器位置”。
- `position_trusted=false` 时仍可显示估算数值，但必须带“约/需校准”警告。

运动中的投影公式为：

```text
projected_steps = committed_steps
                + pending_signed_steps * clamp(executed_steps / requested_steps, 0, 1)
```

`projected_position` 只能在生成只读快照时计算，不能提前写入 `AxisRuntime.position_steps`，否则 `DONE` 时会重复累计。没有收到进度帧时显示“运动中，暂无中间进度”，禁止按时间插值伪造位置。

如果用户最终要求真正的物理实时位置，需要另外安装和验证：

- 两路直线位移传感器/编码尺；
- 两路适合多圈与掉电恢复的旋转编码器；
- 原点/硬限位；
- 对应采样、标定、失联与偏差检测协议。

更换为 TMC2209 本身不会把开环步数变成实测位置。

## 4. 领域模型设计

新增纯 Python 模块：`motor_control/coordinated_control.py`。该模块不得依赖 Tk、HTTP、串口或全局 GUI 状态。

建议类型：

- `LogicalRole`：固定枚举 `Mup1 / Mr1 / Mup2 / Mr2`，顺序稳定。
- `RoleSpec`：固定显示名、机构侧、动作说明和 `required_mode`。
- `ActuatorBinding`：v1 仅包含 `kind="stepper"` 和 `axis`。
- `BindingSet`：完整四角色快照、schema 版本和 revision。
- `AxisMotionTelemetry`：请求/执行步数、开始时间、最后进度时间、终态。
- `LogicalMotorSnapshot`：绑定与物理轴状态组合后的只读输出。
- `BindingValidationError`：带稳定错误码和中文可显示信息。

建议持久化 schema：

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

未绑定项使用 `null`：

```json
"Mup1": null
```

解析和校验规则：

1. 顶层必须是对象，`schema_version` 必须恰好为 `1`。
2. `bindings` 的 key 必须恰好包含四个固定角色，不能缺少或增加未知角色。
3. `kind` 在 v1 必须恰好为 `stepper`。
4. `axis` 必须是真正的整数；明确拒绝 bool、浮点、字符串。
5. 轴范围复用 `topology.validate_stepper_axis()`，即 `0..29`。
6. 同一物理步进轴最多绑定一个逻辑角色。
7. `Mup1/Mup2` 绑定轴的已应用模式必须为 `linear`。
8. `Mr1/Mr2` 绑定轴的已应用模式必须为 `rotary`。
9. 绑定轴的 `axis_param_valid` 必须为真。
10. 允许保存部分未绑定的完整快照，但此时 `configuration_valid=false`。
11. 绑定只引用物理轴自己的 profile/runtime；换绑时不复制校准和位置。
12. 建议映射由代码常量生成，但不能由 loader 自动保存或自动确认。

绑定健康度和自动控制就绪度必须分开：

- `binding_valid` 只说明角色、轴号、唯一性、模式和参数有效。
- `automation_ready` 本阶段固定为 `false`。
- 四个绑定全部有效也不代表硬件就绪。

## 5. 持久化与原子更新

建议新增独立运行时文件 `.coordinated_bindings.json`，不要加入 `.stepper_axis.json`。原因是旧版 `_save_axis_config()` 会重建并覆盖轴配置对象，独立文件可避免旧客户端无意擦除绑定，也保持“单轴物理参数”和“跨轴逻辑关系”分离。

修改点：

- `motor_control/state_store.py`
  - `StatePaths` 增加 `coordinated_bindings`。
  - 增加 `load_coordinated_bindings()` / `save_coordinated_bindings()`。
  - 继续复用现有临时文件 + `fsync` + `os.replace` 原子写入。
- `.gitignore`
  - 忽略 `.coordinated_bindings.json` 及其临时文件。

加载策略：

- 文件不存在：四角色均未绑定，不报警，不自动落盘。
- JSON 损坏、schema 未知、角色缺失、重复或越界：整套拒绝，四角色回到未绑定，并显示启动警告。
- 损坏文件不得被启动过程自动覆盖，保留给现场排查。
- 模式不匹配或轴参数失效：保留可诊断的绑定值，但状态标为 invalid，禁止未来自动消费者使用。

保存策略必须是一次提交完整四角色快照，不能逐行修改产生临时重复绑定。建议控制器流程：

1. 在锁外解析候选对象。
2. 在 `state_lock` 下校验候选，并取“旧绑定轴 ∪ 新绑定轴”。
3. 对涉及的所有轴调用 `_axis_motion_active_locked()`；任何一轴忙均拒绝。
4. 用唯一 reservation token 暂时占用所有涉及轴，防止写盘期间启动运动或修改模式。
5. 原子写入新文件。
6. 仅写入成功后，在 `state_lock` 下替换内存快照并递增 revision。
7. 无论成功或失败均释放 reservation；失败时旧内存和旧文件保持有效。

绑定修改不得发送串口命令。

轴模式的反向约束也要补齐：已绑定的 `Mup*` 轴不能被改成 `rotary`，已绑定的 `Mr*` 轴不能被改成 `linear`。桌面 `_on_axis_mode_change` 和 Web 轴配置入口必须走同一条控制器校验；提示用户先解除绑定，不能静默使绑定失配。

## 6. 共享运行状态与快照

`desktop_app.py` 仍负责线程安全和串口生命周期，但不要把业务判定散落到 Tk 控件中。

新增共享状态建议：

- `self.control_bindings`
- `self.binding_revision`
- `self.axis_motion_telemetry[NUM_STEPPER_AXES]`
- `self.pico_node_health`（未实现节点缓存前统一为 `unknown`）
- `self.safety_state`（至少区分 normal/software_estop/hardware_estop/disconnected）

统一提供一个锁内快照入口，例如：

```text
snapshot_logical_motors_locked() -> tuple[LogicalMotorSnapshot, ...]
```

桌面 UI 和 `web_get_status()` 必须消费同一个快照构造器，避免两处分别推导状态后出现不一致。

事件更新规则：

- MOVE 已被接受：保存 signed pending steps、requested、开始时间，状态 `STARTING/MOVING`。
- `StepProgress`：在 `state_lock` 下更新 executed/requested/last_update；然后通知 UI 刷新。
- `DONE`：按现有逻辑只结算一次 committed position，记录终态并清空 transient progress。
- `ABORT`：按固件实际 executed 比例结算，位置标为不可信，记录 `ABORTED_UNTRUSTED`。
- 命令拒绝/发送失败：清空与本次 generation 匹配的 transient 状态，不影响较新的运动。
- STOP：显示 `STOPPING`，等待真实 `ABORT/DONE`，不能立即假设已经停止。
- 断连/ESTOP：清除未完成投影；若断连时轴在动或终态未知，位置必须标为不可信。
- profile、校准、原点、软限位或绑定变化：立即生成新快照。

建议状态优先级：

```text
UNBOUND
INVALID
DISCONNECTED
NODE_OFFLINE / NODE_UNKNOWN
ESTOP
STOPPING
STARTING
CONTINUOUS
MOVING
ABORTED_UNTRUSTED
IDLE_UNTRUSTED
IDLE
```

远端轴需要特别处理：PC 串口连接 ESP32 不等于对应 Pico 在线。当前 `NodeOfflineEvent` 只写日志，未缓存节点状态；在增加心跳/状态缓存前，远端角色必须显示 `NODE_UNKNOWN`，不能显示“硬件就绪”。

## 7. 桌面配置与可视化页面

新增 `motor_control/ui/coordinated_tab.py`，在 `motor_control/ui/main_window.py` 的主 Notebook 中增加 `🧭 电机绑定与状态`。推荐放在 30 轴步进页之前或紧随其后。

页面分四区。

### 7.1 安全说明横幅

固定显示：

- “本页只配置绑定和监控，不会执行自动 A↔C 换位。”
- “位置来自主机脉冲累计，不是编码器实测。”
- “接触、支撑力/电流、碰撞裕量和硬限位尚未形成完整联锁。”

### 7.2 逻辑电机绑定表

四行固定顺序：`Mup1`、`Mr1`、`Mup2`、`Mr2`。

列：

- 逻辑角色；
- 固定动作说明；
- 要求模式；
- 物理步进轴下拉框；
- 物理标签和路由；
- 当前已应用模式；
- 参数/位置可信状态；
- 校验原因。

下拉项格式示例：

```text
步进轴 2 · 左侧转 · ESP32 本地 · PUL GPIO1 / DIR GPIO2
步进轴 6 · P1-1 · RS485/Pico 1 · 本地轴 1
```

交互：

- `填入建议映射`：只修改草稿，不保存。
- `验证并保存绑定`：一次应用四项。
- `全部解除绑定`：生成四个 null 的草稿，二次确认后保存。
- `跳转到轴配置`：模式不匹配时导航到对应物理步进轴，不自动改模式。
- 下拉列表可禁用被其他草稿行占用的轴，但服务端/控制器仍做最终校验。
- 保存失败时保留本地草稿，已应用映射不变。
- 运动期间保存按钮禁用；控制器仍必须二次拒绝竞态。

### 7.3 四角色状态卡

采用 2×2 布局：左侧 `Mup1/Mr1`，右侧 `Mup2/Mr2`。

每张卡显示：

- 角色、动作、绑定的物理步进轴与路由；
- 期望/实际模式和单位；
- committed position；
- 有真实进度帧时的 projected position；
- 目标位置或本次命令增量；
- executed/requested 与百分比；
- 当前状态、位置可信度、数据来源；
- 最后更新时间和 stale 标记。

图形：

- `Mup1/Mup2` 用竖直标尺。只有 min/max 均有效时才显示归一化填充；否则只显示数值，避免虚构行程比例。
- `Mr1/Mr2` 用圆形刻度盘。指针可显示 `position mod 360°`，旁边保留精确多圈数值；没有可信零位时加明显警告。

颜色语义：

- 蓝：正在运动；
- 绿：绑定、参数、软件位置账本均有效且空闲，仅表示“软件状态正常”；
- 黄：未绑定、需校准、数据陈旧或远端在线未知；
- 红：重复/越界/模式失配、断连、ESTOP 或明确离线。

绿灯绝不能命名为“硬件安全就绪”。

Tk 刷新建议使用 `root.after(100~200 ms)` 拉取一次线程安全快照；锁内只复制数据，Canvas/Label 更新必须在主线程和锁外完成。关闭窗口后必须取消 timer，避免 Tk 回调访问已销毁控件。

### 7.4 协调控制就绪度

本阶段固定显示 `未就绪`，并列出真实 blocker：

- 方向与零位未完成实机标定；
- 接触状态不可用；
- 支撑力/扭矩/过流反馈不可用；
- 碰撞包络未验证；
- 硬限位链不完整；
- 四轴同步预约/启动协议未实现；
- Pico 节点在线状态未完整缓存。

本区不放“开始”“执行”“继续”按钮。

## 8. Web 页面配套策略

现有 Web 控制没有认证，只适合可信局域网。物理角色换绑会改变后续控制含义，风险不低于原点/限位设置，因此 v1 采用：

- 桌面 GUI：可查看、编辑并保存绑定。
- Web：只读查看绑定和实时状态；明确提示“请在桌面 GUI 修改绑定”。
- Web 保留现有单轴 STOP 和全局 ESTOP。
- 不增加绑定写接口，不增加 `/api/coordinated/start`。

`GET /api/status` 增加 additive 字段，保留所有现有字段：

```json
{
  "coordinated_control": {
    "binding_revision": 1,
    "configuration_valid": true,
    "automation_ready": false,
    "edit_scope": "desktop_only",
    "blockers": [
      "contact_feedback_unavailable",
      "collision_model_unverified"
    ]
  },
  "logical_motors": [
    {
      "role": "Mup1",
      "display_name": "左侧升降",
      "required_mode": "linear",
      "binding": {"kind": "stepper", "axis": 0},
      "physical_label": "左侧直",
      "controller": "esp32",
      "node": null,
      "mode": "linear",
      "unit": "mm",
      "committed_position": 12.3,
      "projected_position": 12.8,
      "position_trusted": true,
      "position_source": "host_pulse_accounting",
      "measured": false,
      "state": "MOVING",
      "executed_steps": 400,
      "requested_steps": 1000,
      "progress_percent": 40,
      "last_update_monotonic_ms": 123456
    }
  ]
}
```

同时给 `stepper_axes` 增加单位无关字段 `position/travel_min/travel_max`，暂时保留历史命名 `position_mm/travel_min_mm/travel_max_mm` 兼容旧客户端。

`web/index.html` 增加“四逻辑电机监控”区，复用同一轮 `/api/status` 刷新。增加 refresh-in-flight 护栏，避免网络慢时多个刷新重叠。卡片逻辑不能在浏览器自行积分步数或按时间预测。

如果未来确实要求网页修改绑定，必须作为独立变更先增加认证/权限、CSRF/来源限制和审计日志，然后才增加全量原子 POST；不能在本阶段顺手开放。

## 9. 固件进度上报

为了让本地 ESP32 轴达到接近 Pico 的可视化刷新频率，建议在主机状态链路完成后单独修改 `esp32_stepper/src/stepper.cpp`：

- 保持现有协议形状 `STEP,<axis>,P,<done>,<total>` 不变。
- 从“每 500 步”改为“约每 250 ms 或达到合理步数阈值时上报一次”。
- 不发送 `done=total` 的重复 P 帧，最终仍由 `DONE/ABORT` 唯一结算。
- 对短运动允许直接终态，不人为延迟运动等待 UI 刷新。
- 限制串口输出频率，避免多轴运行时日志/进度阻塞脉冲任务。

Pico 已使用约 250 ms 节拍，只需验证 ESP32 转发后的全局轴号与时间戳/陈旧状态，不需要改变其协议。

## 10. 文件级实施清单

### 新增

- `motor_control/coordinated_control.py`
  - 四角色规范、schema、解析/序列化、校验、状态投影。
- `motor_control/ui/coordinated_tab.py`
  - 绑定表、四状态卡、只读就绪度面板。
- `tests/test_coordinated_control.py`
  - 纯领域模型和快照测试。

### 修改

- `motor_control/state_store.py`
  - 新路径和绑定 load/save。
- `motor_control/desktop_app.py`
  - 初始化/加载绑定、原子应用、运动 telemetry、统一快照、UI 事件代理。
- `motor_control/ui/main_window.py`
  - 插入新主 Tab。
- `motor_control/ui/__init__.py`
  - 导出新 view builder。
- `motor_control/web_adapter.py`
  - 在同一锁边界投影 `logical_motors/coordinated_control`。
- `web/index.html`
  - 增加只读四卡和刷新护栏。
- `esp32_stepper/src/stepper.cpp`
  - 后续独立提交调整进度节拍。
- `.gitignore`
  - 忽略新的运行时绑定文件。
- `tests/test_state_store.py`
  - round-trip、损坏配置和原子写失败。
- `tests/test_desktop_axis_integration.py`
  - P/DONE/ABORT、位置只结算一次、保存与运动竞态。
- `tests/test_web_adapter.py`
  - 四角色快照与现有物理轴状态一致。
- `tests/test_web_control.py`
  - 页面只读区、additive status、确认不存在绑定 POST。
- `docs/ARCHITECTURE.md`
  - 增加“物理拓扑—逻辑绑定—协调器”的模块边界。
- `docs/WEB_CONTROL.md`
  - 标明桌面可编辑、Web 只读及位置来源。
- `docs/HANDOFF.md`
  - 增加本功能交接索引。

## 11. 推荐实施顺序与提交边界

### 阶段 A：纯模型与持久化

- 建立四角色和 schema。
- 完成严格校验、默认全未绑定、建议映射常量。
- 增加独立原子 JSON 存储。
- 先完成纯单元测试。

建议提交：`feat(control): add logical stepper binding model`

### 阶段 B：控制器共享状态

- 加载/应用绑定及 revision。
- 增加全量原子换绑和 involved-axis reservation。
- 保护已绑定轴的模式变更。
- 修正“无校准文件却默认 trusted”的问题。
- 建立统一逻辑电机快照。

建议提交：`feat(control): persist and project logical motor bindings`

### 阶段 C：真实进度链路

- 保存 StepProgress transient telemetry。
- DONE/ABORT 清理并确保只结算一次。
- 覆盖断连、STOP、ESTOP、旧 generation 晚到事件。
- 不做时间插值。

建议提交：`feat(control): expose pulse-reported motion telemetry`

### 阶段 D：桌面配置页

- 新建绑定与状态 Tab。
- 完成草稿/已应用分离、建议映射、验证保存、解除绑定。
- 完成直线标尺、旋转表盘、状态与 blocker。

建议提交：`feat(gui): add logical motor binding and status page`

### 阶段 E：Web 只读配套

- 扩展 `/api/status`。
- 新增四卡监控，保留旧字段兼容。
- 确认网页无绑定写入口、无自动换位入口。

建议提交：`feat(web): show logical motor state read-only`

### 阶段 F：固件刷新率与文档

- 单独调整 ESP32 P 帧节拍并做 native/build 验证。
- 更新架构、Web、handoff 文档。
- 记录离线验证和现场待验项目。

建议提交：`fix(firmware): bound step progress reporting cadence`
建议提交：`docs: hand off logical motor binding and monitoring`

每一阶段都应保持可独立回滚；不要把绑定模型、Tk UI、固件和未来自动轨迹混在一个大提交中。

## 12. 离线测试矩阵

### 12.1 纯模型

- 固定角色及顺序稳定。
- 文件缺失默认全 null。
- 建议映射为 Mup1→0、Mr1→2、Mup2→1、Mr2→3，但不会自动提交。
- 有效映射 round-trip。
- 缺角色、多角色、未知角色、未知 schema 全部拒绝。
- `-1/30/bool/float/string` 轴号全部拒绝。
- 重复物理轴拒绝。
- 四项一次 swap 成功，不出现中间重复状态。
- Mup→rotary、Mr→linear、参数 invalid 均产生稳定错误码。
- 绑定/换绑不改变 AxisProfile、AxisRuntime、位置、限位，也不调用串口 send。

### 12.2 持久化

- 无文件、正常文件、损坏 JSON、顶层非对象、未知 schema。
- 完整 save/load round-trip。
- `os.replace` 失败时旧文件和旧内存映射保持不变。
- 启动读取损坏文件后不会自动覆盖它。

### 12.3 并发与位置

- old/new 任一轴 running/in_progress/dispatching/reserved/web_pending/pending_step 时拒绝换绑。
- 保存占用期间 MOVE 和模式更改被拒绝，STOP/ESTOP 始终允许。
- MOVE 接受前后、发送失败、命令拒绝状态正确。
- P 帧更新 projected position，但不改变 committed position。
- DONE 只累计一次；迟到/重复 DONE 不重复累计。
- ABORT 按 executed/requested 比例结算并置不可信。
- 没有 P 帧时不生成伪进度。
- 旧 generation 的迟到 P/DONE 不污染新运动。
- 运动中断连/ESTOP 清除投影并标记位置不可信。

### 12.4 桌面 UI

- 四行绑定、四张卡固定存在。
- 建议映射只填草稿。
- 刷新不会覆盖未保存草稿。
- 保存失败后草稿保留、已应用值不变。
- 模式不兼容、重复轴、未绑定、需校准、断连、Pico unknown 均显示明确原因。
- 所有 Tk 控件只在主线程更新。
- 页面关闭后无残留 after 回调异常。
- 任何绑定按钮均不会触发 MOVE。

### 12.5 Web/API

- `/api/status` 始终返回四个稳定顺序的 logical motor 条目。
- 每项与同一次锁内 `stepper_axes` 快照一致。
- linear 使用 mm，rotary 使用 °。
- 旧 `position_mm` 字段继续存在。
- 页面显示软件估算和 measured=false 的含义。
- 慢请求不会产生刷新重叠。
- 不存在绑定 POST 和自动协调执行路由。

### 12.6 建议离线命令

```powershell
python -m unittest discover -s tests -v
python -m compileall motor_control pc_gui.py web_control.py
```

若本机已有 PlatformIO/toolchain：

```powershell
platformio test -d esp32_stepper -e native_test
platformio test -d rp2040_stepper_node -e native_test
platformio run -d esp32_stepper -e esp32s3_gear
platformio run -d esp32_stepper -e esp32s3_gear_remote
platformio run -d rp2040_stepper_node -e pico
```

Web 内嵌 JavaScript 还应抽取后执行 Node 语法检查。没有真实浏览器、串口和电机时，只能写“离线通过”，不能写“硬件验证通过”。

## 13. 远端现场验收清单

1. 断开电机功率，保存四角色绑定，重启 GUI 后回读一致。
2. 现场确认 0/1/2/3 的实际线缆、标签和机构角色。
3. 对每轴单独低速、短距离点动，记录机械正方向；任何一轴运动异常立即 STOP/断电。
4. 校准两路直线轴零位、mm/步和软限位。
5. 校准两路旋转轴零位、°/步、减速比和多圈方向。
6. 对比桌面与 Web，确认角色、轴号、路由、模式、单位、可信度一致。
7. 长短两种单轴运动：只有收到真实 P 帧才改变 projected position，DONE 后只结算一次。
8. 中途 STOP：ABORT 后位置标为不可信，重新校准前不显示绿色。
9. 运动中拔掉串口/触发 ESTOP：状态和可信度安全降级。
10. 如果绑定 Pico 轴，分别验证 node unknown/offline/online；不能把 ESP32 串口已连接当作 Pico 在线。
11. 重复轴、Mup→rotary、Mr→linear、运动中换绑必须全部被明确阻止。
12. 本阶段不进行 A→C、B→A 或 S0–S7 自动运动验收。

现场验收记录应包含：固件 commit、PC commit、四轴映射、方向、零位、PPR、微步、减速比/导程、软限位、测试速度、测试距离/角度、STOP/ESTOP 结果和操作人。

## 14. 自动换位进入下一阶段前的硬门槛

后续 AI 不得仅凭“绑定有效”开始实现或开放自动 S0–S7。至少需要先确定：

- `left_contact/right_contact/三低点接触` 的真实传感器和协议；
- 支撑载荷或电机电流/扭矩反馈及阈值；
- 机械净空、碰撞包络和 `collision_margin()` 的可验证模型；
- 四轴方向、零位、横梁 `beta` 和 q/h 坐标变换；
- 四轴从预检到终态的协调器 ownership；
- 四轴原子预约、同步启动/时间戳执行或可证明的误差边界；
- 通信失联、节点离线、过流、异常接触、硬限位的停机状态机；
- 自动恢复被禁止，故障后必须人工确认；
- 仿真、台架低速、单阶段、全流程逐级验收方案。

handoff 方法文档中的 `TORQUE_HOLD`、`motor_overcurrent()`、`unexpected_contact()` 和 `collision_margin()` 当前都是需求/伪代码，不是仓库已有能力。

## 15. 文档整理建议

当前 `docs/handoff control method.txt` 为未跟踪的 GBK/GB2312 文本，普通 UTF-8 工具可能显示乱码；原文件应保持不动，避免破坏用户内容。后续在获得确认后：

1. 另存为 UTF-8 Markdown，例如 `docs/COORDINATED_CONTROL_METHOD.md`。
2. 保留原文，并把公式标成“待实机验证的运动学假设”。
3. 标出 q1/q2 正负号、安装参考系和零偏尚未标定。
4. 把接触、支撑力/电流、碰撞检测明确标成“要求但当前不可用”。
5. 加入建议绑定表和“建议不等于已确认”的说明。
6. 原文“高点扫掠验证图”后目前没有实际图片，需要补充或删除引用。

## 16. 完成定义

只有同时满足以下条件，绑定/监控功能才可交付审核：

- 四角色模型、独立持久化、原子换绑和严格校验全部实现。
- 桌面可编辑，Web 只读，二者消费同一份线程安全快照。
- 页面明确标注软件估算、measured=false 和可信度。
- 运动中位置只来自真实 P 帧，DONE/ABORT 无重复计数。
- invalid/unbound/disconnected/node unknown/ESTOP 不会被显示为就绪。
- 自动换位入口仍不存在且 `automation_ready=false`。
- 全部离线测试通过，未完整环境限制已写入 handoff。
- 远端同事在真实硬件上完成映射、方向、零位、运行、STOP/ESTOP 验收后再合并。

本文件所列阶段 A–F 已在独立分支实现。后续工作应先执行远端现场验收，并满足第 14 节
全部硬门槛；不得直接跳到自动轨迹。
