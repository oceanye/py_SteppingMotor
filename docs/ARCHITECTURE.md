# PC 控制端架构

当前硬件（2026-10-21审计）：ESP32-S3 → TTL-UART → 四台 X42S。
`X42sActuatorState` 的反馈经减速比/导程换算到角色单位，`AxisProfile/Runtime`
和30轴Pico拓扑仅用于历史 STEP/DIR 维护。GUI默认不开放该控制路径；
默认固件不初始化其 GPIO/任务，并拒绝旧运动/诊断命令。GEAR及轨道D为独立子系统，不因DM退役移除。

`pc_gui.py` 是兼容启动入口。PC 是 USB 串口的唯一主站；Tk 界面和局域网 HTTP
服务共同调用桌面控制层，不允许各自直接读取串口。

```text
Tk views ─────────┐
                  ├── desktop controller ── SerialSession ── ESP32-S3
LAN Web API ──────┘          │                       │
                      X42sActuatorState         X42S UART × 4
                              │
                    logical-role bindings
                    Mup1/Mr1/Mup2/Mr2
                              │
                           StateStore
```

## 模块边界

| 模块 | 责任 | 明确不负责 |
|---|---|---|
| `motor_control/topology.py` | 0..29 全局轴号、本地 ESP32 与 Pico 节点映射 | UI、串口 I/O |
| `motor_control/axis_math.py` | 参数校验、mm/°/steps、速度/延时换算 | 持久化、运动调度 |
| `motor_control/axis_model.py` | `AxisProfile` 与以 steps 为规范坐标的 `AxisRuntime` | Tk 控件 |
| `motor_control/coordinated_control.py` | 四个机构逻辑角色、绑定校验、进度投影和跨端状态快照 | 串口下发、自动轨迹 |
| `motor_control/x42s_units.py` | 唯一X42S电机轴/机构单位/脉冲换算与共享常量 | Tk、串口、方向标定策略 |
| `motor_control/x42s_runtime.py` | 桌面X42S点动、整组校验/失败停车、ACK/到位跟踪、轮询与遥测合并 | 创建窗口、建立串口会话、几何规划 |
| `motor_control/gait_recovery.py` | 执行前参考指纹、四轴实测到位核验 | 自动反向运动或绕杆规划 |
| `motor_control/gait_recovery_ui.py` | 恢复说明、人工确认、二次核验及账本原子更新 | MOVE/ZERO/EN |
| `motor_control/ui/actuator_cards.py` | 按机构坐标绘制升降/旋转卡片，失信不画有效指针 | 原始电机角冒充输出角 |
| `motor_control/protocol.py` | 文本命令编码、typed 回复和异步事件解析 | 线程、串口设备 |
| `motor_control/serial_session.py` | 唯一 reader、请求串行化、回复匹配、事件分流 | 业务状态、Tk |
| `motor_control/state_store.py` | 兼容旧 JSON 的读取和原子替换 | 参数含义和 UI |
| `motor_control/ui_dispatch.py` | 工作线程到 Tk 主线程的 post/call | 业务规则 |
| `motor_control/ui/` | 主窗口、逻辑电机、步进、FOC/GEAR、轨道的 Tk 布局 | 串口和协议解析 |
| `motor_control/autotune.py` | 可注入 I/O/时间的 FOC、GEAR 扫描与评分 | Tk 控件 |
| `motor_control/web_adapter.py` | 将 HTTP 控制契约适配到桌面控制状态 | HTTP 路由、Tk 导入 |
| `motor_control/web_contract.py` | HTTP 服务依赖的控制接口 | HTTP 实现 |
| `motor_control/desktop_app.py` | 组合模块、生命周期和 GUI 事件协调 | HTTP 路由、协议文本解析 |
| `web_control.py` | HTTP 路由、输入边界和静态页面 | Tk、串口协议 |
| `web/*.js` | 通用请求、四角色监控、历史维护、页面控制四模块 | 直接串口、X42S步态写入 |

## 不变量

1. 历史 STEP/DIR 位置和软件限位内部以脉冲坐标保存。`linear`/`rotary` 只改变显示和命令单位；
   切换模式不得改变等效脉冲位置。
2. 运动中或 MOVE 已预约但尚未发送时不得切换模式/物理参数；参数非法、位置不可信或
   目标越过软件限位时不得下发 MOVE。绝对定位必须在预约后按最新 steps 计算差值。
3. `SerialSession` 是唯一串口 reader，同时只允许一个请求等待回复。STEP、硬急停、
   轨道超时和节点掉线属于异步事件，不能被下一条同步请求误取。
4. ESTOP/断线会使旧控制代数失效；单轴 STOP 会递增该轴 motion generation。排队中的
   普通命令在真正写串口前必须同时检查控制代数、逐轴代数和 reservation 身份。
5. Tk 只能由主线程访问。需要向 HTTP 调用者确认“已经应用”的操作使用同步 UI call，
   普通显示刷新使用异步 post。
6. `.stepper_axis.json`、`.stepper_calib.json`、`.foc_tune.json`、
   `.gear_tune.json` 和 `.coordinated_bindings.json` 保持各自 schema 可读；写入必须先落
   临时文件再原子替换。校准文件的
   可选 `_profile` 元数据用于检测跨文件部分写入，不匹配时禁止信任旧位置。
7. HTTP 路径、串口文本和 `pc_gui.py` 启动方式属于现场兼容接口，结构重构不得顺便修改。
8. 串口请求超时后无法证明下一条同形回复属于哪个请求，因此必须废弃会话并重连。
9. ESP32 发往 PC 的每个运行期文本帧必须在共享 TX mutex 内整行输出；不得从不同任务
   分段 `Serial.print`。步进延时上限为 10,000,000 us，远端 RP2040 下限为 100 us。
10. 当前四角色均绑定唯一 X42S ID：`Mup1/Mup2` 单位mm，`Mr1/Mr2` 单位°。
    历史后端才要求绑定linear/rotary步进轴。绑定保存不发送运动命令；运动占用时禁止换绑。
11. X42S 使用 `x42s_driver` 位置反馈，指令目标单列，不能冒充当前位置；
    reached/速度/遥测可信度判断运动，不以“目标字段非空”判断忙碌。
    历史后端才使用 `host_pulse_accounting`。整机姿态和间隙仍依赖机械与支点模型，非外部测量。
    三足分阶段步态有独立预检，不以只读绑定页的自动化状态替代执行门槛。
12. X42S手动MOVE的ACK只证明命令接受。四轴换绑、记录基准及开始步态还检查待完成
    命令；新的可信到位实测或精确STOP确认才能清除。双升降先整组验证，再顺序下发；
    任一失败则停止所有已尝试启动的目标，停止未确认必须提示现场急停。
13. 恢复前站位只核验，不生成未经避让规划的逆向路径。四轴到位、参考未变、人工确认
    且确认后再检查一次，才恢复地图账本；不把ACK当DONE，也不发送驱动器ZERO。

## 2026-10-22：步态坐标与 X42S 执行一致性

- `gait_coordinates.py`：统一顺时针模型角、世界投影、四动作映射和实际红杆侧。
  UI 的 L+/R+ 是顺时针动作、模型弧向 −60°，不等于“正弧向”。
- `gait_clearance.py`：完整径向腿胶囊对高杆的表面间隙/扣 δ 后余量，规划与 GUI 共用。
- `gait_x42s_plan.py`：将名义角度折线细分为可覆盖两轴任意进度差的关节矩形；
  不能把独立梯形 MOVE 当成原子五次 SYNC，不能证明无限超调或错误实测尺寸安全。
- `x42s_motion.py`：本次命令身份、目标与发令后新鲜静止实测的到位证据。
- `gait_x42s_wait.py`：双轴轮询、公平等待、故障/越界即中止；前台每0.3秒最多发一条查询。
- `scripts/gait_audit_report.py`：纯离线生成两摆放×四动作矩阵、统计已归档实机日志；不打开串口。

历史 `initial_placement` 键与支座/β 保持兼容；镜像后不要按键名推断实际红杆左右。
图形、标签和基准确认从支座/高杆坐标派生。旧方向参数不自动翻转，旧标定文件不自动写回。
GUI 的当前/已执行窗口/名义计划最低值分别显示；750ms跨轴采样时差只冻结姿态，不伪造同步反馈。
完成步态时四轴实测再次核验，站位账本只原子提交一次。清路径不重置累计步数。

## 线程所有权

- Tk 主线程：控件和 Tk 变量、应用轴配置、界面刷新。
- SerialSession reader：只读串口、解析并分发 typed 消息。
- 控制 worker：串行请求、自动调参和等待固件回复，不直接访问 Tk；MOVE 在 Tk/HTTP
  接受边界先取得逐轴 reservation，worker 只能消费自己的 token。
- HTTP worker：参数校验和调用控制接口；需要 Tk 状态变更时等待 UI dispatcher 完成。

## 扩展轴或协议

- 改轴数时先更新 `topology.py`，再同步 ESP32 `config.h` 和浏览器 status 渲染；必须测试
  边界轴 `0/5/6/29` 以及非法轴 `30`。
- 增加固件异步输出时，先在 `protocol.py` 定义 typed event，再由桌面事件处理器消费。
  未识别文本必须保持为 `UnknownMessage`，不能宽松匹配成任意请求回复。
- 增加同步命令时，为其补充精确 reply matcher 和 fake-serial 交错测试，再接入 GUI。

## 离线与现场验证

本机运行Python/Tk完整回归、fake-serial、生产HTTP适配器及Playwright多浏览器测试。
命令和浏览器覆盖见[网页测试说明](WEB_CONTROL.md#模块与浏览器回归)。桌面测试使用临时
配置并禁止serial.Serial，不接电机、不刷固件；Playwright不覆盖Tk或证明实物避让安全。

当前X42S现场复测：四角色ID/减速比/导程/方向、反馈单位与目标区分、一次双侧基准、
四种动作与连续换步、单/双升降点动、点动未结束禁止换绑、停止/急停/断连、人工回位后
核验恢复、轨道租约到期。若同坐标连续性不能确认，按实际摆放重建一次双侧基准。
旧“直线→旋转→直线/Pico离线/30轴配置”仅属于显式历史维护回归，不是当前X42S使用步骤。

本轮按迁移风险拆出执行器运行层、单位换算、恢复核验和卡片绘制，保留StepperGUI继承
方法名及原串口所有权。`desktop_app.py`仍承担组合、旧维护、步态桥接及生命周期，并非
已完全细分；下一轮可分别抽出历史STEP/DIR和连接生命周期，不宜与避让控制公式重写混做。
