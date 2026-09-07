# PC 控制端架构

`pc_gui.py` 是兼容启动入口。PC 是 USB 串口的唯一主站；Tk 界面和局域网 HTTP
服务共同调用桌面控制层，不允许各自直接读取串口。

```text
Tk views ─────────┐
                  ├── desktop controller ── SerialSession ── ESP32-S3
LAN Web API ──────┘          │                       │
                      AxisProfile/Runtime      RS485 Pico nodes
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

## 不变量

1. 位置和软件限位内部以脉冲坐标保存。`linear`/`rotary` 只改变显示和命令单位；
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
10. `Mup1/Mup2` 只能绑定 `linear` 步进轴，`Mr1/Mr2` 只能绑定 `rotary` 步进轴；同一物理
    轴不得重复绑定。绑定保存只改映射，不自动改轴参数、不下发串口命令，绑定轴运动中
    不允许保存或切换到不兼容模式。
11. 逻辑电机位置来自 `host_pulse_accounting`。运动中只可按真实 `STEP,P` 帧投影，
    `DONE/ABORT` 才结算规范位置；该值不是传感器实测值。当前自动协调运动必须保持关闭。

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

本机可运行纯 Python `unittest`、`compileall` 和 fake-serial 测试。最终合并前，远端现场
还必须验证同一轴直线→旋转→直线、Web 配置后立即 MOVE、运动中切换拒绝、DONE/ABORT、
断线、ESTOP、Pico 节点离线和轨道租约到期。
