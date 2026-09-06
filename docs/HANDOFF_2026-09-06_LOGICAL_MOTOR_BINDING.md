# 四逻辑电机绑定与状态可视化交接

日期：2026-09-06

分支：`codex/coordinated-step-control-2026-09-06`

基线：`24a438c`（`gitea/main`）

## 1. 本分支完成内容

- 固定四个机构角色：`Mup1`、`Mr1`、`Mup2`、`Mr2`。
- 桌面新增“电机绑定与状态”页，可把角色绑定到全局步进轴 `0..29`，查看路由、软件
  位置、目标、协议进度、运行状态和位置可信度。
- `Mup1/Mup2` 强制要求直线模式，`Mr1/Mr2` 强制要求旋转模式；物理轴不能重复绑定。
- 首次启动默认全部未绑定。“填入建议映射”只填草稿，操作员仍需确认并保存。
- 保存使用 `.coordinated_bindings.json` schema v1 和原子替换；只更新映射，不自动改轴
  参数、不发送串口命令。旧/新涉及的轴只要在运行或已预约，就拒绝保存。
- 绑定后禁止把该物理轴切换为与角色不兼容的模式，必须先解除绑定。
- 网页增加四张只读逻辑电机卡；绑定写入和自动协调启动均没有 HTTP 路由。
- ESP32 本地步进进度上报调整为约每 250 ms 一次，并继续使用既有
  `STEP,<axis>,P,<done>,<total>` 协议；最终仍由 `DONE/ABORT` 结算。

建议映射仅供现场填入确认：

| 角色 | 用途 | 必须模式 | 建议步进轴 |
|---|---|---|---:|
| Mup1 | 左侧升降 | linear | 0 |
| Mr1 | 左侧旋转 | rotary | 2 |
| Mup2 | 右侧升降 | linear | 1 |
| Mr2 | 右侧旋转 | rotary | 3 |

## 2. 位置与安全边界

所有位置均为 PC 端按已确认脉冲累计得到的软件坐标：

```text
position_source = host_pulse_accounting
measured = false
```

运动中只在收到真实 `STEP,P` 帧后显示投影位置，不按时间模拟。`DONE` 只结算一次；
`ABORT` 按固件回报的已执行比例结算，并把位置标记为不可信。断线、超时或急停同样不能
被显示成可靠实测位置。

本分支没有实现 A→C、B→A、S0–S7 或四轴同步轨迹，状态接口固定返回
`automation_ready=false`。接触检测、支撑受力、碰撞包络、机械方向/零位、硬限位和真正
位置传感尚未现场完成，因此负责人不得仅凭四个状态卡启用自动换位。

## 3. 主要代码入口

- `motor_control/coordinated_control.py`：领域模型、严格绑定校验、位置投影和状态快照。
- `motor_control/ui/coordinated_tab.py`：桌面配置与可视化。
- `motor_control/desktop_app.py`：加载/保存、模式保护、协议进度与终态结算。
- `motor_control/web_adapter.py`、`web/index.html`：共享快照和网页只读状态卡。
- `esp32_stepper/src/stepper.cpp`：ESP32 本地轴进度上报节拍。

完整设计和后续阶段见 `docs/IMPLEMENTATION_PLAN_2026-09-06_MOTOR_BINDING.md`。

## 4. 本机离线验证

已通过：

- Python `py_compile`：新增和受影响的核心模块。
- Python `unittest discover -s tests -v`：154 项全部通过。
- `web/index.html` 内联 JavaScript 语法解析：Node.js 24.11.1。
- PlatformIO `esp32s3_gear` 构建。
- PlatformIO `esp32s3_gear_remote` 构建。
- PlatformIO `esp32s3`（FOC）、`esp32s3_gear_hwestop` 和
  `esp32s3_gear_remote_hwestop` 条件编译。
- PlatformIO `rp2040_stepper_node` 的 `pico` 构建。

当前机器没有系统 `gcc/g++`，PlatformIO `native_test` 在编译器启动前失败；不是用例断言
失败。项目 `.venv` 指向另一台机器缺失的 Python，PlatformIO Python 又不含 Tk，因此本机
不能做真实 Tk 窗口点测。用户已明确本机环境不完整，以下内容交由远端完成。

RP2040 节点目标仍是普通 Raspberry Pi Pico（`board = pico`，非 Pico W）；通信使用有线
半双工 RS485，不依赖 Wi-Fi。

## 5. 远端负责人合并前检查

1. 在完整 Python/Tk 环境执行 `python -m unittest discover -s tests -v`，再实际启动
   `pc_gui.py`，确认新页布局、缩放、草稿校验和页签跳转。
2. 首次无 `.coordinated_bindings.json` 时确认四角色均为未绑定；建议映射必须经显式保存
   才生效，重启后应恢复同一 revision。
3. 配置轴 0/1 为直线、轴 2/3 为旋转，再保存建议映射；重复轴、错误模式、非法参数、
   运动中/预约中保存均应被拒绝，且不得出现串口 MOVE。
4. 绑定后尝试把 Mup 轴改旋转、Mr 轴改直线，桌面和 Web 均应拒绝；解除绑定后才允许。
5. 每次只给一轴低速、小行程：观察 `STARTING → MOVING → IDLE`、投影位置、目标和进度；
   `DONE` 后位置只增加一次。模拟 `ABORT`、STOP、断线、ESTOP，确认可信度和状态降级。
6. 同时打开桌面和网页，核对角色顺序、物理轴、单位、位置、目标、进度一致；浏览器不得
   存在保存绑定或自动 S0–S7 的请求入口。
7. 烧录 `esp32s3_gear_remote` 和普通 Pico 固件后，验证远端角色初始为 `NODE_UNKNOWN`、
   掉线事件后为 `NODE_OFFLINE`；当前没有正向在线心跳缓存，禁止显示“硬件就绪”。普通
   `esp32s3_gear` 也不应被误认为具备轴 6..29。
8. 用逻辑分析仪确认 ESP32 约 250 ms 的进度帧不会改变 PUL/DIR 时序，且结束时只产生
   合法的 `DONE/ABORT` 终态帧。

通过以上现场检查后再由负责人合并；本分支不直接合入 `main`。
