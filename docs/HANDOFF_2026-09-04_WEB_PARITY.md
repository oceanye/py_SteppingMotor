# py_SteppingMotor 交接（2026-09-04：网页轴配置补齐与速度护栏）

> 基线：Gitea `main` 的 `8d01884`。开发分支：
> `codex/web-parity-hardening-2026-09-04`。

## 1. 本次完成

- 手机网页补齐 30 轴的 PPR、GR、导程和直线/旋转配置；距离与速度单位随模式切换。
- 状态卡显示当前位置可信度、运动状态、软限位及当前已应用参数。
- 每秒状态刷新不再覆盖用户正在编辑的未应用配置；未应用时卡片高亮，并禁止使用新旧
  参数混合发出 MOVE。
- 网页速度候选与桌面 GUI 对齐为 24 档。超过 `1000 pps` 必须确认；服务端再次校验
  `confirm_high_rate`，避免客户端绕过。MOVE 响应报告请求/有效脉冲率及是否钳位。
- 统一本地与远端轴的速度换算入口：ESP32 本地轴保留短周期执行开销补偿；RP2040 PIO
  的 `delay_us` 按完整脉冲周期处理，避免远端轴被错误减去 ESP32 的 `200 us` 后跑得比
  设定更快。
- 扩充 Web HTTP/API、脉冲率边界及 Pico 远端轴周期的离线回归测试。

## 2. 有意保留的桌面端边界

网页未新增设置原点、校准当前位置、软限位编辑、绝对位置、连续步进或诊断入口。这些
操作依赖现场机械基准或更高风险的持续控制，继续只允许在桌面 GUI 完成。网页上的
STOP/ESTOP 仍是软件停止，不能替代硬件断电急停与限位链。

网页始终展示完整 30 轴拓扑；轴 6..29 只有在 ESP32 使用 `*_remote` 固件、RS485 接线
正确且对应普通 Pico（RP2040，无需 Pico W）节点在线时才能运行。

## 3. 离线验证与远端验收

本机只做静态/单元测试和交叉编译，不宣称浏览器真机、串口、烧录或电机运动通过。
本分支提交前结果：

- `python -B -m unittest discover -s tests -v`：131/131 通过；
- `python -m compileall -q pc_gui.py web_control.py motor_control tests`：通过；
- 网页内嵌 JavaScript 由 Node.js 解析：通过；
- ESP32 PlatformIO：`esp32s3`、`esp32s3_gear`、`esp32s3_gear_remote`、
  `esp32s3_gear_hwestop`、`esp32s3_gear_remote_hwestop`、`esp32s3_minimal`，6/6 通过；
- RP2040 PlatformIO `pico`：通过；
- RP2040 `native_test`：本机没有主机端 `gcc/g++`，因此测试程序未能编译启动；这是
  工具缺失，不是断言失败。协议解析器既有源码未在本次修改，仍建议远端有 GCC 的机器
  补跑 `pio test -d rp2040_stepper_node -e native_test`。

合并前由远端同事至少确认：

1. 手机修改模式/PPR/GR/导程后，状态卡回读一致；编辑中等待数秒不会被刷新覆盖。
2. 同一电机在直线与旋转模式各做一次低速短距离运动，方向、单位和脉冲数正确。
3. 超过 `1000 pps` 时取消确认不运动，确认后只发出一次 MOVE；钳位提示数值合理。
4. 本地轴 0 与远端轴 6 在相同 PPR、GR、导程和速度下，用示波器核对上升沿频率；
   Pico 周期不得再套用 ESP32 的 `200 us` 补偿。
5. 未烧 remote 固件或 Pico 离线时，远端 MOVE 必须明确失败；STOP/ESTOP 回执和实际
   停车均需验证。

详细使用和 API 契约见 `docs/WEB_CONTROL.md`。
