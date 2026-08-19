# py_SteppingMotor 离线代码审查交接（2026-08-12）

## 1. 本轮范围

本轮以 `gitea/main` 为基线，只做代码、文档与离线构建验证。本机没有接入 ESP32、
急停按钮、驱动器或电机，未执行烧录、上电或机械运动。

## 2. 审查结论与已完成修改

### GPIO35 常闭急停逻辑

原实现将“常闭 NC 接点接 GND”与“低电平触发”同时使用。两者不能同时成立：NC 接点
在正常状态导通到 GND，GPIO 为低；按下或断线后接点断开，GPIO 由内部上拉变高。

现已改为：

- NC + `INPUT_PULLUP` + 高电平触发，断线与按下均进入停止态。
- 默认 `esp32s3_gear` 不启用 GPIO35 输入，避免未安装 NC 回路的设备在升级后持续触发。
- 新增 `esp32s3_gear_hwestop` 构建环境，仅用于已安装 GPIO35↔GND NC 回路的设备。
- 急停回路断开期间每次安全轮询都重申停止，不再只停止一次。
- 急停激活时拒绝所有可能启动运动的命令：`MOVE`、`STDIAG`、`FOC,...,EN,1`、
  `FOC,...,A`、`FOC,...,H` 和 `TRACK,D,FWD|REV`。停止、状态查询和失能命令仍可用。
- 触发时上报 `HWESTOP,TRIGGERED`；回路恢复时上报 `HWESTOP,CLEARED`。恢复不会自动重启运动。
- GUI 已增加 `HWESTOP,CLEARED` 的状态和日志处理。

### 文档一致性

`docs/HANDOFF.md`、`docs/HANDOFF_2026-08-06.md`、`docs/PROJECT_MEMORY.md`、
`MD422_20K-2M.md` 和 `docs/wiring_check.html` 已统一为相同的 NC 失效安全接线与验证说明。

## 3. 本机离线验证结果

- `py_compile pc_gui.py web_control.py pc_control.py esp32_main.py`：通过。
- `platformio run -e esp32s3_gear`：通过，RAM 6.0%，Flash 24.2%。
- `platformio run -e esp32s3_gear_hwestop`：通过，RAM 6.0%，Flash 24.2%。
- `platformio run -e esp32s3`：通过，RAM 6.4%，Flash 25.7%。
- `platformio test -e native_test`：未执行成功；本机没有 host `gcc/g++`。测试已增加 NC 高电平
  触发和 active-low 兼容用例，需远端开发机执行。

## 4. 远端同事合并前必须验证

1. 在开发机运行：

   ```powershell
   .\.venv\Scripts\python.exe -m platformio test -d esp32_stepper -e native_test
   .\.venv\Scripts\python.exe -m platformio run -d esp32_stepper -e esp32s3_gear
   .\.venv\Scripts\python.exe -m platformio run -d esp32_stepper -e esp32s3_gear_hwestop
   .\.venv\Scripts\python.exe -m platformio run -d esp32_stepper -e esp32s3
   ```

2. 未安装 GPIO35 NC 回路的机器只能烧录 `esp32s3_gear`。
3. 安装 NC 回路后，先只用 ESP32 USB 供电，再烧录 `esp32s3_gear_hwestop`；不要在首次
   故障注入时给电机功率级上电。
4. NC 回路闭合：启动日志应显示 `active-high(NC+pullup, open=fault)`，且 `MODE` 正常。
5. 按下按钮以及单独拆断任一根回路线：均应只上报一次 `HWESTOP,TRIGGERED`，所有输出停止。
6. 保持回路断开，逐项发送上述运动命令：应返回 `ERR:hardware estop active`，不得有输出。
7. 恢复 NC 回路：应上报一次 `HWESTOP,CLEARED`，所有轴仍保持停止/失能；仅在人工确认
   现场安全后再发新命令。
8. 启动 GUI，复核触发/恢复事件在 GUI 中显示，并重做原有 MODE、MOVE、ESTOP、轨道租约回归。

## 5. 本轮不实现的 handoff 项

- R/L 信号线恢复、R 轴 21.5 减速比、六轴开环标定、GEAR PID、限位、PCB 实测尺寸、
  AS5600 诊断/闭环等均依赖实物，保留给远端现场验证。
- 24 路 **GEAR** 扩展仍处于架构决策阶段；不要与随后确认的 24 路 **步进轴**需求混淆。
  步进轴 RP2040 路径现已单独实现，见 `docs/HANDOFF_2026-08-12_RP2040_STEPPERS.md`；
  两种扩展不能互相替代。
