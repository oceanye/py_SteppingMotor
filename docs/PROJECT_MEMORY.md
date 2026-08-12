# py_SteppingMotor 项目记忆

> 整理日期：2026-07-14；当前形态更新：2026-08-04
> 用途：供后续开发者或 AI 接手维护时快速恢复上下文。
> 信息来源：`.claude/history.jsonl` 中本项目 285 条历史记录、Git 历史、当前代码、`docs/superpowers` 设计/计划、GUI 日志和当前调参文件。

## 1. 信息可信度顺序

发生冲突时按以下顺序判断当前事实：

1. 当前硬件实测和最新 GUI/raw 日志
2. 当前工作区代码与 `platformio.ini`
3. `MD422_20K-2M.md` 与 `docs/wiring_check.html`
4. Git 提交历史
5. 本文件
6. `docs/superpowers` 历史设计和实施计划

Claude Code 没有为本项目生成独立的 `memory/MEMORY.md`。相关上下文保存在全局
`.claude/history.jsonl`；原完整会话正文目前未在 `.claude/projects` 或 `.claude/sessions`
中找到，因此历史结论必须与代码、Git 和日志交叉验证。

## 2. 项目目标与当前形态

系统使用 PC 端 Tkinter GUI，通过 115200 波特率串口控制 ESP32-S3。GUI 同时作为
局域网 HTTP 服务器，手机网页只通过 GUI 间接控制串口。当前默认 GEAR 构建包含：

- 6 路 DM422/DM442 步进轴，协议轴号 0–5；
- 2 路 DRV8871 + N20 + AB 编码器位置闭环，协议轴号 0–1；
- 1 路独立轨道 D DRV8871 开环电机，GPIO40/41，租约超时自动停止。

闭环电机仍可在以下两类构建方案之间选择：

- **FOC 模式**：2208 BLDC + SimpleFOC mini + AS5600。
- **GEAR 模式**：12V N20 1000:1 有刷减速电机 + DRV8871 + AB 霍尔编码器。

两种模式通过 PlatformIO 编译环境切换，FOC 代码保留，可随时重新构建：

| 环境 | 编译宏 | 当前端口 | 内容 |
|---|---|---|---|
| `esp32s3` | `DRIVE_MODE_FOC` | COM4 | 双 DM422 步进 + 双 SimpleFOC |
| `esp32s3_gear` | `DRIVE_MODE_GEAR` | COM9 | 6 路步进 + 双闭环 DRV8871/N20 + 轨道 D |
| `esp32s3_minimal` | `MINIMAL_TEST_BUILD` | COM9 | ESP32-S3 健康度/复位循环排查 |

`esp32s3_gear` 是当前默认环境。GUI 可用 `MODE` 自动识别固件，也可强制选择 FOC
或 GEAR；未激活模式的标签页灰显。

## 3. 历史时间线

### 2026-03-06：最初原型

- 阅读 DM422 资料并确认需要 ESP32 控制。
- 用户选择 Arduino/C++ 固件，而非继续使用 MicroPython 作为正式实现。
- PC 端使用 Python GUI、venv 和 bat 启动脚本。
- 建立本地 Git。
- 早期脚本 `esp32_main.py`、`pc_control.py` 仅保留作参考，已不是正式入口。

对应 Claude 会话：`caa09447-a333-4d81-9cea-1ec45bae81d7`。

### 2026-04-14 至 2026-04-17：PlatformIO 与 DM422 联调

- 转为 VS Code + PlatformIO。
- 最终确认芯片是 ESP32-S3 DevKit，曾因错误按普通 ESP32 构建而出现
  `This chip is ESP32-S3, not ESP32`。
- 正式板串口最终使用 CH343 的 COM4。
- DM422 左轴接线稳定为 PUL+ GPIO5、DIR+ GPIO6，PUL-/DIR- 接信号地。
- “串口返回 OK 但电机完全不动”的关键原因之一是 ESP32 GND 没有与 DM422/
  24V 电源负极共地。
- DM422 有供电、绿灯亮且电机有锁力，并不代表 PUL 光耦信号回路已经成立。
- 后续实测 `MOVE,400,...` 对应约 8 cm，最终 GUI 标定采用 `50 pulse/mm`。
- 取消上电自动 DIAG，避免烧录后约 20 秒才出现运动以及撞限位。

对应 Claude 会话：

- `f2fdd428-70d9-452a-b1a1-10c3da1d3bc8`
- `c1e7c7e8-c835-454c-900b-3fb006e0a76c`
- `2c264539-fba4-4ae9-afd5-cd2edcf695a0`

### 2026-04-17 至 2026-04-27：FOC 与双轴演进

- 设计并实现 2208 BLDC + AS5600 + SimpleFOC mini。
- 从单轴扩展为左右双轴；两个 AS5600 地址相同，因此使用两条 I2C 总线。
- 步进改为 FreeRTOS 异步任务，协议返回 `ACK,<axis>`，完成后异步发送
  `STEP,<axis>,DONE`。
- GUI 增加左右步进、左右 FOC、状态轮询、曲线、自动 PID、文件日志和 raw 串口日志。
- FOC 调参通过 `.foc_tune.json` 按轴保存，当前值为：

| 轴 | V | PA | VP | PP |
|---|---:|---:|---:|---:|
| L / 0 | 11.83 | 15.0 | 0.15 | 7 |
| R / 1 | 10.0 | 20.0 | 0.15 | 7 |

- 电源实际演进为 24V PSU；GUI 在超过电机额定 12V 时显示警告，固件上限允许 24V。
  后续上电和调参仍必须以电机温升、电流和驱动板安全为准。
- 曾出现点击后延迟响应，历史判断包括串口共享、步进占用和状态轮询拥堵；现代码用
  reader 线程、响应队列和 `serial_lock` 缓解。

### 2026-05-13 至 2026-05-26：切换到 DRV8871/N20 GEAR

- 用户要求保留 FOC，并增加可切换的减速电机方案。
- 电机规格：12V 有刷 N20、1000:1、基础 7 PPR、AB 双向增量磁性霍尔编码器、
  开漏输出、最高响应频率 100 kHz。
- 最终选用每个电机一块 DRV8871；两个左右电机各自独立闭环。
- 初期先用 ESP32 内置上拉跑通，4.7 kΩ 外部上拉电阻随后到货。
- 曾有一块 ESP32-S3 在所有 GPIO 拔除、仅 USB 供电时仍异常发热，判断为板卡损坏并更换。
- 当前 GEAR 调试板使用 COM9。
- 当前左轴实际物理接线：DRV8871 IN1/IN2 为 GPIO11/16；编码器 C1/C2 为
  GPIO18/21。代码中为统一 PID 正方向交换了 IN1/IN2 和 A/B 的逻辑声明。
- 右轴接线见 `docs/wiring_check.html`；当前代码占用 GPIO12/17 和 GPIO13/14。
- 8 端子 DRV8871 模块实测采用 Power+/Power- 供电，Motor1/2 接电机，IN1/IN2
  接 GPIO；该具体模块的额外 VM/GND 端子保持悬空。不能把此结论泛化到其他版型。
- PCNT 正交解码曾因 B 通道计数方向配置错误导致 A/B 计数互相抵消，现已修正。
- GEAR GUI 增加 PWM、Kp/Ki/Kd、齿轮比、到位 watcher、曲线和自动 PID。
- 最新调参文件：

| 轴 | PWM | Kp | Ki | Kd | GR |
|---|---:|---:|---:|---:|---:|
| L / 0 | 100% | 28.657 | 0.0 | 0.05 | 1000 |
| R / 1 | 100% | 7.314 | 0.482 | 0.05 | 1000 |

- 2026-05-26 日志显示左轴可到约 90°、180°、270°。
- 2026-05-25 日志显示右轴目标 90°时停在约 103.2°，误差约 13°，仍需继续处理
  PID、死区、静摩擦、编码器比例或机械负载差异。

对应 Claude 会话：`280d77c5-2e12-4f99-af68-fe3d3bf71726`。

## 4. 当前协议

协议使用显式轴号。GEAR 步进为 0–5，闭环为 0–1，也支持 `*` 广播。

```text
MODE
ESTOP
MOVE,<axis|*>,<steps>,<dir>,<delay_us>
STOP,<axis|*>
STDIAG,<axis>
DIAG,<axis>[,LIVE]
FOC,<axis|*>,S
FOC,<axis|*>,EN,<0|1>
FOC,<axis|*>,A,<deg>
FOC,<axis|*>,H
FOC,<axis|*>,CLR
FOC,<axis|*>,V,<value>
FOC,<axis|*>,PA,<value>
TRACK,D,FWD|REV,<duty>[,<lease_ms>]
TRACK,D,STOP
TRACK,D,S
```

FOC 专用：`PP`、`VP`、`REALIGN`。  
GEAR 专用：`GR`、`PI`、`PD`，其中 `V` 表示 PWM duty cap 百分比而不是电压。

步进结束事件携带实际执行量：`STEP,<axis>,DONE|ABORT,<executed>,<requested>`。
轨道 D 的 duty 为 1–100%，租约为 100–5000 ms，省略时默认 1000 ms。

## 5. 接线与安全经验

- 24V 步进电源、12V GEAR 电源、ESP32、DM422、DRV8871 和编码器必须共地。
- 上电前用万用表检查电源正负、共地连通和电源对地阻值。
- N20 是 12V 电机，不得直接接 24V。
- 开漏编码器优先用每路 4.7 kΩ 拉到 3.3V；内置上拉只作为调试兜底。
- ESP32-S3 仅 USB 且拔除全部 GPIO 后仍发热，优先判定板卡损坏，不继续带电排查。
- 自动调参或首次闭环前确保机构有足够行程，并准备断开电机电源。
- 当前没有硬件限位开关，GUI 软件限位不能替代物理保护。

## 6. 已完成修复与下一步

2026-08-03 已完成：

- PCNT 改为 1 ms 增量读取/清零并使用任务私有 64 位软件累计；
- PID、齿轮比、PWM 上限、home 请求等跨核状态改为 atomic 或请求式同步；
- 协议改为严格数值、字段数和轴范围解析；
- 步进 STOP 状态机消除启动竞争，并回报实际执行步数；
- native 测试从 smoke 扩展为协议数字和字段边界测试；
- GUI 区分 6 个步进轴和 2 个闭环轴，并处理 ABORT 位置不可信状态；
- 增加轨道 D 和固定地址、无令牌的局域网网页控制；GUI 明示局域网地址与端口。
- 为六路 DM442 增加 TCA9548A + 6×AS5600 的只读诊断代码和模块级 PCB 预留；编码器
  尚未安装，因此诊断默认关闭、闭环受编译期保护，当前原生 PUL/DIR 开环不变。
- 增加 PCF8575 模块级 DNP 预留，仅作为未来 DIR/ENA 等低速逻辑扩展；PUL 永远保留
  在 ESP32 原生 GPIO，PCF 输出在实测并增加成品缓冲模块前不连接 DM 光耦输入。
- 增加 `docs/HANDOFF.md`，作为迁移到真实开发/硬件环境后的构建、接线、验收和未来
  闭环实现基线。

### 中优先级

- 继续解决右 GEAR 轴的稳态误差和两轴 PID 差异。
- 硬件限位仍待（空闲 GPIO 仅 35/36/37，不够 6 轴 ×2 限位，需 PCF8575 扩展或继续用软件行程范围）。GPIO35 急停在 2026-08-12 离线审查时更正为失效安全的常闭 NC 逻辑：安全态接地为低，按下或断线后上拉为高并触发；未安装回路时默认构建不启用，已安装回路时使用 `esp32s3_gear_hwestop`。此更正只完成离线验证，仍需现场按钮/断线测试。GEAR DRV8871 nFAULT 检测条件编译预留（`GEAR_NFAULT_ENABLED` 默认 0，待确认模块是否引出 nFAULT）。
- ~~安装可用的 `gcc/g++` 后恢复 `pio test -e native_test`。~~ 已完成（2026-08-04）：WinLibs MinGW-W64 gcc 16.1.0 + Unity 2.6.1，6 个协议边界测试全 PASSED。
- PCB 当前使用模块 mock 尺寸；拿到 ESP32-S3 DevKit、DRV8871 模块和端子实测尺寸后
  替换封装 courtyard 并复核孔距；TCA9548A、PCF8575、AS5600 的模块尺寸、针序、
  地址跳线和 3.3V 上拉也必须逐项实测。
- 超过 6 路步进时优先增加 Pico/RP2040 脉冲协处理模块；I²C GPIO 扩展只用于
  DIR/ENA/限位等低速信号。

## 7. 维护验证基线

2026-08-04 起已迁移到正式开发机（Windows，`D:\py_SteppingMotor`），环境、基线构建、
烧录与功能验收均已验证通过：

- 系统 Python 3.13.7；项目 `.venv` 可用，已装 `pyserial==3.5`。
- PlatformIO Core 6.1.19 已装进项目 `.venv`（不污染系统 Python）。
- `py_compile pc_gui.py web_control.py pc_control.py esp32_main.py` 全部通过。
- 固件三环境编译通过（espressif32 6.12.0 / Arduino-ESP32 3.20017.241212）：
  - `esp32s3_gear`（默认 GEAR）：SUCCESS，RAM 6.0%，Flash 24.2%。
  - `esp32s3`（FOC，含 SimpleFOC 2.3.3）：SUCCESS，RAM 6.4%，Flash 25.7%。
  - `esp32s3_minimal`：SUCCESS，RAM 5.8%，Flash 21.1%。
- 默认 GEAR 构建与历史 FOC 构建条件编译隔离完好，互不破坏。
- 已烧录 `esp32s3_gear` 到 ESP32-S3（COM9 / CH343），C 阶段无电机功率逻辑验收通过：`MODE`→`MODE,GEAR`、`MOVE`→`ACK`+`STEP,DONE,executed,requested`、`ESTOP`→`OK,ESTOP`、非法轴/格式→`ERR`。
- C5 网页验收通过：`pc_gui.py` 独占串口，LAN 网页 `0.0.0.0:8765` 在线，手机经 `http://<台式机IP>:8765/` 控制；网页 `move`→位置更新、`estop`→confirmed、轨道 D 租约到期→`TRACK,D,TIMEOUT` 自动停车 均验证。
- host gcc/g++（WinLibs MinGW-W64 UCRT POSIX 16.1.0，winget 安装）已就绪；`pio test -e native_test` 通过，6 个协议边界测试全部 PASSED。

注意事项：

- `.venv\Scripts\pio.exe` 这个 console shim 在本机间歇性“拒绝访问”；统一改用
  `python -m platformio ...` 调用，例如
  `.\.venv\Scripts\python.exe -m platformio run -e esp32s3_gear`。
- 实物 ESP32-S3 为 DevKitC-1-N8（8 MB QD）；`platformio.ini` 三环境的 `board_build.flash_size`/`board_upload.flash_size` 已从 4MB 修正为 8MB（分区表仍为 `default.csv`，app 区 1.25 MB；若以后需要 OTA/LittleFS/更大 app 区，再换分区表）。

每次修改公共固件或协议后的最小验证基线：

```powershell
.\.venv\Scripts\python.exe -m py_compile pc_gui.py web_control.py pc_control.py esp32_main.py
cd esp32_stepper
.\.venv\Scripts\python.exe -m platformio run -e esp32s3_gear
.\.venv\Scripts\python.exe -m platformio run -e esp32s3_gear_hwestop
.\.venv\Scripts\python.exe -m platformio run -e esp32s3
```

协议回归测试（host GCC 已装，已验证通过）：

```powershell
cd esp32_stepper
.\.venv\Scripts\python.exe -m platformio test -e native_test
```

烧录、通电、自动 PID、越限测试和故障注入属于硬件操作，不能仅凭“编译成功”视为验证通过。
