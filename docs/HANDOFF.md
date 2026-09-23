# py_SteppingMotor 真实环境交接说明

> 2026-09-23 两类模态、四种换位：按当前站位自动选择腿—高杆角度轨迹，预览与真实双轴 SYNC
> 共用执行折线；新模式间隙不足时禁止抬足。旧配置保留 legacy 行为，需主动切换并重新确认标定。
> 详见 [两模态腿—高杆避让、迁移与现场验收](HANDOFF_2026-09-23_TWO_MODE_LEG_AVOIDANCE.md)。

> 2026-09-19 实时数字孪生：按指令脉冲进度显示左右足、横梁及升降位移，区分当前估算与指令目标；
> 断连/超时灰显历史姿态。见 [实时数字孪生交接](HANDOFF_2026-09-19_GAIT_DIGITAL_TWIN.md)。

> 2026-09-18 步态 GUI 布局与参数精简：修复左侧输入框裁切，运行/标定/几何参考分页，
> 旋转角度独立于结构尺寸。见 [步态页布局与参数用途](HANDOFF_2026-09-18_GAIT_GUI_LAYOUT.md)。
> 注意：2026-09-18 的碰撞仅提示行为保留在 legacy 模式；2026-09-23 新增两模态模式的腿—高杆
> 检查不通过会拦截。请按所选模式对应的交接说明验收，不混用历史门槛。

> 2026-09-18 独立分支更新：自转—公转改为角度全过程一致的双轴原子SYNC执行，
> 补充邻接六边形、结构垂向包络、连续间隙下界、标定签名及四轴互锁。
> 详见 [角度联动交接与实机验收](HANDOFF_2026-09-18_ANGULAR_ORBIT_LINKAGE.md)。
> 不合并2209分支；缺少接触/载荷/编码器反馈时仍需人工确认，不宣称自主闭环步行。

交接日期：2026-08-03

> 2026-08-04 更新：项目已迁移到正式开发机（`D:\py_SteppingMotor`，Windows，Python 3.13.7）。
> `.venv`（pyserial 3.5）与 PlatformIO Core 6.1.19 已就绪，`py_compile` 与 `esp32s3_gear`
> / `esp32s3` / `esp32s3_minimal` 三套固件均已编译通过；详见 `docs/PROJECT_MEMORY.md`
> 第 7 节。下文 2026-08-03 正文作为历史交接基线保留。
>
> 2026-08-12 更新：已按“新增 24 路步进轴”实现普通 Pico（RP2040）RS485 扩展、
> ESP32 主站桥接以及 30 轴桌面/网页 UI。该路径与既有“24 路 GEAR 电机”提案不是
> 同一需求，详细拓扑、构建环境和远端验收项见
> `docs/HANDOFF_2026-08-12_RP2040_STEPPERS.md`。
>
> 2026-08-19 更新：GUI 轴参数手输、日志滚轮和 L/R 现场标定已有新结论；最新状态、
> 仍待复测的轴 L 36:1/行程矛盾以及复审修正见 `docs/HANDOFF_2026-08-19.md`。
>
> 2026-08-20 待合并分支更新：PC GUI 已进行渐进式模块化、step-based 轴状态和
> typed 串口会话重构；ESP32 整行串口 TX 与 ESP32/RP2040 慢速步进上限也已同步。
> 离线结果及远端实机合并清单见
> `docs/HANDOFF_2026-08-20_MODULAR_REFACTOR.md`。
>
> 2026-09-04 更新：手机网页已补齐 PPR/GR/导程、轴状态和高脉冲率确认，并修正
> RP2040 远端轴误套用 ESP32 周期开销补偿的问题。离线结果、功能边界和远端验收清单
> 见 `docs/HANDOFF_2026-09-04_WEB_PARITY.md`。
>
> 2026-09-06 待合并分支更新：已增加 `Mup1/Mr1/Mup2/Mr2` 到实际步进轴的安全绑定页，
> 桌面与网页共享只读位置/进度状态，且明确保持自动协调运动关闭。实现边界、离线结果和
> 现场验收清单见 `docs/HANDOFF_2026-09-06_LOGICAL_MOTOR_BINDING.md`。

本文是后续迁移到开发机、真实控制柜和实物机构时的执行基线。2026-08-20 分支已完成
Python 离线回归及 ESP32/RP2040 PlatformIO 交叉编译；**没有宣称烧录、KiCad DRC 或
电机实测通过**。

## 1. 当前交付状态

| 子系统 | 代码状态 | 当前硬件状态 | 默认行为 |
|---|---|---|---|
| DM422/DM442 步进轴 0–5 | 已落位 | 驱动/电机按现场逐轴接入 | 原生 GPIO PUL/DIR 开环 |
| DM422/DM442 远端步进轴 6–29 | ESP32 桥接、Pico 固件和 30 轴 UI 已落位 | 6 块普通 Pico、收发器及驱动尚需现场验收 | 仅 remote 构建启用；RS485 主从轮询 |
| RS485 总线 | AUX TX/RX/DE 与协议已落位 | 需外接 7 个 3.3 V 收发器并验证终端/偏置 | 默认构建禁用；与诊断 I²C 互斥 |
| AS5600 步进反馈 0–5 | 诊断接口和 PCB 预留已落位 | 尚未安装 | 默认禁用，不占用控制链 |
| TCA9548A | 固件地址/通道和模块占位已落位 | 尚未确认实物模块 | 仅用于隔离 6 个地址均为 `0x36` 的 AS5600 |
| PCF8575 | 地址和模块/逻辑端子预留已落位 | 尚未确认实物模块 | DNP；不接管 PUL/DIR |
| GEAR 0–1 | 已落位 | 既有 DRV8871 + AB 编码器方案 | 两路独立位置闭环 |
| 轨道 D | 已落位 | 第三块 DRV8871 | GPIO40/41 开环前后运行，租约超时停车 |
| PC GUI | 已落位 | 待开发机安装 Python/串口环境 | 唯一串口拥有者 |
| LAN 网页 | 已落位 | 随 GUI 启动 | 手机通过台式机间接控制，默认端口 8765 |
| PCB | 生成脚本、KiCad PCB 和 SVG 预览已落位 | 所有模块尺寸仍为 MOCK | 180 × 210 mm；禁止直接批量打样或生成生产 Gerber |

最重要的边界：当前 6 路 DM442 **仍然是开环**。AS5600 安装后应先启用只读诊断、完成机械与方向验证，再另外实现和启用位置外环。不能因为 I²C 检测到编码器就自动切换闭环。

DM442 + AS5600 的未来控制属于“步进位置外环/失步检测与补偿”，不是 BLDC 的 FOC。仓库里的 `DRIVE_MODE_FOC` 是历史 2208 BLDC + SimpleFOC mini + AS5600 方案，与默认 6 路 DM442 构建相互独立。

## 2. 正式入口与系统关系

```text
手机浏览器 -- LAN --> 台式机 pc_gui.py -- USB 串口 115200 --> ESP32-S3
PC GUI ---------------^                                |-- DM442 × 6
                                                        |-- 可选 RS485 --> Pico × 6 --> DM442 × 24
                                                        |-- GEAR DRV8871 × 2
                                                        `-- 轨道 D DRV8871 × 1
```

- PC 正式入口：项目根目录的 `pc_gui.py`，Windows 可用 `run.bat` 创建环境并启动，之后可用 `start_gui.bat`。
- Python 依赖：`requirements.txt`，当前第三方依赖只有 `pyserial==3.5`；Tkinter 由 Python 安装包提供。
- 网页服务：`web_control.py` + `web/index.html`，由 GUI 自动监听 `0.0.0.0:8765`。
- 固件：`esp32_stepper/`，默认 PlatformIO 环境为 `esp32s3_gear`。
- PCB：`pcb/generate_pcb.py` 是网络、引脚、模块占位和布线的单一数据源；不要只手改生成的 `.kicad_pcb`。
- PCB在线尺寸依据与可信度：`pcb/MOCK_MODULE_REFERENCES.md`。
- `esp32_main.py`、`pc_control.py` 是早期参考程序，不是正式入口。

当前仓库内旧 `.venv` 记录了其他机器/目录的 Python 路径，按用户要求本轮没有修复。
迁移时不要复制使用它，应在新机器上重新创建虚拟环境。

网页使用 GUI 明示的固定局域网地址和端口，不设登录或控制令牌。只允许在可信局域网/Windows 专用网络使用，不做公网端口转发；如需长期保持同一 IP，应在路由器设置 DHCP 地址保留。网页急停仍是软件急停，不能替代切断电机功率的硬件急停。

## 3. 默认 GEAR 固件引脚

| 对象 | GPIO |
|---|---|
| DM442 #0 PUL / DIR | 5 / 6 |
| DM442 #1 PUL / DIR | 7 / 15 |
| DM442 #2 PUL / DIR | 1 / 2 |
| DM442 #3 PUL / DIR | 4 / 8 |
| DM442 #4 PUL / DIR | 9 / 10 |
| DM442 #5 PUL / DIR | 38 / 39 |
| GEAR 0 实体 IN1 / IN2 | 11 / 16 |
| GEAR 0 实体编码器 A / B | 18 / 21 |
| GEAR 1 实体 IN1 / IN2 | 12 / 17 |
| GEAR 1 实体编码器 A / B | 13 / 14 |
| 轨道 D IN1 / IN2 | 40 / 41 |
| 可选 I²C SDA / SCL | 42 / 47 |
| 可选 RS485 DE | 48 |

`config.h` 中两路 GEAR 的逻辑 IN1/IN2、ENCA/ENCB 为匹配既有 PID 正方向而交换；PCB 和上表使用实体端子名。不要依据变量名擅自交换现场线束。

GPIO42/47 是复用资源：AS5600/TCA9548A I²C、AUX UART、RS485 三种用途不能同时启用。
默认 `esp32s3_gear` 两者都不启用；启用远端 24 轴时使用 remote 构建并分配给 RS485，
启用 AS5600 诊断时分配给 I²C。若现场要求两套功能同时工作，必须重选通信引脚并修订
载板与固件，不能仅修改编译开关。

## 4. AS5600、TCA9548A 与 PCF8575

### 4.1 模块关系

```text
ESP32 GPIO42/47 I²C
  |-- TCA9548A @ 0x70
  |     |-- CH0 --> AS5600 #0 @ 0x36
  |     |-- CH1 --> AS5600 #1 @ 0x36
  |     |-- CH2 --> AS5600 #2 @ 0x36
  |     |-- CH3 --> AS5600 #3 @ 0x36
  |     |-- CH4 --> AS5600 #4 @ 0x36
  |     `-- CH5 --> AS5600 #5 @ 0x36
  `-- PCF8575 @ 0x20（DNP，未来低速逻辑扩展）
```

- TCA9548A 解决 6 个 AS5600 固定地址冲突；每个传感器独占一个 mux 通道。
- PCF8575 不能解决 AS5600 地址冲突，也不能生成 DM442 的实时 PUL。
- 6 路 PUL 和当前 DIR 始终保留为 ESP32 原生 GPIO。
- PCF8575 的 P0–P5、P6–P11 分别仅标注为未来 DIR、ENA 逻辑候选，P12–P15 预留。当前 PCB 不把这些端口硬连到 DM442。
- PCF8575 是准双向口且上电默认高；连接 DM442 光耦输入前，必须根据实物输入电流、共阳/共阴接法和电平增加并验证成品开集电极/ULN 类缓冲模块。
- TCA、PCF 及 AS5600 模块必须工作在 3.3 V 兼容 I²C；禁止把模块自带 5 V 上拉直接送入 ESP32 GPIO。

### 4.2 当前诊断协议

安装模块并把 `esp32_stepper/src/config.h` 中 `STEPPER_ENCODER_DIAGNOSTICS_ENABLED` 显式改为 `1` 后，可查询：

```text
ENC,<axis>,S
```

响应格式：

```text
ENC,<axis>,S,<diag>,<mux>,<online>,<magnet>,<raw>,<single_deg>,<runtime_multi_deg>,<age_ms>,<errors>
```

该路径只读取状态和角度，不调用 MOVE/STOP，也不修改 PUL/DIR。TCA 或 AS5600 缺失必须保持非致命，不能阻止默认步进功能启动。

AS5600 是单圈绝对角度传感器。`runtime_multi_deg` 是固件通电期间通过跨零展开得到的 RAM 累计值，重启/掉电后丢失；多圈丝杠位置仍必须配合原点/限位建立机器坐标。

## 5. 串口与网页接口

主要串口命令：

```text
MODE
ESTOP
MOVE,<axis|*>,<steps>,<direction>,<delay_us>
STOP,<axis|*>
STDIAG,<axis>
FOC,<axis|*>,...
TRACK,D,FWD|REV,<duty>[,<lease_ms>]
TRACK,D,STOP
TRACK,D,S
ENC,<axis>,S
```

`MOVE` 的 `delay_us` 上限统一为 `10,000,000 us`。ESP32 直连轴接受
`1..10,000,000 us`；经 RS485 的 RP2040 轴因 50 us 高电平脉宽约束，接受
`100..10,000,000 us`。该上限覆盖默认旋转参数 `0.3°/s`。

ESP32 发往 PC 的运行期文本必须先组装成完整一行，再通过 `serial_tx` 的
共享 mutex 输出。协议回复及异步 `STEP`、`FOC`、`TRACK`、`NODE` 事件均遵循
此规则；新增固件路径不得直接分段调用 `Serial.print/println`。

`FOC,...` 这个历史协议名在 GEAR 构建中控制两路 DRV8871/N20 闭环轴，不代表六路 DM442 已使用 FOC。详细字段以根目录 `MD422_20K-2M.md` 为准。

网页 API：

- `GET /api/status`
- `POST /api/stepper/move`
- `POST /api/stepper/stop`
- `POST /api/stepper/config`
- `POST /api/motor`
- `POST /api/track`
- `POST /api/estop`

所有 `/api/*` 请求均为同源、无令牌接口。更详细的字段和部署边界见 `docs/WEB_CONTROL.md`。

## 6. 迁移到真实开发环境后的验证顺序

不要跳过顺序；前一关失败时不要给后一关上电。

### A. 软件构建

```powershell
cd <项目根目录>
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m compileall -q pc_gui.py web_control.py pc_control.py esp32_main.py motor_control tests
.\.venv\Scripts\python.exe -m unittest discover -s tests -v

cd esp32_stepper
pio run -e esp32s3_gear
pio run -e esp32s3_gear_hwestop
pio run -e esp32s3
pio test -e native_test
```

`esp32s3_gear` 默认不启用 GPIO35 急停输入，适用于尚未安装急停回路的机器。
只有在 GPIO35↔GND 常闭 NC 回路已接好并验证后，才烧录
`esp32s3_gear_hwestop`。该回路安全态为低，按下或断线时由内部上拉变高并触发。

历史 FOC 环境也应编译，以确认条件编译没有被默认 GEAR 改动破坏。根据现场端口修改 `platformio.ini` 的 upload/monitor port，不要把当前 COM4/COM9 当作固定事实。

### B. PCB 和模块尺寸

PCB 的固定实现路线是“完整成品模块载板”：默认用排针/排母，模块本身带邮票孔时才换成
对应模块的专用邮票孔 footprint；禁止为 ESP32、DRV8871、TCA9548A、PCF8575、AS5600
或 RS485 重新绘制芯片级电路。

1. 对 ESP32 DevKit、三块 DRV8871、TCA9548A、PCF8575、RS485 和所有端子逐项测量外形、排针中心距、针序、孔位、USB/天线方向。
2. 若采用邮票孔，还必须测量边缘焊盘节距、长度、板边偏移、模块底部禁布区和模块板厚；没有正式尺寸时继续使用排针 MOCK，不猜测通用邮票孔。
3. 更新 `pcb/generate_pcb.py` 的 `MODULE_GEOMETRY` 和对应 pin helper。
4. 重新运行 `python pcb/generate_pcb.py` 与 `python pcb/preview_pcb.py`。
5. 用 KiCad 8/9 打开工程，检查 courtyard/机械干涉、重新铺铜并执行 DRC。
6. 复核网络表、电源域、连接器方向和丝印，再生成 Gerber；当前 MOCK 版不得直接投板。

当前生成物统计为 212 个焊盘、61 个网络，仅用于设计交接和布局预览，不代表 DRC 或
可制造性已经通过。

### C. 无电机功率的逻辑验收

1. 只给 ESP32 USB 供电，确认无异常发热和复位循环。
2. 串口发送 `MODE`，应返回 `MODE,GEAR`。
3. 逐轴示波器/逻辑分析仪检查 PUL、DIR；先不接 DM442 功率端。
4. 检查 `ESTOP` 响应 `OK,ESTOP`，并确认所有输出停止、旧工作线程不能再次启动电机。
5. 如烧录 `esp32s3_gear_hwestop`：闭合 NC 回路后允许控制；按下急停或拆断任一根回路线应上报 `HWESTOP,TRIGGERED`，并在保持断开时拒绝 `MOVE`、`FOC,...,EN,1`、`FOC,...,A`、`FOC,...,H` 和 `TRACK,D,FWD|REV`；恢复回路后应上报 `HWESTOP,CLEARED`，不得自动恢复运动。
6. 启动 GUI 与手机网页，确认 GUI 显示的网址和端口可由手机访问、串口只有 GUI 占用、网页断开后轨道 D 租约会超时停车。

### D. 当前开环逐轴验收

1. 确认 DM442 24 V 不经过载板，DRV8871 的 `VM_MOTOR` 与电机额定电压匹配，各电源公共地。
2. 每次只接一轴、低速、小步数、机构脱离硬限位，核对方向。
3. 根据驱动器细分拨码和丝杠导程重新标定 `pc_gui.py` 的 `PULSES_PER_MM`；当前数值不能视为六轴实测结果。
4. 逐轴测试 STOP、ESTOP、串口拔出、GUI 关闭和轨道 D 租约超时。
5. 安装并验证硬件限位和切断功率的急停后，才允许扩大行程和速度。

### E. 编码器安装与只读诊断

1. 将径向磁铁与每根步进轴同轴安装，检查间隙、偏心、极性和机械固定。
2. 实测 TCA/AS5600 模块针序与 3.3 V 上拉；I²C 线尽量短，并与电机线分离。
3. 先保持 `STEPPER_ENCODER_CLOSED_LOOP_ENABLED=0`，仅把 `STEPPER_ENCODER_DIAGNOSTICS_ENABLED` 改为 `1`。
4. 查询 6 路 `ENC,<axis>,S`，验证 mux、在线、磁铁、原始角度、方向、跨零连续性和错误计数。
5. 同时运行开环小步运动，对比“命令脉冲推算位置”和编码器位置；建立每轴方向符号、传动比、零位和允许误差。

### F. 未来闭环落地

当前仓库故意用编译期保护阻止把尚未实现的闭环标志直接打开。未来实现至少要包含：

1. 原点/限位流程和掉电后的坐标恢复策略；
2. 每轴命令位置到编码器角度/丝杠位移的标定模型；
3. 失步检测门限、滤波、持续时间和故障锁存；
4. 速度受限的补偿策略，避免外环频繁追加脉冲或正负抖动；
5. 编码器掉线、磁铁异常、I²C 超时、TCA 掉线时立即退回安全停机，而不是盲目继续；
6. 与 STOP/ESTOP、软件限位、硬件限位和网页并发控制的统一状态机；
7. 单轴台架验证完成后再逐轴启用，禁止“检测到编码器即自动闭环”。

## 7. 尚未完成或必须现场确认

- 2026-08-03 初始交接轮次没有执行 Python、PlatformIO、native test、烧录或电机测试；
  后续离线构建结果分别记录在 08-12/08-13/08-19 handoff，但烧录和完整现场验收仍未完成。
- PCB 模块尺寸和针序均为 MOCK，尚未执行正式 KiCad DRC，也没有生产级 Gerber。
- AS5600、TCA9548A、PCF8575 尚未安装；六路编码器方向、安装公差和 I²C 线束未验证。
- 六路 DM442 的细分、丝杠导程、方向和真实 `PULSES_PER_MM` 未逐轴标定。
- 当前 ESTOP 是软件原子停止命令；硬件断电急停、限位链和驱动器 ENA/故障输入仍需落地。
- PCF8575 不在当前控制路径，外接缓冲模块和最终 DIR/ENA 跳线方案要等实物驱动输入参数确认。
- GPIO48 可能与部分 ESP32-S3 DevKit 的板载 RGB LED 冲突，启用 RS485 DE 前需核对开发板版本。
- 长距离 I²C 在电机环境中容易受电容和干扰影响；必要时降低时钟、改善线束，或改用靠近编码器的下游 MCU/差分链路。

## 8. 交接时优先阅读

1. `docs/HANDOFF_2026-09-06_LOGICAL_MOTOR_BINDING.md`：四逻辑电机绑定、位置语义与现场验收
2. `docs/HANDOFF_2026-09-04_WEB_PARITY.md`：网页配套、时序修正与远端验收项
3. `docs/HANDOFF_2026-08-24.md`：轴功能改名与轨道 D 实机结论
4. `docs/HANDOFF_2026-08-21.md`：硬件悬案收官、脉冲率护栏与现场状态
5. 本文 `docs/HANDOFF.md`
6. `MD422_20K-2M.md`：当前系统与串口协议
7. `docs/WEB_CONTROL.md`：局域网网页和 HTTP API
8. `pcb/README.md`：模块载板、电源边界与打样前检查
9. `docs/wiring_check.html`：现场接线核对页
10. `docs/PROJECT_MEMORY.md`：历史决策与调试背景
11. `docs/HANDOFF_2026-08-12_RP2040_STEPPERS.md`：24 路 RP2040 步进扩展与远端验收
