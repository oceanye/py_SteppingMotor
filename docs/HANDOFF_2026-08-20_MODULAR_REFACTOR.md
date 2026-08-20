# 交接：PC GUI 模块化重构（2026-08-20）

## 分支与范围

- 分支：`codex/refactor-pc-gui-modules-2026-08-20`
- 基线：`412299e fix(gui): preserve position across axis mode switch`
- 范围：PC 端模块化、直线/旋转规范坐标、串口会话、运动并发安全，以及
  ESP32/RP2040 慢速步进和串口发送安全。
- 兼容边界：串口命令/回复格式、GPIO、PCB、HTTP 路径及旧 JSON 字段保持兼容；
  校准记录只增加可选 `_profile` 元数据，用于发现配置/校准文件的跨文件不一致。

该分支包含基线的直线/旋转切换修复；合并前请确认主线是否已单独合并 `412299e`。

## 主要变化

1. 根目录 `pc_gui.py` 保留为兼容启动器；`run.bat`、`start_gui.bat` 和
   `start_gui_console.bat` 的调用方式不变。
2. 30 轴拓扑、单位换算、step-based 轴状态、协议、串口会话、JSON 存储、UI 布局、
   Web adapter 和自动调参分别进入 `motor_control/`。
3. `AxisRuntime` 内部统一保存 `position_steps/min_steps/max_steps`。直线与旋转只是
   mm/° 显示映射；模式或 PPR/减速比/导程变化不会改变等效脉冲位置。
4. 串口改为一个 `SerialSession` reader。同步命令按精确 matcher 等待；STEP、硬急停、
   轨道超时、FOC fault 和 Pico 离线作为异步事件分流。请求超时会废弃当前会话，
   迟到回复不能完成下一条请求。
5. Web 轴配置从“投递 Tk 后立即返回”改为等待主线程实际应用，并返回生效后的快照。
6. GUI/Web MOVE 使用逐轴 reservation 与 motion generation。绝对定位在获得预约后按
   最新 steps 重算；STOP 先使未发送 MOVE 失效，并在固件确认前阻止同轴新 MOVE。
7. ESTOP、断线和跨串口重连使用控制/会话代数隔离旧 worker；未确认 ESTOP 会锁住
   普通控制，避免界面显示已停车但旧命令随后写出。
8. 四个旧 JSON 文件保持旧字段可读，写入改为临时文件 + `os.replace` 原子替换；
   校准 `_profile` 元数据不匹配时位置会被标记为不可信，而不是沿用错误单位。
9. PC、ESP32 和 RP2040 的 `delay_us` 上限统一为 `10,000,000`。远端轴 6..29 的下限
   仍为 `100 us`，覆盖默认旋转参数 `0.3°/s`。
10. ESP32 运行期协议回复及 STEP/FOC/TRACK/NODE 等异步事件先组装完整一行，再由
    共享 FreeRTOS mutex 输出，避免多个任务的 `Serial.print` 片段交错破坏帧。

详细边界和线程不变量见 `docs/ARCHITECTURE.md`。

## 离线验证结果

截至提交前：

- Python 3.10：98/98 unittest 通过。
- Python 3.12：98/98 unittest 通过。
- Python 3.13（32-bit）：98/98 unittest 通过。
- 三个 Python 版本的 `compileall`：通过。
- fake serial：同步回复与 STEP/HWESTOP/NODE 事件交错、超时、迟到回复、并发请求、
  ESTOP/逐轴 STOP 控制代数、发送前 guard、绝对定位重算和配置/MOVE 预约均通过。
- 本机 Tk 冒烟：30 轴界面成功创建并销毁；轴 29 使用 PPR=200、GR=36、lead=1 时，
  直线 1 mm = 7200 steps，直线→旋转→直线往返后分别显示 1 mm/360°/1 mm，
  7200 steps、软件限位和 trusted 状态保持不变。因三个系统 Python 均未安装 pyserial，
  此冒烟使用与单测相同的串口桩，未伪造硬件连接。
- PlatformIO：ESP32 `esp32s3`、`esp32s3_gear`、`esp32s3_gear_remote`、
  `esp32s3_gear_hwestop`、`esp32s3_gear_remote_hwestop`、`esp32s3_minimal` 六个环境
  全部构建成功；RP2040 `pico` 构建成功并生成 UF2。
- 两套 `native_test` 未执行：本机 PlatformIO host 环境缺少 `gcc/g++`，失败发生在编译器
  启动前，并非测试断言失败。板级交叉编译已通过，但没有烧录或电机实测。

运行离线回归：

```powershell
py -3.10 -m unittest discover -s tests
py -3.12 -m unittest discover -s tests
py -3.13-32 -m unittest discover -s tests
py -3.10 -m compileall -q pc_gui.py web_control.py motor_control tests
py -3.12 -m compileall -q pc_gui.py web_control.py motor_control tests
py -3.13-32 -m compileall -q pc_gui.py web_control.py motor_control tests

pio run -d esp32_stepper -e esp32s3 -e esp32s3_gear -e esp32s3_gear_remote `
  -e esp32s3_gear_hwestop -e esp32s3_gear_remote_hwestop -e esp32s3_minimal
pio run -d rp2040_stepper_node -e pico
```

## 远端合并前实机检查

请使用低速、可随时断电的条件依次验证：

1. 本地轴 0/5 和远端轴 6/29 均能 MOVE、STOP、DONE。
2. 同一静止轴：直线→旋转→直线，位置和软件限位保持等效脉冲；运动中切换必须拒绝。
3. PPR、减速比或导程改变后，显示坐标改变但规范 steps 不漂移。
4. Web `/api/stepper/config` 后立即 MOVE，必须使用新配置；GUI 已预约但尚未发送的 MOVE
   期间，模式/参数变更必须被拒绝。
5. ABORT 有计数时记入部分 steps 并把位置标记为不可信；旧三字段 ABORT 不虚构位移。
6. USB 断线、软件 ESTOP、硬件 ESTOP 均使旧排队命令失效。
7. MOVE worker 被延迟时发同轴 STOP：STOP 确认后不得再出现该旧 MOVE；STOP 等待确认
   期间同轴新 MOVE 必须被拒绝。
8. Pico 节点离线事件可见，远端轴快速命令不会低于 100 us；默认旋转 `0.3°/s`
   在本地轴和远端轴都能被接受并完成。
9. FOC/GEAR 状态轮询、调参保存、自动调参，以及轨道 D 租约超时保持原行为。
10. 手机页面显示30轴，轴29可配置，非法轴30被拒绝。

若任一运动安全项不符合，不要合并；保留串口原始日志和 `logs/gui_stderr.log` 回传复审。
