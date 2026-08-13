# 24 路 RP2040 步进扩展交接（2026-08-12）

## 1. 结论与范围

本轮按“在既有 6 路 DM422/DM442 步进轴之外新增 24 路步进轴”完成代码。采用 6 块
普通 Raspberry Pi Pico（RP2040、非 Pico W），每节点控制 4 轴，通过半双工 RS485
接入 ESP32-S3。系统共 30 个步进轴：

- 全局轴 0–5：ESP32-S3 本地原生 GPIO；
- 全局轴 6–29：Pico 节点 1–6，每节点本地轴 0–3；
- 全局轴映射：`6 + (node_id - 1) * 4 + local_axis`。

本机只完成代码与离线构建。尚未烧录 Pico/ESP32，未连接 RS485 收发器、DM422/DM442
或电机，也未验证并发脉冲、总线抗干扰、急停功率链及机械运动。远端同事必须完成第 5 节
后再合并到主分支。

## 2. 已实现内容

- `rp2040_stepper_node/`：普通 Pico 四轴节点；4 个 PIO 状态机产生 PUL，DIR 使用 GPIO；
  最小周期 100 us，高电平 50 us；节点 ID 为编译期 1–6。
- `esp32_stepper/`：可选 RS485 主站，使用严格地址/轮询协议；250 ms 心跳、1000 ms
  节点超时、广播 ESTOP；本地轴和远端轴统一使用 0–29 编号。
- `pc_gui.py`、`web_control.py`、`web/index.html`：30 轴 UI 和 API，按 ESP32 本地轴及
  Pico 节点分组；API 接受 0–29，拒绝 30 及以上。
- 普通 6 轴固件不受影响。仅 `esp32s3_gear_remote` / `esp32s3_gear_remote_hwestop`
  启用 RP2040 总线。

RS485 采用主从模型：只有被寻址的节点应答，广播 `HEARTBEAT` / `ESTOP` 不应答，事件
通过主站逐节点 `POLL` 取回，避免多个节点同时驱动总线。

## 3. 接线和资源冲突

ESP32 载板 AUX 定义必须按以下方向接 3.3 V RS485 收发器：

| ESP32-S3 | 收发器 |
|---|---|
| GPIO42 / TX | DI |
| GPIO47 / RX | RO |
| GPIO48 / DE | DE 与 /RE（并接） |
| GND | GND |

每块 Pico：GP0/TX→DI、GP1/RX←RO、GP10→DE+/RE；GP2/4/6/8 为 4 路 PUL，
GP3/5/7/9 为对应 DIR。使用 MAX3485/SP3485 等 3.3 V 器件，不能把 5 V-only MAX485
的 RO 直接接入 ESP32/Pico GPIO。A/B 仅在总线两个物理端点各装一个 120 ohm 终端，
偏置电阻只设一处。

GPIO42/47 同时是可选 AS5600/TCA9548A 诊断 I2C，引脚已经占用，RS485 与该 I2C 路径
在代码中编译期互斥，现场也不得同时接入。GPIO48 还需核对具体 ESP32-S3 开发板是否与
板载 RGB LED 冲突。

DM422/DM442 的光耦输入电流和共阳/共阴接法必须按实物确认。Pico GPIO 不应被假定能
直接可靠驱动，应配置合适的晶体管、开集电极或差分线驱动接口。软件 ESTOP 不替代切断
电机功率的硬件急停链。

## 4. 构建和配置

```powershell
# ESP32 主站（不带/带 GPIO35 常闭硬件急停）
pio run -d esp32_stepper -e esp32s3_gear_remote
pio run -d esp32_stepper -e esp32s3_gear_remote_hwestop

# Pico 节点（platformio.ini 默认 NODE_ID=1）
pio run -d rp2040_stepper_node -e pico

# 主机协议测试；需要 host gcc/g++
pio test -d esp32_stepper -e native_test
pio test -d rp2040_stepper_node -e native_test
```

为 6 块 Pico 分别生成固件时，必须将 `rp2040_stepper_node/platformio.ini` 中
`-DNODE_ID=1` 改为唯一的 1–6，保存和标记各自产物，禁止重复 ID 上总线。

## 5. 远端合并前验收

1. 重跑第 4 节全部构建和 native test；运行 Python `py_compile` 与 Web API 边界测试。
2. 不接驱动器时先用逻辑分析仪验证每块 Pico 的 4 路并发 PUL/DIR：周期、50 us 高电平、
   精确步数、STOP/ESTOP 后停止，以及下一次 MOVE 从正确 PIO 入口启动。
3. 仅接两块 Pico 做总线测试，再逐步扩到 6 块；确认无重复 ID、无广播响应、无总线冲突，
   断开任意节点不应阻塞其余节点控制。
4. 中断主站心跳，确认所有节点在 1000 ms 内终止运动并拒绝新的 MOVE，恢复心跳后不得
   自动恢复旧运动。
5. 分别触发软件 ESTOP 与 GPIO35 常闭硬件急停，确认 ESP32 本地 6 轴和全部远端 24 轴
   都停止；急停恢复后保持停止，必须人工重新下发运动命令。
6. 逐轴接入经过电平/电流验证的 DM422/DM442 接口，低速、小步数、机构脱离限位验证方向，
   再标定细分、丝杠导程和 `PULSES_PER_MM`。
7. 在桌面 GUI 和手机网页核对 0、5、6、29 四个边界轴，确认状态、完成、停止、离线和错误
   事件映射到正确节点/轴，且轴 30 被拒绝。
8. 最后进行长线、全部节点和电机上电的干扰/温升/掉线试验；硬件限位与独立断电急停链
   验收完成前，不得进行大行程或高速运动。

## 6. 已知限制

- Pico 的 STOP 已执行步数是软件/PIO FIFO 状态估计，必须用逻辑分析仪核对误差边界。
- 当前总线是单主站轮询，不提供多主仲裁；不得接入第二个主动发送器。
- 远端轴仍为开环 PUL/DIR，不包含 AS5600 位置闭环或失步补偿。
- `docs/EXPANSION_PLAN_24_GEARS.md` 描述的是 24 路 GEAR 电机提案，并非本方案。
