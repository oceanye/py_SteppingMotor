# PCB 成品模块 MOCK 尺寸调研

调研日期：2026-08-04

目标是为模块载板提供有来源的布局参考尺寸，而不是把参考模块当成已经选定的生产物料。
同一芯片的不同网店模块，板框、针序、安装孔和上拉电压经常完全不同；因此这里分别记录
“来源尺寸”和“载板预留包络”。

## 参考结果

| 功能模块 | 在线参考型号 | 来源尺寸 | 当前 MOCK 使用 | 可信度与限制 |
|---|---|---:|---:|---|
| ESP32-S3 主控 | Espressif ESP32-S3-DevKitC-1 v1.0 | 25.40 × 62.74 mm | 25.40 × 62.74 mm | 高；官方机械图。仍需确认手中是 v1.0/v1.1 及USB、RGB版本 |
| DRV8871 | Adafruit 3190 | 24.4 × 20.4 mm | 用户模块暂保留 24 × 20 mm | 中；仅验证尺寸量级。Adafruit的排针/端子与用户现有模块不一定相同，不能套针序 |
| TCA9548A | Soldered TCA9548A breakout | 54 × 38 mm | 56 × 40 mm包络 | 中高；厂商提供板框和M3孔信息。排针坐标仍需按实际采购型号重画 |
| TCA9548A备选 | SparkFun Qwiic 8-channel mux v1.1 | 55.1 × 36.0 × 4.7 mm | 被56 × 40 mm包络覆盖 | 高；产品页明确尺寸，但它主要使用JST-SH/Qwiic，不能直接套当前排针假设 |
| TCA9548A小型备选 | Adafruit 2717 | 30.6 × 17.6 × 2.7 mm | 不用于当前大包络定针 | 高；适合以后缩板，但要重新布置两排通道针脚 |
| PCF8575 | Adafruit 5611 | 40.8 × 17.7 × 4.5 mm | 43 × 20 mm包络 | 高；产品页明确板框。当前P0-P15孔位仍是2.54 mm网格MOCK |
| AS5600轴端模块 | Seeed Grove 101020692 | 40 × 20 mm | 建议轴端42 × 22 mm包络 | 中；Seeed资料确认型号和磁铁要求，尺寸由经销资料交叉确认；实际可换更小模块 |
| RP2040协处理器 | Raspberry Pi Pico | 51 × 21 × 1 mm | 未来预留53 × 23 mm装配空间 | 高；官方数据手册明确板框、2.54 mm邮票孔/通孔及2.1 mm安装孔 |
| RS485 | Waveshare RS485 Board (3.3V)类别 | 当前尺寸图为图片，文本未给出可靠数值 | 暂保留45 × 20 mm | 低；正式采购前必须按具体SP3485/MAX3485完整模块重画 |

## 资料链接

- [Espressif ESP32-S3-DevKitC-1官方尺寸图](https://dl.espressif.com/dl/PCB_ESP32-S3-DevKitC-1_V1_20210312CB.pdf)
- [Adafruit DRV8871 3190产品/设计资料](https://learn.adafruit.com/adafruit-drv8871-brushed-dc-motor-driver-breakout/download)
- [Soldered TCA9548A硬件与尺寸](https://docs.soldered.com/tca9548a/hardware/)
- [SparkFun TCA9548A Qwiic模块产品尺寸](https://www.adafruit.com/product/4704)
- [Adafruit TCA9548A 2717产品尺寸](https://www.adafruit.com/product/2717)
- [Adafruit PCF8575 5611产品尺寸](https://www.adafruit.com/product/5611)
- [Seeed Grove AS5600模块资料](https://wiki.seeedstudio.com/Grove-12-bit-Magnetic-Rotary-Position-Sensor-AS5600/)
- [Raspberry Pi Pico官方数据手册](https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf)
- [Waveshare RS485 Board (3.3V)](https://www.waveshare.com/product/rs485-board-3.3v.htm)
- [嘉立创EDA公开库中的TCA9548A-Board条目](https://lceda.cn/hcj152190579/components)

嘉立创EDA公开库可以帮助确认“市场上确有该模块/封装条目”，但公开库条目没有提供足够
可审计的机械尺寸和针序，因此本轮没有直接把社区封装当成生产封装。最终选定采购链接后，
应下载该模块官方CAD或用游标卡尺复测，再锁定footprint。

## 机械落地规则

1. 排针模块至少预留模块板框外各1 mm装配间隙，并检查排母高度、螺钉端子进线和USB插拔空间。
2. 邮票孔模块只能使用对应型号的正式机械图；同时核对焊盘节距、半孔直径、模块板厚和底部禁布区。
3. AS5600的关键尺寸不是主载板连接器，而是传感器中心相对电机轴、磁铁和固定孔的位置；轴端小板应独立定版。
4. 参考包络可以做布局评审，不能用于下单。采购型号、实物复测、KiCad DRC和装配检查全部通过后才能生成Gerber。
