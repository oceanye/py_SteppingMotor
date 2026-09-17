# ESP32-S3 + 4×TMC2209 PCB 资料包交接说明

> 资料包：`handoff_pcb_2209.zip`  
> 生成日期：2026-09-08  
> 软件分支：`codex/tmc2209-driver-migration-2026-09-08`

## 1. 本次需要设计的板

设计一块 `ESP32-S3-DevKitC-1 + 4×MKS TMC2209 V2.0` 插接式四轴载板。

已冻结的外部接口：

- 4 块 MKS TMC2209 V2.0；
- 4 组 6P、3.81 mm 电机端子；
- 每组针序固定为 `1B / 1A / 2A / 2B / NC / NC`；
- 独立电机电源输入 `VM+ / GND`；
- STEP/DIR 从 PCB 直接连接 ESP32，不额外设置控制端子；
- ESP32 调试阶段由 USB 供电，VM 不得连接 ESP32 的 5V/3V3；
- V1 使用 STEP/DIR 独立模式，默认 `MS1=MS2=0`、1/8 输入微步；
- UART、DIAG、INDEX 和 CLK 只预留测试点/DNP 焊盘，不作为 V1 功能。

完整电气、Layout、嘉立创制板和验收要求以
`01_requirements/PCB_REQUIREMENTS_2026-09-08_ESP32S3_4X_TMC2209_JLCPCB.md` 为唯一主文档。

## 2. 资料包目录

```text
handoff_pcb_2209/
├─ 00_README/
│  └─ HANDOFF_PCB_2209_PACKAGE.md
├─ 01_requirements/
│  └─ PCB_REQUIREMENTS_2026-09-08_ESP32S3_4X_TMC2209_JLCPCB.md
├─ 02_mks_tmc2209_reference/
│  ├─ README.md
│  ├─ TMC2209_datasheet.pdf
│  └─ MKS_TMC2209_V2.0/
│     ├─ MKS TMC2209 V2.0_001 SCH.pdf
│     ├─ MKS TMC2209 V2.0_001 TOP.pdf
│     ├─ MKS TMC2209 V2.0_001 BOTTOM.pdf
│     ├─ MKS TMC2209 V2.0_001 Layout.png
│     └─ MKS TMC2209 V2.0_001 SIZE_PIN.png
├─ 03_firmware_pin_reference/
│  ├─ platformio.ini
│  ├─ config.h
│  ├─ stepper.cpp
│  ├─ topology.py
│  └─ driver_profile.py
├─ 04_migration_reference/
│  └─ HANDOFF_2026-09-08_TMC2209_MIGRATION.md
└─ 05_existing_6axis_mock_DO_NOT_FABRICATE/
   ├─ README.md
   ├─ generate_pcb.py
   ├─ preview.svg
   ├─ stepper_carrier.kicad_pcb
   └─ stepper_carrier.kicad_pro
```

## 3. 四轴 GPIO 快速表

| 轴 | 功能 | STEP | DIR |
|---|---|---:|---:|
| M0 / axis 0 | 左侧直线 | GPIO5 | GPIO6 |
| M1 / axis 1 | 右侧直线 | GPIO7 | GPIO15 |
| M2 / axis 2 | 左侧旋转 | GPIO1 | GPIO2 |
| M3 / axis 3 | 右侧旋转 | GPIO4 | GPIO8 |

代码中 `PUL` 是旧命名，PCB 丝印和网络名统一使用 `STEP`。

## 4. 重要警告

1. `05_existing_6axis_mock_DO_NOT_FABRICATE/` 是六轴旧版 MOCK 参考设计，只能参考
   ESP32 引脚映射和绘图结构，**不得直接提交嘉立创生产**。
2. 新四轴板必须根据用户手中 ESP32、MKS TMC2209 V2.0 和 6P/3.81 mm 端子实物重新
   建封装并复测尺寸。
3. MKS 模块反插、VM 极性反接、带电插拔电机都可能损坏驱动器。
4. 每块 TMC 的 VM/GND 附近至少放 100 µF 低 ESR 储能；主电源入口还需保险、反接
   保护和总线电容。
5. EN 跳帽不能替代物理急停；整机必须能从电源侧切断 VM。
6. VREF/驱动电流必须等收到四台电机额定相电流和模块实物参数后再定型。

## 5. PCB 工程师需要补齐的交付物

- 四轴新板原理图和 PCB 源文件；
- 原理图 PDF、Gerber、钻孔文件；
- BOM、CPL、DNP 表；
- 正反面装配图、3D 预览；
- ERC/DRC 和 Gerber 回读报告；
- 大电流铜皮、端子额定电流和温升计算；
- 首板电源极性、插装方向和网络连通性检查记录。

## 6. 投板前仍需用户确认

- 6P/3.81 mm 端子具体型号、孔径、进线方向和额定电流；
- 四台步进电机额定相电流及线圈线序；
- 电机电源标称电压和最大电流；
- PCB 外形、安装孔位置和出线方向；
- 是否首版安装逐轴保险、风扇接口和急停接口。

