#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate the modular six-axis ESP32-S3 motor carrier (KiCad 8).

This file is the design's single source of truth.  The PCB is strictly a
carrier for complete ready-made modules: use pin headers/sockets by default,
or a module-specific castellated-pad footprint only when that module actually
has castellated edges.  It must not recreate ESP32, DRV8871, TCA9548A,
PCF8575, AS5600, DM422 or RS485 circuitry from individual ICs.

All dimensions that still depend on the user's physical modules are grouped in
MODULE_GEOMETRY below.  Replace those values (and only the associated footprint
helper when the pin order or attachment style differs) after measuring the real
modules.  Never invent a generic castellated footprint without the module's
edge-pad pitch, pad length, edge offset and outline.
"""

from __future__ import annotations

from pathlib import Path
import uuid


OUT_DIR = Path(__file__).resolve().parent
BOARD_W = 180.0
BOARD_H = 210.0
PITCH = 2.54

# ---------------------------------------------------------------------------
# Mechanical assumptions (MOCK until measured)
# ---------------------------------------------------------------------------
MODULE_GEOMETRY = {
    "esp32_s3_devkitc_1": {
        # Espressif official dimension drawing: 25.40 x 62.74 mm.
        "reference_model": "Espressif ESP32-S3-DevKitC-1 v1.0",
        "body_w": 25.40,
        "body_h": 62.74,
        "row_spacing": 22.86,
        "pins_per_row": 22,
        "pin_pitch": 2.54,
        "courtyard_margin": 1.0,
    },
    "drv8871_module": {
        # Size-only sanity reference: Adafruit 3190 is 24.4 x 20.4 mm, but its
        # connectors/pinout do NOT define the user's existing module footprint.
        "reference_model": "Adafruit 3190 size reference only",
        "body_w": 24.0,
        "body_h": 20.0,
        "header_pins": 4,
        "header_pitch": 2.54,
        "assumed_pin_order": ("VM", "GND", "IN2", "IN1"),
        "mount_holes": ((10.0, 1.0), (10.0, 15.0)),
        "mount_drill": 2.7,
        "courtyard_margin": 1.0,
    },
    "rs485_module": {
        # Conservative generic envelope.  The final full SP3485/MAX3485 module
        # must be selected before its header and terminal coordinates are fixed.
        "reference_model": "Waveshare RS485 Board (3.3V) class",
        "body_w": 45.0,
        "body_h": 20.0,
        "logic_header_pitch": 2.54,
        "bus_header_spacing": 40.0,
        "courtyard_margin": 1.0,
    },
    "tca9548a_module": {
        # Soldered's documented full module is 54 x 38 mm.  Keep 1 mm carrier
        # clearance on every side for a conservative replaceable-module MOCK.
        "reference_model": "Soldered TCA9548A breakout 54x38",
        "reference_body_w": 54.0,
        "reference_body_h": 38.0,
        "body_w": 56.0,
        "body_h": 40.0,
        "header_pitch": 2.54,
        "channel_group_pitch": 8.0,
        "courtyard_margin": 1.0,
        "assumed_host_order": ("3V3", "GND", "SDA", "SCL", "RST", "A0", "A1", "A2"),
        "assumed_channel_order": ("SDA", "SCL"),
    },
    "pcf8575_module": {
        # Adafruit 5611 is 40.8 x 17.7 mm.  The 43 x 20 mm outline is a
        # reference envelope, not a guaranteed clone-module footprint.
        "reference_model": "Adafruit 5611 PCF8575 breakout 40.8x17.7",
        "reference_body_w": 40.8,
        "reference_body_h": 17.7,
        "body_w": 43.0,
        "body_h": 20.0,
        "header_pitch": 2.54,
        "courtyard_margin": 1.0,
        "assumed_host_order": ("3V3", "GND", "SDA", "SCL", "INT", "A0", "A1", "A2"),
        "assumed_port_order": tuple(f"P{index}" for index in range(16)),
    },
    # Remote mechanical references only: neither module is mounted by the
    # current carrier generator.
    "as5600_remote_module": {
        "reference_model": "Seeed Grove AS5600 101020692",
        "reference_body_w": 40.0,
        "reference_body_h": 20.0,
        "reserved_envelope_w": 42.0,
        "reserved_envelope_h": 22.0,
    },
    "rp2040_pico_module": {
        "reference_model": "Raspberry Pi Pico",
        "reference_body_w": 51.0,
        "reference_body_h": 21.0,
        "board_thickness": 1.0,
        "main_pin_pitch": 2.54,
        "mount_drill": 2.1,
    },
    "terminal_5p08": {"pitch": 5.08, "drill": 1.3, "pad_dia": 2.6},
}

# Track and via rules.  VM_MOTOR is a separately supplied low-voltage rail.
SIG_W = 0.40
LOGIC_PWR_W = 0.80
MOTOR_PWR_W = 1.50
VIA_D = 1.20
VIA_DRILL = 0.60

# ESP32-S3-DevKitC-1 v1.1 2x22 header pin names, antenna end first.
# The GPIO mapping follows Espressif's official J1/J3 header table.
DEVKIT_LEFT = [
    "3V3", "3V3", "RST", "GPIO4", "GPIO5", "GPIO6", "GPIO7", "GPIO15",
    "GPIO16", "GPIO17", "GPIO18", "GPIO8", "GPIO3", "GPIO46", "GPIO9",
    "GPIO10", "GPIO11", "GPIO12", "GPIO13", "GPIO14", "5V_USB", "GND",
]
DEVKIT_RIGHT = [
    "GND", "GPIO43", "GPIO44", "GPIO1", "GPIO2", "GPIO42", "GPIO41",
    "GPIO40", "GPIO39", "GPIO38", "GPIO37", "GPIO36", "GPIO35", "GPIO0",
    "GPIO45", "GPIO48", "GPIO47", "GPIO21", "GPIO20", "GPIO19", "GND", "GND",
]

# Firmware mapping from esp32_stepper/src/config.h (DRIVE_MODE_GEAR).
STEPPER_PINS = ((5, 6), (7, 15), (1, 2), (4, 8), (9, 10), (38, 39))
GEAR_PINS = (
    {"IN1": 16, "IN2": 11, "ENCA": 21, "ENCB": 18},
    {"IN1": 17, "IN2": 12, "ENCA": 14, "ENCB": 13},
)
# The names above intentionally follow config.h's logical PID channels.  They
# are swapped relative to the existing physical wiring: logical IN1 drives the
# module's physical IN2, logical IN2 drives physical IN1; logical ENCA reads the
# physical B/C2 line and logical ENCB reads physical A/C1.  Keep that distinction
# explicit whenever assigning connector pads below.
TRACK_D_PINS = {"IN1": 40, "IN2": 41}

# Reserved only; firmware does not currently claim these pins.  UART signals
# use ESP32's GPIO matrix, so GPIO42/47 can be assigned TX/RX later.  GPIO48 is
# reserved as optional RS485 DE+/RE control (some DevKit revisions use it for RGB).
AUX_PINS = {"TX": 42, "RX": 47, "DE": 48}
I2C_MUX_PINS = {"SDA": 42, "SCL": 47}


def gpio_net(pin: int) -> str:
    return f"GPIO{pin}"


NETS = ["", "GND", "3V3", "5V_USB", "VM_MOTOR", "RS485_A", "RS485_B"]
NETS += [gpio_net(pin) for pair in STEPPER_PINS for pin in pair]
NETS += [gpio_net(v) for axis in GEAR_PINS for v in axis.values()]
NETS += [gpio_net(v) for v in TRACK_D_PINS.values()]
NETS += [gpio_net(v) for v in AUX_PINS.values()]
for channel in range(6):
    NETS += [f"AS{channel}_SDA", f"AS{channel}_SCL"]
NETS += [f"PCF_P{index}" for index in range(16)] + ["PCF_INT"]
# Stable order while removing duplicates.
NETS = list(dict.fromkeys(NETS))
N = {name: index for index, name in enumerate(NETS)}


_UID_SEQ = 0
_UID_NS = uuid.UUID("00000000-0000-0000-0000-000000000000")


def uid() -> str:
    global _UID_SEQ
    _UID_SEQ += 1
    return str(uuid.uuid5(_UID_NS, f"pcb-element-{_UID_SEQ}"))


def fnum(value: float) -> str:
    result = f"{value:.3f}".rstrip("0").rstrip(".")
    return result or "0"


class PCB:
    def __init__(self) -> None:
        self.items: list[str] = []

    def segment(self, net: str, layer: str, width: float, start, end) -> None:
        self.items.append(
            f'  (segment (start {fnum(start[0])} {fnum(start[1])}) '
            f'(end {fnum(end[0])} {fnum(end[1])}) (width {fnum(width)}) '
            f'(layer "{layer}") (net {N[net]}) (uuid "{uid()}"))'
        )

    def via(self, net: str, point) -> None:
        self.items.append(
            f'  (via (at {fnum(point[0])} {fnum(point[1])}) (size {VIA_D}) '
            f'(drill {VIA_DRILL}) (layers "F.Cu" "B.Cu") '
            f'(net {N[net]}) (uuid "{uid()}"))'
        )

    def text(self, value: str, point, size=1.0, rotation=0, layer="F.SilkS", justify="") -> None:
        justification = f" (justify {justify})" if justify else ""
        self.items.append(
            f'  (gr_text "{value}" (at {fnum(point[0])} {fnum(point[1])} {rotation}) '
            f'(layer "{layer}") (uuid "{uid()}") '
            f'(effects (font (size {size} {size}) (thickness {max(0.12, size * 0.15):.2f}))'
            f'{justification}))'
        )

    def rect(self, start, end, layer="F.SilkS", width=0.12) -> None:
        self.items.append(
            f'  (gr_rect (start {fnum(start[0])} {fnum(start[1])}) '
            f'(end {fnum(end[0])} {fnum(end[1])}) '
            f'(stroke (width {width}) (type solid)) (fill none) '
            f'(layer "{layer}") (uuid "{uid()}"))'
        )

    def footprint(self, name, reference, value, at, pads, outlines=()) -> None:
        lines = [
            f'  (footprint "{name}" (layer "F.Cu") (uuid "{uid()}")',
            f'    (at {fnum(at[0])} {fnum(at[1])})',
            '    (attr through_hole)',
            f'    (property "Reference" "{reference}" (at 0 -3.5) (layer "F.SilkS") '
            f'(uuid "{uid()}") (effects (font (size 1 1) (thickness 0.15))))',
            f'    (property "Value" "{value}" (at 0 3.5) (layer "F.Fab") '
            f'(uuid "{uid()}") (effects (font (size 1 1) (thickness 0.15))))',
        ]
        for start, end, layer, width in outlines:
            lines.append(
                f'    (fp_rect (start {fnum(start[0])} {fnum(start[1])}) '
                f'(end {fnum(end[0])} {fnum(end[1])}) '
                f'(stroke (width {width}) (type solid)) (fill none) '
                f'(layer "{layer}") (uuid "{uid()}"))'
            )
        # pad tuple: number, net, x, y, diameter, drill, shape
        for number, net, x, y, diameter, drill, shape in pads:
            if shape == "np":
                lines.append(
                    f'    (pad "" np_thru_hole circle (at {fnum(x)} {fnum(y)}) '
                    f'(size {diameter} {diameter}) (drill {drill}) '
                    f'(layers "*.Cu" "*.Mask") (uuid "{uid()}"))'
                )
            else:
                net_attr = f' (net {N[net]} "{net}")' if net else ""
                lines.append(
                    f'    (pad "{number}" thru_hole {shape} (at {fnum(x)} {fnum(y)}) '
                    f'(size {diameter} {diameter}) (drill {drill}) '
                    f'(layers "*.Cu" "*.Mask"){net_attr} (uuid "{uid()}"))'
                )
        lines.append("  )")
        self.items.append("\n".join(lines))


pcb = PCB()


def header_pad(number, net, x, y):
    return (str(number), net, x, y, 1.8, 1.0, "circle")


def terminal(reference, value, at, nets):
    cfg = MODULE_GEOMETRY["terminal_5p08"]
    pads = [
        (str(index + 1), net, index * cfg["pitch"], 0, cfg["pad_dia"], cfg["drill"], "circle")
        for index, net in enumerate(nets)
    ]
    width = (len(nets) - 1) * cfg["pitch"]
    outlines = (((-2.5, -3.8), (width + 2.5, 3.8), "F.SilkS", 0.15),)
    pcb.footprint(f"TerminalBlock_1x{len(nets)}_P5.08", reference, value, at, pads, outlines)


# ---------------------------------------------------------------------------
# Module footprints and connector placement
# ---------------------------------------------------------------------------
DK_X = 12.0
DK_Y = 48.0
DK_ROW = MODULE_GEOMETRY["esp32_s3_devkitc_1"]["row_spacing"]


def devkit_pad(side: str, number: int):
    return (DK_X + (0 if side == "L" else DK_ROW), DK_Y + (number - 1) * PITCH)


GPIO_PAD = {}
devkit_pads = []
for index in range(22):
    for side, names, x in (("L", DEVKIT_LEFT, 0.0), ("R", DEVKIT_RIGHT, DK_ROW)):
        name = names[index]
        net = name if name in N else ""
        devkit_pads.append(header_pad(f"{side}{index + 1}", net, x, index * PITCH))
        if name.startswith("GPIO"):
            GPIO_PAD[int(name[4:])] = devkit_pad(side, index + 1)

devkit_width = MODULE_GEOMETRY["esp32_s3_devkitc_1"]["body_w"]
devkit_height = MODULE_GEOMETRY["esp32_s3_devkitc_1"]["body_h"]
pcb.footprint(
    "ModuleSocket_ESP32-S3-DevKitC-1_2x22",
    "U1",
    "ESP32-S3 DevKitC-1 module (MOCK VERIFY)",
    (DK_X, DK_Y),
    devkit_pads,
    (
        ((-1.65, -1.3), (devkit_width - 1.65, devkit_height - 1.3), "F.SilkS", 0.15),
        ((-2.65, -2.3), (devkit_width - 0.65, devkit_height - 0.3), "F.CrtYd", 0.05),
    ),
)

# Six external DM422 drivers: signals only.  Their 24 V motor supply must use a
# separate fused distribution module/wiring block; no +24 V enters this PCB.
DM_CONNECTOR_X = [6.0, 35.0, 64.0, 93.0, 122.0, 151.0]
DM_CONNECTOR_Y = 10.0
DM_DEST = {}
for axis, x in enumerate(DM_CONNECTOR_X):
    pul, direction = STEPPER_PINS[axis]
    terminal(
        f"J_DM{axis}_SIG",
        f"DM422 #{axis} PUL+/PUL-/DIR+/DIR-",
        (x, DM_CONNECTOR_Y),
        [gpio_net(pul), "GND", gpio_net(direction), "GND"],
    )
    DM_DEST[(axis, "PUL")] = (x, DM_CONNECTOR_Y)
    DM_DEST[(axis, "DIR")] = (x + 2 * 5.08, DM_CONNECTOR_Y)
    pcb.text(f"DM{axis}  P+ P- D+ D-", (x + 7.62, 4.2), 0.75, justify="center")


DRV_AT = [(73.0, 57.0), (108.0, 57.0), (143.0, 57.0)]
DRV_REFS = ("U_DRV0", "U_DRV1", "U_DRVD")
DRV_LABELS = ("GEAR 0 DRV8871", "GEAR 1 DRV8871", "TRACK D DRV8871")
DRV_NETS = (
    # Tuple order is the module's physical (IN2, IN1).
    (gpio_net(GEAR_PINS[0]["IN1"]), gpio_net(GEAR_PINS[0]["IN2"])),
    (gpio_net(GEAR_PINS[1]["IN1"]), gpio_net(GEAR_PINS[1]["IN2"])),
    (gpio_net(TRACK_D_PINS["IN2"]), gpio_net(TRACK_D_PINS["IN1"])),
)
DRV_DEST = {}
drv_cfg = MODULE_GEOMETRY["drv8871_module"]
for index, (at, ref, label, (in2, in1)) in enumerate(zip(DRV_AT, DRV_REFS, DRV_LABELS, DRV_NETS)):
    pads = [
        header_pad(1, "VM_MOTOR", 0, 0),
        header_pad(2, "GND", 0, PITCH),
        header_pad(3, in2, 0, 2 * PITCH),
        header_pad(4, in1, 0, 3 * PITCH),
    ]
    for hole_x, hole_y in drv_cfg["mount_holes"]:
        pads.append(("", "", hole_x, hole_y, 5.0, drv_cfg["mount_drill"], "np"))
    margin = drv_cfg["courtyard_margin"]
    pcb.footprint(
        "ModuleSocket_DRV8871_1x4",
        ref,
        f"{label} module (MOCK VERIFY PIN ORDER)",
        at,
        pads,
        (
            ((-2.0, -2.0), (drv_cfg["body_w"] - 2.0, drv_cfg["body_h"] - 2.0), "F.SilkS", 0.15),
            ((-2.0 - margin, -2.0 - margin),
             (drv_cfg["body_w"] - 2.0 + margin, drv_cfg["body_h"] - 2.0 + margin),
             "F.CrtYd", 0.05),
        ),
    )
    DRV_DEST[(index, "IN2")] = (at[0], at[1] + 2 * PITCH)
    DRV_DEST[(index, "IN1")] = (at[0], at[1] + 3 * PITCH)
    pcb.text(label, (at[0] + 10, at[1] - 4.3), 0.8, justify="center")
    pcb.text("1 VM  2 GND  3 IN2  4 IN1", (at[0] + 10, at[1] + 20.5), 0.65, justify="center")

# A separate low-voltage source powers all three DRV8871 modules.  It is never
# connected to the external DM422 24 V supply on this board.
terminal("J_VM_IN", "N20/TRACK LOW-VOLTAGE VM INPUT", (155.0, 110.0), ["VM_MOTOR", "GND"])
pcb.text("LOW-VOLTAGE VM ONLY", (160.1, 104.5), 0.85, justify="center")
pcb.text("DO NOT CONNECT DM422 24V", (160.1, 101.7), 0.7, justify="center")

# Two encoder connectors for the two closed-loop GEAR axes.
ENC_AT = [(72.0, 90.0), (107.0, 90.0)]
ENC_DEST = {}
for axis, at in enumerate(ENC_AT):
    # Connector labels are physical encoder A/C1 and B/C2; config.h swaps the
    # logical PCNT channel names to preserve the established PID sign.
    enc_a = gpio_net(GEAR_PINS[axis]["ENCB"])
    enc_b = gpio_net(GEAR_PINS[axis]["ENCA"])
    terminal(f"J_ENC{axis}", f"GEAR {axis} ENCODER", at, ["3V3", "GND", enc_a, enc_b])
    ENC_DEST[(axis, "A")] = (at[0] + 2 * 5.08, at[1])
    ENC_DEST[(axis, "B")] = (at[0] + 3 * 5.08, at[1])
    pcb.text(f"ENC{axis} 3V3 GND A B", (at[0] + 7.62, 84.5), 0.75, justify="center")

# Multifunction 3.3 V expansion header.  Preferred use is TTL UART to a
# Pico/RP2040 pulse extender.  GPIO42/47 may instead be SDA/SCL for a slow I2C
# GPIO expander; UART, I2C and the optional RS485 module are mutually exclusive.
UART_AT = (45.0, 110.0)
UART_NETS = ["5V_USB", "3V3", "GND", gpio_net(AUX_PINS["TX"]), gpio_net(AUX_PINS["RX"]), gpio_net(AUX_PINS["DE"])]
uart_pads = [header_pad(i + 1, net, i * PITCH, 0) for i, net in enumerate(UART_NETS)]
pcb.footprint(
    "PinHeader_1x06_P2.54",
    "J_AUX_UART",
    "Pico/RP2040 AUX UART or I2C",
    UART_AT,
    uart_pads,
    (((-2.0, -2.0), (5 * PITCH + 2.0, 2.0), "F.SilkS", 0.15),),
)
pcb.text("AUX: 5V 3V3 G TX/SDA42 RX/SCL47 DE48", (51.4, 105.5), 0.62, justify="center")

# Optional ready-made 3.3 V RS485 transceiver module.  Populate this OR use the
# direct UART header.  The footprint is intentionally generic/mock.
RS_AT = (75.0, 98.0)
rs_cfg = MODULE_GEOMETRY["rs485_module"]
rs_pads = [
    header_pad("VCC", "3V3", 0, 0),
    header_pad("RO", gpio_net(AUX_PINS["RX"]), 0, PITCH),
    header_pad("RE", gpio_net(AUX_PINS["DE"]), 0, 2 * PITCH),
    header_pad("DE", gpio_net(AUX_PINS["DE"]), 0, 3 * PITCH),
    header_pad("DI", gpio_net(AUX_PINS["TX"]), 0, 4 * PITCH),
    header_pad("GND", "GND", rs_cfg["bus_header_spacing"], 0),
    header_pad("A", "RS485_A", rs_cfg["bus_header_spacing"], PITCH),
    header_pad("B", "RS485_B", rs_cfg["bus_header_spacing"], 2 * PITCH),
]
pcb.footprint(
    "ModuleSocket_RS485_Generic",
    "U_RS485",
    "3V3 RS485 TRANSCEIVER MODULE (MOCK)",
    RS_AT,
    rs_pads,
    (
        ((-2.0, -2.0), (rs_cfg["body_w"] - 2.0, rs_cfg["body_h"] - 2.0), "F.SilkS", 0.15),
        ((-3.0, -3.0), (rs_cfg["body_w"] - 1.0, rs_cfg["body_h"] - 1.0), "F.CrtYd", 0.05),
    ),
)
terminal("J_RS485", "RS485 BUS", (128.0, 110.0), ["RS485_A", "RS485_B", "GND"])
pcb.text("RS485 A B GND", (133.1, 104.5), 0.72, justify="center")

# Ready-made TCA9548A I2C multiplexer module.  Six AS5600 modules all use the
# fixed 0x36 address, so each one gets a separate mux channel.  The footprint
# and every pad position are MOCK until the actual breakout board is measured.
TCA_AT = (62.0, 120.0)
tca_cfg = MODULE_GEOMETRY["tca9548a_module"]
tca_host_nets = [
    "3V3", "GND", gpio_net(I2C_MUX_PINS["SDA"]), gpio_net(I2C_MUX_PINS["SCL"]),
    "3V3", "GND", "GND", "GND",
]
tca_pads = [
    header_pad(tca_cfg["assumed_host_order"][index], net, 0, index * PITCH)
    for index, net in enumerate(tca_host_nets)
]
TCA_CHANNEL_PAD = {}
for channel in range(8):
    row = 0 if channel < 4 else 1
    column = channel if channel < 4 else channel - 4
    x = 16.0 + column * tca_cfg["channel_group_pitch"]
    y = 0.0 if row == 0 else 27.0
    sda_net = f"AS{channel}_SDA" if channel < 6 else ""
    scl_net = f"AS{channel}_SCL" if channel < 6 else ""
    tca_pads.append(header_pad(f"SD{channel}", sda_net, x, y))
    tca_pads.append(header_pad(f"SC{channel}", scl_net, x + PITCH, y))
    if channel < 6:
        TCA_CHANNEL_PAD[(channel, "SDA")] = (TCA_AT[0] + x, TCA_AT[1] + y)
        TCA_CHANNEL_PAD[(channel, "SCL")] = (TCA_AT[0] + x + PITCH, TCA_AT[1] + y)

margin = tca_cfg["courtyard_margin"]
pcb.footprint(
    "ModuleSocket_TCA9548A_8CH_MOCK",
    "U_I2CMUX",
    "TCA9548A READY-MADE MODULE (MOCK VERIFY ALL PINS)",
    TCA_AT,
    tca_pads,
    (
        ((-2.0, -2.0), (tca_cfg["body_w"] - 2.0, tca_cfg["body_h"] - 2.0), "F.SilkS", 0.15),
        ((-2.0 - margin, -2.0 - margin),
         (tca_cfg["body_w"] - 2.0 + margin, tca_cfg["body_h"] - 2.0 + margin),
         "F.CrtYd", 0.05),
    ),
)
pcb.text("TCA9548A I2C MUX MODULE - MOCK", (87.0, 116.0), 0.78, justify="center")
pcb.text("HOST: 3V3 G SDA42 SCL47 RST A0 A1 A2", (87.0, 151.5), 0.62, justify="center")

# Optional ready-made PCF8575 module on the same 3.3 V I2C bus.  Its
# quasi-bidirectional pins are logic-only reservations: they are deliberately
# NOT wired to DM optocoupler inputs.  A measured/verified ready-made
# open-collector or ULN-type buffer stage is required before field use.
PCF_AT = (120.0, 120.0)
pcf_cfg = MODULE_GEOMETRY["pcf8575_module"]
pcf_host_nets = [
    "3V3", "GND", gpio_net(I2C_MUX_PINS["SDA"]), gpio_net(I2C_MUX_PINS["SCL"]),
    "PCF_INT", "GND", "GND", "GND",
]
pcf_pads = [
    header_pad(pcf_cfg["assumed_host_order"][index], net, 0, index * PITCH)
    for index, net in enumerate(pcf_host_nets)
]
PCF_PORT_PAD = {}
for port in range(16):
    row = 0 if port < 8 else 1
    column = port if port < 8 else port - 8
    # Two 1x8 rows on a 2.54 mm grid fit the selected Adafruit-size envelope.
    # Exact pad origins remain MOCK until the purchased module fab drawing is
    # matched; do not use this as a production footprint.
    x = 18.0 + column * PITCH
    y = 0.0 if row == 0 else 7 * PITCH
    net = f"PCF_P{port}"
    pcf_pads.append(header_pad(f"P{port}", net, x, y))
    PCF_PORT_PAD[port] = (PCF_AT[0] + x, PCF_AT[1] + y)

margin = pcf_cfg["courtyard_margin"]
pcb.footprint(
    "ModuleSocket_PCF8575_16IO_MOCK",
    "U_IOEXP",
    "PCF8575 READY-MADE MODULE (DNP MOCK VERIFY ALL PINS)",
    PCF_AT,
    pcf_pads,
    (
        ((-2.0, -2.0), (pcf_cfg["body_w"] - 2.0, pcf_cfg["body_h"] - 2.0), "F.SilkS", 0.15),
        ((-2.0 - margin, -2.0 - margin),
         (pcf_cfg["body_w"] - 2.0 + margin, pcf_cfg["body_h"] - 2.0 + margin),
         "F.CrtYd", 0.05),
    ),
)
pcb.text("PCF8575 IO EXPANDER - DNP / MOCK", (145.0, 116.0), 0.74, justify="center")
pcb.text("P0-P5 DIR | P6-P11 ENA | P12-P15 RESERVED", (145.0, 151.5), 0.58, justify="center")

def logic_header(reference, value, at, nets):
    pads = [header_pad(index + 1, net, index * PITCH, 0) for index, net in enumerate(nets)]
    width = (len(nets) - 1) * PITCH
    pcb.footprint(
        f"PinHeader_1x{len(nets):02d}_P2.54",
        reference,
        value,
        at,
        pads,
        (((-2.0, -2.0), (width + 2.0, 2.0), "F.SilkS", 0.15),),
    )

PCF_DIR_HEADER_AT = (115.0, 177.0)
PCF_ENA_HEADER_AT = (140.0, 177.0)
PCF_RESERVED_HEADER_AT = (86.0, 177.0)
logic_header("J_PCF_DIR", "PCF LOGIC ONLY: 3V3 G P0..P5", PCF_DIR_HEADER_AT,
             ["3V3", "GND"] + [f"PCF_P{port}" for port in range(6)])
logic_header("J_PCF_ENA", "PCF LOGIC ONLY: 3V3 G P6..P11", PCF_ENA_HEADER_AT,
             ["3V3", "GND"] + [f"PCF_P{port}" for port in range(6, 12)])
logic_header("J_PCF_RSVD", "PCF RESERVED: 3V3 G P12..P15 INT", PCF_RESERVED_HEADER_AT,
             ["3V3", "GND"] + [f"PCF_P{port}" for port in range(12, 16)] + ["PCF_INT"])
pcb.text("PCF RAW LOGIC ONLY - EXTERNAL BUFFER REQUIRED", (130.0, 182.0), 0.66, justify="center")

# One connector per external AS5600 module: 3V3/GND/SDAx/SCLx.
AS_CONNECTOR_X = DM_CONNECTOR_X
AS_CONNECTOR_Y = 200.0
AS_DEST = {}
for channel, x in enumerate(AS_CONNECTOR_X):
    terminal(
        f"J_AS{channel}",
        f"AS5600 #{channel} CH{channel}",
        (x, AS_CONNECTOR_Y),
        ["3V3", "GND", f"AS{channel}_SDA", f"AS{channel}_SCL"],
    )
    AS_DEST[(channel, "SDA")] = (x + 2 * 5.08, AS_CONNECTOR_Y)
    AS_DEST[(channel, "SCL")] = (x + 3 * 5.08, AS_CONNECTOR_Y)
    pcb.text(f"AS{channel} 3V3 G SDA SCL", (x + 7.62, 204.8), 0.62, justify="center")

# Board mounting holes.
for x, y in ((4, 4), (176, 4), (4, 206), (176, 206)):
    pcb.footprint(
        "MountingHole_M3",
        f"H{x}_{y}",
        "M3",
        (x, y),
        [("", "", 0, 0, 6.0, 3.2, "np")],
    )


# ---------------------------------------------------------------------------
# Routing
# ---------------------------------------------------------------------------
route_index = 0


def routed_signal(net, source, destination, lane_y=None, destination_escape_x=None):
    """Two-layer Manhattan route with a unique vertical spine and B.Cu lane."""
    global route_index
    if lane_y is None:
        lane_y = 24.0 + route_index * 0.85
    spine_x = 43.0 + route_index * 1.25
    # Escape away from the module body/antenna: J1 goes left, J3 goes right.
    source_escape = (source[0] + (-2.0 if source[0] < 25 else 2.0), source[1])
    if destination_escape_x is None:
        destination_escape_x = destination[0] - 2.2 - (route_index % 3) * 1.1
    destination_escape = (destination_escape_x, lane_y)
    pcb.segment(net, "F.Cu", SIG_W, source, source_escape)
    pcb.via(net, source_escape)
    pcb.segment(net, "B.Cu", SIG_W, source_escape, (spine_x, source_escape[1]))
    pcb.via(net, (spine_x, source_escape[1]))
    pcb.segment(net, "F.Cu", SIG_W, (spine_x, source_escape[1]), (spine_x, lane_y))
    pcb.via(net, (spine_x, lane_y))
    pcb.segment(net, "B.Cu", SIG_W, (spine_x, lane_y), destination_escape)
    pcb.via(net, destination_escape)
    pcb.segment(net, "F.Cu", SIG_W, destination_escape, (destination_escape_x, destination[1]))
    pcb.segment(net, "F.Cu", SIG_W, (destination_escape_x, destination[1]), destination)
    route_index += 1


# 6 x PUL/DIR.
for axis, (pul, direction) in enumerate(STEPPER_PINS):
    routed_signal(gpio_net(pul), GPIO_PAD[pul], DM_DEST[(axis, "PUL")])
    routed_signal(gpio_net(direction), GPIO_PAD[direction], DM_DEST[(axis, "DIR")])

# 2 x closed-loop GEAR motor inputs and encoders.
for axis, pins in enumerate(GEAR_PINS):
    routed_signal(gpio_net(pins["IN1"]), GPIO_PAD[pins["IN1"]], DRV_DEST[(axis, "IN2")])
    routed_signal(gpio_net(pins["IN2"]), GPIO_PAD[pins["IN2"]], DRV_DEST[(axis, "IN1")])
    routed_signal(gpio_net(pins["ENCB"]), GPIO_PAD[pins["ENCB"]], ENC_DEST[(axis, "A")])
    routed_signal(gpio_net(pins["ENCA"]), GPIO_PAD[pins["ENCA"]], ENC_DEST[(axis, "B")])

# Independent open-loop TRACK D DRV8871.
routed_signal(gpio_net(TRACK_D_PINS["IN2"]), GPIO_PAD[TRACK_D_PINS["IN2"]], DRV_DEST[(2, "IN2")])
routed_signal(gpio_net(TRACK_D_PINS["IN1"]), GPIO_PAD[TRACK_D_PINS["IN1"]], DRV_DEST[(2, "IN1")])

# AUX UART nets to direct header.  The same nets reach the optional RS485 module.
for offset, key in enumerate(("TX", "RX", "DE"), start=3):
    pin = AUX_PINS[key]
    routed_signal(gpio_net(pin), GPIO_PAD[pin], (UART_AT[0] + offset * PITCH, UART_AT[1]))

# Short module/header fanout for RS485 option.
rs_logic_targets = {
    "TX": (RS_AT[0], RS_AT[1] + 4 * PITCH),
    "RX": (RS_AT[0], RS_AT[1] + PITCH),
    "DE": (RS_AT[0], RS_AT[1] + 2 * PITCH),
}
for index, key in enumerate(("TX", "RX", "DE")):
    src = (UART_AT[0] + (3 + index) * PITCH, UART_AT[1])
    dst = rs_logic_targets[key]
    net = gpio_net(AUX_PINS[key])
    y = 94.0 + index * 1.2
    pcb.segment(net, "B.Cu", SIG_W, src, (src[0], y))
    pcb.segment(net, "B.Cu", SIG_W, (src[0], y), (dst[0], y))
    pcb.segment(net, "B.Cu", SIG_W, (dst[0], y), dst)

# Tie RE and DE pads together on the generic RS485 module footprint.
pcb.segment(gpio_net(AUX_PINS["DE"]), "F.Cu", SIG_W,
            (RS_AT[0], RS_AT[1] + 2 * PITCH), (RS_AT[0], RS_AT[1] + 3 * PITCH))

# RS485 A/B to external bus terminal.
pcb.segment("RS485_A", "F.Cu", SIG_W,
            (RS_AT[0] + rs_cfg["bus_header_spacing"], RS_AT[1] + PITCH),
            (128.0, 110.0))
pcb.segment("RS485_B", "F.Cu", SIG_W,
            (RS_AT[0] + rs_cfg["bus_header_spacing"], RS_AT[1] + 2 * PITCH),
            (128.0 + 5.08, 110.0))

# Low-voltage motor supply bus: J_VM_IN -> all three module VM pads.
vm_input = (155.0, 110.0)
pcb.segment("VM_MOTOR", "B.Cu", MOTOR_PWR_W, vm_input, (155.0, 98.0))
pcb.segment("VM_MOTOR", "B.Cu", MOTOR_PWR_W, (73.0, 98.0), (155.0, 98.0))
for at in DRV_AT:
    pcb.segment("VM_MOTOR", "B.Cu", MOTOR_PWR_W, (at[0], 98.0), (at[0], at[1]))

# 3V3 distribution to both encoders, UART header and RS485 module.
v33_source = devkit_pad("L", 1)
pcb.segment("3V3", "B.Cu", LOGIC_PWR_W, v33_source, (39.0, v33_source[1]))
pcb.segment("3V3", "B.Cu", LOGIC_PWR_W, (39.0, v33_source[1]), (39.0, 93.0))
pcb.segment("3V3", "B.Cu", LOGIC_PWR_W, (39.0, 93.0), (107.0, 93.0))
for x, y in ENC_AT:
    pcb.segment("3V3", "B.Cu", LOGIC_PWR_W, (x, 93.0), (x, y))
pcb.segment("3V3", "B.Cu", LOGIC_PWR_W, (45.0 + PITCH, 93.0), (45.0 + PITCH, 110.0))
pcb.segment("3V3", "B.Cu", LOGIC_PWR_W, (75.0, 93.0), (75.0, 106.0))

# USB-derived 5V is exposed only on the expansion header and is not tied to VM.
v5_source = devkit_pad("L", 21)
pcb.segment("5V_USB", "B.Cu", LOGIC_PWR_W, v5_source, (41.0, v5_source[1]))
pcb.segment("5V_USB", "B.Cu", LOGIC_PWR_W, (41.0, v5_source[1]), (41.0, 110.0))
pcb.segment("5V_USB", "B.Cu", LOGIC_PWR_W, (41.0, 110.0), UART_AT)

# Shared host I2C.  GPIO42/47 remain physically connected to the original AUX
# headers, so I2C-mux use and AUX UART/RS485 use are mutually exclusive.
i2c_sda_uart = (UART_AT[0] + 3 * PITCH, UART_AT[1])
i2c_scl_uart = (UART_AT[0] + 4 * PITCH, UART_AT[1])
tca_sda = (TCA_AT[0], TCA_AT[1] + 2 * PITCH)
tca_scl = (TCA_AT[0], TCA_AT[1] + 3 * PITCH)
pcf_sda = (PCF_AT[0], PCF_AT[1] + 2 * PITCH)
pcf_scl = (PCF_AT[0], PCF_AT[1] + 3 * PITCH)
pcb.segment(gpio_net(I2C_MUX_PINS["SDA"]), "B.Cu", SIG_W, i2c_sda_uart, (58.0, 112.0))
pcb.segment(gpio_net(I2C_MUX_PINS["SDA"]), "B.Cu", SIG_W, (58.0, 112.0), (58.0, tca_sda[1]))
pcb.segment(gpio_net(I2C_MUX_PINS["SDA"]), "B.Cu", SIG_W, (58.0, tca_sda[1]), tca_sda)
pcb.segment(gpio_net(I2C_MUX_PINS["SDA"]), "B.Cu", SIG_W, tca_sda, pcf_sda)
pcb.segment(gpio_net(I2C_MUX_PINS["SCL"]), "F.Cu", SIG_W, i2c_scl_uart, (60.0, 114.0))
pcb.segment(gpio_net(I2C_MUX_PINS["SCL"]), "F.Cu", SIG_W, (60.0, 114.0), (60.0, tca_scl[1]))
pcb.segment(gpio_net(I2C_MUX_PINS["SCL"]), "F.Cu", SIG_W, (60.0, tca_scl[1]), tca_scl)
pcb.segment(gpio_net(I2C_MUX_PINS["SCL"]), "F.Cu", SIG_W, tca_scl, pcf_scl)

# 3V3 for both optional I2C modules, TCA reset-high, six AS5600 connectors and
# the PCF raw-logic headers.  GND is supplied by the common F.Cu plane.
pcb.segment("3V3", "B.Cu", LOGIC_PWR_W, (39.0, 93.0), (39.0, 196.0))
pcb.segment("3V3", "B.Cu", LOGIC_PWR_W, (39.0, 117.0), (118.0, 117.0))
pcb.segment("3V3", "B.Cu", LOGIC_PWR_W, (TCA_AT[0], 117.0), (TCA_AT[0], TCA_AT[1]))
pcb.segment("3V3", "B.Cu", LOGIC_PWR_W, (118.0, 117.0), (PCF_AT[0], PCF_AT[1]))
pcb.segment("3V3", "B.Cu", LOGIC_PWR_W, (TCA_AT[0], TCA_AT[1]),
            (TCA_AT[0] - 4.0, TCA_AT[1]))
pcb.segment("3V3", "B.Cu", LOGIC_PWR_W, (TCA_AT[0] - 4.0, TCA_AT[1]),
            (TCA_AT[0] - 4.0, TCA_AT[1] + 4 * PITCH))
pcb.segment("3V3", "B.Cu", LOGIC_PWR_W, (TCA_AT[0] - 4.0, TCA_AT[1] + 4 * PITCH),
            (TCA_AT[0], TCA_AT[1] + 4 * PITCH))
pcb.segment("3V3", "B.Cu", LOGIC_PWR_W, (AS_CONNECTOR_X[0], 196.0),
            (AS_CONNECTOR_X[-1], 196.0))
for x in AS_CONNECTOR_X:
    pcb.segment("3V3", "B.Cu", LOGIC_PWR_W, (x, 196.0), (x, AS_CONNECTOR_Y))
for x, y in (PCF_DIR_HEADER_AT, PCF_ENA_HEADER_AT, PCF_RESERVED_HEADER_AT):
    pcb.segment("3V3", "B.Cu", LOGIC_PWR_W, (x, 196.0), (x, y))

# Six isolated downstream I2C channels for fixed-address AS5600 modules.
as_route_index = 0
for channel in range(6):
    for signal in ("SDA", "SCL"):
        net = f"AS{channel}_{signal}"
        source = TCA_CHANNEL_PAD[(channel, signal)]
        destination = AS_DEST[(channel, signal)]
        lane_y = 152.0 + as_route_index * 0.8
        escape_x = source[0] + (-0.7 if channel < 4 else 0.7)
        pcb.segment(net, "F.Cu", SIG_W, source, (escape_x, source[1]))
        pcb.segment(net, "F.Cu", SIG_W, (escape_x, source[1]), (escape_x, lane_y))
        pcb.via(net, (escape_x, lane_y))
        pcb.segment(net, "B.Cu", SIG_W, (escape_x, lane_y), (destination[0] - 1.0, lane_y))
        pcb.via(net, (destination[0] - 1.0, lane_y))
        pcb.segment(net, "F.Cu", SIG_W, (destination[0] - 1.0, lane_y),
                    (destination[0] - 1.0, destination[1]))
        pcb.segment(net, "F.Cu", SIG_W, (destination[0] - 1.0, destination[1]), destination)
        as_route_index += 1

# PCF8575 raw logic fanout.  No PCF net is connected to a DM connector.
PCF_LOGIC_DEST = {}
for port in range(6):
    PCF_LOGIC_DEST[port] = (PCF_DIR_HEADER_AT[0] + (port + 2) * PITCH, PCF_DIR_HEADER_AT[1])
for port in range(6, 12):
    PCF_LOGIC_DEST[port] = (PCF_ENA_HEADER_AT[0] + (port - 6 + 2) * PITCH, PCF_ENA_HEADER_AT[1])
for port in range(12, 16):
    PCF_LOGIC_DEST[port] = (PCF_RESERVED_HEADER_AT[0] + (port - 12 + 2) * PITCH,
                            PCF_RESERVED_HEADER_AT[1])

for port in range(16):
    net = f"PCF_P{port}"
    source = PCF_PORT_PAD[port]
    destination = PCF_LOGIC_DEST[port]
    lane_y = 164.0 + port * 0.55
    escape_x = source[0] + (-0.8 if port < 8 else 0.8)
    pcb.segment(net, "F.Cu", SIG_W, source, (escape_x, source[1]))
    pcb.segment(net, "F.Cu", SIG_W, (escape_x, source[1]), (escape_x, lane_y))
    pcb.via(net, (escape_x, lane_y))
    pcb.segment(net, "B.Cu", SIG_W, (escape_x, lane_y), (destination[0], lane_y))
    pcb.segment(net, "B.Cu", SIG_W, (destination[0], lane_y), destination)

pcf_int_source = (PCF_AT[0], PCF_AT[1] + 4 * PITCH)
pcf_int_destination = (PCF_RESERVED_HEADER_AT[0] + 6 * PITCH, PCF_RESERVED_HEADER_AT[1])
pcb.segment("PCF_INT", "F.Cu", SIG_W, pcf_int_source, (PCF_AT[0] - 3.0, pcf_int_source[1]))
pcb.via("PCF_INT", (PCF_AT[0] - 3.0, pcf_int_source[1]))
pcb.segment("PCF_INT", "B.Cu", SIG_W, (PCF_AT[0] - 3.0, pcf_int_source[1]),
            (pcf_int_destination[0], pcf_int_source[1]))
pcb.segment("PCF_INT", "B.Cu", SIG_W, (pcf_int_destination[0], pcf_int_source[1]),
            pcf_int_destination)


# ---------------------------------------------------------------------------
# Copper zones, keepouts and silkscreen
# ---------------------------------------------------------------------------
pcb.items.append(
    f'''  (zone (net {N["GND"]}) (net_name "GND") (layer "F.Cu") (uuid "{uid()}") (hatch edge 0.5)
    (connect_pads yes (clearance 0.5))
    (min_thickness 0.3)
    (fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5))
    (polygon (pts (xy 0 0) (xy {BOARD_W} 0) (xy {BOARD_W} {BOARD_H}) (xy 0 {BOARD_H}))))'''
)

# DevKit antenna keepout: no tracks, vias or copper pour under/just beyond antenna.
pcb.items.append(
    f'''  (zone (net 0) (net_name "") (layer "F.Cu") (uuid "{uid()}") (hatch edge 0.5)
    (connect_pads (clearance 0))
    (min_thickness 0.25)
    (keepout (tracks not_allowed) (vias not_allowed) (pads allowed) (copperpour not_allowed) (footprints allowed))
    (fill no (thermal_gap 0.5) (thermal_bridge_width 0.5))
    (polygon (pts (xy 13.5 44) (xy 33.4 44) (xy 33.4 57) (xy 13.5 57))))'''
)

pcb.rect((0, 0), (BOARD_W, BOARD_H), "Edge.Cuts", 0.05)
pcb.text("ESP32-S3 6xDM422 + 2xGEAR + TRACK-D MODULAR CARRIER", (90, 18.5), 1.25, justify="center")
pcb.text("REV 2 MOCK MODULE FOOTPRINTS - MEASURE BEFORE FABRICATION", (90, 21.5), 0.78, justify="center")
pcb.text("ESP32-S3 DEVKITC-1", (24.8, 43.0), 0.85, justify="center")
pcb.text("ANTENNA KEEPOUT", (24.0, 51.0), 0.72, justify="center")
pcb.text("USB END", (24.0, 104.0), 0.72, justify="center")
pcb.text("DM422 POWER: EXTERNAL FUSED 24V DISTRIBUTION (NOT ON THIS PCB)", (90, 29.0), 0.75, justify="center")
pcb.text("DNP OPTIONS: TCA9548A + 6xAS5600 + PCF8575", (90, 187.0), 0.72, justify="center")
pcb.text("GPIO42/47: I2C OR AUX UART/RS485 - MUTUALLY EXCLUSIVE", (90, 190.0), 0.66, justify="center")
pcb.text("AS5600: 3V3 LOGIC ONLY - KEEP I2C CABLES SHORT", (90, 193.0), 0.66, justify="center")


LAYERS = '''  (layers
    (0 "F.Cu" signal)
    (31 "B.Cu" signal)
    (32 "B.Adhes" user "B.Adhesive")
    (33 "F.Adhes" user "F.Adhesive")
    (34 "B.Paste" user)
    (35 "F.Paste" user)
    (36 "B.SilkS" user "B.Silkscreen")
    (37 "F.SilkS" user "F.Silkscreen")
    (38 "B.Mask" user "B.Mask")
    (39 "F.Mask" user "F.Mask")
    (40 "Dwgs.User" user "User.Drawings")
    (41 "Cmts.User" user "User.Comments")
    (42 "Eco1.User" user "User.Eco1")
    (43 "Eco2.User" user "User.Eco2")
    (44 "Edge.Cuts" user)
    (45 "Margin" user)
    (46 "B.CrtYd" user "B.Courtyard")
    (47 "F.CrtYd" user "F.Courtyard")
    (48 "B.Fab" user)
    (49 "F.Fab" user))'''

netclass = '    (netclass "Default" (clearance 0.3) (track_width 0.4) (via_dia 1.2) (via_drill 0.6)\n'
netclass += "\n".join(f'      (net {index} "{name}")' for index, name in enumerate(NETS) if index)
netclass += ")"
net_defs = "\n".join(f'  (net {index} "{name}")' for index, name in enumerate(NETS))

pcb_text = f'''(kicad_pcb
  (version 20240108)
  (generator "pcbnew")
  (generator_version "8.0")
  (general (thickness 1.6) (legacy_teardrops no))
  (paper "A4")
{LAYERS}
  (setup
    (pad_to_mask_clearance 0)
    (allow_soldermask_bridges_in_footprints no)
{netclass})
{net_defs}
{chr(10).join(pcb.items)}
)
'''

(OUT_DIR / "stepper_carrier.kicad_pcb").write_text(pcb_text, encoding="utf-8", newline="\n")
(OUT_DIR / "stepper_carrier.kicad_pro").write_text(
    '{\n  "meta": { "filename": "stepper_carrier.kicad_pro", "version": 1 },\n'
    '  "sheets": [],\n  "text_variables": {}\n}\n',
    encoding="utf-8",
    newline="\n",
)
print(f"OK: {OUT_DIR / 'stepper_carrier.kicad_pcb'} ({len(pcb.items)} items)")
