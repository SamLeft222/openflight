"""Generate the IWR6843LEVM -> Raspberry Pi 5 SPI adapter KiCad project.

Run with KiCad's bundled Python (it needs the ``pcbnew`` module):

    /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3 \
        cad/iwr6843_spi_adapter/generate.py

Single source of truth: ``PARTS`` and ``NETS`` below drive BOTH the schematic
and the PCB, so the two cannot drift apart. ``make check`` style validation is
done afterwards with kicad-cli (ERC, DRC + schematic parity), see README.md.

Every pinout / land pattern here is taken from the manufacturer documents,
not from memory or from generic KiCad library parts:

* IWR6843LEVM J1 pin map: TI PROC116A schematic sheets 5, 8, 13 (SPI-A via the
  TS3A5018 mux, 0-ohm R7/R9/R10/R11/R12, VIOIN = RADAR_3V3) and BOM
  (J1 = Samtec QSH-030-01-L-D-A).
* QTH-030-01-L-D-A land pattern: Samtec "Recommended PCB layout for
  QTH-XXX-XX-X-D-XXX", Rev D.  Mated height with QSH-01: 5.00 mm (QTH print
  Table 1).
* SN74LVC2G34 pinout (1A=1, 1Y=6, 2A=3, 2Y=4, GND=2, VCC=5), Ioff
  partial-power-down, DBV0006A land pattern: TI datasheet SCES370.
"""

from __future__ import annotations

import json
import re
import uuid
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROJECT = "iwr6843_spi_adapter"
FP_LIB = "openflight_adapter"
KICAD = Path("/Applications/KiCad/KiCad.app/Contents/SharedSupport")
SYM_DIR = KICAD / "symbols"
FP_DIR = KICAD / "footprints"

# Deterministic UUIDs so regenerating produces a stable diff.
_NS = uuid.UUID("6f8d2f3e-2a8e-4c1b-9d0a-0b5e1d7c9a11")


def uid(name: str) -> str:
    return str(uuid.uuid5(_NS, name))


ROOT_UUID = uid("root-sheet")

# ---------------------------------------------------------------------------
# Design data (single source of truth)
# ---------------------------------------------------------------------------


@dataclass
class Part:
    ref: str
    lib_id: str
    value: str
    footprint: str
    units: int = 1
    datasheet: str = ""
    mpn: str = ""
    manufacturer: str = ""
    description: str = ""
    # Whether the assembler places it (J2 is hand-soldered so its tails can
    # be trimmed flush; see README).
    assemble: bool = True
    # Schematic placement for each unit: list of (x, y) in mm.
    sch_at: list = field(default_factory=list)


QTH_FP = f"{FP_LIB}:Samtec_QTH-030-01-L-D-A"
DBV_FP = f"{FP_LIB}:TI_DBV0006A"
R0603 = "Resistor_SMD:R_0603_1608Metric"
C0603 = "Capacitor_SMD:C_0603_1608Metric"
HDR_FP = "Connector_PinHeader_2.54mm:PinHeader_1x11_P2.54mm_Vertical"
LVC_DS = "https://www.ti.com/lit/ds/symlink/sn74lvc2g34.pdf"

PARTS = [
    Part(
        "J1",
        "Connector_Generic_MountingPin:Conn_02x30_Odd_Even_MountingPin",
        "QTH-030-01-L-D-A",
        QTH_FP,
        mpn="QTH-030-01-L-D-A",
        manufacturer="Samtec",
        description="0.5mm 2x30 Q Strip terminal, alignment pins, mates QSH-030-01",
        datasheet="https://suddendocs.samtec.com/prints/qth-xxx-xx-x-d-xx-mkt.pdf",
        sch_at=[(55.88, 101.6)],
    ),
    Part(
        "J2",
        "Connector_Generic:Conn_01x11",
        "Pi5_jumpers",
        HDR_FP,
        mpn="",
        description="1x11 2.54mm male pin header (customer installs, trim tails flush)",
        assemble=False,
        sch_at=[(254.0, 76.2)],
    ),
    # U1: radar-powered, Pi -> radar (ch2 = MOSI, ch1 = SCLK)
    Part(
        "U1",
        "74xGxx:74LVC2G34",
        "SN74LVC2G34DBVR",
        DBV_FP,
        units=3,
        mpn="SN74LVC2G34DBVR",
        manufacturer="Texas Instruments",
        description="Dual buffer, Ioff partial-power-down, SOT-23-6",
        datasheet=LVC_DS,
        sch_at=[(160.02, 45.72), (160.02, 60.96), (160.02, 30.48)],
    ),
    # U2: radar-powered, Pi -> radar (ch2 = CS, ch1 = spare, input grounded)
    Part(
        "U2",
        "74xGxx:74LVC2G34",
        "SN74LVC2G34DBVR",
        DBV_FP,
        units=3,
        mpn="SN74LVC2G34DBVR",
        manufacturer="Texas Instruments",
        description="Dual buffer, Ioff partial-power-down, SOT-23-6",
        datasheet=LVC_DS,
        sch_at=[(160.02, 101.6), (160.02, 86.36), (187.96, 30.48)],
    ),
    # U3: Pi-powered, radar -> Pi (ch1 = HOST_INTR, ch2 = MISO)
    Part(
        "U3",
        "74xGxx:74LVC2G34",
        "SN74LVC2G34DBVR",
        DBV_FP,
        units=3,
        mpn="SN74LVC2G34DBVR",
        manufacturer="Texas Instruments",
        description="Dual buffer, Ioff partial-power-down, SOT-23-6",
        datasheet=LVC_DS,
        sch_at=[(160.02, 132.08), (160.02, 147.32), (215.9, 30.48)],
    ),
    Part(
        "C1",
        "Device:C",
        "100nF",
        C0603,
        mpn="CL10B104KB8NNNC",
        manufacturer="Samsung Electro-Mechanics",
        description="100nF 50V X7R 0603 MLCC",
        sch_at=[(132.08, 172.72)],
    ),
    Part(
        "C2",
        "Device:C",
        "100nF",
        C0603,
        mpn="CL10B104KB8NNNC",
        manufacturer="Samsung Electro-Mechanics",
        description="100nF 50V X7R 0603 MLCC",
        sch_at=[(142.24, 172.72)],
    ),
    Part(
        "C3",
        "Device:C",
        "100nF",
        C0603,
        mpn="CL10B104KB8NNNC",
        manufacturer="Samsung Electro-Mechanics",
        description="100nF 50V X7R 0603 MLCC",
        sch_at=[(152.4, 172.72)],
    ),
    Part(
        "R1",
        "Device:R",
        "100k",
        R0603,
        mpn="RC0603FR-07100KL",
        manufacturer="YAGEO",
        description="100k 1% 0603 resistor",
        sch_at=[(203.2, 101.6)],
    ),
    Part(
        "R2",
        "Device:R",
        "100k",
        R0603,
        mpn="RC0603FR-07100KL",
        manufacturer="YAGEO",
        description="100k 1% 0603 resistor",
        sch_at=[(203.2, 60.96)],
    ),
    Part(
        "R3",
        "Device:R",
        "100k",
        R0603,
        mpn="RC0603FR-07100KL",
        manufacturer="YAGEO",
        description="100k 1% 0603 resistor",
        sch_at=[(203.2, 45.72)],
    ),
    Part(
        "R4",
        "Device:R",
        "100k",
        R0603,
        mpn="RC0603FR-07100KL",
        manufacturer="YAGEO",
        description="100k 1% 0603 resistor",
        sch_at=[(119.38, 147.32)],
    ),
    Part(
        "R5",
        "Device:R",
        "100k",
        R0603,
        mpn="RC0603FR-07100KL",
        manufacturer="YAGEO",
        description="100k 1% 0603 resistor",
        sch_at=[(119.38, 132.08)],
    ),
    Part(
        "R6",
        "Device:R",
        "33",
        R0603,
        mpn="RC0603FR-0733RL",
        manufacturer="YAGEO",
        description="33R 1% 0603 resistor",
        sch_at=[(203.2, 132.08)],
    ),
    Part(
        "R7",
        "Device:R",
        "33",
        R0603,
        mpn="RC0603FR-0733RL",
        manufacturer="YAGEO",
        description="33R 1% 0603 resistor",
        sch_at=[(203.2, 147.32)],
    ),
]

# Nets: name -> list of (ref, pin).  J1 pin numbers are the IWR6843LEVM J1
# pin numbers (QTH pin n mates QSH pin n).
J1_GND = [20, 26, 32, 38, 44, 50]
NETS: dict[str, list[tuple[str, str]]] = {
    "GND": [("J1", str(p)) for p in J1_GND]
    + [("J1", "MP")]
    + [("J2", str(p)) for p in (2, 4, 6, 8, 10)]
    + [
        ("U1", "2"),
        ("U2", "2"),
        ("U3", "2"),
        ("U2", "1"),
        ("C1", "2"),
        ("C2", "2"),
        ("C3", "2"),
        ("R2", "2"),
        ("R3", "2"),
        ("R4", "2"),
        ("R5", "2"),
    ],
    "RADAR_3V3": [
        ("J1", "4"),
        ("J1", "6"),
        ("U1", "5"),
        ("U2", "5"),
        ("C1", "1"),
        ("C2", "1"),
        ("R1", "1"),
    ],
    "PI_3V3": [("J2", "1"), ("U3", "5"), ("C3", "1")],
    "PI_CS": [("J2", "11"), ("U2", "3"), ("R1", "2")],
    "PI_SCLK": [("J2", "9"), ("U1", "1"), ("R2", "1")],
    "PI_MOSI": [("J2", "7"), ("U1", "3"), ("R3", "1")],
    "PI_MISO": [("J2", "5"), ("R7", "2")],
    "PI_INTR": [("J2", "3"), ("R6", "2")],
    "MISO_BUF": [("U3", "4"), ("R7", "1")],
    "INTR_BUF": [("U3", "6"), ("R6", "1")],
    "RADAR_CS": [("U2", "4"), ("J1", "19")],
    "RADAR_SCLK": [("U1", "6"), ("J1", "21")],
    "RADAR_MOSI": [("U1", "4"), ("J1", "23")],
    "RADAR_MISO": [("J1", "25"), ("U3", "3"), ("R4", "1")],
    "RADAR_INTR": [("J1", "16"), ("U3", "1"), ("R5", "1")],
}
NO_CONNECT = [("U2", "6")] + [
    ("J1", str(p)) for p in range(1, 61) if p not in {4, 6, 16, 19, 21, 23, 25, *J1_GND}
]
POWER_FLAG_NETS = ["GND", "RADAR_3V3", "PI_3V3"]

# ---------------------------------------------------------------------------
# Minimal S-expression reader (enough for KiCad symbol libraries)
# ---------------------------------------------------------------------------


def sexp_blocks(text: str, start: int) -> int:
    """Return the index just past the balanced s-expression starting at start."""
    depth = 0
    i = start
    in_str = False
    while i < len(text):
        c = text[i]
        if in_str:
            if c == "\\":
                i += 1
            elif c == '"':
                in_str = False
        elif c == '"':
            in_str = True
        elif c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                return i + 1
        i += 1
    raise ValueError("unbalanced s-expression")


def lib_symbol_text(lib_id: str) -> str:
    """Top-level symbol block from a stock library, renamed to lib_id."""
    lib, name = lib_id.split(":")
    text = (SYM_DIR / f"{lib}.kicad_sym").read_text()
    m = re.search(r'\n\t\(symbol "' + re.escape(name) + r'"\n', text)
    if m is None:
        raise KeyError(lib_id)
    start = m.start() + 2
    block = text[start : sexp_blocks(text, start)]
    if "(extends " in block:
        raise NotImplementedError(f"{lib_id} extends another symbol")
    return block.replace(f'(symbol "{name}"', f'(symbol "{lib_id}"', 1)


@dataclass
class LibPin:
    number: str
    unit: int
    x: float
    y: float
    rot: int


def lib_pins(lib_id: str) -> list[LibPin]:
    block = lib_symbol_text(lib_id)
    name = lib_id.split(":")[1]
    pins: list[LibPin] = []
    for m in re.finditer(r'\(symbol "' + re.escape(name) + r'_(\d+)_(\d+)"', block):
        unit = int(m.group(1))
        sub = block[m.start() : sexp_blocks(block, m.start())]
        for pm in re.finditer(r"\(pin \w+ \w+\s*\(at ([-\d.]+) ([-\d.]+) (\d+)\)", sub):
            seg = sub[pm.start() : sexp_blocks(sub, pm.start())]
            num = re.search(r'\(number "([^"]*)"', seg).group(1)
            pins.append(LibPin(num, unit, float(pm.group(1)), float(pm.group(2)), int(pm.group(3))))
    return pins


# ---------------------------------------------------------------------------
# Footprints (Samtec / TI land patterns)
# ---------------------------------------------------------------------------


def _pad(num, shape, x, y, w, h, layers='"F.Cu" "F.Paste" "F.Mask"', kind="smd", drill=None, rot=0):
    drill_s = f" (drill {drill})" if drill else ""
    at = f"{x:.4f} {y:.4f}" + (f" {rot}" if rot else "")
    return (
        f'\t(pad "{num}" {kind} {shape} (at {at}) (size {w:.4f} {h:.4f}){drill_s} '
        f'(layers {layers}) (uuid "{uid("pad" + str(num) + str(x) + str(y))}"))\n'
    )


def _line(x1, y1, x2, y2, layer, width):
    return (
        f"\t(fp_line (start {x1:.4f} {y1:.4f}) (end {x2:.4f} {y2:.4f}) "
        f'(stroke (width {width}) (type solid)) (layer "{layer}") '
        f'(uuid "{uid("ln" + layer + str((x1, y1, x2, y2)))}"))\n'
    )


def _rect(x1, y1, x2, y2, layer, width):
    return (
        _line(x1, y1, x2, y1, layer, width)
        + _line(x2, y1, x2, y2, layer, width)
        + _line(x2, y2, x1, y2, layer, width)
        + _line(x1, y2, x1, y1, layer, width)
    )


def _circle(x, y, r, layer, width, fill="none"):
    return (
        f"\t(fp_circle (center {x:.4f} {y:.4f}) (end {x + r:.4f} {y:.4f}) "
        f'(stroke (width {width}) (type solid)) (fill {fill}) (layer "{layer}") '
        f'(uuid "{uid("ci" + layer + str((x, y, r)))}"))\n'
    )


def _fp(name, descr, body, ref_y, val_y, attr="smd"):
    return (
        f'(footprint "{name}"\n\t(version 20241229)\n\t(generator "openflight_generate")\n'
        f'\t(layer "F.Cu")\n\t(descr "{descr}")\n\t(attr {attr})\n'
        f'\t(property "Reference" "REF**" (at 0 {ref_y}) (layer "F.SilkS") '
        f'(uuid "{uid(name + "ref")}") (effects (font (size 0.8 0.8) (thickness 0.12))))\n'
        f'\t(property "Value" "{name}" (at 0 {val_y}) (layer "F.Fab") '
        f'(uuid "{uid(name + "val")}") (effects (font (size 0.8 0.8) (thickness 0.12))))\n'
        + body
        + "\t(embedded_fonts no)\n)\n"
    )


# QTH-030-01-L-D-A (Samtec recommended layout Rev D, one 30-position bank).
QTH_PITCH = 0.50
QTH_ROW_Y = 3.09  # .1215 [3.09] pad centre from centreline
QTH_PAD = (0.30, 1.45)  # .0120 x .0570
QTH_X0 = -7.25  # .5710 [14.50] span, centred
QTH_GP = [(-8.445, 2.54), (-3.175, 4.70), (3.175, 4.70), (8.445, 2.54)]  # (x, length)
QTH_GP_H = 0.64  # .0250
QTH_HOLE = (9.24, -2.03, 1.02)  # -A alignment pins: x = +/-(20.00-1.52)/2, y toward row 01


def qth_pin_xy(pin: int) -> tuple[float, float]:
    k = (pin - 1) // 2
    return QTH_X0 + QTH_PITCH * k, (-QTH_ROW_Y if pin % 2 else QTH_ROW_Y)


def footprint_qth() -> str:
    body = ""
    for p in range(1, 61):
        x, y = qth_pin_xy(p)
        body += _pad(p, "rect", x, y, *QTH_PAD)
    for x, length in QTH_GP:
        body += _pad("MP", "rect", x, 0.0, length, QTH_GP_H)
    hx, hy, hd = QTH_HOLE
    for sx in (-1, 1):
        body += (
            f'\t(pad "" np_thru_hole circle (at {sx * hx:.4f} {hy:.4f}) '
            f'(size {hd} {hd}) (drill {hd}) (layers "*.Cu" "*.Mask") '
            f'(uuid "{uid("qthhole" + str(sx))}"))\n'
        )
    # Body outline (QTH print: 20.0 x 3.9 body) on Fab; courtyard incl. holes.
    body += _rect(-10.0, -1.95, 10.0, 1.95, "F.Fab", 0.1)
    body += _rect(-10.3, -4.1, 10.3, 4.1, "F.CrtYd", 0.05)
    # Silk: short end marks clear of pads, pin-1 dot outside row 01.
    for sx in (-1, 1):
        body += _line(sx * 10.0, -1.2, sx * 10.0, 1.2, "F.SilkS", 0.12)
    px, py = qth_pin_xy(1)
    body += _circle(px - 0.9, py - 0.2, 0.2, "F.SilkS", 0.12, fill="yes")
    body += (
        f'\t(fp_text user "1" (at {px - 0.9:.2f} {py - 1.1:.2f}) (layer "F.SilkS") '
        f'(uuid "{uid("qthpin1txt")}") (effects (font (size 0.6 0.6) (thickness 0.1))))\n'
    )
    return _fp(
        "Samtec_QTH-030-01-L-D-A",
        "Samtec QTH-030-01-L-D-A 0.5mm 2x30 terminal, -A alignment pins, "
        "per Samtec recommended PCB layout Rev D; mates QSH-030-01 (5.00mm)",
        body,
        -5.0,
        5.0,
    )


def footprint_dbv() -> str:
    # TI DBV0006A land pattern example: 6X (1.1 x 0.6), 0.95 pitch, (2.6) row span.
    body = ""
    left = [(1, -0.95), (2, 0.0), (3, 0.95)]
    right = [(4, 0.95), (5, 0.0), (6, -0.95)]
    for num, y in left:
        body += _pad(num, "roundrect", -1.3, y, 1.1, 0.6).replace(
            "(layers", "(roundrect_rratio 0.25) (layers"
        )
    for num, y in right:
        body += _pad(num, "roundrect", 1.3, y, 1.1, 0.6).replace(
            "(layers", "(roundrect_rratio 0.25) (layers"
        )
    body += _rect(-0.8, -1.45, 0.8, 1.45, "F.Fab", 0.1)
    body += _rect(-2.1, -1.75, 2.1, 1.75, "F.CrtYd", 0.05)
    body += _line(-0.2, -1.56, 0.2, -1.56, "F.SilkS", 0.12)
    body += _line(-0.2, 1.56, 0.2, 1.56, "F.SilkS", 0.12)
    body += _circle(-1.95, -1.55, 0.15, "F.SilkS", 0.12, fill="yes")
    return _fp("TI_DBV0006A", "TI DBV0006A SOT-23-6 per TI land pattern example", body, -2.6, 2.6)


def write_footprints() -> None:
    lib = HERE / f"{FP_LIB}.pretty"
    lib.mkdir(exist_ok=True)
    (lib / "Samtec_QTH-030-01-L-D-A.kicad_mod").write_text(footprint_qth())
    (lib / "TI_DBV0006A.kicad_mod").write_text(footprint_dbv())
    (HERE / "fp-lib-table").write_text(
        "(fp_lib_table\n\t(version 7)\n"
        f'\t(lib (name "{FP_LIB}") (type "KiCad") (uri "${{KIPRJMOD}}/{FP_LIB}.pretty") '
        '(options "") (descr "OpenFlight SPI adapter footprints"))\n)\n'
    )


# ---------------------------------------------------------------------------
# Schematic
# ---------------------------------------------------------------------------


def _eff(justify: str = "", hide: bool = False) -> str:
    j = f" (justify {justify})" if justify else ""
    h = " (hide yes)" if hide else ""
    return f"(effects (font (size 1.27 1.27)){j}{h})"


def _prop(name, value, x, y, hide=False, rot=0):
    return f'\t\t(property "{name}" "{value}" (at {x:.2f} {y:.2f} {rot}) {_eff(hide=hide)})\n'


def pin_net_map() -> dict[tuple[str, str], str]:
    out = {}
    for net, nodes in NETS.items():
        for node in nodes:
            if node in out:
                raise ValueError(f"{node} on two nets")
            out[node] = net
    return out


def write_schematic() -> None:
    netmap = pin_net_map()
    lib_ids = sorted({p.lib_id for p in PARTS} | {"power:PWR_FLAG"})
    out = [
        '(kicad_sch\n\t(version 20250114)\n\t(generator "eeschema")\n'
        '\t(generator_version "9.0")\n'
        f'\t(uuid "{ROOT_UUID}")\n\t(paper "A4")\n'
        '\t(title_block (title "IWR6843LEVM J1 -> Raspberry Pi 5 SPI adapter") '
        '(rev "A") (company "OpenFlight") '
        '(comment 1 "Set IWR6843LEVM switch S1.4 OFF (SPI, not CAN)") '
        '(comment 2 "Buffers: Ioff partial-power-down protects both sides"))\n'
        "\t(lib_symbols\n"
    ]
    for lid in lib_ids:
        out.append("\t\t" + lib_symbol_text(lid).replace("\n", "\n\t") + "\n")
    out.append("\t)\n")

    labels = []
    no_connects = []
    seen_nodes = set()
    for part in PARTS:
        pins = lib_pins(part.lib_id)
        for unit in range(1, part.units + 1):
            sx, sy = part.sch_at[unit - 1]
            sym_uuid = uid(f"sym-{part.ref}-{unit}")
            unit_pins = [p for p in pins if p.unit in (0, unit)]
            out.append(
                f'\t(symbol (lib_id "{part.lib_id}") (at {sx:.2f} {sy:.2f} 0) (unit {unit})\n'
                "\t\t(exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no)\n"
                f'\t\t(uuid "{sym_uuid}")\n'
                + _prop("Reference", part.ref, sx, sy - 3.0 if part.units == 1 else sy - 6.0)
                + _prop("Value", part.value, sx, sy + 3.0 if part.units == 1 else sy + 6.0)
                + _prop("Footprint", part.footprint, sx, sy, hide=True)
                + _prop("Datasheet", part.datasheet, sx, sy, hide=True)
                + _prop("MPN", part.mpn, sx, sy, hide=True)
            )
            for p in unit_pins:
                out.append(f'\t\t(pin "{p.number}" (uuid "{uid(f"pin-{part.ref}-{p.number}")}"))\n')
            out.append(
                f'\t\t(instances (project "{PROJECT}" (path "/{ROOT_UUID}" '
                f'(reference "{part.ref}") (unit {unit}))))\n\t)\n'
            )
            for p in unit_pins:
                node = (part.ref, p.number)
                px, py = sx + p.x, sy - p.y
                # Label reads away from the symbol body.
                ang = {0: 180, 180: 0, 90: 270, 270: 90}[p.rot]
                if node in netmap:
                    labels.append((netmap[node], px, py, ang))
                    seen_nodes.add(node)
                elif node in NO_CONNECT:
                    no_connects.append((px, py))
                    seen_nodes.add(node)
                else:
                    raise ValueError(f"pin {node} is neither netted nor no-connect")

    flag_pin = lib_pins("power:PWR_FLAG")[0]
    for i, net in enumerate(POWER_FLAG_NETS):
        fx, fy = 132.08 + 20.32 * i, 190.5
        out.append(
            f'\t(symbol (lib_id "power:PWR_FLAG") (at {fx:.2f} {fy:.2f} 0) (unit 1)\n'
            "\t\t(exclude_from_sim no) (in_bom no) (on_board yes) (dnp no)\n"
            f'\t\t(uuid "{uid("flag-" + net)}")\n'
            + _prop("Reference", f"#FLG0{i + 1}", fx, fy - 5.0, hide=True)
            + _prop("Value", "PWR_FLAG", fx, fy - 3.0)
            + _prop("Footprint", "", fx, fy, hide=True)
            + _prop("Datasheet", "", fx, fy, hide=True)
            + f'\t\t(pin "1" (uuid "{uid("flagpin-" + net)}"))\n'
            f'\t\t(instances (project "{PROJECT}" (path "/{ROOT_UUID}" '
            f'(reference "#FLG0{i + 1}") (unit 1))))\n\t)\n'
        )
        labels.append((net, fx + flag_pin.x, fy - flag_pin.y, 270))

    missing = {n for nodes in NETS.values() for n in nodes} - seen_nodes
    if missing:
        raise ValueError(f"net nodes with no schematic pin: {sorted(missing)}")

    for net, x, y, ang in labels:
        just = "right bottom" if ang in (180, 270) else "left bottom"
        out.append(
            f'\t(label "{net}" (at {x:.2f} {y:.2f} {ang}) {_eff(just)} '
            f'(uuid "{uid(f"lbl-{net}-{x}-{y}")}"))\n'
        )
    for x, y in no_connects:
        out.append(f'\t(no_connect (at {x:.2f} {y:.2f}) (uuid "{uid(f"nc-{x}-{y}")}"))\n')
    notes = [
        "Pi 5 wiring (jumpers, <=15 cm, twist each signal with its GND):",
        "J2.1 3V3->Pi pin 1 | J2.3 INTR->Pi 22 (GPIO25) | J2.5 MISO->Pi 21 (GPIO9)",
        "J2.7 MOSI->Pi 19 (GPIO10) | J2.9 SCLK->Pi 23 (GPIO11) | J2.11 CS->Pi 24 (CE0)",
        "J2 even pins GND -> Pi pins 6/9/14/20/25.  Do NOT connect LEVM J1 5V (pin 2).",
    ]
    for i, text in enumerate(notes):
        out.append(
            f'\t(text "{text}" (exclude_from_sim no) (at 20.32 {165.1 + 5.08 * i:.2f} 0) '
            f'{_eff("left bottom")} (uuid "{uid(f"note{i}")}"))\n'
        )
    out.append('\t(sheet_instances (path "/" (page "1")))\n\t(embedded_fonts no)\n)\n')
    (HERE / f"{PROJECT}.kicad_sch").write_text("".join(out))


# ---------------------------------------------------------------------------
# Project file (design rules sized for JLCPCB/PCBWay standard 2-layer)
# ---------------------------------------------------------------------------

RULES = {
    "track": 0.15,
    "clearance": 0.127,
    "via_d": 0.6,
    "via_drill": 0.3,
    "edge": 0.3,
}


def write_project() -> None:
    pro = {
        "board": {
            "design_settings": {
                "rules": {
                    "min_clearance": RULES["clearance"],
                    "min_track_width": 0.127,
                    "min_via_diameter": RULES["via_d"],
                    "min_via_annular_width": 0.13,
                    "min_through_hole_diameter": RULES["via_drill"],
                    "min_hole_to_hole": 0.25,
                    "min_hole_clearance": 0.2,
                    "min_copper_edge_clearance": RULES["edge"],
                    "solder_mask_to_copper_clearance": 0.0,
                    "min_silk_clearance": 0.0,
                    "min_text_height": 0.6,
                    "min_text_thickness": 0.1,
                },
                "track_widths": [0.0, RULES["track"], 0.3],
                "via_dimensions": [
                    {"diameter": 0.0, "drill": 0.0},
                    {"diameter": RULES["via_d"], "drill": RULES["via_drill"]},
                ],
            },
        },
        "net_settings": {
            "classes": [
                {
                    "name": "Default",
                    "clearance": RULES["clearance"],
                    "track_width": RULES["track"],
                    "via_diameter": RULES["via_d"],
                    "via_drill": RULES["via_drill"],
                    "microvia_diameter": 0.3,
                    "microvia_drill": 0.1,
                    "diff_pair_gap": 0.25,
                    "diff_pair_width": 0.2,
                    "diff_pair_via_gap": 0.25,
                    "wire_width": 6,
                    "bus_width": 12,
                    "line_style": 0,
                    "pcb_color": "rgba(0, 0, 0, 0.000)",
                    "schematic_color": "rgba(0, 0, 0, 0.000)",
                    "priority": 2147483647,
                }
            ],
            "meta": {"version": 4},
        },
        "meta": {"filename": f"{PROJECT}.kicad_pro", "version": 3},
        "schematic": {"legacy_lib_dir": "", "legacy_lib_list": []},
        "sheets": [[ROOT_UUID, "Root"]],
    }
    (HERE / f"{PROJECT}.kicad_pro").write_text(json.dumps(pro, indent=2) + "\n")


def write_pcbway_bom() -> None:
    """Turnkey BOM in PCBWay's column format (assembled parts only)."""
    import csv

    groups: dict[str, list[Part]] = {}
    for part in PARTS:
        if part.assemble:
            groups.setdefault(part.mpn, []).append(part)
    out = HERE / "fab" / "pcbway_bom.csv"
    out.parent.mkdir(exist_ok=True)
    with out.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ["Line#", "Qty", "Designator", "MPN", "Manufacturer", "Description", "Package", "Type"]
        )
        for line, (mpn, parts) in enumerate(sorted(groups.items()), start=1):
            first = parts[0]
            writer.writerow(
                [
                    line,
                    len(parts),
                    ",".join(p.ref for p in parts),
                    mpn,
                    first.manufacturer,
                    first.description,
                    first.footprint.split(":")[1],
                    "SMD",
                ]
            )


if __name__ == "__main__":
    import sys

    write_footprints()
    write_project()
    write_schematic()
    write_pcbway_bom()
    if "--sch-only" not in sys.argv:
        from build_pcb import build_pcb  # noqa: E402  (needs KiCad's pcbnew)

        build_pcb()
    print("generated", HERE)
