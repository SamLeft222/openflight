"""PCB half of the adapter generator (needs KiCad's pcbnew module).

Placement (top view, mm; board origin at (100, 100)):

* J1 (Samtec QTH) on the BOTTOM side, mating the LEVM's J1 underneath.
* J2 (1x11 header), U1-U3 and passives on the TOP side, away from the LEVM.
* Bottom copper under J1 is used only for the short fan-out from the 0.5 mm
  pitch pads to vias; both layers are otherwise GND pour.
"""

from __future__ import annotations

import pcbnew
from generate import FP_DIR, FP_LIB, HERE, NETS, NO_CONNECT, PARTS, PROJECT, RULES, uid

MM = pcbnew.FromMM
BOARD_X = (100.0, 134.0)
BOARD_Y = (97.1, 123.0)
HDR_Y = 100.8

# (x, y, rotation_deg, bottom?)  -- see module docstring for the layout idea.
J1_AT = (117.0, 117.5)
PLACEMENT = {
    "J1": (*J1_AT, 0, True),
    "J2": (104.3, HDR_Y, 90, False),  # pin 1 at left, pins run +x
    "U3": (111.9, 108.0, 90, False),  # radar -> Pi: inputs face J1 (down)
    "U1": (122.08, 108.0, 270, False),  # Pi -> radar: inputs face J2 (up)
    "U2": (129.0, 108.0, 270, False),
    "R6": (109.38, 104.2, 90, False),  # 33R INTR, in line under J2.3
    "R7": (114.46, 104.2, 90, False),  # 33R MISO, in line under J2.5
    "R3": (120.81, 104.2, 90, False),  # 100k MOSI pull-down
    "R2": (125.89, 104.2, 90, False),  # 100k SCLK pull-down
    "R1": (131.9, 104.2, 90, False),  # 100k CS pull-up (radar 3V3)
    "R4": (114.5, 111.4, 270, False),  # 100k RADAR_MISO pull-down (pad 1 up)
    "R5": (109.3, 111.4, 270, False),  # 100k RADAR_INTR pull-down (pad 1 up)
    "C1": (122.08, 112.2, 270, False),  # pad 1 (3V3) toward U1 VCC
    "C2": (129.0, 112.2, 270, False),
    "C3": (111.9, 104.2, 0, True),  # bottom, beside U3's PI_3V3 via
}

# Via positions (x, y) by net.  J1 fan-out vias sit directly under the buffer
# pins they feed; J1 inner-row vias sit between pad row 02 and the GND bar.
J1_INNER_VIA_Y = 118.85
FANOUT_VIA_Y = 110.6
VIAS = [
    ("RADAR_MISO", 112.85, FANOUT_VIA_Y),
    ("RADAR_MOSI", 121.13, FANOUT_VIA_Y),
    ("RADAR_SCLK", 123.03, FANOUT_VIA_Y),
    ("RADAR_CS", 128.05, FANOUT_VIA_Y),
    ("RADAR_INTR", 120.75, J1_INNER_VIA_Y),
    ("RADAR_3V3", 123.5, J1_INNER_VIA_Y),
    ("PI_3V3", 111.9, 105.45),
    ("GND", 111.9, FANOUT_VIA_Y),  # U3 GND
    ("GND", 130.95, 106.7),  # U2 GND + spare input
    # GND stitching in open areas of both pours.
    ("GND", 101.2, 104.5),
    ("GND", 101.2, 121.8),
    ("GND", 132.9, 121.8),
    ("GND", 132.9, 113.5),
    ("GND", 106.0, 110.0),
    ("GND", 116.5, 107.0),
    ("GND", 126.0, 108.0),
    ("GND", 104.5, 118.0),
    ("GND", 118.0, 104.0),
]

F, B = "F.Cu", "B.Cu"
FINE = 0.15  # at the 0.5 mm pitch J1 fan-out
SIG = 0.2
PWR = 0.3
# (net, layer, width, points)
ROUTES = [
    # --- Pi side, top layer -------------------------------------------------
    ("PI_INTR", F, SIG, [(109.38, HDR_Y), (109.38, 103.375)]),
    ("PI_MISO", F, SIG, [(114.46, HDR_Y), (114.46, 103.375)]),
    ("INTR_BUF", F, SIG, [(109.38, 105.025), (110.95, 106.7)]),
    ("MISO_BUF", F, SIG, [(114.46, 105.025), (112.85, 106.7)]),
    (
        "PI_MOSI",
        F,
        SIG,
        [(119.54, HDR_Y), (119.54, 104.6), (120.81, 105.025), (121.13, 106.2), (121.13, 106.7)],
    ),
    ("PI_SCLK", F, SIG, [(124.62, HDR_Y), (124.62, 104.6), (125.89, 105.025)]),
    ("PI_SCLK", F, SIG, [(124.62, 104.6), (123.03, 106.2), (123.03, 106.7)]),
    ("PI_CS", F, SIG, [(129.7, HDR_Y), (129.7, 103.375), (131.9, 103.375)]),
    ("PI_CS", F, SIG, [(129.7, 103.375), (129.7, 105.2), (128.05, 106.2), (128.05, 106.7)]),
    ("GND", F, SIG, [(122.08, 106.7), (122.08, HDR_Y)]),  # U1 GND
    ("GND", F, SIG, [(120.81, 103.375), (122.08, 103.375)]),  # R3 GND
    ("GND", F, SIG, [(125.89, 103.375), (127.16, 103.375), (127.16, HDR_Y)]),
    ("GND", F, SIG, [(129.0, 106.7), (129.95, 106.7), (130.95, 106.7)]),
    ("GND", F, SIG, [(111.9, 109.3), (111.9, 110.6)]),  # U3 GND
    ("PI_3V3", F, SIG, [(111.9, 106.7), (111.9, 105.45)]),
    ("PI_3V3", B, SIG, [(111.9, 105.45), (112.675, 104.2)]),
    # --- PI_3V3 on the bottom to J2.1 and C3 --------------------------------
    ("PI_3V3", B, PWR, [(111.9, 105.45), (105.3, 105.45), (104.3, 104.45), (104.3, HDR_Y)]),
    # --- Radar 3V3: J1 pins 4/6 -> via -> C1/U1, C2/U2, R1 (top) ------------
    ("RADAR_3V3", B, FINE, [(123.25, 120.59), (123.25, 119.35), (123.5, 118.85)]),
    ("RADAR_3V3", B, FINE, [(123.75, 120.59), (123.75, 119.35), (123.5, 118.85)]),
    (
        "RADAR_3V3",
        F,
        PWR,
        [
            (123.5, 118.85),
            (123.5, 114.2),
            (129.9, 114.2),
            (129.9, 111.425),
            (131.9, 111.425),
            (131.9, 105.025),
        ],
    ),
    ("RADAR_3V3", F, PWR, [(123.5, 114.2), (123.5, 111.425), (122.08, 111.425)]),
    ("RADAR_3V3", F, SIG, [(122.08, 111.425), (122.08, 109.3)]),
    ("RADAR_3V3", F, PWR, [(129.9, 111.425), (129.0, 111.425)]),
    ("RADAR_3V3", F, SIG, [(129.0, 111.425), (129.0, 109.3)]),
    # --- RADAR_INTR: J1.16 -> via -> top -> U3.1, R5 ------------------------
    ("RADAR_INTR", B, FINE, [(120.75, 120.59), (120.75, 118.85)]),
    ("RADAR_INTR", F, SIG, [(120.75, 118.85), (110.95, 118.85), (110.95, 109.3)]),
    ("RADAR_INTR", F, SIG, [(109.3, 110.575), (110.95, 110.575)]),
    # --- Radar SPI fan-out from J1 row 01 on the bottom, then up ------------
    ("RADAR_CS", B, FINE, [(119.75, 114.41), (119.75, 113.2), (128.05, 113.2), (128.05, 110.6)]),
    ("RADAR_SCLK", B, FINE, [(119.25, 114.41), (119.25, 112.6), (123.03, 112.6), (123.03, 110.6)]),
    ("RADAR_MOSI", B, FINE, [(118.75, 114.41), (118.75, 112.0), (121.13, 112.0), (121.13, 110.6)]),
    ("RADAR_MISO", B, FINE, [(118.25, 114.41), (118.25, 111.4), (112.85, 111.4), (112.85, 110.6)]),
    ("RADAR_CS", F, SIG, [(128.05, 110.6), (128.05, 109.3)]),
    ("RADAR_SCLK", F, SIG, [(123.03, 110.6), (123.03, 109.3)]),
    ("RADAR_MOSI", F, SIG, [(121.13, 110.6), (121.13, 109.3)]),
    ("RADAR_MISO", F, SIG, [(112.85, 110.6), (112.85, 109.3)]),
    ("RADAR_MISO", F, SIG, [(112.85, 110.6), (114.5, 110.575)]),
    # --- J1 row-02 GND pins straight onto the GND bars (bottom) -------------
    ("GND", B, FINE, [(119.75, 120.59), (119.75, 117.5)]),
    ("GND", B, FINE, [(118.25, 120.59), (118.25, 117.5)]),
    ("GND", B, FINE, [(116.75, 120.59), (116.75, 118.6), (116.175, 118.0), (116.175, 117.5)]),
    ("GND", B, FINE, [(115.25, 120.59), (115.25, 117.5)]),
    ("GND", B, FINE, [(113.75, 120.59), (113.75, 117.5)]),
    ("GND", B, FINE, [(112.25, 120.59), (112.25, 117.5)]),
]

SILK_TEXT = [
    # (text, x, y, size, layer)
    ("3V3", 104.3, 98.25, 0.7),
    ("G", 106.84, 98.25, 0.7),
    ("INT", 109.38, 98.25, 0.7),
    ("G", 111.92, 98.25, 0.7),
    ("MISO", 114.46, 98.25, 0.7),
    ("G", 117.0, 98.25, 0.7),
    ("MOSI", 119.54, 98.25, 0.7),
    ("G", 122.08, 98.25, 0.7),
    ("SCK", 124.62, 98.25, 0.7),
    ("G", 127.16, 98.25, 0.7),
    ("CS", 129.7, 98.25, 0.7),
    ("OpenFlight IWR6843LEVM->Pi5 SPI rev A", 117.0, 120.4, 0.8),
    ("LEVM S1.4 = OFF (SPI)", 117.0, 121.9, 0.8),
]


def unconnected_net_name(ref: str, pin: str) -> str:
    """KiCad's schematic name for a no-connect pin's net."""
    if ref == "J1":
        return f"unconnected-(J1-Pin_{pin}-Pad{pin})"
    return f"unconnected-({ref}-Pad{pin})"


def load_fp(fp_id: str) -> pcbnew.FOOTPRINT:
    lib, name = fp_id.split(":")
    path = HERE / f"{FP_LIB}.pretty" if lib == FP_LIB else FP_DIR / f"{lib}.pretty"
    fp = pcbnew.FootprintLoad(str(path), name)
    if fp is None:
        raise FileNotFoundError(fp_id)
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    return fp


def place(board: pcbnew.BOARD, nets: dict) -> dict[str, pcbnew.FOOTPRINT]:
    pin_to_net = {node: name for name, nodes in NETS.items() for node in nodes}
    fps = {}
    for part in PARTS:
        fp = load_fp(part.footprint)
        fp.SetReference(part.ref)
        fp.SetValue(part.value)
        fp.SetField("MPN", part.mpn)
        fp.GetField("MPN").SetVisible(False)
        fp.GetField("MPN").SetLayer(pcbnew.F_Fab)
        fp.GetField(pcbnew.FIELD_T_DATASHEET).SetText(part.datasheet)
        # Dense board: reference designators live on the fab (assembly)
        # layer.  Set before any flip so the flip carries them to B.Fab.
        fp.Reference().SetLayer(pcbnew.F_Fab)
        # Footprint <-> schematic link: root sheet + unit-1 symbol UUID.
        fp.SetPath(pcbnew.KIID_PATH(f"/{uid(f'sym-{part.ref}-1')}"))
        x, y, rot, bottom = PLACEMENT[part.ref]
        board.Add(fp)
        fp.SetPosition(pcbnew.VECTOR2I_MM(x, y))
        fp.SetOrientationDegrees(rot)
        if bottom:
            fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
        for pad in fp.Pads():
            node = (part.ref, pad.GetNumber())
            net = pin_to_net.get(node)
            if net:
                pad.SetNet(nets[net])
            elif node in NO_CONNECT:
                pad.SetNet(nets[unconnected_net_name(*node)])
        fps[part.ref] = fp
    return fps


def new_board() -> tuple[pcbnew.BOARD, dict]:
    board = pcbnew.NewBoard(str(HERE / f"{PROJECT}.kicad_pcb"))
    nets = {}
    for name in NETS:
        item = pcbnew.NETINFO_ITEM(board, f"/{name}")
        board.Add(item)
        nets[name] = item
    for ref, pin in NO_CONNECT:
        name = unconnected_net_name(ref, pin)
        item = pcbnew.NETINFO_ITEM(board, name)
        board.Add(item)
        nets[name] = item
    return board, nets


def _pt(x: float, y: float) -> pcbnew.VECTOR2I:
    return pcbnew.VECTOR2I_MM(x, y)


def add_copper(board: pcbnew.BOARD, nets: dict) -> None:
    layer = {F: pcbnew.F_Cu, B: pcbnew.B_Cu}
    for net, lyr, width, pts in ROUTES:
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(_pt(x1, y1))
            t.SetEnd(_pt(x2, y2))
            t.SetWidth(MM(width))
            t.SetLayer(layer[lyr])
            t.SetNet(nets[net])
            board.Add(t)
    for net, x, y in VIAS:
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(_pt(x, y))
        v.SetDrill(MM(RULES["via_drill"]))
        v.SetWidth(MM(RULES["via_d"]))
        v.SetNet(nets[net])
        board.Add(v)


def add_outline_and_zones(board: pcbnew.BOARD, nets: dict) -> None:
    (x0, x1), (y0, y1) = BOARD_X, BOARD_Y
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    for (ax, ay), (bx, by) in zip(corners, corners[1:] + corners[:1]):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(_pt(ax, ay))
        s.SetEnd(_pt(bx, by))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(MM(0.1))
        board.Add(s)
    for lyr in (pcbnew.F_Cu, pcbnew.B_Cu):
        z = pcbnew.ZONE(board)
        z.SetLayer(lyr)
        z.SetNet(nets["GND"])
        z.SetLocalClearance(MM(0.2))
        z.SetMinThickness(MM(0.2))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THT_THERMAL)
        z.SetThermalReliefGap(MM(0.3))
        z.SetThermalReliefSpokeWidth(MM(0.3))
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        ol = z.Outline()
        ol.NewOutline()
        inset = RULES["edge"]
        for x, y in corners:
            ol.Append(_pt(x + (inset if x == x0 else -inset), y + (inset if y == y0 else -inset)))
        board.Add(z)
    board.BuildConnectivity()
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())


def add_silk(board: pcbnew.BOARD) -> None:
    for text, x, y, size in SILK_TEXT:
        t = pcbnew.PCB_TEXT(board)
        t.SetText(text)
        t.SetPosition(_pt(x, y))
        t.SetLayer(pcbnew.F_SilkS)
        t.SetTextSize(pcbnew.VECTOR2I_MM(size, size))
        t.SetTextThickness(MM(0.15))
        board.Add(t)


def build_pcb() -> pcbnew.BOARD:
    board, nets = new_board()
    place(board, nets)
    add_copper(board, nets)
    add_outline_and_zones(board, nets)
    add_silk(board)
    # Drill/place origin at the board's lower-left corner for fab outputs.
    origin = _pt(BOARD_X[0], BOARD_Y[1])
    board.GetDesignSettings().SetAuxOrigin(origin)
    board.GetDesignSettings().SetGridOrigin(origin)
    path = HERE / f"{PROJECT}.kicad_pcb"
    pcbnew.SaveBoard(str(path), board)
    return board


def dump_pads(fps) -> None:
    for ref, fp in fps.items():
        for pad in fp.Pads():
            pos = pad.GetPosition()
            print(
                f"{ref}.{pad.GetNumber():>3} {pcbnew.ToMM(pos.x):8.3f} "
                f"{pcbnew.ToMM(pos.y):8.3f} {pad.GetNetname()}"
            )


if __name__ == "__main__":
    b, n = new_board()
    dump_pads(place(b, n))
