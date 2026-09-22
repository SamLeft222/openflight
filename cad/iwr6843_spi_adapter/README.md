# IWR6843LEVM → Raspberry Pi 5 SPI adapter

Small 2-layer board (34.0 × 25.9 mm) that plugs into the IWR6843LEVM's 60-pin
**J1** (MMWAVEICBOOST) connector and brings the radar's SPI-A port out to a
0.1" header for jumper wires to the Pi 5. The goal is to move the ~733 KB
wide-profile dump over SPI (~0.6 s at 10 MHz) instead of the ~1 Mbaud CP2105
UART (~7 s).

> **Status: rev A, not yet built or tested.** ERC, DRC and schematic–PCB
> parity pass. The firmware SPI path and Pi driver do not exist yet.

## What's on the board

| Ref | Part | Job |
|---|---|---|
| J1 | Samtec **QTH-030-01-L-D-A** (bottom side) | Mates the LEVM's J1 (QSH-030-01-L-D-A), 5.00 mm stack |
| J2 | 1×11 2.54 mm header (top side) | Jumpers to the Pi 5 |
| U1, U2 | SN74LVC2G34DBVR, powered by **radar 3V3** | Pi → radar: SCLK, MOSI, CS (U2 ch1 spare, input grounded) |
| U3 | SN74LVC2G34DBVR, powered by **Pi 3V3** | Radar → Pi: MISO, HOST_INTR |
| R1 | 100k to radar 3V3 | CS inactive (high) while the Pi pin is floating |
| R2–R5 | 100k to GND | No floating buffer inputs (SCLK, MOSI, MISO, INTR) |
| R6, R7 | 33 Ω | Damp ringing on the INTR/MISO jumpers |
| C1–C3 | 100 nF | Buffer decoupling (C3 is on the bottom) |

**Why buffers:** the IWR6843's digital I/O is not failsafe (TI SWRU585: do not
drive the pins without VIO present). The LVC2G34 has Ioff partial-power-down
(≤10 µA leakage at VCC = 0), so whichever side loses power, its buffer goes
inert and nothing back-feeds the unpowered chip.

## Before ordering (can't be checked in software)

1. **Mating orientation.** The footprint assumes QTH pin *n* mates LEVM J1
   (QSH) pin *n*, with the −A alignment pins on the row-01 side (per the
   Samtec land pattern). Print `fab/gerbers/*B_Cu*` at 1:1, or check a
   Samtec mated-pair drawing, and confirm pin 1 against the LEVM's J1
   silkscreen.
2. **Clearance behind the LEVM.** The adapter sits 5 mm under the LEVM's
   bottom side. The only bottom-side parts are J1, C3 (0.9 mm) and **J2's
   pin tails, which must be trimmed flush (≤1 mm)** so they can't touch the
   LEVM's bottom components (J6 is nearby).

## Ordering and assembly (PCBWay turnkey)

Run `./export_fab.sh` (regenerates, runs ERC/DRC, and exports into `fab/`):

| Upload | File |
|---|---|
| Gerbers + drills | `fab/iwr6843_spi_adapter-gerbers.zip` |
| BOM (PCBWay columns, MPNs) | `fab/pcbway_bom.csv` |
| Centroid / CPL | `fab/pcbway_cpl.csv` (origin = board lower-left, J2 excluded) |
| Assembly drawings | `fab/assembly/assembly_top.pdf`, `fab/assembly/assembly_bottom_mirrored.pdf` |

PCB options: 2 layers, FR-4, 1.6 mm, 1 oz, **ENIG** (flat pads for the
0.5 mm-pitch J1; HASL is too uneven), min track/spacing 5/5 mil, min hole
0.3 mm, tented vias, any mask colour.

Assembly: turnkey, **both sides** (J1 and C3 on the bottom, the rest on top).
J2 is not in the BOM or CPL. Solder it yourself and trim the tails flush.
Notes to include with the order:

> J1 (Samtec QTH-030-01-L-D-A) is on the BOTTOM side; its -A alignment pins go
> into the two 1.02 mm NPTH holes; pin 1 is marked on the bottom assembly
> drawing. U1-U3 pin 1 per the silkscreen dot. J2 (1x11 header) is NOT
> assembled. Please send placement confirmation images before production.

## LEVM setup

* Set switch **S1.4 OFF**. This routes SPI-A through the on-board TS3A5018
  mux to J1 instead of the CAN transceivers (PROC116A sheet 13). OpenFlight
  does not use CAN. TI's "functional mode" table leaves S1.4 ON; that is
  the only change.
* Keep powering the LEVM over USB as today. Do not connect J1 pin 2 (5V).

## Pi 5 wiring (jumpers ≤15 cm, cable-tie each signal to its GND)

| Adapter J2 | Signal | Pi 5 pin |
|---|---|---|
| 1 | 3V3 (powers U3) | 1 |
| 2, 4, 6, 8, 10 | GND | 6, 9, 14, 20, 25 |
| 3 | HOST_INTR (data ready) | 22 (GPIO25) |
| 5 | MISO | 21 (GPIO9) |
| 7 | MOSI | 19 (GPIO10) |
| 9 | SCLK | 23 (GPIO11) |
| 11 | CS | 24 (GPIO8 / CE0) |

None of these clash with the OPS243 UART wiring (Pi pins 2/4, 6, 8, 10, 11).
Enable SPI with `dtparam=spi=on`. Start at ≤10 MHz on jumpers.

## First power-up checks

1. Adapter only, unplugged from the LEVM: continuity between J2 even pins and
   J1 GND. No short between Pi 3V3, radar 3V3 and GND.
2. Plugged in, **everything off**: beep from J1-side nets to the LEVM's 0 Ω
   links: CS → R7, SCLK → R9, MOSI → R11, MISO → R12, INTR → R10
   (PROC116A sheet 8). This confirms the mating orientation.
3. Power the LEVM only: radar 3V3 is present at C1/C2 and U3 stays unpowered.
   Then the Pi only, then both.

## Sources

| Fact | Source |
|---|---|
| J1 pins 19/21/23/25 = SPI CS/CLK/MOSI/MISO, 16 = HOST_INTR, 4/6 = 3V3, even 20–50 = GND | TI PROC116A schematic, sheets 5, 8, 13 |
| VIOIN = 3.3 V, SPI-A behind TS3A5018, S1.4 selects SPI/CAN | PROC116A sheets 5 and 13; SWRU585 table 5-1 |
| J1 = QSH-030-01-L-D-A | PROC116A BOM |
| QTH land pattern, 5.00 mm mated height | Samtec QTH footprint Rev D, QTH print table 1 |
| LVC2G34 pinout, Ioff, DBV0006A land pattern | TI SN74LVC2G34 datasheet |

## Regenerating

Everything is generated from `generate.py` (schematic, footprints, project)
and `build_pcb.py` (placement, routing, pours). `NETS`/`PARTS` are the single
source of truth for both. Edit those, never the KiCad files.

```bash
K=/Applications/KiCad/KiCad.app/Contents
$K/Frameworks/Python.framework/Versions/Current/bin/python3 generate.py
$K/MacOS/kicad-cli sch erc --severity-all iwr6843_spi_adapter.kicad_sch
$K/MacOS/kicad-cli pcb drc --schematic-parity --refill-zones --severity-all iwr6843_spi_adapter.kicad_pcb
```

Expect only `lib_symbol_issues` / `lib_footprint_issues` warnings if your
KiCad has no global library tables configured. They are environment notices,
not design errors.
