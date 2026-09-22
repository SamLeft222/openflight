#!/usr/bin/env bash
# Regenerate the adapter and export everything a fab/assembler needs into fab/.
# Fails if ERC or DRC (incl. schematic parity) report any error.
set -euo pipefail
cd "$(dirname "$0")"

K=/Applications/KiCad/KiCad.app/Contents
PY="$K/Frameworks/Python.framework/Versions/Current/bin/python3"
CLI="$K/MacOS/kicad-cli"
NAME=iwr6843_spi_adapter

"$PY" generate.py

"$CLI" sch erc --severity-error --exit-code-violations -o /dev/null "$NAME.kicad_sch"
"$CLI" pcb drc --schematic-parity --refill-zones --severity-error \
    --exit-code-violations -o /dev/null "$NAME.kicad_pcb"

rm -rf fab/gerbers fab/assembly
mkdir -p fab/gerbers fab/assembly
"$CLI" pcb export gerbers --use-drill-file-origin --subtract-soldermask \
    --layers "F.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts" \
    -o fab/gerbers/ "$NAME.kicad_pcb"
"$CLI" pcb export drill --format excellon --excellon-separate-th --drill-origin plot \
    --generate-map --map-format pdf -o fab/gerbers/ "$NAME.kicad_pcb"
(cd fab/gerbers && rm -f ../"$NAME"-gerbers.zip && zip -q ../"$NAME"-gerbers.zip ./*)

# Placement for assembled parts only (J2 is hand-soldered).
"$CLI" pcb export pos --side both --format csv --units mm --use-drill-file-origin \
    -o fab/positions_all.csv "$NAME.kicad_pcb"
grep -v '^"J2"' fab/positions_all.csv > fab/pcbway_cpl.csv
rm fab/positions_all.csv

"$CLI" sch export bom --fields "Reference,Value,Footprint,MPN,\${QUANTITY}" \
    --labels "Designator,Value,Footprint,MPN,Qty" --group-by "Value,Footprint,MPN" \
    -o fab/bom.csv "$NAME.kicad_sch"
"$CLI" sch export pdf -o fab/schematic.pdf "$NAME.kicad_sch"

# Assembly drawings: refdes live on the Fab layers.
"$CLI" pcb export pdf --mode-single --scale 0 --sketch-pads-on-fab-layers --exclude-value \
    --layers "F.Fab,F.Silkscreen,F.Courtyard,Edge.Cuts" \
    -o fab/assembly/assembly_top.pdf "$NAME.kicad_pcb"
"$CLI" pcb export pdf --mode-single --scale 0 --sketch-pads-on-fab-layers --exclude-value --mirror \
    --layers "B.Fab,B.Silkscreen,B.Courtyard,Edge.Cuts" \
    -o fab/assembly/assembly_bottom_mirrored.pdf "$NAME.kicad_pcb"

rm -f fab/positions.csv
echo "fab/ ready:"
ls -R fab
