#!/usr/bin/env python3
"""
Manufacturing data for JLCPCB from a board (KiCad 10, kicad-cli):

    python3 fab.py carrier

  <board>/fab/gerber.zip    Gerber + drill files (2 layers)
  <board>/fab/bom_jlc.csv   bill of materials for assembly
  <board>/fab/cpl_jlc.csv   component placement list for JLCPCB

Parts without an LCSC number (e.g. the H2-Zero rows J6/J7) and parts marked
DNP are not included; they are soldered by hand.
"""
import csv
import glob
import os
import subprocess
import sys
import tempfile
import zipfile

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))

if len(sys.argv) != 2 or not glob.glob(os.path.join(HERE, sys.argv[1], "*.kicad_pcb")):
    sys.exit("usage: python3 fab.py <board directory>, e.g. carrier")
BOARD = glob.glob(os.path.join(HERE, sys.argv[1], "*.kicad_pcb"))[0]
FAB   = os.path.join(HERE, sys.argv[1], "fab")
os.makedirs(FAB, exist_ok=True)

# Gerber + drill files
with tempfile.TemporaryDirectory() as tmp:
    subprocess.run(["kicad-cli", "pcb", "export", "gerbers", "--output", tmp + "/",
                    "--layers", "F.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts",
                    "--subtract-soldermask", BOARD], check=True, capture_output=True)
    subprocess.run(["kicad-cli", "pcb", "export", "drill", "--output", tmp + "/",
                    "--format", "excellon", "--excellon-separate-th", BOARD], check=True, capture_output=True)
    zpath = os.path.join(FAB, "gerber.zip")
    with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted(os.listdir(tmp)):
            z.write(os.path.join(tmp, f), f)
    print("Gerber:", zpath, sorted(zipfile.ZipFile(zpath).namelist()))

# BOM + CPL straight from the board (LCSC field of the footprints)
board = pcbnew.LoadBoard(BOARD)
groups = {}
cpl = []
for fp in board.GetFootprints():
    lcsc = fp.GetFieldText("LCSC") if fp.HasField("LCSC") else ""
    if not lcsc or fp.IsDNP():
        continue
    ref  = fp.GetReference()
    name = fp.GetFPID().GetLibItemName().wx_str()
    groups.setdefault((fp.GetValue(), name, lcsc), []).append(ref)
    pos = fp.GetPosition()
    cpl.append([ref, f"{pcbnew.ToMM(pos.x):.3f}mm", f"{-pcbnew.ToMM(pos.y):.3f}mm",
                "Bottom" if fp.IsFlipped() else "Top", f"{fp.GetOrientationDegrees():.0f}"])

with open(os.path.join(FAB, "bom_jlc.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
    for (value, name, lcsc), refs in sorted(groups.items(), key=lambda kv: sorted(kv[1])[0]):
        w.writerow([value, ",".join(sorted(refs)), name, lcsc])

with open(os.path.join(FAB, "cpl_jlc.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
    for row in sorted(cpl):
        w.writerow(row)

print(open(os.path.join(FAB, "bom_jlc.csv")).read())
print(open(os.path.join(FAB, "cpl_jlc.csv")).read())
