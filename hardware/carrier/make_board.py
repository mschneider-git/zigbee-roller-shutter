#!/usr/bin/env python3
"""
Generates roller_carrier.kicad_pcb: carrier board for a Waveshare ESP32-H2-Zero (KiCad 10,
pcbnew API). The H2-Zero is soldered into two 1x9 rows on header spacers; USB-C, BOOT and
RESET are on the H2-Zero itself.

Mechanics (all dimensions in mm, origin = top left corner of the original board, y down):
  - original board 34 x 46, relay x 12..24 / y 12..24, 13.5 mm high
  - our board sits ~11 mm high (8.5 mm socket J3 on the keypad header at
    x = 29, y = 32.46 / 35 / 37.54), U-shaped around the relay, extending 7.5 mm to the
    left of the original board (housing is 41.5 mm wide inside)
  - H2-Zero on the left, USB-C up, ceramic antenna down at the board edge 40.6 with no
    copper below it. Regulator and dividers sit below the module.

Circuit see README.md. Battery- is not on this board; GND only comes via J3 (keypad GND).

    python3 make_board.py [--pads]
"""
import os
import sys
import tempfile

import pcbnew

HERE   = os.path.dirname(os.path.abspath(__file__))
KILIB  = "/usr/share/kicad/footprints"
PROJ   = os.path.join(HERE, "..", "lib", "footprints.pretty")
OUT    = os.path.join(HERE, "roller_carrier.kicad_pcb")
ORIGIN = (120.0, 80.0)

OUTLINE = [(-7.5, 11.5), (11.5, 11.5), (11.5, 24.5), (24.5, 24.5), (24.5, 11.5), (34.0, 11.5),
           (34.0, 40.6), (-7.5, 40.6)]

# H2-Zero (18 x 23.5 mm): top left corner; pin rows at x + 1.38 and x + 16.62, from y + 1.59
ZERO_X, ZERO_Y = -7.0, 17.0
ROW_L, ROW_R   = ZERO_X + 1.38, ZERO_X + 16.62
PIN_Y0         = ZERO_Y + 1.59


def P(x, y):
    return pcbnew.VECTOR2I(pcbnew.FromMM(ORIGIN[0] + x), pcbnew.FromMM(ORIGIN[1] + y))


def mm(v):
    return pcbnew.ToMM(v)


# NewBoard() creates the file right away; use a throwaway file for --pads so the routed board survives
board = pcbnew.NewBoard(OUT if "--pads" not in sys.argv else os.path.join(tempfile.mkdtemp(), "pads.kicad_pcb"))
nets  = {}


def net(name):
    if name not in nets:
        n = pcbnew.NETINFO_ITEM(board, name)
        board.Add(n)
        nets[name] = n
    return nets[name]


FP = {}


def place(ref, lib, name, value, x, y, rot=0, bottom=False, lcsc=None, dnp=False, pads=None):
    libpath = lib if lib.startswith("/") or lib == PROJ else os.path.join(KILIB, lib + ".pretty")
    fp = pcbnew.FootprintLoad(libpath, name)
    if fp is None:
        sys.exit(f"Footprint {lib}:{name} not found")
    fp.SetFPID(pcbnew.LIB_ID(os.path.basename(libpath).removesuffix(".pretty"), name))
    fp.SetReference(ref)
    fp.SetValue(value)
    fp.SetPosition(P(x, y))
    fp.SetOrientationDegrees(rot)
    board.Add(fp)
    if bottom:
        fp.Flip(P(x, y), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
    if lcsc:
        fp.SetField("LCSC", lcsc)
        fp.GetField("LCSC").SetVisible(False)
    if dnp:
        fp.SetDNP(True)
        fp.SetExcludedFromBOM(True)
        fp.SetExcludedFromPosFiles(True)
    for p in fp.Pads():
        if pads and p.GetNumber() in pads:
            p.SetNet(net(pads[p.GetNumber()]))
    FP[ref] = fp
    return fp


R0603, C0603, C0805 = ("Resistor_SMD", "R_0603_1608Metric"), ("Capacitor_SMD", "C_0603_1608Metric"), \
    ("Capacitor_SMD", "C_0805_2012Metric")
SOCKET = ("Connector_PinSocket_2.54mm", "PinSocket_1x09_P2.54mm_Vertical")

# --------------------------------------------------------------------------
# H2-Zero in two 1x9 rows. Left from the top: 5V, GND, 3V3, IO0..IO5;
# right from the top: TX, RX, IO25, IO22, IO14, IO13, IO12, IO11, IO10.
# This matches the firmware defaults.
# 5V stays open: the H2-Zero is powered via 3V3.
# The H2-Zero is soldered directly into these holes on spacers (the plastic strip of a pin
# header) so the regulator and dividers fit below it. JLCPCB does not fit J6/J7.
# --------------------------------------------------------------------------
place("J6", *SOCKET, "H2-Zero soldered left", ROW_L, PIN_Y0,
      pads={"2": "GND", "3": "+3V3", "5": "ADC_MM", "6": "ADC_SOL", "8": "ADC_BAT", "9": "ADC_MP"})
place("J7", *SOCKET, "H2-Zero soldered right", ROW_R, PIN_Y0,
      pads={"5": "XTAL_N", "6": "XTAL_P", "8": "IO11", "9": "IO10"})
# No socket is fitted: drop the socket 3D model; J6 carries the model of the H2-Zero instead
for ref in ("J6", "J7"):
    FP[ref].Models().clear()
h2_model = pcbnew.FP_3DMODEL()
h2_model.m_Filename = "${KIPRJMOD}/../lib/footprints.3dshapes/Waveshare_ESP32-H2-Zero.wrl"
FP["J6"].Models().append(h2_model)

# --------------------------------------------------------------------------
# Battery+ looped through: WAGO 2060-452 in the right arm, wire entry upwards
# --------------------------------------------------------------------------
place("J1", PROJ, "WAGO_2060-452", "WAGO 2060-452 battery+", 29.75, 19.5, rot=270,
      lcsc="C2765055", pads={"1": "VBAT_RAW", "2": "VBAT_RAW"})
# Fuse -> reverse polarity protection (P-MOSFET, drain at the fuse) below the terminal
place("F1", "Fuse", "Fuse_1206_3216Metric", "PTC 100mA", 27.0, 27.3, rot=0, lcsc="C70065",
      pads={"1": "VBAT_RAW", "2": "VBAT_F"})
place("Q1", "Package_TO_SOT_SMD", "SOT-23", "AO3407A", 31.8, 27.6, rot=0, lcsc="C15155",
      pads={"1": "GATE", "2": "VBAT", "3": "VBAT_F"})
place("R5", *R0603, "100k", 27.0, 29.4, rot=0, lcsc="C25803", pads={"1": "GATE", "2": "GND"})

# --------------------------------------------------------------------------
# Regulator below the upper part of the H2-Zero: 10 Ohm, TVS, TPS629203
# (route.py lays SW, VIN/GND at C1 and VOS by hand)
# --------------------------------------------------------------------------
place("C1", *C0805, "10u 25V", 3.8, 18.8, rot=90, lcsc="C15850", pads={"1": "GND", "2": "VIN"})
place("R1", *R0603, "10", -2.4, 18.6, rot=90, lcsc="C22859", pads={"1": "VIN", "2": "VBAT"})
place("D1", "Diode_SMD", "D_SOD-123F", "SMF13A", -1.9, 22.6, rot=90, lcsc="C353315",
      pads={"1": "VIN", "2": "GND"})
place("U1", "Package_TO_SOT_SMD", "SOT-583-8", "TPS629203DRLR", 1.5, 18.8, rot=0, lcsc="C5219295",
      pads={"3": "+3V3", "4": "SW", "5": "GND", "6": "VIN", "7": "VIN", "8": "MODE"})
place("R2", *R0603, "64.9k", -0.9, 18.6, rot=90, lcsc="C2960807", pads={"1": "MODE", "2": "GND"})
place("L1", "Inductor_SMD", "L_1008_2520Metric", "2.2u", 1.5, 23.3, rot=0, lcsc="C237482",
      pads={"1": "SW", "2": "+3V3"})
place("C2", *C0805, "22u", 4.9, 22.9, rot=90, lcsc="C784585", pads={"1": "+3V3", "2": "GND"})

# --------------------------------------------------------------------------
# Below the H2-Zero: decoupling at the 3V3 pin, per channel 1 MOhm / 220 kOhm / 100 nF
# (rows from the top: motor-, solar, battery, motor+, like the ADC pins on the left)
# --------------------------------------------------------------------------
place("C4", *C0805, "10u", -2.1, 26.2, rot=180, lcsc="C15850", pads={"1": "+3V3", "2": "GND"})
place("C5", *C0603, "100n", 1.3, 26.2, rot=0, lcsc="C14663", pads={"1": "+3V3", "2": "GND"})

# Each row from the left: capacitor, bottom resistor, top resistor (input on the right so the
# lines from J5 come down in the channel between the regulator and the right row)
DIV_X = (4.1, 1.0, -2.1)  # top resistor, bottom resistor, capacitor
for i, (ch, src, r_top, r_bot, cap) in enumerate((
        ("ADC_MM", "MM", "R16", "R17", "C9"),
        ("ADC_SOL", "SOL", "R18", "R19", "C10"),
        ("ADC_BAT", "VBAT", "R12", "R13", "C7"),
        ("ADC_MP", "MP", "R14", "R15", "C8"))):
    y = 27.95 + i * 1.6
    place(r_top, *R0603, "1M", DIV_X[0], y, rot=180, lcsc="C22935", pads={"1": src, "2": ch})
    place(r_bot, *R0603, "220k", DIV_X[1], y, rot=0, lcsc="C22961", pads={"1": ch, "2": "GND"})
    place(cap, *C0603, "100n", DIV_X[2], y, rot=0, lcsc="C14663", pads={"1": ch, "2": "GND"})

# 32 kHz crystal to the right of the right row (IO13/IO14 = pin 6/5)
place("Y1", "Crystal", "Crystal_SMD_3215-2Pin_3.2x1.5mm", "32.768k", 14.0, 26.1, rot=0,
      lcsc="C620155", pads={"1": "XTAL_P", "2": "XTAL_N"})
# Crystal CL 12.5 pF: 2 x 18 pF in series + about 2.5 pF stray capacitance = 11.5 pF
place("C11", *C0603, "18p", 12.4, 28.9, rot=90, lcsc="C1647", pads={"1": "XTAL_P", "2": "GND"})
place("C12", *C0603, "18p", 14.2, 28.9, rot=90, lcsc="C1647", pads={"1": "XTAL_N", "2": "GND"})

# --------------------------------------------------------------------------
# Centre: series resistors for DOWN/UP
# --------------------------------------------------------------------------
place("R8", *R0603, "1k", 20.9, 29.6, rot=0, lcsc="C21190", pads={"1": "IO11", "2": "DOWN"})
place("R9", *R0603, "1k", 23.9, 29.6, rot=0, lcsc="C21190", pads={"1": "IO10", "2": "UP"})

# Measurement header in the left arm, pins pointing up to the screw terminals: SOL, M+, M-
place("J5", "Connector_PinHeader_2.54mm", "PinHeader_1x03_P2.54mm_Horizontal", "Solar M+ M-",
      4.2, 14.4, rot=90, lcsc="C92159", pads={"1": "SOL", "2": "MP", "3": "MM"})

# --------------------------------------------------------------------------
# Keypad: socket on the bottom onto the original header, 90 degree header on top towards the centre
# --------------------------------------------------------------------------
place("J3", "Connector_PinSocket_2.54mm", "PinSocket_1x03_P2.54mm_Vertical", "to keypad header",
      29.0, 32.46, bottom=True, lcsc="C42431866", pads={"1": "GND", "2": "DOWN", "3": "UP"})
place("J4", "Connector_PinHeader_2.54mm", "PinHeader_1x03_P2.54mm_Horizontal", "Membrane keypad",
      25.0, 37.54, rot=180, lcsc="C92159", pads={"3": "GND", "2": "DOWN", "1": "UP"})  # GND on top as on the original

# --------------------------------------------------------------------------
# Ceramic antenna of the H2-Zero (x + 3.4..14.2, y + 19.6..23.5): no copper below it.
# On the left this leaves a channel for the IO5 line (J6 pin 9).
# --------------------------------------------------------------------------
ka = pcbnew.ZONE(board)
ka.SetIsRuleArea(True)
ka_layers = pcbnew.LSET()
ka_layers.AddLayer(pcbnew.F_Cu)
ka_layers.AddLayer(pcbnew.B_Cu)
ka.SetLayerSet(ka_layers)
ka.SetDoNotAllowTracks(True)
ka.SetDoNotAllowVias(True)
ka.SetDoNotAllowZoneFills(True)
ka.SetDoNotAllowPads(False)
ka.SetDoNotAllowFootprints(False)
ka.SetZoneName("Antenna")
outline = ka.Outline()
outline.NewOutline()
for x, y in ((ZERO_X + 3.4, ZERO_Y + 19.6), (ZERO_X + 14.2, ZERO_Y + 19.6),
             (ZERO_X + 14.2, 40.6), (ZERO_X + 3.4, 40.6)):
    outline.Append(P(x, y))
board.Add(ka)

# --------------------------------------------------------------------------
# Silkscreen: references on the Fab layer; the silkscreen only names connections and the H2-Zero orientation
# --------------------------------------------------------------------------
for fp in FP.values():
    fp.Reference().SetLayer(pcbnew.B_Fab if fp.IsFlipped() else pcbnew.F_Fab)


def label(text, x, y, size=0.8, bottom=False):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(text)
    t.SetPosition(P(x, y))
    t.SetLayer(pcbnew.B_SilkS if bottom else pcbnew.F_SilkS)
    t.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(size), pcbnew.FromMM(size)))
    t.SetTextThickness(pcbnew.FromMM(size * 0.15))
    t.SetMirrored(bottom)
    board.Add(t)


for (x, txt) in ((4.2, "SOL"), (6.74, "M+"), (9.28, "M-")):        # below J5, no room on top
    label(txt, x, 16.5, bottom=True)
for (y, txt) in ((32.46, "GND"), (35.0, "DN"), (37.54, "UP")):     # next to J3/J4, both sides
    label(txt, 31.7, y)
    label(txt, 31.7, y, bottom=True)
label("USB", -3.0, 15.9)                                            # H2-Zero orientation
label("BAT+", 29.75, 19.5, size=1.2, bottom=True)                   # the WAGO covers the top side
label("Roller-H2 Zero v2", 18.0, 26.6, size=0.9, bottom=True)

if "--pads" in sys.argv:
    for ref, fp in sorted(FP.items()):
        bb = fp.GetBoundingBox(False)
        print(f"{ref:4} x {mm(bb.GetX())-ORIGIN[0]:6.2f}..{mm(bb.GetRight())-ORIGIN[0]:6.2f}"
              f"  y {mm(bb.GetY())-ORIGIN[1]:6.2f}..{mm(bb.GetBottom())-ORIGIN[1]:6.2f}")
    sys.exit(0)

# Outline
for (x1, y1), (x2, y2) in zip(OUTLINE, OUTLINE[1:] + OUTLINE[:1]):
    s = pcbnew.PCB_SHAPE(board)
    s.SetShape(pcbnew.SHAPE_T_SEGMENT)
    s.SetStart(P(x1, y1))
    s.SetEnd(P(x2, y2))
    s.SetLayer(pcbnew.Edge_Cuts)
    s.SetWidth(pcbnew.FromMM(0.1))
    board.Add(s)

# Outline of the H2-Zero on the Fab layer (to check its position)
for (x1, y1), (x2, y2) in (((0, 0), (18, 0)), ((18, 0), (18, 23.5)), ((18, 23.5), (0, 23.5)), ((0, 23.5), (0, 0))):
    s = pcbnew.PCB_SHAPE(board)
    s.SetShape(pcbnew.SHAPE_T_SEGMENT)
    s.SetStart(P(ZERO_X + x1, ZERO_Y + y1))
    s.SetEnd(P(ZERO_X + x2, ZERO_Y + y2))
    s.SetLayer(pcbnew.F_Fab)
    s.SetWidth(pcbnew.FromMM(0.1))
    board.Add(s)

ds = board.GetDesignSettings()
ds.m_MinClearance        = pcbnew.FromMM(0.127)
ds.m_CopperEdgeClearance = pcbnew.FromMM(0.25)
ds.m_HoleClearance       = pcbnew.FromMM(0.25)
ds.m_TrackMinWidth       = pcbnew.FromMM(0.15)   # Freerouting rounds 0.127 down to 0.12 when necking
ds.m_ViasMinSize         = pcbnew.FromMM(0.45)
ds.m_MinThroughDrill     = pcbnew.FromMM(0.3)
nc = ds.m_NetSettings.GetDefaultNetclass()
nc.SetClearance(pcbnew.FromMM(0.15))
nc.SetTrackWidth(pcbnew.FromMM(0.2))
nc.SetViaDiameter(pcbnew.FromMM(0.5))
nc.SetViaDrill(pcbnew.FromMM(0.3))

board.Save(OUT)
print("saved:", OUT)
