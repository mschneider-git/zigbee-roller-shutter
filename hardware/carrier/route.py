#!/usr/bin/env python3
"""
Routes roller_carrier.kicad_pcb (placed by make_board.py) with Freerouting, see lib/routing.py.

    python3 route.py [path/to/freerouting.jar]   (default: $FREEROUTING_JAR)
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "lib"))
import pcbnew   # noqa: E402
import routing  # noqa: E402

def hand_routes(board):
    t = routing.Tracks(board)
    # TPS629203 with C1/C2/L1: short SW, VIN/GND at C1, VOS directly to the output capacitor C2
    t.track("SW", 0.3, [(0.76, 19.55), (0.76, 20)])  # U1.4 -> L1.1, narrow at the pin
    t.track("SW", 0.4, [(0.76, 20), (0.76, 22.7)])
    t.track("+3V3", 0.6, [(2.57, 23.3), (2.57, 23.85), (4.9, 23.85)])  # L1.2 -> C2.1
    t.track("VIN", 0.3, [(2.24, 19.05), (2.24, 18.55), (2.9, 18.55), (3.3, 18.15), (3.8, 17.85)])  # U1.6/7 -> C1.2
    t.track("GND", 0.3, [(2.24, 19.55), (3.1, 19.55), (3.8, 19.75)])  # U1.5 -> C1.1
    t.track("+3V3", 0.2, [(0.76, 19.05), (0, 19.05), (0, 21)])  # VOS ...
    t.via("+3V3", 0, 21)
    t.track("+3V3", 0.2, [(0, 21), (4.1, 25)], pcbnew.B_Cu)
    t.via("+3V3", 4.1, 25)
    t.track("+3V3", 0.3, [(4.1, 25), (4.9, 23.85)])  # ... directly to C2.1
    # Decoupling at the H2-Zero: C4 and C5 connected directly (Freerouting found no way between them)
    t.track("+3V3", 0.4, [(-1.15, 26.2), (0.525, 26.2)])
    # Bottom left, J6 pins 8 and 9 (IO4 battery, IO5 motor+) sit between the board edge and the
    # antenna keepout; Freerouting does not find its way out of this corner reliably, so both
    # divider nodes are routed by hand:
    # IO5 on the bottom up, via to the top, then below the motor+ row to C8.1, R15.1 and R14.2
    t.track("ADC_MP", 0.25, [(-5.62, 38.91), (-4.25, 38.91), (-4.25, 34.2), (-3.6, 34.2)], pcbnew.B_Cu)
    t.via("ADC_MP", -3.6, 34.2)
    t.track("ADC_MP", 0.25, [(-3.6, 34.2), (-2.875, 34.2), (-2.875, 32.75)])
    t.track("ADC_MP", 0.25, [(-2.875, 34.2), (0.175, 34.2), (0.175, 32.75)])
    t.track("ADC_MP", 0.25, [(0.175, 34.2), (3.275, 34.2), (3.275, 32.75)])
    # IO4 on the top up, then between the battery and motor+ rows (0.65 mm gap) to C7.1, R13.1, R12.2
    t.track("ADC_BAT", 0.2, [(-5.62, 36.37), (-4.25, 36.37), (-4.25, 31.95), (3.275, 31.95)])
    for x in (-2.875, 0.175, 3.275):
        t.track("ADC_BAT", 0.2, [(x, 31.95), (x, 31.15)])


routing.route(os.path.join(HERE, "roller_carrier.kicad_pcb"), os.path.join(HERE, "make_board.py"), hand_routes,
              sys.argv[1] if len(sys.argv) > 1 else os.environ.get("FREEROUTING_JAR"))
