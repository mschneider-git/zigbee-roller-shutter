"""
Routing steps for the board (KiCad 10 pcbnew API + Freerouting).

route.py calls route() with a callback that adds the board-specific hand-routed tracks:

  1. fresh placement from make_board.py (removing old tracks crashes with KiCad 10 + Python 3.14)
  2. net class "Power" (0.4 mm) for the supply nets
  3. callback: locked hand-routed tracks
  4. copper area VBAT_RAW between the two WAGO poles (the battery current flows through it)
  5. Freerouting via Specctra DSN/SES, GND areas on both layers, stitching vias, DRC;
     Freerouting's result varies between runs and it occasionally hangs, so every attempt is
     checked with the DRC and routed again up to TRIES times
"""
import json
import os
import subprocess
import sys
import tempfile

import pcbnew

ORIGIN  = (120.0, 80.0)
OUTLINE = [(-7.5, 11.5), (11.5, 11.5), (11.5, 24.5), (24.5, 24.5), (24.5, 11.5), (34.0, 11.5),
           (34.0, 40.6), (-7.5, 40.6)]
POWER   = ["VBAT_RAW", "VBAT_F", "VBAT", "VIN", "+3V3", "SW", "VBUS", "GND"]
TRIES   = 5


def P(x, y):
    return pcbnew.VECTOR2I(pcbnew.FromMM(ORIGIN[0] + x), pcbnew.FromMM(ORIGIN[1] + y))


def zone(board, netname, layer, pts, priority=0, full=False):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(board.FindNet(netname))
    poly = z.Outline()
    poly.NewOutline()
    for x, y in pts:
        poly.Append(P(x, y))
    z.SetAssignedPriority(priority)
    z.SetLocalClearance(pcbnew.FromMM(0.25))
    z.SetMinThickness(pcbnew.FromMM(0.2))
    z.SetThermalReliefGap(pcbnew.FromMM(0.3))
    z.SetThermalReliefSpokeWidth(pcbnew.FromMM(0.35))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL if full else pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    board.Add(z)
    return z


class Tracks:
    """Locked hand-routed tracks and vias; coordinates relative to ORIGIN."""

    def __init__(self, board):
        self.board = board

    def track(self, net, w, pts, layer=pcbnew.F_Cu):
        for (xa, ya), (xb, yb) in zip(pts, pts[1:]):
            t = pcbnew.PCB_TRACK(self.board)
            t.SetStart(P(xa, ya))
            t.SetEnd(P(xb, yb))
            t.SetWidth(pcbnew.FromMM(w))
            t.SetLayer(layer)
            t.SetNet(self.board.FindNet(net))
            t.SetLocked(True)
            self.board.Add(t)

    def via(self, net, x, y, size=0.6):
        add_via(self.board, net, x, y, size)


def add_via(board, net, x, y, size=0.6):
    v = pcbnew.PCB_VIA(board)
    v.SetPosition(P(x, y))
    v.SetWidth(pcbnew.FromMM(size))
    v.SetDrill(pcbnew.FromMM(0.3))
    v.SetNet(board.FindNet(net))
    v.SetLocked(True)
    board.Add(v)


def _prepare(board, hand_routes):
    ns = board.GetDesignSettings().m_NetSettings
    pw = pcbnew.NETCLASS("Power")
    pw.SetClearance(pcbnew.FromMM(0.15))   # same as default, Freerouting does not mix class clearances cleanly
    pw.SetTrackWidth(pcbnew.FromMM(0.4))
    pw.SetViaDiameter(pcbnew.FromMM(0.6))
    pw.SetViaDrill(pcbnew.FromMM(0.3))
    ns.SetNetclass("Power", pw)
    ns.ClearNetclassPatternAssignments()
    for n in POWER:
        ns.SetNetclassPatternAssignment(n, "Power")
    ns.RecomputeEffectiveNetclasses()
    for n in POWER:
        if board.FindNet(n):  # VBUS only exists on boards with their own USB connector
            board.FindNet(n).SetNetClass(ns.GetNetClassByName("Power"))

    hand_routes(board)

    # SMD GND pads fully connected to the area (reflow), THT pads keep spokes for hand soldering
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetname() == "GND" and p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD:
                p.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)

    # VBAT_RAW: copper area over the pads of both WAGO poles on both layers, 0.4 mm around them,
    # with a tab down to the input pad of the fuse F1 (below the terminal, to the left)
    def mm_box(item):
        b = item.GetBoundingBox()
        return (pcbnew.ToMM(b.GetX()) - ORIGIN[0], pcbnew.ToMM(b.GetY()) - ORIGIN[1],
                pcbnew.ToMM(b.GetRight()) - ORIGIN[0], pcbnew.ToMM(b.GetBottom()) - ORIGIN[1])

    boxes = [mm_box(p) for p in board.FindFootprintByReference("J1").Pads()]
    x0, y0 = max(min(b[0] for b in boxes) - 0.4, 24.8), max(min(b[1] for b in boxes) - 0.4, 11.8)
    x1, y1 = min(max(b[2] for b in boxes) + 0.4, 33.7), max(b[3] for b in boxes) + 0.4
    f = mm_box(next(p for p in board.FindFootprintByReference("F1").Pads() if p.GetNetname() == "VBAT_RAW"))
    tab_x0, tab_x1, tab_y1 = f[0] + 0.1, x0 + 0.55, (f[1] + f[3]) / 2
    outline = [(x0, y0), (x1, y0), (x1, y1), (tab_x1, y1), (tab_x1, tab_y1), (tab_x0, tab_y1),
               (tab_x0, y1 - 0.3), (x0, y1 - 0.3)]
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        zone(board, "VBAT_RAW", layer, outline, priority=2, full=True)
    board.BuildConnectivity()
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())


def _via_fits(gnd, x, y, r=0.6):
    """True if a GND via at (x, y) lies fully inside the GND fill on both layers within radius r
    (the fill already keeps clearance to other nets, so r = 0.35 is enough for the 0.6 mm via)."""
    ring = [P(x, y)] + [P(x + r * dx, y + r * dy) for dx, dy in
                        ((1, 0), (-1, 0), (0, 1), (0, -1), (.7, .7), (.7, -.7), (-.7, .7), (-.7, -.7))]
    return all(any(z.GetFilledPolysList(z.GetLayer()).Contains(p) for z in gnd if z.GetLayer() == layer)
               for layer in (pcbnew.F_Cu, pcbnew.B_Cu) for p in ring)


def _vias(board):
    tr = board.Tracks()  # the SWIG iterator is broken with Python 3.14, so index access
    return [tr[i].GetPosition() for i in range(len(tr)) if tr[i].Type() == pcbnew.PCB_VIA_T]


def _stitch(board, exclude_refs):
    """GND vias on a 2 mm grid, then one via in every GND fill area that has none yet."""
    gnd     = [z for z in board.Zones() if z.GetNetname() == "GND"]
    exclude = [board.FindFootprintByReference(r).GetBoundingBox(False, False) for r in exclude_refs]
    vias    = _vias(board)
    count   = 0

    def near(c, dist):
        return any(abs(v.x - c.x) < pcbnew.FromMM(dist) and abs(v.y - c.y) < pcbnew.FromMM(dist) for v in vias)

    y = 12.5
    while y < 40.0:
        x = -6.5
        while x < 33.5:
            c = P(x, y)
            if not near(c, 1.0) and not any(b.Contains(c) for b in exclude) and _via_fits(gnd, x, y):
                add_via(board, "GND", x, y)
                vias.append(c)
                count += 1
            x += 2.0
        y += 2.0

    # Small GND areas (e.g. between connector pads) are easily missed by the grid
    board.BuildConnectivity()
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    for z in gnd:
        polys = z.GetFilledPolysList(z.GetLayer())
        for i in range(polys.OutlineCount()):
            chain = polys.COutline(i)
            if any(chain.PointInside(v) for v in vias):
                continue
            bb = chain.BBox()
            xs = [pcbnew.ToMM(bb.GetX()) - ORIGIN[0] + k * 0.25 for k in range(int(pcbnew.ToMM(bb.GetWidth()) / 0.25) + 1)]
            ys = [pcbnew.ToMM(bb.GetY()) - ORIGIN[1] + k * 0.25 for k in range(int(pcbnew.ToMM(bb.GetHeight()) / 0.25) + 1)]
            for y in ys:
                hit = next((x for x in xs if chain.PointInside(P(x, y)) and not near(P(x, y), 1.0)
                            and _via_fits(gnd, x, y, r=0.35)), None)
                if hit is not None:
                    add_via(board, "GND", hit, y)
                    vias.append(P(hit, y))
                    count += 1
                    break
    return count


def _drc_errors(board_path, tmp):
    rep = os.path.join(tmp, "drc.json")
    subprocess.run(["kicad-cli", "pcb", "drc", "--severity-error", "--refill-zones", "--format", "json",
                    "-o", rep, board_path], capture_output=True)
    with open(rep) as f:
        d = json.load(f)
    return [v["type"] for v in d["violations"]] + ["unconnected"] * len(d["unconnected_items"])


def route(board_path, make_script, hand_routes, jar, stitch_exclude=()):
    if not os.path.isfile(jar or ""):
        sys.exit("freerouting.jar not found (path as argument or FREEROUTING_JAR)")
    subprocess.run([sys.executable, make_script], check=True, stdout=subprocess.DEVNULL)
    board = pcbnew.LoadBoard(board_path)
    _prepare(board, hand_routes)

    tmp  = tempfile.mkdtemp()
    prep = os.path.join(tmp, "prep.kicad_pcb")
    dsn  = os.path.join(tmp, "board.dsn")
    ses  = os.path.join(tmp, "board.ses")
    board.Save(prep)
    if not pcbnew.ExportSpecctraDSN(board, dsn):
        sys.exit("DSN export failed")

    for attempt in range(1, TRIES + 1):
        if os.path.exists(ses):
            os.remove(ses)
        # Freerouting is deterministic for the same input; a different number of passes per
        # attempt gives a different result
        passes = (30, 20, 40, 25, 50)[(attempt - 1) % 5]
        try:
            # without --gui.enabled=false Freerouting sometimes opens its window and waits
            r = subprocess.run(["java", "-jar", jar, "--gui.enabled=false", "-de", dsn, "-do", ses,
                                "-mp", str(passes), "-mt", "1"],
                               capture_output=True, text=True, timeout=420)
        except subprocess.TimeoutExpired:
            # Freerouting occasionally hangs while optimising; run() kills Java, then retry
            print(f"Attempt {attempt}: Freerouting hangs, aborted after 7 min")
            continue
        if not os.path.isfile(ses):
            sys.exit("Freerouting produced no SES file:\n" + r.stdout[-3000:] + r.stderr[-3000:])
        board = pcbnew.LoadBoard(prep)
        if not pcbnew.ImportSpecctraSES(board, ses):
            sys.exit("SES import failed")
        # Freerouting necks tracks down at small pads (0.12 mm); widen them to the board minimum,
        # the DRC below checks whether the clearance still holds
        min_w = board.GetDesignSettings().m_TrackMinWidth
        tr = board.Tracks()
        for i in range(len(tr)):
            if tr[i].Type() == pcbnew.PCB_TRACE_T and tr[i].GetWidth() < min_w:
                tr[i].SetWidth(min_w)

        for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
            zone(board, "GND", layer, OUTLINE, priority=0)
        board.BuildConnectivity()
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        stitched = _stitch(board, stitch_exclude)
        board.BuildConnectivity()
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        board.Save(board_path)

        errors = _drc_errors(board_path, tmp)
        print(f"Attempt {attempt} ({passes} passes): {stitched} stitching vias, DRC errors: {errors or 'none'}")
        if not errors:
            print("saved:", board_path, "tracks:", board.Tracks().size())
            return
    sys.exit(f"not error-free after {TRIES} attempts")
