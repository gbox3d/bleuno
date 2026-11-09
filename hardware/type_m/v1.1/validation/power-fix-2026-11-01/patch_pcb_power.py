"""Apply the v1.1 power-section PCB fix with KiCad's own Python (pcbnew). Never touches v1.

Changes (exactly):
- J1 pad 1 (battery +) net VCC -> VIN; the three tracks from J1.1 down to (172.385, 96.731) become VIN.
- The diagonal track 66ef6120 that tied that path into the VCC node (169.845, 94.191) is removed.
- A new 1.0 mm F.Cu VIN track joins (172.385, 96.731) to J2 pad 3 (172.385, 93.048) = switch ON input.
  Result: battery + -> VIN -> J2.3; switch common J2.2 -> VCC -> all loads. J2.1 (OFF) stays open.
- J1 Value "MT1" -> "BAT"; silk "BAT + -" -> "BAT"; new per-pad marks "-" over pad 2 (GND) and
  "+" over pad 1 (VCC/battery +), 0.8 mm text so they meet the project's minimum text height.
- Zones are refilled natively and the board is saved.

Run with "%LOCALAPPDATA%/Programs/KiCad/10.0/bin/python.exe". Refuses a second application.
"""
import hashlib
import json
from pathlib import Path

import pcbnew

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]  # .../hardware/type_m/v1.1
PCB = ROOT / "kicad" / "type_m_v1_1.kicad_pcb"
# Sensor-stage board this patch was written against (validation/revision-comparison.json).
BASELINE_SHA256 = "1105ff3d34db037440e3a9b3378be9334527ece302c66043df2a05d6d55889da"
RENET = {  # uuid: (start, end) mm, all F.Cu width 1.0, currently VCC
    "b06922f1-ea8d-4d66-b454-923d92320ff5": ((181.263, 91.778), (181.263, 92.806)),
    "1e4529bd-6494-4ae0-b11e-f1200a5d3325": ((177.338, 96.731), (181.263, 92.806)),
    "2079d983-7e97-4d08-9d16-d6ba366e0629": ((172.385, 96.731), (177.338, 96.731)),
}
REMOVE = ("66ef6120-c438-4395-8cc0-c941c531d789", ((169.845, 94.191), (172.385, 96.731)))
NEW_TRACK = ((172.385, 96.731), (172.385, 93.048))


def mm(x, y):
    return pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y))


def xy(point):
    return (round(point.x / 1e6, 6), round(point.y / 1e6, 6))


def main():
    if ROOT.name != "v1.1" or ROOT.parent.name != "type_m":
        raise RuntimeError("This patch is scoped to hardware/type_m/v1.1")
    locks = sorted(p.name for p in PCB.parent.glob("~*.lck"))
    if locks:
        raise RuntimeError("KiCad has the project open (%s); close it first" % ", ".join(locks))
    raw = PCB.read_bytes()
    if b'(net "VIN")' in raw and b'(fp_text user "+"' in raw:
        raise RuntimeError("Power fix already applied; refusing to rerun")
    before = hashlib.sha256(raw).hexdigest()
    if before != BASELINE_SHA256:
        raise RuntimeError("Board is not the sensor-stage baseline this patch targets; refusing")
    if not pcbnew.GetBuildVersion().startswith("10.0"):
        raise RuntimeError("Written and verified against KiCad 10.0.x; got " + pcbnew.GetBuildVersion())
    board = pcbnew.LoadBoard(str(PCB))
    footprints = {f.GetReference(): f for f in board.GetFootprints()}
    j1, j2 = footprints["J1"], footprints["J2"]
    pads = {p.GetNumber(): p for p in j1.Pads()}
    if pads["1"].GetNetname() != "VCC" or pads["2"].GetNetname() != "GND":
        raise RuntimeError("J1 pads are not in the sensor-stage state; refusing to rerun")
    j2_pads = {p.GetNumber(): p for p in j2.Pads()}
    assert j2_pads["3"].GetNetname() == "VIN" and xy(j2_pads["3"].GetPosition()) == NEW_TRACK[1]
    assert j2_pads["2"].GetNetname() == "VCC"
    vin = board.FindNet("VIN")
    assert vin is not None and vin.GetNetname() == "VIN"

    tracks = {t.m_Uuid.AsString(): t for t in board.GetTracks()}
    for uid, (start, end) in list(RENET.items()) + [REMOVE]:
        track = tracks[uid]
        assert track.GetNetname() == "VCC" and track.GetLayer() == pcbnew.F_Cu
        assert {xy(track.GetStart()), xy(track.GetEnd())} == {start, end}, uid
        assert track.GetWidth() == pcbnew.FromMM(1.0)
    for uid in RENET:
        tracks[uid].SetNet(vin)
    board.Remove(tracks[REMOVE[0]])

    new_track = pcbnew.PCB_TRACK(board)
    new_track.SetStart(mm(*NEW_TRACK[0]))
    new_track.SetEnd(mm(*NEW_TRACK[1]))
    new_track.SetWidth(pcbnew.FromMM(1.0))
    new_track.SetLayer(pcbnew.F_Cu)
    new_track.SetNet(vin)
    board.Add(new_track)
    pads["1"].SetNet(vin)

    # Naming and polarity marking on the battery connector
    assert j1.GetValue() == "MT1"
    j1.SetValue("BAT")
    silk = [item for item in j1.GraphicalItems()
            if isinstance(item, pcbnew.PCB_TEXT) and item.GetText() == "BAT + -"]
    assert len(silk) == 1
    silk = silk[0]
    assert silk.GetLayer() == pcbnew.F_SilkS and xy(silk.GetFPRelativePosition()) == (-4.064, -0.635)
    silk.SetText("BAT")  # position, size and justification stay as in the original silk
    marks = {}
    for text, x in (("-", -0.75), ("+", 0.75)):  # pad 2 (GND) is at x=-0.75, pad 1 (VCC) at x=+0.75
        mark = pcbnew.PCB_TEXT(j1)
        mark.SetText(text)
        mark.SetLayer(pcbnew.F_SilkS)
        mark.SetTextSize(mm(0.8, 0.8))
        mark.SetTextThickness(pcbnew.FromMM(0.15))
        mark.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_CENTER)
        mark.SetVertJustify(pcbnew.GR_TEXT_V_ALIGN_BOTTOM)
        j1.Add(mark)
        mark.SetFPRelativePosition(mm(x, -0.9))
        marks[text] = {"uuid": mark.m_Uuid.AsString(), "relative_mm": [x, -0.9],
                       "board_mm": list(xy(mark.GetPosition())), "size_mm": 0.8, "thickness_mm": 0.15}
    expected = {"-": xy(pads["2"].GetPosition())[0], "+": xy(pads["1"].GetPosition())[0]}
    for text, mark in marks.items():
        assert mark["board_mm"][0] == expected[text], (text, mark, expected)

    board.BuildConnectivity()
    filler = pcbnew.ZONE_FILLER(board)
    assert filler.Fill(board.Zones())
    pcbnew.SaveBoard(str(PCB), board, True)  # aSkipSettings: do not rewrite the .kicad_pro
    report = {"input_sha256": before,
              "output_sha256": hashlib.sha256(PCB.read_bytes()).hexdigest(),
              "pad_net_changes": {"J1.1": ["VCC", "VIN"]},
              "tracks_renetted_to_VIN": [{"uuid": uid, "start_mm": s, "end_mm": e, "width_mm": 1.0, "layer": "F.Cu"}
                                         for uid, (s, e) in RENET.items()],
              "track_removed": {"uuid": REMOVE[0], "start_mm": REMOVE[1][0], "end_mm": REMOVE[1][1]},
              "track_added": {"uuid": new_track.m_Uuid.AsString(), "net": "VIN", "layer": "F.Cu",
                              "start_mm": NEW_TRACK[0], "end_mm": NEW_TRACK[1], "width_mm": 1.0},
              "J1_value": ["MT1", "BAT"],
              "J1_silk_text": {"uuid": silk.m_Uuid.AsString(), "text": ["BAT + -", "BAT"],
                               "relative_mm": [-4.064, -0.635]},
              "J1_polarity_marks": marks,
              "zones_refilled": True,
              "note": "Run kicad-cli ERC/netlist/DRC --schematic-parity and verify_power_fix.py next."}
    (HERE / "pcb-power-patch.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
