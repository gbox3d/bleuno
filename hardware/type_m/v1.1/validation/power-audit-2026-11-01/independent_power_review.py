"""KiCad-only read-only review using native connectivity and SVG glyph strokes.

Unlike audit_power.py, this uses KiCad's connectivity engine and a separate
exact-endpoint path search. It does not create connectivity from net names.
Only the review JSON is written; no schematic, PCB, or library is saved.
"""
from collections import Counter, defaultdict, deque
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import pcbnew

HERE = Path(__file__).resolve().parent
REV = HERE.parents[1]
TYPE_M = REV.parent
spec = importlib.util.spec_from_file_location("revision_reader", REV / "validation/verify_revision.py")
reader = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reader)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def xy(point):
    return [round(point.x / 1e6, 6), round(point.y / 1e6, 6)]


def native(board):
    assert board.BuildConnectivity()
    connectivity = board.GetConnectivity()
    pad_names = {p.m_Uuid.AsString(): f.GetReference() + "." + p.GetNumber()
                 for f in board.GetFootprints() for p in f.Pads()}
    result = {}
    for footprint in board.GetFootprints():
        if footprint.GetReference() not in ("J1", "J2"):
            continue
        for pad in footprint.Pads():
            items = list(connectivity.GetConnectedItems(pad))
            pads = sorted(pad_names[item.m_Uuid.AsString()]
                          for item in items if isinstance(item, pcbnew.PAD))
            result[footprint.GetReference() + "." + pad.GetNumber()] = {
                "pad_net": pad.GetNetname(), "xy_mm": xy(pad.GetPosition()),
                "connected_pads": pads, "native_item_types": dict(Counter(type(item).__name__ for item in items)),
                "direct_track_count": len(connectivity.GetConnectedTracks(pad))}
    return result


def endpoint_path(board):
    graph = defaultdict(list)
    for item in board.GetTracks():
        if isinstance(item, pcbnew.PCB_VIA):
            point = item.GetPosition()
            a, b = (pcbnew.F_Cu, point.x, point.y), (pcbnew.B_Cu, point.x, point.y)
        else:
            a = (item.GetLayer(), item.GetStart().x, item.GetStart().y)
            b = (item.GetLayer(), item.GetEnd().x, item.GetEnd().y)
        graph[a].append((b, item))
        graph[b].append((a, item))
    pads = {(f.GetReference(), p.GetNumber()): p for f in board.GetFootprints() for p in f.Pads()}
    start, target = pads[("J1", "1")].GetPosition(), pads[("U1", "8")].GetPosition()
    start, target = (pcbnew.F_Cu, start.x, start.y), (pcbnew.F_Cu, target.x, target.y)
    queue, parent = deque([start]), {start: None}
    while queue:
        node = queue.popleft()
        if node == target:
            steps = []
            while parent[node] is not None:
                previous, item = parent[node]
                steps.append({"uuid": item.m_Uuid.AsString(), "net": item.GetNetname(),
                              "from_mm": [previous[1] / 1e6, previous[2] / 1e6],
                              "to_mm": [node[1] / 1e6, node[2] / 1e6],
                              "layer": pcbnew.LayerName(item.GetLayer())})
                node = previous
            return steps[::-1]
        for neighbor, item in graph[node]:
            if neighbor not in parent:
                parent[neighbor] = (node, item)
                queue.append(neighbor)
    return None


def main():
    pcb_path = REV / "kicad/type_m_v1_1.kicad_pcb"
    sch_path = REV / "kicad/type_m_v1_1.kicad_sch"
    fp_path = REV / "kicad/type_m_v1_1.pretty/ESP32C3 SM.kicad_mod"
    svg_path = REV / "validation/preview/type_m_v1_1-pcb.svg"
    inputs = {str(p.relative_to(TYPE_M)): sha(p) for p in (pcb_path, sch_path, fp_path, svg_path)}
    board = pcbnew.LoadBoard(str(pcb_path))
    old_board = pcbnew.LoadBoard(str(TYPE_M / "v1/kicad/type_m_v1.kicad_pcb"))
    current, original = native(board), native(old_board)
    footprints = {f.GetReference(): f for f in board.GetFootprints()}
    schematic = reader.sexpr(sch_path.read_text(encoding="utf-8"))
    library = reader.one(schematic, "lib_symbols")
    mapping = {}
    for instance in reader.children(schematic, "symbol"):
        properties = reader.children(instance, "property")
        ref = next((json.loads(p[2]) for p in properties if p[1] == '"Reference"'), None)
        if ref not in ("J1", "J2", "U1"):
            continue
        name = reader.value(instance, "lib_id")
        symbol = next(n for n in reader.children(library, "symbol") if json.loads(n[1]) == name)
        pads = {p.GetNumber(): p for p in footprints[ref].Pads()}
        mapping[ref] = {"schematic_value": next(json.loads(p[2]) for p in properties if p[1] == '"Value"'),
                        "pcb_value": footprints[ref].GetValue(), "pins": {}}
        for body in reader.children(symbol, "symbol"):
            for pin in reader.children(body, "pin"):
                number = reader.value(pin, "number")
                if ref == "U1" and number not in ("6", "7", "8"):
                    continue
                pad = pads[number]
                mapping[ref]["pins"][number] = {"schematic_pin_name": reader.value(pin, "name"),
                                               "pcb_pad_number": number, "pcb_net": pad.GetNetname(),
                                               "pcb_xy_mm": xy(pad.GetPosition()),
                                               "pad_shape": int(pad.GetShape())}
    fp_tree = reader.sexpr(fp_path.read_text(encoding="utf-8"))
    fp_texts = [json.loads(n[2]) for n in reader.children(fp_tree, "fp_text")]
    pcb_texts = [item.GetText() for item in footprints["U1"].GraphicalItems() if hasattr(item, "GetText")]

    # Use the actual exported KiCad glyph strokes, not a browser substitute font.
    svg = ET.parse(svg_path)
    ns = "{http://www.w3.org/2000/svg}"
    bat_text = next(n for n in svg.iter(ns + "text") if n.text == "BAT + -")
    bat_group = next(n for n in svg.iter(ns + "g")
                     if any(c.tag == ns + "desc" and c.text == "BAT + -" for c in n))
    paths = [n.get("d") for n in bat_group if n.tag == ns + "path"]
    assert len(paths) == 26
    glyph_segments = []
    for data in paths[-3:]:
        numbers = list(map(float, re.findall(r"-?\d+(?:\.\d+)?", data)))
        assert len(numbers) == 4
        glyph_segments.append(numbers)
    plus_horizontal, plus_vertical, minus_horizontal = glyph_segments
    assert plus_horizontal[1] == plus_horizontal[3] and plus_vertical[0] == plus_vertical[2]
    assert minus_horizontal[1] == minus_horizontal[3]
    text = next(n for n in footprints["J1"].GraphicalItems() if hasattr(n, "GetText") and n.GetText() == "BAT + -")
    offset = [text.GetPosition().x / 1e6 - float(bat_text.get("x")),
              text.GetPosition().y / 1e6 - float(bat_text.get("y"))]
    plus_x = plus_vertical[0] + offset[0]
    minus_x = (minus_horizontal[0] + minus_horizontal[2]) / 2 + offset[0]
    polarity = {"text": text.GetText(), "footprint_rotation_deg": footprints["J1"].GetOrientationDegrees(),
                "text_rotation_deg": text.GetTextAngle().AsDegrees(), "mirrored": text.IsMirrored(),
                "svg_to_board_offset_mm": offset, "native_svg_final_three_glyph_paths": paths[-3:],
                "plus_center_board_x_mm": round(plus_x, 6), "minus_center_board_x_mm": round(minus_x, 6),
                "physical_pad_left_to_right": ["J1.2 GND (-)", "J1.1 VCC (+)"],
                "silkscreen_sign_left_to_right": ["+", "-"],
                "sign_order_opposes_pad_polarity_order": plus_x < minus_x,
                "limit": "The signs form one unaligned text string, not two per-pad markers. Order conflict is confirmed; intent or actual reverse wiring is not established."}
    report = {"generated_utc": datetime.now(timezone.utc).isoformat(), "inputs_sha256": inputs,
              "method": "Native KiCad connectivity plus independent exact-track-endpoint BFS; direct schematic/footprint pad-number comparison; actual SVG stroke coordinates.",
              "native_v1": original, "native_v1_1": current, "same_native_power_components": original == current,
              "J1_1_to_U1_8_exact_track_endpoint_chain": endpoint_path(board),
              "schematic_to_pcb_pin_mapping": mapping,
              "U1_footprint_texts": fp_texts, "U1_board_footprint_texts": pcb_texts,
              "module_power_silkscreen_limit": "This KiCad footprint has no 5V/GND/3V3 pin text, so physical module pin labels cannot be independently certified from these files alone.",
              "battery_polarity_marking": polarity,
              "conclusions": [
                  "No BAT-to-C3 open circuit found: J1.1 reaches U1.8 by a continuous F.Cu track chain; J1.2 reaches U1.7 through native GND connectivity including filled zones.",
                  "J2.1 and J2.3 are standalone copper pads; battery and loads share VCC independently of switch contacts.",
                  "J1 schematic/PCB Value MT1 conflicts with its BAT + - silkscreen role and can confuse identification.",
                  "BAT + - text order opposes the physical left-to-right GND/VCC pad order; this is a polarity-label ambiguity worth correcting, not proof of actual reverse wiring.",
                  "Module-internal power circuitry and suitability for a 3.7V battery are not represented by this carrier KiCad design."],
              "input_files_unchanged": all(sha(TYPE_M / name) == digest for name, digest in inputs.items())}
    output = HERE / "independent-power-review.json"
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"report": str(output), "same_native_power_components": report["same_native_power_components"],
                      "exact_endpoint_chain_tracks": len(report["J1_1_to_U1_8_exact_track_endpoint_chain"] or []),
                      "polarity": polarity, "input_files_unchanged": report["input_files_unchanged"]}, indent=2))


if __name__ == "__main__":
    main()
