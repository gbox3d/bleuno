"""Read-only before->after audit of the type_m v1.1 power-section fix.

Baseline: the sensor-stage snapshot preserved next to this script (before-power-fix.*), whose
hashes must equal the inputs recorded by the sensor-stage audit (validation/revision-comparison.json).
Only the JSON report is written; no KiCad design is saved or changed.
Run with KiCad's installed Python (pcbnew is needed for the native copper-contact checks).
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pcbnew

sys.dont_write_bytecode = True  # importing verify_revision.py below must not drop a __pycache__ into the repo
HERE = Path(__file__).resolve().parent
REV = HERE.parents[1]                      # hardware/type_m/v1.1
OLD = REV.parent / "v1"
spec = importlib.util.spec_from_file_location("revision_reader", REV / "validation/verify_revision.py")
reader = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reader)
sexpr, children, one, value, uid, keyed = (reader.sexpr, reader.children, reader.one,
                                           reader.value, reader.uid, reader.keyed)

PWR012 = "cf822914-aece-439c-8a36-4716e4397660"
J1_SYMBOL = "ad2c76f2-73a2-475e-b908-4b6411e7d519"
VIN_AT_J2 = "34783d0f-520b-45c8-b483-83569857bdb8"
RENET = ["b06922f1-ea8d-4d66-b454-923d92320ff5", "1e4529bd-6494-4ae0-b11e-f1200a5d3325",
         "2079d983-7e97-4d08-9d16-d6ba366e0629"]
REMOVED = "66ef6120-c438-4395-8cc0-c941c531d789"
J1_FOOTPRINT = "4d911aaa-4f55-4971-b5a3-3e6be3c701ce"
SILK = "d664905f-4794-57a0-82e9-0b2553fa69e9"
# ERC item uuids (schematic pins / label)
J2_PIN1, J2_PIN3, J1_PIN1, PWR012_PIN = ("557cf05d-d743-4409-8c5d-221796c35201", "7e9e9abb-1d4a-4f98-b0f9-88157bef02f6",
                                        "f36b8395-4bd0-45bb-9e04-f0dd1bdce723", "1a89e1cb-78a0-47bb-bab6-713c606c51cc")
EXPECTED_VIN = [("J1", "1"), ("J2", "3")]
EXPECTED_VCC = [("J2", "2"), ("J5", "1"), ("J6", "1"), ("R1", "1"), ("U1", "8"), ("U2", "9")]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def canon(node):
    return json.dumps(node, ensure_ascii=False)


def nets(path):
    root = ET.parse(path).getroot()
    return {n.get("name"): sorted((x.get("ref"), x.get("pin")) for x in n.findall("node"))
            for n in root.findall("nets/net")}


def props(node):
    return {json.loads(p[1]): json.loads(p[2]) for p in children(node, "property")}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before-sch", type=Path, default=HERE / "before-power-fix.kicad_sch")
    parser.add_argument("--before-pcb", type=Path, default=HERE / "before-power-fix.kicad_pcb")
    parser.add_argument("--before-netlist", type=Path, default=HERE / "before-power-fix-netlist.xml")
    parser.add_argument("--before-erc", type=Path, default=HERE / "before-power-fix-erc.json")
    parser.add_argument("--before-drc", type=Path, default=HERE / "before-power-fix-drc.json")
    parser.add_argument("--sch", type=Path, default=REV / "kicad/type_m_v1_1.kicad_sch")
    parser.add_argument("--pcb", type=Path, default=REV / "kicad/type_m_v1_1.kicad_pcb")
    parser.add_argument("--netlist", type=Path, default=REV / "validation/netlist.xml")
    parser.add_argument("--erc", type=Path, default=REV / "validation/erc.json")
    parser.add_argument("--drc", type=Path, default=REV / "validation/drc.json")
    parser.add_argument("--output", type=Path, default=HERE / "power-fix-verification.json")
    args = parser.parse_args()
    files = {k: getattr(args, k) for k in ("before_sch", "before_pcb", "before_netlist", "before_erc",
                                            "before_drc", "sch", "pcb", "netlist", "erc", "drc")}
    if not pcbnew.GetBuildVersion().startswith("10.0"):
        raise SystemExit("Expected ERC/DRC counts are KiCad 10.0.x specific; got " + pcbnew.GetBuildVersion())
    report = {"generated_utc": datetime.now(timezone.utc).isoformat(),
              "kicad_version": pcbnew.GetBuildVersion(),
              "input_sha256": {k: sha(p) for k, p in files.items()}, "checks": [],
              "allowed_changes": [
                  "SCH: J1 Value MT1 -> BAT; #PWR012 removed; global label VIN at (91.44,40.64,180); no_connect at J2 pin 1",
                  "PCB: J1.1 net VCC -> VIN; tracks b06922f1/1e4529bd/2079d983 VCC -> VIN; track 66ef6120 removed;"
                  " one new 1.0 mm F.Cu VIN track (172.385,96.731)-(172.385,93.048); J1 Value BAT;"
                  " J1 silk 'BAT + -' -> 'BAT' (same place); new 0.8 mm '-' and '+' marks over pads 2 and 1",
                  "PCB: zone filled polygons regenerated natively; all zone settings/outlines unchanged"]}

    def check(name, ok, detail=None):
        item = {"name": name, "passed": bool(ok)}
        if detail is not None:
            item["detail"] = detail
        report["checks"].append(item)

    # Chain of custody: the before snapshot is exactly the sensor-stage design that revision-comparison.json audited.
    sensor_audit = json.loads((REV / "validation/revision-comparison.json").read_text(encoding="utf-8"))
    check("before_snapshot_equals_sensor_stage_audited_files",
          sensor_audit["input_sha256"]["revision_pcb"] == report["input_sha256"]["before_pcb"]
          and sensor_audit["input_sha256"]["revision_schematic"] == report["input_sha256"]["before_sch"]
          and sensor_audit.get("passed") is True)
    check("before_reports_equal_sensor_stage_pinned_netlist_erc_drc",
          sensor_audit["input_sha256"]["netlist"] == report["input_sha256"]["before_netlist"]
          and sensor_audit["input_sha256"]["erc"] == report["input_sha256"]["before_erc"]
          and sensor_audit["input_sha256"]["drc"] == report["input_sha256"]["before_drc"])
    originals = json.loads((REV / "validation/v1-input-hashes.json").read_text(encoding="utf-8-sig"))
    changed = [v["path"] for v in originals if sha(OLD / v["path"]).lower() != v["sha256"].lower()]
    check("v1_snapshot_hashes_preserved", not changed, {"files": len(originals), "changed": changed})
    # The stored reports only describe the current design if they were produced after its last save:
    # netlist/ERC follow the schematic alone, DRC (with schematic parity) follows both files.
    sch_mtime, pcb_mtime = args.sch.stat().st_mtime, args.pcb.stat().st_mtime
    needed = {"netlist": sch_mtime, "erc": sch_mtime, "drc": max(sch_mtime, pcb_mtime)}
    stale = [k for k, mtime in needed.items() if files[k].stat().st_mtime < mtime]
    source = ET.parse(args.netlist).getroot().findtext("design/source") or ""
    check("reports_newer_than_design_files_and_netlist_source_is_this_schematic",
          not stale and source.replace("\\", "/").endswith(args.sch.name),
          {"stale": stale, "netlist_source": source})

    # ---------------- schematic ----------------
    before = sexpr(args.before_sch.read_text(encoding="utf-8"))
    after = sexpr(args.sch.read_text(encoding="utf-8"))
    removed = Counter(map(canon, before[1:])) - Counter(map(canon, after[1:]))
    added = Counter(map(canon, after[1:])) - Counter(map(canon, before[1:]))
    removed_nodes = [json.loads(k) for k in removed.elements()]
    added_nodes = [json.loads(k) for k in added.elements()]
    old_j1 = [n for n in removed_nodes if n[0] == "symbol" and uid(n) == J1_SYMBOL]
    new_j1 = [n for n in added_nodes if n[0] == "symbol" and uid(n) == J1_SYMBOL]
    pwr = [n for n in removed_nodes if n[0] == "symbol" and uid(n) == PWR012]
    labels = [n for n in added_nodes if n[0] == "global_label"]
    ncs = [n for n in added_nodes if n[0] == "no_connect"]
    check("sch_only_four_top_level_changes", len(removed_nodes) == 2 and len(added_nodes) == 3
          and len(old_j1) == len(new_j1) == len(pwr) == len(labels) == len(ncs) == 1,
          {"removed": [n[0] for n in removed_nodes], "added": [n[0] for n in added_nodes]})
    if old_j1 and new_j1:
        expected = json.loads(canon(old_j1[0]).replace('["property", "\\"Value\\"", "\\"MT1\\""',
                                                        '["property", "\\"Value\\"", "\\"BAT\\""'))
        check("sch_J1_only_value_MT1_to_BAT", expected == new_j1[0] and props(new_j1[0])["Value"] == "BAT")
    if pwr:
        check("sch_removed_symbol_is_PWR012_VCC_at_J1_pin1_wire_end",
              value(pwr[0], "lib_id") == "type_m_v1_1:Power-VCC" and one(pwr[0], "at")[1:] == ["91.44", "40.64", "0"]
              and props(pwr[0])["Reference"] == "#PWR012")
    if labels:
        template = next(n for n in children(before, "global_label") if uid(n) == VIN_AT_J2)
        expected = json.loads(canon(template).replace('"52.07", "39.37"', '"91.44", "40.64"')
                              .replace(VIN_AT_J2, uid(labels[0])))
        check("sch_new_VIN_label_clones_existing_style_at_91_44_40_64_rot180",
              expected == labels[0] and json.loads(labels[0][1]) == "VIN")
    if ncs:
        check("sch_no_connect_exactly_at_J2_pin1", [v for v in ncs[0] if isinstance(v, list) and v[0] == "at"]
              == [["at", "54.61", "34.29"]])
    wires = {canon(w) for w in children(after, "wire")}
    wire_points = lambda w: sorted(p[1:] for p in children(one(w, "pts"), "xy"))
    check("sch_wires_unchanged_and_J1_pin1_wire_ends_at_new_label",
          wires == {canon(w) for w in children(before, "wire")}
          and any(wire_points(w) == [["91.44", "40.64"], ["99.06", "40.64"]] for w in children(after, "wire")))
    check("sch_lib_symbols_unchanged", one(before, "lib_symbols") == one(after, "lib_symbols"))

    before_nets, after_nets = nets(args.before_netlist), nets(args.netlist)
    expected_nets = dict(before_nets)
    expected_nets["VCC"] = EXPECTED_VCC
    expected_nets["VIN"] = EXPECTED_VIN
    check("netlist_only_VCC_loses_J1_1_and_VIN_gains_J1_1", after_nets == expected_nets,
          {k: [before_nets.get(k), after_nets.get(k)] for k in sorted(set(before_nets) | set(after_nets))
           if before_nets.get(k) != after_nets.get(k)})

    def violations(path):
        data = json.loads(path.read_text(encoding="utf-8"))
        return [v for s in data["sheets"] for v in s["violations"]]
    signature = lambda v: (v["type"], v["severity"], tuple(x["uuid"] for x in v["items"]))
    old_erc, new_erc = violations(args.before_erc), violations(args.erc)
    erc_removed = Counter(map(signature, old_erc)) - Counter(map(signature, new_erc))
    erc_added = Counter(map(signature, new_erc)) - Counter(map(signature, old_erc))
    check("ERC_delta_exactly_expected",
          erc_removed == Counter({("pin_not_connected", "error", (J2_PIN1,)): 1,
                                  ("isolated_pin_label", "warning", (VIN_AT_J2,)): 1,
                                  ("pin_to_pin", "warning", (J1_PIN1, PWR012_PIN)): 1})
          and erc_added == Counter({("pin_to_pin", "warning", (J1_PIN1, J2_PIN3)): 1}),
          {"before": dict(Counter(v["severity"] for v in old_erc)),
           "after": dict(Counter(v["severity"] for v in new_erc)),
           "removed": [list(k) for k in erc_removed], "added": [list(k) for k in erc_added]})
    report["erc_remaining_imported_symbol_limitations"] = dict(Counter(v["severity"] for v in new_erc))

    # ---------------- PCB (text level) ----------------
    before = sexpr(args.before_pcb.read_text(encoding="utf-8"))
    after = sexpr(args.pcb.read_text(encoding="utf-8"))
    old_fp, new_fp = keyed(children(before, "footprint")), keyed(children(after, "footprint"))
    check("pcb_all_other_footprints_exact", set(old_fp) == set(new_fp)
          and all(old_fp[k] == new_fp[k] for k in old_fp if k != J1_FOOTPRINT))
    j1_old, j1_new = old_fp[J1_FOOTPRINT], new_fp[J1_FOOTPRINT]
    j1_expected = json.loads(json.dumps(j1_old))
    for pad in children(j1_expected, "pad"):
        if pad[1] == '"1"':
            reader.replace_field(pad, "net", "VIN")
    for prop in children(j1_expected, "property"):
        if prop[1] == '"Value"':
            prop[2] = '"BAT"'
    for text in children(j1_expected, "fp_text"):
        if text[2] == '"BAT + -"':
            text[2] = '"BAT"'  # same position (-4.064, -0.635), size and justification
    marks = [t for t in children(j1_new, "fp_text") if t[2] in ('"-"', '"+"')]
    j1_stripped = [v for v in j1_new if not (isinstance(v, list) and v[0] == "fp_text" and v[2] in ('"-"', '"+"'))]
    check("pcb_J1_only_pad1_net_value_and_silk_text_changed", j1_stripped == j1_expected)

    def mark_ok(t, text, x):
        return (t[2] == text and one(t, "at")[1:] == [x, "-0.9", "0"] and value(t, "layer") == "F.SilkS"
                and one(one(one(t, "effects"), "font"), "size")[1:] == ["0.8", "0.8"]
                and one(one(one(t, "effects"), "font"), "thickness")[1] == "0.15"
                and one(one(t, "effects"), "justify")[1:] == ["bottom"])
    check("pcb_J1_polarity_marks_minus_over_pad2_plus_over_pad1_0_8mm",
          len(marks) == 2 and any(mark_ok(t, '"-"', "-0.75") for t in marks)
          and any(mark_ok(t, '"+"', "0.75") for t in marks))
    pads = {pad[1]: pad for pad in children(j1_new, "pad")}
    check("pcb_J1_pad_positions_unchanged_pad2_left_GND_pad1_right_VIN",
          one(pads['"2"'], "at")[1:3] == ["-0.75", "0"] and value(pads['"2"'], "net") == "GND"
          and one(pads['"1"'], "at")[1:3] == ["0.75", "0"] and value(pads['"1"'], "net") == "VIN"
          and one(j1_new, "at")[1:] == ["180.513", "91.778"])

    old_seg, new_seg = keyed(children(before, "segment")), keyed(children(after, "segment"))
    expected_seg = {}
    for k, node in old_seg.items():
        if k == REMOVED:
            continue
        node = json.loads(json.dumps(node))
        if k in RENET:
            reader.replace_field(node, "net", "VIN")
        expected_seg[k] = node
    new_ids = set(new_seg) - set(old_seg)
    track_ok = len(new_ids) == 1
    if track_ok:
        t = new_seg[next(iter(new_ids))]
        track_ok = (one(t, "start")[1:] == ["172.385", "96.731"] and one(t, "end")[1:] == ["172.385", "93.048"]
                    and value(t, "width") == "1" and value(t, "layer") == "F.Cu" and value(t, "net") == "VIN")
    check("pcb_tracks_three_renetted_one_removed_one_added_rest_exact",
          track_ok and {k: v for k, v in new_seg.items() if k not in new_ids} == expected_seg,
          {"before": len(old_seg), "after": len(new_seg)})
    check("pcb_vias_texts_edges_unchanged",
          children(before, "via") == children(after, "via") and children(before, "gr_text") == children(after, "gr_text")
          and children(before, "gr_line") == children(after, "gr_line")
          and sum(value(e, "layer") == "Edge.Cuts" for e in children(after, "gr_line")) == 28)

    def zone_settings(tree):
        return {uid(z): [v for v in z if not (isinstance(v, list) and v[0] == "filled_polygon")]
                for z in children(tree, "zone")}
    check("pcb_zone_outlines_and_settings_unchanged", zone_settings(before) == zone_settings(after))
    excluded = {"footprint", "segment", "via", "gr_text", "zone", "gr_line"}
    stable = lambda tree: [v for v in tree if not (isinstance(v, list) and v[0] in excluded)]
    check("pcb_setup_layers_and_other_fields_unchanged", stable(before) == stable(after))
    sensor_patch = json.loads((REV / "validation/pcb-patch.json").read_text(encoding="utf-8"))
    check("pcb_sensor_stage_SEN1_tracks_and_via_still_present",
          all(k in new_seg and value(new_seg[k], "net") == "SEN1" for k in (t["uuid"] for t in sensor_patch["new_tracks"]))
          and any(uid(v) == sensor_patch["new_via"]["uuid"] for v in children(after, "via")))

    # ---------------- PCB (native) ----------------
    board = pcbnew.LoadBoard(str(args.pcb))
    pad_items = {(f.GetReference(), p.GetNumber()): p for f in board.GetFootprints() for p in f.Pads()}
    groups = {}
    for (ref, num), pad in pad_items.items():
        if pad.GetNetname():
            groups.setdefault(pad.GetNetname(), []).append((ref, num))
    for nodes in groups.values():
        nodes.sort()
    check("pcb_pad_nets_match_schematic_netlist", groups == after_nets)

    layers = (pcbnew.F_Cu, pcbnew.B_Cu)
    objects, shapes = {}, {}
    for (ref, num), pad in pad_items.items():
        if pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
            continue
        key = "pad:" + ref + "." + num
        objects[key] = pad.GetNetname()
        shapes[key] = {layer: pad.GetEffectiveShape(layer) for layer in layers if pad.IsOnLayer(layer)}
    for item in board.GetTracks():
        key = "copper:" + item.m_Uuid.AsString()
        objects[key] = item.GetNetname()
        shapes[key] = {layer: item.GetEffectiveShape() for layer in layers if item.IsOnLayer(layer)}
    for zone in board.Zones():
        for layer in layers:
            if zone.IsOnLayer(layer):
                polys = zone.GetFilledPolysList(layer)
                for index in range(polys.OutlineCount()):
                    key = "zone:" + zone.m_Uuid.AsString() + ":" + pcbnew.LayerName(layer) + ":" + str(index)
                    objects[key] = zone.GetNetname()
                    shapes[key] = {layer: polys.UnitSet(index)}
    graph = {k: set() for k in objects}
    cross = []
    keys = list(objects)
    for i, a in enumerate(keys):
        for b in keys[i + 1:]:
            common = shapes[a].keys() & shapes[b].keys()
            if any(shapes[a][layer].Collide(shapes[b][layer], 0) for layer in common):
                graph[a].add(b)
                graph[b].add(a)
                if objects[a] != objects[b]:
                    cross.append([a, objects[a], b, objects[b]])

    def reach(start):
        seen, pending = set(), [start]
        while pending:
            node = pending.pop()
            if node not in seen:
                seen.add(node)
                pending.extend(graph[node] - seen)
        return seen
    pads_from = lambda start: sorted(k[4:] for k in reach(start) if k.startswith("pad:"))
    check("copper_battery_plus_J1_1_reaches_only_switch_ON_pad_J2_3", pads_from("pad:J1.1") == ["J1.1", "J2.3"],
          pads_from("pad:J1.1"))
    check("copper_switch_common_J2_2_reaches_all_VCC_loads_and_nothing_else",
          pads_from("pad:J2.2") == ["J2.2", "J5.1", "J6.1", "R1.1", "U1.8", "U2.9"], pads_from("pad:J2.2"))
    check("copper_switch_OFF_pad_J2_1_isolated", pads_from("pad:J2.1") == ["J2.1"])
    check("copper_battery_minus_J1_2_reaches_GND_loads",
          {"U1.7", "U2.10", "J5.3", "J6.3", "J7.3", "R2.1"} <= set(pads_from("pad:J1.2")))
    check("copper_no_contact_between_different_nets_anywhere", not cross, cross)
    vin_objects = [k for k, n in objects.items() if n == "VIN"]
    check("copper_VIN_is_J1_1_J2_3_plus_exactly_four_tracks_all_connected",
          len(vin_objects) == 6 and len(reach("pad:J1.1") & set(vin_objects)) == 6)

    silk = {t.GetText(): t for t in board.FindFootprintByReference("J1").GraphicalItems()
            if isinstance(t, pcbnew.PCB_TEXT) and t.GetLayer() == pcbnew.F_SilkS}
    px = {num: pad_items[("J1", num)].GetPosition().x for num in ("1", "2")}
    check("silk_minus_centred_on_GND_pad2_plus_centred_on_battery_plus_pad1",
          set(silk) == {"BAT", "-", "+"} and silk["-"].GetPosition().x == px["2"] and silk["+"].GetPosition().x == px["1"]
          and pad_items[("J1", "2")].GetNetname() == "GND" and pad_items[("J1", "1")].GetNetname() == "VIN",
          {t: [round(i.GetPosition().x / 1e6, 3), round(i.GetPosition().y / 1e6, 3)] for t, i in silk.items()})
    check("J1_value_BAT_in_pcb_and_schematic", board.FindFootprintByReference("J1").GetValue() == "BAT"
          and props(new_j1[0])["Value"] == "BAT" if new_j1 else False)

    drc = json.loads(args.drc.read_text(encoding="utf-8"))
    old_drc = json.loads(args.before_drc.read_text(encoding="utf-8"))
    errors = [v for v in drc["violations"] if v["severity"] == "error"]
    check("native_DRC_zero_errors_unconnected_and_schematic_parity",
          not errors and not drc["unconnected_items"] and not drc["schematic_parity"],
          {"errors": len(errors), "warnings": len(drc["violations"]) - len(errors),
           "unconnected": len(drc["unconnected_items"]), "parity": len(drc["schematic_parity"])})
    check("DRC_warnings_same_12_pre_existing_text_height_items",
          Counter(map(signature, drc["violations"])) == Counter(map(signature, old_drc["violations"]))
          and len(drc["violations"]) == 12)
    check("all_audited_inputs_unchanged_during_read",
          report["input_sha256"] == {k: sha(p) for k, p in files.items()})
    report["passed"] = all(c["passed"] for c in report["checks"])
    report["summary"] = {"checks": len(report["checks"]), "passed": sum(c["passed"] for c in report["checks"])}
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"report": str(args.output), "passed": report["passed"], "summary": report["summary"],
                      "failed": [c for c in report["checks"] if not c["passed"]]}, ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
