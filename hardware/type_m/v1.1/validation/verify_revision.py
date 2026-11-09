"""Read-only, narrowly allowlisted audit of type_m v1 -> v1.1.

Run using KiCad's already installed Python through uv --no-python-downloads.
Only the requested JSON report is written. No KiCad design is saved or changed.
The baseline is the sibling v1 directory, never an inferred external path.
"""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import pcbnew

ROOT = Path(__file__).resolve().parent.parent
OLD = ROOT.parent / "v1"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sexpr(text):
    stack = [[]]
    for token in re.findall(r'\(|\)|"(?:\\.|[^"\\])*"|[^\s()]+', text):
        if token == "(":
            node = []
            stack[-1].append(node)
            stack.append(node)
        elif token == ")":
            if len(stack) == 1:
                raise ValueError("Unexpected closing parenthesis")
            stack.pop()
        else:
            stack[-1].append(token)
    if len(stack) != 1 or len(stack[0]) != 1:
        raise ValueError("Unbalanced or multiple root S-expressions")
    return stack[0][0]


def children(node, tag):
    return [v for v in node if isinstance(v, list) and v and v[0] == tag]


def one(node, tag):
    values = children(node, tag)
    if len(values) != 1:
        raise ValueError((tag, len(values)))
    return values[0]


def value(node, tag):
    token = one(node, tag)[1]
    return json.loads(token) if token.startswith('"') else token


def uid(node):
    return value(node, "uuid")


def replace_field(node, tag, text):
    old = children(node, tag)
    replacement = [tag, json.dumps(text)]
    if old:
        node[node.index(old[0])] = replacement
    else:
        node.insert(node.index(one(node, "uuid")), replacement)


def keyed(nodes):
    result = {uid(n): n for n in nodes}
    if len(result) != len(nodes):
        raise ValueError("Duplicate object UUID")
    return result


def nets(path):
    root = ET.parse(path).getroot()
    return {n.get("name"): sorted((x.get("ref"), x.get("pin"))
                                  for x in n.findall("node"))
            for n in root.findall("nets/net")}


def expected_nets(old):
    result = {name: list(nodes) for name, nodes in old.items()}
    result["SEN1"] = sorted(result["SEN1"] + [("U1", "15")])
    result.pop("unconnected-(U1-GPIO21-Pad15)")
    result["unconnected-(U1-GPIO21-Pad16)"] = result.pop("unconnected-(U1-GPIO22-Pad16)")
    return result


def fix_symbol_pin_names(symbol):
    for child in children(symbol, "symbol"):
        for pin in children(child, "pin"):
            number = value(pin, "number")
            if number in ("15", "16"):
                one(pin, "name")[1] = json.dumps("GPIO20" if number == "15" else "GPIO21")


def fix_frame_text(symbol):
    count = Counter()
    for child in children(symbol, "symbol"):
        for text in children(child, "text"):
            replacements = {'"V1.0"': '"v1.1"', '"2026-03-16"': '"2026-11-01"'}
            if text[1] in replacements:
                count[text[1]] += 1
                text[1] = replacements[text[1]]
    if count != Counter({'"V1.0"': 1, '"2026-03-16"': 1}):
        raise ValueError("Unexpected baseline frame revision text")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        default=ROOT / "validation/revision-comparison.json")
    args = parser.parse_args()
    report = {"generated_utc": datetime.now(timezone.utc).isoformat(),
              "kicad_version": pcbnew.GetBuildVersion(), "checks": [],
              "allowed_changes": [
                  "Project/library path prefix type_m_v1 -> type_m_v1_1 only",
                  "U1.15 net becomes SEN1 and pinfunction becomes GPIO20",
                  "U1.16 remains singleton NC; net/pinfunction use GPIO21",
                  "SCH and local library pin15/16 names GPIO20/GPIO21; one SEN1 label at U1.15",
                  "Imported frame Version v1.1 and Update Date 2026-11-01 in displayed text and matching instance fields",
                  "Exactly six 0.254 mm SEN1 segments and one 0.6/0.3 mm through via, as pcb-patch.json",
                  "Exactly one F.SilkS revision text type_m v1.1, as pcb-patch.json",
                  "Native regenerated zone filled_polygon geometry; all zone settings/outlines unchanged",
              ]}

    def check(name, ok, detail=None):
        item = {"name": name, "passed": bool(ok)}
        if detail is not None:
            item["detail"] = detail
        report["checks"].append(item)

    files = {"v1_pcb": OLD / "kicad/type_m_v1.kicad_pcb",
             "revision_pcb": ROOT / "kicad/type_m_v1_1.kicad_pcb",
             "revision_schematic": ROOT / "kicad/type_m_v1_1.kicad_sch",
             "revision_symbol_library": ROOT / "kicad/type_m_v1_1.kicad_sym",
             "revision_project": ROOT / "kicad/type_m_v1_1.kicad_pro",
             "revision_rules": ROOT / "kicad/type_m_v1_1.kicad_dru",
             "netlist": ROOT / "validation/netlist.xml",
             "drc": ROOT / "validation/drc.json",
             "erc": ROOT / "validation/erc.json",
             "patch_manifest": ROOT / "validation/pcb-patch.json"}
    report["input_sha256"] = {key: sha(path) for key, path in files.items()}
    originals = json.loads((ROOT / "validation/v1-input-hashes.json").read_text(encoding="utf-8-sig"))
    changed = [v["path"] for v in originals if sha(OLD / v["path"]).lower() != v["sha256"].lower()]
    check("v1_snapshot_hashes_preserved", not changed,
          {"files": len(originals), "changed": changed})

    before = sexpr(files["v1_pcb"].read_text(encoding="utf-8").replace("type_m_v1", "type_m_v1_1"))
    after = sexpr(files["revision_pcb"].read_text(encoding="utf-8"))
    patch = json.loads(files["patch_manifest"].read_text(encoding="utf-8"))
    old_fp = keyed(children(before, "footprint"))
    new_fp = keyed(children(after, "footprint"))
    for fp in old_fp.values():
        reference = next(json.loads(p[2]) for p in children(fp, "property")
                         if p[1] == '"Reference"')
        if reference == "U1":
            for pad in children(fp, "pad"):
                if pad[1] == '"15"':
                    replace_field(pad, "net", "SEN1")
                    replace_field(pad, "pinfunction", "GPIO20")
                elif pad[1] == '"16"':
                    replace_field(pad, "net", "unconnected-(U1-GPIO21-Pad16)")
                    replace_field(pad, "pinfunction", "GPIO21")
    check("all_footprint_and_pad_fields_exact_except_allowed_two_pins", old_fp == new_fp,
          {"footprints_including_mounting_holes": len(new_fp),
           "changed_unexpected_uuids": sorted(k for k in old_fp.keys() | new_fp.keys()
                                              if old_fp.get(k) != new_fp.get(k))})
    old_tracks = keyed(children(before, "segment"))
    new_tracks = keyed(children(after, "segment"))
    check("all_118_original_track_uuids_geometry_layers_widths_nets_preserved",
          len(old_tracks) == 118 and all(new_tracks.get(k) == v for k, v in old_tracks.items()))
    added = set(new_tracks) - set(old_tracks)
    expected = {v["uuid"] for v in patch["new_tracks"]}
    track_errors = []
    for track in patch["new_tracks"]:
        node = new_tracks.get(track["uuid"])
        if node is None:
            track_errors.append(track["uuid"])
            continue
        actual = {"start_mm": list(map(float, one(node, "start")[1:])),
                  "end_mm": list(map(float, one(node, "end")[1:])),
                  "width_mm": float(value(node, "width")),
                  "layer": value(node, "layer"), "uuid": uid(node)}
        if actual != track or value(node, "net") != "SEN1":
            track_errors.append(track["uuid"])
    check("exactly_six_manifest_segments_on_SEN1", added == expected and len(added) == 6 and not track_errors,
          {"added": len(added), "mismatches": track_errors})
    vias = children(after, "via")
    via = patch["new_via"]
    via_ok = not children(before, "via") and len(vias) == 1
    if via_ok:
        n = vias[0]
        via_ok = (uid(n) == via["uuid"] and value(n, "net") == "SEN1"
                  and list(map(float, one(n, "at")[1:])) == via["xy_mm"]
                  and float(value(n, "size")) == via["diameter_mm"]
                  and float(value(n, "drill")) == via["drill_mm"]
                  and one(n, "layers")[1:] == ['"F.Cu"', '"B.Cu"'])
    check("exactly_one_manifest_0_6_0_3_through_via_on_SEN1", via_ok)
    original_text = keyed(children(before, "gr_text"))
    revision_text = keyed(children(after, "gr_text"))
    text_add = set(revision_text) - set(original_text)
    text_id = patch["revision_text"]["uuid"]
    text_ok = all(revision_text.get(k) == v for k, v in original_text.items()) and text_add == {text_id}
    if text_ok:
        n = revision_text[text_id]
        text_ok = json.loads(n[1]) == "type_m v1.1" and value(n, "layer") == "F.SilkS"
    check("only_one_new_front_silkscreen_revision_text", text_ok)

    def zone_settings(tree):
        return {uid(z): [v for v in z if not (isinstance(v, list) and v[0] == "filled_polygon")]
                for z in children(tree, "zone")}

    check("zone_outlines_and_all_nonfill_settings_preserved", zone_settings(before) == zone_settings(after))
    excluded = {"footprint", "segment", "via", "gr_text", "zone"}
    stable = lambda tree: [v for v in tree if not (isinstance(v, list) and v[0] in excluded)]
    check("all_remaining_board_fields_and_28_edges_exact", stable(before) == stable(after),
          {"edge_count": sum(value(e, "layer") == "Edge.Cuts" for e in children(after, "gr_line"))})
    check("design_rules_unchanged", (OLD / "kicad/type_m_v1.kicad_dru").read_bytes()
          == (ROOT / "kicad/type_m_v1_1.kicad_dru").read_bytes())
    old_project = json.loads((OLD / "kicad/type_m_v1.kicad_pro").read_text(encoding="utf-8"))
    new_project = json.loads(files["revision_project"].read_text(encoding="utf-8"))
    check("project_design_rules_and_net_settings_unchanged",
          old_project["board"]["design_settings"] == new_project["board"]["design_settings"]
          and old_project["net_settings"] == new_project["net_settings"])

    for extension in ("kicad_sch", "kicad_sym"):
        original = sexpr((OLD / "kicad" / ("type_m_v1." + extension)).read_text(encoding="utf-8")
                         .replace("type_m_v1", "type_m_v1_1"))
        current = sexpr((ROOT / "kicad" / ("type_m_v1_1." + extension)).read_text(encoding="utf-8"))
        library = one(original, "lib_symbols") if extension == "kicad_sch" else original
        name = "type_m_v1_1:ESP32C3 Supermini" if extension == "kicad_sch" else "ESP32C3 Supermini"
        symbols = [n for n in children(library, "symbol") if json.loads(n[1]) == name]
        if len(symbols) != 1:
            raise ValueError("Expected exactly one U1 library symbol")
        fix_symbol_pin_names(symbols[0])
        frame_name = "type_m_v1_1:Drawing-Symbol_A4" if extension == "kicad_sch" else "Drawing-Symbol_A4"
        frames = [n for n in children(library, "symbol") if json.loads(n[1]) == frame_name]
        if len(frames) != 1:
            raise ValueError("Expected exactly one imported drawing frame")
        fix_frame_text(frames[0])
        if extension == "kicad_sch":
            frame_instances = [n for n in children(original, "symbol")
                               if any(p[1:3] == ['"Reference"', '"#FRAME1"']
                                      for p in children(n, "property"))]
            if len(frame_instances) != 1:
                raise ValueError("Expected exactly one #FRAME1 instance")
            for prop in children(frame_instances[0], "property"):
                if prop[1] == '"Version"':
                    prop[2] = '"v1.1"'
                elif prop[1] == '"@Update Date"':
                    prop[2] = '"2026-11-01"'
            labels = [n for n in children(current, "global_label")
                      if uid(n) == "9ece6bf8-2d27-59ea-ad5b-773cbe1ad76a"]
            label_ok = len(labels) == 1
            if label_ok:
                label = labels[0]
                label_ok = (json.loads(label[1]) == "SEN1"
                            and list(map(float, one(label, "at")[1:])) == [31.75, 69.85, 180.0])
                current.remove(label)
            check("one_new_SEN1_label_at_exact_U1_pin15_anchor", label_ok)
        check(extension + "_only_pin_names_label_and_frame_revision_changed", original == current)
    report["revision_metadata"] = {"version": "v1.1", "update_date": "2026-11-01",
                                   "original_board_name_retained": "MotorConT1",
                                   "original_project_name_retained": "MovingTarget"}

    old_board = pcbnew.LoadBoard(str(files["v1_pcb"]))
    new_board = pcbnew.LoadBoard(str(files["revision_pcb"]))
    new_pads = {(fp.GetReference(), p.GetNumber()): p for fp in new_board.GetFootprints() for p in fp.Pads()}
    count_mount = sum(ref in ("MP1", "MP2") for ref, _ in new_pads)
    check("52_component_pads_plus_two_mounting_holes", len(new_pads) == 54 and count_mount == 2)
    old_groups, new_groups = defaultdict(list), defaultdict(list)
    for board, groups in [(old_board, old_groups), (new_board, new_groups)]:
        for fp in board.GetFootprints():
            for p in fp.Pads():
                if p.GetNetname():
                    groups[p.GetNetname()].append((fp.GetReference(), p.GetNumber()))
        for nodes in groups.values():
            nodes.sort()
    check("all_pcb_net_groups_match_exact_expected_change", expected_nets(old_groups) == dict(new_groups))
    old_netlist = nets(OLD / "validation/netlist.xml")
    new_netlist = nets(files["netlist"])
    check("all_schematic_net_groups_match_exact_expected_change", expected_nets(old_netlist) == new_netlist)
    check("pcb_and_schematic_node_memberships_match", dict(new_groups) == new_netlist)
    check("SEN1_exactly_J7_2_R3_2_U1_15", new_netlist.get("SEN1") == [("J7", "2"), ("R3", "2"), ("U1", "15")])
    check("updated_pad_functions_GPIO20_GPIO21", new_pads[("U1", "15")].GetPinFunction() == "GPIO20"
          and new_pads[("U1", "16")].GetPinFunction() == "GPIO21")

    conductors = []
    for node, pad in new_pads.items():
        if pad.GetNetname() == "SEN1":
            conductors.append(("pad:" + ".".join(node), pad, True))
    for item in new_board.GetTracks():
        if item.GetNetname() == "SEN1":
            conductors.append((item.m_Uuid.AsString(), item, False))
    adjacency = {name: set() for name, _, _ in conductors}
    def shape(obj, ispad, layer):
        return obj.GetEffectiveShape(layer) if ispad else obj.GetEffectiveShape()
    for i, (name, a, apad) in enumerate(conductors):
        for other, z, zpad in conductors[i + 1:]:
            if any(a.IsOnLayer(layer) and z.IsOnLayer(layer)
                   and shape(a, apad, layer).Collide(shape(z, zpad, layer), 0)
                   for layer in (pcbnew.F_Cu, pcbnew.B_Cu)):
                adjacency[name].add(other)
                adjacency[other].add(name)
    reached, remaining = set(), [conductors[0][0]]
    while remaining:
        name = remaining.pop()
        if name not in reached:
            reached.add(name)
            remaining.extend(adjacency[name] - reached)
    check("SEN1_actual_copper_graph_connects_all_3_pads_10_tracks_1_via",
          len(conductors) == 14 and len(reached) == 14,
          {"conductors": len(conductors), "reachable": len(reached),
           "disconnected": sorted(set(adjacency) - reached)})
    unexpected_contacts = []
    for node, pad in new_pads.items():
        if pad.GetNetname() == "SEN1":
            continue
        for name, obj, ispad in conductors:
            if any(obj.IsOnLayer(layer) and pad.IsOnLayer(layer)
                   and shape(obj, ispad, layer).Collide(pad.GetEffectiveShape(layer), 0)
                   for layer in (pcbnew.F_Cu, pcbnew.B_Cu)):
                unexpected_contacts.append([name, ".".join(node)])
    check("SEN1_no_physical_contact_with_any_other_pad", not unexpected_contacts, unexpected_contacts)

    drc = json.loads(files["drc"].read_text(encoding="utf-8"))
    errors = [v for v in drc["violations"] if v["severity"] == "error"]
    warnings = [v for v in drc["violations"] if v["severity"] == "warning"]
    old_drc = json.loads((OLD / "validation/drc.json").read_text(encoding="utf-8"))
    signature = lambda v: (v["type"], v["severity"], tuple(x["uuid"] for x in v["items"]))
    check("native_DRC_zero_errors_unconnected_and_schematic_parity", not errors
          and not drc["unconnected_items"] and not drc["schematic_parity"],
          {"errors": len(errors), "warnings": len(warnings),
           "unconnected": len(drc["unconnected_items"]), "parity": len(drc["schematic_parity"])})
    check("same_12_existing_text_height_warnings", len(warnings) == 12
          and Counter(map(signature, warnings)) == Counter(map(signature, old_drc["violations"])))
    erc = json.loads(files["erc"].read_text(encoding="utf-8"))
    ev = [v for s in erc["sheets"] for v in s["violations"]]
    report["erc_existing_symbol_limitations"] = dict(Counter(v["severity"] for v in ev))
    old_erc = json.loads((OLD / "validation/erc.json").read_text(encoding="utf-8"))
    old_ev = [v for s in old_erc["sheets"] for v in s["violations"]]
    old_signatures, new_signatures = Counter(map(signature, old_ev)), Counter(map(signature, ev))
    removed, added = old_signatures - new_signatures, new_signatures - old_signatures
    u1_pin15, r3_pin2 = "e14dac4d-ad3b-4c74-ad1c-3ebb50698d03", "f95872fe-a011-4d08-9a40-b78996c91d54"
    check("ERC_only_U1_pin15_disconnected_error_replaced_by_existing_pin_type_warning",
          removed == Counter({("pin_not_connected", "error", (u1_pin15,)): 1})
          and added == Counter({("pin_to_pin", "warning", (u1_pin15, r3_pin2)): 1}),
          {"before": dict(Counter(v["severity"] for v in old_ev)),
           "after": report["erc_existing_symbol_limitations"]})
    check("all_audited_inputs_unchanged_during_read", report["input_sha256"]
          == {key: sha(path) for key, path in files.items()})
    report["passed"] = all(v["passed"] for v in report["checks"])
    report["summary"] = {"checks": len(report["checks"]),
                         "passed": sum(v["passed"] for v in report["checks"])}
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"report": str(args.output), "passed": report["passed"],
                      "summary": report["summary"],
                      "failed": [v for v in report["checks"] if not v["passed"]]}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
