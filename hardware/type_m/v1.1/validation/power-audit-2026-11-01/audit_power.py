"""Read-only copper-contact power audit; use installed KiCad Python via uv.

The graph uses pads, actual tracks/vias, and each filled-zone polygon separately.
Net names do not create graph edges. PTH pads/vias join their copper layers.
No components are assumed to conduct internally, including the switch at J2.
"""
from collections import deque
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import pcbnew

HERE = Path(__file__).resolve().parent
TYPE_M = HERE.parents[2]
LAYERS = (pcbnew.F_Cu, pcbnew.B_Cu)


def xy(point):
    return [round(point.x / 1e6, 6), round(point.y / 1e6, 6)]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit(path):
    initial_sha = sha(path)
    board = pcbnew.LoadBoard(str(path))
    objects = {}
    shapes = {}
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                continue
            key = "pad:" + fp.GetReference() + "." + pad.GetNumber()
            objects[key] = {"kind": "pad", "reference": fp.GetReference(),
                            "pad": pad.GetNumber(), "uuid": pad.m_Uuid.AsString(),
                            "net": pad.GetNetname(), "xy_mm": xy(pad.GetPosition()),
                            "size_mm": xy(pad.GetSize()), "drill_mm": xy(pad.GetDrillSize())}
            shapes[key] = {layer: pad.GetEffectiveShape(layer) for layer in LAYERS if pad.IsOnLayer(layer)}
    for track in board.GetTracks():
        key = "copper:" + track.m_Uuid.AsString()
        is_via = isinstance(track, pcbnew.PCB_VIA)
        objects[key] = {"kind": "via" if is_via else "track", "uuid": track.m_Uuid.AsString(),
                        "net": track.GetNetname(), "start_mm": xy(track.GetStart()),
                        "end_mm": xy(track.GetEnd()),
                        "width_mm": (track.GetWidth(pcbnew.F_Cu) if is_via else track.GetWidth()) / 1e6,
                        "layers": [pcbnew.LayerName(layer) for layer in LAYERS if track.IsOnLayer(layer)]}
        shapes[key] = {layer: track.GetEffectiveShape() for layer in LAYERS if track.IsOnLayer(layer)}
    for zone in board.Zones():
        for layer in LAYERS:
            if not zone.IsOnLayer(layer):
                continue
            polygons = zone.GetFilledPolysList(layer)
            for index in range(polygons.OutlineCount()):
                key = "zone:" + zone.m_Uuid.AsString() + ":" + pcbnew.LayerName(layer) + ":" + str(index)
                part = polygons.UnitSet(index)
                objects[key] = {"kind": "filled_zone_polygon", "uuid": zone.m_Uuid.AsString(),
                                "net": zone.GetNetname(), "layer": pcbnew.LayerName(layer),
                                "polygon_index": index, "holes": part.HoleCount(0)}
                shapes[key] = {layer: part}
    graph = {key: set() for key in objects}
    contacts = []
    keys = list(objects)
    for index, left in enumerate(keys):
        for right in keys[index + 1:]:
            common = shapes[left].keys() & shapes[right].keys()
            touching_layers = [pcbnew.LayerName(layer) for layer in common
                               if shapes[left][layer].Collide(shapes[right][layer], 0)]
            if touching_layers:
                graph[left].add(right)
                graph[right].add(left)
                contacts.append({"a": left, "b": right, "layers": touching_layers})

    def reach(start):
        seen, pending = set(), [start]
        while pending:
            node = pending.pop()
            if node not in seen:
                seen.add(node)
                pending.extend(graph[node] - seen)
        return seen

    def path_to(start, target):
        queue, parents = deque([start]), {start: None}
        while queue:
            node = queue.popleft()
            if node == target:
                result = []
                while node is not None:
                    result.append(node)
                    node = parents[node]
                return result[::-1]
            for neighbor in sorted(graph[node]):
                if neighbor not in parents:
                    parents[neighbor] = node
                    queue.append(neighbor)
        return None

    requested = {
        "battery_positive": ("pad:J1.1", ["pad:U1.8", "pad:U2.9", "pad:J5.1", "pad:J6.1", "pad:J2.2", "pad:J2.3"]),
        "battery_negative": ("pad:J1.2", ["pad:U1.7", "pad:U2.10", "pad:J5.3", "pad:J6.3", "pad:J7.3"]),
    }
    results = {}
    for name, (start, targets) in requested.items():
        found = reach(start)
        results[name] = {"start": start, "reachable_pads": sorted(n for n in found if n.startswith("pad:")),
                         "paths": {target: path_to(start, target) for target in targets}}
    j2 = {"pad:J2." + str(pin): {"direct_copper_neighbors": sorted(graph["pad:J2." + str(pin)]),
                                  "reachable_pads": sorted(n for n in reach("pad:J2." + str(pin)) if n.startswith("pad:"))}
          for pin in (1, 2, 3)}
    cross_net = [edge for edge in contacts if objects[edge["a"]]["net"] != objects[edge["b"]]["net"]]
    relevant_nets = {"VCC", "GND", "VIN", "unconnected-(J2-Pad1)"}
    result = {"pcb": str(path.relative_to(TYPE_M)), "sha256": initial_sha,
              "sha256_after_read": sha(path), "power_paths": results, "switch_J2": j2,
              "power_pads": {key: data for key, data in objects.items()
                             if data["kind"] == "pad" and data["net"] in relevant_nets},
              "VCC_tracks": [data for data in objects.values() if data["kind"] == "track" and data["net"] == "VCC"],
              "GND_tracks": [data for data in objects.values() if data["kind"] == "track" and data["net"] == "GND"],
              "filled_zone_polygons": {key: data for key, data in objects.items() if data["kind"] == "filled_zone_polygon"},
              "copper_objects": objects,
              "physical_contacts": contacts,
              "different_net_copper_contacts": cross_net,
              "battery_plus_to_minus_path": path_to("pad:J1.1", "pad:J1.2")}
    return result


def main():
    versions = {"v1": audit(TYPE_M / "v1/kicad/type_m_v1.kicad_pcb"),
                "v1.1": audit(TYPE_M / "v1.1/kicad/type_m_v1_1.kicad_pcb")}
    v1, current = versions["v1"], versions["v1.1"]
    results = {
        "no_input_modified": all(v["sha256"] == v["sha256_after_read"] for v in versions.values()),
        "battery_plus_reaches_all_required_VCC_loads": all(
            path for target, path in current["power_paths"]["battery_positive"]["paths"].items() if target != "pad:J2.3"),
        "battery_minus_reaches_all_required_GND_loads": all(current["power_paths"]["battery_negative"]["paths"].values()),
        "battery_plus_has_no_copper_path_to_J2_3_VIN": current["power_paths"]["battery_positive"]["paths"]["pad:J2.3"] is None,
        "J2_3_VIN_isolated_copper_pad": not current["switch_J2"]["pad:J2.3"]["direct_copper_neighbors"],
        "J2_1_isolated_copper_pad": not current["switch_J2"]["pad:J2.1"]["direct_copper_neighbors"],
        "no_detected_plus_minus_short": current["battery_plus_to_minus_path"] is None,
        "no_different_net_copper_touch": not current["different_net_copper_contacts"],
        "same_power_pad_membership_as_v1": all(
            v1["power_paths"][name]["reachable_pads"] == current["power_paths"][name]["reachable_pads"]
            for name in current["power_paths"]),
        "same_VCC_track_geometry_as_v1": v1["VCC_tracks"] == current["VCC_tracks"],
        "same_power_pad_geometry_and_nets_as_v1": v1["power_pads"] == current["power_pads"],
    }
    report = {"generated_utc": datetime.now(timezone.utc).isoformat(),
              "method": "All copper contacts tested without using net names to join objects; each filled zone polygon tested independently. Through-hole pads/vias bridge F.Cu and B.Cu.",
              "scope_limits": ["KiCad saved design only; actual assembled PCB, soldering, cable polarity, and battery voltage have not been measured.",
                               "No switch or module internal behavior is assumed; a J2 contact closure can only connect its existing external pad nets.",
                               "Exact copper contact is checked at zero added clearance; this is not a new full DRC or current/voltage rating review."],
              "checks": results, "all_checks_passed": all(results.values()), "versions": versions}
    output = HERE / "power-audit.json"
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"report": str(output), "checks": results,
                      "current_power_paths": current["power_paths"], "current_switch": current["switch_J2"]}, indent=2))
    return 0 if report["all_checks_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
