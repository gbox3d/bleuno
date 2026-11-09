"""Read-only audit of this EasyEDA V2 board and its KiCad import.

Source-only inventory uses Python's standard library. --pcb additionally needs
KiCad's installed Python/pcbnew. This script writes only the requested JSON report.
Component identity is the EasyEDA primitive ID; duplicated display references do
not merge components. It checks the observed two-layer, front-side, straight-track
design and rejects unsupported geometry instead of silently claiming a match.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

MIL = 0.0254
TOL = 0.001  # mm; EasyEDA importer rounds coordinates to 500 nm.
ROOT = Path(__file__).resolve().parent.parent


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def records(archive, name):
    rows = [json.loads(line) for line in archive.read(name).decode('utf-8-sig').splitlines() if line.strip()]
    if not all(isinstance(row, list) and isinstance(row[0], str) for row in rows):
        raise ValueError(f'{name}: expected a V2 array snapshot, not an append-only editor log')
    return rows


def attributes(rows, pcb=False):
    result = defaultdict(dict)
    for row in rows:
        if row[0] == 'ATTR':
            parent, key, value = (row[3], row[7], row[8]) if pcb else (row[2], row[3], row[4])
            result[parent][key] = value
    return result


def polygon_points(data):
    if data[0] == 'R':
        _, x, y, width, height, angle, *extra = data
        if angle or any(extra):
            raise ValueError('Rotated/rounded rectangle needs a separate geometry check')
        return [[x, y], [x + width, y], [x + width, y - height], [x, y - height]]
    numbers = [value for value in data if value != 'L']
    if any(not isinstance(value, (int, float)) for value in numbers) or len(numbers) % 2:
        raise ValueError('Only straight polygon contours are supported by this board audit')
    points = [numbers[i:i + 2] for i in range(0, len(numbers), 2)]
    return points[:-1] if points[0] == points[-1] else points


def inventory(path, board_name):
    with zipfile.ZipFile(path) as archive:
        manifest = json.loads(archive.read('project.json'))
        board = manifest['boards'][board_name]
        pcb_path = f"PCB/{board['pcb']}.epcb"
        rows = records(archive, pcb_path)
        header = next(row[1] for row in rows if row[0] == 'HEAD')
        if not str(header.get('editorVersion', '')).startswith('2.'):
            raise ValueError('This audit is for the exported EasyEDA V2 snapshot')
        attrs = attributes(rows, pcb=True)
        components, pad_nodes = [], []
        for row in rows:
            if row[0] != 'COMPONENT':
                continue
            at = attrs[row[1]]
            pads = []
            net_rows = [n for n in rows if n[0] == 'PAD_NET' and n[1] == row[1] and str(n[2])]
            primitive_nets = {n[4]: n[3] for n in net_rows}
            number_nets = {str(n[2]): n[3] for n in net_rows}
            for p in records(archive, f"FOOTPRINT/{at['Footprint']}.efoo"):
                if p[0] != 'PAD':
                    continue
                number = str(p[5])
                net = primitive_nets.get(p[1], number_nets.get(number, p[3] or ''))
                pads.append({'id': p[1], 'number': number, 'net': net, 'layer': p[4],
                             'xy_mil': p[6:8], 'angle_deg': p[8], 'hole': p[9], 'shape': p[10]})
                pad_nodes.append([row[1], number, net])
            components.append({'id': row[1], 'reference': at['Designator'],
                               'unique_id': row[7].get('Unique ID'), 'xy_mil': row[4:6],
                               'angle_deg': row[6], 'layer': row[3], 'footprint_id': at['Footprint'],
                               'device_id': at.get('Device'), 'local_properties': row[7], 'pads': pads})
        sheets = []
        for sheet in manifest['schematics'][board['schematic']]['sheets']:
            name = f"SHEET/{board['schematic']}/{sheet['id']}.esch"
            sch_rows = records(archive, name)
            sch_attrs = attributes(sch_rows)
            parts = []
            for row in sch_rows:
                at = sch_attrs.get(row[1], {}) if len(row) > 1 and isinstance(row[1], str) else {}
                if row[0] != 'COMPONENT' or not at.get('Designator'):
                    continue
                device = manifest['devices'].get(at.get('Device'), {})
                merged = {**device.get('attributes', {}), **at}
                symbol = records(archive, f"SYMBOL/{merged['Symbol']}.esym")
                pin_attrs = attributes(symbol)
                pins = [{'number': pin_attrs[p[1]].get('Pin Number', pin_attrs[p[1]].get('NUMBER')),
                         'name': pin_attrs[p[1]].get('Pin Name', pin_attrs[p[1]].get('NAME')),
                         'type': pin_attrs[p[1]].get('Pin Type')} for p in symbol if p[0] == 'PIN']
                parts.append({'id': row[1], 'reference': merged['Designator'],
                              'unique_id': merged.get('Unique ID'), 'xy': row[3:5],
                              'value': merged.get('Value'), 'name': merged.get('Name'),
                              'footprint_id': merged.get('Footprint'), 'pins': pins})
            sheets.append({'path': name, 'record_counts': dict(Counter(r[0] for r in sch_rows)), 'parts': parts})
        outlines = [polygon_points(r[6]) for r in rows if r[0] == 'POLY' and r[4] == 11]
        cutouts = [polygon_points(poly) for r in rows if r[0] == 'FILL' and r[4] == 12 for poly in r[7]]
        pours = [{'id': r[1], 'net': r[3], 'layer': r[4], 'contours': [polygon_points(p) for p in r[8]]}
                 for r in rows if r[0] == 'POUR']
        return {'archive': path.name, 'sha256': sha(path), 'board': board_name,
                'pcb_uuid': board['pcb'], 'schematic_uuid': board['schematic'], 'header': header,
                'record_counts': dict(Counter(r[0] for r in rows)), 'components': components,
                'duplicate_display_references': {k: v for k, v in Counter(c['reference'] for c in components).items() if v > 1},
                'pad_nodes': sorted(pad_nodes), 'declared_nets': [r[1] for r in rows if r[0] == 'NET'],
                'tracks': [{'id': r[1], 'net': r[3], 'layer': r[4], 'start_mil': r[5:7],
                            'end_mil': r[7:9], 'width_mil': r[9]} for r in rows if r[0] == 'LINE'],
                'vias': [r for r in rows if r[0] == 'VIA'], 'arcs': [r for r in rows if r[0] == 'ARC'],
                'standalone_pads': [r for r in rows if r[0] == 'PAD'],
                'outlines_mil': outlines, 'cutouts_mil': cutouts, 'pours': pours, 'schematics': sheets,
                'scope': 'PCB pin assignments are compared independently of net spelling. Source schematic wire connectivity and filled-copper shape are not reconstructed.'}


def compare_connections(expected, actual):
    def groups(rows):
        nets, empty = defaultdict(set), set()
        for component, pad, net in rows:
            node = (component, pad)
            if net and not net.startswith('unconnected-('):
                nets[net].add(node)
            else:
                empty.add(node)
        return {frozenset(nodes): name for name, nodes in nets.items()}, empty
    source, se = groups(expected)
    imported, ie = groups(actual)
    missing = [{'net': name, 'nodes': sorted(nodes)} for nodes, name in source.items() if nodes not in imported]
    extra = [{'net': name, 'nodes': sorted(nodes)} for nodes, name in imported.items() if nodes not in source]
    return {'pass': not missing and not extra and se == ie,
            'matched_net_groups': len(source.keys() & imported.keys()),
            'net_name_map': {source[k]: imported[k] for k in sorted(source.keys() & imported.keys(), key=lambda k: source[k])},
            'unmatched_source_groups': missing, 'unmatched_imported_groups': extra,
            'source_only_unassigned': sorted(se - ie), 'imported_only_unassigned': sorted(ie - se)}


def compare_pcb(source, path):
    import pcbnew
    board = pcbnew.LoadBoard(str(path))
    footprints = list(board.GetFootprints())
    xy = lambda v: [pcbnew.ToMM(v.x), pcbnew.ToMM(v.y)]
    base = lambda c: [c['xy_mil'][0] * MIL, -c['xy_mil'][1] * MIL]
    candidates = []
    for c in source['components']:
        for f in footprints:
            if len(c['pads']) == len(list(f.Pads())):
                pos, origin = xy(f.GetPosition()), base(c)
                candidates.append([pos[0] - origin[0], pos[1] - origin[1]])
    def matches(offset):
        found, used = {}, set()
        for c in source['components']:
            pos = [v + o for v, o in zip(base(c), offset)]
            possible = [(math.dist(pos, xy(f.GetPosition())), i) for i, f in enumerate(footprints)
                        if i not in used and len(c['pads']) == len(list(f.Pads()))]
            if possible:
                error, i = min(possible)
                if error <= TOL:
                    found[c['id']] = footprints[i]
                    used.add(i)
        return found
    if not candidates:
        raise ValueError('No matching footprint-size candidates')
    offset = max(candidates, key=lambda o: len(matches(o)))
    matched = matches(offset)
    transform = lambda p: [p[0] * MIL + offset[0], -p[1] * MIL + offset[1]]
    issues, actual_nodes, identity = [], [], []
    for c in source['components']:
        f = matched.get(c['id'])
        if f is None:
            issues.append({'component': c['id'], 'reason': 'no unique position/pad-count match'})
            continue
        identity.append({'source_id': c['id'], 'source_unique_id': c['unique_id'],
                         'source_reference': c['reference'], 'kicad_reference': f.GetReference(),
                         'kicad_uuid': f.m_Uuid.AsString(), 'schematic_path': f.GetPath().AsString(),
                         'footprint_id': f.GetFPIDAsString()})
        if c['layer'] != 1 or f.GetLayer() != pcbnew.F_Cu:
            issues.append({'component': c['id'], 'reason': 'unsupported/nonmatching component side'})
        angle = math.radians(c['angle_deg'])
        for p in c['pads']:
            found = [pad for pad in f.Pads() if pad.GetNumber() == p['number']]
            if len(found) != 1:
                issues.append({'component': c['id'], 'pad': p['number'], 'matches': len(found)})
                continue
            actual = found[0]
            actual_nodes.append([c['id'], p['number'], actual.GetNetname()])
            lx, ly = p['xy_mil']
            expected = transform([c['xy_mil'][0] + lx * math.cos(angle) - ly * math.sin(angle),
                                  c['xy_mil'][1] + lx * math.sin(angle) + ly * math.cos(angle)])
            error = max(math.dist(expected, xy(actual.GetPosition())),
                        math.dist([v * MIL for v in p['hole'][1:3]], xy(actual.GetDrillSize())),
                        math.dist([v * MIL for v in p['shape'][1:3]], xy(actual.GetSize())))
            if error > TOL:
                issues.append({'component': c['id'], 'pad': p['number'], 'max_geometry_error_mm': error})
            shape = p['shape']
            expected_shape = (pcbnew.PAD_SHAPE_RECT if shape[0] == 'RECT' else
                              pcbnew.PAD_SHAPE_CIRCLE if shape[1] == shape[2] else pcbnew.PAD_SHAPE_OVAL)
            if shape[0] not in ('RECT', 'ELLIPSE', 'OVAL') or (shape[0] == 'RECT' and len(shape) > 3 and shape[3]):
                issues.append({'component': c['id'], 'pad': p['number'], 'reason': 'unsupported source pad shape'})
            elif actual.GetShape() != expected_shape or actual.GetAttribute() != pcbnew.PAD_ATTRIB_PTH:
                issues.append({'component': c['id'], 'pad': p['number'], 'reason': 'pad shape/PTH attribute mismatch'})
            if shape[1] != shape[2] or shape[0] == 'RECT':
                angle_error = abs((actual.GetOrientationDegrees() + c['angle_deg'] + p['angle_deg'] + 90) % 180 - 90)
                if angle_error > 0.001:
                    issues.append({'component': c['id'], 'pad': p['number'], 'orientation_error_deg': angle_error})
    connectivity = compare_connections(source['pad_nodes'], actual_nodes)
    net_map = connectivity['net_name_map']
    tracks = [t for t in board.GetTracks() if type(t).__name__ == 'PCB_TRACK']
    used = set()
    for t in source['tracks']:
        layer = pcbnew.F_Cu if t['layer'] == 1 else pcbnew.B_Cu
        start, end = transform(t['start_mil']), transform(t['end_mil'])
        possible = []
        for i, actual in enumerate(tracks):
            if i in used or actual.GetLayer() != layer or actual.GetNetname() != net_map.get(t['net'], t['net']):
                continue
            a, b = xy(actual.GetStart()), xy(actual.GetEnd())
            error = max(min(max(math.dist(start, a), math.dist(end, b)), max(math.dist(start, b), math.dist(end, a))),
                        abs(pcbnew.ToMM(actual.GetWidth()) - t['width_mil'] * MIL))
            possible.append((error, i))
        if not possible or min(possible)[0] > TOL:
            issues.append({'track': t['id'], 'reason': 'no matching endpoints/width/layer/net'})
        else:
            used.add(min(possible)[1])
    if len(used) != len(tracks):
        issues.append({'extra_imported_tracks': len(tracks) - len(used)})
    if source['vias'] or source['arcs'] or any(type(t).__name__ != 'PCB_TRACK' for t in board.GetTracks()):
        issues.append({'reason': 'This board audit expects no vias or copper arcs'})
    remaining = [f for f in footprints if f not in matched.values()]
    for p in source['standalone_pads']:
        possible = [(math.dist(transform(p[6:8]), xy(pad.GetPosition())), f, pad)
                    for f in remaining for pad in f.Pads()]
        if not possible or min(x[0] for x in possible) > TOL:
            issues.append({'standalone_pad': p[1], 'reason': 'missing position match'})
            continue
        _, f, pad = min(possible, key=lambda x: x[0])
        error = max(math.dist([v * MIL for v in p[9][1:3]], xy(pad.GetDrillSize())),
                    math.dist([v * MIL for v in p[10][1:3]], xy(pad.GetSize())))
        if error > TOL or pad.GetNetname() != (p[3] or ''):
            issues.append({'standalone_pad': p[1], 'reason': 'size/drill/net mismatch'})
        remaining.remove(f)
    if remaining:
        issues.append({'unmatched_footprints': [f.GetReference() for f in remaining]})
    expected_edges = []
    for poly in source['outlines_mil'] + source['cutouts_mil']:
        vertices = [transform(p) for p in poly]
        expected_edges.extend(zip(vertices, vertices[1:] + vertices[:1]))
    edges = [d for d in board.GetDrawings() if d.GetLayer() == pcbnew.Edge_Cuts]
    used_edges = set()
    for start, end in expected_edges:
        possible = [(min(max(math.dist(start, xy(d.GetStart())), math.dist(end, xy(d.GetEnd()))),
                         max(math.dist(start, xy(d.GetEnd())), math.dist(end, xy(d.GetStart())))), i)
                    for i, d in enumerate(edges) if i not in used_edges and d.GetShape() == pcbnew.SHAPE_T_SEGMENT]
        if not possible or min(possible)[0] > TOL:
            issues.append({'edge': [start, end], 'reason': 'outline/cutout segment not matched'})
        else:
            used_edges.add(min(possible)[1])
    if len(used_edges) != len(edges):
        issues.append({'extra_imported_edges': len(edges) - len(used_edges)})
    zones = list(board.Zones())
    canonical_layer = {pcbnew.F_Cu: 'F.Cu', pcbnew.B_Cu: 'B.Cu', pcbnew.Edge_Cuts: 'Edge.Cuts'}
    pour_summary = [{'net': z.GetNetname(), 'layer': canonical_layer.get(z.GetLayer(), str(z.GetLayer())),
                     'display_layer_name': board.GetLayerName(z.GetLayer()),
                     'outline_count': z.Outline().OutlineCount()} for z in zones]
    expected_zones = Counter((net_map.get(z['net'], z['net']), 'F.Cu' if z['layer'] == 1 else 'B.Cu') for z in source['pours'])
    if expected_zones != Counter((z['net'], z['layer']) for z in pour_summary):
        issues.append({'reason': 'zone count/net/layer mismatch'})
    for pour in source['pours']:
        layer = pcbnew.F_Cu if pour['layer'] == 1 else pcbnew.B_Cu
        candidates = [z for z in zones if z.GetLayer() == layer and z.GetNetname() == net_map.get(pour['net'], pour['net'])]
        if len(candidates) != 1 or len(pour['contours']) != 1:
            issues.append({'pour': pour['id'], 'reason': 'zone contour matching is ambiguous'})
            continue
        polyset = candidates[0].Outline()
        if polyset.OutlineCount() != 1:
            issues.append({'pour': pour['id'], 'reason': 'zone contour count differs'})
            continue
        contour = polyset.Outline(0)
        actual = [xy(contour.CPoint(i)) for i in range(contour.PointCount())]
        expected = [transform(p) for p in pour['contours'][0]]
        if len(expected) != len(actual) or any(min(math.dist(p, a) for a in actual) > TOL for p in expected):
            issues.append({'pour': pour['id'], 'reason': 'zone outline vertices differ', 'expected_mm': expected, 'actual_mm': actual})
    return {'pass': not issues and connectivity['pass'], 'source_translation_mm': offset,
            'tolerance_mm': TOL, 'component_identity': identity, 'connectivity': connectivity,
            'footprint_count': len(footprints), 'tracks_matched': len(used),
            'outline_and_cutout_segments_matched': len(used_edges), 'zones': pour_summary,
            'geometry_issues': issues, 'limits': 'Zone outlines/net/layer/count checked; filled-copper geometry and silkscreen graphics require visual/DRC review.'}


def compare_netlist(source, path, identity):
    tree = ET.parse(path).getroot()
    aliases = {row['kicad_reference']: row['source_id'] for row in identity}
    duplicate_refs = {ref: count for ref, count in Counter(row['kicad_reference'] for row in identity).items() if count > 1}
    if duplicate_refs:
        return {'pass': False, 'ambiguous_board_references': duplicate_refs,
                'reason': 'Give each physical component a unique KiCad reference before comparing schematic nets.'}
    if len(aliases) != len(source['components']):
        return {'pass': False, 'reason': 'Complete PCB component identity mapping is required.'}
    components = [{'reference': c.attrib['ref'], 'value': c.findtext('value'),
                   'footprint': c.findtext('footprint'), 'uuid': c.findtext('tstamps')}
                  for c in tree.findall('./components/comp')]
    rows = [[aliases[n.attrib['ref']], n.attrib['pin'], net.attrib['name']]
            for net in tree.findall('./nets/net') for n in net.findall('node') if n.attrib['ref'] in aliases]
    comparison = compare_connections(source['pad_nodes'], rows)
    comparison['components'] = components
    comparison['note'] = 'Original PCB assignment vs imported schematic connectivity. A mismatch can predate import; inspect the original schematic before changing a net.'
    return comparison


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=ROOT / 'source' / 'MovingTarget_MotorConT1_2026-10-31.epro')
    parser.add_argument('--board', default='MotorConT1')
    parser.add_argument('--pcb', type=Path)
    parser.add_argument('--netlist', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.netlist and not args.pcb:
        parser.error('--netlist requires --pcb for component identity mapping')
    inputs = [p for p in (args.source, args.pcb, args.netlist) if p]
    if args.output.resolve() in [p.resolve() for p in inputs]:
        parser.error('Report must not replace any input file')
    hashes = {p: sha(p) for p in inputs}
    report = {'source': inventory(args.source, args.board)}
    if args.pcb:
        report['pcb'] = compare_pcb(report['source'], args.pcb)
    if args.netlist:
        report['schematic_netlist'] = compare_netlist(report['source'], args.netlist, report['pcb']['component_identity'])
    if any(sha(p) != checksum for p, checksum in hashes.items()):
        raise RuntimeError('An input changed during read-only verification; rerun on stable saved files')
    report['input_sha256'] = {p.name: checksum for p, checksum in hashes.items()}
    checked = [section['pass'] for key, section in report.items() if key in ('pcb', 'schematic_netlist')]
    report['checked_comparisons_pass'] = all(checked) if checked else None
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'report': str(args.output), 'components': len(report['source']['components']),
                      'component_pads': len(report['source']['pad_nodes']),
                      'checked_comparisons_pass': report['checked_comparisons_pass']}))
    return 1 if checked and not all(checked) else 0


if __name__ == '__main__':
    raise SystemExit(main())
