"""Sync schematic custom fields and isolated NC net names, without geometry edits.

Use KiCad's installed Python through uv with --no-python-downloads.
Default is a report-only dry run; --write applies a natively verified staged file.
The sibling parser and read-only verifier are part of this repository.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pcbnew

from normalize_import import Document, Node, tree, digest
from verify_migration import inventory, compare_pcb

ROOT = Path(__file__).resolve().parent.parent


def file_hash(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def geometry(doc):
    """All parsed board content except footprint metadata and pad net names."""
    result = []
    for item in doc.root.items:
        if not isinstance(item, Node) or item.tag != 'footprint':
            result.append(tree(item))
            continue
        footprint = []
        for child in item.items:
            if isinstance(child, Node) and child.tag == 'property':
                continue
            if isinstance(child, Node) and child.tag == 'pad':
                footprint.append([tree(p) for p in child.items if not isinstance(p, Node) or p.tag != 'net'])
            else:
                footprint.append(tree(child))
        result.append(footprint)
    return digest(result)


def main():
    if ROOT.name != 'v1' or ROOT.parent.name != 'type_s' or ROOT.parent.parent.name != 'hardware':
        raise ValueError('This script is scoped to hardware/type_s/v1 only')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write', action='store_true')
    args = parser.parse_args()
    pcb_path = ROOT / 'kicad' / 'type_s_v1.kicad_pcb'
    netlist_path = ROOT / 'validation' / 'netlist.xml'
    protected = [ROOT / 'kicad' / ('type_s_v1' + ext) for ext in ('.kicad_sch', '.kicad_pro', '.kicad_dru')]
    protected_hashes = {p: file_hash(p) for p in protected}
    pcb_hash = file_hash(pcb_path)
    doc = Document(pcb_path.read_text(encoding='utf-8'))
    before_geometry = geometry(doc)
    footprints = {doc.property(f, 'Reference').atom(2): f for f in doc.root.nodes('footprint')}
    if len(footprints) != len(doc.root.nodes('footprint')):
        raise ValueError('Duplicate PCB references must be resolved first')
    xml = ET.parse(netlist_path).getroot()
    original_values = {ref: doc.property(f, 'Value').atom(2) for ref, f in footprints.items()}
    schematic_values = {c.attrib['ref']: c.findtext('value') or '' for c in xml.findall('./components/comp')}
    for ref in ('R1', 'R2'):
        if original_values[ref] != '10k\u03a9' or schematic_values[ref] != '220k\u03a9':
            raise ValueError(f'Expected original source value disagreement on {ref}')
    field_changes, net_changes = [], []
    for component in xml.findall('./components/comp'):
        ref = component.attrib['ref']
        footprint = footprints[ref]
        expected_fields = {p.attrib['name']: p.attrib.get('value', '') for p in component.findall('property')}
        expected_fields.update({p.attrib['name']: p.text or '' for p in component.findall('./fields/field')})
        expected_fields['Datasheet'] = component.findtext('datasheet') or ''
        for key, value in expected_fields.items():
            if key in ('Sheetname', 'Sheetfile', 'Reference', 'Value', 'Footprint'):
                continue
            existing = doc.property(footprint, key)
            old = existing.atom(2) if existing else None
            if old == value or (existing is None and not value):
                continue
            if key == 'Original Reference' and old:
                doc.add_hidden_property(footprint, 'Original PCB Reference', old, pcb=True)
            if existing:
                doc.set_property(footprint, key, value)
            else:
                doc.add_hidden_property(footprint, key, value, pcb=True)
            field_changes.append({'reference': ref, 'field': key, 'before': old, 'after': value})
    singleton_names = set()
    for net in xml.findall('./nets/net'):
        name = net.attrib['name']
        if not name.startswith('unconnected-('):
            continue
        nodes = net.findall('node')
        if len(nodes) != 1 or name in singleton_names:
            raise ValueError('NC net must identify exactly one distinct schematic pin')
        singleton_names.add(name)
        ref, number = nodes[0].attrib['ref'], nodes[0].attrib['pin']
        matching = [p for p in footprints[ref].nodes('pad') if p.atom() == number]
        if len(matching) != 1:
            raise ValueError(f'Expected exactly one physical pad for {ref}.{number}')
        pad = matching[0]
        previous = pad.node('net')
        old = previous.atom() if previous else ''
        if old == name:
            continue
        if old:
            raise ValueError(f'Refusing to replace an assigned net on {ref}.{number}: {old}')
        if previous:
            doc.field(pad, 'net', name)
        else:
            doc.append(pad, '(net ' + json.dumps(name) + ')')
        net_changes.append({'reference': ref, 'pad': number, 'before': old, 'after': name})
    text = doc.finish()
    after = Document(text)
    after_values = {after.property(f, 'Reference').atom(2): after.property(f, 'Value').atom(2)
                    for f in after.root.nodes('footprint')}
    if original_values != after_values:
        raise ValueError('PCB component values changed; original disagreements must remain')
    if geometry(after) != before_geometry:
        raise ValueError('Geometry, copper fills, or non-metadata board content changed')
    stage_dir = ROOT / 'validation' / 'parity-stage'
    stage_dir.mkdir(exist_ok=True)
    stage = stage_dir / 'type_s_v1.kicad_pcb'
    stage.write_text(text, encoding='utf-8')
    board = pcbnew.LoadBoard(str(stage))
    usage = Counter(p.GetNetname() for f in board.GetFootprints() for p in f.Pads() if p.GetNetname() in singleton_names)
    if len(singleton_names) != 6 or set(usage) != singleton_names or any(count != 1 for count in usage.values()):
        raise ValueError('A generated NC net joined multiple pads or was lost on native reload')
    if any(item.GetNetname() in singleton_names for item in list(board.GetTracks()) + list(board.Zones())):
        raise ValueError('A generated NC net was attached to routing or a copper zone')
    source = inventory(ROOT / 'source' / 'ZeroFactorySensorBoard_Board_DHT_2026-11-01.epro', 'Board_DHT')
    comparison = compare_pcb(source, stage)
    if not comparison['pass'] or comparison['connectivity']['matched_net_groups'] != 12:
        raise ValueError('Source geometry or the 12 original connection groups were not preserved')
    if file_hash(pcb_path) != pcb_hash or any(file_hash(p) != h for p, h in protected_hashes.items()):
        raise RuntimeError('A design input changed concurrently; refusing to replace the PCB')
    report = {'applied': args.write, 'input_pcb_sha256': pcb_hash, 'output_pcb_sha256': file_hash(stage),
              'netlist_sha256': file_hash(netlist_path), 'field_changes': field_changes, 'nc_net_changes': net_changes,
              'preserved_source_value_differences': [{'reference': ref, 'schematic': schematic_values[ref], 'pcb': original_values[ref]} for ref in ('R1', 'R2')],
              'isolated_nc_net_pad_counts': dict(sorted(usage.items())),
              'checks': {'all_board_geometry_and_zone_fills_unchanged': True,
                         'original_12_connection_groups_unchanged': True,
                         'nc_nets_have_one_pad_no_track_no_zone': True,
                         'schematic_project_rules_unchanged': True, 'native_reload': True},
              'geometry_sha256': before_geometry}
    report_path = ROOT / 'validation' / 'parity-metadata-report.json'
    if report_path.exists() and (field_changes or net_changes):
        previous = json.loads(report_path.read_text(encoding='utf-8'))
        if previous.get('applied') and previous.get('output_pcb_sha256') == pcb_hash:
            report['previous_passes'] = previous.get('previous_passes', []) + [
                {k: v for k, v in previous.items() if k != 'previous_passes'}]
    if args.write and (field_changes or net_changes):
        backup = stage_dir / 'before-parity.kicad_pcb'
        if not backup.exists():
            backup.write_bytes(pcb_path.read_bytes())
        stage.replace(pcb_path)
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'applied': args.write, 'field_changes': len(field_changes),
                      'nc_net_changes': len(net_changes), 'checks': report['checks']}))


if __name__ == '__main__':
    main()
