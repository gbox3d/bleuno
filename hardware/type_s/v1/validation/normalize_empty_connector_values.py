"""Match KiCad's exported '~' empty value for J1-J9, without visible text.

The EasyEDA source and imported schematic have empty connector names/values.
KiCad exports their netlist values as '~'. Preserve that empty meaning, hide the
PCB Value fields, and retain the source evidence in this script's report.
"""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pcbnew

from normalize_import import Document, Node, tree, digest


ROOT = Path(__file__).resolve().parent.parent
REFERENCES = {f'J{i}' for i in range(1, 10)}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def safe(path):
    path = path.resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError('Path outside type_s/v1')
    return path


def fingerprint(document):
    result = []
    for item in document.root.items:
        if not isinstance(item, Node) or item.tag != 'footprint':
            result.append(tree(item))
            continue
        ref = document.property(item, 'Reference').atom(2)
        children = []
        for child in item.items:
            if ref in REFERENCES and isinstance(child, Node) and child.tag == 'property' and child.atom() == 'Value':
                # Preserve placement, layer, size, styling, UUID and all other
                # fields; only the empty token and visibility can differ.
                children.append(['property', 'Value', '<empty>', *[
                    tree(n) for n in child.items[3:] if not (isinstance(n, Node) and n.tag == 'hide')]])
            else:
                children.append(tree(child))
        result.append(children)
    return digest(result)


def main():
    if ROOT.name != 'v1' or ROOT.parent.name != 'type_s':
        raise ValueError('Only type_s/v1 is authorized')
    pcb = safe(ROOT / 'kicad/type_s_v1.kicad_pcb')
    protected = [safe(ROOT / 'kicad' / ('type_s_v1' + ext)) for ext in ('.kicad_sch', '.kicad_pro', '.kicad_dru')]
    hashes = {path: sha(path) for path in [pcb, *protected]}
    netlist = ET.parse(safe(ROOT / 'validation/netlist.xml')).getroot()
    exported = {c.attrib['ref']: c.findtext('value') for c in netlist.findall('./components/comp')}
    normalization = json.loads(safe(ROOT / 'validation/normalization-report.json').read_text(encoding='utf-8'))
    evidence = {row['reference']: row for row in normalization['references']}
    document = Document(pcb.read_text(encoding='utf-8'))
    before = fingerprint(document)
    rows = []
    for fp in document.root.nodes('footprint'):
        ref = document.property(fp, 'Reference').atom(2)
        if ref not in REFERENCES:
            continue
        original = evidence[ref]
        if original['schematic_displayed_value'] != '' or original['pcb_displayed_value'] != '' or exported[ref] != '~':
            raise ValueError(f'Connector {ref} is not an original empty value')
        value = document.property(fp, 'Value')
        old = value.atom(2)
        if old not in ('', '~'):
            raise ValueError(f'Refusing to replace a nonempty value on {ref}')
        document.set_property(fp, 'Value', '~')
        document.field(value, 'hide', 'yes', False)
        rows.append({'reference': ref, 'source_schematic_display': '', 'source_pcb_display': '',
                     'netlist_empty_value_token': '~', 'pcb_value_before': old, 'pcb_value_after': '~',
                     'pcb_value_hidden': True})
    if {row['reference'] for row in rows} != REFERENCES:
        raise ValueError('Expected exactly J1-J9')
    text = document.finish()
    if fingerprint(Document(text)) != before:
        raise ValueError('Geometry, zone fills, nets, silkscreen or other values changed')
    stage_dir = safe(ROOT / 'validation/parity-stage')
    stage_dir.mkdir(exist_ok=True)
    stage = safe(stage_dir / 'empty-values.kicad_pcb')
    stage.write_text(text, encoding='utf-8', newline='\n')
    loaded = pcbnew.LoadBoard(str(stage))
    by_ref = {f.GetReference(): f for f in loaded.GetFootprints()}
    for ref in REFERENCES:
        if by_ref[ref].GetValue() != '~' or by_ref[ref].Value().IsVisible():
            raise ValueError('Empty-value token is missing or visible')
    for ref in ('R1', 'R2'):
        if by_ref[ref].GetValue() != '10k\u03a9' or exported[ref] != '220k\u03a9':
            raise ValueError('Source resistor value disagreement was changed')
    if any(sha(path) != checksum for path, checksum in hashes.items()):
        raise RuntimeError('An input changed concurrently')
    changed = sha(stage) != hashes[pcb]
    if changed:
        backup = safe(stage_dir / 'before-empty-values.kicad_pcb')
        if not backup.exists():
            backup.write_bytes(pcb.read_bytes())
        stage.replace(pcb)
    report = {'connectors': rows, 'input_pcb_sha256': hashes[pcb], 'output_pcb_sha256': sha(pcb),
              'checks': {'only_empty_value_token_and_visibility_changed': True,
                         'all_geometry_nets_and_refilled_copper_unchanged': True,
                         'source_resistor_value_disagreements_preserved': True,
                         'schematic_project_and_rules_unchanged': True, 'native_reload': True}}
    report_path = safe(ROOT / 'validation/empty-value-report.json')
    if changed or not report_path.exists():
        report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(json.dumps({'changed': changed, 'connectors': len(rows), 'checks': report['checks']}))


if __name__ == '__main__':
    main()
