"""Replace only the escaped-colon MCU footprint identifier with a portable name.

Run after normalize_import.py with installed KiCad Python via uv/no downloads.
The project/rules, component values, circuitry, routing and all footprint shapes
remain unchanged. This script is idempotent and never runs the full normalizer.
"""
import hashlib
import json
from pathlib import Path

import pcbnew

from normalize_import import Document, Atom, Node, tree, digest


ROOT = Path(__file__).resolve().parent.parent
KICAD = ROOT / 'kicad'
PROJECT = 'type_s_v1'
OLD_NAME = 'ESP32-C3_SUPERMINI_TH{colon}MODULE_ESP32-C3_SUPERMINI'
NEW_NAME = 'ESP32C3_SUPERMINI_CUSTOM'
OLD_ID, NEW_ID = PROJECT + ':' + OLD_NAME, PROJECT + ':' + NEW_NAME
MCU_UUID = 'a4113187-8991-4bf9-8b8c-b8dcc5c31a94'


def safe(path):
    path = path.resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError('Path outside type_s/v1')
    return path


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def masked(node):
    if isinstance(node, Atom):
        if node.value in (OLD_NAME, NEW_NAME):
            return '<MCU-footprint-name>'
        if node.value in (OLD_ID, NEW_ID):
            return '<MCU-library-id>'
        return node.value
    return [masked(x) for x in node.items]


def rename_identifier(document, kind):
    changes = 0
    if kind in ('schematic', 'symbol_library'):
        def visit(node):
            nonlocal changes
            if node.tag == 'property' and node.atom() == 'Footprint' and node.atom(2) == OLD_ID:
                document.atom(node.items[2], NEW_ID)
                changes += 1
            for child in node.items:
                if isinstance(child, Node):
                    visit(child)
        visit(document.root)
    elif kind == 'pcb':
        footprint = next(f for f in document.root.nodes('footprint') if f.node('uuid').atom() == MCU_UUID)
        if footprint.atom() not in (OLD_ID, NEW_ID):
            raise ValueError('Unexpected MCU footprint ID')
        if footprint.atom() == OLD_ID:
            document.atom(footprint.items[1], NEW_ID)
            changes += 1
    else:
        if document.root.atom() not in (OLD_NAME, NEW_NAME):
            raise ValueError('Unexpected library footprint name')
        if document.root.atom() == OLD_NAME:
            document.atom(document.root.items[1], NEW_NAME)
            changes += 1
    result = document.finish()
    if digest(masked(document.root)) != digest(masked(Document(result).root)):
        raise ValueError('Content other than the MCU identifier changed')
    return result, changes


def main():
    if ROOT.name != 'v1' or ROOT.parent.name != 'type_s' or ROOT.parent.parent.name != 'hardware':
        raise ValueError('Only hardware/type_s/v1 is authorized')
    protected = [safe(KICAD / (PROJECT + ext)) for ext in ('.kicad_pro', '.kicad_dru')]
    protected_hashes = {p: sha(p) for p in protected}
    library_dir = safe(KICAD / (PROJECT + '.pretty'))
    old_module, new_module = [safe(library_dir / (name + '.kicad_mod')) for name in (OLD_NAME, NEW_NAME)]
    if old_module.exists() and new_module.exists():
        raise ValueError('Both old and new MCU footprints exist')
    module = old_module if old_module.exists() else new_module
    inputs = [(safe(KICAD / (PROJECT + '.kicad_sch')), 'schematic'),
              (safe(KICAD / (PROJECT + '.kicad_sym')), 'symbol_library'),
              (safe(KICAD / (PROJECT + '.kicad_pcb')), 'pcb'), (module, 'module')]
    stage = safe(ROOT / 'validation/parity-stage/mcu-library')
    stage.mkdir(parents=True, exist_ok=True)
    stage_library = safe(stage / (PROJECT + '.pretty'))
    stage_library.mkdir(exist_ok=True)
    input_hashes = {p: sha(p) for p, _ in inputs}
    rows, files = [], []
    for path, kind in inputs:
        document = Document(path.read_text(encoding='utf-8'))
        result, changes = rename_identifier(document, kind)
        destination = new_module if kind == 'module' else path
        staged_path = safe(stage_library / new_module.name) if kind == 'module' else safe(stage / path.name)
        staged_path.write_text(result, encoding='utf-8', newline='\n')
        rows.append({'kind': kind, 'source': str(path.relative_to(ROOT)), 'destination': str(destination.relative_to(ROOT)),
                     'identifier_changes': changes, 'input_sha256': input_hashes[path], 'output_sha256': sha(staged_path)})
        files.append((staged_path, destination))
    changes = {row['kind']: row['identifier_changes'] for row in rows}
    if changes not in ({'schematic': 2, 'symbol_library': 1, 'pcb': 1, 'module': 1},
                       {'schematic': 0, 'symbol_library': 0, 'pcb': 0, 'module': 0}):
        raise ValueError(f'Unexpected partial identifier mapping: {changes}')
    loaded = pcbnew.FootprintLoad(str(stage_library), NEW_NAME)
    if loaded is None or len(list(loaded.Pads())) != 16:
        raise ValueError('Native footprint library load failed')
    board = pcbnew.LoadBoard(str(stage / (PROJECT + '.kicad_pcb')))
    mcu = next(f for f in board.GetFootprints() if f.m_Uuid.AsString() == MCU_UUID)
    if mcu.GetFPIDAsString() != NEW_ID:
        raise ValueError('Native board footprint ID differs')
    if sorted(p.GetNumber() for p in loaded.Pads()) != sorted(p.GetNumber() for p in mcu.Pads()):
        raise ValueError('Library and board MCU pad identifiers differ')
    if any(sha(p) != value for p, value in {**input_hashes, **protected_hashes}.items()):
        raise RuntimeError('An input changed concurrently')
    if any(changes.values()):
        for staged_path, destination in files:
            staged_path.replace(destination)
        old_module.replace(safe(stage / 'original-escaped-name.kicad_mod'))
    report = {'old_identifier': OLD_ID, 'new_identifier': NEW_ID, 'files': rows,
              'checks': {'only_footprint_identifiers_changed': True, 'native_library_and_board_reload': True,
                         'all_16_mcu_pad_identifiers_preserved': True, 'project_and_rules_unchanged': True},
              'known_source_value_disagreements': ['R1: SCH 220kOhm vs PCB 10kOhm', 'R2: SCH 220kOhm vs PCB 10kOhm']}
    report_path = safe(ROOT / 'validation/mcu-footprint-id-report.json')
    if any(changes.values()) or not report_path.exists():
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'changed_identifiers': sum(changes.values()), 'checks': report['checks']}))


if __name__ == '__main__':
    main()
