"""Extract type_s rules from its captured EasyEDA configuration.

Default: write a proposal under validation only. --apply updates the final
KiCad project and custom rules, never PCB/schematic geometry or DRC severities.
No type_m data is read. Unknown selectors or unequal grouped values are rejected.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def one_default(options):
    names = [key for key, value in options.items() if value.get('isSetDefault')]
    if len(names) != 1:
        raise ValueError(f'Expected one default, got {names}')
    return names[0]


def equal(values):
    if not values or len(set(values)) != 1:
        raise ValueError(f'Cannot collapse unequal source rule values: {values}')
    return values[0]


def pair(table, a, b):
    if table['unit'] != 'mm' or set(table['tables']) != {'1'}:
        raise ValueError('Only this source mm table is supported')
    matrix = table['tables']['1']['content']
    for row, column in ((a, b), (b, a)):
        if row in table['row'] and column in table['column']:
            i, j = table['row'].index(row), table['column'].index(column)
            if j < len(matrix[i]):
                return matrix[i][j]
    raise ValueError(f'No source spacing for {a}/{b}')


def extract(baseline):
    config = baseline['configuration']['config']
    spacing = config['Spacing']['Safe Spacing']
    tracks = config['Physics']['Track']
    spacing_default, track_default = one_default(spacing), one_default(tracks)
    nets = baseline['netRules']
    if any(n['type'] != 'net' or n['Safe Spacing'] not in ('default', spacing_default)
           or n['Track'] not in ('default', track_default) for n in nets):
        raise ValueError('This board has unexpected net-specific spacing/track selectors')
    copper_names = sorted({n['Copper Safe Spacing'] for n in nets})
    plane_names = sorted({n['Plane Safe Spacing'] for n in nets})
    base = spacing[spacing_default]
    zone_tables = [spacing[name] for name in set(copper_names + plane_names)]
    types = ('Track', 'TH Pad')
    width = tracks[track_default]
    if width['unit'] != 'mm' or set(width['form']['data']) != {'1'}:
        raise ValueError('Unexpected source track-width table')
    other = config['Spacing']['Other Spacing']
    other_default = one_default(other)
    values = {
        'track_track': pair(base, 'Track', 'Track'),
        'ordinary_copper': equal([pair(base, 'Track', 'TH Pad'), pair(base, 'TH Pad', 'TH Pad')]),
        'zone_copper': equal([pair(t, 'Copper/Plane Zone', o) for t in zone_tables for o in (*types, 'Copper/Plane Zone')]),
        'ordinary_edge': equal([pair(base, 'Board Outline', o) for o in types]),
        'zone_edge': equal([pair(t, 'Copper/Plane Zone', 'Board Outline') for t in zone_tables]),
        'ordinary_hole': equal([pair(base, 'Hole', o) for o in types]),
        'zone_hole': equal([pair(t, 'Copper/Plane Zone', 'Hole') for t in zone_tables]),
        'hole_hole': other[other_default]['form']['hole2Hole'],
        'slot_ordinary': equal([pair(base, 'Slot Region', o) for o in types]),
        'width_min': width['form']['data']['1']['minValue'],
        'width_default': width['form']['data']['1']['defaultValue'],
        'width_max': width['form']['data']['1']['maxValue'],
    }
    return values, {'safe_spacing_default': spacing_default, 'track_default': track_default,
                    'copper_selectors': copper_names, 'plane_selectors': plane_names,
                    'net_count': len(nets), 'net_names': sorted(n['name'] for n in nets)}


def custom_rules(v):
    def rule(name, constraint, value, condition=None):
        lines = [f'(rule "EasyEDA type_s {name}"']
        if condition:
            lines.append(f'  (condition "{condition}")')
        lines.append(f'  (constraint {constraint} (min {json.dumps(value)}mm)))')
        return '\n'.join(lines)
    parts = ['# Generated from type_s validation/easyeda-baseline.json; see rules-mapping.md.',
             '# Global object types only. No net, reference, coordinate or DRC-ID exceptions.',
             '(version 1)',
             rule('ordinary copper', 'clearance', v['ordinary_copper']),
             rule('track to track', 'clearance', v['track_track'], "A.Type == 'Track' && B.Type == 'Track'"),
             rule('zone to copper', 'clearance', v['zone_copper'], "A.Type == 'Zone' || B.Type == 'Zone'"),
             rule('ordinary board edge', 'edge_clearance', v['ordinary_edge']),
             rule('zone to board edge', 'edge_clearance', v['zone_edge'], "A.Type == 'Zone'"),
             rule('ordinary hole', 'hole_clearance', v['ordinary_hole']),
             rule('zone to hole', 'hole_clearance', v['zone_hole'], "A.Type == 'Zone' || B.Type == 'Zone'"),
             rule('hole to hole', 'hole_to_hole', v['hole_hole']),
             '(rule "EasyEDA type_s track width"\n  (condition "A.Type == \'Track\'")\n'
             f"  (constraint track_width (min {json.dumps(v['width_min'])}mm) "
             f"(opt {json.dumps(v['width_default'])}mm) (max {json.dumps(v['width_max'])}mm)))"]
    return '\n\n'.join(parts) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    baseline_path = ROOT / 'validation' / 'easyeda-baseline.json'
    baseline = json.loads(baseline_path.read_text(encoding='utf-8'))
    values, selectors = extract(baseline)
    settings = {
        'min_clearance': min(values[k] for k in ('track_track', 'ordinary_copper', 'zone_copper')),
        'min_copper_edge_clearance': min(values['ordinary_edge'], values['zone_edge']),
        'min_hole_clearance': min(values['ordinary_hole'], values['zone_hole']),
        'min_hole_to_hole': values['hole_hole'],
        'min_track_width': values['width_min'],
    }
    text = custom_rules(values)
    report = {'applied': args.apply, 'baseline_sha256': sha(baseline_path),
              'configuration_name': baseline['configuration']['name'], 'selectors': selectors,
              'source_values_mm': values, 'proposed_board_rules': settings,
              'proposed_default_netclass': {'clearance': values['ordinary_copper'], 'track_width': values['width_default']},
              'scope': 'Source values for current Track/TH Pad/Zone objects; slot vs edge semantics differ. See rules-mapping.md.'}
    if args.apply:
        pro = ROOT / 'kicad' / 'type_s_v1.kicad_pro'
        dru = pro.with_suffix('.kicad_dru')
        guarded = [pro, pro.with_suffix('.kicad_pcb'), pro.with_suffix('.kicad_sch'), baseline_path]
        original_hashes = {p: sha(p) for p in guarded}
        project = json.loads(pro.read_text(encoding='utf-8'))
        original = deepcopy(project)
        changed = []
        rules = project['board']['design_settings']['rules']
        for key, value in settings.items():
            changed.append({'path': 'board.design_settings.rules.' + key, 'before': rules[key], 'after': value})
            rules[key] = value
        default = next(c for c in project['net_settings']['classes'] if c['name'] == 'Default')
        for key, value in report['proposed_default_netclass'].items():
            changed.append({'path': 'net_settings.classes.Default.' + key, 'before': default[key], 'after': value})
            default[key] = value
        assert project['board']['design_settings']['rule_severities'] == original['board']['design_settings']['rule_severities']
        assert project['board']['design_settings']['drc_exclusions'] == original['board']['design_settings']['drc_exclusions']
        if any(sha(p) != digest for p, digest in original_hashes.items()):
            raise RuntimeError('An input changed during rule preparation')
        pro.write_text(json.dumps(project, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        dru.write_text(text, encoding='utf-8')
        if any(sha(p) != digest for p, digest in original_hashes.items() if p != pro):
            raise RuntimeError('A geometry or source file changed concurrently; verify saved inputs')
        report.update({'changes': changed, 'input_sha256': {p.name: h for p, h in original_hashes.items()},
                       'project_sha256_after': sha(pro), 'custom_rules_sha256': sha(dru),
                       'pcb_and_schematic_unchanged': True, 'severities_and_exclusions_unchanged': True})
        output = ROOT / 'validation' / 'rules-mapping.json'
    else:
        (ROOT / 'validation' / 'proposed-type_s_v1.kicad_dru').write_text(text, encoding='utf-8')
        output = ROOT / 'validation' / 'rules-proposal.json'
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'report': str(output), 'applied': args.apply, 'source_values_mm': values}))


if __name__ == '__main__':
    main()
