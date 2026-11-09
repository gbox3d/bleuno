"""Apply the v1.1 SENSOR1 connection to this revision only; never modify v1.

Run with the existing KiCad Python via uv --no-python-downloads.
Refuses a second application, so routed user work cannot be silently duplicated.
"""
import hashlib
import json
from pathlib import Path

import pcbnew

ROOT = Path(__file__).resolve().parent.parent
PCB = ROOT / 'kicad' / 'type_m_v1_1.kicad_pcb'


def point(x, y):
    return pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y))


def main():
    if ROOT.name != 'v1.1' or ROOT.parent.name != 'type_m':
        raise RuntimeError('This patch is scoped to type_m/v1.1')
    before = hashlib.sha256(PCB.read_bytes()).hexdigest()
    board = pcbnew.LoadBoard(str(PCB))
    module = next(f for f in board.GetFootprints() if f.GetReference() == 'U1')
    pads = {p.GetNumber(): p for p in module.Pads()}
    if pads['15'].GetNetname() != 'unconnected-(U1-GPIO21-Pad15)':
        raise RuntimeError('Expected untouched copied v1 GPIO20 pad; refusing to rerun')
    if pads['16'].GetNetname() != 'unconnected-(U1-GPIO22-Pad16)':
        raise RuntimeError('Unexpected pad 16 net')
    sensor_net = board.FindNet('SEN1')
    pads['15'].SetNet(sensor_net)
    pads['15'].SetPinFunction('GPIO20')
    next_uart_net = pcbnew.NETINFO_ITEM(board, 'unconnected-(U1-GPIO21-Pad16)')
    board.Add(next_uart_net)
    pads['16'].SetNet(next_uart_net)
    pads['16'].SetPinFunction('GPIO21')

    routes = [
        (pcbnew.F_Cu, [(145.461, 109.304), (147.955, 109.304),
                       (148.25, 109.599), (148.25, 112.75)]),
        (pcbnew.B_Cu, [(148.25, 112.75), (157.7, 112.75),
                       (159.461, 114.511), (168.575, 114.511)]),
    ]
    new_tracks = []
    for layer, vertices in routes:
        for start, end in zip(vertices, vertices[1:]):
            track = pcbnew.PCB_TRACK(board)
            track.SetStart(point(*start))
            track.SetEnd(point(*end))
            track.SetWidth(pcbnew.FromMM(0.254))
            track.SetLayer(layer)
            track.SetNet(sensor_net)
            board.Add(track)
            new_tracks.append({'uuid': track.m_Uuid.AsString(),
                               'layer': 'F.Cu' if layer == pcbnew.F_Cu else 'B.Cu',
                               'start_mm': start, 'end_mm': end, 'width_mm': 0.254})
    via = pcbnew.PCB_VIA(board)
    via.SetPosition(point(148.25, 112.75))
    via.SetViaType(pcbnew.VIATYPE_THROUGH)
    via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    via.SetWidth(pcbnew.FromMM(0.6))
    via.SetDrill(pcbnew.FromMM(0.3))
    via.SetNet(sensor_net)
    board.Add(via)

    revision = pcbnew.PCB_TEXT(board)
    revision.SetText('type_m v1.1')
    revision.SetPosition(point(134.5, 120.2))
    revision.SetTextSize(point(1.0, 1.0))
    revision.SetTextThickness(pcbnew.FromMM(0.15))
    revision.SetLayer(pcbnew.F_SilkS)
    board.Add(revision)

    board.BuildConnectivity()
    pcbnew.SaveBoard(str(PCB), board)
    report = {'input_sha256': before,
              'output_sha256_before_zone_refill': hashlib.sha256(PCB.read_bytes()).hexdigest(),
              'connection': ['J7.2', 'R3.2', 'U1.15 GPIO20'],
              'new_tracks': new_tracks,
              'new_via': {'uuid': via.m_Uuid.AsString(), 'xy_mm': [148.25, 112.75],
                          'diameter_mm': 0.6, 'drill_mm': 0.3},
              'revision_text': {'uuid': revision.m_Uuid.AsString(), 'text': 'type_m v1.1'},
              'note': 'Run native zone refill, DRC with schematic parity, and independent comparison next.'}
    (ROOT / 'validation' / 'pcb-patch.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'added_tracks': len(new_tracks), 'added_vias': 1, 'pcb': str(PCB)}))


if __name__ == '__main__':
    main()
