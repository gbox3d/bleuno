"""Apply the v1.1 power-section schematic fix. Never touches v1, the PCB, or libraries.

Exactly four changes:
1. J1 Value "MT1" -> "BAT" (the battery connector was mislabelled as motor output MT1).
2. Remove power symbol #PWR012 (VCC) that sat on the J1 pin 1 wire end (91.44, 40.64).
3. Add a global label "VIN" at that wire end, so battery + enters the switch (J2.3).
4. Add a no-connect flag on J2 pin 1 (54.61, 34.29): the switch OFF position is open by design.

Run with the already installed Python via uv --no-python-downloads (text patch, no KiCad import).
Refuses to run twice.
"""
from pathlib import Path
import hashlib
import json
import uuid

BASE = Path(__file__).resolve().parents[2]  # .../hardware/type_m/v1.1
SCH = BASE / "kicad" / "type_m_v1_1.kicad_sch"
# Sensor-stage schematic this patch was written against (validation/revision-comparison.json).
BASELINE_SHA256 = "574c1ca528cba32e3ad642419d7f4b3a9aad8a8a8fa6d7d08b7733d3f197abea"
PWR012 = "cf822914-aece-439c-8a36-4716e4397660"
J1 = "ad2c76f2-73a2-475e-b908-4b6411e7d519"
VIN_AT_J2 = "34783d0f-520b-45c8-b483-83569857bdb8"
LABEL_ID = str(uuid.uuid5(uuid.NAMESPACE_URL, "bleuno/type_m/v1.1/VIN/J1.1"))
NC_ID = str(uuid.uuid5(uuid.NAMESPACE_URL, "bleuno/type_m/v1.1/NC/J2.1"))


def top_level_blocks(text):
    """Yield (start, end) of each depth-1 S-expression inside the root form."""
    pos = text.index("(") + 1
    depth = 1
    quoted = escaped = False
    start = None
    while pos < len(text):
        char = text[pos]
        if quoted:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                quoted = False
        elif char == '"':
            quoted = True
        elif char == "(":
            depth += 1
            if depth == 2:
                start = pos
        elif char == ")":
            depth -= 1
            if depth == 1:
                yield start, pos + 1
                start = None
            elif depth == 0:
                return
        pos += 1


def block_with(text, marker):
    found = [(s, e) for s, e in top_level_blocks(text) if marker in text[s:e]]
    assert len(found) == 1, (marker, len(found))
    return found[0]


def main():
    if BASE.name != "v1.1" or BASE.parent.name != "type_m":
        raise RuntimeError("This patch is scoped to hardware/type_m/v1.1")
    locks = sorted(p.name for p in SCH.parent.glob("~*.lck"))
    if locks:
        raise RuntimeError("KiCad has the project open (%s); close it first" % ", ".join(locks))
    before = SCH.read_bytes()
    assert b"\r\n" not in before
    text = before.decode("utf-8")
    if LABEL_ID in text or NC_ID in text:
        raise RuntimeError("Power fix already applied; refusing to rerun")
    if hashlib.sha256(before).hexdigest() != BASELINE_SHA256:
        raise RuntimeError("Schematic is not the sensor-stage baseline this patch targets; refusing")

    # 1. J1 value
    start, end = block_with(text, J1)
    block = text[start:end]
    assert block.count('(property "Value" "MT1"') == 1
    assert '(lib_id "type_m_v1_1:BX-ZH1.5-2PWZ")' in block and "(at 104.14 41.91 0)" in block
    text = text[:start] + block.replace('(property "Value" "MT1"', '(property "Value" "BAT"') + text[end:]

    # 2. remove #PWR012
    start, end = block_with(text, PWR012)
    block = text[start:end]
    assert block.startswith("(symbol") and '(lib_id "type_m_v1_1:Power-VCC")' in block
    assert "(at 91.44 40.64 0)" in block and '(reference "#PWR012")' in block
    assert text[start - 1] == "\t" and text[end] == "\n"
    text = text[:start - 1] + text[end + 1:]
    assert PWR012 not in text and "#PWR012" not in text

    # 3. VIN label at J1 pin 1 wire end, cloned from the existing VIN label style
    start, end = block_with(text, VIN_AT_J2)
    template = text[start:end]
    assert template.startswith('(global_label "VIN"') and "(at 52.07 39.37 180)" in template
    label = (template.replace("(at 52.07 39.37 180)", "(at 91.44 40.64 180)")
             .replace("(at 52.07 39.37 0)", "(at 91.44 40.64 0)")
             .replace(VIN_AT_J2, LABEL_ID))
    assert label.count("91.44 40.64") == 2 and LABEL_ID in label

    text = text[:start] + label + "\n\t" + text[start:]

    # 4. no-connect on J2 pin 1 (symbol at 59.69,36.83 rot 0; library pin 1 at -5.08,+2.54 -> y flips).
    #    Placed after the last junction, where eeschema itself writes no_connect items.
    no_connect = '(no_connect\n\t\t(at 54.61 34.29)\n\t\t(uuid "' + NC_ID + '")\n\t)'
    assert text.count("(no_connect") == 0
    junctions = [(s, e) for s, e in top_level_blocks(text) if text[s:e].startswith("(junction")]
    assert junctions, "expected junction blocks before the wires"
    last = junctions[-1][1]
    text = text[:last] + "\n\t" + no_connect + text[last:]

    assert text.count('(global_label "VIN"') == 2
    assert text.count("(no_connect") == 1
    assert "(xy 99.06 40.64) (xy 91.44 40.64)" in text  # J1.1 wire still ends at the new label
    after = text.encode("utf-8")
    SCH.write_bytes(after)
    report = {"file": SCH.relative_to(BASE).as_posix(),
              "before_sha256": hashlib.sha256(before).hexdigest(),
              "after_sha256": hashlib.sha256(after).hexdigest(),
              "changes": ["J1 Value MT1 -> BAT",
                          "removed #PWR012 Power-VCC at (91.44, 40.64)",
                          "added global_label VIN at (91.44, 40.64, 180) uuid " + LABEL_ID,
                          "added no_connect at (54.61, 34.29) uuid " + NC_ID]}
    (Path(__file__).resolve().parent / "schematic-patch.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
