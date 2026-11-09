"""Apply the v1.1 SEN1 connection, pin-name corrections, and frame revision.

Run with an installed Python via uv --no-python-downloads. This script never
writes the source v1, PCB, project rules, or footprint libraries.
"""
from pathlib import Path
import hashlib
import json
import uuid

BASE = Path(__file__).resolve().parent.parent


def block_end(text, start):
    depth = 0
    quoted = escaped = False
    for pos in range(start, len(text)):
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
        elif char == ")":
            depth -= 1
            if depth == 0:
                return pos + 1
    raise ValueError("Unbalanced S-expression")


def correct_symbol(text, name):
    marker = '(symbol "' + name + '"'
    assert text.count(marker) == 1, marker
    start = text.index(marker)
    end = block_end(text, start)
    symbol = text[start:end]
    for number, previous, current in [("15", "GPIO21", "GPIO20"),
                                       ("16", "GPIO22", "GPIO21")]:
        position = 0
        found = []
        while True:
            position = symbol.find("(pin ", position)
            if position < 0:
                break
            pin_end = block_end(symbol, position)
            pin = symbol[position:pin_end]
            if '(number "' + number + '"' in pin:
                found.append((position, pin_end, pin))
            position = pin_end
        assert len(found) == 1, (name, number, len(found))
        first, last, pin = found[0]
        old, new = '(name "' + previous + '"', '(name "' + current + '"'
        assert (old in pin) != (new in pin), (name, number)
        symbol = symbol[:first] + pin.replace(old, new) + symbol[last:]
    return text[:start] + symbol + text[end:]


def replace_once_or_current(text, previous, current):
    if previous in text:
        assert text.count(previous) == 1
        return text.replace(previous, current)
    assert text.count(current) == 1
    return text


def correct_frame(text, prefix):
    marker = '(symbol "' + prefix + 'Drawing-Symbol_A4"'
    assert text.count(marker) == 1
    start = text.index(marker)
    end = block_end(text, start)
    frame = text[start:end]
    for old, new in [('(text "V1.0"', '(text "v1.1"'),
                     ('(text "2026-03-16"', '(text "2026-11-01"')]:
        frame = replace_once_or_current(frame, old, new)
    return text[:start] + frame + text[end:]


def main():
    records = []
    for suffix, symbol_name in [
        ("kicad_sch", "type_m_v1_1:ESP32C3 Supermini"),
        ("kicad_sym", "ESP32C3 Supermini"),
    ]:
        path = BASE / "kicad" / ("type_m_v1_1." + suffix)
        before = path.read_bytes()
        text = before.decode("utf-8").replace("\r\n", "\n")
        text = correct_symbol(text, symbol_name)
        text = correct_frame(text, "type_m_v1_1:" if suffix == "kicad_sch" else "")
        if suffix == "kicad_sch":
            for old, new in [
                ('(property "Version" "V1.0"', '(property "Version" "v1.1"'),
                ('(property "@Update Date" "2026-03-16"', '(property "@Update Date" "2026-11-01"'),
            ]:
                text = replace_once_or_current(text, old, new)
            assert '(no_connect\n\t\t(at 31.75 69.85)' not in text
            label_id = str(uuid.uuid5(uuid.NAMESPACE_URL,
                                     "bleuno/type_m/v1.1/SEN1/U1.15"))
            if label_id not in text:
                start = text.index('\t(global_label "SERVO1"\n\t\t(shape input)\n\t\t(at 31.75 82.55 180)')
                end = block_end(text, start + 1)
                template = text[start:end]
                label = template.replace('"SERVO1"', '"SEN1"')
                label = label.replace("31.75 82.55", "31.75 69.85")
                label = label.replace("a5fdeb6f-f32a-4c13-a972-c329e2bc10b4", label_id)
                assert label_id in label
                text = text[:start] + label + "\n" + text[start:]
            assert text.count('(global_label "SEN1"') == 2
        newline = "\r\n" if b"\r\n" in before else "\n"
        after = text.replace("\n", newline).encode("utf-8")
        if before != after:
            path.write_bytes(after)
        records.append({"file": str(path.relative_to(BASE)),
                        "before_sha256": hashlib.sha256(before).hexdigest(),
                        "after_sha256": hashlib.sha256(after).hexdigest(),
                        "changed": before != after})
    print(json.dumps(records, indent=2))


if __name__ == "__main__":
    main()
