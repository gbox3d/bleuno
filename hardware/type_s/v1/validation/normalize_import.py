"""Normalize this archived EasyEDA import without changing its circuit or routing.

Run with the installed KiCad Python through uv, with Python downloads disabled.
All paths are fixed beneath hardware/type_s/v1. No external migration scripts are loaded.
The original archive and initial-kicad-import.zip remain unchanged.
"""
from __future__ import annotations

import hashlib
import json
import re
import uuid
import zipfile
from dataclasses import dataclass
from pathlib import Path

import pcbnew


ROOT = Path(__file__).resolve().parent.parent
KICAD = ROOT / "kicad"
VALIDATION = ROOT / "validation"
OLD = "ZeroFactorySensorBoard_Board_DHT_2026-11-01"
NEW = "type_s_v1"
OLD_SYMBOL = "ZeroFactor-easyedapro"
OLD_FP = OLD + "-import-fps"
SHEET_UUID = "9159d041-722e-49f6-91a4-e7eb254dfd69"
BAD_ZONE_UUID = "61ffe24f-ac6d-4d97-8646-7b8917885445"
NAMESPACE = uuid.UUID("bdc40d70-d306-4a2c-a5e6-82c2ce8cd805")
EMPTY_WORKSHEET = """(page_layout
  (setup
    (textsize 1.5 1.5)
    (linewidth 0.15)
    (textlinewidth 0.15)
    (left_margin 10)
    (right_margin 10)
    (top_margin 10)
    (bottom_margin 10)
  )
)
"""

# Source Unique ID, final reference, schematic instance UUID, board UUID.
COMPONENTS = [
    ("gge2", "U1", "16d4ee59-573d-4071-be66-6d395d545188", "a4113187-8991-4bf9-8b8c-b8dcc5c31a94"),
    ("gge10_1", "R1", "763a134d-4751-4cb2-9c93-a1829e076003", "2de39a15-423e-46dd-88ae-52d09540f3a3"),
    ("gge10_2", "R2", "69c5d9b7-2043-4934-91da-45bb99ce6eb7", "798fe9e6-e91f-4674-93cc-2a2d59465165"),
    ("gge10", "R3", "c8e4fa50-cad0-45a1-b6b0-7550e3e23d16", "beec6548-f104-42f5-9ac0-7df2d4bb8fe1"),
    ("gge11", "R4", "35512c5a-39eb-4fa0-a848-4eddbe0ad80b", "864dea44-bdbe-4c8c-9da3-b2b0d8891e1f"),
    ("gge12", "R5", "c3f53d50-6fe6-4e65-aabb-89f1c7424ca5", "3f720b2c-714c-4e01-b3cb-a3a7b75772f6"),
    ("gge26", "J1", "c377d1d4-397f-4823-81d6-9ee191841484", "241927ba-92d2-403b-acd7-b8bcabc6a2cf"),
    ("gge27", "J2", "65bdba75-5926-4509-b8e7-823c47d15bdc", "7266b957-34d9-40d5-96af-1f3bb8f8c047"),
    ("gge17", "J3", "38089453-1421-4c2c-9de9-c0e682179ab6", "5deb5ff8-8a31-4d93-b859-b559b9f5383c"),
    ("gge18", "J4", "1f1c882e-4915-4fe8-ab4a-4e23b900f3eb", "a9e5d16a-da4d-4175-a309-65459a6c5b96"),
    ("gge18_1", "J5", "e6b5369e-cf3e-4a67-80cd-9414af8437d6", "0e60e259-4315-4873-8b6d-37713b33c00b"),
    ("gge23", "J6", "606f25ce-2b2e-4194-8c15-fbabf5bb8108", "72d57ef8-e498-4115-bb22-35b1b7171343"),
    ("gge24_1", "J7", "e5f20d52-c4a2-4208-8b68-b028b1dd5396", "a439c7d8-c870-400f-8333-b548ec6f4786"),
    ("gge24", "J8", "9d6138bc-81ee-48fd-8873-b39e15fb587e", "fc210689-59f7-4054-9583-27f709f8fdd8"),
    ("gge25", "J9", "ffc490d6-8d30-4b80-b31c-07edfb255e55", "0d5a053b-ca15-4b24-8749-7e0cba27c84a"),
]
FREE_PADS = {
    "d55b69c5-9904-4c33-a1f6-d03819176d1e": "MP1",
    "3c5a3727-dd89-4be5-b3b6-154800b1dedf": "MP2",
    "bbddde8d-fe2b-400d-9d8d-9320ca0f317d": "MP3",
}
LIB_FP = {
    "Header-Female-2.54_1x2": ("J", "HDR-TH_2P-P2.54-V-F"),
    "PZ254V-11-03P_C2937625": ("J", "HDR-TH_3P-P2.54-V-M"),
    "PM254V-11-03-H85": ("J", "HDR-TH_3P-P2.54-V-F"),
    "ESP32-C3_SUPERMINI_TH_ESP32-C3_SUPERMINI_TH": ("U", "ESP32-C3_SUPERMINI_TH{colon}MODULE_ESP32-C3_SUPERMINI"),
    "Res": ("R", "RES-TH_BD1.8-L3.2-P7.20-D0.4"),
    "Power-VCC": ("#PWR", ""),
    "Ground-GND": ("#PWR", ""),
    "Drawing-Symbol_A4": ("#FRAME", ""),
}


def safe(path: Path) -> Path:
    path = path.resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError(f"Path outside hardware/type_s/v1: {path}")
    return path


def digest(value) -> str:
    return hashlib.sha256(json.dumps(value, ensure_ascii=True, sort_keys=True).encode()).hexdigest()


def stable_id(label: str) -> str:
    return str(uuid.uuid5(NAMESPACE, label))


@dataclass
class Atom:
    value: str
    start: int
    end: int


@dataclass
class Node:
    items: list
    start: int
    end: int = 0

    @property
    def tag(self):
        return self.items[0].value

    def nodes(self, tag):
        return [x for x in self.items if isinstance(x, Node) and x.tag == tag]

    def node(self, tag):
        return next(iter(self.nodes(tag)), None)

    def atom(self, index=1):
        return self.items[index].value


class Document:
    def __init__(self, text):
        self.text = text
        self.replacements = {}
        self.insertions = {}
        stack = []
        for m in re.finditer(r'"(?:\\.|[^"\\])*"|[()]|[^\s()]+', text):
            token = m.group()
            if token == "(":
                node = Node([], m.start())
                if stack:
                    stack[-1].items.append(node)
                stack.append(node)
            elif token == ")":
                node = stack.pop()
                node.end = m.end()
                if not stack:
                    self.root = node
            else:
                value = json.loads(token) if token.startswith('"') else token
                stack[-1].items.append(Atom(value, m.start(), m.end()))
        assert not stack

    def raw(self, node):
        return self.text[node.start:node.end]

    def replace(self, node, text):
        self.replacements[(node.start, node.end)] = text

    def atom(self, atom, value, quoted=True):
        text = json.dumps(value, ensure_ascii=False) if quoted else value
        if self.raw(atom) != text:
            self.replace(atom, text)

    def append(self, node, text):
        self.insertions.setdefault(node.end - 1, []).append("\n\t\t" + text + "\n\t")

    def field(self, node, key, value, quoted=True):
        target = node.node(key)
        if target:
            self.atom(target.items[1], value, quoted)
        else:
            val = json.dumps(value, ensure_ascii=False) if quoted else value
            self.append(node, f"({key} {val})")

    def property(self, node, key):
        return next((p for p in node.nodes("property") if p.atom() == key), None)

    def set_property(self, node, key, value):
        target = self.property(node, key)
        assert target is not None, key
        self.atom(target.items[2], value)

    def add_hidden_property(self, node, key, value, pcb=False):
        if self.property(node, key):
            return
        ref = self.property(node, "Reference")
        at = self.raw(ref.node("at"))
        layer = '(layer "F.Fab")' if pcb else ""
        ident = f'(uuid "{stable_id(node.node("uuid").atom() + ":" + key)}")' if pcb else ""
        self.append(node, f'(property {json.dumps(key)} {json.dumps(value, ensure_ascii=False)} {at} {layer} (hide yes) {ident} (effects (font (size 1 1))))')

    def finish(self):
        edits = [(a, b, v) for (a, b), v in self.replacements.items()]
        edits += [(p, p, "".join(values)) for p, values in self.insertions.items()]
        edits.sort()
        for prior, following in zip(edits, edits[1:]):
            assert prior[1] <= following[0], "Overlapping text edits"
        result = self.text
        for start, end, replacement in reversed(edits):
            result = result[:start] + replacement + result[end:]
        Document(result)
        return result


def tree(node):
    return node.value if isinstance(node, Atom) else [tree(x) for x in node.items]


def uuid_of(node):
    return node.node("uuid").atom()


def normalize_library(doc, embedded=False):
    symbols = doc.root.node("lib_symbols").nodes("symbol") if embedded else doc.root.nodes("symbol")
    for symbol in symbols:
        name = symbol.atom().split(":")[-1]
        if name not in LIB_FP:
            # The exported library also contains unused symbols; retain them verbatim.
            assert not embedded, name
            continue
        prefix, footprint = LIB_FP[name]
        if embedded:
            doc.atom(symbol.items[1], f"{NEW}:{name}")
        doc.set_property(symbol, "Reference", prefix)
        doc.set_property(symbol, "Footprint", f"{NEW}:{footprint}" if footprint else "")
        if not footprint:
            for flag in ("in_bom", "on_board", "in_pos_files"):
                doc.field(symbol, flag, "no", False)


def synchronize_drawing_symbol(library_text, schematic_text):
    """The importer populated the cached title frame but left its library empty.

    Copy only this non-electrical symbol from the unchanged schematic cache so
    later library updates cannot erase the existing frame. The external symbol
    name is unqualified; all graphics and fields are kept exactly as cached.
    """
    library, schematic = Document(library_text), Document(schematic_text)
    name = "Drawing-Symbol_A4"
    cached = next(s for s in schematic.root.node("lib_symbols").nodes("symbol")
                  if s.atom() == f"{NEW}:{name}")
    existing = next(s for s in library.root.nodes("symbol") if s.atom() == name)
    replacement = Document(schematic.raw(cached))
    replacement.atom(replacement.root.items[1], name)
    replacement_text = replacement.finish()
    assert not any(n.tag == "pin" for unit in cached.nodes("symbol") for n in unit.items if isinstance(n, Node))
    if tree(existing) != tree(Document(replacement_text).root):
        library.replace(existing, replacement_text)
    return library.finish()


def board_fingerprint(doc, cutout_ids, preserved_ids):
    items = []
    for item in doc.root.items:
        if not isinstance(item, Node):
            items.append(tree(item))
        elif item.tag == "zone" and uuid_of(item) == BAD_ZONE_UUID:
            continue
        elif item.tag == "gr_line" and uuid_of(item) in cutout_ids:
            continue
        elif item.tag == "footprint":
            geometry = ["footprint", uuid_of(item)]
            for child in item.items[2:]:
                if isinstance(child, Node) and child.tag in ("property", "path", "sheetfile", "sheetname", "attr"):
                    continue
                if isinstance(child, Node) and child.tag == "fp_text" and uuid_of(child) in preserved_ids:
                    continue
                geometry.append(tree(child))
            items.append(geometry)
        else:
            items.append(tree(item))
    return digest(items)


def pad_net_fingerprint(doc):
    return digest(sorted((uuid_of(fp), uuid_of(pad), pad.atom(), tree(pad.node("net")) if pad.node("net") else None)
                         for fp in doc.root.nodes("footprint") for pad in fp.nodes("pad")))


def schematic_geometry(doc):
    items = []
    for item in doc.root.items:
        if isinstance(item, Node) and item.tag == "lib_symbols":
            # Ignore library metadata, retain symbol drawing and pin definitions.
            items.append([[s.atom().split(":")[-1], [tree(x) for x in s.items[2:] if isinstance(x, Node) and x.tag not in ("property", "in_bom", "on_board", "in_pos_files")]] for s in item.nodes("symbol")])
        elif isinstance(item, Node) and item.tag == "symbol":
            items.append([tree(x) for x in item.items if not (isinstance(x, Node) and x.tag in ("property", "instances", "lib_id", "in_bom", "on_board", "in_pos_files"))])
        else:
            items.append(tree(item))
    return digest(items)


def main():
    assert ROOT.name == "v1" and ROOT.parent.name == "type_s" and ROOT.parent.parent.name == "hardware"
    assert safe(VALIDATION / "initial-kicad-import.zip").is_file(), "Archive baseline first"
    assert not list(KICAD.glob("*.lck")), "Close KiCad editors before normalization"
    inventory = json.loads(safe(VALIDATION / "source-inventory.json").read_text(encoding="utf-8"))["source"]
    source = safe(ROOT / "source" / inventory["archive"])
    assert hashlib.sha256(source.read_bytes()).hexdigest() == inventory["sha256"]
    with zipfile.ZipFile(source) as archive:
        source_project = json.loads(archive.read("project.json"))
    staged = safe(VALIDATION / "normalization-stage")
    staged.mkdir(exist_ok=True)

    def input_path(extension):
        new, old = safe(KICAD / (NEW + extension)), safe(KICAD / (OLD + extension))
        assert not (new.exists() and old.exists()), f"Ambiguous inputs: {extension}"
        return new if new.exists() else old

    sch_path, pcb_path, pro_path = [input_path(ext) for ext in (".kicad_sch", ".kicad_pcb", ".kicad_pro")]
    sch = Document(sch_path.read_text(encoding="utf-8"))
    pcb = Document(pcb_path.read_text(encoding="utf-8"))
    symbols = {uuid_of(s): s for s in sch.root.nodes("symbol")}
    footprints = {uuid_of(f): f for f in pcb.root.nodes("footprint")}
    assert len(footprints) == 18 and len(symbols) == 28
    assert sch.root.node("uuid").atom() == SHEET_UUID
    source_components = {c["unique_id"]: c for c in inventory["components"]}
    source_symbols = {c["unique_id"]: c for sheet in inventory["schematics"] for c in sheet["parts"]}
    cutout = inventory["cutouts_mil"][0]
    points = [(round((120 + x * 0.0254) * 1e6), round((90 - y * 0.0254) * 1e6)) for x, y in cutout]
    if points[-1] == points[0]:
        points.pop()
    assert len(points) == 8
    cutout_ids = {stable_id(f"source-cutout-e197:{i}") for i in range(len(points))}
    preserved_ids = {stable_id(fp_uuid + ":original-reference-text") for _, _, _, fp_uuid in COMPONENTS}
    before_geometry = board_fingerprint(pcb, cutout_ids, preserved_ids)
    before_nets = pad_net_fingerprint(pcb)
    before_schematic = schematic_geometry(sch)
    reference_map = []
    values = {}
    source_value_differences = []
    displayed_value_differences = []

    normalize_library(sch, embedded=True)
    for unique_id, ref, sym_uuid, fp_uuid in COMPONENTS:
        symbol, footprint = symbols[sym_uuid], footprints[fp_uuid]
        source_component = source_components[unique_id]
        source_symbol = source_symbols[unique_id]
        # Source geometry and each pin's assigned net disambiguate repeated values.
        actual_xy = [float(v.value) for v in footprint.node("at").items[1:3]]
        expected_xy = [120 + source_component["xy_mil"][0] * .0254, 90 - source_component["xy_mil"][1] * .0254]
        assert max(abs(a - b) for a, b in zip(actual_xy, expected_xy)) < .00001
        schematic_xy = [float(v.value) for v in symbol.node("at").items[1:3]]
        expected_schematic_xy = [source_symbol["xy"][0] * .254, 209.55 - source_symbol["xy"][1] * .254]
        assert max(abs(a - b) for a, b in zip(schematic_xy, expected_schematic_xy)) < .00001
        expected_nets = {p["number"]: p["net"] or "" for p in source_component["pads"]}
        actual_nets = {p.atom(): p.node("net").atom() if p.node("net") else "" for p in footprint.nodes("pad")}
        assert actual_nets == expected_nets, ref
        original_sch = sch.property(symbol, "Original Reference") or sch.property(symbol, "Reference")
        original_pcb = pcb.property(footprint, "Original Reference") or pcb.property(footprint, "Reference")
        sch.add_hidden_property(symbol, "Original Reference", original_sch.atom(2))
        pcb.add_hidden_property(footprint, "Original Reference", original_pcb.atom(2), pcb=True)
        sch.set_property(symbol, "Reference", ref)
        lib_name = symbol.node("lib_id").atom().split(":")[-1]
        sch.field(symbol, "lib_id", f"{NEW}:{lib_name}")
        footprint_name = footprint.atom().split(":")[-1]
        fp_id = f"{NEW}:{footprint_name}"
        sch.set_property(symbol, "Footprint", fp_id)
        value = sch.property(symbol, "Value").atom(2)
        # Preserve the original schematic display and separately resolve the PCB
        # display from its own component/device data. R1/R2 intentionally differ.
        pcb_attributes = dict(source_project["devices"][source_component["device_id"]]["attributes"])
        pcb_attributes.update(source_component["local_properties"])
        pcb_source_value = pcb_attributes.get("Value")
        schematic_source_value = source_symbol.get("value")
        pcb_value = pcb_attributes.get("Name", "")
        if pcb_value == "={Value}":
            pcb_value = pcb_source_value or ""
        assert isinstance(pcb_value, str) and not pcb_value.startswith("={"), (ref, pcb_value)
        values[ref] = pcb_value
        pcb.set_property(footprint, "Value", pcb_value)
        for key, raw_value in (("Source Schematic Value", schematic_source_value), ("Source PCB Value", pcb_source_value)):
            if raw_value is not None:
                sch.add_hidden_property(symbol, key, str(raw_value))
                pcb.add_hidden_property(footprint, key, str(raw_value), pcb=True)
        if schematic_source_value != pcb_source_value:
            source_value_differences.append({"reference": ref, "schematic": schematic_source_value, "pcb": pcb_source_value})
        if value != pcb_value:
            displayed_value_differences.append({"reference": ref, "schematic": value, "pcb": pcb_value})
        pcb.atom(footprint.items[1], fp_id)
        pcb.field(footprint, "path", f"/{SHEET_UUID}/{sym_uuid}")
        pcb.field(footprint, "sheetname", "")
        pcb.field(footprint, "sheetfile", NEW + ".kicad_sch")
        old_ref = pcb.property(footprint, "Reference")
        preserved_id = stable_id(fp_uuid + ":original-reference-text")
        existing_user = [x for x in footprint.nodes("fp_text") if uuid_of(x) == preserved_id]
        if not existing_user:
            assert old_ref.atom(2) == original_pcb.atom(2)
            copied = [pcb.raw(n) for n in old_ref.items[3:] if isinstance(n, Node) and n.tag not in ("uuid", "hide", "show_name", "do_not_autoplace")]
            pcb.append(footprint, '(fp_text user ' + json.dumps(original_pcb.atom(2), ensure_ascii=False) + '\n' + '\n'.join(copied) + f'\n(uuid "{preserved_id}"))')
        pcb.set_property(footprint, "Reference", ref)
        pcb.field(old_ref, "hide", "yes", False)
        reference_map.append({"reference": ref, "source_unique_id": unique_id,
                              "source_reference": original_sch.atom(2), "source_pcb_text": original_pcb.atom(2),
                              "source_schematic_id": source_symbol["id"], "source_pcb_id": source_component["id"],
                              "source_schematic_value": schematic_source_value, "source_pcb_value": pcb_source_value,
                              "source_pcb_value_is_local": "Value" in source_component["local_properties"],
                              "schematic_displayed_value": value, "pcb_displayed_value": pcb_value,
                              "symbol_uuid": sym_uuid, "footprint_uuid": fp_uuid, "footprint": fp_id})

    assert {row["reference"] for row in source_value_differences} == {"R1", "R2"}
    assert {row["reference"] for row in displayed_value_differences} == {"R1", "R2"}

    power_index = 0
    for symbol in sch.root.nodes("symbol"):
        ident = uuid_of(symbol)
        lib_name = symbol.node("lib_id").atom().split(":")[-1]
        sch.field(symbol, "lib_id", f"{NEW}:{lib_name}")
        if lib_name in ("Power-VCC", "Ground-GND", "Drawing-Symbol_A4"):
            if lib_name == "Drawing-Symbol_A4":
                reference = "#FRAME1"
            else:
                power_index += 1
                reference = f"#PWR{power_index:03}"
            sch.set_property(symbol, "Reference", reference)
            sch.set_property(symbol, "Footprint", "")
            for flag in ("in_bom", "on_board", "in_pos_files"):
                sch.field(symbol, flag, "no", False)
        else:
            reference = next(ref for _, ref, sid, _ in COMPONENTS if sid == ident)
        for project in symbol.node("instances").nodes("project"):
            sch.atom(project.items[1], NEW)
            for path in project.nodes("path"):
                sch.field(path, "reference", reference)
    assert power_index == 12

    for ident, ref in FREE_PADS.items():
        footprint = footprints[ident]
        pcb.set_property(footprint, "Reference", ref)
        pcb.field(pcb.property(footprint, "Reference"), "hide", "yes", False)
        pcb.atom(footprint.items[1], f"{NEW}:{footprint.atom().split(':')[-1]}")
        attribute = footprint.node("attr")
        flags = set(x.value for x in attribute.items[1:]) if attribute else set()
        flags.update(("board_only", "exclude_from_pos_files", "exclude_from_bom"))
        attr_text = "(attr " + " ".join(sorted(flags)) + ")"
        if attribute:
            pcb.replace(attribute, attr_text)
        else:
            pcb.append(footprint, attr_text)

    for zone in pcb.root.nodes("zone"):
        if uuid_of(zone) == BAD_ZONE_UUID:
            assert zone.node("layer").atom() == "Edge.Cuts"
            pcb.replace(zone, "")
    existing_ids = {uuid_of(n) for n in pcb.root.nodes("gr_line")}
    for i, a in enumerate(points):
        b = points[(i + 1) % len(points)]
        edge_uuid = stable_id(f"source-cutout-e197:{i}")
        if edge_uuid not in existing_ids:
            mm = lambda v: format(v / 1e6, ".7f").rstrip("0").rstrip(".")
            pcb.append(pcb.root, f'(gr_line (start {mm(a[0])} {mm(a[1])}) (end {mm(b[0])} {mm(b[1])}) (stroke (width 0.05) (type default)) (layer "Edge.Cuts") (uuid "{edge_uuid}"))')

    sch_text, pcb_text = sch.finish(), pcb.finish()
    after_sch, after_pcb = Document(sch_text), Document(pcb_text)
    assert before_geometry == board_fingerprint(after_pcb, cutout_ids, preserved_ids)
    assert before_nets == pad_net_fingerprint(after_pcb)
    assert before_schematic == schematic_geometry(after_sch)
    pro = json.loads(pro_path.read_text(encoding="utf-8"))
    pro["meta"]["filename"] = NEW + ".kicad_pro"
    for sheet in pro["schematic"]["top_level_sheets"]:
        if sheet["filename"] == OLD + ".kicad_sch":
            sheet["filename"] = NEW + ".kicad_sch"
    symbol_path = safe(KICAD / (NEW + ".kicad_sym"))
    if not symbol_path.exists():
        symbol_path = safe(KICAD / (OLD_SYMBOL + ".kicad_sym"))
    lib = Document(symbol_path.read_text(encoding="utf-8"))
    normalize_library(lib)
    outputs = {
        NEW + ".kicad_sch": sch_text,
        NEW + ".kicad_pcb": pcb_text,
        NEW + ".kicad_pro": json.dumps(pro, indent=2, ensure_ascii=False) + "\n",
        NEW + ".kicad_sym": synchronize_drawing_symbol(lib.finish(), sch_text),
        "empty.kicad_wks": EMPTY_WORKSHEET,
        "fp-lib-table": f'(fp_lib_table (version 7) (lib (name "{NEW}") (type "KiCad") (uri "${{KIPRJMOD}}/{NEW}.pretty") (options "") (descr "Project-local imported footprints")))\n',
        "sym-lib-table": f'(sym_lib_table (version 7) (lib (name "{NEW}") (type "KiCad") (uri "${{KIPRJMOD}}/{NEW}.kicad_sym") (options "") (descr "Project-local imported symbols")))\n',
    }
    for name, text in outputs.items():
        safe(staged / name).write_text(text, encoding="utf-8", newline="\n")
    loaded = pcbnew.LoadBoard(str(staged / (NEW + ".kicad_pcb")))
    loaded_fp = {f.m_Uuid.AsString(): f for f in loaded.GetFootprints()}
    assert len(loaded_fp) == 18 and sum(len(list(f.Pads())) for f in loaded_fp.values()) == 55
    assert len(list(loaded.GetTracks())) == 114
    for _, ref, sym_uuid, fp_uuid in COMPONENTS:
        f = loaded_fp[fp_uuid]
        assert f.GetReference() == ref
        assert f.GetPath().AsString() == f"/{SHEET_UUID}/{sym_uuid}"
        assert f.GetValue() == values[ref] and f.GetSheetfile() == NEW + ".kicad_sch"
    assert not any(z.GetLayer() == pcbnew.Edge_Cuts for z in loaded.Zones())
    # Commit staged files only after structural and native-reader checks pass.
    for src in (sch_path, pcb_path, pro_path, symbol_path):
        destination = safe(KICAD / (NEW + src.suffix))
        if src != destination:
            src.replace(destination)
    old_pretty, new_pretty = safe(KICAD / (OLD_FP + ".pretty")), safe(KICAD / (NEW + ".pretty"))
    if old_pretty.exists():
        assert not new_pretty.exists()
        old_pretty.replace(new_pretty)
    assert new_pretty.is_dir()
    old_prl, new_prl = safe(KICAD / (OLD + ".kicad_prl")), safe(KICAD / (NEW + ".kicad_prl"))
    if old_prl.exists():
        assert not new_prl.exists()
        old_prl.replace(new_prl)
    for name in outputs:
        safe(staged / name).replace(safe(KICAD / name))
    report = {
        "project": NEW, "source_sha256": inventory["sha256"], "references": reference_map,
        "virtual_power_references": power_index, "board_only_references": list(FREE_PADS.values()),
        "board_geometry_sha256": before_geometry, "pad_net_sha256": before_nets,
        "schematic_geometry_sha256": before_schematic, "preserved_original_silkscreen_texts": 15,
        "repaired_cutout": {"source_id": "e197", "edges": 8, "source_vertices_mil": cutout},
        "source_value_differences_preserved": source_value_differences,
        "displayed_value_differences_preserved": displayed_value_differences,
        "checks": {"board_geometry_unchanged_except_cutout": True, "pad_nets_unchanged": True,
                   "schematic_geometry_unchanged": True, "native_board_reload": True},
    }
    safe(VALIDATION / "normalization-report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"project": str(KICAD / (NEW + ".kicad_pro")), "checks": report["checks"]}))


if __name__ == "__main__":
    main()
