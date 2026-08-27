"""Minimal read-only XLSX loader for the supplied GB5753 object dictionary."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re
from xml.etree import ElementTree
from zipfile import ZipFile


NS = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"


@dataclass(frozen=True)
class ObjectEntry:
    index: int
    sub_index: int
    name: str
    access: str
    pdo_mapping: bool
    storable: bool
    data_type: str
    default_display: str
    default_value: str
    description: str


def _number(text: str) -> int:
    return int(text, 16) if text.lower().startswith("0x") else int(text)


def load_xlsx(path: str | Path) -> list[ObjectEntry]:
    with ZipFile(path) as archive:
        shared_root = ElementTree.fromstring(archive.read("xl/sharedStrings.xml"))
        strings = [
            "".join(node.text or "" for node in item.iter(f"{{{NS}}}t"))
            for item in shared_root.findall(f"{{{NS}}}si")
        ]
        sheet = ElementTree.fromstring(archive.read("xl/worksheets/sheet1.xml"))

    entries: list[ObjectEntry] = []
    for row in sheet.findall(f".//{{{NS}}}row"):
        values: dict[str, str] = {}
        for cell in row.findall(f"{{{NS}}}c"):
            match = re.match(r"[A-Z]+", cell.attrib.get("r", ""))
            value_node = cell.find(f"{{{NS}}}v")
            if match is None or value_node is None:
                continue
            raw = value_node.text or ""
            values[match.group()] = strings[int(raw)] if cell.attrib.get("t") == "s" else raw
        if not values.get("A", "").lower().startswith("0x"):
            continue
        try:
            entries.append(
                ObjectEntry(
                    index=_number(values["A"]),
                    sub_index=_number(values.get("B", "0")),
                    name=values.get("C", ""),
                    access=values.get("D", ""),
                    pdo_mapping=values.get("E") == "Y",
                    storable=values.get("F") == "Y",
                    data_type=values.get("G", ""),
                    default_display=values.get("H", ""),
                    default_value=values.get("I", ""),
                    description=values.get("J", ""),
                )
            )
        except (KeyError, ValueError):
            continue
    return entries
