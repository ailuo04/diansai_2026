#!/usr/bin/env python3
"""Merge an application Intel HEX file with the generated OLED font HEX file."""

from __future__ import annotations

import argparse
from pathlib import Path


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--application", type=Path, required=True)
    parser.add_argument("--font", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def parse_record(line: str) -> tuple[int, int, bytes]:
    raw = bytes.fromhex(line.strip()[1:])
    if len(raw) < 5 or sum(raw) & 0xFF:
        raise ValueError(f"Invalid Intel HEX record: {line.strip()}")
    length = raw[0]
    if len(raw) != length + 5:
        raise ValueError(f"Invalid Intel HEX length: {line.strip()}")
    address = (raw[1] << 8) | raw[2]
    return address, raw[3], raw[4 : 4 + length]


def intel_hex_record(address: int, record_type: int, data: bytes) -> str:
    payload = bytes((len(data), (address >> 8) & 0xFF, address & 0xFF, record_type)) + data
    checksum = (-sum(payload)) & 0xFF
    return ":" + (payload + bytes((checksum,))).hex().upper()


def load_hex(path: Path) -> tuple[dict[int, int], list[tuple[int, bytes]]]:
    memory: dict[int, int] = {}
    metadata: list[tuple[int, bytes]] = []
    upper_address = 0

    for line in path.read_text(encoding="ascii").splitlines():
        address, record_type, data = parse_record(line)
        if record_type == 0x00:
            absolute_address = upper_address + address
            for offset, value in enumerate(data):
                target = absolute_address + offset
                if target in memory and memory[target] != value:
                    raise ValueError(f"Conflicting byte at 0x{target:08X}")
                memory[target] = value
        elif record_type == 0x04:
            upper_address = int.from_bytes(data, "big") << 16
        elif record_type not in (0x01, 0x02, 0x03):
            metadata.append((record_type, data))
        elif record_type in (0x03,):
            metadata.append((record_type, data))
    return memory, metadata


def write_hex(
    memory: dict[int, int], metadata: list[tuple[int, bytes]], output_path: Path
) -> None:
    lines: list[str] = []
    addresses = sorted(memory)
    index = 0
    current_upper = -1

    while index < len(addresses):
        start = addresses[index]
        upper = start >> 16
        if upper != current_upper:
            lines.append(intel_hex_record(0, 0x04, upper.to_bytes(2, "big")))
            current_upper = upper

        data = bytearray((memory[start],))
        index += 1
        while (
            index < len(addresses)
            and len(data) < 16
            and addresses[index] == start + len(data)
            and addresses[index] >> 16 == current_upper
        ):
            data.append(memory[addresses[index]])
            index += 1
        lines.append(intel_hex_record(start & 0xFFFF, 0x00, data))

    for record_type, data in metadata:
        lines.append(intel_hex_record(0, record_type, data))
    lines.append(intel_hex_record(0, 0x01, b""))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n", encoding="ascii", newline="\n")


def main() -> None:
    arguments = parse_arguments()
    application_memory, metadata = load_hex(arguments.application)
    font_memory, _ = load_hex(arguments.font)
    overlap = set(application_memory).intersection(font_memory)
    if overlap:
        raise RuntimeError(f"Application overlaps font at 0x{min(overlap):08X}")
    application_memory.update(font_memory)
    write_hex(application_memory, metadata, arguments.output)
    print(
        f"Merged {len(application_memory)} bytes into {arguments.output}; "
        f"font bytes: {len(font_memory)}"
    )


if __name__ == "__main__":
    main()
