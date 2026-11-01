#!/usr/bin/env python3
"""Validate the ReceiverPart1/USART6 board-link protocol without hardware."""

from __future__ import annotations

import struct
import sys
from pathlib import Path

FRAME_SYNC = b"\xA5\x5A"
CONTROL_SYNC = b"\xC5\x5C"
PROTOCOL_VERSION = 0x01
FRAME_TYPE_BOTH = 0x30
FRAME_FLAGS_BOTH = 0x03
SAMPLE_COUNT = 500
HEADER_SIZE = 12
CRC_SIZE = 2
PAYLOAD_SIZE = SAMPLE_COUNT * 4
FRAME_SIZE = HEADER_SIZE + PAYLOAD_SIZE + CRC_SIZE


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(sequence: int = 0, interval_us: int = 200) -> bytes:
    frame = bytearray(FRAME_SIZE)
    frame[0:2] = FRAME_SYNC
    frame[2] = PROTOCOL_VERSION
    frame[3] = FRAME_TYPE_BOTH
    struct.pack_into("<H", frame, 4, sequence & 0xFFFF)
    struct.pack_into("<H", frame, 6, SAMPLE_COUNT)
    struct.pack_into("<H", frame, 8, interval_us)
    frame[10] = FRAME_FLAGS_BOTH
    frame[11] = 0

    for index in range(SAMPLE_COUNT):
        phase1 = ((index * 4 * 256) // SAMPLE_COUNT + sequence * 3) & 0xFF
        phase2 = ((index * 9 * 256) // SAMPLE_COUNT + sequence * 5) & 0xFF
        triangle1 = phase1 if phase1 < 128 else 255 - phase1
        triangle2 = phase2 if phase2 < 128 else 255 - phase2
        ch1 = 200 + (triangle1 * (2900 - 200)) // 127
        ch2 = 300 + (triangle2 * (2700 - 300)) // 127
        struct.pack_into("<HH", frame, HEADER_SIZE + index * 4, ch1, ch2)

    crc = crc16_ccitt_false(bytes(frame[:-CRC_SIZE]))
    struct.pack_into("<H", frame, FRAME_SIZE - CRC_SIZE, crc)
    return bytes(frame)


def parse_frame(frame: bytes) -> dict[str, int | bool]:
    if len(frame) != FRAME_SIZE:
        raise ValueError(f"frame size {len(frame)} != {FRAME_SIZE}")

    count = struct.unpack_from("<H", frame, 6)[0]
    expected_size = HEADER_SIZE + count * 4 + CRC_SIZE
    received_crc = struct.unpack_from("<H", frame, len(frame) - 2)[0]
    calculated_crc = crc16_ccitt_false(frame[:-2])
    result = {
        "sync_ok": frame[:2] == FRAME_SYNC,
        "version_ok": frame[2] == PROTOCOL_VERSION,
        "type_ok": frame[3] == FRAME_TYPE_BOTH,
        "flags_ok": frame[10] == FRAME_FLAGS_BOTH,
        "reserved_ok": frame[11] == 0,
        "length_ok": len(frame) == expected_size,
        "crc_ok": received_crc == calculated_crc,
        "sequence": struct.unpack_from("<H", frame, 4)[0],
        "count": count,
        "interval_us": struct.unpack_from("<H", frame, 8)[0],
        "crc_calculated": calculated_crc,
        "crc_received": received_crc,
        "ch1_first": struct.unpack_from("<H", frame, 12)[0],
        "ch2_first": struct.unpack_from("<H", frame, 14)[0],
    }
    result["all_ok"] = all(
        result[key]
        for key in (
            "sync_ok",
            "version_ok",
            "type_ok",
            "flags_ok",
            "reserved_ok",
            "length_ok",
            "crc_ok",
        )
    )
    return result


def build_control(index: int, interval_us: int) -> bytes:
    packet = bytearray(9)
    packet[0:2] = CONTROL_SYNC
    packet[2] = PROTOCOL_VERSION
    packet[3] = 0x01
    packet[4] = index
    struct.pack_into("<H", packet, 5, interval_us)
    struct.pack_into("<H", packet, 7, crc16_ccitt_false(packet[:7]))
    return bytes(packet)


def parse_control(packet: bytes) -> dict[str, int | bool]:
    if len(packet) != 9:
        raise ValueError(f"control size {len(packet)} != 9")
    received_crc = struct.unpack_from("<H", packet, 7)[0]
    calculated_crc = crc16_ccitt_false(packet[:7])
    result = {
        "sync_ok": packet[:2] == CONTROL_SYNC,
        "version_ok": packet[2] == PROTOCOL_VERSION,
        "command_ok": packet[3] == 0x01,
        "crc_ok": received_crc == calculated_crc,
        "timebase_index": packet[4],
        "interval_us": struct.unpack_from("<H", packet, 5)[0],
        "all_ok": False,
    }
    result["all_ok"] = all(
        result[key] for key in ("sync_ok", "version_ok", "command_ok", "crc_ok")
    )
    return result


def main() -> int:
    test_frame = build_frame(sequence=7, interval_us=100)
    frame_result = parse_frame(test_frame)
    if not frame_result["all_ok"]:
        print(f"FAIL built data frame: {frame_result}")
        return 1

    for index, interval in enumerate((200, 100, 20)):
        control = build_control(index, interval)
        control_result = parse_control(control)
        if not control_result["all_ok"]:
            print(f"FAIL control {index}: {control_result}")
            return 1

    corrupted = bytearray(test_frame)
    corrupted[100] ^= 0xFF
    if parse_frame(bytes(corrupted))["crc_ok"]:
        print("FAIL corrupted frame was accepted")
        return 1

    print("PASS: built frame and control packets are valid")
    print(f"frame={frame_result}")

    if len(sys.argv) > 1:
        memory_frame = Path(sys.argv[1]).read_bytes()
        memory_result = parse_frame(memory_frame)
        print(f"captured_frame={memory_result}")
        if not memory_result["all_ok"]:
            return 1
        print("PASS: captured MCU frame matches the specification")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
