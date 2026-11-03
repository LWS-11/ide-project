"""ReceiverPart1-compatible acquisition protocol helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

DATA_SYNC = b"\xA5\x5A"
CONTROL_SYNC = b"\xC5\x5C"
VERSION = 0x01

TYPE_CH1 = 0x10
TYPE_CH2 = 0x20
TYPE_DUAL = 0x30

FLAG_CH1 = 0x01
FLAG_CH2 = 0x02
FLAG_DUAL = 0x03
FLAG_DIGITAL = 0x04
FLAG_ALL = 0x07

DATA_HEADER_SIZE = 12
CONTROL_FRAME_SIZE = 9
SAMPLES_PER_CHANNEL = 500
BYTES_PER_POINT = 5

SCAN_10MS = 0
SCAN_5MS = 1
SCAN_1MS = 2
SCAN_500US = 3
SCAN_100US = 4
SCAN_INTERVALS_US = (200, 100, 20, 10, 2)
SCAN_LABELS = ("10 ms", "5 ms", "1 ms", "500 us", "100 us")


def crc16_ccitt_false(data: bytes | bytearray | memoryview) -> int:
    """Return CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF."""

    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def scan_mode_from_interval(interval_us: int) -> int:
    try:
        return SCAN_INTERVALS_US.index(interval_us)
    except ValueError as exc:
        raise ValueError(f"Unsupported sample interval: {interval_us} us") from exc


@dataclass(frozen=True)
class DataFrame:
    frame_type: int
    channel_flags: int
    sample_count: int
    interval_us: int
    sequence: int
    ch1_mv: tuple[int, ...]
    ch2_mv: tuple[int, ...]
    digital: tuple[int, ...]

    @property
    def scan_mode(self) -> int:
        return scan_mode_from_interval(self.interval_us)

    @property
    def scan_seconds(self) -> float:
        return self.interval_us * self.sample_count / 1_000_000.0


@dataclass(frozen=True)
class ControlFrame:
    mode: int
    interval_us: int


def build_control(mode: int) -> bytes:
    """Build the 9-byte ReceiverPart1 timebase-control packet."""

    if mode < SCAN_10MS or mode >= len(SCAN_INTERVALS_US):
        raise ValueError(f"Invalid scan mode: {mode}")

    interval = SCAN_INTERVALS_US[mode]
    body = (
        CONTROL_SYNC
        + bytes((VERSION, 0x01, mode))
        + interval.to_bytes(2, "little")
    )
    crc = crc16_ccitt_false(body)
    return body + bytes((crc & 0xFF, crc >> 8))


def build_data_frame(
    ch1_mv: Iterable[int],
    ch2_mv: Iterable[int],
    mode: int,
    sequence: int,
    digital_samples: Iterable[int] | None = None,
) -> bytes:
    """Build a dual-channel data frame for tests and replay."""

    ch1 = [int(value) & 0xFFFF for value in ch1_mv]
    ch2 = [int(value) & 0xFFFF for value in ch2_mv]
    digital = (
        [0] * len(ch1)
        if digital_samples is None
        else [int(value) & 0x07 for value in digital_samples]
    )
    if not ch1 or len(ch1) != len(ch2) or len(ch1) > 500:
        raise ValueError("CH1/CH2 must contain the same positive number of points")
    if len(digital) != len(ch1):
        raise ValueError("Digital sample count must match the analog count")
    if mode < SCAN_10MS or mode >= len(SCAN_INTERVALS_US):
        raise ValueError(f"Invalid scan mode: {mode}")

    points = len(ch1)
    header = (
        DATA_SYNC
        + bytes((VERSION, TYPE_DUAL))
        + int(sequence & 0xFFFF).to_bytes(2, "little")
        + points.to_bytes(2, "little")
        + SCAN_INTERVALS_US[mode].to_bytes(2, "little")
        + bytes((FLAG_ALL, 0))
    )

    payload = bytearray()
    for value_ch1, value_ch2, digital_value in zip(ch1, ch2, digital):
        payload.extend(value_ch1.to_bytes(2, "little"))
        payload.extend(value_ch2.to_bytes(2, "little"))
        payload.append(digital_value)

    without_crc = header + payload
    crc = crc16_ccitt_false(without_crc)
    return without_crc + bytes((crc & 0xFF, crc >> 8))


def decode_data_frame(frame: bytes) -> DataFrame:
    if len(frame) < DATA_HEADER_SIZE + 2 or frame[:2] != DATA_SYNC:
        raise ValueError("Invalid data-frame header")
    if frame[2] != VERSION:
        raise ValueError("Unsupported protocol version")

    frame_type = frame[3]
    point_count = int.from_bytes(frame[6:8], "little")
    interval_us = int.from_bytes(frame[8:10], "little")
    flags = frame[10]
    if point_count < 1 or point_count > 500:
        raise ValueError("Invalid point count")
    if flags & ~FLAG_ALL:
        raise ValueError("Invalid channel flags")

    analog_flags = flags & FLAG_DUAL
    channel_count = {FLAG_CH1: 1, FLAG_CH2: 1, FLAG_DUAL: 2}.get(analog_flags)
    expected_type = {
        FLAG_CH1: TYPE_CH1,
        FLAG_CH2: TYPE_CH2,
        FLAG_DUAL: TYPE_DUAL,
    }.get(analog_flags)
    if channel_count is None or expected_type is None:
        raise ValueError("Invalid channel flags")
    if frame_type != expected_type:
        raise ValueError("Frame type and channel flags disagree")

    has_digital = bool(flags & FLAG_DIGITAL)
    bytes_per_point = channel_count * 2 + int(has_digital)
    payload_length = point_count * bytes_per_point
    expected_length = DATA_HEADER_SIZE + payload_length + 2
    if len(frame) != expected_length:
        raise ValueError("Invalid data-frame length")
    scan_mode_from_interval(interval_us)

    expected_crc = int.from_bytes(frame[-2:], "little")
    if crc16_ccitt_false(frame[:-2]) != expected_crc:
        raise ValueError("Data-frame CRC mismatch")

    ch1: list[int] = []
    ch2: list[int] = []
    digital: list[int] = []
    index = DATA_HEADER_SIZE
    for _ in range(point_count):
        if analog_flags == FLAG_CH1:
            ch1.append(int.from_bytes(frame[index : index + 2], "little"))
            index += 2
        elif analog_flags == FLAG_CH2:
            ch2.append(int.from_bytes(frame[index : index + 2], "little"))
            index += 2
        else:
            ch1.append(int.from_bytes(frame[index : index + 2], "little"))
            index += 2
            ch2.append(int.from_bytes(frame[index : index + 2], "little"))
            index += 2
        if has_digital:
            digital.append(frame[index] & 0x07)
            index += 1

    return DataFrame(
        frame_type=frame_type,
        channel_flags=flags,
        sample_count=point_count,
        interval_us=interval_us,
        sequence=int.from_bytes(frame[4:6], "little"),
        ch1_mv=tuple(ch1),
        ch2_mv=tuple(ch2),
        digital=tuple(digital),
    )


def decode_control_frame(frame: bytes) -> ControlFrame:
    if len(frame) != CONTROL_FRAME_SIZE or frame[:2] != CONTROL_SYNC:
        raise ValueError("Invalid control-frame header")
    if frame[2] != VERSION or frame[3] != 0x01:
        raise ValueError("Unsupported control command")
    expected_crc = int.from_bytes(frame[-2:], "little")
    if crc16_ccitt_false(frame[:-2]) != expected_crc:
        raise ValueError("Control-frame CRC mismatch")

    mode = frame[4]
    interval_us = int.from_bytes(frame[5:7], "little")
    if mode >= len(SCAN_INTERVALS_US) or interval_us != SCAN_INTERVALS_US[mode]:
        raise ValueError("Control-frame timebase mismatch")
    return ControlFrame(mode=mode, interval_us=interval_us)


class FrameParser:
    """Incremental parser for acquisition data and timebase-control packets."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.crc_errors = 0
        self.unknown_frames = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes | bytearray) -> list[DataFrame | ControlFrame]:
        self._buffer.extend(data)
        frames: list[DataFrame | ControlFrame] = []

        while True:
            data_index = self._buffer.find(DATA_SYNC)
            control_index = self._buffer.find(CONTROL_SYNC)
            candidates = [index for index in (data_index, control_index) if index >= 0]
            if not candidates:
                if len(self._buffer) > 1:
                    self.discarded_bytes += len(self._buffer) - 1
                    del self._buffer[:-1]
                break

            sync_index = min(candidates)
            if sync_index > 0:
                self.discarded_bytes += sync_index
                del self._buffer[:sync_index]

            if self._buffer.startswith(CONTROL_SYNC):
                frame_size = CONTROL_FRAME_SIZE
            elif self._buffer.startswith(DATA_SYNC):
                if len(self._buffer) < DATA_HEADER_SIZE:
                    break
                flags = self._buffer[10]
                if flags & ~FLAG_ALL:
                    self.unknown_frames += 1
                    self.discarded_bytes += 1
                    del self._buffer[0]
                    continue
                point_count = int.from_bytes(self._buffer[6:8], "little")
                if point_count < 1 or point_count > 500:
                    self.unknown_frames += 1
                    self.discarded_bytes += 1
                    del self._buffer[0]
                    continue
                analog_flags = flags & FLAG_DUAL
                if analog_flags not in (FLAG_CH1, FLAG_CH2, FLAG_DUAL):
                    self.unknown_frames += 1
                    self.discarded_bytes += 1
                    del self._buffer[0]
                    continue
                channel_count = 2 if analog_flags == FLAG_DUAL else 1
                bytes_per_point = channel_count * 2
                if flags & FLAG_DIGITAL:
                    bytes_per_point += 1
                frame_size = DATA_HEADER_SIZE + point_count * bytes_per_point + 2
            else:
                break

            if len(self._buffer) < frame_size:
                break

            candidate = bytes(self._buffer[:frame_size])
            try:
                if candidate[:2] == CONTROL_SYNC:
                    frames.append(decode_control_frame(candidate))
                else:
                    frames.append(decode_data_frame(candidate))
                del self._buffer[:frame_size]
            except ValueError:
                self.crc_errors += 1
                self.discarded_bytes += 1
                del self._buffer[0]

        return frames
