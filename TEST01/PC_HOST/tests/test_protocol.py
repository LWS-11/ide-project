import pathlib
import sys
import unittest

HOST_DIR = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOST_DIR))

import pc_protocol as protocol


class ProtocolTests(unittest.TestCase):
    def test_crc_known_vector(self):
        self.assertEqual(protocol.crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_dual_frame_round_trip(self):
        ch1 = [index * 6 for index in range(500)]
        ch2 = [3300 - index for index in range(500)]
        digital = [index & 0x07 for index in range(500)]
        raw = protocol.build_data_frame(
            ch1, ch2, protocol.SCAN_1MS, 65535, digital
        )
        self.assertEqual(len(raw), 2514)
        self.assertEqual(
            raw[:12],
            b"\xA5\x5A\x01\x30\xFF\xFF\xF4\x01\x14\x00\x07\x00",
        )

        parsed = protocol.decode_data_frame(raw)
        self.assertEqual(parsed.sequence, 65535)
        self.assertEqual(parsed.sample_count, 500)
        self.assertEqual(parsed.interval_us, 20)
        self.assertEqual(parsed.scan_mode, protocol.SCAN_1MS)
        self.assertEqual(parsed.ch1_mv, tuple(ch1))
        self.assertEqual(parsed.ch2_mv, tuple(ch2))
        self.assertEqual(parsed.digital, tuple(digital))

    def test_control_packet_round_trip(self):
        for mode, interval in enumerate(protocol.SCAN_INTERVALS_US):
            raw = protocol.build_control(mode)
            self.assertEqual(len(raw), 9)
            self.assertEqual(raw[:2], b"\xC5\x5C")
            parsed = protocol.decode_control_frame(raw)
            self.assertEqual(parsed.mode, mode)
            self.assertEqual(parsed.interval_us, interval)

    def test_incremental_parser_handles_partial_and_sticky_frames(self):
        first = protocol.build_data_frame(
            [100] * 500,
            [200] * 500,
            protocol.SCAN_10MS,
            7,
        )
        control = protocol.build_control(protocol.SCAN_5MS)
        second = protocol.build_data_frame(
            [300] * 500,
            [400] * 500,
            protocol.SCAN_5MS,
            8,
        )
        parser = protocol.FrameParser()
        parsed = []
        combined = b"\x00\x11" + first + control + second
        for index in range(0, len(combined), 17):
            parsed.extend(parser.feed(combined[index : index + 17]))

        self.assertEqual(
            [type(item) for item in parsed],
            [protocol.DataFrame, protocol.ControlFrame, protocol.DataFrame],
        )
        self.assertEqual(parsed[0].sequence, 7)
        self.assertEqual(parsed[1].mode, protocol.SCAN_5MS)
        self.assertEqual(parsed[2].sequence, 8)
        self.assertGreaterEqual(parser.discarded_bytes, 2)

    def test_corrupted_frame_is_counted_and_parser_recovers(self):
        corrupted = bytearray(
            protocol.build_data_frame([0] * 500, [0] * 500, protocol.SCAN_10MS, 1)
        )
        corrupted[20] ^= 0x40
        valid = protocol.build_data_frame(
            [123] * 500,
            [456] * 500,
            protocol.SCAN_1MS,
            2,
        )
        parser = protocol.FrameParser()
        parsed = parser.feed(bytes(corrupted) + valid)
        self.assertEqual(len(parsed), 1)
        self.assertEqual(parsed[0].sequence, 2)
        self.assertGreaterEqual(parser.crc_errors, 1)


if __name__ == "__main__":
    unittest.main()
