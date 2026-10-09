import binascii
import contextlib
import io
import random
import struct
import unittest

from protocol import (
    FrameParser, build_frame, crc16_ccitt, MAX_FRAME_LENGTH,
    FRAME_TYPE_SENSOR, FRAME_TYPE_STATUS, CMD_STOP,
)
from receiver import process_frame


def sensor(sequence):
    return build_frame(FRAME_TYPE_SENSOR, struct.pack("<IIH", sequence, sequence * 200, 1986))


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.received = []
        self.parser = FrameParser(self.received.append)
        self.frame = sensor(1)
        # Enough subsequent traffic to resolve even a false length of 32.
        self.recovery = [sensor(i) for i in range(100, 104)]

    def feed_chunks(self, data, rng):
        pos = 0
        while pos < len(data):
            size = rng.randint(1, 23)
            self.parser.feed(data[pos:pos + size])
            self.assertLessEqual(len(self.parser.buffer), MAX_FRAME_LENGTH)
            pos += size

    def check_recovery(self, damaged, seed=0):
        self.received = []
        self.parser = FrameParser(self.received.append)
        self.feed_chunks(damaged + b"".join(self.recovery), random.Random(seed))
        self.assertEqual(self.received[-4:], self.recovery)
        self.assertEqual(self.parser.buffer, bytearray())
        for frame in self.received:
            self.assertEqual(struct.unpack("<H", frame[-2:])[0],
                             binascii.crc_hqx(frame[2:-2], 0xffff))

    def test_known_crc_and_stop(self):
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29b1)
        self.assertEqual(build_frame(CMD_STOP), bytes.fromhex("aa 55 01 11 00 ee cb"))
        with self.assertRaises(ValueError):
            build_frame(1, bytes(33))

    def test_every_split_point(self):
        for split in range(len(self.frame) + 1):
            with self.subTest(split=split):
                received = []
                parser = FrameParser(received.append)
                parser.feed(self.frame[:split])
                parser.feed(self.frame[split:])
                self.assertEqual(received, [self.frame])

    def test_byte_at_a_time_and_back_to_back(self):
        for byte in self.frame * 3:
            self.parser.feed(bytes([byte]))
        self.assertEqual(self.received, [self.frame] * 3)

    def test_corrupted_crc_is_rejected(self):
        damaged = bytearray(self.frame)
        damaged[-1] ^= 1
        self.check_recovery(bytes(damaged))
        self.assertEqual(self.received, self.recovery)
        self.assertGreater(self.parser.crc_errors, 0)

    def test_all_single_bit_corruptions(self):
        for position in range(len(self.frame)):
            for bit in range(8):
                with self.subTest(position=position, bit=bit):
                    damaged = bytearray(self.frame)
                    damaged[position] ^= 1 << bit
                    self.check_recovery(bytes(damaged))
                    self.assertEqual(self.received, self.recovery)

    def test_every_truncation(self):
        for cut in range(len(self.frame)):
            with self.subTest(cut=cut):
                self.check_recovery(self.frame[:cut])
                self.assertEqual(self.received, self.recovery)

    def test_attach_at_every_offset(self):
        for offset in range(1, len(self.frame)):
            with self.subTest(offset=offset):
                self.check_recovery(self.frame[offset:])
                self.assertEqual(self.received, self.recovery)

    def test_invalid_and_plausible_lengths(self):
        for length in range(256):
            if length == 10:
                continue
            with self.subTest(length=length):
                damaged = bytearray(self.frame)
                damaged[4] = length
                self.check_recovery(bytes(damaged))
                self.assertEqual(self.received, self.recovery)

    def test_each_byte_deleted_or_inserted(self):
        for position in range(len(self.frame)):
            with self.subTest(deleted=position):
                self.check_recovery(self.frame[:position] + self.frame[position + 1:])
                self.assertEqual(self.received, self.recovery)
        for position in range(len(self.frame) + 1):
            with self.subTest(inserted=position):
                self.check_recovery(self.frame[:position] + b"\xff" + self.frame[position:])

    def test_sync_inside_valid_payload_and_maximum_length(self):
        frames = [build_frame(1, b"\xaa\x55" * 16), build_frame(2), self.frame]
        self.parser.feed(b"\x00\xaa" + b"".join(frames))
        self.assertEqual(self.received, frames)

    def test_seeded_fuzz_1000_streams(self):
        rng = random.Random(446)
        for trial in range(1000):
            damaged = bytearray(self.frame)
            for _ in range(rng.randint(1, 8)):
                action = rng.randrange(3)
                position = rng.randrange(len(damaged) + 1)
                if action == 0 and position < len(damaged):
                    damaged[position] ^= 1 << rng.randrange(8)
                elif action == 1 and position < len(damaged):
                    del damaged[position]
                else:
                    damaged[position:position] = bytes([rng.randrange(256)])
            noise = bytes(rng.randrange(256) for _ in range(rng.randrange(80)))
            with self.subTest(trial=trial):
                self.check_recovery(noise + bytes(damaged), seed=trial)

    def test_live_display_decodes_sensor_and_status(self):
        parser = FrameParser(process_frame)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            parser.feed(self.frame)
            parser.feed(build_frame(FRAME_TYPE_STATUS, struct.pack("<BI", 0, 200)))
        self.assertIn("SEQ=1, TIME=200 ms, ADC=1986, CRC=OK", output.getvalue())
        self.assertIn("STATUS: sampling=OFF, period=200 ms", output.getvalue())


if __name__ == "__main__":
    unittest.main(verbosity=2)
