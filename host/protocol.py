import struct

SYNC = b"\xaa\x55"
VERSION = 1
MAX_PAYLOAD_LENGTH = 32
MAX_FRAME_LENGTH = 7 + MAX_PAYLOAD_LENGTH
FRAME_TYPE_SENSOR = 0x01
FRAME_TYPE_STATUS = 0x02
CMD_START = 0x10
CMD_STOP = 0x11
CMD_SET_RATE = 0x12
CMD_STATUS = 0x13


def crc16_ccitt(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xffff
    return crc


def build_frame(frame_type, payload=b""):
    if len(payload) > MAX_PAYLOAD_LENGTH:
        raise ValueError("Payload is too long")
    body = bytes((VERSION, frame_type, len(payload))) + payload
    return SYNC + body + struct.pack("<H", crc16_ccitt(body))


class FrameParser:
    """Incremental parser. Only complete CRC-valid frames reach on_frame.

    A plausible damaged length can delay recovery until that candidate has
    enough bytes for a CRC check (at most 39 bytes). No idle timeout is used:
    a quiet or slowly delivered valid partial frame remains buffered.
    """

    def __init__(self, on_frame):
        self.on_frame = on_frame
        self.buffer = bytearray()
        self.valid_frames = 0
        self.bad_headers = 0
        self.crc_errors = 0
        self.discarded_bytes = 0

    def _discard(self, count):
        del self.buffer[:count]
        self.discarded_bytes += count

    def feed(self, data):
        # Drain after each byte so even a large input chunk cannot make the
        # retained buffer grow past the maximum frame length.
        for byte in data:
            self.buffer.append(byte)
            self._drain()

    def _drain(self):
        while True:
            sync_index = self.buffer.find(SYNC)
            if sync_index < 0:
                keep = int(bool(self.buffer) and self.buffer[-1] == SYNC[0])
                self._discard(len(self.buffer) - keep)
                return
            if sync_index:
                self._discard(sync_index)
            if len(self.buffer) < 5:
                return
            payload_length = self.buffer[4]
            if self.buffer[2] != VERSION or payload_length > MAX_PAYLOAD_LENGTH:
                self.bad_headers += 1
                self._discard(1)
                continue
            total_length = 7 + payload_length
            if len(self.buffer) < total_length:
                return
            frame = bytes(self.buffer[:total_length])
            received_crc = struct.unpack_from("<H", frame, total_length - 2)[0]
            if received_crc != crc16_ccitt(frame[2:-2]):
                self.crc_errors += 1
                self._discard(1)
                continue
            del self.buffer[:total_length]
            self.valid_frames += 1
            self.on_frame(frame)
