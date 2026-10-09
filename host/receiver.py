import argparse
import struct
import threading
import queue

# ============================================================
# Serial configuration
# ============================================================

BAUDRATE = 115200
MIN_SAMPLE_PERIOD_MS = 10
MAX_SAMPLE_PERIOD_MS = 1000

from protocol import (
    FrameParser, build_frame, CMD_START, CMD_STOP, CMD_SET_RATE, CMD_STATUS,
    FRAME_TYPE_SENSOR, FRAME_TYPE_STATUS,
)

# Thread-safe queue for commands typed by the user
command_queue = queue.Queue()


def send_command(ser, frame_type, payload=b""):
    frame = build_frame(frame_type, payload)
    print("TX:", frame.hex(" "))
    if ser.write(frame) != len(frame):
        raise OSError("Incomplete command write")
    ser.flush()


# ============================================================
# Keyboard input runs separately from serial reception
# ============================================================

def keyboard_worker():
    print("\nCommands:")
    print("  start:       resume sampling")
    print("  stop:        top sampling")
    print("  rate 200:    sample every 200 ms")
    print("  status:      request current status")
    print("  quit:        exit program")

    while True:
        try:
            command = input("\nCommand> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            return

        if command == "quit":
            command_queue.put(("quit", None))
            return

        elif command == "start":
            command_queue.put(("send", (CMD_START, b"")))

        elif command == "stop":
            command_queue.put(("send", (CMD_STOP, b"")))

        elif command == "status":
            command_queue.put(("send", (CMD_STATUS, b"")))

        elif command.split()[:1] == ["rate"]:
            try:
                parts = command.split()
                if len(parts) == 1:
                    print("Missing period: use rate 200.")
                    continue
                if len(parts) != 2:
                    print("Usage: rate 200 (one whole-number period in ms).")
                    continue
                period_ms = int(parts[1])

                if not MIN_SAMPLE_PERIOD_MS <= period_ms <= MAX_SAMPLE_PERIOD_MS:
                    print(f"Period must be between {MIN_SAMPLE_PERIOD_MS} and {MAX_SAMPLE_PERIOD_MS} ms.")
                    continue

                payload = struct.pack("<I", period_ms)
                command_queue.put(("send", (CMD_SET_RATE, payload)))

            except ValueError:
                print("Invalid period: enter a whole number, for example rate 200.")

        else:
            print("Unknown command. Try start, stop, rate 200, status, or quit.")


# ============================================================
# Incoming frame parser
# ============================================================

def process_frame(frame):
    frame_type = frame[3]
    payload_length = frame[4]

    if frame_type == FRAME_TYPE_SENSOR and payload_length == 10:
        sequence = struct.unpack_from("<I", frame, 5)[0]
        timestamp = struct.unpack_from("<I", frame, 9)[0]
        adc_value = struct.unpack_from("<H", frame, 13)[0]

        print(
            f"SEQ={sequence}, "
            f"TIME={timestamp} ms, "
            f"ADC={adc_value}, CRC=OK"
        )

    elif frame_type == FRAME_TYPE_STATUS and payload_length == 5:
        sampling_enabled = frame[5]
        sample_period_ms = struct.unpack_from("<I", frame, 6)[0]

        print(
            f"STATUS: sampling="
            f"{'ON' if sampling_enabled else 'OFF'}, "
            f"period={sample_period_ms} ms"
        )

    else:
        print(
            f"Unknown frame: type=0x{frame_type:02X}, "
            f"payload_length={payload_length}"
        )


# ============================================================
# Main
# ============================================================

def is_stlink_port(port):
    # Description/product labels identify the USB bridge, not the firmware.
    # Avoid selecting every STMicroelectronics USB device by vendor ID alone.
    labels = " ".join(str(getattr(port, field, None) or "")
                      for field in ("description", "product"))
    normalized = "".join(char for char in labels.casefold() if char.isalnum())
    return "stlink" in normalized


def print_ports(ports):
    if not ports:
        print("No serial ports found.")
        return
    for port in ports:
        vid = getattr(port, "vid", None)
        pid = getattr(port, "pid", None)
        usb_id = f"{vid:04X}:{pid:04X}" if vid is not None and pid is not None else "unknown"
        serial_number = getattr(port, "serial_number", None) or "unavailable"
        print(f"{port.device}: {port.description} | USB={usb_id} | serial={serial_number}")


def select_port(ports, explicit_port=None, serial_number=None):
    if explicit_port:
        # Permit direct paths even if enumeration does not expose the port.
        return explicit_port
    if serial_number:
        candidates = [port for port in ports
                      if getattr(port, "serial_number", None) is not None
                      and port.serial_number.casefold() == serial_number.casefold()]
    else:
        candidates = [port for port in ports if is_stlink_port(port)]
    if len(candidates) == 1:
        return candidates[0].device
    if not candidates:
        reason = "No port matches that serial number." if serial_number else "No recognizable ST-LINK serial port found."
        raise ValueError(reason + " Use --list-ports, then --port PORT.")
    raise ValueError("Multiple matching serial ports found. Use --port PORT or a unique --serial-number SERIAL.")


def main():
    argument_parser = argparse.ArgumentParser(description="STM32 sensor stream receiver")
    selection = argument_parser.add_mutually_exclusive_group()
    selection.add_argument("--port", help="Explicit port, e.g. COM5 or /dev/ttyACM0")
    selection.add_argument("--serial-number", help="USB serial number shown by --list-ports")
    argument_parser.add_argument("--list-ports", action="store_true", help="List ports and exit")
    args = argument_parser.parse_args()

    # The offline tests need only the standard library, not a serial port.
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        print("Install pyserial first: python -m pip install pyserial")
        return

    try:
        ports = sorted(list_ports.comports(), key=lambda port: port.device)
        if args.list_ports:
            print_ports(ports)
            return
        port_name = select_port(ports, args.port, args.serial_number)
    except (ValueError, OSError) as exc:
        print(f"Port selection failed: {exc}")
        if "ports" in locals():
            print_ports(ports)
        return

    try:
        ser = serial.Serial(port_name, BAUDRATE, timeout=0.1, write_timeout=1.0)
    except serial.SerialException as exc:
        print(f"Could not open {port_name}: {exc}")
        return

    print(f"Connected to {port_name} at {BAUDRATE} baud.")

    # Start keyboard input without blocking serial reception.
    keyboard_thread = threading.Thread(
        target=keyboard_worker,
        daemon=True
    )
    keyboard_thread.start()

    parser = FrameParser(process_frame)

    try:
        while True:
            # Send commands entered by the user.
            while not command_queue.empty():
                action, payload = command_queue.get_nowait()

                if action == "quit":
                    return

                frame_type, frame_payload = payload

                try:
                    send_command(ser, frame_type, frame_payload)
                    print(f"Sent command 0x{frame_type:02X}")
                except (serial.SerialException, OSError) as exc:
                    print(f"Command transmission failed: {exc}")

            # Receive telemetry and responses.
            try:
                data = ser.read(128)
            except serial.SerialException as exc:
                print(f"Serial receive error: {exc}")
                break

            if data:
                parser.feed(data)

    except KeyboardInterrupt:
        print("\nStopping receiver.")

    finally:
        ser.close()
        print("Serial port closed.")


if __name__ == "__main__":
    main()
