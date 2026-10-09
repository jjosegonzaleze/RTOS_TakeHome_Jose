# STM32 RTOS Sensor Streaming

A NUCLEO-F446RE samples an analog input, timestamps each reading, and streams binary frames to a Python host through the onboard ST-LINK virtual COM port. The host displays readings and sends commands to start or stop sampling, change the sample period, and request status.

## Hardware and software

- STM32 NUCLEO-F446RE and a USB data cable connected to the ST-LINK USB connector.
- Analog sensor signal on **PA4 / ADC1_IN4**, using the board's ground reference and supported ADC input range. A floating input produces readings but does not provide a meaningful sensor measurement.
- STM32CubeIDE **2.2.0**, with its bundled ARM compiler and ST-LINK support.
- **FreeRTOS**, accessed through **CMSIS-RTOS2**; RTOS tick rate is 1 kHz.
- Python **3.10 or newer**, Git, and `pyserial==3.5`.
- On Windows, install the ST-LINK USB driver if the virtual COM port does not appear.

The sensor value is a raw 12-bit ADC count, not a calibrated physical measurement.

## Repository layout

Place the existing STM32CubeIDE project at the repository root and the Python files together under `host/`:

```text
README.md
.project
.cproject
.mxproject
.settings/
RTOS_TakeHome_Jose.ioc
Core/
Drivers/
Middlewares/
STM32F446RETX_FLASH.ld
STM32F446RETX_RAM.ld
host/
    receiver.py
    protocol.py
    test_protocol.py
    requirements.txt
```

Commit the project sources, supplied drivers and middleware, startup files, linker scripts, and IDE configuration. Generated `Debug/` and `Release/` output, Python `.venv/`, `__pycache__/`, and editor-local files are not needed to build from source. Preserve the project name `RTOS_TakeHome_Jose` when importing it.

## Clone, build, and flash

1. Clone this repository using its Git hosting URL and open STM32CubeIDE with a fresh workspace.
2. Select **File → Import → General → Existing Projects into Workspace**. Choose the cloned repository as the root directory and import `RTOS_TakeHome_Jose`.
3. Select the **Debug** build configuration and build the project. STM32CubeIDE generates the build files and produces `Debug/RTOS_TakeHome_Jose.elf`; a previously generated Debug directory is not required.
4. Connect the NUCLEO board through its ST-LINK USB connector.
5. Launch a **STM32 Cortex-M C/C++ Application** debug configuration for the project, using ST-LINK and the newly built ELF. If a supplied launch configuration contains machine-specific settings, create a new configuration in the IDE.
6. After programming, press **Resume** if execution is suspended. Avoid breakpoints during serial reception. Alternatively, use the IDE's Run configuration to program and run the board.

The device starts sampling automatically with a default period of **100 ms**. The LED task continues to toggle the onboard LED independently of sampling.

## Set up Python

Open a terminal in the cloned repository. On Windows PowerShell:

```powershell
cd host
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Using the virtual environment's Python directly avoids needing to activate it. If your environment is already active, `python` can replace `.\.venv\Scripts\python.exe` in the commands below.

`requirements.txt` contains:

```text
pyserial==3.5
```

## Find the port and run

List ports and their USB serial numbers:

```powershell
.\.venv\Scripts\python.exe receiver.py --list-ports
```

Automatically select a single recognizable ST-LINK serial port:

```powershell
.\.venv\Scripts\python.exe receiver.py
```

Select a port explicitly:

```powershell
.\.venv\Scripts\python.exe receiver.py --port COM5
```

**COM5 is an example from the development computer; use the port listed on your machine.** On other platforms, the explicit port can be a device path such as `/dev/ttyACM0`.

If several boards are connected, select one by USB serial number:

```powershell
.\.venv\Scripts\python.exe receiver.py --serial-number YOUR_USB_SERIAL
```

Replace `YOUR_USB_SERIAL` with the value shown by `--list-ports`. Detection uses the USB bridge's description/product metadata. It does not identify the firmware running on the target MCU, and it refuses to guess when multiple ports match. Use `--port` if the bridge label is unavailable.

The serial link uses **115200 baud, 8 data bits, no parity, 1 stop bit, no hardware flow control**.

## Commands and output

Once the receiver is running, type these commands at its `Command>` prompt:

| Command | Effect |
| --- | --- |
| `start` | Resume sampling |
| `stop` | Disable sampling; already queued/in-flight telemetry may still arrive |
| `rate 200` | Set the sample period to 200 milliseconds |
| `status` | Report sampling state and configured period |
| `quit` | Close the host receiver; the MCU continues running |

Commands are case-insensitive. The host accepts integer periods from **10 to 1,000 ms**, rejects out-of-range values, and explains nonnumeric or missing input. `rate` specifies a period, not a frequency: 200 ms corresponds to approximately 5 readings per second.

**Current firmware limit:** Both firmware and host enforce a sample-period range of 10–1,000 ms. Invalid values leave the current period unchanged.

Example readings and status:

```text
SEQ=15940, TIME=2103501 ms, ADC=1997, CRC=OK
SEQ=15941, TIME=2103701 ms, ADC=1975, CRC=OK
STATUS: sampling=OFF, period=200 ms
```

`SEQ` is a sample sequence number. `TIME` is elapsed milliseconds since boot, resetting on reboot and wrapping as a 32-bit counter after approximately 49.7 days. ADC conversion and scheduling add overhead, so timing is approximate. Sampling and sequence progression pause when stopped; uptime continues.

Telemetry output may appear between typed characters or command help lines because reception and keyboard input run concurrently. Enter only commands such as `stop`; do not paste output lines into PowerShell.

## RTOS structure and disconnect behavior

Four tasks handle LED activity, ADC sampling, telemetry transmission, and commands. Samples pass through a bounded 16-entry queue. A transmit mutex prevents sensor and status frames from interleaving.

The command task uses one-byte polling reception without sleeping after a successful read. The RTOS scheduler still preempts it. This fixed the observed loss of STOP command bytes caused by delaying between reads; interrupt or DMA reception would improve reliability under heavier scheduling load.

The device does not detect host presence or replay a historical log. It continues sampling and attempting transmission when the host closes. Missed telemetry is not guaranteed to be retained; if the sample queue fills, new samples are dropped. Reopening the receiver resumes decoding subsequent valid frames without resetting the board. Keep the USB connection powered during this test: unplugging the only power source will reboot the MCU.

## Wire format

```text
AA 55 | version:u8 | type:u8 | payload_length:u8 | payload | crc:u16 little-endian
```

Version is `01`. Maximum payload length is 32 bytes. CRC-16/CCITT-FALSE uses polynomial `0x1021`, initial value `0xFFFF`, no reflection, and no final XOR; it covers version, type, length, and payload, excluding synchronization and CRC bytes. Multi-byte payload fields are little-endian.

| Type | Payload |
| --- | --- |
| `01` sensor | sequence:u32, timestamp_ms:u32, adc_value:u16 |
| `02` status | sampling_enabled:u8, period_ms:u32 |
| `10` START | Empty |
| `11` STOP | Empty |
| `12` SET_RATE | period_ms:u32 |
| `13` STATUS request | Empty |

START, STOP, and SET_RATE do not send acknowledgments; request STATUS to inspect the resulting state.

## Automated resilience tests

From `host/`:

```powershell
.\.venv\Scripts\python.exe -m unittest -v test_protocol
```

Expected result: **12 tests, OK**. These tests need no board or serial access. They exercise the same `FrameParser` used by the live receiver, including:

- Every single-bit corruption of a sensor frame, invalid lengths, and bad CRCs.
- Truncated frames, byte insertion/deletion, and attaching at every mid-frame offset.
- Split reads, back-to-back frames, and synchronization bytes inside valid payloads.
- 1,000 repeatable randomized damaged streams, followed by valid recovery frames.
- Sensor/status display decoding and known CRC vectors.

The parser discards invalid candidates and searches for synchronization again. It preserves partial frames between reads and bounds retained data to the maximum 39-byte frame. A plausible corrupted length may delay recovery until enough subsequent bytes arrive for its CRC check. These tests verify host-side R8 recovery for the tested inputs; they do not exercise physical UART errors or firmware malformed-command handling.

## Hardware checks and clean-clone verification

Hardware testing has demonstrated sensor streaming, STOP, START, period changes, STATUS, and closing/reopening the Python receiver while the MCU stays powered. The 12 framing tests have also passed on the development computer. Explicit connection with `--port COM5` has been demonstrated; check automatic detection on the reviewer's machine or use the explicit option.

To verify R9, use a fresh clone rather than the existing development folders, then follow the build/flash and Python steps above. Confirm:

1. Firmware builds from the committed sources and runs on the board.
2. Python dependencies install in a new virtual environment and all 12 tests pass.
3. `stop`, `status`, `start`, and `rate 200` work with the running firmware.
4. `rate 10` and `rate 1000` are accepted; `rate 9`, `rate 1001`, and `rate abc` are rejected by the host.
5. Closing and reopening Python during streaming restores readings and commands without an MCU reset.

A successful run from the development folders alone does not complete the clean-clone verification.

## Troubleshooting

- **Port busy:** close other serial monitors or receiver instances using that port.
- **No ST-LINK match / multiple matches:** use `--list-ports`, then `--port` or `--serial-number`.
- **Connected but no readings:** ensure the MCU is running, then try `status` and `start`.
- **PowerShell says `Command>` or `Sent` is not recognized:** start `receiver.py` first; those words are program output, not shell commands.
- **Missing `protocol` import:** keep all host Python files together and run from `host/`.
- **Missing requirements file:** run dependency installation from the folder containing `requirements.txt`.
