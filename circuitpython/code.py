import json
import time

import usb_cdc

from config import LOOP_DELAY_S, SAMPLE_INTERVAL_S, USE_SIMULATION
from hardware import HardwareTesterBackend
from simulation import SimulatedTesterBackend

SERIAL_CHANNEL = getattr(usb_cdc, "data", None) or usb_cdc.console
READ_BUFFER = b""

def read_command_line():
    global READ_BUFFER

    waiting = getattr(SERIAL_CHANNEL, "in_waiting", 0)
    if not waiting:
        return None

    raw_chunk = SERIAL_CHANNEL.read(waiting)
    if not raw_chunk:
        return None

    READ_BUFFER += raw_chunk
    if b"\n" not in READ_BUFFER:
        return None

    raw_line, READ_BUFFER = READ_BUFFER.split(b"\n", 1)
    line = raw_line.replace(b"\r", b"").decode("utf-8", "replace").strip()
    return line or None


def write_message(payload):
    SERIAL_CHANNEL.write((json.dumps(payload) + "\n").encode("utf-8"))


def create_backend_for_mode(mode):
    if mode == "simulation":
        return SimulatedTesterBackend(sample_interval_s=SAMPLE_INTERVAL_S)
    if mode == "hardware":
        return HardwareTesterBackend()
    raise ValueError("Unsupported mode: %r" % mode)


def main():
    initial_mode = "simulation" if USE_SIMULATION else "hardware"
    backend = create_backend_for_mode(initial_mode)
    write_message({"type": "status", "state": backend.state, "message": backend.startup_message})

    while True:
        now = time.monotonic()
        raw_line = read_command_line()
        if raw_line is not None:
            try:
                command = json.loads(raw_line)
            except ValueError:
                write_message(
                    {
                        "type": "error",
                        "code": "invalid_json",
                        "message": "Malformed JSON command.",
                    }
                )
            else:
                if str(command.get("cmd", "")) in {"set_mode", "set mode"}:
                    mode = str(command.get("mode", ""))
                    try:
                        backend = create_backend_for_mode(mode)
                    except ValueError:
                        write_message(
                            {
                                "type": "error",
                                "code": "invalid_mode",
                                "message": "Mode must be 'simulation' or 'hardware'.",
                            }
                        )
                    else:
                        write_message(
                            {
                                "type": "status",
                                "state": backend.state,
                                "message": (
                                    "Virtual simulation enabled."
                                    if mode == "simulation"
                                    else "Hardware mode enabled."
                                ),
                            }
                        )
                else:
                    for message in backend.handle_command(command, now):
                        write_message(message)

        for message in backend.poll(now):
            write_message(message)

        time.sleep(LOOP_DELAY_S)


main()
