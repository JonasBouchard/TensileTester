import gc
import json
import time

import usb_cdc

from config import LOOP_DELAY_S, SAMPLE_INTERVAL_S, USE_SIMULATION

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


def serial_channel_connected():
    return bool(getattr(SERIAL_CHANNEL, "connected", True))


def write_message(payload):
    if not serial_channel_connected():
        return False

    try:
        SERIAL_CHANNEL.write((json.dumps(payload) + "\n").encode("utf-8"))
    except OSError:
        return False

    return True


def create_backend_for_mode(mode):
    gc.collect()
    if mode == "simulation":
        from simulation import SimulatedTesterBackend

        backend = SimulatedTesterBackend(sample_interval_s=SAMPLE_INTERVAL_S)
        backend.mode = mode
        return backend
    if mode == "hardware":
        from hardware import HardwareTesterBackend

        backend = HardwareTesterBackend()
        backend.mode = mode
        return backend
    raise ValueError("Unsupported mode: %r" % mode)


def main():
    initial_mode = "simulation" if USE_SIMULATION else "hardware"
    current_mode = initial_mode
    backend = create_backend_for_mode(initial_mode)
    pending_startup_status = {
        "type": "status",
        "state": backend.state,
        "message": backend.startup_message,
    }

    while True:
        now = time.monotonic()
        if pending_startup_status is not None and write_message(pending_startup_status):
            pending_startup_status = None

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
                        if mode == current_mode:
                            write_message(
                                {
                                    "type": "status",
                                    "state": backend.state,
                                    "message": backend.startup_message,
                                }
                            )
                            continue

                        old_backend = backend
                        backend = None
                        del old_backend
                        gc.collect()
                        backend = create_backend_for_mode(mode)
                        current_mode = mode
                    except ValueError:
                        write_message(
                            {
                                "type": "error",
                                "code": "invalid_mode",
                                "message": "Mode must be 'simulation' or 'hardware'.",
                            }
                        )
                    except Exception as exc:
                        write_message(
                            {
                                "type": "error",
                                "code": "mode_switch_failed",
                                "message": str(exc) or "Mode switch failed.",
                            }
                        )
                    else:
                        write_message(
                            {
                                "type": "status",
                                "state": backend.state,
                                "message": backend.startup_message,
                            }
                        )
                else:
                    for message in backend.handle_command(command, now):
                        write_message(message)

        for message in backend.poll(now):
            write_message(message)

        time.sleep(LOOP_DELAY_S)


main()
