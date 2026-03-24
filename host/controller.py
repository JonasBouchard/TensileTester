from __future__ import annotations

import json
import queue
import re
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Protocol


DEFAULT_BAUD_RATE = 115200
SIMULATION_CONNECTION_LABEL = "Virtual Simulation"
PY_SERIAL_MISSING_MESSAGE = (
    "pyserial is not installed. Install it with 'py -3 -m pip install pyserial'."
)
MICROCONTROLLER_REPL_MESSAGE = (
    "CircuitPython console detected. Copy 'circuitpython/boot.py' to CIRCUITPY and hard reset the board."
)


@dataclass(frozen=True)
class SpecimenDimensions:
    area_mm2: float
    gauge_length_mm: float


@dataclass(frozen=True)
class SerialPortInfo:
    device: str
    description: str
    hwid: str
    label: str


@dataclass(frozen=True)
class TesterEvent:
    kind: str
    state: str = ""
    message: str = ""
    code: str = ""
    timestamp_s: float | None = None
    force_n: float | None = None
    displacement_mm: float | None = None
    stress_mpa: float | None = None
    strain_percent: float | None = None

    @classmethod
    def status(cls, state: str, message: str = "") -> "TesterEvent":
        return cls(kind="status", state=state, message=message)

    @classmethod
    def sample(
        cls,
        timestamp_s: float,
        force_n: float,
        displacement_mm: float,
        stress_mpa: float | None,
        strain_percent: float | None,
        state: str = "",
    ) -> "TesterEvent":
        return cls(
            kind="sample",
            state=state,
            timestamp_s=timestamp_s,
            force_n=force_n,
            displacement_mm=displacement_mm,
            stress_mpa=stress_mpa,
            strain_percent=strain_percent,
        )

    @classmethod
    def error(cls, code: str, message: str) -> "TesterEvent":
        return cls(kind="error", code=code, message=message)


class Transport(Protocol):
    def close(self) -> None:
        ...

    def read_line(self, timeout: float = 0.1) -> str | None:
        ...

    def write_command(self, command: dict[str, Any]) -> None:
        ...


def validate_specimen_dimensions(
    area_mm2: float,
    gauge_length_mm: float,
) -> SpecimenDimensions:
    area_value = float(area_mm2)
    gauge_value = float(gauge_length_mm)

    if area_value <= 0:
        raise ValueError("Area must be greater than zero.")
    if gauge_value <= 0:
        raise ValueError("Gauge length must be greater than zero.")

    return SpecimenDimensions(
        area_mm2=area_value,
        gauge_length_mm=gauge_value,
    )


def compute_derived_values(
    force_n: float,
    displacement_mm: float,
    specimen: SpecimenDimensions,
) -> tuple[float, float]:
    stress_mpa = float(force_n) / specimen.area_mm2
    strain_percent = float(displacement_mm) / specimen.gauge_length_mm * 100.0
    return stress_mpa, strain_percent


def parse_device_message(
    raw_message: str,
    specimen: SpecimenDimensions | None = None,
    fallback_state: str = "",
) -> TesterEvent:
    cleaned_message = raw_message.strip()
    if not cleaned_message:
        return TesterEvent.status(fallback_state or "connected")

    if not cleaned_message.startswith("{"):
        preview = _sanitize_plain_device_output(cleaned_message)
        if _looks_like_circuitpython_console(preview):
            return TesterEvent.status(
                state=fallback_state or "connected",
                message=MICROCONTROLLER_REPL_MESSAGE,
            )

        if not preview:
            return TesterEvent.status(fallback_state or "connected")

        preview = preview if len(preview) <= 120 else f"{preview[:117]}..."
        return TesterEvent.status(
            state=fallback_state or "connected",
            message=f"Device output: {preview}",
        )

    try:
        payload = json.loads(cleaned_message)
    except json.JSONDecodeError as exc:
        return TesterEvent.error("invalid_json", f"Malformed JSON: {exc.msg}")

    message_type = payload.get("type")

    if message_type == "status":
        return TesterEvent.status(
            state=str(payload.get("state", fallback_state or "unknown")),
            message=str(payload.get("message", "")),
        )

    if message_type == "error":
        return TesterEvent.error(
            code=str(payload.get("code", "device_error")),
            message=str(payload.get("message", "Unknown device error.")),
        )

    if message_type == "sample":
        try:
            timestamp_s = float(payload["timestamp_s"])
            force_n = float(payload["force_n"])
            displacement_mm = float(payload["displacement_mm"])
        except (KeyError, TypeError, ValueError) as exc:
            return TesterEvent.error(
                "invalid_sample",
                f"Invalid sample payload: {exc}",
            )

        stress_mpa = None
        strain_percent = None
        if specimen is not None:
            stress_mpa, strain_percent = compute_derived_values(
                force_n=force_n,
                displacement_mm=displacement_mm,
                specimen=specimen,
            )

        return TesterEvent.sample(
            timestamp_s=timestamp_s,
            force_n=force_n,
            displacement_mm=displacement_mm,
            stress_mpa=stress_mpa,
            strain_percent=strain_percent,
            state=str(payload.get("state", fallback_state)),
        )

    return TesterEvent.error(
        "unknown_message_type",
        f"Unknown device message type: {message_type!r}",
    )


def list_serial_ports() -> list[str]:
    return [port.device for port in list_serial_port_infos()]


def list_serial_port_infos() -> list[SerialPortInfo]:
    try:
        from serial.tools import list_ports
    except ImportError:
        return []

    port_infos = list(list_ports.comports())
    port_infos.sort(key=_serial_port_sort_key)

    results: list[SerialPortInfo] = []
    for port in port_infos:
        description = str(getattr(port, "description", "") or "Serial Port")
        hwid = str(getattr(port, "hwid", "") or "")
        label = _build_serial_port_label(port.device, description, port)
        results.append(
            SerialPortInfo(
                device=port.device,
                description=description,
                hwid=hwid,
                label=label,
            )
        )

    return results


def get_serial_support_error() -> str | None:
    try:
        from serial.tools import list_ports
    except ImportError:
        return PY_SERIAL_MISSING_MESSAGE

    _ = list_ports
    return None


def _build_serial_port_label(device: str, description: str, port: Any) -> str:
    suffix = f"({device})"
    normalized_description = description.removesuffix(suffix).rstrip()
    vid = getattr(port, "vid", None)

    if vid == 0x239A and "usb serial device" in normalized_description.lower():
        normalized_description = "CircuitPython USB Serial"

    label = device
    if normalized_description and normalized_description != device:
        label = f"{device} - {normalized_description}"

    return label


def _serial_port_sort_key(port: Any) -> tuple[int, str]:
    description = str(getattr(port, "description", "") or "")
    hwid = str(getattr(port, "hwid", "") or "")
    combined = f"{port.device} {description} {hwid}".lower()
    vid = getattr(port, "vid", None)

    if vid == 0x239A or "circuitpython" in combined or "usb serial" in combined or "usb" in combined:
        priority = 0
    elif "bluetooth" in combined or "bthenum" in combined:
        priority = 2
    else:
        priority = 1

    return priority, port.device


def _sanitize_plain_device_output(message: str) -> str:
    without_ansi = re.sub(r"\x1b\[[0-9;?]*[ -/]*[@-~]", "", message)
    cleaned = "".join(character for character in without_ansi if character.isprintable() or character.isspace())
    return " ".join(cleaned.split())


def _looks_like_circuitpython_console(message: str) -> bool:
    normalized = message.lower()
    return any(
        token in normalized
        for token in (
            "repl",
            "circuitpython",
            "auto-reload",
            "press any key to enter the repl",
        )
    )


class SerialTransport:
    def __init__(self, port: str, baud: int = DEFAULT_BAUD_RATE) -> None:
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError(PY_SERIAL_MISSING_MESSAGE) from exc

        self._serial = serial.Serial(port=port, baudrate=baud, timeout=0.1)

    def close(self) -> None:
        if self._serial.is_open:
            self._serial.close()

    def read_line(self, timeout: float = 0.1) -> str | None:
        previous_timeout = self._serial.timeout
        self._serial.timeout = timeout
        try:
            raw_line = self._serial.readline()
        finally:
            self._serial.timeout = previous_timeout

        if not raw_line:
            return None
        return raw_line.decode("utf-8", errors="replace").strip()

    def write_command(self, command: dict[str, Any]) -> None:
        line = json.dumps(command) + "\n"
        self._serial.write(line.encode("utf-8"))
        self._serial.flush()


class SimulationTransport:
    SAMPLE_INTERVAL_S = 0.1

    def __init__(self, _port: str, _baud: int) -> None:
        self._pending_lines: queue.Queue[str] = queue.Queue()
        self._state = "idle"
        self._speed_mm_per_min = 0.0
        self._absolute_position_mm = 0.0
        self._zero_reference_mm = 0.0
        self._tare_reference_n = 0.0
        self._last_motion_time = 0.0
        self._last_sample_time = 0.0
        self._run_started_at: float | None = None
        self._closed = False
        self._enqueue_status("idle", "Virtual simulation ready.")

    def close(self) -> None:
        self._closed = True

    def read_line(self, timeout: float = 0.1) -> str | None:
        stop_at = time.monotonic() + timeout
        while not self._closed:
            try:
                return self._pending_lines.get_nowait()
            except queue.Empty:
                pass

            now = time.monotonic()
            self._advance_motion(now)

            if self._state == "running" and now - self._last_sample_time >= self.SAMPLE_INTERVAL_S:
                self._last_sample_time = now
                return json.dumps(self._build_sample_message(now))

            remaining = stop_at - now
            if remaining <= 0:
                return None
            time.sleep(min(0.02, remaining))

        return None

    def write_command(self, command: dict[str, Any]) -> None:
        cmd = str(command.get("cmd", ""))
        now = time.monotonic()
        self._advance_motion(now)

        if cmd == "set_mode":
            mode = str(command.get("mode", ""))
            if mode == "simulation":
                self._state = "idle"
                self._enqueue_status("idle", "Virtual simulation enabled.")
            elif mode == "hardware":
                self._state = "idle"
                self._enqueue_status("idle", "Hardware mode selected.")
            else:
                self._enqueue_error("invalid_mode", "Mode must be 'simulation' or 'hardware'.")
            return

        if self._state == "estop" and cmd in {"tare_force", "zero_displacement", "jog", "start_test"}:
            self._enqueue_error("estop_active", "Disconnect to clear E-Stop.")
            return

        if cmd == "tare_force":
            self._tare_reference_n = self._raw_force_for_displacement(self._current_displacement_mm())
            self._enqueue_status("idle", "Force tared.")
            self._enqueue_sample(now)
            return

        if cmd == "zero_displacement":
            self._zero_reference_mm = self._absolute_position_mm
            self._enqueue_status("idle", "Displacement zeroed.")
            self._enqueue_sample(now)
            return

        if cmd == "jog":
            if self._state == "running":
                self._enqueue_error("busy", "Cannot jog while the test is running.")
                return

            direction = str(command.get("direction", "forward"))
            distance_mm = max(float(command.get("distance_mm", 0.0)), 0.0)
            delta = distance_mm if direction == "forward" else -distance_mm
            self._absolute_position_mm += delta
            self._enqueue_status("idle", f"Jogged {direction} by {distance_mm:.3f} mm.")
            self._enqueue_sample(now)
            return

        if cmd == "start_test":
            self._speed_mm_per_min = max(float(command.get("speed_mm_per_min", 0.0)), 0.0)
            self._state = "running"
            self._run_started_at = now
            self._last_motion_time = now
            self._last_sample_time = 0.0
            self._enqueue_status("running", f"Pull started at {self._speed_mm_per_min:.3f} mm/min.")
            return

        if cmd == "stop":
            if self._state == "running":
                self._state = "idle"
                self._speed_mm_per_min = 0.0
                self._enqueue_sample(now)
                self._enqueue_status("idle", "Test stopped.")
            return

        if cmd == "estop":
            self._state = "estop"
            self._speed_mm_per_min = 0.0
            self._enqueue_sample(now)
            self._enqueue_status("estop", "Emergency stop triggered.")
            return

        self._enqueue_error("unknown_command", f"Unsupported command: {cmd!r}")

    def _advance_motion(self, now: float) -> None:
        if self._last_motion_time == 0.0:
            self._last_motion_time = now
            return

        if self._state == "running":
            delta_s = max(0.0, now - self._last_motion_time)
            self._absolute_position_mm += self._speed_mm_per_min * delta_s / 60.0
        self._last_motion_time = now

    def _current_displacement_mm(self) -> float:
        return self._absolute_position_mm - self._zero_reference_mm

    def _raw_force_for_displacement(self, displacement_mm: float) -> float:
        extension = max(0.0, displacement_mm)
        if extension < 3.0:
            return extension * 15.0
        if extension < 6.0:
            return 45.0 + (extension - 3.0) * 8.0
        return max(0.0, 69.0 - (extension - 6.0) * 10.0)

    def _measured_force_n(self) -> float:
        measured = self._raw_force_for_displacement(self._current_displacement_mm()) - self._tare_reference_n
        return max(0.0, measured)

    def _build_sample_message(self, now: float) -> dict[str, float | str]:
        if self._run_started_at is None:
            timestamp_s = 0.0
        else:
            timestamp_s = max(0.0, now - self._run_started_at)

        return {
            "type": "sample",
            "timestamp_s": round(timestamp_s, 3),
            "force_n": round(self._measured_force_n(), 3),
            "displacement_mm": round(self._current_displacement_mm(), 3),
            "state": self._state,
        }

    def _enqueue_status(self, state: str, message: str) -> None:
        self._pending_lines.put(json.dumps({"type": "status", "state": state, "message": message}))

    def _enqueue_error(self, code: str, message: str) -> None:
        self._pending_lines.put(json.dumps({"type": "error", "code": code, "message": message}))

    def _enqueue_sample(self, now: float) -> None:
        self._pending_lines.put(json.dumps(self._build_sample_message(now)))


class TesterController:
    def __init__(
        self,
        transport_factory: Callable[[str, int], Transport] | None = None,
        simulation_transport_factory: Callable[[str, int], Transport] | None = None,
    ) -> None:
        self._events: queue.Queue[TesterEvent] = queue.Queue()
        self._transport: Transport | None = None
        self._reader_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._specimen: SpecimenDimensions | None = None
        self._transport_factory = transport_factory or SerialTransport
        self._simulation_transport_factory = simulation_transport_factory or SimulationTransport
        self._connected = False
        self._state = "disconnected"
        self._connection_label = ""

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def state(self) -> str:
        return self._state

    @property
    def connection_label(self) -> str:
        return self._connection_label

    def set_specimen_dimensions(
        self,
        area_mm2: float,
        gauge_length_mm: float,
    ) -> SpecimenDimensions:
        specimen = validate_specimen_dimensions(area_mm2, gauge_length_mm)
        with self._lock:
            self._specimen = specimen
        return specimen

    def clear_specimen_dimensions(self) -> None:
        with self._lock:
            self._specimen = None

    def connect(self, port: str, baud: int = DEFAULT_BAUD_RATE, simulate: bool = False) -> None:
        with self._lock:
            if self._connected:
                raise RuntimeError("Controller is already connected.")

            if simulate:
                connection_label = SIMULATION_CONNECTION_LABEL
                transport = self._simulation_transport_factory(connection_label, baud)
            else:
                connection_label = port
                transport = self._transport_factory(port, baud)

            self._transport = transport
            self._stop_event.clear()
            self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._connected = True
            self._state = "connected"
            self._connection_label = connection_label
            self._reader_thread.start()

        self._push_event(TesterEvent.status("connected", f"Connected to {connection_label}."))

    def set_device_mode(self, test_mode: bool) -> None:
        mode = "simulation" if test_mode else "hardware"
        self._send_command({"cmd": "set_mode", "mode": mode})

    def disconnect(self) -> None:
        with self._lock:
            transport = self._transport
            reader_thread = self._reader_thread
            self._transport = None
            self._reader_thread = None
            self._connected = False
            self._state = "disconnected"
            self._connection_label = ""
            self._stop_event.set()

        if transport is not None:
            transport.close()
        if reader_thread is not None and reader_thread.is_alive():
            reader_thread.join(timeout=1.0)

        self._push_event(TesterEvent.status("disconnected", "Disconnected from tester."))

    def tare_force(self) -> None:
        self._send_command({"cmd": "tare_force"})

    def zero_displacement(self) -> None:
        self._send_command({"cmd": "zero_displacement"})

    def jog(self, direction: str, distance_mm: float, speed_mm_per_min: float) -> None:
        direction_value = str(direction)
        if direction_value not in {"forward", "reverse"}:
            raise ValueError("Direction must be 'forward' or 'reverse'.")
        if float(distance_mm) <= 0:
            raise ValueError("Jog distance must be greater than zero.")
        if float(speed_mm_per_min) <= 0:
            raise ValueError("Jog speed must be greater than zero.")

        self._send_command(
            {
                "cmd": "jog",
                "direction": direction_value,
                "distance_mm": float(distance_mm),
                "speed_mm_per_min": float(speed_mm_per_min),
            }
        )

    def start_test(self, speed_mm_per_min: float) -> None:
        if float(speed_mm_per_min) <= 0:
            raise ValueError("Test speed must be greater than zero.")

        self._send_command(
            {
                "cmd": "start_test",
                "speed_mm_per_min": float(speed_mm_per_min),
            }
        )

    def stop(self) -> None:
        self._send_command({"cmd": "stop"})

    def estop(self) -> None:
        self._send_command({"cmd": "estop"})

    def poll_events(self) -> list[TesterEvent]:
        events: list[TesterEvent] = []
        while True:
            try:
                events.append(self._events.get_nowait())
            except queue.Empty:
                return events

    def _send_command(self, command: dict[str, Any]) -> None:
        with self._lock:
            if not self._connected or self._transport is None:
                raise RuntimeError("Controller is not connected.")
            transport = self._transport

        transport.write_command(command)

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            transport = self._transport
            if transport is None:
                return

            try:
                raw_line = transport.read_line(timeout=0.1)
            except Exception as exc:
                self._state = "fault"
                self._push_event(TesterEvent.error("transport_error", str(exc)))
                self._push_event(TesterEvent.status("fault", f"Transport failure: {exc}"))
                return

            if not raw_line:
                continue

            specimen = self._specimen
            event = parse_device_message(raw_line, specimen=specimen, fallback_state=self._state)

            if event.kind == "status" and event.state:
                self._state = event.state
            elif event.kind == "sample" and event.state:
                self._state = event.state
            elif event.kind == "error":
                self._state = "fault"

            self._push_event(event)

            if event.kind == "error":
                self._push_event(TesterEvent.status("fault", event.message))

    def _push_event(self, event: TesterEvent) -> None:
        self._events.put(event)
