from __future__ import annotations

import json
import queue
import time
import unittest

from host.tensile_tester import (
    TesterController,
    compute_derived_values,
    parse_device_message,
    validate_specimen_dimensions,
)


def wait_for_events(
    controller: TesterController,
    predicate,
    timeout_s: float = 2.0,
) -> list:
    deadline = time.monotonic() + timeout_s
    events = []
    while time.monotonic() < deadline:
        events.extend(controller.poll_events())
        if predicate(events):
            return events
        time.sleep(0.05)
    return events


class FakeSerialTransport:
    SAMPLE_INTERVAL_S = 0.1

    def __init__(self, _port: str, _baud: int) -> None:
        self._pending_lines: queue.Queue[str] = queue.Queue()
        self._state = "idle"
        self._speed_mm_per_min = 0.0
        self._absolute_position_mm = 0.0
        self._zero_reference_mm = 0.0
        self._tare_reference_n = 0.0
        self._last_motion_time = time.monotonic()
        self._last_sample_time = 0.0
        self._run_started_at: float | None = None
        self._closed = False
        self._enqueue_status("idle", "Fake serial transport connected.")

    def close(self) -> None:
        self._closed = True

    def read_line(self, timeout: float = 0.1) -> str | None:
        deadline = time.monotonic() + timeout

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

            remaining = deadline - now
            if remaining <= 0:
                return None

            time.sleep(min(0.02, remaining))

        return None

    def write_command(self, command: dict[str, float | str]) -> None:
        cmd = str(command.get("cmd", ""))
        now = time.monotonic()
        self._advance_motion(now)

        if cmd in {"set_mode", "set mode"}:
            mode = str(command.get("mode", ""))
            if mode == "simulation":
                self._state = "idle"
                self._enqueue_status("idle", "Virtual simulation enabled.")
            elif mode == "hardware":
                self._state = "idle"
                self._enqueue_status("idle", "Hardware mode enabled.")
            else:
                self._enqueue_error("invalid_mode", "Mode must be 'simulation' or 'hardware'.")
            return

        if self._state == "estop" and cmd in {"tare_force", "zero_displacement", "jog", "start_test"}:
            self._enqueue_error("estop_active", "Reconnect to clear E-Stop.")
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
            self._enqueue_status("idle", "Jog complete.")
            self._enqueue_sample(now)
            return

        if cmd == "start_test":
            self._speed_mm_per_min = max(float(command.get("speed_mm_per_min", 0.0)), 0.0)
            self._state = "running"
            self._run_started_at = now
            self._last_motion_time = now
            self._last_sample_time = 0.0
            self._enqueue_status("running", "Pull started.")
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


class SpecimenMathTests(unittest.TestCase):
    def test_validate_specimen_dimensions_rejects_non_positive_values(self) -> None:
        with self.assertRaises(ValueError):
            validate_specimen_dimensions(0, 1)
        with self.assertRaises(ValueError):
            validate_specimen_dimensions(10, 0)

    def test_compute_derived_values_returns_engineering_stress_and_strain(self) -> None:
        specimen = validate_specimen_dimensions(20.0, 25.0)
        stress_mpa, strain_percent = compute_derived_values(100.0, 5.0, specimen)

        self.assertAlmostEqual(stress_mpa, 5.0)
        self.assertAlmostEqual(strain_percent, 20.0)


class DeviceParsingTests(unittest.TestCase):
    def test_parse_status_message(self) -> None:
        event = parse_device_message('{"type":"status","state":"idle","message":"Ready"}')

        self.assertEqual(event.kind, "status")
        self.assertEqual(event.state, "idle")
        self.assertEqual(event.message, "Ready")

    def test_parse_sample_message_computes_derived_values(self) -> None:
        specimen = validate_specimen_dimensions(20.0, 25.0)
        event = parse_device_message(
            '{"type":"sample","timestamp_s":1.2,"force_n":100.0,"displacement_mm":5.0,"state":"running"}',
            specimen=specimen,
        )

        self.assertEqual(event.kind, "sample")
        self.assertAlmostEqual(event.stress_mpa or 0.0, 5.0)
        self.assertAlmostEqual(event.strain_percent or 0.0, 20.0)
        self.assertEqual(event.state, "running")

    def test_parse_invalid_json_returns_error_event(self) -> None:
        event = parse_device_message("{not json")

        self.assertEqual(event.kind, "error")
        self.assertEqual(event.code, "invalid_json")

    def test_parse_plain_text_device_output_returns_status_event(self) -> None:
        event = parse_device_message(
            "Auto-reload is on. Simply save files over USB to run them.",
            fallback_state="connected",
        )

        self.assertEqual(event.kind, "status")
        self.assertEqual(event.state, "connected")
        self.assertIn("boot.py", event.message)

    def test_parse_repl_output_returns_setup_hint(self) -> None:
        event = parse_device_message("\x1b]0;@REPL | 10.1.4\x07", fallback_state="connected")

        self.assertEqual(event.kind, "status")
        self.assertEqual(event.state, "connected")
        self.assertIn("CircuitPython console detected", event.message)

    def test_parse_invalid_sample_payload_returns_error_event(self) -> None:
        event = parse_device_message('{"type":"sample","timestamp_s":"bad"}')

        self.assertEqual(event.kind, "error")
        self.assertEqual(event.code, "invalid_sample")


class ControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = TesterController(transport_factory=FakeSerialTransport)

    def tearDown(self) -> None:
        if self.controller.connected:
            self.controller.disconnect()

    def test_mock_controller_emits_samples_with_derived_values(self) -> None:
        self.controller.set_specimen_dimensions(20.0, 25.0)
        self.controller.connect(port="TEST-COM")
        wait_for_events(
            self.controller,
            lambda events: any(event.kind == "status" and event.state == "idle" for event in events),
        )

        self.controller.start_test(12.0)
        events = wait_for_events(
            self.controller,
            lambda events: any(event.kind == "sample" for event in events),
        )

        sample_event = next(event for event in events if event.kind == "sample")
        self.assertIsNotNone(sample_event.stress_mpa)
        self.assertIsNotNone(sample_event.strain_percent)
        self.assertEqual(sample_event.state, "running")

    def test_disconnect_emits_disconnected_status(self) -> None:
        self.controller.connect(port="TEST-COM")
        wait_for_events(
            self.controller,
            lambda events: any(event.kind == "status" and event.state == "idle" for event in events),
        )

        self.controller.disconnect()
        events = self.controller.poll_events()

        self.assertTrue(
            any(event.kind == "status" and event.state == "disconnected" for event in events)
        )

    def test_device_error_moves_controller_to_fault_state(self) -> None:
        self.controller.connect(port="TEST-COM")
        wait_for_events(
            self.controller,
            lambda events: any(event.kind == "status" and event.state == "idle" for event in events),
        )

        self.controller.estop()
        wait_for_events(
            self.controller,
            lambda events: any(event.kind == "status" and event.state == "estop" for event in events),
        )

        self.controller.start_test(5.0)
        events = wait_for_events(
            self.controller,
            lambda events: any(event.kind == "error" and event.code == "estop_active" for event in events),
        )

        self.assertTrue(
            any(event.kind == "status" and event.state == "fault" for event in events)
        )

    def test_set_device_mode_emits_status(self) -> None:
        self.controller.connect(port="TEST-COM")
        wait_for_events(
            self.controller,
            lambda events: any(event.kind == "status" and event.state == "idle" for event in events),
        )

        self.controller.set_device_mode(True)
        events = wait_for_events(
            self.controller,
            lambda events: any(
                event.kind == "status" and event.message == "Virtual simulation enabled."
                for event in events
            ),
        )

        self.assertTrue(
            any(event.kind == "status" and event.message == "Virtual simulation enabled." for event in events)
        )


if __name__ == "__main__":
    unittest.main()
