from __future__ import annotations

import time
import unittest

from host.tensile_tester import (
    MOCK_PORT_NAME,
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


class SpecimenMathTests(unittest.TestCase):
    def test_validate_specimen_dimensions_rejects_non_positive_values(self) -> None:
        with self.assertRaises(ValueError):
            validate_specimen_dimensions(0, 1, 1)
        with self.assertRaises(ValueError):
            validate_specimen_dimensions(1, -1, 1)
        with self.assertRaises(ValueError):
            validate_specimen_dimensions(1, 1, 0)

    def test_compute_derived_values_returns_engineering_stress_and_strain(self) -> None:
        specimen = validate_specimen_dimensions(10.0, 2.0, 25.0)
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
        specimen = validate_specimen_dimensions(10.0, 2.0, 25.0)
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

    def test_parse_invalid_sample_payload_returns_error_event(self) -> None:
        event = parse_device_message('{"type":"sample","timestamp_s":"bad"}')

        self.assertEqual(event.kind, "error")
        self.assertEqual(event.code, "invalid_sample")


class ControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = TesterController()

    def tearDown(self) -> None:
        if self.controller.connected:
            self.controller.disconnect()

    def test_mock_controller_emits_samples_with_derived_values(self) -> None:
        self.controller.set_specimen_dimensions(10.0, 2.0, 25.0)
        self.controller.connect(port=MOCK_PORT_NAME, use_mock=True)
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
        self.controller.connect(port=MOCK_PORT_NAME, use_mock=True)
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
        self.controller.connect(port=MOCK_PORT_NAME, use_mock=True)
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


if __name__ == "__main__":
    unittest.main()
