from __future__ import annotations

import time
import unittest

from circuitpython.simulation import SimulatedTesterBackend


class SimulationBackendTests(unittest.TestCase):
    def test_home_command_moves_through_homing_to_idle(self) -> None:
        backend = SimulatedTesterBackend(sample_interval_s=0.05)
        now = time.monotonic()

        messages = backend.handle_command({"cmd": "home"}, now)
        self.assertTrue(any(message["type"] == "status" and message["state"] == "homing" for message in messages))

        for step in range(1, 300):
            events = backend.poll(now + step * 0.05)
            if any(event["type"] == "status" and event["message"] == "Homing complete." for event in events):
                self.assertEqual(backend.state, "idle")
                return

        self.fail("Simulation backend never completed homing.")

    def test_start_test_auto_homes_before_running(self) -> None:
        backend = SimulatedTesterBackend(sample_interval_s=0.05)
        now = time.monotonic()

        messages = backend.handle_command({"cmd": "start_test", "speed_mm_per_min": 10.0}, now)
        self.assertTrue(any(message["type"] == "status" and message["state"] == "homing" for message in messages))

        for step in range(1, 300):
            events = backend.poll(now + step * 0.05)
            if any(event["type"] == "sample" and event["state"] == "running" for event in events):
                self.assertEqual(backend.state, "running")
                return

        self.fail("Simulation backend never transitioned to running.")

    def test_stop_cancels_jogging(self) -> None:
        backend = SimulatedTesterBackend(sample_interval_s=0.05)
        now = time.monotonic()

        backend.handle_command(
            {"cmd": "jog", "direction": "forward", "distance_mm": 5.0, "speed_mm_per_min": 60.0},
            now,
        )
        backend.handle_command({"cmd": "stop"}, now + 0.1)

        self.assertEqual(backend.state, "idle")


if __name__ == "__main__":
    unittest.main()
