from __future__ import annotations

import time
import unittest

from circuitpython.hardware import (
    AS5600Sensor,
    AxisConfig,
    AxisDevices,
    DriverConfig,
    HardwareConfig,
    HardwareDevices,
    HardwareTesterBackend,
    HomingConfig,
    LoadCellConfig,
    MechanicalConfig,
    SwitchConfig,
    calculate_steps_per_mm,
    counts_to_newtons,
    unwrap_encoder_step,
)


class FakeDriver:
    def __init__(self) -> None:
        self.configured = False

    def configure(self) -> None:
        self.configured = True


class FakeAxisRig:
    def __init__(
        self,
        *,
        steps_per_mm: float,
        lead_mm_per_rev: float,
        encoder_counts_per_rev: int,
        start_mm: float,
        encoder_scale: float = 1.0,
        magnet_detected: bool = True,
    ) -> None:
        self.steps_per_mm = steps_per_mm
        self.lead_mm_per_rev = lead_mm_per_rev
        self.encoder_counts_per_rev = encoder_counts_per_rev
        self.position_steps = int(round(start_mm * steps_per_mm))
        self.encoder_scale = encoder_scale
        self.magnet_ok = magnet_detected
        self.enabled = True
        self.driver = FakeDriver()
        self.motor = self.FakeMotor(self)
        self.encoder = self.FakeEncoder(self)
        self.switch = self.FakeSwitch(self)

    class FakeMotor:
        def __init__(self, rig: "FakeAxisRig") -> None:
            self.rig = rig

        def set_enabled(self, enabled: bool) -> None:
            self.rig.enabled = enabled

        def step(self, direction: int) -> None:
            self.rig.position_steps += 1 if direction >= 0 else -1

    class FakeEncoder:
        def __init__(self, rig: "FakeAxisRig") -> None:
            self.rig = rig

        def magnet_detected(self) -> bool:
            return self.rig.magnet_ok

        def read_raw_angle(self) -> int:
            scaled_mm = self.rig.position_mm * self.rig.encoder_scale
            revolutions = scaled_mm / self.rig.lead_mm_per_rev
            counts = int(round(revolutions * self.rig.encoder_counts_per_rev))
            return counts % self.rig.encoder_counts_per_rev

    class FakeSwitch:
        def __init__(self, rig: "FakeAxisRig") -> None:
            self.rig = rig

        def is_pressed(self) -> bool:
            return self.rig.position_mm <= 0.0

    @property
    def position_mm(self) -> float:
        return self.position_steps / self.steps_per_mm


class FakeLoadCell:
    def __init__(self, rigs: list[FakeAxisRig]) -> None:
        self._rigs = rigs
        self._tare_offset = 0.0

    def tare(self) -> float:
        self._tare_offset = self._raw_force()
        return self._tare_offset

    def read_newtons(self) -> float:
        return max(0.0, self._raw_force() - self._tare_offset)

    def _raw_force(self) -> float:
        average_mm = sum(rig.position_mm for rig in self._rigs) / len(self._rigs)
        return max(0.0, average_mm * 10.0)


class LockedI2CBus:
    def try_lock(self) -> bool:
        return False

    def unlock(self) -> None:
        raise AssertionError("unlock should not be called when the lock is never acquired")


def build_settings() -> HardwareConfig:
    return HardwareConfig(
        sample_interval_s=0.05,
        step_pulse_s=0.0,
        axes=(
            AxisConfig("left", "gpio10", "gpio9", "gpio8", "gpio20", "gpio24", "gpio0", "gpio1", 0.4, 0.1),
            AxisConfig("right", "gpio16", "gpio15", "gpio14", "gpio17", "gpio25", "gpio2", "gpio3", 0.55, 0.1),
        ),
        mechanics=MechanicalConfig(
            lead_mm_per_rev=8.0,
            full_steps_per_rev=200,
            microsteps=16,
            encoder_counts_per_rev=4096,
            encoder_slip_tolerance_mm=0.25,
        ),
        homing=HomingConfig(
            fast_speed_mm_per_min=1200.0,
            slow_speed_mm_per_min=600.0,
            backoff_mm=0.5,
            timeout_s=10.0,
        ),
        load_cell=LoadCellConfig("gpio4", "gpio5", 1000.0, 4, 4),
        switches=SwitchConfig(active_low=True, pull_up=True),
        driver=DriverConfig(baudrate=115200, interpolate=True, sense_resistor_ohms=0.110),
    )


def build_backend(
    *,
    left_start_mm: float = 5.0,
    right_start_mm: float = 5.0,
    left_encoder_scale: float = 1.0,
    right_encoder_scale: float = 1.0,
    left_magnet_detected: bool = True,
    right_magnet_detected: bool = True,
) -> tuple[HardwareTesterBackend, list[FakeAxisRig]]:
    settings = build_settings()
    steps_per_mm = calculate_steps_per_mm(
        settings.mechanics.full_steps_per_rev,
        settings.mechanics.microsteps,
        settings.mechanics.lead_mm_per_rev,
    )
    rigs = [
        FakeAxisRig(
            steps_per_mm=steps_per_mm,
            lead_mm_per_rev=settings.mechanics.lead_mm_per_rev,
            encoder_counts_per_rev=settings.mechanics.encoder_counts_per_rev,
            start_mm=left_start_mm,
            encoder_scale=left_encoder_scale,
            magnet_detected=left_magnet_detected,
        ),
        FakeAxisRig(
            steps_per_mm=steps_per_mm,
            lead_mm_per_rev=settings.mechanics.lead_mm_per_rev,
            encoder_counts_per_rev=settings.mechanics.encoder_counts_per_rev,
            start_mm=right_start_mm,
            encoder_scale=right_encoder_scale,
            magnet_detected=right_magnet_detected,
        ),
    ]
    devices = HardwareDevices(
        axes=(
            AxisDevices(settings.axes[0], rigs[0].motor, rigs[0].encoder, rigs[0].switch, rigs[0].driver),
            AxisDevices(settings.axes[1], rigs[1].motor, rigs[1].encoder, rigs[1].switch, rigs[1].driver),
        ),
        load_cell=FakeLoadCell(rigs),
    )
    return HardwareTesterBackend(settings=settings, devices=devices), rigs


class HardwareMathTests(unittest.TestCase):
    def test_math_helpers_cover_steps_wrap_and_force_conversion(self) -> None:
        self.assertEqual(calculate_steps_per_mm(200, 16, 8.0), 400.0)
        self.assertEqual(unwrap_encoder_step(4090, 4), 10)
        self.assertAlmostEqual(counts_to_newtons(1200.0, 200.0, 1000.0), 1.0)

    def test_as5600_sensor_times_out_when_i2c_lock_never_arrives(self) -> None:
        sensor = AS5600Sensor(LockedI2CBus(), lock_timeout_s=0.01, lock_poll_s=0.0)

        with self.assertRaisesRegex(RuntimeError, "Timed out waiting for the AS5600 I2C bus lock"):
            sensor.magnet_detected()


class HardwareBackendTests(unittest.TestCase):
    def test_backend_faults_when_encoder_magnet_is_missing(self) -> None:
        backend, _ = build_backend(left_magnet_detected=False)

        self.assertEqual(backend.state, "fault")
        self.assertIn("magnet", backend.startup_message.lower())

    def test_home_completes_and_zeroes_displacement(self) -> None:
        backend, _ = build_backend(left_start_mm=5.0, right_start_mm=6.0)
        now = time.monotonic()

        messages = backend.handle_command({"cmd": "home"}, now)
        self.assertTrue(any(message["type"] == "status" and message["state"] == "homing" for message in messages))

        samples = []
        for step in range(1, 300):
            for message in backend.poll(now + step * 0.05):
                if message["type"] == "sample":
                    samples.append(message)
                if message["type"] == "status" and message["message"] == "Homing complete.":
                    self.assertEqual(message["state"], "idle")
                    self.assertAlmostEqual(samples[-1]["displacement_mm"], 0.0, places=2)
                    return

        self.fail("Homing never completed.")

    def test_start_test_auto_homes_then_enters_running_state(self) -> None:
        backend, _ = build_backend(left_start_mm=4.0, right_start_mm=4.0)
        now = time.monotonic()

        messages = backend.handle_command({"cmd": "start_test", "speed_mm_per_min": 12.0}, now)
        self.assertTrue(any(message["type"] == "status" and message["state"] == "homing" for message in messages))

        for step in range(1, 300):
            for message in backend.poll(now + step * 0.05):
                if message["type"] == "sample" and message["state"] == "running":
                    self.assertLess(abs(message["displacement_mm"]), 0.1)
                    return

        self.fail("Auto-home never transitioned into running.")

    def test_running_faults_when_encoder_slips(self) -> None:
        backend, rigs = build_backend(left_start_mm=0.0, right_start_mm=0.0, right_encoder_scale=0.0)
        now = time.monotonic()

        backend.handle_command({"cmd": "home"}, now)
        for step in range(1, 200):
            for message in backend.poll(now + step * 0.05):
                if message["type"] == "status" and message["message"] == "Homing complete.":
                    break
            else:
                continue
            break

        rigs[1].encoder_scale = 0.0
        messages = backend.handle_command({"cmd": "start_test", "speed_mm_per_min": 60.0}, now + 20.0)
        self.assertTrue(any(message["type"] == "status" and message["state"] == "running" for message in messages))

        for step in range(1, 50):
            backend.poll(now + 20.0 + step * 0.05)
            if backend.state == "fault":
                self.assertIn("mismatch", backend._fault_message.lower())
                return

        self.fail("Encoder slip did not trigger a fault.")


if __name__ == "__main__":
    unittest.main()
