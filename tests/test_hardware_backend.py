from __future__ import annotations

import sys
import time
import types
import unittest
from unittest.mock import patch

import circuitpython.hardware as hardware_module
from circuitpython.hardware import (
    AS5600Sensor,
    AxisConfig,
    AxisDevices,
    DigitalStepperMotor,
    EncoderTracker,
    DriverConfig,
    HardwareConfig,
    HardwareDevices,
    HardwareTesterBackend,
    HomingConfig,
    LoadCellConfig,
    MechanicalConfig,
    SoftwareUARTTransmitter,
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
    load_cell=None,
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
        load_cell=load_cell if load_cell is not None else FakeLoadCell(rigs),
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

    def test_encoder_tracker_can_invert_reported_direction(self) -> None:
        tracker = EncoderTracker(counts_per_rev=4096, lead_mm_per_rev=8.0, inverted=True)

        tracker.update(0)
        tracker.update(256)

        self.assertEqual(tracker.relative_counts, -256)

    def test_default_hardware_devices_support_positional_only_circuitpython_apis(self) -> None:
        settings = hardware_module.build_default_config()
        board = types.ModuleType("board")
        pin_names = {
            settings.axes[0].step_pin,
            settings.axes[0].dir_pin,
            settings.axes[0].enable_pin,
            settings.axes[0].uart_pin,
            settings.axes[0].home_switch_pin,
            settings.axes[0].encoder_scl_pin,
            settings.axes[0].encoder_sda_pin,
            settings.axes[1].step_pin,
            settings.axes[1].dir_pin,
            settings.axes[1].enable_pin,
            settings.axes[1].uart_pin,
            settings.axes[1].home_switch_pin,
            settings.axes[1].encoder_scl_pin,
            settings.axes[1].encoder_sda_pin,
            settings.load_cell.data_pin,
            settings.load_cell.clock_pin,
        }
        for name in pin_names:
            setattr(board, name, name)
            setattr(board, name.upper(), name)
            setattr(board, name.lower(), name)
            setattr(board, name.replace("gpio", "GP"), name)
            setattr(board, name.replace("GPIO", "GP"), name)

        class Pull:
            UP = object()

        class DigitalInOut:
            def __init__(self, pin, /) -> None:
                self.pin = pin
                self.value = False
                self.pull = None

            def switch_to_output(self, value=False, /) -> None:
                self.value = value

            def switch_to_input(self, pull=None, /) -> None:
                self.pull = pull

        digitalio = types.ModuleType("digitalio")
        digitalio.DigitalInOut = DigitalInOut
        digitalio.Pull = Pull

        class I2C:
            def __init__(self, scl, sda, frequency=100000, /) -> None:
                self.scl = scl
                self.sda = sda
                self.frequency = frequency

            def try_lock(self) -> bool:
                return True

            def unlock(self) -> None:
                return None

            def writeto_then_readfrom(self, address, out_buffer, in_buffer, in_end=None) -> None:
                _ = (address, out_buffer, in_end)
                if len(in_buffer) > 0:
                    in_buffer[0] = 0x20
                if len(in_buffer) > 1:
                    in_buffer[1] = 0x00

        class UART:
            def __init__(self, tx, rx, baudrate=9600, timeout=1, /) -> None:
                self.tx = tx
                self.rx = rx
                self.baudrate = baudrate
                self.timeout = timeout

            def write(self, frame) -> None:
                _ = frame

            def flush(self) -> None:
                return None

        busio = types.ModuleType("busio")
        busio.I2C = I2C
        busio.UART = UART

        with patch.dict(sys.modules, {"board": board, "digitalio": digitalio, "busio": busio}):
            devices = hardware_module.create_default_hardware_devices(settings)

        self.assertEqual(len(devices.axes), 2)
        self.assertEqual(devices.axes[0].motor._step_pin.pin, settings.axes[0].step_pin)
        expected_left_pull = (
            Pull.UP
            if (
                settings.axes[0].home_switch_pull_up
                if settings.axes[0].home_switch_pull_up is not None
                else settings.switches.pull_up
            )
            else None
        )
        self.assertEqual(devices.axes[0].home_switch._pin.pull, expected_left_pull)
        self.assertEqual(devices.axes[0].encoder._i2c.frequency, 400000)
        self.assertIsInstance(devices.axes[0].driver._uart, SoftwareUARTTransmitter)
        self.assertEqual(devices.axes[0].driver._uart.baudrate, 9600)
        self.assertTrue(
            any("Left driver software UART active" in warning for warning in devices.warnings),
            devices.warnings,
        )

    def test_default_hardware_devices_falls_back_to_software_uart_for_non_tx_pin(self) -> None:
        settings = hardware_module.build_default_config()
        board = types.ModuleType("board")
        pin_names = {
            settings.axes[0].step_pin,
            settings.axes[0].dir_pin,
            settings.axes[0].enable_pin,
            settings.axes[0].uart_pin,
            settings.axes[0].home_switch_pin,
            settings.axes[0].encoder_scl_pin,
            settings.axes[0].encoder_sda_pin,
            settings.axes[1].step_pin,
            settings.axes[1].dir_pin,
            settings.axes[1].enable_pin,
            settings.axes[1].uart_pin,
            settings.axes[1].home_switch_pin,
            settings.axes[1].encoder_scl_pin,
            settings.axes[1].encoder_sda_pin,
            settings.load_cell.data_pin,
            settings.load_cell.clock_pin,
        }
        for name in pin_names:
            setattr(board, name, name)
            setattr(board, name.upper(), name)
            setattr(board, name.lower(), name)
            setattr(board, name.replace("gpio", "GP"), name)
            setattr(board, name.replace("GPIO", "GP"), name)

        class Pull:
            UP = object()

        class DigitalInOut:
            def __init__(self, pin, /) -> None:
                self.pin = pin
                self.value = False
                self.pull = None

            def switch_to_output(self, value=False, /) -> None:
                self.value = value

            def switch_to_input(self, pull=None, /) -> None:
                self.pull = pull

        digitalio = types.ModuleType("digitalio")
        digitalio.DigitalInOut = DigitalInOut
        digitalio.Pull = Pull

        class I2C:
            def __init__(self, scl, sda, frequency=100000, /) -> None:
                self.scl = scl
                self.sda = sda
                self.frequency = frequency

            def try_lock(self) -> bool:
                return True

            def unlock(self) -> None:
                return None

            def writeto_then_readfrom(self, address, out_buffer, in_buffer, in_end=None) -> None:
                _ = (address, out_buffer, in_end)
                if len(in_buffer) > 0:
                    in_buffer[0] = 0x20
                if len(in_buffer) > 1:
                    in_buffer[1] = 0x00

        class UART:
            def __init__(self, tx, rx, baudrate=9600, timeout=1, /) -> None:
                if tx == settings.axes[1].uart_pin:
                    raise ValueError("Invalid pins")
                self.tx = tx
                self.rx = rx
                self.baudrate = baudrate
                self.timeout = timeout

            def write(self, frame) -> None:
                _ = frame

        busio = types.ModuleType("busio")
        busio.I2C = I2C
        busio.UART = UART

        with patch.dict(sys.modules, {"board": board, "digitalio": digitalio, "busio": busio}):
            devices = hardware_module.create_default_hardware_devices(settings)

        self.assertIsInstance(devices.axes[1].driver._uart, SoftwareUARTTransmitter)
        self.assertEqual(devices.axes[1].driver._uart.baudrate, 9600)
        self.assertTrue(
            any("software UART active" in warning for warning in devices.warnings),
            devices.warnings,
        )

    def test_default_hardware_devices_falls_back_to_hardware_uart_when_software_uart_unavailable(self) -> None:
        settings = hardware_module.build_default_config()
        board = types.ModuleType("board")
        pin_names = {
            settings.axes[0].step_pin,
            settings.axes[0].dir_pin,
            settings.axes[0].enable_pin,
            settings.axes[0].uart_pin,
            settings.axes[0].home_switch_pin,
            settings.axes[0].encoder_scl_pin,
            settings.axes[0].encoder_sda_pin,
            settings.axes[1].step_pin,
            settings.axes[1].dir_pin,
            settings.axes[1].enable_pin,
            settings.axes[1].uart_pin,
            settings.axes[1].home_switch_pin,
            settings.axes[1].encoder_scl_pin,
            settings.axes[1].encoder_sda_pin,
            settings.load_cell.data_pin,
            settings.load_cell.clock_pin,
        }
        for name in pin_names:
            setattr(board, name, name)
            setattr(board, name.upper(), name)
            setattr(board, name.lower(), name)
            setattr(board, name.replace("gpio", "GP"), name)
            setattr(board, name.replace("GPIO", "GP"), name)

        class Pull:
            UP = object()

        class DigitalInOut:
            def __init__(self, pin, /) -> None:
                self.pin = pin
                self.value = False
                self.pull = None

            def switch_to_output(self, value=False, /) -> None:
                if self.pin == settings.axes[0].uart_pin:
                    raise RuntimeError("software UART pin busy")
                self.value = value

            def switch_to_input(self, pull=None, /) -> None:
                self.pull = pull

        digitalio = types.ModuleType("digitalio")
        digitalio.DigitalInOut = DigitalInOut
        digitalio.Pull = Pull

        class I2C:
            def __init__(self, scl, sda, frequency=100000, /) -> None:
                self.scl = scl
                self.sda = sda
                self.frequency = frequency

            def try_lock(self) -> bool:
                return True

            def unlock(self) -> None:
                return None

            def writeto_then_readfrom(self, address, out_buffer, in_buffer, in_end=None) -> None:
                _ = (address, out_buffer, in_end)
                if len(in_buffer) > 0:
                    in_buffer[0] = 0x20
                if len(in_buffer) > 1:
                    in_buffer[1] = 0x00

        class UART:
            def __init__(self, tx, rx, baudrate=9600, timeout=1, /) -> None:
                self.tx = tx
                self.rx = rx
                self.baudrate = baudrate
                self.timeout = timeout

            def write(self, frame) -> None:
                _ = frame

            def flush(self) -> None:
                return None

        busio = types.ModuleType("busio")
        busio.I2C = I2C
        busio.UART = UART

        with patch.dict(sys.modules, {"board": board, "digitalio": digitalio, "busio": busio}):
            devices = hardware_module.create_default_hardware_devices(settings)

        self.assertEqual(devices.axes[0].driver._uart.tx, settings.axes[0].uart_pin)
        self.assertEqual(devices.axes[0].driver._uart.timeout, 0.02)
        self.assertTrue(
            any("hardware UART active" in warning for warning in devices.warnings),
            devices.warnings,
        )

    def test_build_default_config_uses_per_axis_home_direction_overrides(self) -> None:
        config_module = types.ModuleType("config")
        config_module.SAMPLE_INTERVAL_S = 0.1
        config_module.STEP_PULSE_S = 0.0
        config_module.MOTOR_A_STEP_PIN = "gpio10"
        config_module.MOTOR_A_DIR_PIN = "gpio9"
        config_module.MOTOR_A_ENABLE_PIN = "gpio8"
        config_module.MOTOR_A_UART_PIN = "gpio20"
        config_module.MOTOR_B_STEP_PIN = "gpio16"
        config_module.MOTOR_B_DIR_PIN = "gpio15"
        config_module.MOTOR_B_ENABLE_PIN = "gpio14"
        config_module.MOTOR_B_UART_PIN = "gpio17"
        config_module.LEFT_HOME_SWITCH_PIN = "gpio22"
        config_module.RIGHT_HOME_SWITCH_PIN = "gpio23"
        config_module.LEFT_ENCODER_SCL_PIN = "gpio0"
        config_module.LEFT_ENCODER_SDA_PIN = "gpio1"
        config_module.RIGHT_ENCODER_SCL_PIN = "gpio2"
        config_module.RIGHT_ENCODER_SDA_PIN = "gpio3"
        config_module.MOTOR_A_RUN_CURRENT_A = 1.0
        config_module.MOTOR_A_HOLD_CURRENT_A = 0.75
        config_module.MOTOR_B_RUN_CURRENT_A = 1.0
        config_module.MOTOR_B_HOLD_CURRENT_A = 0.75
        config_module.SCREW_LEAD_MM_PER_REV = 8.0
        config_module.STEPPER_FULL_STEPS_PER_REV = 200
        config_module.STEPPER_MICROSTEPS = 16
        config_module.ENCODER_COUNTS_PER_REV = 4096
        config_module.ENCODER_SLIP_TOLERANCE_MM = 0.25
        config_module.HOME_FAST_SPEED_MM_PER_MIN = 120.0
        config_module.HOME_SLOW_SPEED_MM_PER_MIN = 20.0
        config_module.HOME_BACKOFF_MM = 2.0
        config_module.HOME_TIMEOUT_S = 20.0
        config_module.HOME_SWITCH_MODE = "single"
        config_module.SINGLE_HOME_SWITCH_AXIS = "right"
        config_module.HX711_DATA_PIN = "gpio4"
        config_module.HX711_CLOCK_PIN = "gpio5"
        config_module.HX711_COUNTS_PER_NEWTON = 1000.0
        config_module.HX711_AVERAGE_SAMPLES = 5
        config_module.HX711_TARE_SAMPLES = 8
        config_module.HOME_SWITCH_ACTIVE_LOW = True
        config_module.HOME_SWITCH_PULL_UP = True
        config_module.LEFT_HOME_SWITCH_ACTIVE_LOW = False
        config_module.RIGHT_HOME_SWITCH_ACTIVE_LOW = True
        config_module.LEFT_HOME_SWITCH_PULL_UP = False
        config_module.RIGHT_HOME_SWITCH_PULL_UP = True
        config_module.TMC_UART_BAUDRATE = 115200
        config_module.TMC_INTERPOLATE = True
        config_module.TMC_SENSE_RESISTOR_OHMS = 0.110
        config_module.HOME_DIRECTION = -1
        config_module.LEFT_HOME_DIRECTION = 1
        config_module.RIGHT_HOME_DIRECTION = -1
        config_module.MOTOR_A_DIRECTION_INVERTED = True
        config_module.MOTOR_B_DIRECTION_INVERTED = False
        config_module.LEFT_ENCODER_INVERTED = True
        config_module.RIGHT_ENCODER_INVERTED = False

        with patch.dict(sys.modules, {"config": config_module}):
            settings = hardware_module.build_default_config()

        self.assertEqual(settings.axes[0].home_direction, 1)
        self.assertEqual(settings.axes[1].home_direction, -1)
        self.assertTrue(settings.axes[0].direction_inverted)
        self.assertFalse(settings.axes[1].direction_inverted)
        self.assertTrue(settings.axes[0].encoder_inverted)
        self.assertFalse(settings.axes[1].encoder_inverted)
        self.assertFalse(settings.axes[0].home_switch_active_low)
        self.assertTrue(settings.axes[1].home_switch_active_low)
        self.assertFalse(settings.axes[0].home_switch_pull_up)
        self.assertTrue(settings.axes[1].home_switch_pull_up)
        self.assertEqual(settings.homing.switch_mode, "single")
        self.assertEqual(settings.homing.single_switch_axis, "right")

    def test_digital_stepper_motor_can_invert_direction_output(self) -> None:
        class FakePin:
            def __init__(self) -> None:
                self.value = False

        step_pin = FakePin()
        dir_pin = FakePin()
        enable_pin = FakePin()
        motor = DigitalStepperMotor(
            step_pin,
            dir_pin,
            enable_pin,
            pulse_width_s=0.0,
            direction_inverted=True,
        )

        motor.step(1)
        self.assertFalse(dir_pin.value)

        motor.step(-1)
        self.assertTrue(dir_pin.value)


class HardwareBackendTests(unittest.TestCase):
    def test_idle_poll_emits_samples_for_live_metrics(self) -> None:
        backend, _ = build_backend()
        now = time.monotonic()

        messages = backend.poll(now + 0.05)

        sample = next(message for message in messages if message["type"] == "sample")
        self.assertEqual(sample["state"], "idle")
        self.assertEqual(sample["force_n"], 50.0)

    def test_startup_message_warns_when_load_cell_probe_has_no_sample(self) -> None:
        class SilentLoadCell:
            def tare(self) -> float:
                return 0.0

            def read_newtons(self) -> float:
                return 0.0

            def probe(self, timeout_s: float = 0.0) -> bool:
                _ = timeout_s
                return False

        backend, _ = build_backend(load_cell=SilentLoadCell())

        self.assertIn("Load cell did not produce an HX711 sample", backend.startup_message)

    def test_backend_faults_when_encoder_magnet_is_missing(self) -> None:
        backend, _ = build_backend(left_magnet_detected=False)

        self.assertEqual(backend.state, "fault")
        self.assertIn("magnet", backend.startup_message.lower())

    def test_homing_logs_initial_axis_diagnostics(self) -> None:
        backend, _ = build_backend(left_start_mm=0.0, right_start_mm=5.0)
        now = time.monotonic()

        backend.handle_command({"cmd": "home"}, now)
        messages = backend.poll(now + 0.05)

        detail_message = next(
            message["message"]
            for message in messages
            if message["type"] == "status" and message["message"].startswith("Homing detail:")
        )
        self.assertIn("Left phase=seek_fast switch=pressed", detail_message)
        self.assertIn("Right phase=seek_fast switch=open", detail_message)
        self.assertIn("pin=gpio24", detail_message)
        self.assertIn("raw=low", detail_message)

    def test_homing_faults_when_switch_stays_pressed_after_backoff(self) -> None:
        backend, rigs = build_backend(left_start_mm=0.0, right_start_mm=5.0)
        now = time.monotonic()

        rigs[0].switch.is_pressed = lambda: True

        backend.handle_command({"cmd": "home"}, now)
        for step in range(1, 200):
            backend.poll(now + step * 0.05)
            if backend.state == "fault":
                self.assertEqual(backend._fault_code, "home_switch_stuck")
                self.assertIn("remained pressed after", backend._fault_message)
                self.assertIn("pin='gpio24'", backend._fault_message)
                return

        self.fail("A stuck home switch did not trigger a fault.")

    def test_homing_samples_include_per_axis_diagnostics(self) -> None:
        backend, _ = build_backend(left_start_mm=5.0, right_start_mm=5.0)
        now = time.monotonic()

        backend.handle_command({"cmd": "home"}, now)
        messages = backend.poll(now + 0.05)

        sample = next(message for message in messages if message["type"] == "sample")
        self.assertIn("axes", sample)
        self.assertEqual(len(sample["axes"]), 2)
        self.assertEqual(sample["axes"][0]["axis"], "left")
        self.assertEqual(sample["axes"][1]["axis"], "right")
        self.assertIn(sample["axes"][0]["phase"], {"seek_fast", "backoff", "seek_slow", "done"})
        self.assertIn("last_step_direction", sample["axes"][0])

    def test_single_endstop_homing_can_complete_with_only_right_switch(self) -> None:
        backend, rigs = build_backend(left_start_mm=5.0, right_start_mm=5.0)
        backend._settings.homing.switch_mode = "single"
        backend._settings.homing.single_switch_axis = "right"
        backend._shared_home_axis = backend._resolve_shared_home_axis()
        rigs[0].switch.is_pressed = lambda: False
        now = time.monotonic()

        backend.handle_command({"cmd": "home"}, now)
        initial_messages = backend.poll(now + 0.05)
        self.assertTrue(
            any(
                message["type"] == "status" and "Single-endstop homing active" in message["message"]
                for message in initial_messages
            ),
            initial_messages,
        )

        saw_complete = False
        for step in range(2, 300):
            for message in backend.poll(now + step * 0.05):
                if message["type"] == "status" and message["message"] == "Homing complete.":
                    saw_complete = True
                    break
            if saw_complete:
                break

        self.assertTrue(saw_complete, "Single-endstop homing never completed.")
        self.assertTrue(backend._homed)
        self.assertEqual(backend.state, "idle")

    def test_single_endstop_running_ignores_unused_axis_switch(self) -> None:
        backend, rigs = build_backend(left_start_mm=0.0, right_start_mm=0.0)
        backend._settings.homing.switch_mode = "single"
        backend._settings.homing.single_switch_axis = "right"
        backend._shared_home_axis = backend._resolve_shared_home_axis()
        backend._homed = True
        rigs[0].switch.is_pressed = lambda: True
        rigs[1].switch.is_pressed = lambda: False
        now = time.monotonic()

        messages = backend.handle_command({"cmd": "start_test", "speed_mm_per_min": 60.0}, now)
        self.assertTrue(any(message["type"] == "status" and message["state"] == "running" for message in messages))

        backend.poll(now + 0.1)
        self.assertEqual(backend.state, "running")

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
