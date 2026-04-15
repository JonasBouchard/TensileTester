from __future__ import annotations

from collections import deque
import math
import time

try:
    from typing import Protocol
except (ImportError, AttributeError):
    class Protocol:
        pass


AS5600_RAW_ANGLE_REGISTER = 0x0C
AS5600_STATUS_REGISTER = 0x0B
AS5600_STATUS_MAGNET_DETECTED = 0x20
AS5600_COUNTS_PER_REV = 4096

TMC2209_SYNC = 0x05
TMC2209_GCONF = 0x00
TMC2209_IHOLD_IRUN = 0x10
TMC2209_CHOPCONF = 0x6C

ACTIVE_STATES = {"homing", "jogging", "running"}


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def format_axis_name(name) -> str:
    text = str(name)
    if not text:
        return "Axis"
    return text[0:1].upper() + text[1:]


def switch_pin_to_output(pin, initial_value: bool) -> None:
    try:
        pin.switch_to_output(initial_value)
    except TypeError:
        pin.switch_to_output(value=initial_value)


def switch_pin_to_input(pin, pull) -> None:
    try:
        pin.switch_to_input(pull)
    except TypeError:
        pin.switch_to_input(pull=pull)


def create_i2c_bus(busio_module, scl, sda, frequency: int):
    try:
        return busio_module.I2C(scl, sda, frequency)
    except TypeError:
        return busio_module.I2C(scl, sda, frequency=frequency)


def create_uart_bus(busio_module, tx, rx, baudrate: int, timeout: float):
    try:
        return busio_module.UART(tx, rx, baudrate, timeout)
    except TypeError:
        return busio_module.UART(tx=tx, rx=rx, baudrate=baudrate, timeout=timeout)


def delay_us_compatible(delay_us: int) -> None:
    delay = max(0, int(delay_us))
    if delay <= 0:
        return
    try:
        import microcontroller

        microcontroller.delay_us(delay)
        return
    except ImportError:
        pass
    time.sleep(delay / 1_000_000.0)


def i2c_write_then_read(i2c, address: int, out_buffer, in_buffer, in_end: int | None = None) -> None:
    if in_end is None:
        i2c.writeto_then_readfrom(address, out_buffer, in_buffer)
        return
    try:
        i2c.writeto_then_readfrom(address, out_buffer, in_buffer, in_end)
    except TypeError:
        i2c.writeto_then_readfrom(address, out_buffer, in_buffer, in_end=in_end)


def calculate_steps_per_mm(
    full_steps_per_rev: int,
    microsteps: int,
    lead_mm_per_rev: float,
) -> float:
    if full_steps_per_rev <= 0:
        raise ValueError("full_steps_per_rev must be greater than zero.")
    if microsteps <= 0:
        raise ValueError("microsteps must be greater than zero.")
    if lead_mm_per_rev <= 0:
        raise ValueError("lead_mm_per_rev must be greater than zero.")
    return full_steps_per_rev * microsteps / lead_mm_per_rev


def unwrap_encoder_step(
    previous_raw: int,
    new_raw: int,
    counts_per_rev: int = AS5600_COUNTS_PER_REV,
) -> int:
    delta = int(new_raw) - int(previous_raw)
    half_turn = counts_per_rev // 2
    if delta > half_turn:
        delta -= counts_per_rev
    elif delta < -half_turn:
        delta += counts_per_rev
    return delta


def calculate_encoder_mm(
    counts: int,
    lead_mm_per_rev: float,
    counts_per_rev: int = AS5600_COUNTS_PER_REV,
) -> float:
    return counts / float(counts_per_rev) * lead_mm_per_rev


def counts_to_newtons(
    raw_counts: float,
    tare_counts: float,
    counts_per_newton: float,
) -> float:
    if counts_per_newton <= 0:
        raise ValueError("counts_per_newton must be greater than zero.")
    return (float(raw_counts) - float(tare_counts)) / counts_per_newton


def microsteps_to_mres(microsteps: int) -> int:
    mapping = {
        256: 0,
        128: 1,
        64: 2,
        32: 3,
        16: 4,
        8: 5,
        4: 6,
        2: 7,
        1: 8,
    }
    if microsteps not in mapping:
        raise ValueError("Unsupported microstep value: %r" % microsteps)
    return mapping[microsteps]


def tmc2209_current_to_cs(current_a: float, sense_resistor_ohms: float) -> int:
    if current_a <= 0.0:
        return 0
    raw_scale = 32.0 * current_a * math.sqrt(2.0) * (sense_resistor_ohms + 0.02) / 0.325 - 1.0
    return int(clamp(round(raw_scale), 0, 31))


def tmc2209_crc(frame: bytes) -> int:
    crc = 0
    for byte in frame:
        current = byte
        for _ in range(8):
            if (crc >> 7) ^ (current & 0x01):
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
            current >>= 1
    return crc


class StepperMotorInterface(Protocol):
    def set_enabled(self, enabled: bool) -> None:
        ...

    def step(self, direction: int) -> None:
        ...


class LimitSwitchInterface(Protocol):
    def is_pressed(self) -> bool:
        ...


class EncoderInterface(Protocol):
    def magnet_detected(self) -> bool:
        ...

    def read_raw_angle(self) -> int:
        ...


class LoadCellInterface(Protocol):
    def tare(self) -> float:
        ...

    def read_newtons(self) -> float:
        ...


class DriverInterface(Protocol):
    def configure(self) -> None:
        ...


class AxisConfig:
    def __init__(
        self,
        name: str,
        step_pin: str,
        dir_pin: str,
        enable_pin: str,
        uart_pin: str,
        home_switch_pin: str,
        encoder_scl_pin: str,
        encoder_sda_pin: str,
        run_current_a: float,
        hold_current_a: float,
        home_direction: int = -1,
        direction_inverted: bool = False,
        encoder_inverted: bool = False,
        home_switch_active_low: bool | None = None,
        home_switch_pull_up: bool | None = None,
    ) -> None:
        self.name = name
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.enable_pin = enable_pin
        self.uart_pin = uart_pin
        self.home_switch_pin = home_switch_pin
        self.encoder_scl_pin = encoder_scl_pin
        self.encoder_sda_pin = encoder_sda_pin
        self.run_current_a = run_current_a
        self.hold_current_a = hold_current_a
        self.home_direction = home_direction
        self.direction_inverted = direction_inverted
        self.encoder_inverted = encoder_inverted
        self.home_switch_active_low = home_switch_active_low
        self.home_switch_pull_up = home_switch_pull_up


class MechanicalConfig:
    def __init__(
        self,
        lead_mm_per_rev: float,
        full_steps_per_rev: int,
        microsteps: int,
        encoder_counts_per_rev: int,
        encoder_slip_tolerance_mm: float,
    ) -> None:
        self.lead_mm_per_rev = lead_mm_per_rev
        self.full_steps_per_rev = full_steps_per_rev
        self.microsteps = microsteps
        self.encoder_counts_per_rev = encoder_counts_per_rev
        self.encoder_slip_tolerance_mm = encoder_slip_tolerance_mm


class HomingConfig:
    def __init__(
        self,
        fast_speed_mm_per_min: float,
        slow_speed_mm_per_min: float,
        backoff_mm: float,
        timeout_s: float,
        switch_mode: str = "independent",
        single_switch_axis: str = "right",
    ) -> None:
        self.fast_speed_mm_per_min = fast_speed_mm_per_min
        self.slow_speed_mm_per_min = slow_speed_mm_per_min
        self.backoff_mm = backoff_mm
        self.timeout_s = timeout_s
        self.switch_mode = str(switch_mode).strip().lower() or "independent"
        self.single_switch_axis = str(single_switch_axis).strip().lower() or "right"


class LoadCellConfig:
    def __init__(
        self,
        data_pin: str,
        clock_pin: str,
        counts_per_newton: float,
        average_samples: int,
        tare_samples: int,
    ) -> None:
        self.data_pin = data_pin
        self.clock_pin = clock_pin
        self.counts_per_newton = counts_per_newton
        self.average_samples = average_samples
        self.tare_samples = tare_samples


class SwitchConfig:
    def __init__(self, active_low: bool, pull_up: bool) -> None:
        self.active_low = active_low
        self.pull_up = pull_up


class DriverConfig:
    def __init__(self, baudrate: int, interpolate: bool, sense_resistor_ohms: float) -> None:
        self.baudrate = baudrate
        self.interpolate = interpolate
        self.sense_resistor_ohms = sense_resistor_ohms


class HardwareConfig:
    def __init__(
        self,
        sample_interval_s: float,
        step_pulse_s: float,
        axes: tuple[AxisConfig, AxisConfig],
        mechanics: MechanicalConfig,
        homing: HomingConfig,
        load_cell: LoadCellConfig,
        switches: SwitchConfig,
        driver: DriverConfig,
    ) -> None:
        self.sample_interval_s = sample_interval_s
        self.step_pulse_s = step_pulse_s
        self.axes = axes
        self.mechanics = mechanics
        self.homing = homing
        self.load_cell = load_cell
        self.switches = switches
        self.driver = driver


class AxisDevices:
    def __init__(
        self,
        config: AxisConfig,
        motor: StepperMotorInterface,
        encoder: EncoderInterface,
        home_switch: LimitSwitchInterface,
        driver: DriverInterface | None = None,
    ) -> None:
        self.config = config
        self.motor = motor
        self.encoder = encoder
        self.home_switch = home_switch
        self.driver = driver


class HardwareDevices:
    def __init__(
        self,
        axes: tuple[AxisDevices, AxisDevices],
        load_cell: LoadCellInterface,
        warnings: tuple[str, ...] = (),
    ) -> None:
        self.axes = axes
        self.load_cell = load_cell
        self.warnings = warnings


class EncoderTracker:
    def __init__(
        self,
        counts_per_rev: int,
        lead_mm_per_rev: float,
        *,
        inverted: bool = False,
    ) -> None:
        self.counts_per_rev = counts_per_rev
        self.lead_mm_per_rev = lead_mm_per_rev
        self.inverted = bool(inverted)
        self._last_raw: int | None = None
        self._absolute_counts = 0
        self._reference_counts = 0

    def update(self, raw_angle: int) -> int:
        raw = int(raw_angle) % self.counts_per_rev
        if self._last_raw is None:
            self._last_raw = raw
            return self._absolute_counts

        delta = unwrap_encoder_step(
            previous_raw=self._last_raw,
            new_raw=raw,
            counts_per_rev=self.counts_per_rev,
        )
        if self.inverted:
            delta = -delta
        self._absolute_counts += delta
        self._last_raw = raw
        return self._absolute_counts

    def zero_here(self) -> None:
        self._reference_counts = self._absolute_counts

    @property
    def relative_counts(self) -> int:
        return self._absolute_counts - self._reference_counts

    @property
    def relative_mm(self) -> float:
        return calculate_encoder_mm(
            counts=self.relative_counts,
            lead_mm_per_rev=self.lead_mm_per_rev,
            counts_per_rev=self.counts_per_rev,
        )


class NullDriver:
    def configure(self) -> None:
        return


class NullLoadCell:
    def tare(self) -> float:
        return 0.0

    def read_newtons(self) -> float:
        return 0.0

    def probe(self, timeout_s: float = 0.0) -> bool:
        _ = timeout_s
        return False


class DigitalStepperMotor:
    def __init__(
        self,
        step_pin,
        dir_pin,
        enable_pin,
        *,
        pulse_width_s: float,
        enable_active_low: bool = True,
        direction_inverted: bool = False,
    ) -> None:
        self._step_pin = step_pin
        self._dir_pin = dir_pin
        self._enable_pin = enable_pin
        self._pulse_width_s = max(0.0, pulse_width_s)
        self._enable_active_low = enable_active_low
        self._direction_inverted = direction_inverted
        self.set_enabled(True)

    def set_enabled(self, enabled: bool) -> None:
        if self._enable_pin is None:
            return
        self._enable_pin.value = (not enabled) if self._enable_active_low else bool(enabled)

    def step(self, direction: int) -> None:
        output_direction = bool(direction > 0)
        if self._direction_inverted:
            output_direction = not output_direction
        self._dir_pin.value = output_direction
        self._step_pin.value = True
        if self._pulse_width_s > 0.0:
            time.sleep(self._pulse_width_s)
        self._step_pin.value = False


class StepCounter:
    def __init__(self) -> None:
        self.position_steps = 0


class CoupledDigitalStepperMotor(DigitalStepperMotor):
    def __init__(self, *args, step_counter: StepCounter | None = None, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._step_counter = step_counter

    def step(self, direction: int) -> None:
        super().step(direction)
        if self._step_counter is not None:
            self._step_counter.position_steps += 1 if direction >= 0 else -1


class DigitalLimitSwitch:
    def __init__(self, pin, *, active_low: bool, pin_name: str = "", pull_up: bool | None = None) -> None:
        self._pin = pin
        self._active_low = active_low
        self._pin_name = str(pin_name)
        self._pull_up = pull_up

    def is_pressed(self) -> bool:
        raw_value = bool(self._pin.value)
        return not raw_value if self._active_low else raw_value

    def raw_value(self) -> bool:
        return bool(self._pin.value)

    @property
    def active_low(self) -> bool:
        return self._active_low

    @property
    def pin_name(self) -> str:
        return self._pin_name

    @property
    def pull_up(self) -> bool | None:
        return self._pull_up


class AS5600Sensor:
    def __init__(
        self,
        i2c,
        address: int = 0x36,
        *,
        lock_timeout_s: float = 0.5,
        lock_poll_s: float = 0.001,
    ) -> None:
        self._i2c = i2c
        self._address = address
        self._register = bytearray(1)
        self._buffer = bytearray(2)
        self._lock_timeout_s = max(0.0, float(lock_timeout_s))
        self._lock_poll_s = max(0.0, float(lock_poll_s))

    def magnet_detected(self) -> bool:
        status = self._read_register_8(AS5600_STATUS_REGISTER)
        return bool(status & AS5600_STATUS_MAGNET_DETECTED)

    def read_raw_angle(self) -> int:
        self._register[0] = AS5600_RAW_ANGLE_REGISTER
        self._lock()
        try:
            try:
                i2c_write_then_read(self._i2c, self._address, self._register, self._buffer)
            except OSError as exc:
                raise RuntimeError(
                    "AS5600 angle read failed on I2C address 0x%02X." % self._address
                ) from exc
        finally:
            self._unlock()
        return ((self._buffer[0] << 8) | self._buffer[1]) & 0x0FFF

    def _read_register_8(self, register: int) -> int:
        self._register[0] = register & 0xFF
        self._lock()
        try:
            try:
                i2c_write_then_read(self._i2c, self._address, self._register, self._buffer, 1)
            except OSError as exc:
                raise RuntimeError(
                    "AS5600 register 0x%02X read failed on I2C address 0x%02X."
                    % (register & 0xFF, self._address)
                ) from exc
        finally:
            self._unlock()
        return self._buffer[0]

    def _lock(self) -> None:
        deadline = time.monotonic() + self._lock_timeout_s
        while not self._i2c.try_lock():
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "Timed out waiting for the AS5600 I2C bus lock on address 0x%02X."
                    % self._address
                )
            if self._lock_poll_s > 0.0:
                time.sleep(self._lock_poll_s)

    def _unlock(self) -> None:
        self._i2c.unlock()


class StepTrackingEncoder:
    def __init__(
        self,
        step_counter: StepCounter,
        *,
        steps_per_mm: float,
        lead_mm_per_rev: float,
        counts_per_rev: int,
    ) -> None:
        self._step_counter = step_counter
        self._steps_per_mm = steps_per_mm
        self._lead_mm_per_rev = lead_mm_per_rev
        self._counts_per_rev = counts_per_rev

    def magnet_detected(self) -> bool:
        return True

    def read_raw_angle(self) -> int:
        position_mm = self._step_counter.position_steps / self._steps_per_mm
        revolutions = position_mm / self._lead_mm_per_rev
        counts = int(round(revolutions * self._counts_per_rev))
        return counts % self._counts_per_rev


class HX711ADC:
    def __init__(self, data_pin, clock_pin) -> None:
        self._data_pin = data_pin
        self._clock_pin = clock_pin
        self._clock_pin.value = False

    def is_ready(self) -> bool:
        return not bool(self._data_pin.value)

    def read_raw(self, timeout_s: float = 0.0) -> int | None:
        deadline = time.monotonic() + max(0.0, timeout_s)
        while not self.is_ready():
            if timeout_s <= 0.0 or time.monotonic() >= deadline:
                return None

        raw = 0
        for _ in range(24):
            self._clock_pin.value = True
            raw = (raw << 1) | (1 if self._data_pin.value else 0)
            self._clock_pin.value = False

        self._clock_pin.value = True
        self._clock_pin.value = False

        if raw & 0x800000:
            raw -= 1 << 24
        return raw


class HX711LoadCell:
    def __init__(
        self,
        adc: HX711ADC,
        *,
        counts_per_newton: float,
        average_samples: int,
        tare_samples: int,
    ) -> None:
        self._adc = adc
        self._counts_per_newton = counts_per_newton
        self._history: deque[int] = deque((), max(1, average_samples))
        self._tare_counts = 0.0
        self._tare_samples = max(1, tare_samples)
        self._last_force_n = 0.0

    def probe(self, timeout_s: float = 0.2) -> bool:
        sample = self._adc.read_raw(timeout_s=timeout_s)
        if sample is None:
            return False
        self._history.append(sample)
        return True

    def tare(self) -> float:
        samples: list[int] = []
        for _ in range(self._tare_samples):
            sample = self._adc.read_raw(timeout_s=0.2)
            if sample is not None:
                samples.append(sample)
        if not samples:
            raise RuntimeError("HX711 did not provide a tare sample.")
        self._tare_counts = sum(samples) / len(samples)
        self._history.clear()
        self._last_force_n = 0.0
        return self._tare_counts

    def read_newtons(self) -> float:
        sample = self._adc.read_raw(timeout_s=0.0)
        if sample is not None:
            self._history.append(sample)
        if not self._history:
            return self._last_force_n
        average_counts = sum(self._history) / len(self._history)
        self._last_force_n = counts_to_newtons(
            raw_counts=average_counts,
            tare_counts=self._tare_counts,
            counts_per_newton=self._counts_per_newton,
        )
        return self._last_force_n


class SoftwareUARTTransmitter:
    def __init__(self, tx_pin, *, baudrate: int) -> None:
        self._tx_pin = tx_pin
        self._baudrate = max(1200, int(baudrate))
        self._bit_time_ns = max(1, int(round(1_000_000_000 / self._baudrate)))
        self._tx_pin.value = True

    @property
    def baudrate(self) -> int:
        return self._baudrate

    def write(self, data) -> int:
        payload = bytes(data)
        if not payload:
            return 0
        for byte in payload:
            self._write_byte(byte)
        return len(payload)

    def flush(self) -> None:
        return

    def _write_byte(self, value: int) -> None:
        try:
            deadline_ns = time.monotonic_ns()
        except AttributeError:
            deadline_ns = None

        bits = [False]
        for bit_index in range(8):
            bits.append(bool((value >> bit_index) & 0x01))
        bits.append(True)

        for bit_value in bits:
            self._tx_pin.value = bit_value
            if deadline_ns is None:
                delay_us_compatible(int(round(self._bit_time_ns / 1000)))
            else:
                deadline_ns += self._bit_time_ns
                self._wait_until(deadline_ns)
        self._tx_pin.value = True

    def _wait_until(self, deadline_ns: int) -> None:
        while True:
            remaining_ns = deadline_ns - time.monotonic_ns()
            if remaining_ns <= 0:
                return
            if remaining_ns > 50_000:
                delay_us_compatible(int((remaining_ns - 20_000) / 1000))


class TMC2209WriteOnlyDriver:
    def __init__(
        self,
        uart,
        *,
        run_current_a: float,
        hold_current_a: float,
        sense_resistor_ohms: float,
        microsteps: int,
        interpolate: bool,
        address: int = 0,
    ) -> None:
        self._uart = uart
        self._run_current_a = run_current_a
        self._hold_current_a = hold_current_a
        self._sense_resistor_ohms = sense_resistor_ohms
        self._microsteps = microsteps
        self._interpolate = interpolate
        self._address = address & 0xFF

    def configure(self) -> None:
        gconf = (1 << 6) | (1 << 7)
        ihold = tmc2209_current_to_cs(self._hold_current_a, self._sense_resistor_ohms)
        irun = tmc2209_current_to_cs(self._run_current_a, self._sense_resistor_ohms)
        ihold_irun = ihold | (irun << 8) | (6 << 16)
        chopconf = 3 | (microsteps_to_mres(self._microsteps) << 24)
        if self._interpolate:
            chopconf |= 1 << 28

        self._write_register(TMC2209_GCONF, gconf)
        self._write_register(TMC2209_IHOLD_IRUN, ihold_irun)
        self._write_register(TMC2209_CHOPCONF, chopconf)

    def _write_register(self, register: int, value: int) -> None:
        frame = bytearray(
            (
                TMC2209_SYNC,
                self._address,
                (register | 0x80) & 0xFF,
                (value >> 24) & 0xFF,
                (value >> 16) & 0xFF,
                (value >> 8) & 0xFF,
                value & 0xFF,
            )
        )
        frame.append(tmc2209_crc(frame))
        self._uart.write(frame)
        flush = getattr(self._uart, "flush", None)
        if flush is not None:
            flush()
        time.sleep(0.002)


def build_default_config() -> HardwareConfig:
    try:
        import config as settings
    except ImportError:
        from . import config as settings

    default_home_direction = int(settings.HOME_DIRECTION)
    left_home_direction = int(getattr(settings, "LEFT_HOME_DIRECTION", default_home_direction))
    right_home_direction = int(getattr(settings, "RIGHT_HOME_DIRECTION", default_home_direction))
    left_direction_inverted = bool(getattr(settings, "MOTOR_A_DIRECTION_INVERTED", False))
    right_direction_inverted = bool(getattr(settings, "MOTOR_B_DIRECTION_INVERTED", False))
    left_encoder_inverted = bool(getattr(settings, "LEFT_ENCODER_INVERTED", False))
    right_encoder_inverted = bool(getattr(settings, "RIGHT_ENCODER_INVERTED", False))
    default_switch_active_low = bool(settings.HOME_SWITCH_ACTIVE_LOW)
    default_switch_pull_up = bool(settings.HOME_SWITCH_PULL_UP)
    left_switch_active_low = bool(
        getattr(settings, "LEFT_HOME_SWITCH_ACTIVE_LOW", default_switch_active_low)
    )
    right_switch_active_low = bool(
        getattr(settings, "RIGHT_HOME_SWITCH_ACTIVE_LOW", default_switch_active_low)
    )
    left_switch_pull_up = bool(getattr(settings, "LEFT_HOME_SWITCH_PULL_UP", default_switch_pull_up))
    right_switch_pull_up = bool(
        getattr(settings, "RIGHT_HOME_SWITCH_PULL_UP", default_switch_pull_up)
    )
    home_switch_mode = str(getattr(settings, "HOME_SWITCH_MODE", "independent")).strip().lower()
    if home_switch_mode == "shared":
        home_switch_mode = "single"
    if home_switch_mode not in {"independent", "single"}:
        raise ValueError("HOME_SWITCH_MODE must be 'independent' or 'single'.")
    single_home_switch_axis = str(
        getattr(settings, "SINGLE_HOME_SWITCH_AXIS", "right")
    ).strip().lower() or "right"

    return HardwareConfig(
        sample_interval_s=float(settings.SAMPLE_INTERVAL_S),
        step_pulse_s=float(settings.STEP_PULSE_S),
        axes=(
            AxisConfig(
                name="left",
                step_pin=settings.MOTOR_A_STEP_PIN,
                dir_pin=settings.MOTOR_A_DIR_PIN,
                enable_pin=settings.MOTOR_A_ENABLE_PIN,
                uart_pin=settings.MOTOR_A_UART_PIN,
                home_switch_pin=settings.LEFT_HOME_SWITCH_PIN,
                encoder_scl_pin=settings.LEFT_ENCODER_SCL_PIN,
                encoder_sda_pin=settings.LEFT_ENCODER_SDA_PIN,
                run_current_a=float(settings.MOTOR_A_RUN_CURRENT_A),
                hold_current_a=float(settings.MOTOR_A_HOLD_CURRENT_A),
                home_direction=left_home_direction,
                direction_inverted=left_direction_inverted,
                encoder_inverted=left_encoder_inverted,
                home_switch_active_low=left_switch_active_low,
                home_switch_pull_up=left_switch_pull_up,
            ),
            AxisConfig(
                name="right",
                step_pin=settings.MOTOR_B_STEP_PIN,
                dir_pin=settings.MOTOR_B_DIR_PIN,
                enable_pin=settings.MOTOR_B_ENABLE_PIN,
                uart_pin=settings.MOTOR_B_UART_PIN,
                home_switch_pin=settings.RIGHT_HOME_SWITCH_PIN,
                encoder_scl_pin=settings.RIGHT_ENCODER_SCL_PIN,
                encoder_sda_pin=settings.RIGHT_ENCODER_SDA_PIN,
                run_current_a=float(settings.MOTOR_B_RUN_CURRENT_A),
                hold_current_a=float(settings.MOTOR_B_HOLD_CURRENT_A),
                home_direction=right_home_direction,
                direction_inverted=right_direction_inverted,
                encoder_inverted=right_encoder_inverted,
                home_switch_active_low=right_switch_active_low,
                home_switch_pull_up=right_switch_pull_up,
            ),
        ),
        mechanics=MechanicalConfig(
            lead_mm_per_rev=float(settings.SCREW_LEAD_MM_PER_REV),
            full_steps_per_rev=int(settings.STEPPER_FULL_STEPS_PER_REV),
            microsteps=int(settings.STEPPER_MICROSTEPS),
            encoder_counts_per_rev=int(settings.ENCODER_COUNTS_PER_REV),
            encoder_slip_tolerance_mm=float(settings.ENCODER_SLIP_TOLERANCE_MM),
        ),
        homing=HomingConfig(
            fast_speed_mm_per_min=float(settings.HOME_FAST_SPEED_MM_PER_MIN),
            slow_speed_mm_per_min=float(settings.HOME_SLOW_SPEED_MM_PER_MIN),
            backoff_mm=float(settings.HOME_BACKOFF_MM),
            timeout_s=float(settings.HOME_TIMEOUT_S),
            switch_mode=home_switch_mode,
            single_switch_axis=single_home_switch_axis,
        ),
        load_cell=LoadCellConfig(
            data_pin=settings.HX711_DATA_PIN,
            clock_pin=settings.HX711_CLOCK_PIN,
            counts_per_newton=float(settings.HX711_COUNTS_PER_NEWTON),
            average_samples=int(settings.HX711_AVERAGE_SAMPLES),
            tare_samples=int(settings.HX711_TARE_SAMPLES),
        ),
        switches=SwitchConfig(
            active_low=bool(settings.HOME_SWITCH_ACTIVE_LOW),
            pull_up=bool(settings.HOME_SWITCH_PULL_UP),
        ),
        driver=DriverConfig(
            baudrate=int(settings.TMC_UART_BAUDRATE),
            interpolate=bool(settings.TMC_INTERPOLATE),
            sense_resistor_ohms=float(settings.TMC_SENSE_RESISTOR_OHMS),
        ),
    )


def resolve_board_pin(board_module, pin_name: str):
    normalized = str(pin_name).strip()
    if not normalized:
        raise RuntimeError("Pin name cannot be empty.")

    candidates = (
        normalized,
        normalized.upper(),
        normalized.lower(),
        normalized.replace("gpio", "GP"),
        normalized.replace("GPIO", "GP"),
        normalized.replace("gp", "GP"),
        normalized.replace("Gp", "GP"),
    )
    for candidate in candidates:
        if hasattr(board_module, candidate):
            return getattr(board_module, candidate)
    raise RuntimeError("Could not resolve board pin %r." % pin_name)


def create_default_hardware_devices(settings: HardwareConfig) -> HardwareDevices:
    try:
        import board
        import busio
        import digitalio
    except ImportError as exc:
        raise RuntimeError("CircuitPython hardware modules are unavailable.") from exc

    warnings: list[str] = []
    fallback_steps_per_mm = calculate_steps_per_mm(
        full_steps_per_rev=settings.mechanics.full_steps_per_rev,
        microsteps=settings.mechanics.microsteps,
        lead_mm_per_rev=settings.mechanics.lead_mm_per_rev,
    )
    software_uart_baudrate = max(1200, min(int(settings.driver.baudrate), 9600))

    def wrap_pin_error(label: str, pin_name: str, exc: Exception) -> RuntimeError:
        return RuntimeError("%s pin %r is unavailable: %s" % (label, pin_name, exc))

    def create_output_pin(pin_name: str, initial_value: bool = False):
        try:
            pin = digitalio.DigitalInOut(resolve_board_pin(board, pin_name))
            switch_pin_to_output(pin, initial_value)
            return pin
        except Exception as exc:
            raise wrap_pin_error("Output", pin_name, exc) from exc

    def create_input_pin(pin_name: str, *, pull_up: bool):
        try:
            pin = digitalio.DigitalInOut(resolve_board_pin(board, pin_name))
            switch_pin_to_input(pin, digitalio.Pull.UP if pull_up else None)
            return pin
        except Exception as exc:
            raise wrap_pin_error("Input", pin_name, exc) from exc

    def create_axis(axis_config: AxisConfig) -> AxisDevices:
        step_counter = StepCounter()
        switch_pull_up = (
            settings.switches.pull_up
            if axis_config.home_switch_pull_up is None
            else bool(axis_config.home_switch_pull_up)
        )
        switch_active_low = (
            settings.switches.active_low
            if axis_config.home_switch_active_low is None
            else bool(axis_config.home_switch_active_low)
        )
        step_pin = create_output_pin(axis_config.step_pin, initial_value=False)
        dir_pin = create_output_pin(axis_config.dir_pin, initial_value=False)
        enable_pin = create_output_pin(axis_config.enable_pin, initial_value=True)
        switch_pin = create_input_pin(
            axis_config.home_switch_pin,
            pull_up=switch_pull_up,
        )
        motor = CoupledDigitalStepperMotor(
            step_pin=step_pin,
            dir_pin=dir_pin,
            enable_pin=enable_pin,
            pulse_width_s=settings.step_pulse_s,
            enable_active_low=True,
            direction_inverted=axis_config.direction_inverted,
            step_counter=step_counter,
        )
        try:
            encoder_i2c = create_i2c_bus(
                busio,
                resolve_board_pin(board, axis_config.encoder_scl_pin),
                resolve_board_pin(board, axis_config.encoder_sda_pin),
                400000,
            )
            encoder = AS5600Sensor(encoder_i2c)
        except Exception as exc:
            encoder = StepTrackingEncoder(
                step_counter,
                steps_per_mm=fallback_steps_per_mm,
                lead_mm_per_rev=settings.mechanics.lead_mm_per_rev,
                counts_per_rev=settings.mechanics.encoder_counts_per_rev,
            )
            warnings.append(
                "%s encoder fallback active: I2C pins %r/%r unavailable (%s)."
                % (
                    format_axis_name(axis_config.name),
                    axis_config.encoder_scl_pin,
                    axis_config.encoder_sda_pin,
                    exc,
                )
            )

        try:
            soft_uart_pin = create_output_pin(axis_config.uart_pin, initial_value=True)
            uart = SoftwareUARTTransmitter(
                soft_uart_pin,
                baudrate=software_uart_baudrate,
            )
            driver = TMC2209WriteOnlyDriver(
                uart,
                run_current_a=axis_config.run_current_a,
                hold_current_a=axis_config.hold_current_a,
                sense_resistor_ohms=settings.driver.sense_resistor_ohms,
                microsteps=settings.mechanics.microsteps,
                interpolate=settings.driver.interpolate,
            )
            warnings.append(
                "%s driver software UART active on pin %r at %d baud."
                % (
                    format_axis_name(axis_config.name),
                    axis_config.uart_pin,
                    software_uart_baudrate,
                )
            )
        except Exception as soft_exc:
            try:
                uart = create_uart_bus(
                    busio,
                    resolve_board_pin(board, axis_config.uart_pin),
                    None,
                    settings.driver.baudrate,
                    0.02,
                )
                driver = TMC2209WriteOnlyDriver(
                    uart,
                    run_current_a=axis_config.run_current_a,
                    hold_current_a=axis_config.hold_current_a,
                    sense_resistor_ohms=settings.driver.sense_resistor_ohms,
                    microsteps=settings.mechanics.microsteps,
                    interpolate=settings.driver.interpolate,
                )
                warnings.append(
                    "%s driver hardware UART active on pin %r after software UART failed (%s)."
                    % (
                        format_axis_name(axis_config.name),
                        axis_config.uart_pin,
                        soft_exc,
                    )
                )
            except Exception as hard_exc:
                driver = NullDriver()
                warnings.append(
                    "%s driver UART disabled: pin %r unavailable (software UART failed: %s; hardware UART failed: %s)."
                    % (
                        format_axis_name(axis_config.name),
                        axis_config.uart_pin,
                        soft_exc,
                        hard_exc,
                    )
                )
        return AxisDevices(
            config=axis_config,
            motor=motor,
            encoder=encoder,
            home_switch=DigitalLimitSwitch(
                switch_pin,
                active_low=switch_active_low,
                pin_name=axis_config.home_switch_pin,
                pull_up=switch_pull_up,
            ),
            driver=driver,
        )

    try:
        data_pin = create_input_pin(settings.load_cell.data_pin, pull_up=False)
        clock_pin = create_output_pin(settings.load_cell.clock_pin, initial_value=False)
        load_cell = HX711LoadCell(
            HX711ADC(data_pin, clock_pin),
            counts_per_newton=settings.load_cell.counts_per_newton,
            average_samples=settings.load_cell.average_samples,
            tare_samples=settings.load_cell.tare_samples,
        )
    except Exception as exc:
        load_cell = NullLoadCell()
        warnings.append("Load cell disabled: %s" % exc)

    return HardwareDevices(
        axes=(
            create_axis(settings.axes[0]),
            create_axis(settings.axes[1]),
        ),
        load_cell=load_cell,
        warnings=tuple(warnings),
    )


class AxisRuntime:
    def __init__(
        self,
        devices: AxisDevices,
        tracker: EncoderTracker,
        steps_per_mm: float,
        expected_steps: int = 0,
        home_phase: str = "done",
        step_budget: float = 0.0,
        backoff_remaining_steps: int = 0,
        switch_pressed: bool = False,
        magnet_detected: bool = True,
        last_step_direction: int = 0,
    ) -> None:
        self.devices = devices
        self.tracker = tracker
        self.steps_per_mm = steps_per_mm
        self.expected_steps = expected_steps
        self.home_phase = home_phase
        self.step_budget = step_budget
        self.backoff_remaining_steps = backoff_remaining_steps
        self.switch_pressed = switch_pressed
        self.magnet_detected = magnet_detected
        self.last_step_direction = last_step_direction

    @property
    def expected_mm(self) -> float:
        return self.expected_steps / self.steps_per_mm


class HardwareTesterBackend:
    def __init__(
        self,
        *,
        settings: HardwareConfig | None = None,
        devices: HardwareDevices | None = None,
    ) -> None:
        self._settings = settings or build_default_config()
        self._devices = devices
        self._steps_per_mm = calculate_steps_per_mm(
            full_steps_per_rev=self._settings.mechanics.full_steps_per_rev,
            microsteps=self._settings.mechanics.microsteps,
            lead_mm_per_rev=self._settings.mechanics.lead_mm_per_rev,
        )
        self.sample_interval_s = self._settings.sample_interval_s
        self.state = "idle"
        self.startup_message = "Hardware mode enabled."
        self._last_motion_time = 0.0
        self._last_sample_time = 0.0
        self._run_started_at: float | None = None
        self._zero_displacement_mm = 0.0
        self._latest_force_n = 0.0
        self._motion_speed_mm_per_min = 0.0
        self._motion_direction = 1
        self._sync_step_budget = 0.0
        self._jog_remaining_steps = 0
        self._home_started_at: float | None = None
        self._homed = False
        self._pending_run_speed_mm_per_min: float | None = None
        self._fault_code = ""
        self._fault_message = ""
        self._motion_completed_message: str | None = None
        self._pending_status_messages: list[dict[str, str]] = []
        self._startup_warnings: list[str] = []
        self._shared_home_axis: AxisRuntime | None = None

        try:
            hardware = self._devices or create_default_hardware_devices(self._settings)
            self._startup_warnings = list(getattr(hardware, "warnings", ()))
            self._axes = [
                AxisRuntime(
                    devices=hardware.axes[0],
                    tracker=EncoderTracker(
                        counts_per_rev=self._settings.mechanics.encoder_counts_per_rev,
                        lead_mm_per_rev=self._settings.mechanics.lead_mm_per_rev,
                        inverted=hardware.axes[0].config.encoder_inverted,
                    ),
                    steps_per_mm=self._steps_per_mm,
                ),
                AxisRuntime(
                    devices=hardware.axes[1],
                    tracker=EncoderTracker(
                        counts_per_rev=self._settings.mechanics.encoder_counts_per_rev,
                        lead_mm_per_rev=self._settings.mechanics.lead_mm_per_rev,
                        inverted=hardware.axes[1].config.encoder_inverted,
                    ),
                    steps_per_mm=self._steps_per_mm,
                ),
            ]
            self._load_cell = hardware.load_cell
            self._shared_home_axis = self._resolve_shared_home_axis()
            self._initialise_hardware()
        except Exception as exc:
            self.state = "fault"
            self.startup_message = "Hardware init failed: %s" % exc
            self._axes = []
            self._load_cell = None
            self._fault_code = "hardware_init_failed"
            self._fault_message = str(exc)

    def _initialise_hardware(self) -> None:
        for axis in self._axes:
            axis.devices.motor.set_enabled(True)
            driver = axis.devices.driver or NullDriver()
            driver.configure()
        self._probe_load_cell()
        self._refresh_measurements()
        if self.state == "fault":
            raise RuntimeError(self._fault_message or "Hardware self-check failed.")
        for axis in self._axes:
            axis.tracker.zero_here()
        self._zero_displacement_mm = self._average_encoder_mm()
        self.startup_message = "Hardware backend ready. Home before testing."
        if self._shared_home_axis is not None:
            self.startup_message += (
                " Single-endstop homing uses %s switch on pin %r."
                % (
                    format_axis_name(self._shared_home_axis.devices.config.name),
                    self._shared_home_axis.devices.config.home_switch_pin,
                )
            )
        if self._startup_warnings:
            self.startup_message += " " + " ".join(self._startup_warnings)

    def _append_startup_warning(self, message: str) -> None:
        if message not in self._startup_warnings:
            self._startup_warnings.append(message)

    def _probe_load_cell(self) -> None:
        probe = getattr(self._load_cell, "probe", None)
        if probe is None:
            return
        try:
            ready = bool(probe(timeout_s=0.5))
        except Exception as exc:
            self._append_startup_warning("Load cell probe failed: %s" % exc)
            return
        if not ready:
            self._append_startup_warning(
                "Load cell did not produce an HX711 sample during startup. Check HX711 power, DT, SCK, and pin mapping."
            )

    def _resolve_shared_home_axis(self) -> AxisRuntime | None:
        if self._settings.homing.switch_mode != "single":
            return None
        target_axis = self._settings.homing.single_switch_axis
        for axis in self._axes:
            if str(axis.devices.config.name).strip().lower() == target_axis:
                return axis
        raise RuntimeError(
            "SINGLE_HOME_SWITCH_AXIS %r does not match any configured axis."
            % self._settings.homing.single_switch_axis
        )

    def _activate_encoder_fallback(self, axis: AxisRuntime, reason: str) -> bool:
        step_counter = getattr(axis.devices.motor, "_step_counter", None)
        if step_counter is None:
            return False
        if isinstance(axis.devices.encoder, StepTrackingEncoder):
            return True
        axis.devices.encoder = StepTrackingEncoder(
            step_counter,
            steps_per_mm=self._steps_per_mm,
            lead_mm_per_rev=self._settings.mechanics.lead_mm_per_rev,
            counts_per_rev=self._settings.mechanics.encoder_counts_per_rev,
        )
        self._append_startup_warning(reason)
        return True

    def handle_command(self, command, now):
        self._advance_motion(now)
        cmd = str(command.get("cmd", ""))

        if self.state == "fault":
            return [self._error(self._fault_code or "fault", self._fault_message or "Device fault active.")]

        if self.state == "estop" and cmd in {"tare_force", "zero_displacement", "jog", "home", "start_test"}:
            return [self._error("estop_active", "Reset the board to clear E-Stop.")]

        if cmd == "home":
            if self.state in ACTIVE_STATES:
                return [self._error("busy", "Cannot home while motion is active.")]
            self._begin_homing(now, pending_run_speed_mm_per_min=None)
            return [self._status("homing", "Homing started.")]

        if cmd == "tare_force":
            if self.state != "idle":
                return [self._error("busy", "Force can only be tared while idle.")]
            self._load_cell.tare()
            self._latest_force_n = self._load_cell.read_newtons()
            return [self._status("idle", "Force tared."), self._sample(now)]

        if cmd == "zero_displacement":
            if self.state != "idle":
                return [self._error("busy", "Displacement can only be zeroed while idle.")]
            self._refresh_measurements()
            self._zero_displacement_mm = self._average_encoder_mm()
            return [self._status("idle", "Displacement zeroed."), self._sample(now)]

        if cmd == "jog":
            if self.state in ACTIVE_STATES:
                return [self._error("busy", "Cannot jog while motion is active.")]
            direction = str(command.get("direction", "forward"))
            if direction not in {"forward", "reverse"}:
                return [self._error("invalid_direction", "Direction must be 'forward' or 'reverse'.")]
            distance_mm = float(command.get("distance_mm", 0.0))
            speed_mm_per_min = float(command.get("speed_mm_per_min", 0.0))
            if distance_mm <= 0.0:
                return [self._error("invalid_distance", "Jog distance must be greater than zero.")]
            if speed_mm_per_min <= 0.0:
                return [self._error("invalid_speed", "Jog speed must be greater than zero.")]

            self._begin_jog(
                now=now,
                direction=1 if direction == "forward" else -1,
                distance_mm=distance_mm,
                speed_mm_per_min=speed_mm_per_min,
            )
            return [self._status("jogging", "Jog started.")]

        if cmd == "start_test":
            speed_mm_per_min = float(command.get("speed_mm_per_min", 0.0))
            if speed_mm_per_min <= 0.0:
                return [self._error("invalid_speed", "Test speed must be greater than zero.")]
            if self.state in ACTIVE_STATES:
                return [self._error("busy", "Cannot start a test while motion is active.")]

            if not self._homed:
                self._begin_homing(now, pending_run_speed_mm_per_min=speed_mm_per_min)
                return [self._status("homing", "Homing before test.")]

            self._start_running(speed_mm_per_min)
            self._run_started_at = now
            return [self._status("running", "Pull started at %.3f mm/min." % speed_mm_per_min)]

        if cmd == "stop":
            if self.state in ACTIVE_STATES:
                self._stop_motion(clear_pending_run=True)
                return [self._sample(now), self._status("idle", "Motion stopped.")]
            return [self._status(self.state, "Stop ignored.")]

        if cmd == "estop":
            self._estop()
            return [self._sample(now), self._status("estop", "Emergency stop triggered.")]

        return [self._error("unknown_command", "Unsupported command: %r" % cmd)]

    def poll(self, now):
        if self.state == "fault":
            return []

        self._advance_motion(now)
        messages = []
        sample_emitted = False
        if now - self._last_sample_time >= self.sample_interval_s:
            self._last_sample_time = now
            messages.append(self._sample(now))
            sample_emitted = True

        if self._pending_status_messages:
            messages.extend(self._pending_status_messages)
            self._pending_status_messages = []

        if self._motion_completed_message is not None:
            message = self._motion_completed_message
            self._motion_completed_message = None
            if not sample_emitted:
                messages.append(self._sample(now))
            messages.append(self._status("idle", message))

        return messages

    def _advance_motion(self, now: float) -> None:
        if not self._axes:
            return
        self._refresh_measurements()

        if self._last_motion_time == 0.0:
            self._last_motion_time = now
            return

        delta_s = max(0.0, now - self._last_motion_time)
        self._last_motion_time = now

        if self.state == "homing":
            self._advance_homing(delta_s)
        elif self.state == "jogging":
            self._advance_sync_motion(delta_s, finish_message="Jog complete.")
        elif self.state == "running":
            self._advance_sync_motion(delta_s, finish_message=None)

        self._refresh_measurements()
        self._check_unexpected_switches()
        self._check_encoder_slip()

        if self.state == "homing" and self._home_started_at is not None:
            if now - self._home_started_at > self._settings.homing.timeout_s:
                self._fault(
                    "home_timeout",
                    "Homing did not complete within %.1f seconds." % self._settings.homing.timeout_s,
                )

    def _begin_homing(
        self,
        now: float,
        pending_run_speed_mm_per_min: float | None,
    ) -> None:
        self._stop_motion(clear_pending_run=False)
        self._pending_run_speed_mm_per_min = pending_run_speed_mm_per_min
        self._home_started_at = now
        self._motion_speed_mm_per_min = 0.0
        self._motion_direction = 0
        self._sync_step_budget = 0.0
        self._jog_remaining_steps = 0
        self.state = "homing"
        self._motion_completed_message = None
        for axis in self._axes:
            axis.devices.motor.set_enabled(True)
            axis.home_phase = "seek_fast"
            axis.step_budget = 0.0
            axis.backoff_remaining_steps = 0
            axis.last_step_direction = 0
            axis.switch_pressed = axis.devices.home_switch.is_pressed()
        self._queue_status_message("homing", self._format_homing_snapshot("Homing detail"))
        if self._shared_home_axis is not None:
            self._queue_status_message(
                "homing",
                "Single-endstop homing active using %s switch (%s)."
                % (
                    format_axis_name(self._shared_home_axis.devices.config.name),
                    self._describe_axis_switch(self._shared_home_axis),
                ),
            )
        for axis in self._axes:
            if self._shared_home_axis is not None and axis is not self._shared_home_axis:
                continue
            if axis.switch_pressed:
                self._queue_status_message(
                    "homing",
                    "%s home switch is already pressed before motion (%s)."
                    % (
                        format_axis_name(axis.devices.config.name),
                        self._describe_axis_switch(axis),
                    ),
                )

    def _begin_jog(
        self,
        *,
        now: float,
        direction: int,
        distance_mm: float,
        speed_mm_per_min: float,
    ) -> None:
        _ = now
        self._stop_motion(clear_pending_run=True)
        self.state = "jogging"
        self._motion_direction = 1 if direction >= 0 else -1
        self._motion_speed_mm_per_min = speed_mm_per_min
        self._sync_step_budget = 0.0
        self._jog_remaining_steps = max(1, int(round(distance_mm * self._steps_per_mm)))
        self._motion_completed_message = None
        for axis in self._axes:
            axis.devices.motor.set_enabled(True)

    def _start_running(self, speed_mm_per_min: float) -> None:
        self._stop_motion(clear_pending_run=False)
        self.state = "running"
        self._motion_direction = 1
        self._motion_speed_mm_per_min = speed_mm_per_min
        self._sync_step_budget = 0.0
        self._jog_remaining_steps = 0
        self._motion_completed_message = None
        for axis in self._axes:
            axis.devices.motor.set_enabled(True)

    def _advance_sync_motion(self, delta_s: float, finish_message: str | None) -> None:
        if delta_s <= 0.0:
            return
        steps_per_second = self._motion_speed_mm_per_min * self._steps_per_mm / 60.0
        self._sync_step_budget += steps_per_second * delta_s
        whole_steps = int(self._sync_step_budget)
        if whole_steps <= 0:
            return
        self._sync_step_budget -= whole_steps

        for _ in range(whole_steps):
            if self.state == "jogging" and self._jog_remaining_steps <= 0:
                break
            for axis in self._axes:
                axis.devices.motor.step(self._motion_direction)
                axis.expected_steps += self._motion_direction
            if self.state == "jogging":
                self._jog_remaining_steps -= 1
                if self._jog_remaining_steps <= 0:
                    self._stop_motion(clear_pending_run=True)
                    self._motion_completed_message = finish_message
                    break

    def _advance_homing(self, delta_s: float) -> None:
        if self._shared_home_axis is not None:
            self._advance_single_switch_homing(delta_s)
            return
        self._advance_independent_homing(delta_s)

    def _advance_independent_homing(self, delta_s: float) -> None:
        if delta_s <= 0.0:
            return

        fast_steps_per_second = (
            self._settings.homing.fast_speed_mm_per_min * self._steps_per_mm / 60.0
        )
        slow_steps_per_second = (
            self._settings.homing.slow_speed_mm_per_min * self._steps_per_mm / 60.0
        )
        backoff_steps = max(1, int(round(self._settings.homing.backoff_mm * self._steps_per_mm)))

        for axis in self._axes:
            if axis.home_phase == "done":
                continue

            axis.switch_pressed = axis.devices.home_switch.is_pressed()
            if axis.home_phase in {"seek_fast", "seek_slow"} and axis.switch_pressed:
                if axis.home_phase == "seek_fast":
                    previous_phase = axis.home_phase
                    axis.home_phase = "backoff"
                    axis.backoff_remaining_steps = backoff_steps
                    axis.last_step_direction = 0
                    self._queue_homing_transition(
                        axis,
                        previous_phase,
                        "switch already pressed",
                    )
                else:
                    previous_phase = axis.home_phase
                    axis.home_phase = "done"
                    axis.expected_steps = 0
                    axis.tracker.zero_here()
                    axis.last_step_direction = 0
                    self._queue_homing_transition(axis, previous_phase, "switch confirmed")
                continue

            if axis.home_phase == "backoff" and axis.backoff_remaining_steps <= 0:
                axis.switch_pressed = axis.devices.home_switch.is_pressed()
                if axis.switch_pressed:
                    self._fault(
                        "home_switch_stuck",
                        "%s home switch remained pressed after %.3f mm backoff (%s)."
                        % (
                            format_axis_name(axis.devices.config.name),
                            self._settings.homing.backoff_mm,
                            self._describe_axis_switch(axis),
                        ),
                    )
                    return
                previous_phase = axis.home_phase
                axis.home_phase = "seek_slow"
                axis.step_budget = 0.0
                axis.last_step_direction = 0
                self._queue_homing_transition(axis, previous_phase, "backoff complete")
                continue

            if axis.home_phase == "seek_fast":
                rate = fast_steps_per_second
                direction = axis.devices.config.home_direction
            elif axis.home_phase == "backoff":
                rate = slow_steps_per_second
                direction = -axis.devices.config.home_direction
            elif axis.home_phase == "seek_slow":
                rate = slow_steps_per_second
                direction = axis.devices.config.home_direction
            else:
                continue

            axis.step_budget += rate * delta_s
            whole_steps = int(axis.step_budget)
            if whole_steps <= 0:
                continue
            axis.step_budget -= whole_steps

            for _ in range(whole_steps):
                if axis.home_phase in {"seek_fast", "seek_slow"} and axis.devices.home_switch.is_pressed():
                    if axis.home_phase == "seek_fast":
                        previous_phase = axis.home_phase
                        axis.home_phase = "backoff"
                        axis.backoff_remaining_steps = backoff_steps
                        axis.last_step_direction = 0
                        self._queue_homing_transition(
                            axis,
                            previous_phase,
                            "switch triggered during move",
                        )
                    else:
                        previous_phase = axis.home_phase
                        axis.home_phase = "done"
                        axis.expected_steps = 0
                        axis.tracker.zero_here()
                        axis.last_step_direction = 0
                        self._queue_homing_transition(axis, previous_phase, "switch confirmed")
                    break

                axis.devices.motor.step(direction)
                axis.expected_steps += direction
                axis.last_step_direction = direction

                if axis.home_phase == "backoff":
                    axis.backoff_remaining_steps -= 1
                    if axis.backoff_remaining_steps <= 0:
                        axis.switch_pressed = axis.devices.home_switch.is_pressed()
                        if axis.switch_pressed:
                            self._fault(
                                "home_switch_stuck",
                                "%s home switch remained pressed after %.3f mm backoff (%s)."
                                % (
                                    format_axis_name(axis.devices.config.name),
                                    self._settings.homing.backoff_mm,
                                    self._describe_axis_switch(axis),
                                ),
                            )
                            return
                        previous_phase = axis.home_phase
                        axis.home_phase = "seek_slow"
                        axis.step_budget = 0.0
                        axis.last_step_direction = 0
                        self._queue_homing_transition(axis, previous_phase, "backoff complete")
                        break

        if all(axis.home_phase == "done" for axis in self._axes):
            self._homed = True
            self._zero_displacement_mm = self._average_encoder_mm()
            self._home_started_at = None
            pending_speed = self._pending_run_speed_mm_per_min
            self._pending_run_speed_mm_per_min = None
            if pending_speed is not None:
                self._start_running(pending_speed)
                self._run_started_at = time.monotonic()
            else:
                self._stop_motion(clear_pending_run=True)
                self._motion_completed_message = "Homing complete."

    def _advance_single_switch_homing(self, delta_s: float) -> None:
        if delta_s <= 0.0 or not self._axes or self._shared_home_axis is None:
            return

        for axis in self._axes:
            axis.switch_pressed = axis.devices.home_switch.is_pressed()

        fast_steps_per_second = (
            self._settings.homing.fast_speed_mm_per_min * self._steps_per_mm / 60.0
        )
        slow_steps_per_second = (
            self._settings.homing.slow_speed_mm_per_min * self._steps_per_mm / 60.0
        )
        backoff_steps = max(1, int(round(self._settings.homing.backoff_mm * self._steps_per_mm)))
        group_phase = self._axes[0].home_phase
        control_axis = self._shared_home_axis

        if group_phase == "done":
            self._complete_homing_if_ready()
            return

        if group_phase in {"seek_fast", "seek_slow"} and control_axis.switch_pressed:
            if group_phase == "seek_fast":
                self._transition_all_axes_to_backoff(
                    backoff_steps=backoff_steps,
                    reason="shared switch already pressed",
                )
            else:
                self._mark_all_axes_homed(reason="shared switch confirmed")
            self._complete_homing_if_ready()
            return

        if group_phase == "backoff" and self._axes[0].backoff_remaining_steps <= 0:
            if control_axis.switch_pressed:
                self._fault_shared_home_switch_stuck()
                return
            self._transition_all_axes_to_seek_slow(reason="backoff complete")
            return

        if group_phase == "seek_fast":
            rate = fast_steps_per_second
        elif group_phase == "backoff":
            rate = slow_steps_per_second
        elif group_phase == "seek_slow":
            rate = slow_steps_per_second
        else:
            return

        master_axis = self._axes[0]
        master_axis.step_budget += rate * delta_s
        whole_steps = int(master_axis.step_budget)
        if whole_steps <= 0:
            return
        master_axis.step_budget -= whole_steps
        for axis in self._axes[1:]:
            axis.step_budget = master_axis.step_budget

        for _ in range(whole_steps):
            control_axis.switch_pressed = control_axis.devices.home_switch.is_pressed()
            if group_phase in {"seek_fast", "seek_slow"} and control_axis.switch_pressed:
                if group_phase == "seek_fast":
                    self._transition_all_axes_to_backoff(
                        backoff_steps=backoff_steps,
                        reason="shared switch triggered during move",
                    )
                else:
                    self._mark_all_axes_homed(reason="shared switch confirmed")
                break

            for axis in self._axes:
                if group_phase == "backoff":
                    direction = -axis.devices.config.home_direction
                else:
                    direction = axis.devices.config.home_direction
                axis.devices.motor.step(direction)
                axis.expected_steps += direction
                axis.last_step_direction = direction

            if group_phase == "backoff":
                remaining_steps = max(0, self._axes[0].backoff_remaining_steps - 1)
                for axis in self._axes:
                    axis.backoff_remaining_steps = remaining_steps
                if remaining_steps <= 0:
                    for axis in self._axes:
                        axis.switch_pressed = axis.devices.home_switch.is_pressed()
                    if control_axis.switch_pressed:
                        self._fault_shared_home_switch_stuck()
                        return
                    self._transition_all_axes_to_seek_slow(reason="backoff complete")
                    break

        self._complete_homing_if_ready()

    def _transition_all_axes_to_backoff(self, *, backoff_steps: int, reason: str) -> None:
        for axis in self._axes:
            previous_phase = axis.home_phase
            axis.home_phase = "backoff"
            axis.backoff_remaining_steps = backoff_steps
            axis.last_step_direction = 0
            self._queue_homing_transition(axis, previous_phase, reason)

    def _transition_all_axes_to_seek_slow(self, *, reason: str) -> None:
        for axis in self._axes:
            previous_phase = axis.home_phase
            axis.home_phase = "seek_slow"
            axis.step_budget = 0.0
            axis.backoff_remaining_steps = 0
            axis.last_step_direction = 0
            self._queue_homing_transition(axis, previous_phase, reason)

    def _mark_all_axes_homed(self, *, reason: str) -> None:
        for axis in self._axes:
            previous_phase = axis.home_phase
            axis.home_phase = "done"
            axis.expected_steps = 0
            axis.backoff_remaining_steps = 0
            axis.tracker.zero_here()
            axis.last_step_direction = 0
            self._queue_homing_transition(axis, previous_phase, reason)

    def _fault_shared_home_switch_stuck(self) -> None:
        control_axis = self._shared_home_axis
        if control_axis is None:
            return
        self._fault(
            "home_switch_stuck",
            "%s home switch remained pressed after %.3f mm backoff (%s)."
            % (
                format_axis_name(control_axis.devices.config.name),
                self._settings.homing.backoff_mm,
                self._describe_axis_switch(control_axis),
            ),
        )

    def _complete_homing_if_ready(self) -> None:
        if not self._axes or not all(axis.home_phase == "done" for axis in self._axes):
            return
        self._homed = True
        self._zero_displacement_mm = self._average_encoder_mm()
        self._home_started_at = None
        pending_speed = self._pending_run_speed_mm_per_min
        self._pending_run_speed_mm_per_min = None
        if pending_speed is not None:
            self._start_running(pending_speed)
            self._run_started_at = time.monotonic()
        else:
            self._stop_motion(clear_pending_run=True)
            self._motion_completed_message = "Homing complete."

    def _refresh_measurements(self) -> None:
        if not self._axes:
            return

        for axis in self._axes:
            try:
                axis.magnet_detected = axis.devices.encoder.magnet_detected()
                if not axis.magnet_detected:
                    raise RuntimeError("magnet not detected")
                raw_angle = axis.devices.encoder.read_raw_angle()
            except Exception as exc:
                fallback_reason = "%s encoder fallback active: %s." % (
                    format_axis_name(axis.devices.config.name),
                    exc,
                )
                if not self._activate_encoder_fallback(axis, fallback_reason):
                    self._fault(
                        "encoder_missing_magnet",
                        "%s encoder unavailable: %s." % (
                            format_axis_name(axis.devices.config.name),
                            exc,
                        ),
                    )
                    return
                axis.magnet_detected = True
                raw_angle = axis.devices.encoder.read_raw_angle()
            axis.tracker.update(raw_angle)
            axis.switch_pressed = axis.devices.home_switch.is_pressed()

        if self._load_cell is not None:
            self._latest_force_n = self._load_cell.read_newtons()

    def _check_unexpected_switches(self) -> None:
        if self.state not in {"running", "jogging"}:
            return
        monitored_axes = (self._shared_home_axis,) if self._shared_home_axis is not None else self._axes
        for axis in monitored_axes:
            if axis is None:
                continue
            if axis.switch_pressed and self._motion_direction == axis.devices.config.home_direction:
                self._fault(
                    "limit_switch_triggered",
                    "%s home switch triggered outside homing." % format_axis_name(axis.devices.config.name),
                )
                return

    def _check_encoder_slip(self) -> None:
        if self.state not in {"running", "jogging"}:
            return
        tolerance_mm = self._settings.mechanics.encoder_slip_tolerance_mm
        for axis in self._axes:
            slip_mm = abs(axis.tracker.relative_mm - axis.expected_mm)
            if slip_mm > tolerance_mm:
                self._fault(
                    "step_mismatch",
                    "%s axis encoder mismatch %.3f mm exceeds %.3f mm."
                    % (format_axis_name(axis.devices.config.name), slip_mm, tolerance_mm),
                )
                return

    def _average_encoder_mm(self) -> float:
        if not self._axes:
            return 0.0
        return sum(axis.tracker.relative_mm for axis in self._axes) / len(self._axes)

    def _current_displacement_mm(self) -> float:
        return self._average_encoder_mm() - self._zero_displacement_mm

    def _stop_motion(self, *, clear_pending_run: bool) -> None:
        self._motion_speed_mm_per_min = 0.0
        self._sync_step_budget = 0.0
        self._jog_remaining_steps = 0
        if clear_pending_run:
            self._pending_run_speed_mm_per_min = None
        for axis in self._axes:
            axis.home_phase = "done"
            axis.step_budget = 0.0
            axis.backoff_remaining_steps = 0

    def _estop(self) -> None:
        self._stop_motion(clear_pending_run=True)
        for axis in self._axes:
            axis.devices.motor.set_enabled(False)
        self._run_started_at = None
        self.state = "estop"

    def _fault(self, code: str, message: str) -> None:
        self._fault_code = code
        self._fault_message = message
        self._stop_motion(clear_pending_run=True)
        for axis in self._axes:
            axis.devices.motor.set_enabled(False)
        self._run_started_at = None
        self.state = "fault"

    def _sample(self, now):
        if self._run_started_at is None:
            timestamp_s = 0.0
        else:
            timestamp_s = max(0.0, now - self._run_started_at)

        sample = {
            "type": "sample",
            "timestamp_s": round(timestamp_s, 3),
            "force_n": round(self._latest_force_n, 3),
            "displacement_mm": round(self._current_displacement_mm(), 3),
            "state": self.state,
        }
        if self.state == "homing":
            sample["axes"] = [self._build_axis_diagnostics(axis) for axis in self._axes]
        return sample

    def _status(self, state, message):
        self.state = state
        return {"type": "status", "state": state, "message": message}

    def _error(self, code, message):
        return {"type": "error", "code": code, "message": message}

    def _queue_status_message(self, state: str, message: str) -> None:
        self._pending_status_messages.append(
            {
                "type": "status",
                "state": state,
                "message": message,
            }
        )

    def _queue_homing_transition(
        self,
        axis: AxisRuntime,
        previous_phase: str,
        reason: str,
    ) -> None:
        if previous_phase == axis.home_phase:
            return
        message = (
            "%s homing %s -> %s (%s; switch=%s; home_dir=%+d; backoff_steps=%d; %s)"
            % (
                format_axis_name(axis.devices.config.name),
                previous_phase,
                axis.home_phase,
                reason,
                "pressed" if axis.switch_pressed else "open",
                axis.devices.config.home_direction,
                axis.backoff_remaining_steps,
                self._describe_axis_switch(axis),
            )
        )
        self._queue_status_message("homing", message)

    def _format_homing_snapshot(self, prefix: str) -> str:
        details = []
        for axis in self._axes:
            diagnostics = self._build_axis_diagnostics(axis)
            details.append(
                "%s phase=%s switch=%s home_dir=%+d motor_dir=%+d inverted=%s pin=%s active_low=%s pull_up=%s raw=%s"
                % (
                    format_axis_name(str(diagnostics["axis"])),
                    diagnostics["phase"],
                    "pressed" if diagnostics["switch_pressed"] else "open",
                    diagnostics["home_direction"],
                    diagnostics["last_step_direction"],
                    "yes" if diagnostics["direction_inverted"] else "no",
                    diagnostics["home_switch_pin"],
                    "yes" if diagnostics["home_switch_active_low"] else "no",
                    "yes" if diagnostics["home_switch_pull_up"] else "no",
                    "high" if diagnostics["home_switch_raw_value"] else "low",
                )
            )
        return "%s: %s" % (prefix, "; ".join(details))

    def _build_axis_diagnostics(self, axis: AxisRuntime) -> dict[str, object]:
        home_switch = axis.devices.home_switch
        raw_value_getter = getattr(home_switch, "raw_value", None)
        active_low = getattr(home_switch, "active_low", axis.devices.config.home_switch_active_low)
        if active_low is None:
            active_low = self._settings.switches.active_low
        pull_up = getattr(home_switch, "pull_up", axis.devices.config.home_switch_pull_up)
        if pull_up is None:
            pull_up = self._settings.switches.pull_up
        if raw_value_getter is None:
            raw_value = not axis.switch_pressed if active_low else axis.switch_pressed
        else:
            raw_value = bool(raw_value_getter())
        return {
            "axis": axis.devices.config.name,
            "phase": axis.home_phase,
            "switch_pressed": axis.switch_pressed,
            "home_direction": axis.devices.config.home_direction,
            "last_step_direction": axis.last_step_direction,
            "direction_inverted": axis.devices.config.direction_inverted,
            "backoff_remaining_steps": axis.backoff_remaining_steps,
            "encoder_mm": round(axis.tracker.relative_mm, 3),
            "expected_mm": round(axis.expected_mm, 3),
            "home_switch_pin": getattr(home_switch, "pin_name", axis.devices.config.home_switch_pin),
            "home_switch_active_low": active_low,
            "home_switch_pull_up": pull_up,
            "home_switch_raw_value": raw_value,
            "home_switch_role": (
                "shared-control"
                if self._shared_home_axis is axis
                else ("shared-ignored" if self._shared_home_axis is not None else "independent")
            ),
        }

    def _describe_axis_switch(self, axis: AxisRuntime) -> str:
        diagnostics = self._build_axis_diagnostics(axis)
        return "pin=%r active_low=%s pull_up=%s raw=%s" % (
            diagnostics["home_switch_pin"],
            "yes" if diagnostics["home_switch_active_low"] else "no",
            "yes" if diagnostics["home_switch_pull_up"] else "no",
            "high" if diagnostics["home_switch_raw_value"] else "low",
        )
    warnings: list[str] = []
