from __future__ import annotations

try:
    from .controller import (
        DEFAULT_BAUD_RATE,
        SpecimenDimensions,
        TesterController,
        TesterEvent,
        compute_derived_values,
        list_serial_ports,
        parse_device_message,
        validate_specimen_dimensions,
    )
    from .gui import TensileTesterApp, launch_gui
except ImportError:
    from controller import (
        DEFAULT_BAUD_RATE,
        SpecimenDimensions,
        TesterController,
        TesterEvent,
        compute_derived_values,
        list_serial_ports,
        parse_device_message,
        validate_specimen_dimensions,
    )
    from gui import TensileTesterApp, launch_gui


__all__ = [
    "DEFAULT_BAUD_RATE",
    "SpecimenDimensions",
    "TensileTesterApp",
    "TesterController",
    "TesterEvent",
    "compute_derived_values",
    "list_serial_ports",
    "main",
    "parse_device_message",
    "validate_specimen_dimensions",
]


def main() -> None:
    launch_gui()


if __name__ == "__main__":
    main()
