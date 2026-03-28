from __future__ import annotations

from pathlib import Path
import sys

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

try:
    from .debug_logging import configure_debug_logging, get_app_logger
except ImportError:
    from debug_logging import configure_debug_logging, get_app_logger


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

LOGGER = get_app_logger("main")


def main() -> None:
    log_path = configure_debug_logging()
    if log_path is not None:
        LOGGER.info(
            "Application starting. cwd=%s python=%s log=%s",
            Path.cwd(),
            sys.version.split()[0],
            log_path,
        )
    else:
        LOGGER.warning(
            "Application starting without a debug log file. cwd=%s python=%s",
            Path.cwd(),
            sys.version.split()[0],
        )

    try:
        launch_gui()
    except Exception:
        LOGGER.exception("Application terminated because of an unhandled exception.")
        raise


if __name__ == "__main__":
    main()
