from __future__ import annotations

import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
import sys
import tempfile
import threading


LOGGER_NAME = "tensile_tester"
DEFAULT_LOG_DIR_NAME = "logs"
DEFAULT_LOG_FILENAME = "tensile_tester.log"
LOG_MAX_BYTES = 1_000_000
LOG_BACKUP_COUNT = 5
_HANDLER_MARKER = "_tensile_tester_debug_handler"
_ORIGINAL_SYS_EXCEPTHOOK = sys.excepthook
_ORIGINAL_THREADING_EXCEPTHOOK = getattr(threading, "excepthook", None)
_EXCEPTION_HOOKS_INSTALLED = False
_CONFIGURED_LOG_PATH: Path | None = None


def get_app_logger(name: str | None = None) -> logging.Logger:
    if not name:
        return logging.getLogger(LOGGER_NAME)

    normalized_name = name
    prefix = f"{LOGGER_NAME}."
    if normalized_name.startswith(prefix):
        normalized_name = normalized_name[len(prefix) :]
    normalized_name = normalized_name.lstrip(".")

    if not normalized_name:
        return logging.getLogger(LOGGER_NAME)
    return logging.getLogger(f"{LOGGER_NAME}.{normalized_name}")


def get_default_log_path(base_dir: str | Path | None = None) -> Path:
    root_path = Path(base_dir) if base_dir is not None else Path(__file__).resolve().parents[1]
    return root_path / DEFAULT_LOG_DIR_NAME / DEFAULT_LOG_FILENAME


def get_configured_log_path() -> Path | None:
    return _CONFIGURED_LOG_PATH


def configure_debug_logging(
    log_path: str | Path | None = None,
    level: int = logging.DEBUG,
) -> Path | None:
    global _CONFIGURED_LOG_PATH

    logger = get_app_logger()
    requested_path = Path(log_path) if log_path is not None else get_default_log_path()
    existing_handler = _find_debug_handler(logger)

    if existing_handler is not None:
        existing_path = Path(existing_handler.baseFilename)
        if _path_key(existing_path) == _path_key(requested_path):
            logger.setLevel(level)
            _CONFIGURED_LOG_PATH = existing_path
            _install_exception_hooks(logger)
            return existing_path

        logger.removeHandler(existing_handler)
        existing_handler.close()

    logger.setLevel(level)
    logger.propagate = False

    for candidate_path in _candidate_log_paths(requested_path):
        try:
            candidate_path.parent.mkdir(parents=True, exist_ok=True)
            handler = RotatingFileHandler(
                candidate_path,
                maxBytes=LOG_MAX_BYTES,
                backupCount=LOG_BACKUP_COUNT,
                encoding="utf-8",
            )
        except OSError:
            continue

        setattr(handler, _HANDLER_MARKER, True)
        handler.setLevel(level)
        handler.setFormatter(
            logging.Formatter(
                "%(asctime)s %(levelname)s [%(threadName)s] %(name)s: %(message)s"
            )
        )
        logger.addHandler(handler)

        _CONFIGURED_LOG_PATH = Path(handler.baseFilename)
        _install_exception_hooks(logger)
        logger.info("Debug logging initialized.")
        logger.info("Writing debug log to %s", _CONFIGURED_LOG_PATH)
        return _CONFIGURED_LOG_PATH

    _CONFIGURED_LOG_PATH = None
    return None


def _candidate_log_paths(primary_path: Path) -> list[Path]:
    candidates = [
        primary_path.expanduser(),
        Path.cwd() / DEFAULT_LOG_DIR_NAME / DEFAULT_LOG_FILENAME,
        Path(tempfile.gettempdir()) / LOGGER_NAME / DEFAULT_LOG_FILENAME,
    ]

    unique_candidates: list[Path] = []
    seen_keys: set[str] = set()
    for candidate in candidates:
        key = _path_key(candidate)
        if key in seen_keys:
            continue
        seen_keys.add(key)
        unique_candidates.append(candidate)

    return unique_candidates


def _find_debug_handler(logger: logging.Logger) -> RotatingFileHandler | None:
    for handler in logger.handlers:
        if getattr(handler, _HANDLER_MARKER, False):
            return handler
    return None


def _install_exception_hooks(logger: logging.Logger) -> None:
    global _EXCEPTION_HOOKS_INSTALLED

    if _EXCEPTION_HOOKS_INSTALLED:
        return

    def handle_exception(
        exc_type: type[BaseException],
        exc_value: BaseException,
        exc_traceback,
    ) -> None:
        if issubclass(exc_type, KeyboardInterrupt):
            _ORIGINAL_SYS_EXCEPTHOOK(exc_type, exc_value, exc_traceback)
            return

        logger.critical(
            "Unhandled exception.",
            exc_info=(exc_type, exc_value, exc_traceback),
        )
        _ORIGINAL_SYS_EXCEPTHOOK(exc_type, exc_value, exc_traceback)

    sys.excepthook = handle_exception

    if _ORIGINAL_THREADING_EXCEPTHOOK is not None:
        def handle_thread_exception(args: threading.ExceptHookArgs) -> None:
            if issubclass(args.exc_type, KeyboardInterrupt):
                _ORIGINAL_THREADING_EXCEPTHOOK(args)
                return

            thread_name = args.thread.name if args.thread is not None else "unknown"
            logger.critical(
                "Unhandled exception in thread %s.",
                thread_name,
                exc_info=(args.exc_type, args.exc_value, args.exc_traceback),
            )
            _ORIGINAL_THREADING_EXCEPTHOOK(args)

        threading.excepthook = handle_thread_exception

    _EXCEPTION_HOOKS_INSTALLED = True


def _path_key(path: Path) -> str:
    try:
        return str(path.expanduser().resolve())
    except OSError:
        return str(path.expanduser())
