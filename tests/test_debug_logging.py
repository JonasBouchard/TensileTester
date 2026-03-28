from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

from host.debug_logging import configure_debug_logging, get_app_logger, get_default_log_path


class DebugLoggingTests(unittest.TestCase):
    def test_get_default_log_path_uses_logs_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            base_path = Path(temp_dir)
            self.assertEqual(
                get_default_log_path(base_path),
                base_path / "logs" / "tensile_tester.log",
            )

    def test_configure_debug_logging_writes_messages_to_requested_file(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            log_path = Path(temp_dir) / "debug" / "tester.log"
            configured_path = configure_debug_logging(log_path)

            self.assertEqual(configured_path, log_path.resolve())
            logger = get_app_logger("tests")
            logger.info("Logging smoke test.")

            for handler in get_app_logger().handlers:
                handler.flush()

            contents = log_path.read_text(encoding="utf-8")
            self.assertIn("Logging smoke test.", contents)
            configure_debug_logging(Path(tempfile.gettempdir()) / "tensile_tester_test_cleanup.log")


if __name__ == "__main__":
    unittest.main()
