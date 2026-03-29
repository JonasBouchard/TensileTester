from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

from host.deployment import (
    copy_circuitpython_tree,
    find_circuitpython_drive_candidates,
    is_probable_circuitpython_drive,
    list_circuitpython_source_files,
)


class DeploymentTests(unittest.TestCase):
    def test_list_circuitpython_source_files_returns_sorted_files_and_skips_cache_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            source_dir = Path(temp_dir) / "circuitpython"
            (source_dir / "subdir").mkdir(parents=True)
            (source_dir / "__pycache__").mkdir()
            (source_dir / "code.py").write_text("print('main')\n", encoding="utf-8")
            (source_dir / "subdir" / "config.json").write_text('{"mode": "test"}\n', encoding="utf-8")
            (source_dir / "__pycache__" / "code.cpython-312.pyc").write_bytes(b"junk")

            files = list_circuitpython_source_files(source_dir)

        self.assertEqual(
            [path.relative_to(source_dir).as_posix() for path in files],
            ["code.py", "subdir/config.json"],
        )

    def test_copy_circuitpython_tree_copies_nested_files_and_overwrites_existing_content(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root_dir = Path(temp_dir)
            source_dir = root_dir / "circuitpython"
            destination_dir = root_dir / "CIRCUITPY"
            (source_dir / "lib").mkdir(parents=True)
            destination_dir.mkdir()

            (source_dir / "code.py").write_text("print('updated')\n", encoding="utf-8")
            (source_dir / "lib" / "driver.mpy").write_bytes(b"driver-bytes")
            (destination_dir / "code.py").write_text("print('stale')\n", encoding="utf-8")

            result = copy_circuitpython_tree(source_dir, destination_dir)

            self.assertEqual(result.file_count, 2)
            self.assertEqual(
                [path.as_posix() for path in result.copied_files],
                ["code.py", "lib/driver.mpy"],
            )
            self.assertEqual(
                (destination_dir / "code.py").read_text(encoding="utf-8"),
                "print('updated')\n",
            )
            self.assertEqual(
                (destination_dir / "lib" / "driver.mpy").read_bytes(),
                b"driver-bytes",
            )

    def test_is_probable_circuitpython_drive_accepts_named_mount_or_boot_out_marker(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root_dir = Path(temp_dir)
            named_mount = root_dir / "CIRCUITPY"
            named_mount.mkdir()

            boot_marker_mount = root_dir / "board"
            boot_marker_mount.mkdir()
            (boot_marker_mount / "boot_out.txt").write_text("CircuitPython 9.x\n", encoding="utf-8")

            self.assertTrue(is_probable_circuitpython_drive(named_mount))
            self.assertTrue(is_probable_circuitpython_drive(boot_marker_mount))
            self.assertFalse(is_probable_circuitpython_drive(root_dir / "other"))

    def test_find_circuitpython_drive_candidates_scans_mount_roots(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            search_root = Path(temp_dir) / "media"
            direct_mount = search_root / "CIRCUITPY"
            nested_mount = search_root / "user" / "board"
            direct_mount.mkdir(parents=True)
            nested_mount.mkdir(parents=True)
            (nested_mount / "boot_out.txt").write_text("CircuitPython 9.x\n", encoding="utf-8")

            candidates = find_circuitpython_drive_candidates(
                search_roots=[search_root],
                include_windows_drives=False,
            )

        self.assertEqual(
            [path.name for path in candidates],
            ["CIRCUITPY", "board"],
        )


if __name__ == "__main__":
    unittest.main()
