from __future__ import annotations

import ctypes
from ctypes import wintypes
import os
from dataclasses import dataclass
from pathlib import Path
import shutil
from typing import Iterable


CIRCUITPY_VOLUME_NAME = "CIRCUITPY"
_IGNORED_SOURCE_NAMES = {"__pycache__"}
_IGNORED_SOURCE_SUFFIXES = {".pyc"}


@dataclass(frozen=True)
class DeploymentResult:
    source_dir: Path
    destination_dir: Path
    copied_files: tuple[Path, ...]

    @property
    def file_count(self) -> int:
        return len(self.copied_files)


def get_default_firmware_source_dir(base_dir: str | Path | None = None) -> Path:
    root_dir = Path(base_dir) if base_dir is not None else Path(__file__).resolve().parents[1]
    return root_dir / "circuitpython"


def list_circuitpython_source_files(source_dir: str | Path) -> list[Path]:
    source_root = Path(source_dir)
    if not source_root.exists():
        raise FileNotFoundError(f"CircuitPython source directory was not found: {source_root}")
    if not source_root.is_dir():
        raise NotADirectoryError(f"CircuitPython source path is not a directory: {source_root}")

    files = [
        path
        for path in source_root.rglob("*")
        if path.is_file() and not _should_skip_source_path(path)
    ]
    files.sort(key=lambda path: path.relative_to(source_root).as_posix())

    if not files:
        raise ValueError(f"No deployable files were found in {source_root}.")
    return files


def copy_circuitpython_tree(source_dir: str | Path, destination_dir: str | Path) -> DeploymentResult:
    source_root = Path(source_dir)
    destination_root = Path(destination_dir)
    source_files = list_circuitpython_source_files(source_root)
    destination_root.mkdir(parents=True, exist_ok=True)

    copied_files: list[Path] = []
    for source_path in source_files:
        relative_path = source_path.relative_to(source_root)
        target_path = destination_root / relative_path
        target_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_path, target_path)
        copied_files.append(relative_path)

    return DeploymentResult(
        source_dir=source_root,
        destination_dir=destination_root,
        copied_files=tuple(copied_files),
    )


def is_probable_circuitpython_drive(path: str | Path) -> bool:
    candidate = Path(path)
    if not candidate.exists() or not candidate.is_dir():
        return False

    if candidate.name.upper() == CIRCUITPY_VOLUME_NAME:
        return True

    return (candidate / "boot_out.txt").is_file()


def find_circuitpython_drive_candidates(
    search_roots: Iterable[str | Path] | None = None,
    include_windows_drives: bool = True,
) -> list[Path]:
    candidates: list[Path] = []
    seen_paths: set[str] = set()

    if include_windows_drives:
        for candidate in _iter_windows_drive_candidates():
            _append_unique_candidate(candidates, seen_paths, candidate)

    mount_roots = search_roots if search_roots is not None else _default_mount_search_roots()
    for root in mount_roots:
        for candidate in _iter_mount_search_candidates(Path(root)):
            if is_probable_circuitpython_drive(candidate):
                _append_unique_candidate(candidates, seen_paths, candidate)

    return candidates


def _should_skip_source_path(path: Path) -> bool:
    return any(
        part in _IGNORED_SOURCE_NAMES for part in path.parts
    ) or path.suffix.lower() in _IGNORED_SOURCE_SUFFIXES


def _default_mount_search_roots() -> list[Path]:
    home_dir = Path.home()
    return [
        Path("/Volumes"),
        Path("/media"),
        Path("/run/media"),
        home_dir / "media",
        home_dir / "mnt",
        Path("/mnt"),
    ]


def _iter_mount_search_candidates(root: Path) -> list[Path]:
    if not root.exists() or not root.is_dir():
        return []

    candidates: list[Path] = [root]
    try:
        first_level = sorted(root.iterdir(), key=lambda path: path.name.casefold())
    except OSError:
        return candidates

    candidates.extend(first_level)

    for child in first_level:
        if not child.is_dir():
            continue
        try:
            candidates.extend(sorted(child.iterdir(), key=lambda path: path.name.casefold()))
        except OSError:
            continue

    return [candidate for candidate in candidates if candidate.is_dir()]


def _iter_windows_drive_candidates() -> list[Path]:
    if os.name != "nt":
        return []

    drives_bitmask = ctypes.windll.kernel32.GetLogicalDrives()
    candidates: list[Path] = []
    for index in range(26):
        if not drives_bitmask & (1 << index):
            continue

        drive_root = Path(f"{chr(ord('A') + index)}:/")
        if not drive_root.exists():
            continue

        if _get_windows_volume_label(drive_root).upper() == CIRCUITPY_VOLUME_NAME:
            candidates.append(drive_root)
            continue

        if is_probable_circuitpython_drive(drive_root):
            candidates.append(drive_root)

    return candidates


def _get_windows_volume_label(path: Path) -> str:
    volume_name_buffer = ctypes.create_unicode_buffer(261)
    filesystem_name_buffer = ctypes.create_unicode_buffer(261)
    serial_number = wintypes.DWORD()
    max_component_length = wintypes.DWORD()
    filesystem_flags = wintypes.DWORD()

    success = ctypes.windll.kernel32.GetVolumeInformationW(
        ctypes.c_wchar_p(str(path)),
        volume_name_buffer,
        len(volume_name_buffer),
        ctypes.byref(serial_number),
        ctypes.byref(max_component_length),
        ctypes.byref(filesystem_flags),
        filesystem_name_buffer,
        len(filesystem_name_buffer),
    )
    if not success:
        return ""
    return volume_name_buffer.value


def _append_unique_candidate(candidates: list[Path], seen_paths: set[str], candidate: Path) -> None:
    path_key = _path_key(candidate)
    if path_key in seen_paths:
        return

    seen_paths.add(path_key)
    candidates.append(candidate)


def _path_key(path: Path) -> str:
    try:
        return str(path.expanduser().resolve())
    except OSError:
        return str(path.expanduser())
