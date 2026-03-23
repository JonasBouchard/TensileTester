from __future__ import annotations

from datetime import datetime
from pathlib import Path
import tempfile
import unittest

from host.gui import (
    RunMetadata,
    RunSample,
    build_run_filename,
    export_run_csv,
    sanitize_filename_component,
    validate_specimen_inputs,
)


class GuiHelperTests(unittest.TestCase):
    def test_sanitize_filename_component_removes_unsafe_characters(self) -> None:
        self.assertEqual(sanitize_filename_component(" sample / 01 "), "sample_01")

    def test_build_run_filename_uses_expected_timestamp_format(self) -> None:
        filename = build_run_filename("specimen A", datetime(2026, 3, 23, 14, 15, 16))
        self.assertEqual(filename, "20260323_141516_specimen_A.csv")

    def test_validate_specimen_inputs_requires_sample_id(self) -> None:
        with self.assertRaises(ValueError):
            validate_specimen_inputs("", "10", "2", "25")

    def test_export_run_csv_writes_metadata_header_and_rows(self) -> None:
        metadata = RunMetadata(
            sample_id="sample-001",
            width_mm=10.0,
            thickness_mm=2.0,
            gauge_length_mm=25.0,
            connection_label="Simulator",
            started_at="2026-03-23T14:15:16",
            plot_mode="force_displacement",
        )
        samples = [
            RunSample(
                timestamp_s=0.0,
                force_n=12.5,
                displacement_mm=0.2,
                stress_mpa=0.625,
                strain_percent=0.8,
                state="running",
            ),
            RunSample(
                timestamp_s=0.1,
                force_n=14.0,
                displacement_mm=0.4,
                stress_mpa=0.7,
                strain_percent=1.6,
                state="idle",
            ),
        ]

        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = Path(temp_dir) / "run.csv"
            export_run_csv(output_path, metadata, samples)
            contents = output_path.read_text(encoding="utf-8").splitlines()

        self.assertEqual(contents[0], "# sample_id,sample-001")
        self.assertEqual(contents[1], "# width_mm,10.000000")
        self.assertEqual(
            contents[7],
            "timestamp_s,force_n,displacement_mm,stress_mpa,strain_percent,state",
        )
        self.assertEqual(contents[8], "0.000000,12.500000,0.200000,0.625000,0.800000,running")
        self.assertEqual(contents[9], "0.100000,14.000000,0.400000,0.700000,1.600000,idle")


if __name__ == "__main__":
    unittest.main()
