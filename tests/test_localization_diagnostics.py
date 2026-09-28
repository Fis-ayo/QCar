"""Local tests for measurement-only localization diagnostics."""

import csv
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "qcar"))

from localization_diagnostics import LocalizationDiagnostics  # noqa: E402


class LocalizationDiagnosticsTests(unittest.TestCase):
    def test_large_innovation_records_csv_and_lidar_event(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            csv_path = temporary_path / "diagnostics.csv"
            event_path = temporary_path / "events"
            diagnostics = LocalizationDiagnostics(
                car_id="test-car",
                csv_path=str(csv_path),
                event_directory=str(event_path),
            )
            diagnostics.update_lidar(
                elapsed=1.0,
                angles=np.linspace(0.0, 1.0, 4),
                distances=np.array([1.0, 2.0, 0.0, 4.0]),
            )
            diagnostics.record_gps_update(
                elapsed=1.1,
                raw_pose=np.array([1.0, 2.0, 0.5]),
                ekf_pre=np.array([0.0, 2.0, 0.0]),
                ekf_post=np.array([0.5, 2.0, 0.25]),
                position_innovation=1.0,
                heading_innovation=0.5,
                motor_speed=0.25,
                steering_command=0.1,
                gyro_z=0.2,
            )
            diagnostics.close()

            with csv_path.open(newline="", encoding="utf-8") as csv_file:
                rows = list(csv.DictReader(csv_file))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["car_id"], "test-car")
            self.assertEqual(rows[0]["event_capture"], "1")
            self.assertEqual(rows[0]["lidar_valid_points"], "3")

            event_files = list(event_path.glob("event_*.npz"))
            self.assertEqual(len(event_files), 1)
            with np.load(event_files[0]) as event:
                self.assertEqual(event["lidar_distances"].shape, (1, 4))
                self.assertAlmostEqual(
                    float(event["position_innovation"][0]),
                    1.0,
                )


if __name__ == "__main__":
    unittest.main()
