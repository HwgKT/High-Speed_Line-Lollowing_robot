"""Regression tests for isolated map-specific SPP CSV destinations."""
import csv
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from spp_csv_recorder import TelemetryCsvRecorder


class MapLoggingTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.header = ["host_iso", "t_ms"] + [f"adc{i}" for i in range(16)]
        self.thresholds = [1500] * 16

    def test_each_map_has_separate_csv_with_thresholds(self):
        paths = []
        for map_id in (1, 2, 3):
            rec = TelemetryCsvRecorder(
                self.header, self.thresholds, map_id, logs_root=self.root)
            paths.append(rec.path)
            rec.start()
            rec.stop()
            self.assertEqual(rec.map_id, map_id)
            self.assertEqual(rec.path.parent, self.root / f"map_{map_id:02d}")
            self.assertTrue(rec.path.name.startswith(f"linerobot_map{map_id:02d}_"))
            with rec.path.open(encoding="utf-8-sig", newline="") as source:
                rows = list(csv.reader(source))
            self.assertEqual(rows[0], self.header)
            self.assertEqual(rows[1][0], "CAL_THRESHOLDS")
            self.assertEqual([int(v) for v in rows[1][2:18]], self.thresholds)
        self.assertEqual(len(set(paths)), 3)

    def test_invalid_map_is_rejected(self):
        for value in (None, 0, 4, "1", True):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    TelemetryCsvRecorder(
                        self.header, self.thresholds, value, logs_root=self.root)


if __name__ == "__main__":
    unittest.main()
