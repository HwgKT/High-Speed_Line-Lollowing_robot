"""GPIO34 external START and MAP log synchronization regression checks.

Run: python -m unittest discover -s tests -p 'test_gpio34_start.py'
"""
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from robot_debug import App


class Var:
    def __init__(self, value=None):
        self.value = value
    def set(self, value):
        self.value = value
    def get(self):
        return self.value


class Btn:
    def configure(self, **kwargs):
        self.props = kwargs


class Recorder:
    def __init__(self, path):
        self.path = path


class CsvLifecycle:
    def __init__(self, root):
        self.root = root
        self.active = None
        self.calls = []
    def begin(self, reader, header, calib, thresholds, map_id):
        self.calls.append(map_id)
        folder = self.root / f"map_{map_id:02d}"
        folder.mkdir(exist_ok=True, parents=True)
        self.active = Recorder(folder / f"gpio34_map_{map_id}.csv")
        return self.active


class FakeApp:
    begin_external_run = App.begin_external_run
    select_map = App.select_map
    def __init__(self, root):
        self.run_csv = CsvLifecycle(root)
        self.reader = SimpleNamespace(is_alive=lambda: True)
        self.selected_map = None
        self.map_status = Var()
        self.map_buttons = {i: Btn() for i in (1, 2, 3)}
        self.auto_running = False
        self.start_pending = False
        self.stop_ack_deadline = None
        self.stop_verify_pending = False
        self.calibration_ok = True
        self.cal_min = [800] * 16
        self.cal_max = [2500] * 16
        self.cal_thresholds = [1480] * 16
        self.run_txt_file = None
        self.run_txt_path = None
        self.csv_status = Var()
        self.last_csv_total = 0
        self.start_confirmed = False
        self.commands = []
        self.logs = []
    def send_command(self, cmd):
        self.commands.append(cmd)
        return True
    def log(self, msg):
        self.logs.append(msg)
    def update_control_buttons(self):
        pass


class GPIO34StartTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.ui = FakeApp(Path(temp.name))
        self.addCleanup(self.close_txt)
    def close_txt(self):
        if self.ui.run_txt_file is not None:
            self.ui.run_txt_file.close()

    def test_physical_start_opens_actual_map_csv_without_gui_start(self):
        # Even if the GUI selected a different map, firmware's RUN_ORIGIN
        # always decides the physical START log folder.
        self.ui.selected_map = 1
        self.ui.begin_external_run("RUN_ORIGIN map=3 encL0=200 encR0=220 base=120 peak=220")
        self.assertEqual(self.ui.run_csv.calls, [3])
        self.assertEqual(self.ui.selected_map, 3)
        self.assertEqual(self.ui.run_csv.active.path.parent.name, "map_03")
        self.assertEqual(self.ui.run_txt_path.parent.name, "map_03")
        self.assertTrue(self.ui.run_txt_path.exists())
        self.assertTrue(self.ui.start_confirmed)
        self.assertEqual(self.ui.commands, [])

    def test_gui_start_already_has_log_no_duplicate_external_run(self):
        self.ui.begin_external_run("RUN_ORIGIN map=2 encL0=1 encR0=2")
        self.ui.begin_external_run("RUN_ORIGIN map=2 encL0=1 encR0=2")
        self.assertEqual(self.ui.run_csv.calls, [2])

    def test_map_selection_is_sent_to_firmware_before_gpio_start(self):
        self.ui.select_map(2)
        self.assertEqual(self.ui.commands, ["MAP:2"])
        self.assertEqual(self.ui.selected_map, 2)
        self.assertEqual(self.ui.map_status.get(), "Đã chọn 2")

    def test_map_selection_blocked_while_auto_is_running(self):
        self.ui.auto_running = True
        self.ui.select_map(2)
        self.assertEqual(self.ui.commands, [])
        self.assertIsNone(self.ui.selected_map)

    def test_calibration_missing_does_not_create_mislabeled_csv(self):
        self.ui.calibration_ok = False
        self.ui.begin_external_run("RUN_ORIGIN map=2 encL0=0 encR0=0")
        self.assertIsNone(self.ui.run_csv.active)
        self.assertEqual(self.ui.commands, ["CAL_DUMP"])


if __name__ == "__main__":
    unittest.main()
