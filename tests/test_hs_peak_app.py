"""Headless HS_PEAK UI protocol tests. Run: python -m unittest discover -s tests -p test_hs_peak_app.py

Requires pyserial, but deliberately does not construct a Tk window.
"""
import unittest
from types import SimpleNamespace
from robot_debug import App


class Var:
    def __init__(self, value=None):
        self.value = value
    def get(self):
        return self.value
    def set(self, value):
        self.value = value


class Reader:
    def is_alive(self):
        return True


class FakeApp:
    """Exercise the real App command/reply handlers without a GUI/display."""
    _can_edit_hs_peak = App._can_edit_hs_peak
    _hs_base_limit = App._hs_base_limit

    def __init__(self, base=120):
        self.reader = Reader()
        self.estop_latched = False
        self.estop_pending = False
        self.auto_running = False
        self.start_pending = False
        self.stop_ack_deadline = None
        self.stop_verify_pending = False
        self.run_csv = SimpleNamespace(active=None)
        self.hs_peak_pending = None
        self.hs_peak_pending_at = None
        self.hs_peak_confirmed = 255
        self.hs_peak_dirty = True
        self.hs_peak_var = Var(180)
        self.hs_peak_feedback = Var()
        self.hs_peak_value_text = Var()
        self.high_speed_peak = 255
        self.hs_brake_start = 1800
        self.high_speed_status = Var()
        self.pid_vars = {"Base": Var(str(base))}
        self.messages = []
        self.logs = []
        self.update_count = 0

    def send_command(self, message):
        self.messages.append(message)
        return True

    def update_control_buttons(self):
        self.update_count += 1

    def log(self, message):
        self.logs.append(message)


class PeakUiTest(unittest.TestCase):
    def test_apply_waits_for_firmware_ack(self):
        ui = FakeApp()
        self.assertTrue(App.apply_hs_peak(ui))
        self.assertEqual(ui.messages, ["HS_PEAK:180"])
        self.assertEqual(ui.hs_peak_confirmed, 255)  # Not optimistic.
        self.assertEqual(ui.hs_peak_pending, 180)
        App.on_text(ui, "HS_PEAK: PWM=180 BASE=120 RAMP_END=300 BRAKE_START=1800 BRAKE_END=2300 HOLD_PWM=110 HOLD_MIN_END=3100")
        self.assertEqual(ui.hs_peak_confirmed, 180)
        self.assertIsNone(ui.hs_peak_pending)
        self.assertEqual(ui.hs_peak_var.get(), 180)
        self.assertFalse(ui.hs_peak_dirty)

    def test_prevents_peak_lower_than_base(self):
        ui = FakeApp(base=200)
        self.assertFalse(App.apply_hs_peak(ui))
        self.assertEqual(ui.messages, [])
        self.assertIn("BASE 200", ui.logs[-1])

    def test_no_hs_change_while_auto(self):
        ui = FakeApp()
        ui.auto_running = True
        self.assertFalse(App.apply_hs_peak(ui))
        self.assertEqual(ui.messages, [])

    def test_error_clears_pending_and_queries_firmware(self):
        ui = FakeApp()
        self.assertTrue(App.apply_hs_peak(ui))
        App.on_text(ui, "ERR: HS_PEAK_RANGE_100_TO_255_AND_GE_BASE")
        self.assertIsNone(ui.hs_peak_pending)
        self.assertEqual(ui.messages[-1], "HS_PEAK")
        self.assertIn("từ chối", ui.hs_peak_feedback.get())

    def test_readback_does_not_replace_unapplied_slider(self):
        ui = FakeApp()
        ui.hs_peak_var.set(220)
        App.on_text(ui, "HS_PEAK: PWM=180 BASE=120 RAMP_END=300 BRAKE_START=1800 BRAKE_END=2300 HOLD_PWM=110 HOLD_MIN_END=3100")
        self.assertEqual(ui.hs_peak_confirmed, 180)
        self.assertEqual(ui.hs_peak_var.get(), 220)
        self.assertTrue(ui.hs_peak_dirty)

    def test_base_rejection_does_not_claim_peak_success(self):
        ui = FakeApp()
        App.on_text(ui, "ERR: BASE_GT_HS_PEAK")
        self.assertIn("BASE vượt HS_PEAK", ui.hs_peak_feedback.get())
        self.assertEqual(ui.messages[-1], "PID")

    def test_read_refreshes_firmware_value(self):
        ui = FakeApp()
        self.assertTrue(App.read_hs_peak(ui))
        self.assertIsNone(ui.hs_peak_confirmed)
        self.assertFalse(ui.hs_peak_dirty)
        self.assertEqual(ui.messages[-1], "HS_PEAK")
        App.on_text(ui, "HS_PEAK: PWM=220 BASE=120 RAMP_END=300 BRAKE_START=1800 BRAKE_END=2300 HOLD_PWM=110 HOLD_MIN_END=3100")
        self.assertEqual(ui.hs_peak_confirmed, 220)
        self.assertEqual(ui.hs_peak_var.get(), 220)

    def test_dynamic_brake_start_on_229_230_and_239_240_boundaries(self):
        ui = FakeApp()
        for peak, expected_start in (
            (229, 1800), (230, 1600), (239, 1600), (240, 1400), (255, 1400)
        ):
            App.on_text(
                ui,
                f"HS_PEAK: PWM={peak} BASE=120 RAMP_END=300 "
                f"BRAKE_START={expected_start} BRAKE_END=2300 "
                "HOLD_PWM=110 HOLD_MIN_END=3100",
            )
            self.assertEqual(ui.hs_brake_start, expected_start)
            self.assertIn(
                f"BRAKE={expected_start}-2300", ui.high_speed_status.get()
            )

    def test_stage_event_reflects_actual_early_brake(self):
        ui = FakeApp()
        App.on_text(
            ui,
            "HS_STAGE BRAKE map=3 runEnc=1400 base=120 peak=255 "
            "brake=1400 end=2300 hold=110 minEnd=3100 t_ms=123456",
        )
        self.assertEqual(ui.hs_brake_start, 1400)
        self.assertIn("BRAKE=1400-2300", ui.high_speed_status.get())

    def test_hold_and_complete_stage_messages(self):
        ui = FakeApp()
        App.on_text(
            ui,
            "HS_STAGE HOLD map=2 runEnc=2300 base=120 peak=220 brake=1800 end=2300 hold=110 minEnd=3100",
        )
        self.assertIn("HS: HOLD", ui.high_speed_status.get())
        self.assertIn("HOLD=110", ui.high_speed_status.get())
        App.on_text(
            ui,
            "DIAG: bt=1 auto=1 loop=620.0Hz pid=620.0Hz sent=100.0Hz hs=6 runEnc=2650 peak=220",
        )
        self.assertIn("HS: HOLD", ui.high_speed_status.get())
        App.on_text(
            ui,
            "HS_STAGE COMPLETE map=2 runEnc=3200 base=120 peak=220 brake=1800 end=2300 hold=110 minEnd=3100",
        )
        self.assertIn("HS: COMPLETE", ui.high_speed_status.get())

    def test_base_apply_rejects_larger_than_confirmed_peak(self):
        ui = FakeApp()
        ui.hs_peak_confirmed = 180
        ui.pwm_var = Var(200)
        App.apply_pwm(ui)
        self.assertEqual(ui.messages, [])
        self.assertIn("BASE 200 vượt HS_PEAK 180", ui.logs[-1])


if __name__ == "__main__":
    unittest.main()
