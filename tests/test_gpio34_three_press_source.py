"""Regression checks for the physical GPIO34 WHITE/BLACK/START workflow.

These are source-level invariants; they do not replace an ESP32 build or a
hardware-in-the-loop test. Run with:
python -m unittest discover -s tests -p 'test_gpio34_three_press_source.py'
"""
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
SENSOR = (ROOT / "src" / "sensor.cpp").read_text(encoding="utf-8")
PINOUT = (ROOT / "include" / "pinout.h").read_text(encoding="utf-8")
PID = (ROOT / "src" / "pid_controller.cpp").read_text(encoding="utf-8")


class GPIO34ThreePressSourceTests(unittest.TestCase):
    def test_gpio34_active_low_and_debounced(self):
        self.assertRegex(PINOUT, r"#define PIN_START_BUTTON\s+34")
        self.assertIn("pressed = buttonState == LOW", SENSOR)
        self.assertIn("BUTTON_DEBOUNCE_MS", SENSOR)

    def test_three_phases_and_boot_resets_to_white(self):
        self.assertIn("button34Phase=BUTTON34_WHITE_NEXT;", MAIN)
        self.assertIn("button34Phase==BUTTON34_WHITE_NEXT", MAIN)
        self.assertIn("button34Phase==BUTTON34_BLACK_NEXT", MAIN)
        self.assertIn("button34Phase==BUTTON34_BLACK_CAPTURING", MAIN)
        self.assertIn("BUTTON34_START_NEXT", MAIN)
        self.assertIn("BUTTON34_START_REUSE", MAIN)
        self.assertLess(MAIN.index('reply("BUTTON34: CAL_WHITE_REQUEST")'),
                        MAIN.index('reply("BUTTON34: CAL_BLACK_REQUEST")'))
        self.assertLess(MAIN.index('reply("BUTTON34: CAL_BLACK_REQUEST")'),
                        MAIN.index('reply("BUTTON34: START_REQUEST")'))

    def test_waits_until_white_and_black_complete(self):
        self.assertIn("sensor_calibration_state()!=CAL_WHITE_READY", MAIN)
        self.assertIn("ERR: BUTTON34_WAIT_CAL_WHITE_DONE", MAIN)
        self.assertIn("ERR: BUTTON34_WAIT_CAL_BLACK_DONE", MAIN)
        self.assertIn("case CAL_SAVED:", MAIN)
        self.assertIn("setButton34Phase(BUTTON34_START_NEXT);", MAIN)
        self.assertIn("case CAL_ERROR:", MAIN)
        self.assertIn("setButton34Phase(BUTTON34_WHITE_NEXT);", MAIN)

    def test_led_fast_then_slow_then_bt(self):
        self.assertIn("CAL_WHITE_LED_BLINK_MS=40", MAIN)
        self.assertIn("CAL_BLACK_LED_BLINK_MS=90", MAIN)
        self.assertIn("void updateLedIndicator(uint32_t now)", MAIN)
        self.assertIn("updateLedIndicator(now);", MAIN)
        self.assertIn("BUTTON34_WHITE_CAPTURING", MAIN)
        self.assertIn("setButton34Phase(BUTTON34_BLACK_NEXT); // LED OFF", MAIN)
        self.assertIn("setButton34Phase(BUTTON34_START_NEXT);", MAIN)
        self.assertIn("if(button34Phase!=BUTTON34_START_REUSE)", MAIN)
        self.assertIn("ledWrite(phase==BUTTON34_WHITE_CAPTURING ||", MAIN)
        self.assertIn("setButton34Phase(BUTTON34_START_REUSE)", MAIN)

    def test_start_guard_and_finish_detection_unchanged(self):
        self.assertIn("BUTTON34_START_CENTER_MASK=0x07E0U", MAIN)
        self.assertIn("(bits & BUTTON34_START_CENTER_MASK)==0U", MAIN)
        self.assertIn("bits==0xFFFFU", MAIN)
        self.assertIn('command("AUTO");', MAIN)
        self.assertIn("FINISH_CONFIRM_FRAMES = 3", PID)
        self.assertIn("FULL_LINE_MIN_SENSORS = N_SENSORS", PID)
        self.assertIn("HS_CORNER_HOLD_PWM = 110.0f", PID)
        self.assertIn("HS_VERY_EARLY_BRAKE_START_ENC = 1400", PID)


if __name__ == "__main__":
    unittest.main()
