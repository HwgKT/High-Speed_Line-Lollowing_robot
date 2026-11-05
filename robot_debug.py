#!/usr/bin/env python3
"""ESP32 Line Robot Windows dashboard over Bluetooth Classic SPP.

Install: pip install pyserial
Run:     python robot_debug.py

First pair the ESP32 in Windows and select its SPP COM port.
mainv4 firmware now starts Classic SPP as LineRobot and sends 72-byte frames.
Use its outgoing Windows Bluetooth COM port, separate from the USB serial COM port.

The 69-byte SPP payload includes raw ADC. The GUI requests exact 16-channel
white/black references from the ESP32 NVS profile through CAL_DUMP, then
normalizes telemetry to 0..1000 using the same reference levels.
"""
import collections
import csv
import datetime
import math
import operator
import functools
import queue
import re
import shutil
import tempfile
import struct
import threading
import time
import tkinter as tk
from tkinter import filedialog, ttk
from pathlib import Path

import serial
from serial.tools import list_ports
from run_csv_lifecycle import RunCsvLifecycle

HDR = b"\xAA\x55"
FMT = "<I16H2i2h5fB"
PAYLOAD_LEN = struct.calcsize(FMT)  # 69
FRAME_LEN = len(HDR) + PAYLOAD_LEN + 1  # 72
ADC_MAX, PWM_MAX, HIST_LEN = 4095, 255, 300
MIN_CALIB_DELTA = 150
BLACK_THRESHOLD_CAL = 400  # 40% of each eye's calibrated W->B contrast (not RAW)
def eye_raw_threshold(white, black):
    """Exact RAW cutoff for firmware's normalized CAL>=400 rule."""
    return white + ((black - white) * BLACK_THRESHOLD_CAL + 999) // 1000
CSV_HEADER = (["host_iso", "t_ms"] +
              [f"adc{i}" for i in range(16)] +
              [f"cal{i}" for i in range(16)] +
              [f"logic{i}" for i in range(16)] +
              ["encL", "encR", "pwmL", "pwmR", "err", "p", "i", "d", "out", "mode", "logic16"])
BG, PANEL, FG, MUTED = "#0f1420", "#161d31", "#e2e8f0", "#94a3b8"
GREEN, GREY, AMBER, CYAN, ROSE = "#10b981", "#475569", "#f59e0b", "#22d3ee", "#f43f5e"


def xor_sum(data):
    return functools.reduce(operator.xor, data, 0)


def normalize_adc(raw, low, high, black_is_high=True):
    """Calibrate raw ADC to 0..1000; calibrated 1000 always means black."""
    if high - low < MIN_CALIB_DELTA:
        return 0
    raw = max(low, min(high, raw))
    if black_is_high:
        return (raw - low) * 1000 // (high - low)
    return (high - raw) * 1000 // (high - low)


class Reader(threading.Thread):
    """Read a mixed SPP stream of framed binary samples and text replies."""

    def __init__(self, port, out_queue, recorder=None, control_queue=None):
        super().__init__(daemon=True)
        # Never let a stalled Windows SPP COM write freeze the Tk UI.
        self.ser = serial.Serial(port, 115200, timeout=0.05, write_timeout=0.5)
        self.out_queue = out_queue
        self.recorder = recorder
        self.control_queue = control_queue if control_queue is not None else out_queue
        self.running = True
        self.buf = bytearray()
        self.text_buf = bytearray()
        self.bad = 0
        self.frames_rx = 0
        self.frames_dropped = 0

    def run(self):
        while self.running:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except (serial.SerialException, OSError):
                self.control_queue.put(("error", "Bluetooth/COM đã ngắt kết nối"))
                break
            if chunk:
                self.buf.extend(chunk)
                self._parse()
        try:
            self.ser.close()
        except Exception:
            pass

    def _text_bytes(self, chunk):
        for b in chunk:
            if b in (10, 13):
                if self.text_buf:
                    line = self.text_buf.decode("utf-8", "replace").strip()
                    if line:
                        self.control_queue.put(("text", line))
                    self.text_buf.clear()
            elif 32 <= b <= 126 or b >= 128:
                if len(self.text_buf) < 512:
                    self.text_buf.append(b)
                else:
                    self.text_buf.clear()

    def _parse(self):
        while self.buf:
            offset = self.buf.find(HDR)
            if offset < 0:
                # A leading 0xAA may be split across read() calls.
                keep = 1 if self.buf[-1] == 0xAA else 0
                self._text_bytes(self.buf[:-1] if keep else self.buf)
                self.buf = bytearray(b"\xAA") if keep else bytearray()
                return
            if offset:
                self._text_bytes(self.buf[:offset])
                del self.buf[:offset]
            if len(self.buf) < FRAME_LEN:
                return
            data = bytes(self.buf[2:2 + PAYLOAD_LEN])
            if xor_sum(data) == self.buf[2 + PAYLOAD_LEN]:
                self.frames_rx += 1
                sample = struct.unpack(FMT, data)
                # Logging precedes GUI delivery; Tk slowdown cannot throttle disk writes.
                if self.recorder is not None:
                    self.recorder.submit(sample)
                try:
                    self.out_queue.put_nowait(("data", sample))
                except queue.Full:
                    self.frames_dropped += 1
                del self.buf[:FRAME_LEN]
            else:
                self.bad += 1
                del self.buf[:1]

    def send(self, text):
        data = (text + "\n").encode("utf-8")
        if self.ser.write(data) != len(data):
            raise serial.SerialTimeoutException("SPP command was only partially written")


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ESP32 Line Robot — Bluetooth Classic SPP")
        self.geometry("1440x900")
        self.minsize(1080, 720)
        self.configure(bg=BG)
        self.q = queue.Queue(maxsize=4096)  # bounded GUI data queue
        self.controls_q = queue.SimpleQueue()  # priority text replies / E-STOP ACK
        self.reader = None
        self.run_csv = RunCsvLifecycle(self.controls_q)
        self.stop_ack_deadline = None
        self.stop_verify_pending = False
        self.stop_verify_deadline = None
        self.stop_verify_retries = 0
        self.start_confirmed = False
        self.csv_status = tk.StringVar(value="CSV: chờ START")
        self.selected_map = None  # Require explicit choice to avoid mislabeled experiments.
        self.map_status = tk.StringVar(value="Chưa chọn")
        self.map_buttons = {}
        self.run_txt_file = None
        self.run_txt_path = None
        self.session_log_file = tempfile.TemporaryFile(mode="w+t", encoding="utf-8")
        self.hist = collections.deque(maxlen=HIST_LEN)
        self.recent_rows = collections.deque(maxlen=15000)  # UI cache only, ~150 sec at 100 Hz
        self.log_lines = collections.deque(maxlen=30000)  # display cache; TXT export uses disk
        self.log_window = None
        self.log_mirror = None
        self.sensor_window = None
        self.sensor_table = None
        self.preview_pid_log = tk.BooleanVar(value=True)
        self.last_preview_ms = None
        self.stream_rate = tk.StringVar(value="100")
        self.prev = None
        self.wheel_speed = [0.0, 0.0]
        self.frames = 0
        self.last_rx_total = 0
        self.last_csv_total = 0
        self.rate_time = time.monotonic()
        # Report actual firmware loop/PID rates using periodic SPP DIAG.
        self.diag_interval_s = 2.0
        self.next_diag_poll = None
        self.last_diag_rx = None
        self.loop_debug = tk.StringVar(
            value="LOOP: -- Hz    PID: -- Hz    SPP TX: -- Hz    (chưa có DIAG)")
        self.high_speed_peak = 220  # Expected startup default; read back from firmware.
        self.hs_brake_start = 1800  # Read back actual threshold for Peak >= 230.
        self.hs_peak_confirmed = None
        self.hs_peak_pending = None
        self.hs_peak_pending_at = None
        self.hs_peak_dirty = False
        self.hs_peak_var = tk.DoubleVar(value=220)
        self.hs_peak_value_text = tk.StringVar(value="220 (chưa xác nhận)")
        self.hs_peak_feedback = tk.StringVar(value="Đọc HS từ ESP32 trước khi START")
        self.high_speed_status = tk.StringVar(
            value="HS: -- | BRAKE 1800 (<230), 1600 (230–239), 1400 (240–255) → 2300 | HOLD 110")
        self.values = {}
        self.last_adc = [0] * 16
        # Display only firmware calibration. Keep last valid NVS profile on failures.
        self.calibration_ok = False
        self.black_high = True
        self.cal_min = [0] * 16
        self.cal_max = [4095] * 16
        self.cal_thresholds = [None] * 16
        self.cal_data = {}
        self.estop_latched = None  # Unknown until firmware STATUS.
        self.estop_pending = False
        self.start_pending = False
        self.auto_running = False
        self.run_elapsed_seconds = 0.0
        self.run_started_at = None
        self.run_timer_var = tk.StringVar(value="00:00:00")
        self._build()
        self.update_control_buttons()  # Includes HS widgets created after START/STOP.
        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.bind_all("<Escape>", lambda _event: self.stop_line())
        self.after(30, self.tick)

    def label(self, parent, text="", fg=FG, bg=None, **kw):
        return tk.Label(parent, text=text, fg=fg, bg=BG if bg is None else bg, **kw)

    def _build(self):
        top = tk.Frame(self, bg=BG)
        top.pack(fill="x", padx=8, pady=(4, 2))
        self.label(top, "Bluetooth SPP", font=("Segoe UI", 10, "bold")).pack(side="left", padx=(0, 6))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=11)
        self.port_combo.pack(side="left", padx=3)
        ttk.Button(top, text="Quét COM", command=self.refresh_ports).pack(side="left", padx=3)
        self.connect_btn = ttk.Button(top, text="Kết nối", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=3)
        self.label(top, "CSV: tự động START/STOP", fg=GREEN,
                   font=("Segoe UI", 9)).pack(side="left", padx=4)
        self.label(top, "SPP Hz", fg=MUTED).pack(side="left", padx=(4, 2))
        self.rate_combo = ttk.Combobox(top, textvariable=self.stream_rate,
                                       values=("300", "200", "100", "50", "20", "0"),
                                       width=5, state="readonly")
        self.rate_combo.pack(side="left")
        self.rate_combo.bind("<<ComboboxSelected>>",
                             lambda _e: self.send_command("STREAM:" + self.stream_rate.get()))
        self.status_label = self.label(top, "Chưa kết nối", fg=ROSE)
        self.status_label.pack(side="left", padx=5)
        ttk.Button(top, text="Đo TX/Loop", command=self.request_diagnostics).pack(side="left", padx=3)
        self.rate_label = self.label(top, "RX 0 Hz | APP 0 Hz | CRC 0", fg=MUTED)
        self.rate_label.pack(side="right")

        diag_bar = tk.Frame(self, bg=PANEL)
        diag_bar.pack(fill="x", padx=8, pady=(0, 2))
        map_bar = tk.Frame(diag_bar, bg=PANEL)
        map_bar.pack(side="right", padx=6, pady=1)
        self.label(map_bar, "Chọn MAP:", fg=MUTED, bg=PANEL,
                   font=("Segoe UI", 9, "bold")).pack(side="left", padx=(0, 3))
        for map_id in (1, 2, 3):
            button = tk.Button(map_bar, text=f"MAP {map_id}", width=6,
                               command=lambda selected=map_id: self.select_map(selected),
                               bg=GREY, fg=FG, relief="flat", height=1,
                               font=("Segoe UI", 9, "bold"))
            button.pack(side="left", padx=2)
            self.map_buttons[map_id] = button
        self.label(map_bar, textvariable=self.map_status, fg=AMBER, bg=PANEL,
                   font=("Consolas", 8)).pack(side="left", padx=(5, 0))
        self.label(diag_bar, textvariable=self.loop_debug, fg=CYAN, bg=PANEL,
                   font=("Consolas", 9, "bold")).pack(anchor="w", padx=8, pady=(1, 0))
        self.label(diag_bar, textvariable=self.high_speed_status, fg=AMBER, bg=PANEL,
                   font=("Consolas", 8)).pack(anchor="w", padx=8, pady=(0, 1))
        self.label(diag_bar, textvariable=self.csv_status, fg=GREEN, bg=PANEL,
                   font=("Consolas", 8)).pack(anchor="w", padx=8, pady=(0, 1))

        # Calibration is a prerequisite for START. Keep its controls visible at
        # minimum window size rather than below the non-scrollable HS_PEAK panel.
        cal_bar = tk.Frame(self, bg=PANEL, padx=8, pady=3)
        cal_bar.pack(fill="x", padx=8, pady=(0, 2))
        self.label(cal_bar, "Calibration — 16 mắt / NVS", bg=PANEL,
                   font=("Segoe UI", 9, "bold")).pack(side="left", padx=(0, 8))
        self.cal_white_btn = ttk.Button(
            cal_bar, text="1. Lấy TRẮNG", command=self.start_calibration)
        self.cal_white_btn.pack(side="left", padx=(0, 4))
        self.cal_black_btn = ttk.Button(
            cal_bar, text="2. Lấy ĐEN", command=self.stop_calibration)
        self.cal_black_btn.pack(side="left", padx=(0, 8))
        ttk.Button(cal_bar, text="CAL Status",
                   command=lambda: self.send_command("CAL_STATUS")).pack(
                       side="left", padx=(0, 4))
        ttk.Button(cal_bar, text="Đọc NVS 16 mắt",
                   command=lambda: self.send_command("CAL_DUMP")).pack(
                       side="left", padx=(0, 8))
        self.cal_state = tk.StringVar(
            value="Chưa tải profile calibration từ ESP32")
        self.label(cal_bar, textvariable=self.cal_state, bg=PANEL, fg=AMBER,
                   font=("Segoe UI", 8), justify="left", anchor="w",
                   wraplength=290).pack(side="left", fill="x", expand=True)

        # Balanced 50/50 columns; bottom-right quarter reserved for SPP console.
        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=8, pady=(0, 5))
        body.grid_columnconfigure(0, weight=1, uniform="dashboard", minsize=490)
        body.grid_columnconfigure(1, weight=1, uniform="dashboard", minsize=490)
        body.grid_rowconfigure(0, weight=1)
        left = tk.Frame(body, bg=BG)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        right = tk.Frame(body, bg=PANEL)
        right.grid(row=0, column=1, sticky="nsew", padx=(5, 0))
        right.grid_columnconfigure(0, weight=1)
        # Give compact controls enough space, without clipping at 720p/HiDPI.
        right.grid_rowconfigure(0, weight=56, minsize=275)
        right.grid_rowconfigure(1, weight=44, minsize=170)

        # Upper-right quarter: compact fixed controls, no scrolling.
        # Lower-right quarter remains a full-height SPP terminal.
        panel = tk.Frame(right, bg=PANEL, padx=6, pady=3)
        panel.grid(row=0, column=0, sticky="nsew")

        terminal_panel = tk.Frame(right, bg=PANEL, padx=8, pady=5)
        terminal_panel.grid(row=1, column=0, sticky="nsew")

        control = tk.Frame(panel, bg=PANEL)
        control.pack(fill="x", pady=(0, 2))
        self.start_btn = tk.Button(control, text="BẮT ĐẦU DÒ LINE",
                                   command=self.start_line, font=("Segoe UI", 9, "bold"),
                                   bg=GREEN, fg="#ffffff", activebackground="#059669",
                                   relief="flat", height=1, pady=0)
        self.start_btn.pack(side="left", fill="x", expand=True, padx=(0, 5))
        self.stop_btn = tk.Button(control, text="DỪNG",
                                  command=self.stop_line, font=("Segoe UI", 9, "bold"),
                                  bg=ROSE, fg="#ffffff", activebackground="#be123c",
                                  relief="flat", height=1, pady=1)
        self.stop_btn.pack(side="left", fill="x", expand=True)
        run_timer = tk.Frame(panel, bg=PANEL)
        run_timer.pack(fill="x", pady=(1, 2))
        self.label(run_timer, "Thời gian xe chạy", bg=PANEL, fg=MUTED,
                   font=("Segoe UI", 9)).pack(side="left")
        self.label(run_timer, textvariable=self.run_timer_var, bg=PANEL, fg=CYAN,
                   font=("Consolas", 10, "bold")).pack(side="left", padx=5)
        ttk.Button(run_timer, text="Reset giờ", width=9,
                   command=self.reset_run_timer).pack(side="right")
        self.control_state = tk.StringVar(value="Chưa kết nối / chưa xác định trạng thái")
        self.control_state_label = self.label(panel, textvariable=self.control_state,
                                              bg=PANEL, fg=AMBER, wraplength=430,
                                              font=("Segoe UI", 9),
                                              anchor="w", justify="left")
        self.control_state_label.pack(fill="x", pady=0)
        self.adc_status = tk.StringVar(value="ADC: chờ STATUS")
        self.label(panel, textvariable=self.adc_status, bg=PANEL, fg=CYAN,
                   font=("Consolas", 8)).pack(anchor="w", pady=(0, 1))
        self.update_control_buttons()

        sensor_head = tk.Frame(left, bg=BG)
        sensor_head.pack(fill="x")
        self.label(sensor_head, "16 mắt: RAW / T_i / CAL / 0-1", font=("Segoe UI", 11, "bold")).pack(side="left")
        ttk.Button(sensor_head, text="Bảng 16 mắt", command=self.open_sensor_table).pack(side="right", padx=4)
        self.label(sensor_head, "Mỗi mắt có ngưỡng riêng", fg=AMBER).pack(side="right", padx=6)
        self.bars = tk.Canvas(left, bg=PANEL, width=570, height=282, highlightthickness=0)
        self.bars.pack(fill="x", pady=5)
        self.logic_var = tk.StringVar(value="0 = TRẮNG | 1 = ĐEN | Chưa có NVS: ----------------")
        self.label(left, textvariable=self.logic_var, fg=CYAN,
                   font=("Consolas", 10, "bold")).pack(anchor="w", pady=(0, 8))

        metrics = tk.Frame(left, bg=PANEL)
        metrics.pack(fill="x", pady=4)
        names = [("encL", "Encoder L"), ("encR", "Encoder R"),
                 ("spdL", "Tốc độ L (tick/s)"), ("spdR", "Tốc độ R (tick/s)"),
                 ("pwmL", "PWM L"), ("pwmR", "PWM R"),
                 ("err", "Error"), ("out", "PID out"),
                 ("p", "P"), ("i", "I"), ("d", "D"), ("mode", "Chế độ")]
        for n, (key, name) in enumerate(names):
            r, c = divmod(n, 3)
            self.label(metrics, name, fg=MUTED, bg=PANEL, width=12,
                       anchor="e").grid(row=r, column=c * 2, padx=(2, 1), pady=4)
            self.values[key] = tk.StringVar(value="—")
            self.label(metrics, textvariable=self.values[key], bg=PANEL, fg=CYAN,
                       font=("Consolas", 9, "bold"), width=8,
                       anchor="w").grid(row=r, column=c * 2 + 1, padx=(1, 2))

        self.label(left, "Scope: PWM trái / PWM phải / PID out", fg=MUTED).pack(anchor="w", pady=(11, 3))
        self.scope = tk.Canvas(left, bg=PANEL, width=570, height=165, highlightthickness=0)
        self.scope.pack(fill="x")
        encoder_actions = tk.Frame(left, bg=BG)
        encoder_actions.pack(fill="x", pady=5)
        ttk.Button(encoder_actions, text="Reset ENC",
                   command=lambda: self.send_command("ENC_RESET")).pack(side="left", padx=4)
        self.label(left, "RAW: 0–4095  |  T_i=W_i+40%×(B_i−W_i)  |  CAL: 0–1000  |  0=TRẮNG, 1=ĐEN",
                   fg=MUTED).pack(anchor="w")

        # Compact one-line PWM and PID controls keep HS buttons visible.
        pwm_row = tk.Frame(panel, bg=PANEL)
        pwm_row.pack(fill="x", pady=(1, 0))
        self.label(pwm_row, "PWM", bg=PANEL,
                   font=("Segoe UI", 9, "bold")).pack(side="left", padx=(0, 5))
        self.pwm_var = tk.DoubleVar(value=120)
        self.pwm_text = tk.StringVar(value="120 / 255")
        self.label(pwm_row, textvariable=self.pwm_text, bg=PANEL, fg=CYAN,
                   font=("Consolas", 9), width=9, anchor="e").pack(side="right")
        self.pwm_slider = ttk.Scale(
            pwm_row, from_=0, to=255, variable=self.pwm_var,
            command=self.preview_pwm)
        self.pwm_slider.pack(side="left", fill="x", expand=True, padx=(4, 6))
        self.pwm_slider.bind("<ButtonRelease-1>", lambda _e: self.apply_pwm())
        self.pwm_slider.bind("<KeyRelease>", lambda _e: self.apply_pwm())
        speed_buttons = tk.Frame(panel, bg=PANEL)
        speed_buttons.pack(fill="x", pady=(0, 1))
        self.label(speed_buttons, "Mức PWM:", bg=PANEL, fg=MUTED,
                   font=("Segoe UI", 8)).pack(side="left", padx=(0, 4))
        for val in (60, 120, 180, 255):
            ttk.Button(speed_buttons, text=str(val), width=4,
                       command=lambda v=val: self.preset_pwm(v)).pack(
                           side="left", expand=True, padx=1)

        pid_row = tk.Frame(panel, bg=PANEL)
        pid_row.pack(fill="x", pady=(1, 2))
        self.label(pid_row, "PID", bg=PANEL,
                   font=("Segoe UI", 9, "bold")).pack(side="left", padx=(0, 4))
        self.pid_vars = {}
        ttk.Button(pid_row, text="Đọc", width=5,
                   command=lambda: self.send_command("PID")).pack(
                       side="right", padx=(2, 0))
        ttk.Button(pid_row, text="Áp dụng", width=7,
                   command=self.apply_pid).pack(side="right", padx=(2, 0))
        for key, initial in (("Kp", "1.6"), ("Ki", "0"),
                             ("Kd", "0.8"), ("Base", "120")):
            field = tk.Frame(pid_row, bg=PANEL)
            field.pack(side="left", padx=(0, 4))
            self.label(field, key, bg=PANEL, fg=MUTED,
                       font=("Segoe UI", 8)).pack(side="left", padx=(0, 1))
            var = tk.StringVar(value=initial)
            self.pid_vars[key] = var
            ttk.Entry(field, textvariable=var, width=5).pack(side="left")

        # HS_PEAK changes the first straight only, independently of Base.
        hs_row = tk.Frame(panel, bg=PANEL)
        hs_row.pack(fill="x", pady=(1, 0))
        self.label(hs_row, "HS_PEAK", bg=PANEL,
                   font=("Segoe UI", 9, "bold")).pack(side="left", padx=(0, 5))
        self.label(hs_row, textvariable=self.hs_peak_value_text,
                   bg=PANEL, fg=CYAN, font=("Consolas", 9),
                   anchor="e").pack(side="right")
        self.hs_peak_slider = ttk.Scale(
            hs_row, from_=100, to=255, variable=self.hs_peak_var,
            command=self.preview_hs_peak)
        self.hs_peak_slider.pack(side="left", fill="x", expand=True, padx=(4, 6))
        hs_actions = tk.Frame(panel, bg=PANEL)
        hs_actions.pack(fill="x", pady=(0, 1))
        self.hs_peak_widgets = [self.hs_peak_slider]
        for val in (180, 220, 255):
            button = ttk.Button(hs_actions, text=str(val), width=4,
                                command=lambda v=val: self.preset_hs_peak(v))
            button.pack(side="left", expand=True, padx=1)
            self.hs_peak_widgets.append(button)
        self.hs_peak_apply_btn = ttk.Button(
            hs_actions, text="Áp dụng HS", width=10, command=self.apply_hs_peak)
        self.hs_peak_apply_btn.pack(side="left", expand=True, padx=1)
        self.hs_peak_read_btn = ttk.Button(
            hs_actions, text="Đọc HS", width=7, command=self.read_hs_peak)
        self.hs_peak_read_btn.pack(side="left", expand=True, padx=1)
        self.label(panel, textvariable=self.hs_peak_feedback,
                   bg=PANEL, fg=MUTED, font=("Segoe UI", 8),
                   anchor="w").pack(fill="x", pady=(0, 1))

        terminal_heading = tk.Frame(terminal_panel, bg=PANEL)
        terminal_heading.pack(fill="x", pady=(6, 3))
        self.label(terminal_heading, "Terminal TX / RX", bg=PANEL,
                   font=("Segoe UI", 11, "bold")).pack(side="left")
        ttk.Button(terminal_heading, text="Xóa", command=self.clear_terminal).pack(side="right", padx=2)
        ttk.Button(terminal_heading, text="Mở rộng", command=self.open_log_window).pack(side="right", padx=2)
        ttk.Button(terminal_heading, text="Xuất TXT", command=self.export_txt).pack(side="right", padx=2)
        ttk.Button(terminal_heading, text="Xuất CSV", command=self.export_csv).pack(side="right", padx=2)
        ttk.Checkbutton(terminal_panel, text="PID preview 5 Hz (CSV đầy đủ)",
                        variable=self.preview_pid_log).pack(anchor="w", pady=(1, 2))
        text_area = tk.Frame(terminal_panel, bg=PANEL)
        text_area.pack(fill="both", expand=True)
        self.console = tk.Text(text_area, bg=BG, fg=FG, insertbackground=FG,
                               font=("Consolas", 10), height=8, wrap="none",
                               state="disabled", relief="flat")
        scrollbar = ttk.Scrollbar(text_area, orient="vertical", command=self.console.yview)
        self.console.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.console.pack(side="left", fill="both", expand=True)
        hscroll = ttk.Scrollbar(terminal_panel, orient="horizontal", command=self.console.xview)
        self.console.configure(xscrollcommand=hscroll.set)
        hscroll.pack(fill="x")
        cmd_row = tk.Frame(terminal_panel, bg=PANEL)
        cmd_row.pack(fill="x", pady=(5, 0))
        self.cmd_var = tk.StringVar()
        command_entry = ttk.Entry(cmd_row, textvariable=self.cmd_var)
        command_entry.pack(side="left", fill="x", expand=True, padx=(0, 4))
        command_entry.bind("<Return>", lambda _e: self.send_terminal())
        ttk.Button(cmd_row, text="Gửi", command=self.send_terminal).pack(side="right")
        self.log("Chọn COM Bluetooth SPP (outgoing). Chưa có xác nhận firmware.")

    def log(self, message):
        line = f"[{datetime.datetime.now().astimezone().isoformat(timespec='milliseconds')}] {message}\n"
        self.log_lines.append(line)
        self.session_log_file.write(line)
        if self.run_txt_file is not None:
            try:
                self.run_txt_file.write(line)
            except OSError as exc:
                self.csv_status.set(f"TXT WRITE ERROR: {exc}")
                try:
                    self.run_txt_file.close()
                except OSError:
                    pass
                self.run_txt_file = None
        self.console.configure(state="normal")
        self.console.insert("end", line)
        if int(self.console.index("end-1c").split(".")[0]) > 1200:
            self.console.delete("1.0", "201.0")
        self.console.see("end")
        self.console.configure(state="disabled")
        if self.log_mirror is not None and self.log_mirror.winfo_exists():
            self.log_mirror.configure(state="normal")
            self.log_mirror.insert("end", line)
            self.log_mirror.see("end")
            self.log_mirror.configure(state="disabled")

    def clear_terminal(self):
        self.log_lines.clear()
        self.session_log_file.seek(0)
        self.session_log_file.truncate()
        for widget in (self.console, self.log_mirror):
            if widget is not None and widget.winfo_exists():
                widget.configure(state="normal")
                widget.delete("1.0", "end")
                widget.configure(state="disabled")

    def open_log_window(self):
        if self.log_window is not None and self.log_window.winfo_exists():
            self.log_window.lift()
            return
        win = tk.Toplevel(self)
        win.title("LineRobot — Terminal TX/RX và PID preview")
        win.geometry("1100x640")
        txt = tk.Text(win, bg=BG, fg=FG, font=("Consolas", 10), wrap="none")
        sb = ttk.Scrollbar(win, orient="vertical", command=txt.yview)
        txt.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        txt.pack(fill="both", expand=True)
        txt.insert("end", "".join(self.log_lines))
        txt.see("end")
        txt.configure(state="disabled")
        self.log_window, self.log_mirror = win, txt
        def on_close():
            self.log_mirror = None
            self.log_window = None
            win.destroy()
        win.protocol("WM_DELETE_WINDOW", on_close)

    def open_sensor_table(self):
        if self.sensor_window is not None and self.sensor_window.winfo_exists():
            self.sensor_window.lift()
            return
        win = tk.Toplevel(self)
        win.title("LineRobot — 16 ngưỡng riêng từ NVS")
        win.geometry("780x560")
        cols = ("eye", "raw", "white", "black", "threshold", "cal", "logic")
        table = ttk.Treeview(win, columns=cols, show="headings", height=16)
        for col, label in zip(cols, ("Mắt", "ADC RAW", "Trắng W", "Đen B", "Ngưỡng T_i",
                                      "CAL 0–1000", "0 trắng / 1 đen")):
            table.heading(col, text=label)
            table.column(col, width=95, anchor="center")
        table.pack(fill="both", expand=True)
        for i in range(16):
            table.insert("", "end", iid=str(i), values=(i, "—", "—", "—", "—", "—", "—"))
        self.sensor_window, self.sensor_table = win, table
        self.update_sensor_table()
        def on_close():
            self.sensor_window, self.sensor_table = None, None
            win.destroy()
        win.protocol("WM_DELETE_WINDOW", on_close)

    def update_sensor_table(self):
        if self.sensor_table is None or not self.sensor_table.winfo_exists():
            return
        for i, raw in enumerate(self.last_adc):
            if self.calibration_ok:
                white, black = self.cal_min[i], self.cal_max[i]
                cal = normalize_adc(raw, white, black)
                self.sensor_table.item(str(i), values=(i, raw, white, black,
                                     eye_raw_threshold(white, black), cal, int(cal >= BLACK_THRESHOLD_CAL)))
            else:
                self.sensor_table.item(str(i), values=(i, raw, "—", "—", "—", "—", "—"))

    def export_txt(self):
        recorder = self.run_csv.last
        if recorder is None:
            self.log("Chưa có lượt chạy nào để xuất TXT.")
            return
        source_path = recorder.path.with_suffix(".txt")
        if not source_path.is_file():
            self.log(f"Không tìm thấy TXT của lượt chạy: {source_path}")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".txt", initialdir=str(source_path.parent),
            initialfile=source_path.name, filetypes=[("Text log", "*.txt")])
        if not path:
            return
        if Path(path).resolve() == source_path.resolve():
            self.log(f"TXT đã được lưu sẵn tại: {source_path}")
            return
        try:
            if self.run_txt_file is not None and self.run_txt_path == source_path:
                self.run_txt_file.flush()
            with source_path.open("rb") as source, open(path, "wb") as target:
                shutil.copyfileobj(source, target)
            self.log(f"Đã xuất TXT lượt chạy: {path}")
        except OSError as exc:
            self.log(f"Không thể xuất TXT: {exc}")

    def export_csv(self):
        selected = self.run_csv.last
        if selected is None:
            self.log("Chưa có lượt chạy CSV nào để xuất.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", initialdir=str(selected.path.parent),
            initialfile=selected.path.name, filetypes=[("CSV", "*.csv")])
        if not path:
            return

        def export_job():
            try:
                count = selected.export_snapshot(path)
                message = f"Đã xuất CSV ({count} mẫu): {path}"
            except (OSError, ValueError) as exc:
                message = f"Lỗi xuất CSV: {exc}"
            self.controls_q.put(("csv_export", message))

        threading.Thread(target=export_job, daemon=True).start()

    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def toggle_connect(self):
        if self.reader:
            self.disconnect()
            return
        port = self.port_var.get().strip()
        if not port:
            self.log("Chưa chọn COM")
            return
        try:
            # Merely connecting Bluetooth never opens a CSV file.
            self.reader = Reader(port, self.q, None, self.controls_q)
        except (serial.SerialException, OSError, ValueError) as exc:
            self.status_label.config(text=f"Lỗi COM: {exc}", fg=ROSE)
            self.log(str(exc))
            return
        self.reader.start()
        self.hs_brake_start = 1800
        self.hs_peak_confirmed = None
        self.hs_peak_pending = None
        self.hs_peak_pending_at = None
        self.hs_peak_dirty = False
        self.hs_peak_value_text.set("— / 255")
        self.hs_peak_feedback.set("Đang đọc HS_PEAK từ ESP32...")
        self.estop_latched = None
        self.estop_pending = False
        self.start_pending = False
        self.set_auto_running(False)
        self.control_state.set("Đang kết nối — chờ STATUS từ firmware.")
        self.adc_status.set("ADC: chờ STATUS")
        self.control_state_label.configure(fg=AMBER)
        self.update_control_buttons()
        self.prev = None
        self.last_rx_total = self.reader.frames_rx
        self.last_csv_total = 0
        self.frames = 0
        self.rate_time = time.monotonic()
        self.last_diag_rx = None
        self.stop_ack_deadline = None
        self.stop_verify_pending = False
        self.stop_verify_deadline = None
        self.stop_verify_retries = 0
        self.next_diag_poll = self.rate_time + self.diag_interval_s
        self.loop_debug.set("LOOP: đang đo...    PID: -- Hz    SPP TX: -- Hz")
        self.connect_btn.config(text="Ngắt")
        self.status_label.config(text=f"Đã mở {port}", fg=GREEN)
        self.log(f"Đã mở {port}; chờ frame SPP từ ESP32")
        self.log("CSV: BẮT ĐẦU tạo file mới, DỪNG chốt file")
        self.csv_status.set("CSV: SẴN SÀNG — chưa ghi")
        self.calibration_ok = False
        self.cal_thresholds = [None] * 16
        self.cal_data = {}
        self.update_control_buttons()
        self.send_command("STATUS")
        self.send_command("CAL_STATUS")
        self.send_command("CAL_DUMP")
        self.send_command("HS_PEAK")
        self.send_command("PID")
        self.send_command("STREAM:" + self.stream_rate.get())

    def request_diagnostics(self):
        if self.send_command("DIAG"):
            self.next_diag_poll = time.monotonic() + self.diag_interval_s
        self.send_command("PERF")

    def disconnect(self):
        if self.reader:
            self.reader.running = False
            self.reader.join(timeout=1)
            if self.run_csv.active is not None:
                self.finish_run_csv("MẤT KẾT NỐI; CHƯA XÁC NHẬN DỪNG")
            self.reader = None
        self.hs_peak_confirmed = None
        self.hs_peak_pending = None
        self.hs_peak_pending_at = None
        self.hs_peak_feedback.set("Chưa kết nối ESP32 — HS chưa xác nhận")
        self.connect_btn.config(text="Kết nối")
        self.status_label.config(text="Chưa kết nối", fg=ROSE)
        self.estop_latched = None
        self.estop_pending = False
        self.start_pending = False
        self.stop_ack_deadline = None
        self.stop_verify_pending = False
        self.stop_verify_deadline = None
        self.stop_verify_retries = 0
        self.set_auto_running(False)
        self.control_state.set("Đã ngắt SPP — không thể xác nhận tình trạng động cơ.")
        self.adc_status.set("ADC: chưa kết nối")
        self.next_diag_poll = None
        self.last_diag_rx = None
        self.loop_debug.set("LOOP: -- Hz    PID: -- Hz    SPP TX: -- Hz    (mất kết nối)")
        self.control_state_label.configure(fg=ROSE)
        self.update_control_buttons()
        self.log("Đã đóng COM. Firmware phải dừng động cơ khi SPP mất kết nối.")

    def send_command(self, command):
        command = command.strip()
        if not command:
            return False
        if self.reader is None or not self.reader.is_alive():
            self.log(f"Chưa kết nối — không gửi: {command}")
            return False
        try:
            self.reader.send(command)
            self.log(f"TX > {command}")
            if command.upper() in ("ENC_RESET", "RESET_ENCODER"):
                # Discard the old tick/time baseline before computing speed.
                self.prev = None
                self.wheel_speed = [0.0, 0.0]
            return True
        except (serial.SerialException, OSError) as exc:
            self.log(f"Lỗi gửi: {exc}")
            self.disconnect()
            return False

    def send_terminal(self):
        cmd = self.cmd_var.get().strip()
        if self.send_command(cmd):
            self.cmd_var.set("")

    def preview_pwm(self, *_args):
        self.pwm_text.set(f"{round(self.pwm_var.get())} / 255")

    def _hs_base_limit(self):
        """Use the app's requested BASE for early validation; ESP32 rechecks."""
        try:
            return int(self.pid_vars["Base"].get().strip())
        except (ValueError, AttributeError):
            return 0

    def preview_hs_peak(self, *_args):
        value = max(100, min(255, round(self.hs_peak_var.get())))
        self.hs_peak_dirty = True
        self.hs_peak_value_text.set(f"{value} (chưa áp dụng)")
        self.hs_peak_feedback.set("Nhấn Áp dụng HS để gửi ESP32, hoặc Đọc HS để hủy chọn.")
        if hasattr(self, "hs_peak_widgets"):
            self.update_control_buttons()

    def preset_hs_peak(self, value):
        if not self._can_edit_hs_peak() or self.hs_peak_pending is not None:
            return
        self.hs_peak_var.set(value)
        self.preview_hs_peak()

    def _can_edit_hs_peak(self):
        return (self.reader is not None and self.reader.is_alive()
                and self.estop_latched is False and not self.estop_pending
                and not self.auto_running and not self.start_pending
                and self.stop_ack_deadline is None
                and not self.stop_verify_pending
                and self.run_csv.active is None)

    def read_hs_peak(self):
        if self.hs_peak_pending is not None:
            self.log("Chờ ESP32 xác nhận HS_PEAK trước khi đọc lại.")
            return False
        if self.reader is None or not self.reader.is_alive():
            self.log("Chưa kết nối ESP32 để đọc HS_PEAK.")
            return False
        self.hs_peak_dirty = False
        self.hs_peak_confirmed = None
        self.hs_peak_feedback.set("Đang đọc lại HS_PEAK từ firmware...")
        self.update_control_buttons()
        return self.send_command("HS_PEAK")

    def apply_hs_peak(self):
        if not self._can_edit_hs_peak() or self.hs_peak_pending is not None:
            self.log("HS_PEAK chỉ chỉnh được khi đã kết nối và xe đã dừng.")
            return False
        peak = max(100, min(255, round(self.hs_peak_var.get())))
        base = self._hs_base_limit()
        if peak < base:
            self.hs_peak_feedback.set(f"Không hợp lệ: HS_PEAK {peak} < BASE {base}")
            self.log(f"Không áp dụng HS_PEAK={peak}: thấp hơn BASE={base}.")
            return False
        self.hs_peak_pending = peak
        self.hs_peak_pending_at = time.monotonic()
        self.hs_peak_feedback.set(f"Đang chờ ESP32 xác nhận PWM {peak}...")
        self.update_control_buttons()
        if not self.send_command(f"HS_PEAK:{peak}"):
            self.hs_peak_pending = None
            self.hs_peak_pending_at = None
            self.update_control_buttons()
            return False
        return True

    def apply_pwm(self):
        pwm = max(0, min(255, round(self.pwm_var.get())))
        if self.hs_peak_confirmed is not None and pwm > self.hs_peak_confirmed:
            self.log(f"BASE {pwm} vượt HS_PEAK {self.hs_peak_confirmed}; tăng HS_PEAK trước.")
            return
        if self.send_command(f"V:{pwm}"):
            self.pid_vars["Base"].set(str(pwm))

    def preset_pwm(self, value):
        self.pwm_var.set(value)
        self.preview_pwm()
        self.apply_pwm()

    def apply_pid(self):
        try:
            vals = [float(self.pid_vars[key].get()) for key in ("Kp", "Ki", "Kd")]
            base_s = self.pid_vars["Base"].get().strip()
            base = int(base_s)
            if not all(math.isfinite(value) and 0 <= value <= 50 for value in vals):
                raise ValueError("Kp, Ki, Kd phải là số hữu hạn từ 0 đến 50")
            if str(base) != base_s or not 0 <= base <= 255:
                raise ValueError("Base phải là số nguyên từ 0 đến 255")
            if self.hs_peak_confirmed is not None and base > self.hs_peak_confirmed:
                raise ValueError(
                    f"BASE={base} vượt HS_PEAK={self.hs_peak_confirmed}; tăng HS trước")
        except ValueError as exc:
            self.log(f"Thông số PID không hợp lệ: {exc}")
            return False
        for command in (f"KP:{vals[0]:.2f}", f"KI:{vals[1]:.2f}",
                        f"KD:{vals[2]:.2f}", f"BASE:{base}"):
            if not self.send_command(command):
                return False
        self.pwm_var.set(base)
        self.preview_pwm()
        self.log("Đã GỬI các lệnh PID, đang chờ firmware xác nhận.")
        return True

    def select_map(self, map_id):
        """Select the destination for future runs, never relabel a live run."""
        if map_id not in (1, 2, 3):
            raise ValueError("Map must be 1, 2 or 3")
        if (self.run_csv.active is not None or self.auto_running
                or self.start_pending or self.stop_ack_deadline is not None
                or self.stop_verify_pending):
            self.log("Không thể đổi MAP khi xe đang chạy hoặc chưa xác nhận STOP.")
            return
        # A physical GPIO34 START uses the firmware's stored MAP.
        # Update the ESP32 as soon as the user selects a map, not only
        # when the Python START button sends AUTO:map_id.
        if self.reader is not None and self.reader.is_alive():
            if not self.send_command(f"MAP:{map_id}"):
                self.log("Chưa đồng bộ MAP với ESP32; kiểm tra kết nối.")
                return
        self.selected_map = map_id
        self.map_status.set(f"Đã chọn {map_id}")
        for number, button in self.map_buttons.items():
            chosen = number == map_id
            button.configure(bg=CYAN if chosen else GREY,
                             fg=BG if chosen else FG)
        self.log(f"Đã chọn MAP {map_id}; các lượt START kế tiếp lưu trong logs/map_{map_id:02d}/")
        self.update_control_buttons()

    def update_control_buttons(self):
        connected = self.reader is not None and self.reader.is_alive()
        can_start = (connected and self.estop_latched is False
                     and not self.estop_pending and not self.start_pending
                     and not self.auto_running and self.run_csv.active is None
                     and self.stop_ack_deadline is None
                     and not self.stop_verify_pending
                     and self.hs_peak_pending is None
                     and self.hs_peak_confirmed is not None
                     and not self.hs_peak_dirty
                     and self.calibration_ok and self.selected_map is not None
                     and all(v is not None for v in self.cal_thresholds))
        self.start_btn.configure(state="normal" if can_start else "disabled")
        self.stop_btn.configure(state="normal" if connected else "disabled")
        locked = (self.run_csv.active is not None or self.auto_running
                  or self.start_pending or self.stop_ack_deadline is not None
                  or self.stop_verify_pending)
        for button in self.map_buttons.values():
            button.configure(state="disabled" if locked else "normal")
        if hasattr(self, "hs_peak_widgets"):
            editable = self._can_edit_hs_peak() and self.hs_peak_pending is None
            for widget in self.hs_peak_widgets:
                widget.configure(state="normal" if editable else "disabled")
            self.hs_peak_apply_btn.configure(
                state="normal" if editable else "disabled")
            self.hs_peak_read_btn.configure(
                state="normal" if connected and self.hs_peak_pending is None
                else "disabled")

    def set_auto_running(self, running):
        now = time.monotonic()
        if running and not self.auto_running:
            self.run_started_at = now
        elif not running and self.auto_running:
            if self.run_started_at is not None:
                self.run_elapsed_seconds += now - self.run_started_at
            self.run_started_at = None
        self.auto_running = running
        self.update_run_timer_display(now)

    def reset_run_timer(self):
        self.run_elapsed_seconds = 0.0
        self.run_started_at = time.monotonic() if self.auto_running else None
        self.update_run_timer_display()

    def update_run_timer_display(self, now=None):
        elapsed = self.run_elapsed_seconds
        if self.auto_running and self.run_started_at is not None:
            elapsed += (time.monotonic() if now is None else now) - self.run_started_at
        total_ticks = max(0, int(elapsed * 100))
        total_seconds, ticks = divmod(total_ticks, 100)
        hours, remainder = divmod(total_seconds, 3600)
        minutes, seconds = divmod(remainder, 60)
        self.run_timer_var.set(
            f"{hours:02d}:{minutes:02d}:{seconds:02d}.{ticks:02d}")

    def finish_run_csv(self, reason):
        recorder = self.run_csv.finish(self.reader, reason)
        self.stop_ack_deadline = None
        self.start_confirmed = False
        if recorder is None:
            return
        self.csv_status.set(f"CSV ĐANG ĐÓNG: {recorder.path.name}")
        self.log(f"CSV đang chốt ({reason}): {recorder.path}")
        if self.run_txt_file is not None:
            try:
                self.run_txt_file.close()
            except OSError as exc:
                self.csv_status.set(f"TXT CLOSE ERROR: {exc}")
            self.run_txt_file = None

    def start_line(self):
        if self.selected_map is None:
            self.log("Hãy chọn MAP 1, MAP 2 hoặc MAP 3 trước khi START.")
            return
        if (self.hs_peak_confirmed is None or self.hs_peak_pending is not None
                or self.hs_peak_dirty):
            self.log("Chưa xác nhận HS_PEAK: chọn Áp dụng HS hoặc Đọc HS trước START.")
            return
        if (self.estop_latched is not False or self.auto_running or self.start_pending
                or self.stop_ack_deadline is not None or self.stop_verify_pending):
            self.log("Chưa thể START: AUTO đang chạy hoặc chưa xác nhận STOP.")
            return
        if self.reader is None or not self.reader.is_alive():
            self.log("Chưa kết nối Bluetooth SPP.")
            return
        if not self.calibration_ok or any(v is None for v in self.cal_thresholds):
            self.log("Chưa nhận đủ 16 ngưỡng TH từ ESP32; gửi lại CAL_DUMP.")
            self.send_command("CAL_DUMP")
            return
        try:
            calib = (self.cal_min, self.cal_max)
            recorder = self.run_csv.begin(
                self.reader, CSV_HEADER, calib, list(self.cal_thresholds),
                self.selected_map)
        except (OSError, RuntimeError, ValueError) as exc:
            self.log(f"Không tạo được CSV, chưa gửi AUTO: {exc}")
            return
        try:
            self.run_txt_path = recorder.path.with_suffix(".txt")
            self.run_txt_file = self.run_txt_path.open(
                "w", encoding="utf-8", buffering=1)
        except OSError as exc:
            self.run_txt_file = None
            self.run_csv.finish(self.reader, "KHÔNG TẠO ĐƯỢC TXT")
            self.log(f"Không tạo được TXT, chưa gửi AUTO: {exc}")
            self.update_control_buttons()
            return
        self.last_csv_total = 0
        self.stop_ack_deadline = None
        self.start_confirmed = False
        self.csv_status.set(f"CSV GHI: {recorder.path.name}")
        self.log(f"CSV BẮT ĐẦU: {recorder.path}")
        # Firmware receives the same MAP ID as the CSV recorder in one START command.
        if self.send_command(f"AUTO:{self.selected_map}"):
            self.start_pending = True
            self.control_state.set("Đã gửi AUTO — chờ ESP32 xác nhận.")
            self.update_control_buttons()
        else:
            self.finish_run_csv("KHÔNG GỬI ĐƯỢC AUTO")

    def begin_external_run(self, origin_message):
        """Start MAP-specific CSV/TXT when GPIO34 starts AUTO on the ESP32.

        GUI START already opens its recorder before sending AUTO; in that
        case the existing run owns the log and must not be replaced.
        """
        if self.run_csv.active is not None:
            return
        match = re.search(r"\bmap=([123])\b", origin_message)
        if match is None:
            self.log("GPIO34 START: RUN_ORIGIN không có MAP hợp lệ, không tạo CSV.")
            return
        map_id = int(match.group(1))
        self.selected_map = map_id  # Actual firmware MAP always wins for labels.
        self.map_status.set(f"Đã chọn {map_id} (GPIO34)")
        for number, button in self.map_buttons.items():
            button.configure(bg=CYAN if number == map_id else GREY,
                             fg=BG if number == map_id else FG)
        if self.reader is None or not self.reader.is_alive():
            self.log("GPIO34 START: không có kết nối SPP, không thể ghi CSV.")
            return
        if not self.calibration_ok or any(v is None for v in self.cal_thresholds):
            self.log("GPIO34 START: chưa tải CAL_DUMP, không thể tạo CSV đủ ngưỡng.")
            self.send_command("CAL_DUMP")
            return
        try:
            recorder = self.run_csv.begin(
                self.reader, CSV_HEADER, (self.cal_min, self.cal_max),
                list(self.cal_thresholds), map_id)
        except (OSError, RuntimeError, ValueError) as exc:
            self.log(f"GPIO34 START: không thể mở CSV: {exc}")
            return
        try:
            self.run_txt_path = recorder.path.with_suffix(".txt")
            self.run_txt_file = self.run_txt_path.open(
                "w", encoding="utf-8", buffering=1)
        except OSError as exc:
            self.run_txt_file = None
            self.run_csv.finish(self.reader, "GPIO34 START: TXT OPEN FAILED")
            self.log(f"GPIO34 START: lỗi mở TXT: {exc}")
            return
        self.last_csv_total = 0
        self.stop_ack_deadline = None
        self.start_confirmed = True
        self.csv_status.set(f"CSV GHI: {recorder.path.name}")
        self.log(f"GPIO34 START: lưu dữ liệu MAP {map_id} tại {recorder.path}")
        self.update_control_buttons()

    def stop_line(self):
        # Require an ESP32 ACK even if the current CSV has already closed.
        if self.send_command("STOP"):
            self.stop_ack_deadline = time.monotonic() + 2.0
            self.stop_verify_pending = False
            self.stop_verify_deadline = None
            self.stop_verify_retries = 0
            self.start_pending = False
            self.control_state.set("Đã gửi STOP — chờ ESP32 xác nhận.")
            self.update_control_buttons()
        else:
            self.finish_run_csv("STOP KHÔNG GỬI ĐƯỢC / CHƯA XÁC NHẬN DỪNG")

    def confirm_stopped(self, reason):
        # Only explicit ESP32 stop/finish ACK or STATUS with zero PWM
        # is allowed to release START after a missing STOP acknowledgment.
        self.stop_ack_deadline = None
        self.stop_verify_pending = False
        self.stop_verify_deadline = None
        self.stop_verify_retries = 0
        self.start_pending = False
        self.set_auto_running(False)
        self.finish_run_csv("ESP32 ĐÃ XÁC NHẬN DỪNG: " + reason)
        self.start_confirmed = False
        if self.estop_latched is False:
            self.control_state.set("ESP32 đã xác nhận dừng — có thể START lượt mới.")
            self.control_state_label.configure(fg=GREEN)
        self.update_control_buttons()

    def handle_stop_timeout(self):
        # Freeze log, but keep START interlocked until ESP32 confirms stopped.
        self.stop_ack_deadline = None
        self.stop_verify_pending = True
        self.stop_verify_deadline = time.monotonic() + 1.5
        self.stop_verify_retries = 0
        self.finish_run_csv("STOP ACK QUÁ HẠN — CHƯA XÁC NHẬN ĐỘNG CƠ")
        self.control_state.set("Mất ACK STOP; đang hỏi STATUS để xác nhận động cơ.")
        self.control_state_label.configure(fg=AMBER)
        self.log("STOP timeout: đang hỏi STATUS, không cần đóng app.")
        self.send_command("STATUS")
        self.update_control_buttons()

    def start_calibration(self):
        if self.send_command("CAL_WHITE"):
            self.cal_state.set("Đang lấy TRẮNG trên ESP32; giữ 16 mắt cố định.")

    def stop_calibration(self):
        if self.send_command("CAL_BLACK"):
            self.cal_state.set("Đã gửi Lấy ĐEN; chờ ESP32 xác nhận.")

    def sample_csv_row(self, s):
        adc = list(s[1:17])
        if self.calibration_ok:
            cal = [normalize_adc(adc[i], self.cal_min[i], self.cal_max[i]) for i in range(16)]
            bits = [int(value >= BLACK_THRESHOLD_CAL) for value in cal]
            word = "".join(str(bit) for bit in bits)
        else:
            cal, bits, word = [""] * 16, [""] * 16, ""
        host_time = datetime.datetime.now().astimezone().isoformat(timespec="milliseconds")
        return [host_time, s[0], *adc, *cal, *bits, *s[17:27], word]

    def on_sample(self, s):
        self.hist.append(s)
        self.frames += 1
        row = self.sample_csv_row(s)
        self.recent_rows.append(row)
        # Independent CSV worker handles all received frames, even if UI drops.
        self.last_adc = list(s[1:17])
        t, left, right = s[0], s[17], s[18]
        if self.preview_pid_log.get() and s[26] == 1:
            if self.last_preview_ms is None or (t - self.last_preview_ms) % (2**32) >= 200:
                self.last_preview_ms = t
                self.log(f"PID t={t}ms adc={','.join(map(str, s[1:17]))}"
                         f" cal={','.join(str(v) for v in row[18:34])}"
                         f" logic={row[-1] or '?'} pwm=({s[19]},{s[20]})"
                         f" enc=({s[17]},{s[18]}) err={s[21]:.3f}"
                         f" p={s[22]:.3f} i={s[23]:.3f} d={s[24]:.3f}"
                         f" out={s[25]:.3f}")
        if self.prev and t > self.prev[0]:
            dt = (t - self.prev[0]) / 1000.0
            v_left = (left - self.prev[1]) / dt
            v_right = (right - self.prev[2]) / dt
            self.wheel_speed[0] = 0.8 * self.wheel_speed[0] + 0.2 * v_left
            self.wheel_speed[1] = 0.8 * self.wheel_speed[1] + 0.2 * v_right
        self.prev = (t, left, right)

    def on_text(self, message):
        # Hardware START has no GUI callback, so RUN_ORIGIN opens its log.
        # Do this before mirroring RUN_ORIGIN to TXT.
        if message.startswith("RUN_ORIGIN "):
            self.begin_external_run(message)
        self.log(f"RX < {message}")
        if message == "ESTOP: LATCHED" or message == "ERR: ESTOP_LATCHED":
            self.stop_ack_deadline = None
            self.stop_verify_pending = False
            self.stop_verify_deadline = None
            self.stop_verify_retries = 0
            self.estop_latched = True
            self.estop_pending = False
            self.start_pending = False
            self.set_auto_running(False)
            self.control_state.set("Firmware báo E-STOP đã chốt; có thể RESET_ESTOP qua terminal.")
            self.control_state_label.configure(fg=ROSE)
            self.finish_run_csv("ESTOP ĐÃ XÁC NHẬN")
            self.update_control_buttons()
        elif message == "ESTOP: CLEARED (MOTORS OFF)":
            self.estop_latched = False
            self.estop_pending = False
            self.start_pending = False
            self.set_auto_running(False)
            self.control_state.set("Đã mở khóa E-STOP — động cơ vẫn dừng.")
            self.control_state_label.configure(fg=GREEN)
            self.update_control_buttons()
        elif message == "MODE: AUTO PID":
            self.start_pending = False
            self.start_confirmed = True
            self.set_auto_running(True)
            self.control_state.set("ĐANG DÒ LINE — PID AUTO đang chạy.")
            self.control_state_label.configure(fg=GREEN)
            self.update_control_buttons()
        elif message in ("ERR: MAP_NOT_SELECTED", "ERR: MAP_USE_1_2_3",
                              "ERR: MAP_SELECT_FAILED", "ERR: AUTO_ALREADY_RUNNING"):
            if self.start_pending and not self.start_confirmed:
                self.start_pending = False
                self.set_auto_running(False)
                self.control_state.set("Firmware từ chối START/MAP: " + message)
                self.control_state_label.configure(fg=AMBER)
                self.finish_run_csv("AUTO BỊ TỪ CHỐI: " + message)
                self.update_control_buttons()
        elif message.startswith("ERR: CALIB_REQUIRED_OR_PENDING"):
            self.start_pending = False
            self.set_auto_running(False)
            self.control_state.set("Không thể bắt đầu: cần hoàn tất calibration TRẮNG/ĐEN.")
            self.control_state_label.configure(fg=AMBER)
            self.finish_run_csv("AUTO BỊ TỪ CHỐI: CALIBRATION")
            self.update_control_buttons()
        elif message in ("ERR: ADC_NOT_READY", "ERR: ADC_BENCH_ACTIVE"):
            if self.start_pending and not self.start_confirmed:
                self.start_pending = False
                self.set_auto_running(False)
                self.finish_run_csv("AUTO BỊ TỪ CHỐI: " + message)
                self.update_control_buttons()
        elif message.startswith("HS_PEAK: PWM="):
            peak_match = re.search(r"\bPWM=(\d+)\b", message)
            if peak_match:
                confirmed = int(peak_match.group(1))
                if 100 <= confirmed <= 255:
                    self.high_speed_peak = confirmed
                    # Prefer the actual BRAKE_START echoed by firmware.
                    start_match = re.search(r"\bBRAKE_START=(\d+)\b", message)
                    self.hs_brake_start = (
                        int(start_match.group(1)) if start_match
                        else (1400 if confirmed >= 240 else 1600 if confirmed >= 230 else 1800))
                    self.hs_peak_confirmed = confirmed
                    expected = self.hs_peak_pending
                    if expected is None or expected == confirmed:
                        self.hs_peak_pending = None
                        self.hs_peak_pending_at = None
                        # Explicit firmware readbacks supersede local edits
                        # only when the user clicked Read or Apply.
                        if expected is not None or not self.hs_peak_dirty:
                            self.hs_peak_var.set(confirmed)
                            self.hs_peak_dirty = False
                            self.hs_peak_value_text.set(f"{confirmed} / 255")
                        self.hs_peak_feedback.set(
                            f"ESP32 xác nhận HS_PEAK = {confirmed} PWM")
                    else:
                        self.hs_peak_feedback.set(
                            f"ESP32 báo {confirmed}; chờ xác nhận PWM {expected}")
                    self.high_speed_status.set(
                        f"HS: READY | PEAK={confirmed} | BRAKE={self.hs_brake_start}-2300 | HOLD=110")
                    self.update_control_buttons()
        elif message.startswith("ERR: HS_PEAK_"):
            if self.hs_peak_pending is not None:
                self.hs_peak_pending = None
                self.hs_peak_pending_at = None
                self.hs_peak_feedback.set(f"ESP32 từ chối: {message}")
                self.update_control_buttons()
                self.send_command("HS_PEAK")
        elif message == "ERR: BASE_GT_HS_PEAK":
            self.hs_peak_feedback.set(
                "BASE vượt HS_PEAK; tăng HS_PEAK trước khi đổi BASE")
            self.send_command("PID")
        elif message.startswith("HS_STAGE "):
            peak = re.search(r"\bpeak=(\d+)\b", message)
            if peak:
                self.high_speed_peak = int(peak.group(1))
            start = re.search(r"\bbrake=(\d+)\b", message)
            self.hs_brake_start = (int(start.group(1)) if start
                                   else (1400 if self.high_speed_peak >= 240 else 1600 if self.high_speed_peak >= 230 else 1800))
            stage = re.search(r"HS_STAGE (\w+)", message)
            tick = re.search(r"\brunEnc=(-?\d+)", message)
            if stage:
                self.high_speed_status.set(
                    f"HS: {stage.group(1)} | ENC={tick.group(1) if tick else '--'} "
                    f"| PEAK={self.high_speed_peak} | BRAKE={self.hs_brake_start}-2300 | HOLD=110"
                )
        elif message.startswith("DIAG:"):
            peak = re.search(r"\bpeak=(\d+)\b", message)
            if peak:
                value = int(peak.group(1))
                if 100 <= value <= 255:
                    self.high_speed_peak = value
                    self.hs_brake_start = 1400 if value >= 240 else 1600 if value >= 230 else 1800
                    # DIAG is telemetry, not an ACK for a pending HS write.
                    if self.hs_peak_pending is None:
                        self.hs_peak_confirmed = value
                        if not self.hs_peak_dirty:
                            self.hs_peak_var.set(value)
                            self.hs_peak_value_text.set(f"{value} / 255")
            hs = re.search(r"\bhs=(\d+)\b", message)
            tick = re.search(r"\brunEnc=(-?\d+)\b", message)
            if hs:
                names = ("OFF", "RAMP", "CRUISE", "BRAKE", "COMPLETE", "ABORTED", "HOLD")
                state = int(hs.group(1))
                title = names[state] if 0 <= state < len(names) else "UNKNOWN"
                self.high_speed_status.set(
                    f"HS: {title} | ENC={tick.group(1) if tick else '--'} "
                    f"| PEAK={self.high_speed_peak} | BRAKE={self.hs_brake_start}-2300 | HOLD=110"
                )
            loop = re.search(r"\bloop=(\d+(?:\.\d+)?)Hz\b", message)
            pid = re.search(r"\bpid=(\d+(?:\.\d+)?)Hz\b", message)
            sent = re.search(r"\bsent=(\d+(?:\.\d+)?)Hz\b", message)
            dropped = re.search(r"\bdrop=(\d+)\b", message)
            if loop and pid and sent:
                self.last_diag_rx = time.monotonic()
                warning = f"    TX drop: {dropped.group(1)}" if dropped else ""
                self.loop_debug.set(
                    f"LOOP: {float(loop.group(1)):.1f} Hz    "
                    f"PID: {float(pid.group(1)):.1f} Hz    "
                    f"SPP TX: {float(sent.group(1)):.1f} Hz{warning}")
        elif message.startswith("STATUS:"):
            estop = re.search(r"\bestop=(\d)\b", message)
            auto = re.search(r"\bauto=(\d)\b", message)
            pwm_l = re.search(r"\bpwmL=(-?\d+)\b", message)
            pwm_r = re.search(r"\bpwmR=(-?\d+)\b", message)
            is_stopped = (auto is not None and auto.group(1) == "0"
                          and pwm_l is not None and pwm_r is not None
                          and int(pwm_l.group(1)) == 0
                          and int(pwm_r.group(1)) == 0)
            if is_stopped and (self.stop_ack_deadline is not None
                               or self.stop_verify_pending
                               or self.start_confirmed or self.auto_running):
                self.confirm_stopped("STATUS auto=0 pwmL=0 pwmR=0")
            adc_mode = re.search(r"\badc=([A-Z0-9_]+)\b", message)
            adc_ok = re.search(r"\badc_ok=(\d)\b", message)
            if adc_mode:
                ready = bool(adc_ok and adc_ok.group(1) == "1")
                self.adc_status.set(f"ADC: {adc_mode.group(1)} | {'READY' if ready else 'FAULT'}")
            if estop:
                self.estop_latched = estop.group(1) == "1"
                self.estop_pending = False
                self.start_pending = False
                self.set_auto_running(bool(auto and auto.group(1) == "1"))
                if self.estop_latched:
                    self.set_auto_running(False)
                    self.control_state.set("E-STOP đang khóa — gửi RESET_ESTOP trong terminal để mở khóa.")
                    self.control_state_label.configure(fg=ROSE)
                    self.finish_run_csv("STATUS: ESTOP ĐANG KHÓA")
                elif self.start_confirmed and not self.auto_running:
                    self.finish_run_csv("STATUS: AUTO ĐÃ DỪNG")
                elif self.auto_running:
                    self.control_state.set("ĐANG DÒ LINE — PID AUTO đang chạy.")
                    self.control_state_label.configure(fg=GREEN)
                else:
                    self.control_state.set("SẴN SÀNG — động cơ đang dừng.")
                    self.control_state_label.configure(fg=GREEN)
                self.update_control_buttons()
        elif (message in ("MOTOR: STOP", "MODE: MANUAL",
                              "[SAFE] MANUAL DEADMAN: MOTOR STOP",
                              "ERR: ADC_READ_FAULT_MOTORS_STOPPED")
              or message.startswith("AUTO_FINISHED:")
              or message.startswith("AUTO_STOPPED:")):
            self.confirm_stopped(message)
        pid = re.search(r"PID:\s*Kp=([\d.]+)\s+Ki=([\d.]+)\s+Kd=([\d.]+)\s+Base=(\d+)",
                        message, flags=re.IGNORECASE)
        if pid:
            for key, value in zip(("Kp", "Ki", "Kd", "Base"), pid.groups()):
                self.pid_vars[key].set(value)
            self.pwm_var.set(int(pid.group(4)))
            self.preview_pwm()
        if message == "CAL_DUMP_BEGIN":
            self.cal_data = {}
        elif message.startswith("CAL_DATA "):
            parts = message.split()
            if len(parts) == 5:
                try:
                    eye, white, black, threshold = map(int, parts[1:])
                except ValueError:
                    return
                if (0 <= eye < 16 and 0 <= white <= 4095 and
                        0 <= black <= 4095 and black - white >= MIN_CALIB_DELTA and
                        threshold == eye_raw_threshold(white, black)):
                    self.cal_data[eye] = (white, black, threshold)
        elif message == "CAL_DUMP_DONE":
            if len(self.cal_data) == 16:
                self.cal_min = [self.cal_data[i][0] for i in range(16)]
                self.cal_max = [self.cal_data[i][1] for i in range(16)]
                self.cal_thresholds = [self.cal_data[i][2] for i in range(16)]
                self.calibration_ok = True
                self.update_control_buttons()
                # Update the current START/STOP CSV with newly downloaded NVS data.
                if self.run_csv.active is not None:
                    self.run_csv.active.set_calibration(self.cal_min, self.cal_max)
                self.cal_state.set("Đã tải NVS 16/16 mắt; dữ liệu CAL 0–1000 theo ESP32.")
                self.draw_bars(self.last_adc)
            else:
                self.cal_state.set(f"Chưa nhận đủ NVS: {len(self.cal_data)}/16 mắt.")
        elif message == "CAL_WHITE_DONE":
            self.cal_state.set("Đã lấy TRẮNG. Đặt nền ĐEN rồi bấm 2. Lấy ĐEN.")
        elif message == "CAL_WHITE_STARTED":
            self.cal_state.set("Đang thu 128 khung TRẮNG.")
        elif message == "CAL_BLACK_STARTED":
            self.cal_state.set("Đang thu 128 khung ĐEN.")
        elif message == "CAL_OK_SAVED":
            self.cal_state.set("ESP32 đã lưu NVS; đang đọc lại profile 16 mắt.")
            self.calibration_ok = False
            self.cal_thresholds = [None] * 16
            self.cal_data = {}
            self.update_control_buttons()
            self.send_command("CAL_DUMP")
        elif message == "CAL_FAILED":
            self.cal_state.set("Calib mới thất bại; giữ profile cũ nếu có.")
        elif message.startswith("CAL_STATUS"):
            self.cal_state.set(message)
        elif message == "CAL_NEED_WHITE":
            self.cal_state.set("Chưa lấy TRẮNG; bấm 1. Lấy TRẮNG trước.")
        elif message == "CAL_NO_PROFILE":
            self.cal_state.set("ESP32 chưa có profile NVS hợp lệ.")
            self.calibration_ok = False
            self.cal_thresholds = [None] * 16
            self.update_control_buttons()
        elif message.startswith("STREAM Hz="):
            hz = re.search(r"STREAM Hz=(\d+)", message)
            if hz:
                self.stream_rate.set(hz.group(1))

    def tick(self):
        last = None
        # Process safety-critical Bluetooth replies before bulk telemetry.
        try:
            for _ in range(64):
                kind, payload = self.controls_q.get_nowait()
                if kind == "text":
                    self.on_text(payload)
                elif kind == "error":
                    self.disconnect()
                    self.status_label.config(text=payload, fg=ROSE)
                elif kind == "csv_export":
                    self.log(payload)
                elif kind == "csv_closed":
                    self.csv_status.set(payload)
                    self.log(payload)
        except queue.Empty:
            pass
        try:
            for _ in range(512):
                kind, payload = self.q.get_nowait()
                if kind == "data":
                    self.on_sample(payload)
                    last = payload
                elif kind == "csv_export":
                    self.log(payload)
        except queue.Empty:
            pass
        if last is not None:
            self.update_view(last)
        now = time.monotonic()
        self.update_run_timer_display(now)
        if (self.hs_peak_pending is not None and self.hs_peak_pending_at is not None
                and now - self.hs_peak_pending_at >= 3.0):
            expected = self.hs_peak_pending
            self.hs_peak_pending = None
            self.hs_peak_pending_at = None
            self.hs_peak_confirmed = None
            self.hs_peak_feedback.set(
                f"Chưa nhận ACK cho HS_PEAK={expected}; đang đọc lại ESP32.")
            self.log(f"HS_PEAK={expected}: timeout xác nhận, chưa coi là đã áp dụng.")
            self.update_control_buttons()
            self.send_command("HS_PEAK")
        if now - self.rate_time >= 1:
            interval = now - self.rate_time
            total = self.reader.frames_rx if self.reader else self.last_rx_total
            rx_hz = (total - self.last_rx_total) / interval
            app_hz = self.frames / interval
            errors = self.reader.bad if self.reader else 0
            drops = self.reader.frames_dropped if self.reader else 0
            self.rate_label.config(
                text=f"RX {rx_hz:.0f} Hz | APP {app_hz:.0f} Hz | "
                     f"q={self.q.qsize()} dropUI={drops} CRC={errors}")
            if self.run_csv.active is not None:
                rec = self.run_csv.active
                csv_hz = (rec.written - self.last_csv_total) / interval
                self.csv_status.set(
                    f"CSV GHI: {csv_hz:.0f} Hz | tổng={rec.written} "
                    f"đợi={rec.queue.qsize()} dropCSV={rec.dropped}"
                    + (f" LỖI={rec.error}" if rec.error else "")
                    + f" | MAP {rec.map_id} | {rec.path.name}")
                self.last_csv_total = rec.written
            self.last_rx_total = total
            self.frames = 0
            self.rate_time = now
        # One small SPP text request per 2 seconds; no USB traffic.
        if self.next_diag_poll is not None and now >= self.next_diag_poll:
            if self.reader is not None and self.reader.is_alive():
                self.send_command("DIAG")
                self.next_diag_poll = now + self.diag_interval_s
            else:
                self.next_diag_poll = None
        if self.stop_ack_deadline is not None and now >= self.stop_ack_deadline:
            self.handle_stop_timeout()
        if (self.stop_verify_pending and self.stop_verify_deadline is not None
                and now >= self.stop_verify_deadline):
            if self.stop_verify_retries < 3:
                self.stop_verify_retries += 1
                self.stop_verify_deadline = now + 1.5
                self.send_command("STATUS")
            else:
                # No ACK: preserve the interlock; STOP/STATUS are still usable.
                self.stop_verify_deadline = None
                self.control_state.set(
                    "Chưa xác nhận STOP; gửi lại STOP hoặc STATUS, kiểm tra bánh xe.")
                self.control_state_label.configure(fg=ROSE)
        if self.last_diag_rx is not None and now - self.last_diag_rx > 6.0:
            self.loop_debug.set("LOOP: -- Hz    PID: -- Hz    SPP TX: -- Hz    (DIAG quá hạn)")
            self.last_diag_rx = None
        self.after(30, self.tick)

    def update_view(self, s):
        values = {"encL": str(s[17]), "encR": str(s[18]),
                  "spdL": f"{self.wheel_speed[0]:.0f}", "spdR": f"{self.wheel_speed[1]:.0f}",
                  "pwmL": str(s[19]), "pwmR": str(s[20]),
                  "err": f"{s[21]:.2f}", "p": f"{s[22]:.2f}",
                  "i": f"{s[23]:.2f}", "d": f"{s[24]:.2f}",
                  "out": f"{s[25]:.2f}", "mode": {0: "MANUAL", 1: "AUTO PID"}.get(s[26], str(s[26]))}
        for key, value in values.items():
            self.values[key].set(value)
        self.draw_bars(s[1:17])
        self.update_sensor_table()
        self.draw_scope()

    def draw_bars(self, adc_values):
        c = self.bars
        c.delete("all")
        width = max(420, c.winfo_width())
        top, height = 25, 154
        step = width / 16
        bits = []
        for i, adc in enumerate(adc_values):
            low, high = self.cal_min[i], self.cal_max[i]
            cal = normalize_adc(adc, low, high) if self.calibration_ok else None
            black = cal >= BLACK_THRESHOLD_CAL if cal is not None else None
            digit = ("1" if black else "0") if black is not None else "—"
            bits.append(digit)
            x0, x1 = i * step + 4, (i + 1) * step - 4
            bar_height = height * adc / ADC_MAX
            color = GREEN if black is True else GREY if black is False else MUTED
            c.create_rectangle(x0, top + height - bar_height, x1,
                               top + height, fill=color, outline="")
            if self.calibration_ok:
                threshold = eye_raw_threshold(low, high)
                yt = top + height * (1 - threshold / ADC_MAX)
                c.create_line(x0, yt, x1, yt, fill=AMBER, width=2)
            else:
                threshold = None
            c.create_text((x0 + x1) / 2, 12, text=str(adc), fill=FG,
                          font=("Consolas", 8))
            c.create_text((x0 + x1) / 2, 197,
                          text="—" if threshold is None else str(threshold),
                          fill=AMBER, font=("Consolas", 8))
            c.create_text((x0 + x1) / 2, 220,
                          text="—" if cal is None else str(cal),
                          fill=CYAN, font=("Consolas", 8))
            if digit != "—":
                c.create_rectangle(x0+4, 235, x1-4, 255, fill=GREEN if black else "#475569",
                                   outline="")
            c.create_text((x0 + x1) / 2, 245, text=digit, fill=FG,
                          font=("Consolas", 10, "bold"))
            c.create_text((x0 + x1) / 2, 273, text=str(i), fill=MUTED,
                          font=("Consolas", 9))
        if self.calibration_ok:
            self.logic_var.set("0 = TRẮNG | 1 = ĐEN   [" +
                               "".join(bits[:8]) + " | " + "".join(bits[8:]) +
                               "]   (ngưỡng riêng từng mắt, NVS)")
        else:
            self.logic_var.set("Chưa tải được profile NVS: logic chưa xác định (—)")

    def draw_scope(self):
        c = self.scope
        c.delete("all")
        width, height = max(420, c.winfo_width()), int(c["height"])
        mid = height / 2
        c.create_line(0, mid, width, mid, fill="#334155")
        for i, (name, color) in enumerate((("pwmL", CYAN), ("pwmR", AMBER), ("PID out", ROSE))):
            c.create_text(45 + i * 95, 10, text=name, fill=color, font=("Consolas", 9))
        if len(self.hist) < 2:
            return
        for idx, color in ((19, CYAN), (20, AMBER), (25, ROSE)):
            pts = []
            for k, row in enumerate(self.hist):
                x = k * width / (len(self.hist) - 1)
                y = mid - row[idx] / PWM_MAX * (mid - 6)
                pts.extend((x, max(2, min(height - 2, y))))
            c.create_line(*pts, fill=color)

    def on_close(self):
        self.disconnect()
        self.run_csv.close_all()
        if self.run_txt_file is not None:
            self.run_txt_file.close()
            self.run_txt_file = None
        self.session_log_file.close()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()