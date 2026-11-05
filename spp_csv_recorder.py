"""Background, loss-aware CSV recording of SPP telemetry, independent of Tkinter."""
import csv
import datetime
from pathlib import Path
import queue
import threading
import time

class TelemetryCsvRecorder(threading.Thread):
    def __init__(self, header, thresholds, map_id, logs_root=None):
        super().__init__(name="spp-csv-writer", daemon=True)
        if len(thresholds) != 16 or any(
                not isinstance(v, int) or not 0 <= v <= 4095
                for v in thresholds):
            raise ValueError("Need all 16 raw calibration thresholds from CAL_DUMP")
        if len(header) < 18 or header[:2] != ["host_iso", "t_ms"] or (
                header[2:18] != [f"adc{i}" for i in range(16)]):
            raise ValueError("Unexpected CSV header alignment")
        if type(map_id) is not int or map_id not in (1, 2, 3):
            raise ValueError("Select MAP 1, MAP 2 or MAP 3 before recording")
        self.map_id = map_id
        root = Path(logs_root) if logs_root is not None else Path(__file__).resolve().parent / "logs"
        folder = root / f"map_{map_id:02d}"
        folder.mkdir(parents=True, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.path = folder / f"linerobot_map{map_id:02d}_{stamp}.csv"
        self.output = self.path.open("w", newline="", encoding="utf-8-sig")
        self.writer = csv.writer(self.output)
        self.writer.writerow(header)
        # Single metadata row: 16 raw thresholds aligned under adc0..adc15.
        self.writer.writerow(
            ["CAL_THRESHOLDS", "", *thresholds] + [""] * (len(header) - 18))
        self.output.flush()
        self.queue = queue.Queue(maxsize=8192)
        self.file_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.recording = True
        self.calibration = None
        self.enqueued = 0
        self.written = 0
        self.dropped = 0
        self.error = None

    def set_calibration(self, white=None, black=None):
        if white is None or black is None:
            self.calibration = None
        else:
            self.calibration = (tuple(white), tuple(black))

    def submit(self, sample):
        if not self.recording or self.stop_event.is_set() or self.error:
            return
        received_at = datetime.datetime.now().astimezone().isoformat(
            timespec="milliseconds")
        try:
            self.queue.put_nowait((received_at, sample))
            self.enqueued += 1
        except queue.Full:
            self.dropped += 1

    def run(self):
        last_flush = time.monotonic()
        try:
            while not self.stop_event.is_set() or not self.queue.empty():
                try:
                    received_at, sample = self.queue.get(timeout=0.15)
                except queue.Empty:
                    pass
                else:
                    raw = list(sample[1:17])
                    profile = self.calibration
                    if profile is None:
                        calibrated = [""] * 16
                        logic = [""] * 16
                        logic_word = ""
                    else:
                        white, black = profile
                        calibrated = [
                            max(0, min(1000, ((raw[i] - white[i]) * 1000 //
                                (black[i] - white[i])))) if black[i] - white[i] >= 150 else 0
                            for i in range(16)
                        ]
                        logic = [int(value >= 400) for value in calibrated]
                        logic_word = "".join(str(bit) for bit in logic)
                    row = ([received_at, sample[0], *raw, *calibrated, *logic,
                            *sample[17:27], logic_word])
                    with self.file_lock:
                        self.writer.writerow(row)
                        self.written += 1
                    self.queue.task_done()
                if time.monotonic() - last_flush >= 1:
                    with self.file_lock:
                        self.output.flush()
                    last_flush = time.monotonic()
        except (OSError, ValueError, csv.Error) as exc:
            self.error = str(exc)
        finally:
            with self.file_lock:
                try:
                    self.output.flush()
                finally:
                    self.output.close()

    def export_snapshot(self, destination):
        destination = Path(destination)
        if destination.resolve() == self.path.resolve():
            raise ValueError("Khong duoc ghi de file CSV dang thu thap")
        with self.file_lock:
            if not self.output.closed:
                self.output.flush()
            size = self.path.stat().st_size
            count = self.written
        with self.path.open("rb") as source, destination.open("wb") as target:
            left = size
            while left:
                chunk = source.read(min(1024 * 1024, left))
                if not chunk:
                    raise OSError("File CSV source was truncated during export")
                target.write(chunk)
                left -= len(chunk)
        return count

    def stop(self):
        self.recording = False
        self.stop_event.set()
        self.join()
