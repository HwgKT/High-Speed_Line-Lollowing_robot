"""One persistent 300-Hz CSV per START/STOP run, never tied to SPP pairing."""
import threading
from spp_csv_recorder import TelemetryCsvRecorder

class RunCsvLifecycle:
    def __init__(self, notifications):
        self.notifications = notifications
        self.active = None
        self.last = None
        self.closers = []

    def begin(self, reader, header, calibration, thresholds, map_id):
        """Open a CSV BEFORE sending AUTO; the reader immediately starts logging."""
        if self.active is not None:
            raise RuntimeError("A CSV run is already open")
        if calibration is None:
            raise ValueError("Complete calibration profile is required")
        recorder = TelemetryCsvRecorder(header, thresholds, map_id)
        white, black = calibration
        recorder.set_calibration(white, black)
        try:
            recorder.start()
        except Exception:
            recorder.output.close()
            raise
        self.active = recorder
        self.last = recorder
        reader.recorder = recorder
        return recorder

    def finish(self, reader, reason):
        """Detach instantly and close asynchronously after draining received samples."""
        recorder = self.active
        if recorder is None:
            return None
        if reader is not None:
            reader.recorder = None
        self.active = None
        recorder.recording = False

        def close_worker():
            try:
                recorder.stop()
                result = (f"CSV DA DONG [{reason}]: {recorder.path} | "
                          f"rows={recorder.written} dropCSV={recorder.dropped}")
                if recorder.error:
                    result += f" WRITE_ERROR={recorder.error}"
            except Exception as exc:
                result = f"CSV CLOSE FAILED [{reason}]: {recorder.path}: {exc}"
            self.notifications.put(("csv_closed", result))

        job = threading.Thread(target=close_worker, daemon=True, name="csv-close-run")
        self.closers.append(job)
        job.start()
        return recorder

    def close_all(self):
        for job in self.closers:
            job.join()
