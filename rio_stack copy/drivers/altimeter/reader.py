import threading
import serial
import time
from drivers.altimeter.decoder import AltimeterDecoder
from drivers.altimeter.quality_filter import AltimeterQualityFilter

class AltimeterReader:
    def __init__(self, port='/dev/ttyUSB1', baud=115200):
        self.port = port
        self.baud = baud
        self.decoder = AltimeterDecoder()
        self.filter = AltimeterQualityFilter()
        self.latest_sample = None
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self._stop.set()
        
    def read(self):
        s = self.latest_sample
        self.latest_sample = None
        return s

    def _run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.5)
        except Exception as e:
            print(f"AltimeterReader failed to open {self.port}: {e}")
            return
            
        while not self._stop.is_set():
            sample = self.decoder.read_from_serial(ser)
            if sample and sample.checksum_valid:
                filtered = self.filter.process(sample)
                if filtered and filtered.quality > 0:
                    self.latest_sample = filtered
        ser.close()
