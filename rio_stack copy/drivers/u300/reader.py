import threading
import serial
import time
import struct
import numpy as np
from drivers.u300.decoder import U300Decoder, MAGIC_WORD
from estimator.state import RadarFrame

class U300Reader:
    def __init__(self, port='/dev/ttyUSB0', baud=921600, adapter=None):
        self.port = port
        self.baud = baud
        self.adapter = adapter
        self.decoder = U300Decoder()
        self.latest_frame = None
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self._stop.set()
        
    def read(self):
        f = self.latest_frame
        self.latest_frame = None
        return f

    def _run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.5)
        except Exception as e:
            print(f"U300Reader failed to open {self.port}: {e}")
            return
            
        while not self._stop.is_set():
            frame_data = self._read_one_frame(ser)
            if frame_data:
                res = self.decoder.decode_frame(frame_data)
                if res:
                    pts, health = res
                    if self.adapter:
                        self.latest_frame = self.adapter.convert_frame(pts, health.rx_timestamp)
        ser.close()

    def _read_one_frame(self, ser) -> bytearray | None:
        index = 0
        magic_byte = ser.read(1)
        frame_data = bytearray(b'')
        if not magic_byte: return None
        
        while not self._stop.is_set():
            if not magic_byte:
                magic_byte = ser.read(1)
                continue
            if magic_byte[0] == MAGIC_WORD[index]:
                index += 1
                frame_data.append(magic_byte[0])
                if index == 8: break
                magic_byte = ser.read(1)
            else:
                if index == 0: magic_byte = ser.read(1)
                index = 0
                frame_data = bytearray(b'')

        version_bytes = ser.read(4)
        frame_data += bytearray(version_bytes)
        length_bytes = ser.read(4)
        frame_data += bytearray(length_bytes)
        frame_length = int.from_bytes(length_bytes, byteorder='little')
        frame_length -= 16
        if frame_length < 0 or frame_length > 1 << 20: return None
        
        payload = bytearray(b'')
        while len(payload) < frame_length and not self._stop.is_set():
            chunk = ser.read(frame_length - len(payload))
            if chunk: payload += bytearray(chunk)
            else: return None
            
        frame_data += payload
        return frame_data
