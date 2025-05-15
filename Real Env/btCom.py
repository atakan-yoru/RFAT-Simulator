from PyQt5.QtCore import QThread, pyqtSignal
import numpy as np

class BluetoothReader(QThread):
    feedback_received = pyqtSignal(str, float)  # (axis, value)

    def __init__(self, serial_obj, axis, get_speed_func, search_mode):
        super().__init__()
        self.serial = serial_obj
        self.axis = axis
        self.get_speed = get_speed_func
        self.search_mode = search_mode
        self.buffer = ""
        self.running = True

    def run(self):
        if self.serial is None:
            print(f"[BT {self.axis.upper()}] Skipping — serial not available.")
            return

        while self.running:
            try:
                speed = self.get_speed()
                self.serial.write(f"{self.search_mode},{speed:.2f}\n".encode())
                data = self.serial.read(self.serial.in_waiting).decode(errors='ignore')
                self.buffer += data

                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    try:
                        val = float(line.strip())
                        if np.isfinite(val):
                            self.feedback_received.emit(self.axis, val)
                    except ValueError:
                        pass

            except Exception as e:
                print(f"[BT {self.axis.upper()}] Error: {e}")
            self.msleep(10)

    def stop(self):
        self.running = False
        self.wait()

