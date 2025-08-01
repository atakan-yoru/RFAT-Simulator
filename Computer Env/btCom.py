from PyQt5.QtCore import QThread, pyqtSignal
import numpy as np

class BluetoothReader(QThread):
    feedback_received = pyqtSignal(str, float)  # (axis, value)

    def __init__(self, serial_obj, axis, getter_func):
        super().__init__()
        self.serial = serial_obj
        self.axis = axis
        self.getter = getter_func
        self.buffer = ""
        self.running = True

    def run(self):
        if self.serial is None:
            print(f"[BT {self.axis.upper()}] Skipping — serial not available.")
            return

        while self.running:
            try:
                mes_string = self.getter()
                self.serial.write((mes_string+'\n').encode())
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
            self.msleep(2)

    def stop(self):
        self.running = False
        self.wait()

