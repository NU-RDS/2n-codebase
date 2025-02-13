# communication.py

import serial
import time

class TeensyComm:
    def __init__(self, port='/dev/teensy4', baudrate=115200, timeout=0.01):
        """
        Leave the GUI to open the port.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None

    def open_serial(self):
        """open serial"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            print(f"[TeensyComm] Serial {self.port} open.")
        except Exception as e:
            print(f"[TeensyComm] Fail to open serial: {e}")

    def close_serial(self):
        """close serial """
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[TeensyComm] Serial Close.")

    def is_open(self):
        """See the serial is still open or not."""
        return (self.ser is not None) and self.ser.is_open

    def read_data(self):
        """
        Read from serial and return strings.
        Return "" if not read anything.
        """
        if self.is_open():
            try:
                line = self.ser.readline().decode('utf-8').strip()
                return line
            except Exception as e:
                print(f"[TeensyComm] Reading error: {e}")
                return ""
        return ""

    def write_data(self, msg: str):
        """
        path the string to teensy.
        """
        if self.is_open():
            try:
                self.ser.write((msg + "\n").encode('utf-8'))
                # wait to write next commend.
                time.sleep(0.001)
            except Exception as e:
                print(f"[TeensyComm] Writing error: {e}")
