# gui.py

import tkinter as tk
from tkinter import ttk
import math

from communication import TeensyComm
from mock_com import MockComm

class FingerGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Planar Two-Joint Finger")

        # 1) Serial port
        self.comm = TeensyComm(port="/dev/tty0", baudrate=115200)
        # self.comm = MockComm()
        # 2) GUI layout
        self.create_widgets()

        # 3) Update serial info 
        self.update_serial()

    def create_widgets(self):
        # ========== buttons ==========
        btn_frame = tk.Frame(self.master)
        btn_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        connect_btn = tk.Button(btn_frame, text="Connect", command=self.connect_serial)
        connect_btn.pack(side=tk.LEFT, padx=5)

        disconnect_btn = tk.Button(btn_frame, text="Disconnect", command=self.disconnect_serial)
        disconnect_btn.pack(side=tk.LEFT, padx=5)

        home_btn = tk.Button(btn_frame, text="HOME", command=self.send_home)
        home_btn.pack(side=tk.LEFT, padx=5)

        stop_btn = tk.Button(btn_frame, text="STOP", command=self.send_stop)
        stop_btn.pack(side=tk.LEFT, padx=5)

        # ========== bars ==========
        # TODO The logic here need to be more clear.
        slider_frame = tk.Frame(self.master)
        slider_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        self.joint1_scale = tk.Scale(slider_frame, from_=-90, to=90,
                                     orient=tk.HORIZONTAL, label="Joint1 (deg)")
        self.joint1_scale.pack(fill=tk.X)

        self.joint2_scale = tk.Scale(slider_frame, from_=-90, to=90,
                                     orient=tk.HORIZONTAL, label="Joint2 (deg)")
        self.joint2_scale.pack(fill=tk.X)

        # ========== Commend input ==========
        entry_frame = tk.Frame(self.master)
        entry_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        tk.Label(entry_frame, text="Send Command:").pack(side=tk.LEFT)
        self.cmd_entry = tk.Entry(entry_frame)
        self.cmd_entry.pack(side=tk.LEFT, padx=5)
        send_btn = tk.Button(entry_frame, text="Send", command=self.send_cmd)
        send_btn.pack(side=tk.LEFT, padx=5)

        # ========== Log info ==========
        log_frame = tk.Frame(self.master)
        log_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.log_text = tk.Text(log_frame, height=20, width=40)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # ========== visualize ==========
        canvas_frame = tk.Frame(self.master)
        canvas_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self.canvas = tk.Canvas(canvas_frame, bg="white", width=300, height=300)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # TODO: Adding direct torque input

    def connect_serial(self):
        self.comm.open_serial()
        self.log("[GUI] Serial Connecting...")

    def disconnect_serial(self):
        self.comm.close_serial()
        self.log("[GUI] Serial Disconnected.")

    def send_home(self):
        """sending home position direction"""
        self.comm.write_data("HOME")
        self.log("[GUI] Commend Sending: HOME")

    def send_stop(self):
        """sending home position direction"""
        self.comm.write_data("STOP")
        self.log("[GUI] Commend Sending: STOP")

    def send_cmd(self):
        """sending commend from the text box"""
        cmd = self.cmd_entry.get()
        if cmd:
            self.comm.write_data(cmd)
            self.log(f"[GUI] Commend Sending: {cmd}")

    def update_serial(self):
        """
        Read info from teensy to update the GUI log and visualize

        """
        line = self.comm.read_data()
        if line:
            # Assume the info from the teensy is "45.0,30.0". Joint 1 and joint 2 position

            parts = line.split(',')
            if len(parts) == 2:
                try:
                    j1 = float(parts[0])
                    j2 = float(parts[1])
                    self.update_joint_display(j1, j2)
                except ValueError:
                    # TODO This can be change to better error raise
                    pass  # If the decode is wrong, just ignore

        # next update time.
        self.master.after(50, self.update_serial)

    def update_joint_display(self, j1, j2):
        """
        update the bar and finger joint.
        """
        self.joint1_scale.set(j1)
        self.joint2_scale.set(j2)
        self.log(f"[GUI] Joint1={j1:.1f}, Joint2={j2:.1f}")
        self.draw_finger(j1, j2)

    def draw_finger(self, j1, j2):
        """
        Draw the simple two joint finger on the canvas 
        """
        self.canvas.delete("all")
        
        # TODO The value are selected randomly, replace with the true value
        # link length
        L1, L2 = 80, 60
        # base poisition 
        x0, y0 = 150, 200

        # joint 2 pos
        x1 = x0 + L1 * math.cos(math.radians(j1))
        y1 = y0 - L1 * math.sin(math.radians(j1))

        # End of link 2
        total_angle = j1 + j2
        x2 = x1 + L2 * math.cos(math.radians(total_angle))
        y2 = y1 - L2 * math.sin(math.radians(total_angle))

        # link 1
        self.canvas.create_line(x0, y0, x1, y1, fill="red", width=5)
        # link 2
        self.canvas.create_line(x1, y1, x2, y2, fill="blue", width=5)
        # joints
        self.canvas.create_oval(x1 - 5, y1 - 5, x1 + 5, y1 + 5, fill="green")
        self.canvas.create_oval(x2 - 5, y2 - 5, x2 + 5, y2 + 5, fill="green")

    def log(self, msg):
        """
        Log data
        roll to the bottom of the windows.
        """
        self.log_text.insert(tk.END, msg + "\n")
        self.log_text.see(tk.END)

def main():
    root = tk.Tk()
    app = FingerGUI(root)
    root.mainloop()

if __name__ == '__main__':
    main()
