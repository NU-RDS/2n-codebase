import tkinter as tk
import math
import time
import threading
import numpy as np

from serial_com import MotorSerialInterface
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class FingerGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Planar Two-Joint Finger")

        self.msi = None  # MotorSerialInterface instance (created after user clicks Connect)

        # Define DoubleVar for joint1 and joint2 (user input in degrees)
        self.j1_var = tk.DoubleVar(value=0.0)
        self.j2_var = tk.DoubleVar(value=0.0)
        
        # Home position (in radians); here we set home as [0, 0]
        self.joint_home = [0.0, 0.0]
        
        # Current joint state (in radians)
        self.joint_current = None

        # For plotting: record time and data for real-time plots
        self.start_time = time.time()
        self.time_data = []
        self.joint1_actual_data = []
        self.joint1_desired_data = []
        self.joint2_actual_data = []
        self.joint2_desired_data = []

        # Create the control area (top part)
        self.create_control_area()

        # Create the bottom area with two columns: left for log, right for plots and canvas.
        bottom_frame = tk.Frame(self.master)
        bottom_frame.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

        # Left side: Log area
        self.log_text = tk.Text(bottom_frame, height=15, width=40)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Right side: A frame containing a small canvas (for finger drawing) on top and a real-time plot below
        right_frame = tk.Frame(bottom_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # Top of right_frame: Canvas for drawing the finger (smaller height)
        self.canvas = tk.Canvas(right_frame, bg="white")
        self.canvas.grid(row=0, column=0, sticky="nsew")

        # Bottom of right_frame: Plot frame for real-time joint plots
        self.plot_frame = tk.Frame(right_frame)
        self.plot_frame.grid(row=1, column=0, sticky="nsew")

        # Configure right_frame so that the canvas and plot each take half of the available height
        right_frame.rowconfigure(0, weight=1)
        right_frame.rowconfigure(1, weight=1)
        right_frame.columnconfigure(0, weight=1)

        # Create a matplotlib Figure with two subplots (for Joint1 and Joint2)
        self.fig = Figure(figsize=(5, 3), dpi=100)
        self.ax1 = self.fig.add_subplot(211)
        self.ax2 = self.fig.add_subplot(212)
        self.fig.tight_layout()

        self.canvas_fig = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas_fig.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Start updating the plots every 100 ms
        self.update_plots()

    def create_control_area(self):
        # Create the top control panel area
        top_frame = tk.Frame(self.master)
        top_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        # Button area
        btn_frame = tk.Frame(top_frame)
        btn_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        connect_btn = tk.Button(btn_frame, text="Connect", command=self.connect_serial)
        connect_btn.pack(side=tk.LEFT, padx=5)

        disconnect_btn = tk.Button(btn_frame, text="Disconnect", command=self.disconnect_serial)
        disconnect_btn.pack(side=tk.LEFT, padx=5)

        home_btn = tk.Button(btn_frame, text="HOME", command=self.send_home)
        home_btn.pack(side=tk.LEFT, padx=5)

        stop_btn = tk.Button(btn_frame, text="STOP", command=self.send_stop)
        stop_btn.pack(side=tk.LEFT, padx=5)

        # Joint Position input area (slider + text entry; both share the same variable)
        pos_frame = tk.LabelFrame(top_frame, text="Set Joint Position (deg)")
        pos_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        tk.Label(pos_frame, text="Joint1:", font=("Arial", 10)).grid(row=0, column=0, padx=5, pady=2)
        self.joint1_scale = tk.Scale(pos_frame, from_=0, to=90, orient=tk.HORIZONTAL,
                                      variable=self.j1_var, length=300)
        self.joint1_scale.grid(row=0, column=1, padx=5, pady=2)
        self.joint1_entry = tk.Entry(pos_frame, width=6, textvariable=self.j1_var)
        self.joint1_entry.grid(row=0, column=2, padx=5, pady=2)

        tk.Label(pos_frame, text="Joint2:", font=("Arial", 10)).grid(row=1, column=0, padx=5, pady=2)
        self.joint2_scale = tk.Scale(pos_frame, from_=-90, to=90, orient=tk.HORIZONTAL,
                                      variable=self.j2_var, length=300)
        self.joint2_scale.grid(row=1, column=1, padx=5, pady=2)
        self.joint2_entry = tk.Entry(pos_frame, width=6, textvariable=self.j2_var)
        self.joint2_entry.grid(row=1, column=2, padx=5, pady=2)

        send_pos_btn = tk.Button(pos_frame, text="Send Position", command=self.send_position_cmd)
        send_pos_btn.grid(row=2, column=0, columnspan=3, pady=5)

    def connect_serial(self):
        if self.msi:
            self.log("[GUI] Already connected.")
            return
        try:
            self.msi = MotorSerialInterface(port="/dev/teensy4", baud_rate=115200, timeout=1)
            self.log("[GUI] Connected to /dev/teensy4.")
            # Initialize joint state variables
            self.joint_home = [0.0, 0.0]
            self.joint_current = self.joint_home
        except Exception as e:
            self.log(f"[GUI] Connect error: {e}")
            self.msi = None

    def disconnect_serial(self):
        if self.msi:
            self.msi.close()
            self.msi = None
            self.log("[GUI] Disconnected.")
        else:
            self.log("[GUI] No interface to disconnect.")

    def send_home(self):
        if not self.msi:
            self.log("[GUI] Not connected.")
            return
        try:
            self.msi.set_joint_positions(self.joint_home)
            self.log("[GUI] Sent HOME command (joint positions set to 0).")
        except Exception as e:
            self.log(f"[GUI] Error sending HOME command: {e}")

    def send_stop(self):
        if not self.msi:
            self.log("[GUI] Not connected.")
            return
        try:
            current = self.msi.get_joint_states()
            self.msi.set_joint_positions(current)
            self.log("[GUI] Sent STOP command (holding current joint positions).")
        except Exception as e:
            self.log(f"[GUI] Error sending STOP command: {e}")

    def send_position_cmd(self):
        if not self.msi:
            self.log("[GUI] Not connected.")
            return
        # Get desired joint positions (in degrees) from the UI
        j1_deg = self.j1_var.get()
        j2_deg = self.j2_var.get()
        # Convert to radians
        j1_rad = math.radians(j1_deg)
        j2_rad = math.radians(j2_deg)
        desired_positions = [j1_rad, j2_rad]
        try:
            self.msi.set_joint_positions(desired_positions)
            self.log(f"[GUI] Sent Position: Joint1={j1_deg}°, Joint2={j2_deg}°")
        except Exception as e:
            self.log(f"[GUI] Error sending position command: {e}")

    def update_gui(self):
        if self.msi:
            joint_states = self.msi.get_joint_states()  # Returns a tuple of 2 floats (radians)
            if joint_states is not None and len(joint_states) >= 2:
                j1_actual = joint_states[0]
                j2_actual = joint_states[1]
                self.joint_current = [j1_actual, j2_actual]
                self.draw_finger(j1_actual, j2_actual)
                # Update plot data
                t = time.time() - self.start_time
                self.time_data.append(t)
                self.joint1_actual_data.append(math.degrees(j1_actual))
                self.joint2_actual_data.append(math.degrees(j2_actual))
                self.joint1_desired_data.append(self.j1_var.get())
                self.joint2_desired_data.append(self.j2_var.get())
        self.master.after(50, self.update_gui)

    def draw_finger(self, j1, j2):
        self.canvas.delete("all")
        # Link lengths and base position
        L1, L2 = 80, 60
        x0, y0 = 150, 200
        # Assume j1 and j2 are in radians
        r1 = j1
        r2 = j2
        x1 = x0 + L1 * math.cos(r1)
        y1 = y0 - L1 * math.sin(r1)
        x2 = x1 + L2 * math.cos(r1 + r2)
        y2 = y1 - L2 * math.sin(r1 + r2)
        self.canvas.create_line(x0, y0, x1, y1, fill="red", width=5)
        self.canvas.create_line(x1, y1, x2, y2, fill="blue", width=5)
        self.canvas.create_oval(x1-5, y1-5, x1+5, y1+5, fill="green")
        self.canvas.create_oval(x2-5, y2-5, x2+5, y2+5, fill="green")
        # Display current joint angles in the top-right corner (converted to degrees)
        j1_deg = math.degrees(j1)
        j2_deg = math.degrees(j2)
        self.canvas.create_text(
            290, 10,
            text=f"Joint1={j1_deg:.1f}°\nJoint2={j2_deg:.1f}°",
            fill="black",
            anchor='ne',
            font=("Arial", 16, "bold")
        )

    def update_plots(self):
        # Clear previous plot content
        self.ax1.clear()
        self.ax2.clear()

        # Plot Joint1 data
        self.ax1.plot(self.time_data, self.joint1_actual_data, label="Actual Joint1")
        self.ax1.plot(self.time_data, self.joint1_desired_data, label="Desired Joint1", linestyle="--")
        self.ax1.set_title("Joint1")
        self.ax1.set_xlabel("Time (s)")
        self.ax1.set_ylabel("Angle (deg)")
        self.ax1.legend()

        # Plot Joint2 data
        self.ax2.plot(self.time_data, self.joint2_actual_data, label="Actual Joint2")
        self.ax2.plot(self.time_data, self.joint2_desired_data, label="Desired Joint2", linestyle="--")
        self.ax2.set_title("Joint2")
        self.ax2.set_xlabel("Time (s)")
        self.ax2.set_ylabel("Angle (deg)")
        self.ax2.legend()

        self.fig.tight_layout()
        self.canvas_fig.draw()
        self.master.after(100, self.update_plots)

    def log(self, msg):
        self.log_text.insert(tk.END, msg + "\n")
        self.log_text.see(tk.END)

    def on_closing(self):
        if self.msi:
            self.msi.close()
            self.msi = None
        self.master.destroy()

def main():
    root = tk.Tk()
    app = FingerGUI(root)
    root.mainloop()

if __name__ == '__main__':
    main()
