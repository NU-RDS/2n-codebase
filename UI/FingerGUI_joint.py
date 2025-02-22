import tkinter as tk
import math
import time
import threading
import numpy as np

from serial_com_han import MotorSerialInterface

class FingerGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Planar Two-Joint Finger")

        self.msi = None  # MotorSerialInterface instance (created after user clicks Connect)

        # Define DoubleVar for joint1 and joint2 (user inputs in degrees)
        self.j1_var = tk.DoubleVar(value=0.0)
        self.j2_var = tk.DoubleVar(value=0.0)
        
        # Define home position (in radians); here we set home as zero.
        self.joint_home = [0.0, 0.0]
        
        # Store last joint state if needed
        self.joint_last = None
        self.joint_current = None

        # Build the interface
        self.create_widgets()

        # Periodically refresh the GUI (read joint states, update canvas, etc.)
        self.update_gui()

    def create_widgets(self):
        # --- Top button area: Connect, Disconnect, HOME, STOP ---
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

        # --- Joint Position input (slider + text entry) ---
        pos_frame = tk.LabelFrame(self.master, text="Set Joint Position (deg)")
        pos_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        tk.Label(pos_frame, text="Joint1 (deg):", font=("Arial",10)).grid(row=0, column=0, padx=5, pady=2)
        self.joint1_scale = tk.Scale(pos_frame, from_=-90, to=90, orient=tk.HORIZONTAL,
                                      variable=self.j1_var, length=300)
        self.joint1_scale.grid(row=0, column=1, padx=5, pady=2)
        self.joint1_entry = tk.Entry(pos_frame, width=6, textvariable=self.j1_var)
        self.joint1_entry.grid(row=0, column=2, padx=5, pady=2)

        tk.Label(pos_frame, text="Joint2 (deg):", font=("Arial",10)).grid(row=1, column=0, padx=5, pady=2)
        self.joint2_scale = tk.Scale(pos_frame, from_=-90, to=90, orient=tk.HORIZONTAL,
                                      variable=self.j2_var, length=300)
        self.joint2_scale.grid(row=1, column=1, padx=5, pady=2)
        self.joint2_entry = tk.Entry(pos_frame, width=6, textvariable=self.j2_var)
        self.joint2_entry.grid(row=1, column=2, padx=5, pady=2)

        send_pos_btn = tk.Button(pos_frame, text="Send Position", command=self.send_position_cmd)
        send_pos_btn.grid(row=2, column=0, columnspan=3, pady=5)

        # --- Log display ---
        self.log_text = tk.Text(self.master, height=10, width=40)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # --- Canvas: draw finger & display current joint angles ---
        self.canvas = tk.Canvas(self.master, bg="white", width=300, height=300)
        self.canvas.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)

    # --- Serial connection/disconnection ---
    def connect_serial(self):
        if self.msi:
            self.log("[GUI] Already connected.")
            return
        try:
            self.msi = MotorSerialInterface(port="/dev/teensy4", baud_rate=115200, timeout=1)
            self.log("[GUI] Connected to /dev/teensy4.")
            # Initialize home and state variables
            self.joint_home = [0.0, 0.0]
            self.joint_current = self.joint_home
            self.joint_last = self.joint_home
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

    # --- HOME: send home position command ---
    def send_home(self):
        if not self.msi:
            self.log("[GUI] Not connected.")
            return
        try:
            self.msi.set_joint_positions(self.joint_home)
            self.log("[GUI] Sent HOME command (joint positions set to 0).")
        except Exception as e:
            self.log(f"[GUI] Error sending HOME command: {e}")

    # --- STOP: hold current position ---
    def send_stop(self):
        if not self.msi:
            self.log("[GUI] Not connected.")
            return
        try:
            current_joints = self.msi.get_joint_states()
            self.msi.set_joint_positions(current_joints)
            self.log("[GUI] Sent STOP command (holding current joint positions).")
        except Exception as e:
            self.log(f"[GUI] Error sending STOP command: {e}")

    # --- Send joint position command ---
    # Convert desired positions from degrees to radians, then send via set_joint_positions.
    def send_position_cmd(self):
        if not self.msi:
            self.log("[GUI] Not connected.")
            return
        # Get desired joint positions (in degrees) from UI
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

    # --- Periodic refresh: Read joint states and update canvas ---
    def update_gui(self):
        if self.msi:
            joint_states = self.msi.get_joint_states()  # returns a tuple of 2 float values (radians)
            if joint_states is not None and len(joint_states) >= 2:
                j1_actual = joint_states[0]
                j2_actual = joint_states[1]
                self.draw_finger(j1_actual, j2_actual)
        self.master.after(50, self.update_gui)

    def draw_finger(self, j1, j2):
        self.canvas.delete("all")
        # Example link lengths and base position
        L1, L2 = 80, 60
        x0, y0 = 150, 200
        # Assume j1, j2 are in radians
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
        # Display current joint angles (convert from radians to degrees)
        j1_deg = math.degrees(j1)
        j2_deg = math.degrees(j2)
        self.canvas.create_text(
            290, 10,
            text=f"Joint1={j1_deg:.1f}°\nJoint2={j2_deg:.1f}°",
            fill="black",
            anchor='ne',
            font=("Arial", 16, "bold")
        )

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
