import tkinter as tk
import math
import time
import threading
import numpy as np

from serial_com import MotorSerialInterface
from controller import Controller 
class FingerGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Planar Two-Joint Finger")

        self.msi = None  # MotorSerialInterface instance (created after user clicks Connect)
        # create a Controller 
        self.controller = Controller()
        # TODO This we need to talk about.
        # The controller will create a new MSI,
        # But better just use one MSI. So here I will give the controller the Finger GUI MSI.

        # 1) Define DoubleVar for joint1 and joint2: shared by sliders & text entries
        self.j1_var = tk.DoubleVar(value=0.0)
        self.j2_var = tk.DoubleVar(value=0.0)
        # Joint [J2, J1]. 
        # J2 is the pip 
        # J1 is the mip
        self.joint_home = None
        
        # motor last status (position), update after each torque_ctrl
        self.motor_last = None
        self.joint_last = None 
        
        self.motor_current = None 
        self.joint_current = None 

        # Build the interface
        self.create_widgets()

        # Periodically refresh the GUI (read motor states, update canvas, etc.)
        self.update_gui()

    def create_widgets(self):
        # -------------- Top button area (Connect/Disconnect/Home/Stop) --------------
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

        # -------------- Joint Position input (slider + text entry) --------------
        pos_frame = tk.LabelFrame(self.master, text="Set Joint Position")
        pos_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        tk.Label(pos_frame, text="Joint1 (deg):", font=("Arial",10)).grid(row=0, column=0, padx=5, pady=2)
        self.joint1_scale = tk.Scale(pos_frame, from_=-90, to=90,
                                     orient=tk.HORIZONTAL,
                                     variable=self.j1_var,
                                     length=300)  # Adjust length for better dragging
        self.joint1_scale.grid(row=0, column=1, padx=5, pady=2)
        self.joint1_entry = tk.Entry(pos_frame, width=6,
                                     textvariable=self.j1_var)  # Shared variable with slider
        self.joint1_entry.grid(row=0, column=2, padx=5, pady=2)

        tk.Label(pos_frame, text="Joint2 (deg):", font=("Arial",10)).grid(row=1, column=0, padx=5, pady=2)
        self.joint2_scale = tk.Scale(pos_frame, from_=-90, to=90,
                                     orient=tk.HORIZONTAL,
                                     variable=self.j2_var,
                                     length=300)
        self.joint2_scale.grid(row=1, column=1, padx=5, pady=2)
        self.joint2_entry = tk.Entry(pos_frame, width=6,
                                     textvariable=self.j2_var)
        self.joint2_entry.grid(row=1, column=2, padx=5, pady=2)

        send_pos_btn = tk.Button(pos_frame, text="Send Position", command=self.send_position_cmd)
        send_pos_btn.grid(row=2, column=0, columnspan=3, pady=5)

        # -------------- 4-channel motor torque input --------------
        torque_frame = tk.LabelFrame(self.master, text="Motor Torques")
        torque_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        tk.Label(torque_frame, text="Motor0:").grid(row=0, column=0, padx=5, pady=2)
        self.torque0_entry = tk.Entry(torque_frame, width=6)
        self.torque0_entry.insert(0, "0.0")
        self.torque0_entry.grid(row=0, column=1, padx=5, pady=2)

        tk.Label(torque_frame, text="Motor1:").grid(row=0, column=2, padx=5, pady=2)
        self.torque1_entry = tk.Entry(torque_frame, width=6)
        self.torque1_entry.insert(0, "0.0")
        self.torque1_entry.grid(row=0, column=3, padx=5, pady=2)

        tk.Label(torque_frame, text="Motor2:").grid(row=1, column=0, padx=5, pady=2)
        self.torque2_entry = tk.Entry(torque_frame, width=6)
        self.torque2_entry.insert(0, "0.0")
        self.torque2_entry.grid(row=1, column=1, padx=5, pady=2)

        tk.Label(torque_frame, text="Motor3:").grid(row=1, column=2, padx=5, pady=2)
        self.torque3_entry = tk.Entry(torque_frame, width=6)
        self.torque3_entry.insert(0, "0.0")
        self.torque3_entry.grid(row=1, column=3, padx=5, pady=2)

        # >>>>>>>>>>> Added "Duration" input <<<<<<<<<<<
        tk.Label(torque_frame, text="Duration (s):").grid(row=2, column=0, padx=5, pady=2)
        self.torque_duration_entry = tk.Entry(torque_frame, width=6)
        self.torque_duration_entry.insert(0, "5")  # Default duration: 5 seconds
        self.torque_duration_entry.grid(row=2, column=1, padx=5, pady=2)

        send_torque_btn = tk.Button(torque_frame, text="Send Torque", command=self.send_torque_cmd)
        send_torque_btn.grid(row=2, column=2, columnspan=2, pady=5)

        # -------------- Log display --------------
        self.log_text = tk.Text(self.master, height=10, width=40)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # -------------- Canvas (draw finger + display current joint angles in top-right corner) --------------
        self.canvas = tk.Canvas(self.master, bg="white", width=300, height=300)
        self.canvas.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)

    # -------------------------------------------------------
    #   Serial connection / disconnection
    # -------------------------------------------------------
    def connect_serial(self):
        if self.msi:
            self.log("[GUI] Already connected.")
            return
        try:
            self.msi = MotorSerialInterface(port="/dev/teensy4", baud_rate=115200, timeout=0.5)
            self.log("[GUI] Connected to /dev/teensy4.")
            # Connect the controller. 
            # give the controller the GUI msi.
            self.controller.MSI = self.msi
            # Do home calibration 
            self.motor_home, tensioned = self.controller.tension()
            self.motor_home = self.controller.go_home(tensioned)
            # set this configuration as the joint home.
            self.joint_home = [0.0, 0.0]
            # Update the last motor position and joint position 
            self.motor_current = self.motor_home
            self.joint_current = self.joint_home
            self.motor_last = self.motor_current
            self.motor_last = self.motor_current
            self.log(f"[Controller] Home Position Calibration Finish")
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

    # -------------------------------------------------------
    #   HOME / STOP
    # -------------------------------------------------------
    def send_home(self):
        if not self.msi:
            self.log("[GUI] Not connected.")
            return
        # Just do home here. No tension part. 
        # TODO Done. But need to test this. 
        # Real home position. Need Poistion control
        # Simple example: Set torque to 0
        # self.msi.set_motor_torques([0, 0, 0, 0])

        # Read current motor status 
        ms_0, vel = self.msi.get_motor_states()
        self.motor_current = ms_0
        self.joint_current = self.controller.motor_to_joint(self.motor_current,
                                                            self.motor_last,
                                                            self.joint_last,
                                                            self.controller.trans_mat)
        # run home ctrl
        def run_home():
            # TODO change to go_home function.
            self.motor_current, self.joint_current = self.controller.torque_ctrl(self.joint_current, self.joint_current, self.joint_home)
            self.log(f"[Controller] Finished torque_control. Final joint state: {self.joint_current}")
            self.motor_last = self.motor_current
            self.joint_last = self.joint_current
        # Open a thread for this.
        thread = threading.Thread(target=run_home, daemon=True)
        thread.start()
        self.log(f"[GUI] Started position control. To {self.joint_home}...")

    def send_stop(self):
        if not self.msi:
            self.log("[GUI] Not connected.")
            return
        # Stop all motor. Set torque to 0
        # TODO Delay of the position reading after hard stop 
        self.msi.set_motor_torques([0, 0, 0, 0])
        ms_0, vel = self.msi.get_motor_states()
        self.motor_current = ms_0
        self.joint_current = self.controller.motor_to_joint(self.motor_current,
                                                            self.motor_last,
                                                            self.joint_last,
                                                            self.controller.trans_mat)
        self.motor_last = self.motor_current
        self.joint_last = self.joint_current
        self.log("[GUI] STOP -> set torques to 0.")

    # -------------------------------------------------------
    #   Send joint position command ("POS j1 j2")
    #   (Slider & text entry share DoubleVar -> only one Send button needed)
    # -------------------------------------------------------
    def send_position_cmd(self):
        if not self.msi:
            self.log("[GUI] Not connected.")
            return
        # Get the desired joints position.
        j1_desired = self.j1_var.get()
        j2_desired = self.j2_var.get()
        
        js_d = math.radians([j2_desired, j1_desired])
        
        # read the current position and velocities
        position, velocities = self.msi.get_motor_states()
        if len(position) != 4:
            self.log(f"[GUI] Motor states error.")
            return 
        # current motor position.
        self.motor_current = position 
        self.joint_current = self.controller.motor_to_joint(self.motor_current,
                                                            self.motor_last,
                                                            self.joint_last,
                                                            self.controller.trans_mat)

        # TODO Test the poistion control. 
        # Sent desire position commend to the controller.
        # Call a contoller function:
        #   Convert the desire joint position to motor torque.
        # Example to sent the desired torque. 
        # self.msi.set_motor_torques([t0, t1, t2, t3])
        try:
            self.log(f"[GUI] Sent Position: j1={j1_desired}, j2={j2_desired}")
            def run_position_ctrl():
                self.motor_current, self.joint_current = self.controller.torque_ctrl(self.joint_current, self.motor_current, js_d)
                self.log(f"[Controller] Finished torque_control. Final joint state: {self.joint_current}")
                self.motor_last = self.motor_current
                self.joint_last = self.joint_current
            thread = threading.Thread(target=run_position_ctrl, daemon=True)
            thread.start()
            self.log(f"[GUI] Started position control. To {js_d}...")
        except Exception as e:
            self.log(f"[GUI] Error sending position cmd: {e}")

    # -------------------------------------------------------
    #   Send 4-channel motor torque (MotorSerialInterface binary protocol)
    #   + Automatically reset to zero after specified duration
    # -------------------------------------------------------
    def send_torque_cmd(self):
        if not self.msi:
            self.log("[GUI] Not connected.")
            return
        try:
            t0 = float(self.torque0_entry.get())
            t1 = float(self.torque1_entry.get())
            t2 = float(self.torque2_entry.get())
            t3 = float(self.torque3_entry.get())
        except ValueError:
            self.log("[GUI] Invalid torque input (must be float).")
            return

        # The time duration is only for testing
        # Not sure we want to keep this when control the finger.
        # Read duration (seconds), default is 5
        try:
            duration_s = float(self.torque_duration_entry.get())
        except ValueError:
            duration_s = 5.0

        # First, set the torque
        try:
            self.msi.set_motor_torques([t0, t1, t2, t3])
            self.log(f"[GUI] set_motor_torques -> [{t0}, {t1}, {t2}, {t3}] for {duration_s} s")
        except Exception as e:
            self.log(f"[GUI] Error calling set_motor_torques: {e}")
            return

        # Start a timer to reset torque to zero after duration_s seconds
        ms_delay = int(duration_s * 1000)
        self.master.after(ms_delay, self.reset_torque)
    
    def reset_torque(self):
        """Timer callback: Reset motor torque to 0"""
        if self.msi:
            self.msi.set_motor_torques([0, 0, 0, 0])
            self.log("[GUI] torque reset to 0 after duration.")

    # -------------------------------------------------------
    #   Periodic refresh: Read motor states -> Draw & display in top-right corner
    # -------------------------------------------------------
    def update_gui(self):
        if self.msi:
            positions, velocities = self.msi.get_motor_states()
            joint_current = self.controller.motor_to_joint(positions,
                                                            self.motor_last,
                                                            self.joint_last,
                                                            self.controller.trans_mat)
            j1_actual = joint_current[1]
            j2_actual = joint_current[0]
            self.draw_finger(j1_actual, j2_actual)
        self.master.after(50, self.update_gui)

    def draw_finger(self, j1, j2):
        # TODO The real configuration of the finger 
        # TODO visualize the URDF yourdfpy or urdfpy pacakge.
        self.canvas.delete("all")

        # Link lengths & base position
        L1, L2 = 80, 60
        x0, y0 = 150, 200

        r1 = j1
        r2 = j2

        x1 = x0 + L1 * math.cos(r1)
        y1 = y0 - L1 * math.sin(r1)

        x2 = x1 + L2 * math.cos(r1 + r2)
        y2 = y1 - L2 * math.sin(r1 + r2)

        # Draw two links
        self.canvas.create_line(x0, y0, x1, y1, fill="red", width=5)
        self.canvas.create_line(x1, y1, x2, y2, fill="blue", width=5)

        # Joint points
        self.canvas.create_oval(x1-5, y1-5, x1+5, y1+5, fill="green")
        self.canvas.create_oval(x2-5, y2-5, x2+5, y2+5, fill="green")

        # Display current joint angles in top-right corner
        j1_deg = math.degrees(j1)
        j2_deg = math.degrees(j2)
        self.canvas.create_text(
            290, 10,
            text=f"Joint1={j1_deg:.1f}°\nJoint2={j2_deg:.1f}°",
            fill="black",
            anchor='ne',
            font=("Arial", 16, "bold")
        )

    # -------------------------------------------------------
    #   Logging & Exit
    # -------------------------------------------------------
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