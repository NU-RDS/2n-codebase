import tkinter as tk
import math
import time

from serial_com import MotorSerialInterface
from pid import PIDController

class FingerGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Planar Two-Joint Finger")

        self.msi = None  # MotorSerialInterface 实例（在用户点击Connect后创建）

        # 1) 定义 DoubleVar，用于关节1、关节2：滑条 & 文本框共享
        self.j1_var = tk.DoubleVar(value=0.0)
        self.j2_var = tk.DoubleVar(value=0.0)

        self.control_mode = "init"  # 控制模式： init/ position / torque

        # Create PID controllers
        self.create_pid_controller()

        # 构建界面
        self.create_widgets()

        # 周期刷新 GUI（读取电机状态、更新画布等）
        self.update_gui()

    def create_pid_controller(self):
        # 创建 PID 控制器实例
        self.pid_controllers = [PIDController(p=0.5, i=0.0, d=5.0, window_size=10, max_input=0.04),
                                PIDController(p=0.5, i=0.0, d=5.0, window_size=10, max_input=0.04),
                                PIDController(p=0.5, i=0.0, d=5.0, window_size=10, max_input=0.04),
                                PIDController(p=0.5, i=0.0, d=5.0, window_size=10, max_input=0.04)]
        # 初始化 PID 控制器状态
        for pid in self.pid_controllers:
            pid.update_state(0.0, 0.0)
        
        self.j1_position_target = 0.0
        self.j2_position_target = 0.0
        self.j3_position_target = 0.0
        self.j4_position_target = 0.0

        self.j1_torque_target = 0.0
        self.j2_torque_target = 0.0
        self.j3_torque_target = 0.0
        self.j4_torque_target = 0.0

    def create_widgets(self):
        # -------------- 顶部按钮区 (Connect/Disconnect/Home/Stop) --------------
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

        # -------------- Joint Position 输入（滑条 + 文本框）--------------
        pos_frame = tk.LabelFrame(self.master, text="Set Joint Position")
        pos_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        tk.Label(pos_frame, text="Joint1 (deg):", font=("Arial",10)).grid(row=0, column=0, padx=5, pady=2)
        self.joint1_scale = tk.Scale(pos_frame, from_=-90, to=90,
                                     orient=tk.HORIZONTAL,
                                     variable=self.j1_var,
                                     length=200)  # 适当调整长度以便拖动
        self.joint1_scale.grid(row=0, column=1, padx=5, pady=2)
        self.joint1_entry = tk.Entry(pos_frame, width=6,
                                     textvariable=self.j1_var)  # 与 slider 共享变量
        self.joint1_entry.grid(row=0, column=2, padx=5, pady=2)

        tk.Label(pos_frame, text="Joint2 (deg):", font=("Arial",10)).grid(row=1, column=0, padx=5, pady=2)
        self.joint2_scale = tk.Scale(pos_frame, from_=-90, to=90,
                                     orient=tk.HORIZONTAL,
                                     variable=self.j2_var,
                                     length=200)
        self.joint2_scale.grid(row=1, column=1, padx=5, pady=2)
        self.joint2_entry = tk.Entry(pos_frame, width=6,
                                     textvariable=self.j2_var)
        self.joint2_entry.grid(row=1, column=2, padx=5, pady=2)

        send_pos_btn = tk.Button(pos_frame, text="Send Position", command=self.set_position_cmd)
        send_pos_btn.grid(row=2, column=0, columnspan=3, pady=5)

        # -------------- 4路电机力矩输入 --------------
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

        # >>>>>>>>>>> 新增 “持续时间” 输入 <<<<<<<<<<<
        tk.Label(torque_frame, text="Duration (s):").grid(row=2, column=0, padx=5, pady=2)
        self.torque_duration_entry = tk.Entry(torque_frame, width=6)
        self.torque_duration_entry.insert(0, "5")  # 默认持续5秒
        self.torque_duration_entry.grid(row=2, column=1, padx=5, pady=2)

        send_torque_btn = tk.Button(torque_frame, text="Send Torque", command=self.set_torque_cmd)
        send_torque_btn.grid(row=2, column=2, columnspan=2, pady=5)

        # -------------- 日志显示 --------------
        self.log_text = tk.Text(self.master, height=10, width=40)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # -------------- 画布 (画手指 + 右上角显示当前关节角度) --------------
        self.canvas = tk.Canvas(self.master, bg="white", width=300, height=300)
        self.canvas.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)

    # -------------------------------------------------------
    #   串口连接 / 断开
    # -------------------------------------------------------
    def connect_serial(self):
        if self.msi:
            self.log("[GUI] Already connected.")
            return
        try:
            self.msi = MotorSerialInterface(port="/dev/teensy4", baud_rate=115200, timeout=0.5)
            self.log("[GUI] Connected to /dev/teensy4.")
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
        # 简单示例：将力矩设为0
        self.msi.set_motor_torques([0, 0, 0, 0])
        self.log("[GUI] HOME -> set torques to 0.")

    def send_stop(self):
        if not self.msi:
            self.log("[GUI] Not connected.")
            return
        self.msi.set_motor_torques([0, 0, 0, 0])
        self.log("[GUI] STOP -> set torques to 0.")

    # -------------------------------------------------------
    #   发送关节位置命令 ("POS j1 j2")
    #   (滑条 & 文本框共用 DoubleVar -> 只需一个 Send 按钮)
    # -------------------------------------------------------
    def set_position_cmd(self):
        self.control_mode = "position"
        if not self.msi:
            self.log("[GUI] Not connected.")
            # return
        j1_deg = self.j1_var.get()
        j2_deg = self.j2_var.get()
        self.j1_position_target = math.radians(j1_deg)
        self.j2_position_target = math.radians(j2_deg)

    # -------------------------------------------------------
    #   发送 4 路电机力矩 (MotorSerialInterface 二进制协议)
    #   + 持续指定秒数后自动归零
    # -------------------------------------------------------
    def set_torque_cmd(self):
        self.control_mode = "torque"
        if not self.msi:
            self.log("[GUI] Not connected.")
            return
        self.j1_torque_target = float(self.torque0_entry.get())
        self.j2_torque_target = float(self.torque1_entry.get())
        self.j3_torque_target = float(self.torque2_entry.get())
        self.j4_torque_target = float(self.torque3_entry.get())

        # 读取持续时间(秒), 默认 5
        try:
            duration_s = float(self.torque_duration_entry.get())
        except ValueError:
            duration_s = 5.0

        # 启动一个定时器, 在duration_s秒后将力矩归零
        ms_delay = int(duration_s * 1000)
        self.master.after(ms_delay, self.reset_torque)

    def reset_torque(self):
        """定时回调: 将电机力矩设为0"""
        if self.msi:
            self.msi.set_motor_torques([0, 0, 0, 0])
            self.log("[GUI] torque reset to 0 after duration.")
    # -------------------------------------------------------
    #   周期刷新：读取电机状态 -> 画图 & 右上角显示
    # -------------------------------------------------------
    def update_gui(self):
        if self.msi:
            positions, velocities = self.msi.get_motor_states()
            j1_actual = positions[0]
            j2_actual = positions[1]
            j1_vel = velocities[0]
            j2_vel = velocities[1]
            self.draw_finger(j1_actual, j2_actual)
            self.pid_controllers[0].update_state(j1_actual, j1_vel)
            self.pid_controllers[1].update_state(j2_actual, j2_vel)
            if self.control_mode == "init":
                # 控制模式：初始化
                pass
            elif self.control_mode == "position":
                # 控制模式：位置控制
                j1_torque = self.pid_controllers[0].get_input(self.j1_position_target)
                j2_torque = self.pid_controllers[1].get_input(self.j2_position_target)
                self.msi.set_motor_torques([j1_torque, j2_torque, 0, 0])
                print([j1_torque, j2_torque, 0, 0])
            elif self.control_mode == "torque":
                # 控制模式：力矩控制
                self.msi.set_motor_torques([self.j1_torque_target, self.j2_torque_target, self.j3_torque_target, self.j4_torque_target])
            else:
                # Unknown control mode
                print("Unknown control mode")
        self.master.after(1, self.update_gui)

    def draw_finger(self, j1, j2):
        self.canvas.delete("all")

        # 连杆长 & 基座位置
        L1, L2 = 80, 60
        x0, y0 = 150, 200

        # r1 = math.radians(j1)
        # r2 = math.radians(j2)
        r1 = j1
        r2 = j2

        x1 = x0 + L1 * math.cos(r1)
        y1 = y0 - L1 * math.sin(r1)

        x2 = x1 + L2 * math.cos(r1 + r2)
        y2 = y1 - L2 * math.sin(r1 + r2)

        # 画两段连杆
        self.canvas.create_line(x0, y0, x1, y1, fill="red", width=5)
        self.canvas.create_line(x1, y1, x2, y2, fill="blue", width=5)

        # 关节点
        self.canvas.create_oval(x1-5, y1-5, x1+5, y1+5, fill="green")
        self.canvas.create_oval(x2-5, y2-5, x2+5, y2+5, fill="green")

        # 右上角文字显示当前硬件反馈到的关节角度
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
    #   日志 & 退出
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