import numpy as np
import math
import time
import matplotlib.pyplot as plt 

from serial_com import MotorSerialInterface

class Controller:
    def __init__(self):
        self.MSI = MotorSerialInterface(port='/dev/ttyACM0', baud_rate=115200, timeout=1)
        self.kp = 0.15
        self.ki = 0.0
        self.kd = 0.0
        self._r = 0.1
        self._R1 = 0.5
        self._R2 = 0.7
        self.trans_mat = np.array([[self._r/self._R1, -self._r/self._R1, 0.0, 0.0],
                                   [-self._r/self._R2, self._r/self._R2, self._r/self._R2, -self._r/self._R2]])
        
    def go_home(self, tensioned):
        # Moving to home position
        motor_states = self.MSI.get_motor_states()[0]
        if tensioned == True:
            motor_torques = np.array([0.68, -0.68, -0.68, 0.68])
            velocities = self.MSI.get_motor_states()[1]
            self.MSI.set_motor_torques(motor_torques)   
            while np.any(np.abs(velocities) > 0.1):
                for i in range(4):
                    if velocities[i] > 0.1:
                        motor_torques[i] = -1**(i+1)*0.68
                    else:
                        motor_torques[i] = 0.0
                self.MSI.set_motor_torques(motor_torques)
                velocities = self.MSI.get_motor_states()[1]
            motor_states = self.MSI.get_motor_states()[0]
        else:
            print("Tendons are not tensioned, please tension first!")
        return motor_states  
    
    def tension(self):
        motor_torques = np.array([0.34, 0.34, 0.34, 0.34])
        velocities = self.MSI.get_motor_states()[1]
        self.MSI.set_motor_torques(motor_torques)
        while np.any(np.abs(velocities) > 0.1):
            for i in range(4):
                if velocities[i] > 0.1:
                    motor_torques[i] = 0.34
                else:
                    motor_torques[i] = 0.0
            self.MSI.set_motor_torques(motor_torques)
            velocities = self.MSI.get_motor_states()[1]
        motor_states = self.MSI.get_motor_states()[0]
        tensioned = True
        return motor_states, tensioned

    def torque_ctrl(self, js_0, ms_0, js_d):
        # js - joint_states
        # ms - motor_states
        # js_d - desired joint_states
        prev_time = time.time()
        js_0 = np.array(math.radians(js_0))
        js = np.array(js_0)
        ms_0 = np.array(ms_0) 
        ms = np.array(ms_0)
        js_d = np.array(math.radians(js_d))
        js_e = js_d - js
        js_e_sum = 0.0 + js_e
        # Lists to store data for plotting
        time_list = []
        error_list = []
        while np.any(np.abs(js_e) > 0.1 * np.abs(js_d)):
            current_time = time.time()
            dt = current_time - prev_time
            joint_torques = self.kp*js_e + self.ki*js_e_sum + self.kd*(js_e - js)*dt
            motor_torques = self.trans_mat.conjugate().transpose() @ joint_torques
            self.MSI.set_motor_torques(motor_torques)
            print(f"Motor torque: {motor_torques[0]}") 
            pos = self.MSI.get_motor_states()[0]
            ms = np.array(pos)
            js = self.trans_mat @ (ms.T - ms_0.T) + js_0
            js_e = js_d - js
            js_e_sum += js_e
            prev_time = current_time
            print(f"Error norm: {np.linalg.norm(js_e)}") 
            # Log data
            time_list.append(current_time)
            error_list.append(np.linalg.norm(js_e))  # Store norm of error for easy plotting

        # Plot results
        plt.figure(figsize=(10, 5))

        # Plot Joint Error
        plt.subplot(2, 1, 1)
        plt.plot(time_list, error_list, label='Joint Error')
        plt.xlabel('Time (s)')
        plt.ylabel('Error')
        plt.title('Joint Error Over Time')
        plt.legend()
        plt.grid()

        plt.tight_layout()
        plt.show()
        return ms, math.degrees(js)

if __name__ == '__main__':
    msi = MotorSerialInterface(port='/dev/ttyACM0', baud_rate=115200)
    controller = Controller()
    ms_0, velocities = msi.get_motor_states()
    js_0 = controller.trans_mat @ ms_0
    js_d = np.array([math.pi/2, math.pi/4])
    ms, js = controller.torque_ctrl(js_0, ms_0, js_d)
    msi.close()