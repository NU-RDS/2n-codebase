import tkinter as tk

class MotorUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Motor Controller")

        # Torque Input
        self.torque_label = tk.Label(root, text="Torque Command:")
        self.torque_label.pack()

        self.torque_entry = tk.Entry(root)
        self.torque_entry.pack()

        # Run Button
        self.run_button = tk.Button(root, text="Run Motor", command=self.run_motor)
        self.run_button.pack()

        # Home Button
        self.home_button = tk.Button(root, text="Home Position", command=self.home_motor)
        self.home_button.pack()

        # Status Label
        self.status_label = tk.Label(root, text="Awaiting Command...", fg="blue")
        self.status_label.pack()

    def run_motor(self):
        torque_value = self.torque_entry.get()
        self.status_label.config(text=f"Running motor with torque: {torque_value}", fg="green")

    def home_motor(self):
        self.status_label.config(text="Returning motor to home position", fg="red")

if __name__ == "__main__":
    root = tk.Tk()
    app = MotorUI(root)
    root.mainloop()
