# Planar Two-Joint Finger GUI

This project provides a Python-based GUI for controlling a planar two-joint finger mechanism via a Teensy 4. The GUI sends desired joint positions to the Teensy and receives the actual joint positions via a custom serial communication interface. In addition, the GUI displays a schematic drawing of the finger and real-time plots showing the actual vs. desired joint angles.

## Features 
### Serial Communication: 
Uses a custom ``serial_com`` module that implements:
- `get_joint_states()` Returns two joint positions (in radians) from the Teensy.
- `set_joint_position()` Sends desired joint positions (in radians) to the Teensy.

### User Input 
Users can set the desired joint positions (in degrees) using sliders or text entries. The values are converted to radians before being sent.

### HOME and STOP Commands:
- **Home**: Sets the joint positions to the home state (default [0, 0] in radians).
- **STOP**: Holds the current joint positions by re-sending the current state.

### Real-Time Feedback and Visualization:
- A drawing canvas displays a schematic (two-link representation) of the finger based on the current joint positions.
- Two real-time plots (one for each joint) show both the actual joint angles and the desired joint angles over time.

### Layouts: 
- The top part of the window contains all control panels (buttons and joint position input).
- The bottom part is split into two columns: the left column is a log area, and the right column contains a small canvas on top (for the finger drawing) and the real-time plots below.

## Setup 
### Teensy4 Setup with Udev
To automatically recognize the Teensy 4 serial port, a udev rule is used. A sample setup script (``setup_udev.sh``) is provided below. Run the script as root to create the udev rule:

```sh
#!/bin/bash
# setup_teensy.sh
# This script creates a udev rule for the Teensy 4.1 device and reloads udev.

RULE_FILE="/etc/udev/rules.d/50-teensy.rules"
RULE='SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", MODE:="0666", SYMLINK+="teensy4"'

# Ensure the script is run as root
if [[ $EUID -ne 0 ]]; then
  echo "This script must be run as root. Try: sudo $0"
  exit 1
fi

echo "Creating udev rule at ${RULE_FILE}..."
echo "$RULE" > "$RULE_FILE"

echo "Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

echo "Udev rule installed and service reloaded successfully."

```
After running this script, your Teensy 4 device will be automatically recognized and available via the symlink `/dev/teensy4`.
### Python Dependencies 
Ensure you have the following dependencies installed:
- Python >=3.10 
- Tkinter
- NumPy 
- Matplotlib

Also, ensure your custom `serial_com.py` module is accessible in your PYTHONPATH.

## How it Works
1. **Connections**
    
    Click the **Connect** button to initialize serial communication with the Teensy 4 using the custom `MotorSerialInterface`.

2. **Sending Joint Positions**
    
    Adjust the sliders or enter values in the text boxes for Joint1 and Joint2 (in degrees) and click **Send Position**. The GUI converts these values to radians and calls `set_joint_positions()` to send the command to the Teensy.

3. **Real-Time Feedback**

    The GUI continuously reads the joint states using ``get_joint_states()``. It updates the finger drawing on the canvas and the real-time plots (displaying both actual and desired joint angles).

4. **HOME and STOP Commands**
    
    -  **HOME** Sends a command to set the joints to the home position ([0, 0] radians).

    - **STOP** Reads the current joint state and re-sends it, effectively holding the current position.

5. **Logging**

    All events (connections, commands, errors, etc.) are logged in the log area.


## Running the GUI 
To run the GUI application, execute:
```terminal
python Finger_GUI.py
```

## Customization 
- **Serial Port:**
    
    If needed, adjust the serial port in the `connect_serial()` function (default is `/dev/teensy4`). For Windows it might be `COM`.

- **Plot Appearance:**
    
    The real-time plotting area is built using Matplotlib. Modify the subplot settings in **update_plots()** to customize titles, labels, and styles. And you can also add other plots in this function.

- **Drawing:**
    
    The schematic drawing of the finger is a simple two-link model. You can adjust the link lengths and base position or integrate a URDF visualization if desired.