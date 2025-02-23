# Team 2n Testbench 
Finger design, fabrication and control of ```2n``` configured finger with tendon-driven actuators. This repository should contain all of the software required for the project 

[InsertImages]

## Hardware 
- 2 DOF finger
- 4 ODrive Pro
- 4 BLDC Motors 
- 4 AMS SPI Encoders 
- Emergency Stop
- 24V to 12V custom power board
- Connecting wires, connectors and crimps

## Repository Structure
- ```include```: Includes header files for all classes used
- ```UI```: Python files used for user interface and serial interfacing
- ```lib```: Contains any external libraries used
- ```test```: Containts any tests utilized
- ```src```: Source code used to operate electromechanical system
    - ```motor_protocols```: Motor state interfacing
    - ```serial_protocols```: Serial-UI interfacing
    - ```finger ```: Finger kinematics
    - ```controller```: Torque controller used

## Installation
### General
Clone repository locally and configure all paths included within vscode. 

Download and install PlatformIO to operete Teensy (https://platformio.org/)

### ODrive Firmware
Download odrivetool and set up can using following commands or other OS equivalent: 
```bash
sudo ip link set can0 up type can bitrate 250000
```
Configure following parameters for each odrive in odrivetool:
```bash
odrv0.config.dc_max_negative_current = -0.1
odrv0.config.max_regen_current = 0.1
odrv0.save_configuration()
```
Set appropriate Can_ID through odrive GUI online

### User Interface
For user interface, only python is required. Navigate into ```UI``` sub-directory and run the following command: 
```bash
python finger_GUI.py
```
Connect serially through the user interface

### Operation
Build and upload ```src/main.cpp``` onto Teensy for full operation