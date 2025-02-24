/**
 * @file main.cpp
 * @brief Main entry point for the motor control and serial feedback system.
 *
 * This file sets up CAN communication, motor and serial protocols, and implements the main control loop.
 * It integrates functionalities from various modules such as motor protocols (R1806), serial protocols,
 * finger kinematics, and a Controller for torque control.
 */

 #include <FlexCAN_T4.h>
 #include <math.h>
 #include "motor_protocols/r1806.h"
 #include "motor_protocols/utils.h"
 #include "serial_protocols/serial_protocols.h"
 #include "serial_protocols/utils.h"
 #include "finger/finger.h"
 #include "controller/controller.h"
 
 // CAN and protocol objects
 FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;  ///< CAN bus object.
 CAN_message_t msg;                              ///< Temporary CAN message storage.
 R1806 r1806_protocols;                          ///< R1806 motor protocol handler.
 SerialProtocols serial_protocols;               ///< Serial protocol handler.
 Finger finger;                                  ///< Finger kinematics handler.
 Controller controller;                          ///< Controller for torque control.
 
 // Global state variables
 float* joint_states;                            ///< Pointer to the current joint states.
 float motor_offsets[4] = {0.0, 0.0, 0.0, 0.0};   ///< Offsets for motor positions.
 float print[2] = {0.0, 0.0};                      ///< Array used for serial feedback (positions).
 float* joint_commands;                          ///< Pointer to decoded joint command values.
 float motor_position_states[4] = {0.0, 0.0, 0.0, 0.0};  ///< Current motor position states.
 float motor_velocity_states[4] = {0.0, 0.0, 0.0, 0.0};  ///< Current motor velocity states.
 float motor_position_offsets[4] = {0.0, 0.0, 0.0, 0.0}; ///< Calibration offsets for motor positions.
 float joint_targets[2] = {0.0, 0.0};              ///< Desired joint targets.
 float static_torque = 0.065;                      ///< Static torque used during calibration.
 float velocity_limit = 50.0;                      ///< Velocity limit for safety.
 bool calibrated = false;                          ///< Flag indicating whether calibration is complete.
 std::vector<double> motor_states = {0.0, 0.0, 0.0, 0.0};       ///< Current motor positions (std::vector format).
 std::vector<double> motor_torques = {0.0, 0.0, 0.0, 0.0};      ///< Computed motor torques.
 std::vector<double> joint_error_sum = {0.0, 0.0};              ///< Accumulated joint error sum.
 std::vector<double> joint_states_desired = {0.0, 0.0};           ///< Desired joint states.
 std::vector<double> motor_velocities = {0.0, 0.0, 0.0, 0.0};     ///< Current motor velocities.
 uint8_t* packet;                                  ///< Pointer to a serial packet.
 static uint32_t timeout = millis();               ///< Timeout marker for CAN message sending.
 int danger_mode = false;                          ///< Flag indicating if danger mode is active.
 
 // Joint command reception flags and variables.
 bool received_joint_commands = false;             ///< Flag indicating if valid joint commands were received.
 float j1_input = 0.0;                             ///< Raw input for joint 1.
 float j2_input = 0.0;                             ///< Raw input for joint 2.
 
 // CAN messages for torque commands and closed-loop control.
 // These messages are initialized using the R1806 protocol functions.
 CAN_message_t motor1_torque_cmd = r1806_protocols.encodeTorqueCommand(1, 0.0);
 CAN_message_t motor1_close_loop_request_cmd = r1806_protocols.encodeRequestedStateCommand(1, ODrive.Axis.AxisState.CLOSED_LOOP_CONTROL);
 
 CAN_message_t motor2_torque_cmd = r1806_protocols.encodeTorqueCommand(1, 0.0);
 CAN_message_t motor2_close_loop_request_cmd = r1806_protocols.encodeRequestedStateCommand(2, ODrive.Axis.AxisState.CLOSED_LOOP_CONTROL);
 
 CAN_message_t motor3_torque_cmd = r1806_protocols.encodeTorqueCommand(1, 0.0);
 CAN_message_t motor3_close_loop_request_cmd = r1806_protocols.encodeRequestedStateCommand(3, ODrive.Axis.AxisState.CLOSED_LOOP_CONTROL);
 
 CAN_message_t motor4_torque_cmd = r1806_protocols.encodeTorqueCommand(1, 0.0);
 CAN_message_t motor4_close_loop_request_cmd = r1806_protocols.encodeRequestedStateCommand(4, ODrive.Axis.AxisState.CLOSED_LOOP_CONTROL);
 
 // Timeout parameters: if no valid torque command is received within 100 ms, set torques to 0.
 const uint32_t POSITION_TIMEOUT_MS = 100;         ///< Maximum allowed delay for receiving joint commands.
 uint32_t lastpositionCmdTime = millis();            ///< Timestamp of the last valid joint command received.
 
 /**
  * @brief Callback function for received CAN messages.
  *
  * This function processes incoming CAN messages by extracting the motor ID and command ID,
  * updating motor position and velocity states, and applying calibration offsets when needed.
  *
  * @param msg The received CAN message.
  */
 void canSniff(const CAN_message_t &msg) {
     // Extract motor and command IDs.
     uint32_t motor_id = r1806_protocols.extract_node_id(msg.id);
     uint32_t cmd_id = r1806_protocols.extract_command_id(msg.id);
 
     // If not calibrated, directly update motor states.
     if (calibrated == false) {
         if (motor_id == 1 && cmd_id == 9) {
             motor_position_states[0] = r1806_protocols.decodePositionFeedback(msg);
             motor_states[0] = motor_position_states[0];
             motor_velocity_states[0] = r1806_protocols.decodeVelocityFeedback(msg);
             motor_velocities[0] = motor_velocity_states[0];
         } else if (motor_id == 2 && cmd_id == 9) {
             motor_position_states[1] = r1806_protocols.decodePositionFeedback(msg);
             motor_states[1] = motor_position_states[1];
             motor_velocity_states[1] = r1806_protocols.decodeVelocityFeedback(msg);
             motor_velocities[1] = motor_velocity_states[1];
         } else if (motor_id == 3 && cmd_id == 9) {
             motor_position_states[2] = r1806_protocols.decodePositionFeedback(msg);
             motor_states[2] = motor_position_states[2];
             motor_velocity_states[2] = r1806_protocols.decodeVelocityFeedback(msg);
             motor_velocities[2] = motor_velocity_states[2];
         } else if (motor_id == 4 && cmd_id == 9) {
             motor_position_states[3] = r1806_protocols.decodePositionFeedback(msg);
             motor_states[3] = motor_position_states[3];
             motor_velocity_states[3] = r1806_protocols.decodeVelocityFeedback(msg);
             motor_velocities[3] = motor_velocity_states[3];
         }
     } else {
         // If calibrated, apply position offsets.
         if (motor_id == 1 && cmd_id == 9) {
             float raw_motor1_position = r1806_protocols.decodePositionFeedback(msg);
             motor_position_states[0] = raw_motor1_position - motor_position_offsets[0];
             motor_states[0] = motor_position_states[0];
             motor_velocity_states[0] = r1806_protocols.decodeVelocityFeedback(msg);
             motor_velocities[0] = motor_velocity_states[0];
         } else if (motor_id == 2 && cmd_id == 9) {
             float raw_motor2_position = r1806_protocols.decodePositionFeedback(msg);
             motor_position_states[1] = raw_motor2_position - motor_position_offsets[1];
             motor_states[1] = motor_position_states[1];
             motor_velocity_states[1] = r1806_protocols.decodeVelocityFeedback(msg);
             motor_velocities[1] = motor_velocity_states[1];
         } else if (motor_id == 3 && cmd_id == 9) {
             float raw_motor3_position = r1806_protocols.decodePositionFeedback(msg);
             motor_position_states[2] = raw_motor3_position - motor_position_offsets[2];
             motor_states[2] = motor_position_states[2];
             motor_velocity_states[2] = r1806_protocols.decodeVelocityFeedback(msg);
             motor_velocities[2] = motor_velocity_states[2];
         } else if (motor_id == 4 && cmd_id == 9) {
             float raw_motor4_position = r1806_protocols.decodePositionFeedback(msg);
             motor_position_states[3] = raw_motor4_position - motor_position_offsets[3];
             motor_states[3] = motor_position_states[3];
             motor_velocity_states[3] = r1806_protocols.decodeVelocityFeedback(msg);
             motor_velocities[3] = motor_velocity_states[3];
         }
     }
 }
 
 /**
  * @brief Calibrates motor position offsets.
  *
  * Sends a static torque command for a short duration to settle the system, then records the current
  * motor positions as offsets. After calibration, the system adjusts subsequent motor position readings.
  */
 void calibrate() {
     // Send static torque commands for calibration.
     motor1_torque_cmd = r1806_protocols.encodeTorqueCommand(1, -static_torque);
     motor2_torque_cmd = r1806_protocols.encodeTorqueCommand(2, -static_torque);
     motor3_torque_cmd = r1806_protocols.encodeTorqueCommand(3, -static_torque);
     motor4_torque_cmd = r1806_protocols.encodeTorqueCommand(4, -static_torque);
 
     int c = 0;
     // Send torque commands at roughly 50 Hz for a fixed period.
     while (c < 150) {
         if (millis() - timeout > 20) {
             can1.events();
             can1.write(motor1_torque_cmd);
             can1.write(motor2_torque_cmd);
             can1.write(motor3_torque_cmd);
             can1.write(motor4_torque_cmd);
             timeout = millis();
             c += 1;
         }
     }
     // Set current motor positions as calibration offsets.
     for (int i = 0; i < 4; i++) {
         motor_position_offsets[i] = motor_position_states[i];
     }
     calibrated = true;
 }
 
 /**
  * @brief Initializes the system.
  *
  * Sets up Serial communication, configures the CAN bus (baud rate, RX/TX pins, FIFO, etc.),
  * registers the CAN receive callback, sends closed-loop control requests, and initiates calibration.
  */
 void setup(void) {
     Serial.begin(115200);
     delay(500);
     can1.begin();
     can1.setBaudRate(250000);
     can1.setRX(ALT);          // CRX for CAN1 is actually Pin 13 for the ME472 Dev Board V3
     can1.setTX(ALT);
     can1.setMaxMB(16);
     can1.enableFIFO();
     can1.enableFIFOInterrupt();
     can1.onReceive(canSniff);
     can1.mailboxStatus();
     delay(500);
 
     can1.events();
     can1.write(motor1_close_loop_request_cmd);
     can1.write(motor2_close_loop_request_cmd);
     can1.write(motor3_close_loop_request_cmd);
     can1.write(motor4_close_loop_request_cmd);
     delay(500);
 
     calibrate();
     // Uncomment and modify the following if joint_states initialization is required.
     // joint_states = {0, 0};
     delay(500);
 }
 
 /**
  * @brief Main control loop.
  *
  * Processes incoming CAN messages and serial commands, performs torque control using a PID controller,
  * updates joint states, checks safety conditions (e.g., velocity limits), and sends feedback packets over Serial.
  */
 void loop() {
     can1.events();
 
     if (danger_mode == false) {
         // --- Joint Position Command Reception ---
         if (Serial.available() >= JOINT_COMMAND_PACKET_SIZE) {
             uint8_t joint_command_buffer[JOINT_COMMAND_PACKET_SIZE];
             // Read incoming serial joint command packet.
             Serial.readBytes((char*)joint_command_buffer, JOINT_COMMAND_PACKET_SIZE);
             joint_commands = serial_protocols.decodeJointCommandPacket(joint_command_buffer);
             if (joint_commands != nullptr) {
                 received_joint_commands = true;
                 // Update the last valid command timestamp.
                 lastpositionCmdTime = millis();
             }
         }
         
         // --- Timeout Check: Reset torques to 0 if no command received ---
         if (millis() - lastpositionCmdTime > POSITION_TIMEOUT_MS) {
             motor1_torque_cmd = r1806_protocols.encodeTorqueCommand(1, 0.0);
             motor2_torque_cmd = r1806_protocols.encodeTorqueCommand(2, 0.0);
             motor3_torque_cmd = r1806_protocols.encodeTorqueCommand(3, 0.0);
             motor4_torque_cmd = r1806_protocols.encodeTorqueCommand(4, 0.0);
         }
     
         // Override torque command for testing purposes.
         motor1_torque_cmd = r1806_protocols.encodeTorqueCommand(1, -static_torque);
         motor2_torque_cmd = r1806_protocols.encodeTorqueCommand(2, -static_torque);
         motor3_torque_cmd = r1806_protocols.encodeTorqueCommand(3, -static_torque);
         motor4_torque_cmd = r1806_protocols.encodeTorqueCommand(4, -static_torque);
     
         // --- Update Joint States Feedback Data ---
         joint_states = finger.getJointStates(motor_position_states);
         
         if (received_joint_commands == true) {
             // Convert joint states to std::vector format for the controller.
             std::vector<double> js = {joint_states[0], joint_states[1]};
             std::vector<double> js_d = {joint_states_desired[0], joint_states_desired[1]};
             // Update desired joint states from received commands.
             js_d[0] = joint_commands[0];
             js_d[1] = joint_commands[1];
             // Compute torque control using PID controller.
             std::pair<std::vector<double>, std::vector<double>> result = 
                 controller.torque_control(js, js_d, joint_error_sum, motor_velocities);
             motor_torques = result.first;
             joint_error_sum = result.second;
             // Determine maximum torque value.
            //  double max = motor_torques[0];
            //  for (int i = 0; i < 4; i++) {
            //      if (motor_torques[i] >= max) {
            //          max = motor_torques[i];
            //      }
            //  }
             // Adjust individual motor torque commands based on computed values.
             motor1_torque_cmd = r1806_protocols.encodeTorqueCommand(1, - static_torque + motor_torques[0]);
             motor2_torque_cmd = r1806_protocols.encodeTorqueCommand(2, - static_torque + motor_torques[1]);
             motor3_torque_cmd = r1806_protocols.encodeTorqueCommand(3, - static_torque + motor_torques[2]);
             motor4_torque_cmd = r1806_protocols.encodeTorqueCommand(4, - static_torque + motor_torques[3]);
         }
     
         // --- Check for Danger Mode: Disable commands if velocity exceeds limits ---
         for (int i = 0; i < NUM_MOTORS; i++) {
             if (motor_velocity_states[i] > velocity_limit || motor_velocity_states[i] < -velocity_limit) {
                 danger_mode = true;
             }
         }
     } else {
         // In danger mode, reset all torque commands to zero.
         motor1_torque_cmd = r1806_protocols.encodeTorqueCommand(1, 0.0);
         motor2_torque_cmd = r1806_protocols.encodeTorqueCommand(2, 0.0);
         motor3_torque_cmd = r1806_protocols.encodeTorqueCommand(3, 0.0);
         motor4_torque_cmd = r1806_protocols.encodeTorqueCommand(4, 0.0);
     }
 
     // --- Periodically send CAN messages with torque commands ---
     if (millis() - timeout > 20) {
         can1.write(motor1_torque_cmd);
         can1.write(motor2_torque_cmd);
         can1.write(motor3_torque_cmd);
         can1.write(motor4_torque_cmd);
         timeout = millis();
     }
 
     // --- Update joint states feedback and prepare serial feedback data ---
     joint_states = finger.getJointStates(motor_position_states);
    //  print[0] = -static_torque + motor_torques[0];
    //  print[1] = -static_torque + motor_torques[1];
 
     // --- Encode and send feedback packet over Serial ---
     packet = serial_protocols.encodeFeedbackPacket(joint_states);
     Serial.write(packet, FEEDBACK_PACKET_SIZE);
 }
 