#include <FlexCAN_T4.h>
#include <math.h>
#include "motor_protocols/r1806.h"
#include "motor_protocols/utils.h"
#include "serial_protocols/serial_protocols.h"
#include "serial_protocols/utils.h"
#include "finger/finger.h"
#include "controller/controller.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t msg;
R1806 r1806_protocols;
SerialProtocols serial_protocols;
Finger finger;
Controller controller;

float* joint_states;
float motor_offsets[4] = {0.0, 0.0, 0.0, 0.0};
float print[2] = {0.0, 0.0};
float* joint_commands;
float motor_position_states[4] = {0.0, 0.0, 0.0, 0.0};
float motor_velocity_states[4] = {0.0, 0.0, 0.0, 0.0};
float motor_position_offsets[4] = {0.0, 0.0, 0.0, 0.0};
float joint_targets[2] = {0.0, 0.0};
float static_torque = 0.065;
float velocity_limit = 24.0;
bool calibrated = false;
std::vector<double> motor_states = {0.0, 0.0, 0.0, 0.0}; // position
std::vector<double> motor_torques = {0.0, 0.0, 0.0, 0.0};
std::vector<double> joint_error_sum = {0.0, 0.0};
std::vector<double> joint_states_desired = {0.0, 0.0};
std::vector<double> motor_velocities = {0.0, 0.0, 0.0, 0.0};
uint8_t* packet;
static uint32_t timeout = millis();
int danger_mode = false;

bool received_joint_commands = false;
float j1_input = 0.0;
float j2_input = 0.0;

// CAN messages for torque commands and close loop control.
CAN_message_t motor1_torque_cmd = r1806_protocols.encodeTorqueCommand(1, 0.0);
CAN_message_t motor1_close_loop_request_cmd = r1806_protocols.encodeRequestedStateCommand(1, ODrive.Axis.AxisState.CLOSED_LOOP_CONTROL);

CAN_message_t motor2_torque_cmd = r1806_protocols.encodeTorqueCommand(1, 0.0);
CAN_message_t motor2_close_loop_request_cmd = r1806_protocols.encodeRequestedStateCommand(2, ODrive.Axis.AxisState.CLOSED_LOOP_CONTROL);

CAN_message_t motor3_torque_cmd = r1806_protocols.encodeTorqueCommand(1, 0.0);
CAN_message_t motor3_close_loop_request_cmd = r1806_protocols.encodeRequestedStateCommand(3, ODrive.Axis.AxisState.CLOSED_LOOP_CONTROL);

CAN_message_t motor4_torque_cmd = r1806_protocols.encodeTorqueCommand(1, 0.0);
CAN_message_t motor4_close_loop_request_cmd = r1806_protocols.encodeRequestedStateCommand(4, ODrive.Axis.AxisState.CLOSED_LOOP_CONTROL);

// Timeout parameters: if no valid torque command is received within 100 ms, set torques to 0.
const uint32_t POSITION_TIMEOUT_MS = 100;
uint32_t lastpositionCmdTime = millis();

void canSniff(const CAN_message_t &msg) {
    // Extract motor id and command id from the CAN message
    uint32_t motor_id = r1806_protocols.extract_node_id(msg.id);
    uint32_t cmd_id = r1806_protocols.extract_command_id(msg.id);

    if(calibrated == false) {
        if(motor_id == 1) {
            if(cmd_id == 9) {
                motor_position_states[0] = r1806_protocols.decodePositionFeedback(msg);
                motor_states[0] = motor_position_states[0];
                motor_velocity_states[0] = r1806_protocols.decodeVelocityFeedback(msg);
                motor_velocities[0] = motor_velocity_states[0];
            }
        }
        else if(motor_id == 2) {
            if(cmd_id == 9) {
                motor_position_states[1] = r1806_protocols.decodePositionFeedback(msg);
                motor_states[1] = motor_position_states[1];
                motor_velocity_states[1] = r1806_protocols.decodeVelocityFeedback(msg);
                motor_velocities[1] = motor_velocity_states[1];
            }
        }
        else if(motor_id == 3) {
            if(cmd_id == 9) {
                motor_position_states[2] = r1806_protocols.decodePositionFeedback(msg);
                motor_states[2] = motor_position_states[2];
                motor_velocity_states[2] = r1806_protocols.decodeVelocityFeedback(msg);
                motor_velocities[2] = motor_velocity_states[2];
            }
        }
        else if(motor_id == 4) {
            if(cmd_id == 9) {
                motor_position_states[3] = r1806_protocols.decodePositionFeedback(msg);
                motor_states[3] = motor_position_states[3];
                motor_velocity_states[3] = r1806_protocols.decodeVelocityFeedback(msg);
                motor_velocities[3] = motor_velocity_states[3];
            }
        }
    }
    else{
        if(motor_id == 1) {
            if(cmd_id == 9) {
                float raw_motor1_position = r1806_protocols.decodePositionFeedback(msg);
                motor_position_states[0] = raw_motor1_position - motor_position_offsets[0];
                motor_states[0] = motor_position_states[0];
                motor_velocity_states[0] = r1806_protocols.decodeVelocityFeedback(msg);
                motor_velocities[0] = motor_velocity_states[0];
            }
        }
        else if(motor_id == 2) {
            if(cmd_id == 9) {
                float raw_motor2_position = r1806_protocols.decodePositionFeedback(msg);
                motor_position_states[1] = raw_motor2_position - motor_position_offsets[1];
                motor_states[1] = motor_position_states[1];
                motor_velocity_states[1] = r1806_protocols.decodeVelocityFeedback(msg);
                motor_velocities[1] = motor_velocity_states[1];
            }
        }
        else if(motor_id == 3) {
            if(cmd_id == 9) {
                float raw_motor3_position = r1806_protocols.decodePositionFeedback(msg);
                motor_position_states[2] = raw_motor3_position - motor_position_offsets[2];
                motor_states[2] = motor_position_states[2];
                motor_velocity_states[2] = r1806_protocols.decodeVelocityFeedback(msg);
                motor_velocities[2] = motor_velocity_states[2];
            }
        }
        else if(motor_id == 4) {
            if(cmd_id == 9) {
                float raw_motor4_position = r1806_protocols.decodePositionFeedback(msg);
                motor_position_states[3] = raw_motor4_position - motor_position_offsets[3];
                motor_states[3] = motor_position_states[3];
                motor_velocity_states[3] = r1806_protocols.decodeVelocityFeedback(msg);
                motor_velocities[3] = motor_velocity_states[3];
            }
        }
    }

}

void calibrate() {
    motor1_torque_cmd = r1806_protocols.encodeTorqueCommand(1, -static_torque);
    motor2_torque_cmd = r1806_protocols.encodeTorqueCommand(2, -static_torque);
    motor3_torque_cmd = r1806_protocols.encodeTorqueCommand(3, -static_torque);
    motor4_torque_cmd = r1806_protocols.encodeTorqueCommand(4, -static_torque);
    int c = 0;
    while (c < 150){
        // --- Send Torque Commands at 50 Hz ---
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
    for (int i = 0; i < 4; i++) {
        motor_position_offsets[i] = motor_position_states[i];
    }
    calibrated = true;
}

void setup(void) {
    Serial.begin(115200);
    delay(500);
    can1.begin();
    can1.setBaudRate(250000);
    can1.setRX(ALT); // CRX for CAN1 is actually Pin 13 for the ME472 Dev Board V3
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
    //joint_states = {0, 0};
    delay(500);
}
//
//void SerialEvent(){
//    // ---- Moved Serial Read to SerialEvent ----
//    // Read desired position from serial 
//    if (Serial.available() >= JOINT_COMMAND_PACKET_SIZE) {
//        uint8_t joint_command_buffer[JOINT_COMMAND_PACKET_SIZE];
//        // Cast is required because readBytes expects a char pointer.
//        Serial.readBytes((char*)joint_command_buffer, JOINT_COMMAND_PACKET_SIZE);
//        joint_commands = serial_protocols.decodeJointCommandPacket(joint_command_buffer);
//        joint_states_desired = {{joint_commands[0]}, {joint_commands[1]}};
//        if (joint_commands != nullptr) {
//            received_joint_commands = true;
//            // Update the last valid torque command time.
//            lastpositionCmdTime = millis();
//        }
//    }
//    joint_error_sum = {0.0, 0.0};
//    Serial.print("Received Joint Commands");
//}
//
void loop() {
    can1.events();
    //Serial.print("CAN Events");
    if(danger_mode == false){
        // --- Joint Position Command Reception ---
        if (Serial.available() >= JOINT_COMMAND_PACKET_SIZE) {
            uint8_t joint_command_buffer[JOINT_COMMAND_PACKET_SIZE];
            // Cast is required because readBytes expects a char pointer.
            Serial.readBytes((char*)joint_command_buffer, JOINT_COMMAND_PACKET_SIZE);
            joint_commands = serial_protocols.decodeJointCommandPacket(joint_command_buffer);
            if (joint_commands != nullptr) {
                received_joint_commands = true;
                // Update the last valid torque command time.
                lastpositionCmdTime = millis();
            }
        }
        
        // --- Timeout Check: if no torque command received recently, reset torques to 0 ---
        if (millis() - lastpositionCmdTime > POSITION_TIMEOUT_MS) {
            motor1_torque_cmd = r1806_protocols.encodeTorqueCommand(1, 0.0);
            motor2_torque_cmd = r1806_protocols.encodeTorqueCommand(2, 0.0);
            motor3_torque_cmd = r1806_protocols.encodeTorqueCommand(3, 0.0);
            motor4_torque_cmd = r1806_protocols.encodeTorqueCommand(4, 0.0);
        }
    
        // Override Testing
        motor1_torque_cmd = r1806_protocols.encodeTorqueCommand(1, -static_torque);
        motor2_torque_cmd = r1806_protocols.encodeTorqueCommand(2, -static_torque);
        motor3_torque_cmd = r1806_protocols.encodeTorqueCommand(3, -static_torque);
        motor4_torque_cmd = r1806_protocols.encodeTorqueCommand(4, -static_torque);
    
        // --- Update Joint States Feedback Data ---
        joint_states = finger.getJointStates(motor_position_states);
        
        
        //Serial.print(received_joint_commands);
        if(received_joint_commands == true){
            //Serial.print(received_joint_commands);
            std::vector<double> js = {joint_states[0], joint_states[1]}; // Converting to std::vector
            std::vector<double> js_d = {joint_states_desired[0], joint_states_desired[1]};
            std::pair<std::vector<double>, std::vector<double>> result = (
                controller.torque_control(js , js_d, joint_error_sum, motor_velocities)
            );
            motor_torques = result.first;
            joint_error_sum = result.second;
            motor1_torque_cmd = r1806_protocols.encodeTorqueCommand(1, -static_torque-motor_torques[0]);
            motor2_torque_cmd = r1806_protocols.encodeTorqueCommand(2, -static_torque-motor_torques[1]);
            motor3_torque_cmd = r1806_protocols.encodeTorqueCommand(3, -static_torque-motor_torques[2]);
            motor4_torque_cmd = r1806_protocols.encodeTorqueCommand(4, -static_torque-motor_torques[3]);
            // print[0] = -static_torque-motor_torques[0];
            // print[1] = -static_torque-motor_torques[1];
            //Serial.print(motor_torques[2]);
        }
    
        for (int i=0; i<NUM_MOTORS; i++) {
            if(motor_velocity_states[i] > velocity_limit || motor_velocity_states[i] < -velocity_limit){ {
                danger_mode = true;
            }
            }
        }
    }
    else {
        motor1_torque_cmd = r1806_protocols.encodeTorqueCommand(1, 0.0);
        motor2_torque_cmd = r1806_protocols.encodeTorqueCommand(2, 0.0);
        motor3_torque_cmd = r1806_protocols.encodeTorqueCommand(3, 0.0);
        motor4_torque_cmd = r1806_protocols.encodeTorqueCommand(4, 0.0);
    }

    if (millis() - timeout > 20) {
        // can1.write(motor1_torque_cmd);
        // can1.write(motor2_torque_cmd);
        // can1.write(motor3_torque_cmd);
        // can1.write(motor4_torque_cmd);
        timeout = millis();
    }

    // --- Update Joint States Feedback Data ---
    joint_states = finger.getJointStates(motor_position_states);
    print[0] = joint_states[0];
    print[1] = joint_states[1];

    // --- Send Feedback Packet over Serial ---
    packet = serial_protocols.encodeFeedbackPacket(joint_states);
    Serial.write(packet, FEEDBACK_PACKET_SIZE);

}
