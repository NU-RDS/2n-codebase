#include <FlexCAN_T4.h>
#include "motor_protocols/r1806.h"
#include "motor_protocols/utils.h"
#include "serial_protocols/serial_protocols.h"
#include "serial_protocols/utils.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t msg;
R1806 r1806_protocols;
SerialProtocols serial_protocols;

float motorPositions[4] = {0.0, 0.0, 0.0, 0.0};
float motorVelocities[4] = {0.0, 0.0, 0.0, 0.0};
uint8_t* packet;
static uint32_t timeout = millis();

// Global variables to hold the latest encoder feedback values.
float motor1_position = 0.0;
float motor1_velocity = 0.0;
float motor2_position = 0.0;
float motor2_velocity = 0.0;
float motor3_position = 0.0;
float motor3_velocity = 0.0;
float motor4_position = 0.0;
float motor4_velocity = 0.0;

// CAN messages for torque commands and close loop control.
CAN_message_t motor1_torque_cmd = r1806_protocols.encodeTorqueCommand(1, 0.0);
CAN_message_t motor1_close_loop_request_cmd = r1806_protocols.encodeRequestedStateCommand(1, ODrive.Axis.AxisState.CLOSED_LOOP_CONTROL);

CAN_message_t motor2_torque_cmd = r1806_protocols.encodeTorqueCommand(1, 0.0);
CAN_message_t motor2_close_loop_request_cmd = r1806_protocols.encodeRequestedStateCommand(1, ODrive.Axis.AxisState.CLOSED_LOOP_CONTROL);

CAN_message_t motor3_torque_cmd = r1806_protocols.encodeTorqueCommand(1, 0.0);
CAN_message_t motor3_close_loop_request_cmd = r1806_protocols.encodeRequestedStateCommand(1, ODrive.Axis.AxisState.CLOSED_LOOP_CONTROL);

CAN_message_t motor4_torque_cmd = r1806_protocols.encodeTorqueCommand(1, 0.0);
CAN_message_t motor4_close_loop_request_cmd = r1806_protocols.encodeRequestedStateCommand(1, ODrive.Axis.AxisState.CLOSED_LOOP_CONTROL);

// Timeout parameters: if no valid torque command is received within 100 ms, set torques to 0.
const uint32_t TORQUE_TIMEOUT_MS = 100;
uint32_t lastTorqueCmdTime = millis();

void canSniff(const CAN_message_t &msg) {
    // Extract motor id and command id from the CAN message
    uint32_t motor_id = r1806_protocols.extract_node_id(msg.id);
    uint32_t cmd_id = r1806_protocols.extract_command_id(msg.id);

    if(motor_id == 1) {
        if(cmd_id == 9) {
            motor1_position = r1806_protocols.decodePositionFeedback(msg);
            motor1_velocity = r1806_protocols.decodeVelocityFeedback(msg);
        }
    }
    else if(motor_id == 2) {
        if(cmd_id == 9) {
            motor2_position = r1806_protocols.decodePositionFeedback(msg);
            motor2_velocity = r1806_protocols.decodeVelocityFeedback(msg);
        }
    }
    else if(motor_id == 3) {
        if(cmd_id == 9) {
            motor3_position = r1806_protocols.decodePositionFeedback(msg);
            motor3_velocity = r1806_protocols.decodeVelocityFeedback(msg);
        }
    }
    else if(motor_id == 4) {
        if(cmd_id == 9) {
            motor4_position = r1806_protocols.decodePositionFeedback(msg);
            motor4_velocity = r1806_protocols.decodeVelocityFeedback(msg);
        }
    }
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
}

void loop() {
    can1.events();

    // --- Torque Command Reception ---
    if (Serial.available() >= TORQUE_PACKET_SIZE) {
        uint8_t torqueBuffer[TORQUE_PACKET_SIZE];
        // Cast is required because readBytes expects a char pointer.
        Serial.readBytes((char*)torqueBuffer, TORQUE_PACKET_SIZE);
        float* torques = serial_protocols.decodeTorquePacket(torqueBuffer);
        if (torques != nullptr) {
            motor1_torque_cmd = r1806_protocols.encodeTorqueCommand(1, torques[0]);
            motor2_torque_cmd = r1806_protocols.encodeTorqueCommand(2, torques[1]);
            motor3_torque_cmd = r1806_protocols.encodeTorqueCommand(3, torques[2]);
            motor4_torque_cmd = r1806_protocols.encodeTorqueCommand(4, torques[3]);
            // Update the last valid torque command time.
            lastTorqueCmdTime = millis();
        }
    }
    
    // --- Timeout Check: if no torque command received recently, reset torques to 0 ---
    if (millis() - lastTorqueCmdTime > TORQUE_TIMEOUT_MS) {
        motor1_torque_cmd = r1806_protocols.encodeTorqueCommand(1, 0.0);
        motor2_torque_cmd = r1806_protocols.encodeTorqueCommand(2, 0.0);
        motor3_torque_cmd = r1806_protocols.encodeTorqueCommand(3, 0.0);
        motor4_torque_cmd = r1806_protocols.encodeTorqueCommand(4, 0.0);
    }

    // --- Send Torque Commands at 50 Hz ---
    if (millis() - timeout > 20) {
        can1.write(motor1_torque_cmd);
        can1.write(motor2_torque_cmd);
        can1.write(motor3_torque_cmd);
        can1.write(motor4_torque_cmd);
        timeout = millis();
    }

    // --- Update Encoder Feedback Data ---
    motorPositions[0] = motor1_position;
    motorPositions[1] = motor2_position;
    motorPositions[2] = motor3_position;
    motorPositions[3] = motor4_position;
    motorVelocities[0] = motor1_velocity;
    motorVelocities[1] = motor2_velocity;
    motorVelocities[2] = motor3_velocity;
    motorVelocities[3] = motor4_velocity;

    // --- Send Feedback Packet over Serial ---
    packet = serial_protocols.encodeFeedbackPacket(motorPositions, motorVelocities);
    Serial.write(packet, FEEDBACK_PACKET_SIZE);
}