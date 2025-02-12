#include <FlexCAN_T4.h>
#include "motor_protocols/r1806.h"
#include "motor_protocols/utils.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t msg;
R1806 r1806_protocols;
static uint32_t timeout = millis();

float motor1_position = 0.0;
float motor1_velocity = 0.0;
CAN_message_t motor1_torque_cmd = r1806_protocols.encodeTorqueCommand(1, 0.0);
CAN_message_t motor1_close_loop_request_cmd = r1806_protocols.encodeRequestedStateCommand(1, ODrive.Axis.AxisState.CLOSED_LOOP_CONTROL);

void canSniff(const CAN_message_t &msg) {
    // Extract the motor id and command id from the CAN message
    uint32_t motor_id = r1806_protocols.extract_node_id(msg.id);
    uint32_t cmd_id = r1806_protocols.extract_command_id(msg.id);

    // Motor 1
    if(motor_id == 1) {
        // 0x009: Position Feedback
        if(cmd_id == 9) {
            motor1_position = r1806_protocols.decodePositionFeedback(msg);
            motor1_velocity = r1806_protocols.decodeVelocityFeedback(msg);
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
}

void loop() {
    can1.events();
    Serial.println("Motor 1 Position: " + String(motor1_position) + " rad. " + "Motor 1 Velocity: " + String(motor1_velocity) + " rad/s");

    motor1_torque_cmd = r1806_protocols.encodeTorqueCommand(1, -0.03);

    if ( millis() - timeout > 500 ) {
        can1.write(motor1_torque_cmd);
        timeout = millis();
    }

}
