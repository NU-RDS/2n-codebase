#ifndef SERIAL_PROTOCOLS_H
#define SERIAL_PROTOCOLS_H

#include <stdint.h>
#include "serial_protocols/utils.h"

#define NUM_JOINTS 2

class SerialProtocols{
    public:
        SerialProtocols();
        uint8_t computeChecksum(uint8_t msgType, uint8_t payloadLen, const uint8_t* payload);
        uint8_t* encodeFeedbackPacket(const float positions[NUM_JOINTS]);
        float* decodeJointCommandPacket(const uint8_t* data);

    private:
        uint8_t _encoder_feedback_packet[FEEDBACK_PACKET_SIZE];
        uint8_t _joint_command_packet[JOINT_COMMAND_PACKET_SIZE];
        float _joint_command[NUM_MOTORS];
};

#endif
