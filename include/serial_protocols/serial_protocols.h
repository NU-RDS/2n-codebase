#ifndef SERIAL_PROTOCOLS_H
#define SERIAL_PROTOCOLS_H

#include <stdint.h>
#include "serial_protocols/utils.h"

class SerialProtocols{
    public:
        SerialProtocols();
        uint8_t computeChecksum(uint8_t msgType, uint8_t payloadLen, const uint8_t* payload);
        uint8_t* encodeFeedbackPacket(const float positions[4], const float velocities[4]);
        float* decodeTorquePacket(const uint8_t* data);

    private:
        uint8_t _encoder_feedback_packet[FEEDBACK_PACKET_SIZE];
        uint8_t _torque_packet[TORQUE_PACKET_SIZE];
        float _torque_commands[NUM_MOTORS];
};

#endif
