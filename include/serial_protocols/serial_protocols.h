#ifndef SERIAL_PROTOCOLS_H
#define SERIAL_PROTOCOLS_H

#include <stdint.h>
#include "serial_protocols/utils.h"

class SerialProtocols{
    public:
        SerialProtocols();
        uint8_t computeChecksum(uint8_t msgType, uint8_t payloadLen, const uint8_t* payload);
        uint8_t* encodeFeedbackPacket(const float positions[4], const float velocities[4]);

    private:
        uint8_t _packet[FEEDBACK_PACKET_SIZE];
};

#endif
