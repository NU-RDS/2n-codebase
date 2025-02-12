#include <stdint.h>
#include <string.h>
#include"serial_protocols/serial_protocols.h"

SerialProtocols::SerialProtocols() {
}

uint8_t SerialProtocols::computeChecksum(uint8_t msgType, uint8_t payloadLen, const uint8_t* payload){
    uint16_t sum = msgType + payloadLen;
    for (uint8_t i = 0; i < payloadLen; i++) {
        sum += payload[i];
    }
    return (uint8_t)(sum & 0xFF);
}

uint8_t* SerialProtocols::encodeFeedbackPacket(const float positions[4], const float velocities[4]){
    _packet[0] = START_BYTE;
    _packet[1] = MSG_FEEDBACK;
    _packet[2] = FEEDBACK_PAYLOAD_LENGTH;
    memcpy(&_packet[3], positions, 4 * sizeof(float));
    memcpy(&_packet[3 + 16], velocities, 4 * sizeof(float));
    uint8_t checksum = computeChecksum(_packet[1], _packet[2], &_packet[3]);
    _packet[3 + FEEDBACK_PAYLOAD_LENGTH] = checksum;  // index 35
    _packet[3 + FEEDBACK_PAYLOAD_LENGTH + 1] = END_BYTE;  // index 36
    return _packet;
}