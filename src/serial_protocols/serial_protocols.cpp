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

uint8_t* SerialProtocols::encodeFeedbackPacket(const float positions[4]){
    _encoder_feedback_packet[0] = START_BYTE;
    _encoder_feedback_packet[1] = MSG_FEEDBACK;
    _encoder_feedback_packet[2] = FEEDBACK_PAYLOAD_LENGTH;
    memcpy(&_encoder_feedback_packet[3], positions, 4 * sizeof(float));
    uint8_t checksum = computeChecksum(_encoder_feedback_packet[1], _encoder_feedback_packet[2], &_encoder_feedback_packet[3]);
    _encoder_feedback_packet[3 + FEEDBACK_PAYLOAD_LENGTH] = checksum;  // index 35
    _encoder_feedback_packet[3 + FEEDBACK_PAYLOAD_LENGTH + 1] = END_BYTE;  // index 36
    return _encoder_feedback_packet;
}

float* SerialProtocols::decodePositionPacket(const uint8_t* data) {
    // Copy the incoming position packet data into our internal buffer.
    memcpy(_position_packet, data, POSITION_PACKET_SIZE);

    // Expected position packet layout:
    // Byte 0: START_BYTE
    // Byte 1: MSG_POSITION (0x02)
    // Byte 2: POSITION_PAYLOAD_LENGTH (16)
    // Bytes 3 - 18: 4 float position commands (16 bytes)
    // Byte 19: Checksum (computed over bytes 1-18)
    // Byte 20: END_BYTE

    // Validate start byte
    if (_position_packet[0] != START_BYTE) {
        return nullptr;
    }
    // Validate message type
    if (_position_packet[1] != MSG_POSITION) {
        return nullptr;
    }
    // Validate payload length
    if (_position_packet[2] != POSITION_PAYLOAD_LENGTH) {
        return nullptr;
    }
    // Compute and check the checksum
    uint8_t computedChecksum = computeChecksum(_position_packet[1], _position_packet[2], &_position_packet[3]);
    if (computedChecksum != _position_packet[3 + POSITION_PAYLOAD_LENGTH]) {
        return nullptr;
    }
    // Validate end byte
    if (_position_packet[3 + POSITION_PAYLOAD_LENGTH + 1] != END_BYTE) {
        return nullptr;
    }

    // If valid, decode the payload into our position commands array.
    memcpy(_position_commands, &_position_packet[3], POSITION_PAYLOAD_LENGTH);
    return _position_commands;
}
