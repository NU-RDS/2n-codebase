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
    _encoder_feedback_packet[0] = START_BYTE;
    _encoder_feedback_packet[1] = MSG_FEEDBACK;
    _encoder_feedback_packet[2] = FEEDBACK_PAYLOAD_LENGTH;
    memcpy(&_encoder_feedback_packet[3], positions, 4 * sizeof(float));
    memcpy(&_encoder_feedback_packet[3 + 16], velocities, 4 * sizeof(float));
    uint8_t checksum = computeChecksum(_encoder_feedback_packet[1], _encoder_feedback_packet[2], &_encoder_feedback_packet[3]);
    _encoder_feedback_packet[3 + FEEDBACK_PAYLOAD_LENGTH] = checksum;  // index 35
    _encoder_feedback_packet[3 + FEEDBACK_PAYLOAD_LENGTH + 1] = END_BYTE;  // index 36
    return _encoder_feedback_packet;
}

float* SerialProtocols::decodeTorquePacket(const uint8_t* data) {
    // Copy the incoming torque packet data into our internal buffer.
    memcpy(_torque_packet, data, TORQUE_PACKET_SIZE);

    // Expected torque packet layout:
    // Byte 0: START_BYTE
    // Byte 1: MSG_TORQUE (0x02)
    // Byte 2: TORQUE_PAYLOAD_LENGTH (16)
    // Bytes 3 - 18: 4 float torque commands (16 bytes)
    // Byte 19: Checksum (computed over bytes 1-18)
    // Byte 20: END_BYTE

    // Validate start byte
    if (_torque_packet[0] != START_BYTE) {
        return nullptr;
    }
    // Validate message type
    if (_torque_packet[1] != MSG_TORQUE) {
        return nullptr;
    }
    // Validate payload length
    if (_torque_packet[2] != TORQUE_PAYLOAD_LENGTH) {
        return nullptr;
    }
    // Compute and check the checksum
    uint8_t computedChecksum = computeChecksum(_torque_packet[1], _torque_packet[2], &_torque_packet[3]);
    if (computedChecksum != _torque_packet[3 + TORQUE_PAYLOAD_LENGTH]) {
        return nullptr;
    }
    // Validate end byte
    if (_torque_packet[3 + TORQUE_PAYLOAD_LENGTH + 1] != END_BYTE) {
        return nullptr;
    }

    // If valid, decode the payload into our torque commands array.
    memcpy(_torque_commands, &_torque_packet[3], TORQUE_PAYLOAD_LENGTH);
    return _torque_commands;
}