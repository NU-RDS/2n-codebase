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

uint8_t* SerialProtocols::encodeFeedbackPacket(const float positions[NUM_JOINTS]){
    _encoder_feedback_packet[0] = START_BYTE;
    _encoder_feedback_packet[1] = MSG_FEEDBACK;
    _encoder_feedback_packet[2] = FEEDBACK_PAYLOAD_LENGTH;
    memcpy(&_encoder_feedback_packet[3], positions, 2 * sizeof(float));
    uint8_t checksum = computeChecksum(_encoder_feedback_packet[1], _encoder_feedback_packet[2], &_encoder_feedback_packet[3]);
    _encoder_feedback_packet[3 + FEEDBACK_PAYLOAD_LENGTH] = checksum;  // index 35
    _encoder_feedback_packet[3 + FEEDBACK_PAYLOAD_LENGTH + 1] = END_BYTE;  // index 36
    return _encoder_feedback_packet;
}

float* SerialProtocols::decodeJointCommandPacket(const uint8_t* data) {
    // Copy the incoming position packet data into our internal buffer.
    memcpy(_joint_command_packet, data, JOINT_COMMAND_PACKET_SIZE);

    // Expected position packet layout:
    // Byte 0: START_BYTE
    // Byte 1: MSG_JOINT_COMMAND (0x02)
    // Byte 2: JOINT_COMMAND_PAYLOAD_LENGTH (16)
    // Bytes 3 - 18: 4 float position commands (16 bytes)
    // Byte 19: Checksum (computed over bytes 1-18)
    // Byte 20: END_BYTE

    // Validate start byte
    if (_joint_command_packet[0] != START_BYTE) {
        return nullptr;
    }
    // Validate message type
    if (_joint_command_packet[1] != MSG_JOINT_COMMAND) {
        return nullptr;
    }
    // Validate payload length
    if (_joint_command_packet[2] != JOINT_COMMAND_PAYLOAD_LENGTH) {
        return nullptr;
    }
    // Compute and check the checksum
    uint8_t computedChecksum = computeChecksum(_joint_command_packet[1], _joint_command_packet[2], &_joint_command_packet[3]);
    if (computedChecksum != _joint_command_packet[3 + JOINT_COMMAND_PAYLOAD_LENGTH]) {
        return nullptr;
    }
    // Validate end byte
    if (_joint_command_packet[3 + JOINT_COMMAND_PAYLOAD_LENGTH + 1] != END_BYTE) {
        return nullptr;
    }

    // If valid, decode the payload into our position commands array.
    memcpy(_joint_command, &_joint_command_packet[3], JOINT_COMMAND_PAYLOAD_LENGTH);
    return _joint_command;
}
