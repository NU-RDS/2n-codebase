#include <stdint.h>
#include <string.h>
#include "serial_protocols/serial_protocols.h"

/**
 * @brief Constructs a new SerialProtocols object.
 */
SerialProtocols::SerialProtocols() {
}

/**
 * @brief Computes a checksum for a serial packet.
 *
 * The checksum is calculated by summing the message type, payload length, and every byte in the payload.
 * The result is truncated to an 8-bit value.
 *
 * @param msgType The message type.
 * @param payloadLen Length of the payload.
 * @param payload Pointer to the payload data.
 * @return An 8-bit checksum value.
 */
uint8_t SerialProtocols::computeChecksum(uint8_t msgType, uint8_t payloadLen, const uint8_t* payload) {
    uint16_t sum = msgType + payloadLen;
    for (uint8_t i = 0; i < payloadLen; i++) {
        sum += payload[i];
    }
    return (uint8_t)(sum & 0xFF);
}

/**
 * @brief Encodes a feedback packet from joint positions.
 *
 * The packet is structured as follows:
 * - Byte 0: START_BYTE
 * - Byte 1: MSG_FEEDBACK
 * - Byte 2: FEEDBACK_PAYLOAD_LENGTH
 * - Bytes 3-10: Payload (2 floats representing positions)
 * - Byte 11: Checksum (computed over bytes 1-10)
 * - Byte 12: END_BYTE
 *
 * @param positions An array of two float positions.
 * @return Pointer to the encoded feedback packet.
 */
uint8_t* SerialProtocols::encodeFeedbackPacket(const float positions[NUM_JOINTS]) {
    _encoder_feedback_packet[0] = START_BYTE;
    _encoder_feedback_packet[1] = MSG_FEEDBACK;
    _encoder_feedback_packet[2] = FEEDBACK_PAYLOAD_LENGTH;
    
    // Copy the two float positions into the payload.
    memcpy(&_encoder_feedback_packet[3], positions, 2 * sizeof(float));
    
    // Compute checksum over the message type, payload length, and payload.
    uint8_t checksum = computeChecksum(_encoder_feedback_packet[1], _encoder_feedback_packet[2], &_encoder_feedback_packet[3]);
    _encoder_feedback_packet[3 + FEEDBACK_PAYLOAD_LENGTH] = checksum;  // Set checksum byte.
    _encoder_feedback_packet[3 + FEEDBACK_PAYLOAD_LENGTH + 1] = END_BYTE;  // Set end byte.
    
    return _encoder_feedback_packet;
}

/**
 * @brief Decodes a joint command packet.
 *
 * The expected packet layout is:
 * - Byte 0: START_BYTE
 * - Byte 1: MSG_JOINT_COMMAND
 * - Byte 2: JOINT_COMMAND_PAYLOAD_LENGTH
 * - Bytes 3-10: Payload (4 float position commands, 16 bytes total)
 * - Byte 11: Checksum (computed over bytes 1-10)
 * - Byte 12: END_BYTE
 *
 * The method validates each part of the packet and returns a pointer to the decoded joint commands.
 * If any validation fails, the method returns nullptr.
 *
 * @param data Pointer to the received packet data.
 * @return Pointer to the decoded joint command array, or nullptr if the packet is invalid.
 */
float* SerialProtocols::decodeJointCommandPacket(const uint8_t* data) {
    // Copy the incoming packet data into the internal buffer.
    memcpy(_joint_command_packet, data, JOINT_COMMAND_PACKET_SIZE);

    // Validate start byte.
    if (_joint_command_packet[0] != START_BYTE) {
        return nullptr;
    }
    // Validate message type.
    if (_joint_command_packet[1] != MSG_JOINT_COMMAND) {
        return nullptr;
    }
    // Validate payload length.
    if (_joint_command_packet[2] != JOINT_COMMAND_PAYLOAD_LENGTH) {
        return nullptr;
    }
    // Compute and validate the checksum.
    uint8_t computedChecksum = computeChecksum(_joint_command_packet[1], _joint_command_packet[2], &_joint_command_packet[3]);
    if (computedChecksum != _joint_command_packet[3 + JOINT_COMMAND_PAYLOAD_LENGTH]) {
        return nullptr;
    }
    // Validate end byte.
    if (_joint_command_packet[3 + JOINT_COMMAND_PAYLOAD_LENGTH + 1] != END_BYTE) {
        return nullptr;
    }

    // Decode the payload into the joint command array.
    memcpy(_joint_command, &_joint_command_packet[3], JOINT_COMMAND_PAYLOAD_LENGTH);
    return _joint_command;
}
