#ifndef SERIAL_PROTOCOLS_H
#define SERIAL_PROTOCOLS_H

#include <stdint.h>
#include "serial_protocols/utils.h"

/**
 * @file serial_protocols.h
 * @brief Declaration of the SerialProtocols class for handling serial communication.
 */

/**
 * @def NUM_JOINTS
 * @brief Number of joints used in the serial communication protocol.
 */
#define NUM_JOINTS 2

/**
 * @class SerialProtocols
 * @brief Provides functions to encode and decode serial packets for feedback and joint commands.
 *
 * This class includes methods to compute packet checksums, encode feedback packets,
 * and decode joint command packets received via serial communication.
 */
class SerialProtocols{
public:
    /**
     * @brief Constructs a new SerialProtocols object.
     */
    SerialProtocols();

    /**
     * @brief Computes a checksum for a serial packet.
     *
     * The checksum is computed as the 8-bit sum of the message type, payload length,
     * and each byte of the payload.
     *
     * @param msgType The message type byte.
     * @param payloadLen The length of the payload.
     * @param payload Pointer to the payload data.
     * @return Computed 8-bit checksum.
     */
    uint8_t computeChecksum(uint8_t msgType, uint8_t payloadLen, const uint8_t* payload);

    /**
     * @brief Encodes a feedback packet from joint positions.
     *
     * The feedback packet is constructed using a start byte, message type, payload length,
     * the payload containing two float positions, a checksum, and an end byte.
     *
     * @param positions An array of joint positions (2 floats).
     * @return Pointer to the encoded feedback packet.
     */
    uint8_t* encodeFeedbackPacket(const float positions[NUM_JOINTS]);

    /**
     * @brief Decodes a joint command packet.
     *
     * This method validates the packet structure (start byte, message type, payload length,
     * checksum, and end byte) and decodes the payload into joint command values.
     *
     * @param data Pointer to the received packet data.
     * @return Pointer to the decoded joint command array, or nullptr if validation fails.
     */
    float* decodeJointCommandPacket(const uint8_t* data);

private:
    /// Buffer for storing the encoded feedback packet.
    uint8_t _encoder_feedback_packet[FEEDBACK_PACKET_SIZE];

    /// Buffer for storing the received joint command packet.
    uint8_t _joint_command_packet[JOINT_COMMAND_PACKET_SIZE];

    /// Array to hold decoded joint commands (size NUM_MOTORS).
    float _joint_command[NUM_MOTORS];
};

#endif // SERIAL_PROTOCOLS_H
