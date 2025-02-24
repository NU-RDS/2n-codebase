#ifndef SERIAL_PROTOCOLS_UTILS_H
#define SERIAL_PROTOCOLS_UTILS_H

#include <stdint.h>

/**
 * @file utils.h
 * @brief Defines constants used in the serial communication protocols.
 */

/**
 * @def NUM_MOTORS
 * @brief Number of motors controlled via the serial protocol.
 */
#define NUM_MOTORS 4

/**
 * @def START_BYTE
 * @brief Special byte indicating the start of a packet.
 */
#define START_BYTE 0xAA

/**
 * @def END_BYTE
 * @brief Special byte indicating the end of a packet.
 */
#define END_BYTE   0xBB

/**
 * @def MSG_FEEDBACK
 * @brief Message type for feedback packets.
 */
#define MSG_FEEDBACK 0x01

/**
 * @def FEEDBACK_PAYLOAD_LENGTH
 * @brief Payload length for feedback packets (contains 2 floats).
 */
#define FEEDBACK_PAYLOAD_LENGTH 8 // 2 floats

/**
 * @def FEEDBACK_PACKET_SIZE
 * @brief Total packet size for feedback packets.
 *
 * Calculated as: START_BYTE + MSG_FEEDBACK + payload length + checksum + END_BYTE.
 */
#define FEEDBACK_PACKET_SIZE 13 // 1 + 1 + 1 + FEEDBACK_PAYLOAD_LENGTH + 1 + 1

/**
 * @def MSG_JOINT_COMMAND
 * @brief Message type for joint command packets.
 */
#define MSG_JOINT_COMMAND 0x02

/**
 * @def JOINT_COMMAND_PAYLOAD_LENGTH
 * @brief Payload length for joint command packets (contains 2 floats).
 */
#define JOINT_COMMAND_PAYLOAD_LENGTH 8 // 2 floats

/**
 * @def JOINT_COMMAND_PACKET_SIZE
 * @brief Total packet size for joint command packets.
 *
 * Calculated as: START_BYTE + MSG_JOINT_COMMAND + payload length + checksum + END_BYTE.
 */
#define JOINT_COMMAND_PACKET_SIZE 13 // 1 + 1 + 1 + JOINT_COMMAND_PAYLOAD_LENGTH + 1 + 1

#endif // SERIAL_PROTOCOLS_UTILS_H
