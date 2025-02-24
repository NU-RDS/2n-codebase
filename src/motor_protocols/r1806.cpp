#include <cmath>
#include <cstring>
#include "motor_protocols/r1806.h"
#include "motor_protocols/utils.h"

/**
 * @brief Constructs a new R1806 object and sets the gear reduction ratio.
 */
R1806::R1806() {
    _gear_reduction_ratio = GEAR_REDUCION_RATIO;
}

/**
 * @brief Extract the node ID from a CAN ID.
 *
 * Node ID occupies bits [10..5] of a standard 11-bit ID. We mask
 * and shift to retrieve it.
 *
 * @param can_id The raw CAN identifier.
 * @return Extracted node ID.
 */
uint32_t R1806::extract_node_id(uint32_t can_id) {
    return (can_id >> 5) & 0x3F;
}

/**
 * @brief Extract the command ID from a CAN ID.
 *
 * Command ID occupies bits [4..0] of the CAN identifier. We mask
 * out the lower 5 bits to retrieve it.
 *
 * @param can_id The raw CAN identifier.
 * @return Extracted command ID.
 */
uint32_t R1806::extract_command_id(uint32_t can_id) {
    return can_id & 0x1F;
}

/**
 * @brief Encode a "requested state" command into a CAN message.
 *
 * This implementation forces the state to CLOSED_LOOP_CONTROL for demonstration.
 * You can modify it to use the passed-in requested_state argument fully.
 *
 * @param node_id The target node (motor) ID.
 * @param requested_state The desired axis state (not fully used in this example).
 * @return A CAN_message_t configured with the appropriate ID and payload.
 */
CAN_message_t R1806::encodeRequestedStateCommand(uint32_t node_id, uint32_t requested_state) {
    CAN_message_t msg;
    // Create a CAN ID from node_id (bits [10..5]) and the command ID (bits [4..0]).
    msg.id = ((node_id & 0x3F) << 5) | (0x07 & 0x1F);

    // Overriding requested_state to CLOSED_LOOP_CONTROL (for demonstration).
    requested_state = ODrive.Axis.AxisState.CLOSED_LOOP_CONTROL;

    // Fill the payload with the requested axis state in little-endian byte order.
    msg.buf[0] = requested_state & 0xFF;
    msg.buf[1] = (requested_state >> 8) & 0xFF;
    msg.buf[2] = (requested_state >> 16) & 0xFF;
    msg.buf[3] = (requested_state >> 24) & 0xFF;
    // Unused bytes are set to zero.
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = 0;
    msg.buf[7] = 0;

    return msg;
}

/**
 * @brief Encode a torque command into a CAN message.
 *
 * The gear reduction ratio is factored into the torque command so that
 * the motor sees the correct torque value. The 4-byte float is written
 * in little-endian format into the CAN frame buffer.
 *
 * @param node_id The target node (motor) ID.
 * @param torque Desired torque before gear reduction is accounted for.
 * @return A CAN_message_t with the correct ID and payload to set torque on ODrive.
 */
CAN_message_t R1806::encodeTorqueCommand(uint32_t node_id, float torque) {
    CAN_message_t msg;
    // Create a CAN ID from node_id (bits [10..5]) and the command ID (bits [4..0]).
    msg.id = ((node_id & 0x3F) << 5) | (0x0E & 0x1F);

    // Adjust torque by the gear reduction ratio.
    float value = torque / _gear_reduction_ratio;
    uint32_t torque_int;
    std::memcpy(&torque_int, &value, sizeof(torque_int));

    // Write the float to the first 4 bytes of the buffer in little-endian order.
    msg.buf[0] = torque_int & 0xFF;
    msg.buf[1] = (torque_int >> 8) & 0xFF;
    msg.buf[2] = (torque_int >> 16) & 0xFF;
    msg.buf[3] = (torque_int >> 24) & 0xFF;
    // Unused bytes are set to zero.
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = 0;
    msg.buf[7] = 0;

    return msg;
}

/**
 * @brief Decode position feedback from a CAN message.
 *
 * The first 4 bytes of the CAN frame are interpreted as a float
 * representing the number of revolutions. This is then converted
 * to radians, factoring in the gear reduction ratio.
 *
 * @param frame A CAN_message_t containing position data in its first 4 bytes.
 * @return Position in radians.
 */
float R1806::decodePositionFeedback(CAN_message_t frame) {
    // Interpret bytes [0..3] as a float containing number of revolutions.
    float num_rounds = *(reinterpret_cast<const float*>(frame.buf));
    // Convert to radians, accounting for gear reduction.
    float position = num_rounds * M_PI * 2.0f / _gear_reduction_ratio;
    return position;
}

/**
 * @brief Decode velocity feedback from a CAN message.
 *
 * Bytes [4..7] of the CAN frame are interpreted as a float representing
 * the number of revolutions per second, then converted to rad/s with
 * gear reduction considered.
 *
 * @param frame A CAN_message_t containing velocity data in bytes [4..7].
 * @return Velocity in rad/s.
 */
float R1806::decodeVelocityFeedback(CAN_message_t frame) {
    // Interpret bytes [4..7] as a float containing revolutions per second.
    float num_rounds_per_sec = *(reinterpret_cast<const float*>(frame.buf + 4));
    // Convert to rad/s, factoring in gear reduction.
    float velocity = num_rounds_per_sec * M_PI * 2.0f / _gear_reduction_ratio;
    return velocity;
}
