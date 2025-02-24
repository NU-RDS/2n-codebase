#ifndef R1806_H
#define R1806_H

/**
 * @file r1806.h
 * @brief Motor protocols for the R1806 motor used with ODrive Pro.
 *
 * The R1806 motor is fabricated by Repeat Robotics, and this file
 * defines the R1806 class for CAN-based communication with ODrive.
 *
 * @details
 * - R1806 motor details: https://repeat-robotics.com/buy/repeat-compact-1806
 * - ODrive CAN Protocol Details: https://docs.odriverobotics.com/v/latest/manual/can-protocol.html
 */

#include <FlexCAN_T4.h>

/**
 * @def GEAR_REDUCION_RATIO
 * @brief Gear reduction ratio for the R1806 motor.
 */
#define GEAR_REDUCION_RATIO 22.6f

/**
 * @class R1806
 * @brief Provides methods to encode and decode CAN messages for controlling an R1806 motor via ODrive.
 *
 * This class contains utility functions to extract node/command IDs, encode state and torque commands,
 * and decode feedback such as position and velocity.
 */
class R1806
{
public:
    /**
     * @brief Constructs a new R1806 object.
     *
     * Initializes internal parameters, including the gear reduction ratio.
     */
    R1806();

    /**
     * @brief Extract the node ID from a given CAN identifier.
     *
     * @param can_id 29-bit or 11-bit CAN identifier.
     * @return Node ID (extracted bits from can_id).
     */
    uint32_t extract_node_id(uint32_t can_id);

    /**
     * @brief Extract the command ID from a given CAN identifier.
     *
     * @param can_id 29-bit or 11-bit CAN identifier.
     * @return Command ID (extracted bits from can_id).
     */
    uint32_t extract_command_id(uint32_t can_id);

    /**
     * @brief Encode a "requested state" command for ODrive.
     *
     * Creates a CAN message instructing ODrive to enter a specific axis state.
     *
     * @param node_id ID of the node (motor) on the CAN bus.
     * @param requested_state The requested axis state (e.g., CLOSED_LOOP_CONTROL).
     * @return A CAN_message_t struct with the encoded command.
     */
    CAN_message_t encodeRequestedStateCommand(uint32_t node_id, uint32_t requested_state);

    /**
     * @brief Encode a torque command for the ODrive.
     *
     * Creates a CAN message to set a torque value for a given node. The torque is
     * automatically adjusted by the gear reduction ratio.
     *
     * @param node_id ID of the node (motor) on the CAN bus.
     * @param torque Desired torque in Nm (or consistent units).
     * @return A CAN_message_t struct with the encoded torque command.
     */
    CAN_message_t encodeTorqueCommand(uint32_t node_id, float torque);

    /**
     * @brief Decode position feedback from a CAN message.
     *
     * Interprets the first 4 bytes of the CAN message payload as a float for the
     * number of revolutions, then converts it to a position (radians) factoring
     * in the gear reduction ratio.
     *
     * @param frame A CAN_message_t containing position feedback data.
     * @return Position in radians.
     */
    float decodePositionFeedback(CAN_message_t frame);

    /**
     * @brief Decode velocity feedback from a CAN message.
     *
     * Interprets bytes [4..7] of the CAN message payload as a float for the number
     * of revolutions per second, then converts it to a velocity (rad/s), factoring
     * in the gear reduction ratio.
     *
     * @param frame A CAN_message_t containing velocity feedback data.
     * @return Velocity in rad/s.
     */
    float decodeVelocityFeedback(CAN_message_t frame);

private:
    /// Internal gear reduction ratio used for torque and feedback calculations.
    float _gear_reduction_ratio;
};

#endif // R1806_H
