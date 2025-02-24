#include <string.h>
#include "finger/finger.h"

/**
 * @brief Constructs a new Finger object.
 *
 * The constructor currently does not perform additional initialization.
 */
Finger::Finger() {
}

/**
 * @brief Computes the finger joint states from motor states.
 *
 * This function calculates the positions of two joints by converting motor states using
 * the specified pulley diameters. The first joint is computed as the difference between
 * two positions (averaged), while the second joint is a combination of four positions.
 *
 * @param motor_states An array of motor states (length NUM_MOTORS).
 * @return Pointer to an array containing the computed joint states (length NUM_JOINTS).
 */
float* Finger::getJointStates(const float motor_states[NUM_MOTORS]) {
    // Calculate positions for joint 1 using motor states 0 and 1.
    float joint1_position_1 = motor_states[0] * D_MOTOR_PULLEY / D_FINGER_PULLEY_1;
    float joint1_position_2 = motor_states[1] * D_MOTOR_PULLEY / D_FINGER_PULLEY_1;

    // Calculate positions for joint 2 using motor states 0, 1, 2, and 3.
    float joint2_position_1 = motor_states[0] * D_MOTOR_PULLEY / D_FINGER_PULLEY_2;
    float joint2_position_2 = motor_states[1] * D_MOTOR_PULLEY / D_FINGER_PULLEY_2;
    float joint2_position_3 = motor_states[2] * D_MOTOR_PULLEY / D_FINGER_PULLEY_2;
    float joint2_position_4 = motor_states[3] * D_MOTOR_PULLEY / D_FINGER_PULLEY_2;

    // Compute joint states by averaging the appropriate motor states.
    _joint_states[0] = (joint1_position_1 - joint1_position_2) / 2.0f;
    _joint_states[1] = (joint2_position_1 + joint2_position_2 - joint2_position_3 + joint2_position_4) / 4.0f;
    
    return _joint_states;
}

/**
 * @brief Computes motor commands based on desired joint inputs.
 *
 * This function derives motor commands from the provided joint inputs. For now, it only uses the first
 * joint input to compute commands for motor 0 and motor 2, with motor 1 and motor 3 set to zero.
 *
 * @param joint_inputs An array of desired joint inputs (length NUM_JOINTS).
 * @return Pointer to an array containing the computed motor commands (length NUM_MOTORS).
 */
float* Finger::getMotorCommands(const float joint_inputs[NUM_JOINTS]) {
    _motor_commands[0] = joint_inputs[0] / 2.0f;
    _motor_commands[1] = 0.0f;
    _motor_commands[2] = -joint_inputs[0] / 2.0f;
    _motor_commands[3] = 0.0f;

    return _motor_commands;
}
