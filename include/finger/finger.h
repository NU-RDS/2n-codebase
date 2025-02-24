#ifndef FINGER_H
#define FINGER_H

#include <stdint.h>

/**
 * @file finger.h
 * @brief Declaration of the Finger class and related constants.
 */

/**
 * @def NUM_JOINTS
 * @brief Number of finger joints.
 */
#define NUM_JOINTS 2

/**
 * @def NUM_MOTORS
 * @brief Number of motors controlling the finger.
 */
#define NUM_MOTORS 4

/**
 * @def D_MOTOR_PULLEY
 * @brief Diameter of the motor pulley in millimeters.
 */
#define D_MOTOR_PULLEY 11.9998f // 12 mm

/**
 * @def D_FINGER_PULLEY_1
 * @brief Diameter of the first finger pulley in millimeters.
 */
#define D_FINGER_PULLEY_1 29.0f // 29 mm

/**
 * @def D_FINGER_PULLEY_2
 * @brief Diameter of the second finger pulley in millimeters.
 */
#define D_FINGER_PULLEY_2 35.0f // 35 mm

/**
 * @class Finger
 * @brief Provides functionality to convert between motor states and finger joint states.
 *
 * This class defines methods to compute joint states from motor states and vice versa.
 */
class Finger {
public:
    /**
     * @brief Constructs a new Finger object.
     */
    Finger();

    /**
     * @brief Computes the finger joint states based on motor states.
     *
     * Given an array of motor states, this function calculates the positions of the finger joints.
     *
     * @param motor_states An array of motor states with length NUM_MOTORS.
     * @return A pointer to an array of computed joint states (length NUM_JOINTS).
     */
    float* getJointStates(const float motor_states[NUM_MOTORS]);

    /**
     * @brief Computes motor commands based on desired joint inputs.
     *
     * Given an array of desired joint inputs, this function calculates the motor commands required.
     *
     * @param joint_inputs An array of desired joint inputs with length NUM_JOINTS.
     * @return A pointer to an array of computed motor commands (length NUM_MOTORS).
     */
    float* getMotorCommands(const float joint_inputs[NUM_JOINTS]);

private:
    /// Array storing the current joint states.
    float _joint_states[NUM_JOINTS];
    
    /// Array storing the computed motor commands.
    float _motor_commands[NUM_MOTORS];
};

#endif // FINGER_H
