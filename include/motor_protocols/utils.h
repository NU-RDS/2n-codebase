#ifndef MOTOR_PROTOCOLS_UTILS_H
#define MOTOR_PROTOCOLS_UTILS_H

#include <stdint.h>

/**
 * @file utils.h
 * @brief Contains utility data structures and definitions for motor protocols.
 */

/**
 * @struct ODrive
 * @brief Namespace-like struct containing ODrive axis state values.
 *
 * The ODrive struct is used to hold various enumerations that represent
 * different axis states in an ODrive motor driver.
 */
struct {
    /**
     * @struct Axis
     * @brief Nested struct for handling axis-based data and states.
     */
    struct {
        /**
         * @struct AxisState
         * @brief Holds constants representing possible ODrive axis states.
         */
        struct {
            uint32_t UNDEFINED = 0;                        /**< Undefined state. */
            uint32_t IDLE = 1;                             /**< Idle state. */
            uint32_t STARTUP_SEQUENCE = 2;                 /**< Startup sequence in progress. */
            uint32_t FULL_CALIBRATION_SEQUENCE = 3;        /**< Full calibration sequence. */
            uint32_t MOTOR_CALIBRATION = 4;                /**< Motor calibration. */
            uint32_t ENCODER_INDEX_SEARCH = 6;             /**< Encoder index search. */
            uint32_t ENCODER_OFFSET_CALIBRATION = 7;       /**< Encoder offset calibration. */
            uint32_t CLOSED_LOOP_CONTROL = 8;              /**< Closed-loop control. */
            uint32_t LOCKIN_SPIN = 9;                      /**< Lock-in spin. */
            uint32_t ENCODER_DIR_FIND = 10;                /**< Encoder direction find. */
            uint32_t HOMING = 11;                          /**< Homing procedure. */
            uint32_t ENCODER_HALL_POLARITY_CALIBRATION = 12; /**< Hall polarity calibration. */
            uint32_t ENCODER_HALL_PHASE_CALIBRATION = 13;  /**< Hall phase calibration. */
            uint32_t ANTICOGGING_CALIBRATION = 14;         /**< Anti-cogging calibration. */
        } AxisState;
    } Axis;
} ODrive;

#endif // MOTOR_PROTOCOLS_UTILS_H
