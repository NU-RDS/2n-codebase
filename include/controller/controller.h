#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>
#include "finger/finger.h"
#include <iostream>
#include <vector>
#include <array>
#include <chrono>
#include <math.h>

/**
 * @file controller.h
 * @brief Declaration of the Controller class for torque control.
 */

/**
 * @class Controller
 * @brief Provides torque control functionality using a PID algorithm.
 *
 * This class calculates joint torques by comparing current and desired joint states,
 * accumulating joint errors, and adjusting PID gains accordingly.
 */
class Controller {
public:
    /**
     * @brief Constructs a new Controller object.
     *
     * Initializes the PID gains and transformation matrix used for converting joint torques
     * to motor torques.
     */
    Controller();

    /**
     * @brief Computes motor torques based on PID control.
     *
     * Calculates the difference between the desired and current joint states, updates the error
     * sum, and computes the derivative of the error from motor velocities. It then applies a PID
     * control algorithm to determine the joint torques, which are transformed into motor torques.
     *
     * @param joint_states Vector of current joint states.
     * @param joint_states_desired Vector of desired joint states.
     * @param joint_error_sum Vector containing the accumulated joint error over time.
     * @param motor_velocities Vector of current motor velocities.
     * @return A std::pair consisting of:
     *         - A vector of computed motor torques.
     *         - The updated joint error sum vector.
     */
    std::pair<std::vector<double>, std::vector<double>> torque_control(
        std::vector<double> joint_states,
        std::vector<double> joint_states_desired,
        std::vector<double> joint_error_sum,
        std::vector<double> motor_velocities
    );

    /**
     * @brief Adjusts PID gains based on the current joint error.
     *
     * Depending on whether the joint errors are negative or positive, the PID gains (kp, ki, kd)
     * are updated to use either lower or higher predefined values.
     *
     * @param joint_error Vector containing the error for each joint.
     */
    void check_error(std::vector<double> joint_error);

private:
    /// Instance of the Finger class used for finger control.
    Finger finger;

    /// Low and high proportional gains.
    double low_kp, high_kp;
    /// Low and high derivative gains.
    double low_kd, high_kd;
    /// Low and high integral gains.
    double low_ki, high_ki;

    /// Motor and finger pulley dimensions (expected to be defined elsewhere).
    double r, R1, R2; 

    /// 2x2 matrices for PID gains.
    std::array<std::array<double, 2>, 2> kp, kd, ki;
    
    /// 2x4 transformation matrix converting joint torques to motor torques.
    std::array<std::array<double, 4>, 2> trans_mat;
};

#endif // CONTROLLER_H
