#include <controller/controller.h>

/**
 * @brief Constructs a new Controller object.
 *
 * Initializes the PID gains, pulley dimensions, and transformation matrix used for motor torque conversion.
 * Note: D_MOTOR_PULLEY, D_FINGER_PULLEY_1, and D_FINGER_PULLEY_2 should be defined elsewhere.
 */
Controller::Controller()
    : low_kp(0.45), high_kp(0.65), low_kd(0.1), high_kd(0.2), low_ki(0.0), high_ki(0.0),
      kp{ std::array<double, 2>{low_kp, 0}, std::array<double, 2>{0, low_kp} },
      ki{ std::array<double, 2>{low_ki, 0}, std::array<double, 2>{0, low_ki} },
      kd{ std::array<double, 2>{low_kd, 0}, std::array<double, 2>{0, low_kd} },
      r(D_MOTOR_PULLEY), R1(D_FINGER_PULLEY_1), R2(D_FINGER_PULLEY_2),
      trans_mat{{
            {{r/R1, -r/R1, 0.0, 0.0}},
            {{r/R2, r/R2, -r/R2, r/R2}}
      }}{}

/**
 * @brief Computes motor torques using PID control.
 *
 * This method calculates the error between desired and current joint states, updates the accumulated error,
 * and computes the derivative of the error from the motor velocities. Then, it applies a PID algorithm
 * to compute the joint torques and converts these into motor torques using a transformation matrix.
 *
 * @param joint_states Vector of current joint states.
 * @param joint_states_desired Vector of desired joint states.
 * @param joint_error_sum Vector containing the accumulated joint error.
 * @param motor_velocities Vector of current motor velocities.
 * @return A std::pair containing:
 *         - A vector of computed motor torques.
 *         - The updated joint error sum vector.
 */
std::pair<std::vector<double>, std::vector<double>> Controller::torque_control(
    std::vector<double> joint_states, 
    std::vector<double> joint_states_desired, 
    std::vector<double> joint_error_sum, 
    std::vector<double> motor_velocities)
{
    // Calculate the error between desired and current joint states.
    std::vector<double> joint_error = {
        joint_states_desired[0] - joint_states[0],
        joint_states_desired[1] - joint_states[1]
    };

    // Adjust PID gains based on the joint error.
    Controller::check_error(joint_error);

    // Update the accumulated joint error.
    joint_error_sum = {
        joint_error_sum[0] + joint_error[0],
        joint_error_sum[1] + joint_error[1]
    };

    // Calculate the derivative of the error based on motor velocities using the transformation matrix.
    std::vector<double> error_change = {0.0, 0.0};
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 4; j++) {
            error_change[i] = trans_mat[i][j] * motor_velocities[j];
        }
    }
    
    // Apply PID control to compute joint torques.
    std::vector<double> joint_torques = {
        (kp[0][0] * joint_error[0] + kd[0][0] * error_change[0] + ki[0][0] * joint_error_sum[0]) / 2,
        (kp[1][1] * joint_error[1] + kd[1][1] * error_change[1] + ki[1][1] * joint_error_sum[1]) / 4
    };

    // Convert joint torques to motor torques using the transformation matrix.
    std::vector<double> motor_torques(4, 0.0);
    motor_torques[0] = r / R1 * joint_torques[0] + r / R2 * joint_torques[1];
    motor_torques[1] = -r / R1 * joint_torques[0] + r / R2 * joint_torques[1];
    motor_torques[2] = -r / R2 * joint_torques[1];
    motor_torques[3] = r / R2 * joint_torques[1];
    
    return {motor_torques, joint_error_sum};
}

/**
 * @brief Adjusts the PID gains based on the current joint error.
 *
 * The function selects between low and high PID gains depending on whether each joint error is positive or negative.
 *
 * @param joint_error Vector containing the error for each joint.
 */
void Controller::check_error(std::vector<double> joint_error) {
    if (joint_error[0] <= 0.0 && joint_error[1] <= 0.0) {
        kp = {{
            {low_kp, 0},
            {0, low_kp}
        }};
        ki = {{
            {low_ki, 0},
            {0, low_ki}
        }};
        kd = {{
            {low_kd, 0},
            {0, low_kd}
        }};
    } else if (joint_error[0] <= 0.0 && joint_error[1] > 0.0) {
        kp = {{
            {low_kp, 0},
            {0, high_kp}
        }};
        ki = {{
            {low_ki, 0},
            {0, high_ki}
        }};
        kd = {{
            {low_kd, 0},
            {0, high_kd}
        }};
    } else if (joint_error[0] > 0.0 && joint_error[1] <= 0.0) {
        kp = {{
            {high_kp, 0},
            {0, low_kp}
        }};
        ki = {{
            {high_ki, 0},
            {0, low_ki}
        }};
        kd = {{
            {high_kd, 0},
            {0, low_kd}
        }};
    } else {
        kp = {{
            {high_kp, 0},
            {0, high_kp}
        }};
        ki = {{
            {high_ki, 0},
            {0, high_ki}
        }};
        kd = {{
            {high_kd, 0},
            {0, high_kd}
        }};
    }
}
