#include <controller/controller.h>

Controller::Controller()
    :   low_kp(0.15), high_kp(1.15), low_kd(0.15), high_kd(1.15), low_ki(0.15), high_ki(1.15),
        kp{ std::array<double, 2>{low_kp, 0}, std::array<double, 2>{0, low_kp} },
        ki{ std::array<double, 2>{low_ki, 0}, std::array<double, 2>{0, low_ki} },
        kd{ std::array<double, 2>{low_kd, 0}, std::array<double, 2>{0, low_kd} },
        r(D_MOTOR_PULLEY), R1(D_FINGER_PULLEY_1), R2(D_FINGER_PULLEY_2),
        trans_mat{{
            {{r/R1, -r/R1, 0.0, 0.0}}, 
            {{-r/R2, r/R2, r/R2, -r/R2}}
        }}{}

std::pair<std::vector<double>, std::vector<double>> Controller::torque_control(std::vector<double> joint_states, 
                                    std::vector<double> joint_states_desired,std::vector<double> joint_error_sum, 
                                    std::vector<double> motor_velocities){

    // Calculate joint error, joint error sum
    std::vector<double> joint_error = {{joint_states_desired[0] - joint_states[0]}, {joint_states_desired[1] - joint_states[1]}};
    Controller::check_error(joint_error); // Check error and update gains
    joint_error_sum = {{joint_error_sum[0] + joint_error[0]}, {joint_error_sum[1] + joint_error[1]}};
    std::vector<double> error_change = {0.0, 0.0};
    for(int i = 0; i < 2; i++){
        for(int j = 0; j < 4; j++){
            error_change[i] = trans_mat[i][j] * motor_velocities[j];
        } 
    }
    
    // PID Control of joint torques
    std::vector<double> joint_torques = {
        {kp[0][0] * joint_error[0] + kd[0][0] * error_change[0] + ki[0][0] * joint_error_sum[0]},
        {kp[1][1] * joint_error[1] + kd[1][1] * error_change[1] + ki[1][1] * joint_error_sum[1]}
    };
    // Convert motor torques
    std::vector<double> motor_torques(4, 0.0);
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 2; ++j) {
            motor_torques[i] += trans_mat[j][i] * joint_torques[j];
        }
    }
    return {motor_torques, joint_error_sum};
}

void Controller::check_error(std::vector<double> joint_error){
    if (std::abs(joint_error[0]) <= 0.0 && std::abs(joint_error[1]) <= 0.0) {
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
    } else if (std::abs(joint_error[0]) <= 0.0 && std::abs(joint_error[1]) > 0.0) {
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
    } else if (std::abs(joint_error[0]) > 0.0 && std::abs(joint_error[1]) <= 0.0) {
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
    