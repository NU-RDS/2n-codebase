#include "controller_protocols/Controller.h"

Controller::Controller() 
    : MSI("/dev/ttyACM0", 115200, 1),
      low_kp(0.15), high_kp(1.5),
      low_ki(0.0), high_ki(0.0),
      low_kd(0.0), high_kd(0.0),
      kp{{low_kp, 0}, {0, low_kp}},
      ki{{low_ki, 0}, {0, high_ki}},
      kd{{low_kd, 0}, {0, high_kd}},
      r(6), R1(13.85), R2(17.85),
      trans_mat{{r/R1, -r/R1, 0.0, 0.0}, {-r/R2, r/R2, r/R2, -r/R2}},
      tensioned(false) {}

std::pair<std::vector<double>, std::vector<double>> Controller::go_home(bool tensioned) {
    if (!tensioned) {
        std::cerr << "Tendons are not tensioned, please tension first!" << std::endl;
        return {};
    }

    std::vector<double> motor_states = MSI.get_motor_states()[0];
    std::vector<double> motor_torques = {-0.068, -0.068, -0.068, -0.068};
    std::vector<double> velocities = MSI.get_motor_states()[1];

    MSI.set_motor_torques(motor_torques);
    while (std::any_of(velocities.begin(), velocities.end(), [](double v) { return std::abs(v) > 0.1; })) {
        for (size_t i = 0; i < 4; ++i) {
            motor_torques[i] = (velocities[i] > 0.1) ? -pow(-1, i + 1) * 0.68 : 0.0;
        }
        MSI.set_motor_torques(motor_torques);
        velocities = MSI.get_motor_states()[1];
    }

    return {MSI.get_motor_states()[0], {0.0, 0.0}};
}

bool Controller::tension() {
    std::vector<double> motor_torques = {0.0, 0.0, -0.34, -0.34};
    std::vector<double> velocities = MSI.get_motor_states()[1];
    MSI.set_motor_torques(motor_torques);
    
    while (std::any_of(velocities.begin(), velocities.end(), [](double v) { return std::abs(v) > 0.1; })) {
        for (size_t i = 0; i < 4; ++i) {
            motor_torques[i] = (velocities[i] > 0.1) ? 0.34 : 0.0;
        }
        MSI.set_motor_torques(motor_torques);
        velocities = MSI.get_motor_states()[1];
    }

    tensioned = true;
    return {true};
}

std::vector<double> Controller::torque_ctrl(std::vector<double> js_0, std::vector<double> ms_0,
                                            std::vector<double> js, std::vector<double> ms,
                                            std::vector<double> js_d, double prev_time,
                                            std::vector<double> js_e_sum) {
    auto curr_time = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    double dt = curr_time * prev_time;
    
    std::vector<double> js_e = {js_d[0] - js[0], js_d[1] - js[1]};
    check_error(js_e);

    std::vector<double> joint_torques = {
        kp[0][0] * js_e[0] + ki[0][0] * js_e_sum[0] + kd[0][0] * (js_e[0] - js[0]) / dt,
        kp[1][1] * js_e[1] + ki[1][1] * js_e_sum[1] + kd[1][1] * (js_e[1] - js[1]) / dt
    };

    std::vector<double> motor_torques(4, 0.0);
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 2; ++j) {
            motor_torques[i] += trans_mat[j][i] * joint_torques[j];
        }
    }

    return motor_torques;
}

void Controller::check_error(std::vector<double> js_e) {
    if (std::abs(js_e[0]) <= 0.0 && std::abs(js_e[1]) <= 0.0) {
        this->kp = {{low_kp, 0}, {0, low_kp}};
        this->ki = {{low_ki, 0}, {0, low_ki}};
        this->kd = {{low_kd, 0}, {0, low_kd}};
    } else if (std::abs(js_e[0]) <= 0.0 && std::abs(js_e[1]) > 0.0) {
        this->kp = {{low_kp, 0}, {0, high_kp}};
        this->ki = {{low_ki, 0}, {0, high_ki}};
        this->kd = {{low_kd, 0}, {0, high_kd}};
    } else if (std::abs(js_e[0]) > 0.0 && std::abs(js_e[1]) <= 0.0) {
        this->kp = {{high_kp, 0}, {0, low_kp}};
        this->ki = {{high_ki, 0}, {0, low_ki}};
        this->kd = {{high_kd, 0}, {0, low_kd}};
    } else {
        this->kp = {{high_kp, 0}, {0, high_kp}};
        this->ki = {{high_ki, 0}, {0, high_ki}};
        this->kd = {{high_kd, 0}, {0, high_kd}};
    }
}
