#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <iostream>
#include <vector>
#include <array>
#include <chrono>
#include <cmath>
//------------ Han correct this if needed ------------//
#include "MotorSerialInterface.h" // Ensure this file is correctly located and included
//------------ Han correct this if needed ------------//

class Controller {
public:
    Controller();

    std::pair<std::vector<double>, std::vector<double>> go_home(bool tensioned);
    bool tension();
    
    std::vector<double> torque_ctrl(std::vector<double> js_0, std::vector<double> ms_0,
                                    std::vector<double> js, std::vector<double> ms,
                                    std::vector<double> js_d, double prev_time,
                                    std::vector<double> js_e_sum = {0.0, 0.0});
    void check_error(std::vector<double> js_e);

private:
    MotorSerialInterface MSI;
    
    double low_kp, high_kp, low_ki, high_ki, low_kd, high_kd;
    std::array<std::array<double, 2>, 2> kp, ki, kd;
    double r, R1, R2;
    std::array<std::array<double, 4>, 2> trans_mat;
    bool tensioned;
};

#endif
