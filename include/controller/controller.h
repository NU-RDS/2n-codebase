#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>
#include "finger/finger.h"
#include <iostream>
#include <vector>
#include <array>
#include <chrono>
#include <math.h>

class Controller{
    public:
        Controller();
        std::pair<std::vector<double>, std::vector<double>> torque_control(std::vector<double> joint_states, 
                                           std::vector<double> joint_states_desired, std::vector<double> joint_error_sum, 
                                           std::vector<double> motor_velocities);
        void check_error(std::vector<double> joint_error);
    
    private:
        Finger finger;
        double low_kp, high_kp, low_kd, high_kd, low_ki, high_ki;
        double r, R1, R2; 
        std::array<std::array<double, 2>,2> kp, kd, ki;
        std::array<std::array<double, 4>,2> trans_mat;
};

#endif