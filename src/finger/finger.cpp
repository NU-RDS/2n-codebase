#include <string.h>
#include "finger/finger.h"

Finger::Finger() {
}

float* Finger::getJointStates(const float motor_states[NUM_MOTORS]) {
    float joint1_position_1 = motor_states[0] * D_MOTOR_PULLEY / D_FINGER_PULLEY_1;
    float joint1_position_2 = - motor_states[2] * D_MOTOR_PULLEY / D_FINGER_PULLEY_1;
    _joint_states[0] = (joint1_position_1 + joint1_position_2) / 2.0f;

    _joint_states[1] = 0.0;

    return _joint_states;
}