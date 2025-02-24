#include <string.h>
#include "finger/finger.h"

Finger::Finger() {
}

float* Finger::getJointStates(const float motor_states[NUM_MOTORS]) {
    float joint1_position_1 = motor_states[0] * D_MOTOR_PULLEY / D_FINGER_PULLEY_1;
    float joint1_position_2 = motor_states[1] * D_MOTOR_PULLEY / D_FINGER_PULLEY_1;
    float joint2_position_1 = motor_states[0] * D_MOTOR_PULLEY / D_FINGER_PULLEY_2;
    float joint2_position_2 = motor_states[1] * D_MOTOR_PULLEY / D_FINGER_PULLEY_2;
    float joint2_position_3 = motor_states[2] * D_MOTOR_PULLEY / D_FINGER_PULLEY_2;
    float joint2_position_4 = motor_states[3] * D_MOTOR_PULLEY / D_FINGER_PULLEY_2;
    _joint_states[0] = joint1_position_1 - joint1_position_2;
    _joint_states[1] = -joint2_position_1 + joint2_position_2 + joint2_position_3 - joint2_position_4;

    return _joint_states;
}

float* Finger::getMotorCommands(const float joint_inputs[NUM_JOINTS]) {
    _motor_commands[0] = joint_inputs[0]/2.0f;
    _motor_commands[1] = 0.0;
    _motor_commands[2] = -joint_inputs[0]/2.0f;
    _motor_commands[3] = 0.0;

    return _motor_commands;
}