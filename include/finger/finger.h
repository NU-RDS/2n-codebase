#ifndef FINGER_H
#define FINGER_H

#include <stdint.h>

#define NUM_JOINTS 2
#define NUM_MOTORS 4
#define D_MOTOR_PULLEY 11.9998f // 12 mm
#define D_FINGER_PULLEY_1 29.0f // 29 mm
#define D_FINGER_PULLEY_2 35.0f // 35 mm

class Finger{
    public:
        Finger();
        float* getJointStates(const float motor_states[NUM_MOTORS]);
        float* getMotorCommands(const float joint_inputs[NUM_JOINTS]);

    private:
        float _joint_states[NUM_JOINTS];
        float _motor_commands[NUM_MOTORS];
};

#endif
