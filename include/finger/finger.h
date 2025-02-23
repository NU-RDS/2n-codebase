#ifndef FINGER_H
#define FINGER_H

#include <stdint.h>

#define NUM_JOINTS 2
#define NUM_MOTORS 4
#define D_MOTOR_PULLEY 11.9998f // mm
#define D_FINGER_PULLEY_1 35.7186f // mm
#define D_FINGER_PULLEY_2 21.7176f // mm

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
