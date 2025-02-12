#ifndef MOTOR_PROTOCOLS_UTILS_H
#define MOTOR_PROTOCOLS_UTILS_H

#include <stdint.h>

struct{
    struct{
        struct{
            uint32_t UNDEFINED = 0;
            uint32_t IDLE = 1;
            uint32_t STARTUP_SEQUENCE = 2;
            uint32_t FULL_CALIBRATION_SEQUENCE = 3;
            uint32_t MOTOR_CALIBRATION = 4;
            uint32_t ENCODER_INDEX_SEARCH = 6;
            uint32_t ENCODER_OFFSET_CALIBRATION = 7;
            uint32_t CLOSED_LOOP_CONTROL = 8;
            uint32_t LOCKIN_SPIN = 9;
            uint32_t ENCODER_DIR_FIND = 10;
            uint32_t HOMING = 11;
            uint32_t ENCODER_HALL_POLARITY_CALIBRATION = 12;
            uint32_t ENCODER_HALL_PHASE_CALIBRATION = 13;
            uint32_t ANTICOGGING_CALIBRATION = 14;
        }AxisState;
    }Axis;
}ODrive;

#endif
