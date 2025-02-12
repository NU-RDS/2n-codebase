#include <cmath>
#include <cstring>
#include "motor_protocols/r1806.h"
#include "motor_protocols/utils.h"

R1806::R1806() {
    _gear_reduction_ratio = GEAR_REDUCION_RATIO;
}

uint32_t R1806::extract_node_id(uint32_t can_id) {
    return (can_id >> 5) & 0x3F;
}

uint32_t R1806::extract_command_id(uint32_t can_id) {
    return can_id & 0x1F;
}

CAN_message_t R1806::encodeRequestedStateCommand(uint32_t node_id, uint32_t requested_state){
    CAN_message_t msg;
    msg.id = ((node_id & 0x3F) << 5) | (0x07 & 0x1F);

    requested_state = ODrive.Axis.AxisState.CLOSED_LOOP_CONTROL;
    msg.buf[0] = requested_state & 0xFF;
    msg.buf[1] = (requested_state >> 8) & 0xFF;
    msg.buf[2] = (requested_state >> 16) & 0xFF;
    msg.buf[3] = (requested_state >> 24) & 0xFF;
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = 0;
    msg.buf[7] = 0;
    return msg;
}

CAN_message_t R1806::encodeTorqueCommand(uint32_t node_id, float torque){
    CAN_message_t msg;
    msg.id = ((node_id & 0x3F) << 5) | (0x0E & 0x1F);

    float value = torque / _gear_reduction_ratio;
    uint32_t torque_int;
    std::memcpy(&torque_int, &value, sizeof(torque_int));

    msg.buf[0] = torque_int & 0xFF;
    msg.buf[1] = (torque_int >> 8) & 0xFF;
    msg.buf[2] = (torque_int >> 16) & 0xFF;
    msg.buf[3] = (torque_int >> 24) & 0xFF;
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = 0;
    msg.buf[7] = 0;
    return msg;
}

float R1806::decodePositionFeedback(CAN_message_t frame) {
    float num_rounds = *( (const float *) frame.buf);
    float position = num_rounds * M_PI * 2 / _gear_reduction_ratio;
    return position;
}

float R1806::decodeVelocityFeedback(CAN_message_t frame) {
    float num_rounds_per_sec = *( (const float *) (frame.buf + 4) );
    float velocity = num_rounds_per_sec * M_PI * 2 / _gear_reduction_ratio;
    return velocity;
}