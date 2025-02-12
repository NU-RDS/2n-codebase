#ifndef R1806_H
#define R1806_H
// Motor protocols for R1806 motor and ODrive Pro.
// R1806 motor is fabricated by Repeat Robotics.
// Motor Details: https://repeat-robotics.com/buy/repeat-compact-1806
// ODrive CAN Protocol Details: https://docs.odriverobotics.com/v/latest/manual/can-protocol.html

#include <FlexCAN_T4.h>

#define GEAR_REDUCION_RATIO 22.6f

class R1806
{
    public:
        R1806();
        uint32_t extract_node_id(uint32_t can_id);
        uint32_t extract_command_id(uint32_t can_id);

        CAN_message_t encodeRequestedStateCommand(uint32_t node_id, uint32_t requested_state);
        CAN_message_t encodeTorqueCommand(uint32_t node_id, float torque);

        float decodePositionFeedback(CAN_message_t frame);
        float decodeVelocityFeedback(CAN_message_t frame);

    private:
        float _gear_reduction_ratio;
};

#endif
