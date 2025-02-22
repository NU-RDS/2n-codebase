#ifndef PID_H
#define PID_H

#include <vector>

class PID
{
    public:
        PID(unsigned int i_size, float kp, float ki, float kd);
        float getInput(float state, float new_state);

    private:
        void _updateState(float data);

        float _current_state;

        unsigned int _i_size;
        std::vector<float> _i_list;
        float _getISum(float ref_state);

        float _kp;
        float _ki;
        float _kd;
};

#endif