#include "pid/pid.h"
#include <iostream>

PID::PID(unsigned int i_size, float kp, float ki, float kd)
{
    _i_size = i_size;
    for(unsigned int i=0; i<_i_size; i++)
    {
        _i_list.push_back(0.0f);
    }

    _current_state = 0.0f;

    _kp = kp;
    _ki = ki;
    _kd = kd;
}

float PID::getInput(float ref_state, float new_state)
{
    float input = (ref_state - _current_state) * _kp + this->_getISum(ref_state) * _ki - (_current_state - _i_list[_i_size - 2]) * _kd;
    // std::cout<<"ref_state: "<<ref_state<<", current_state: "<<_current_state<<", i_sum: "<<this->_getISum(ref_state)<<", input: "<<input<<std::endl;
    _updateState(new_state);
    return input;
}

void PID::_updateState(float data)
{
    _current_state = data;

    for(int i=0; i<_i_size-1; i++)
    {
        _i_list[i] = _i_list[i+1];
    }
    _i_list[_i_size-1] = data;
}

float PID::_getISum(float ref_state)
{
    float sum = 0.0f;
    for(int i=0; i<_i_size; i++)
        sum += ref_state - _i_list[i];
    return sum;
}