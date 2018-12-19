#include <cmath>
#ifndef POINT_H
#define POINT_H
#endif

class PID{
public:
    PID();
    float get_control(float d);

private:
    float error;
    float pre_error;
    float error_sum;
    float error_diff;
    float Kp;
    float Ki;
    float Kd;
};
