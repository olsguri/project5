#include <project5/pid.h>

PID::PID() {
    //Kp = 0.8;
    //Ki = 0.000;
    //Kd = 3.000; 4m/s
    //Kp = 0.5;
    //Ki = 0.000;
    //Kd = 3.000; //5m/s
    //Kp = 0.7;
    //Ki = 0.000;
    //Kd = 2.000; //5m/s
    Kp = 0.4;
    Ki = 0.000;
    Kd = 2.43; //7m/s
    //Kp = 0.35;
    //Ki = 0.000;
    //Kd = 2.10; //10m/s
    error = 0;
    pre_error = 0;
    error_diff = 0;
    error_sum = 0;
}

float PID::get_control(float d) {
    error = d;
    error_sum += error;
    error_diff = (error - pre_error);
    float ctrl = Kp*error + Ki*error_sum + Kd*error_diff;
    pre_error = error;

    return ctrl;
}
