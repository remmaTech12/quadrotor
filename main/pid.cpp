#include "pid.h"

PID::PID() {}

void PID::setup() {
    sampling_time_ms = SAMPLING_TIME_MS;
}

void PID::get_pid(float data[3]) {
    data[0] = pid_rpy[0];
    data[1] = pid_rpy[1];
    data[2] = pid_rpy[2];
}

void PID::calculate_pid() { 
    calculate_rpy();
    for (int i=0; i<3; i++) {
        pid_rpy[i] = Kp[i]*rpy_p[i] + Ki[i]*rpy_i[i] + Kd[i]*rpy_d[i];
    }
}

void PID::set_rpy(float *data) {
    for (int i=0; i<3; i++) {
        rpy_p[i] = data[i];
    }
}

void PID::calculate_rpy() {
    for (int i=0; i<3; i++) {
        rpy_d[i] = (rpy_p[i] - rpy_pre[i]) / ((float)sampling_time_ms/1000);
        rpy_i[i] += rpy_p[i];
        rpy_pre[i] = rpy_p[i];
    }
}