#include "../include/pid.h"

PID::PID() {}

void PID::setup() {}

void PID::get_pid(float data[3]) {
    data[0] = pid_rpy[0];
    data[1] = pid_rpy[1];
    data[2] = pid_rpy[2];
}

void PID::calculate_pid(float data[3]) {
    for (int i=0; i<3; i++) {
        rpy_p[i] = data[i];
    }

    calculate_id_term();
    for (int i=0; i<3; i++) {
        pid_rpy[i] = Kp[i]*rpy_p[i] + Ki[i]*rpy_i[i] + Kd[i]*rpy_d[i];
    }
}

void PID::calculate_id_term() {
    for (int i=0; i<3; i++) {
        rpy_i[i]  += rpy_p[i];
        rpy_d[i]   = (rpy_p[i] - rpy_pre[i]) / ((float)SAMPLING_TIME_MS/1000);
        rpy_pre[i] = rpy_p[i];
    }
}