#include "../include/control.h"

Control::Control() {}

void Control::setup() {}

void Control::calculate_pid_ang(int cmd_data[3], float ang_data[3]) {
    float ref_data[3];
    float out_data[3];
    for (int i=0; i<3; i++) {
        ref_data[i] = (float) cmd_data[i];
    }

    calculate_pid(ref_data, ang_data, pre_ang_err_data_, out_data, Kp_ang_, Ki_ang_, Kd_ang_);

    for (int i=0; i<3; i++) {
        limit_val(out_data[i], -2.0f*M_PI, 2.0f*M_PI);
        ang_ref_data_[i] = out_data[i];
    }
}

void Control::calculate_pid_angvel(float angvel_data[3]) {
    float out_data[3];
    calculate_pid(ang_ref_data_, angvel_data, pre_angvel_err_data_, out_data, Kp_angvel_, Ki_angvel_, Kd_angvel_);

    for (int i=0; i<3; i++) {
        limit_val(out_data[i], -5.0f*M_PI, 5.0f*M_PI);
        angvel_ctl_data_[i] = out_data[i];
    }
}

void Control::calculate_pid(float ref_data[3], float cur_data[3], float pre_err_data[3], float out_data[3], float Kp[3], float Ki[3], float Kd[3]) {
    float err_data_p[3];
    float err_data_i[3];
    float err_data_d[3];

    for (int i=0; i<3; i++) {
        err_data_p[i]   = ref_data[i] - cur_data[i];
        err_data_i[i]  += err_data_p[i];
        err_data_d[i]   = (err_data_p[i] - pre_err_data[i]) / ((float)SAMPLING_TIME_MS/1000.0f);

        limit_val(err_data_i[i], -5.0f*M_PI, 5.0f*M_PI);
        pre_err_data[i] = err_data_p[i];
    }

    float filtered_err_data_d[3];
    float cutoff_freq = 1000.0f;
    low_pass_filter(cutoff_freq, err_data_d, filtered_err_data_d);

    for (int i=0; i<3; i++) {
        out_data[i] = Kp[i]*err_data_p[i] + Ki[i]*err_data_i[i] + Kd[i]*filtered_err_data_d[i];
    }
}

void Control::limit_val(float &val, float min, float max) {
    if (val > max) { val = max; }
    if (val < min) { val = min; }
}

void Control::low_pass_filter(float cutoff_freq, float cur_data[3], float filtered_data[3]) {
    float Tsamp = SAMPLING_TIME_MS / 1000.0f;
    float tau   = 1.0f / (2.0f * M_PI * cutoff_freq);
    float kpre  = tau / (Tsamp + tau);
    // reference: https://qiita.com/motorcontrolman/items/39d4abc6c4862817e646
    // cutoff_freq = 1000: kpre = 0.0157

    for (int i=0; i<3; i++) {
        filtered_data[i] = kpre*pre_filtered_data_[i] + (1.0f - kpre)*cur_data[i];
        pre_filtered_data_[i] = filtered_data[i];
    }
}

void Control::get_control_val(float ctl_data[3]) {
    ctl_data[0] = angvel_ctl_data_[0];
    ctl_data[1] = angvel_ctl_data_[1];
    ctl_data[2] = angvel_ctl_data_[2];
}