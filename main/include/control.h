#ifndef Control_h
#define Control_h
#include "def_system.h"
#include "Arduino.h"

class Control {
   public:
    Control();

    void setup();
    void get_pid(float data[3]);
    void calculate_pid_ang(int cmd_data[4], float ang_data[3]);
    void calculate_pid_angvel(float angvel_data[3]);
    void calculate_and_remove_bias(bool is_armed);
    void get_control_val(float ctl_data[3]);

   private:
    void calculate_pid(float ref_data[3], float cur_data[3], float err_data_i[3], float pre_data[3], float pre_filtered_dterm_data[3], float out_data[3], float Kp[3], float Ki[3], float Kd[3]);
    void calculate_id_term();
    void limit_val(float &val, float min, float max);
    void low_pass_filter(float cutoff_freq,float pre_filtered_data[3],  float cur_data[3], float filtered_data[3]);

    // Gain for angles: roll, pitch, yaw
    float Kp_ang_[3] = { 1.5f,  1.5f,  1.0f};
    float Ki_ang_[3] = { 0.0f,  0.0f,  0.0f};
    float Kd_ang_[3] = { 0.0f,  0.0f,  0.0f};

    // Gain for angular velocities: roll, pitch, yaw
    float Kp_angvel_[3] = { 0.5f,  0.5f,  0.5f};
    float Ki_angvel_[3] = { 0.0f,  0.0f,  0.0f};
    float Kd_angvel_[3] = { 0.0f,  0.0f,  0.0f};

    // I values
    float err_ang_data_i_[3]    = {0.0f, 0.0f, 0.0f};
    float err_angvel_data_i_[3] = {0.0f, 0.0f, 0.0f};

    // Previous values
    float pre_ang_data_[3]    = {0.0f, 0.0f, 0.0f};
    float pre_angvel_data_[3] = {0.0f, 0.0f, 0.0f};

    float pre_filtered_ang_dterm_data_[3]    = {0.0f, 0.0f, 0.0f};
    float pre_filtered_angvel_dterm_data_[3] = {0.0f, 0.0f, 0.0f};
    
    // Previous filtered value
    float pre_filtered_dterm_data_[3]   = {0.0f, 0.0f, 0.0f};
    float pre_filtered_control_data_[3] = {0.0f, 0.0f, 0.0f};

    // Output data
    float ang_ref_data_[3]    = {0.0f, 0.0f, 0.0f};
    float angvel_ctl_data_[3] = {0.0f, 0.0f, 0.0f};

    // Trim
    float ctl_bias_sum_data_[3] = {0.0f, 0.0f, 0.0f};
    float ctl_bias_ave_data_[3] = {0.0f, 0.0f, 0.0f};
    int cnt = 0;
};

#endif  // #ifndef Control_h