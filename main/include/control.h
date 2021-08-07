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
    void get_control_val(float ctl_data[3]);

   private:
    void calculate_pid(float ref_data[3], float cur_data[3], float pre_data[3], float out_data[3], float Kp[3], float Ki[3], float Kd[3]);
    void calculate_id_term();
    void limit_val(float &val, float min, float max);
    void low_pass_filter(float cutoff_freq,float pre_filtered_data[3],  float cur_data[3], float filtered_data[3]);

    // Gain for angles: roll, pitch, yaw
    float Kp_ang_[3] = { 0.5f,  0.5f,  0.5f};
    float Ki_ang_[3] = { 0.15f,  0.35f,  0.15f};
    float Kd_ang_[3] = { 0.0f,  0.0f,  0.0f};

    // Gain for angular velocities: roll, pitch, yaw
    float Kp_angvel_[3] = { 0.4f,  0.4f,  0.4f};
    float Ki_angvel_[3] = { 0.05f,  0.05f,  0.05f};
    float Kd_angvel_[3] = { 0.01f,  0.01f,  0.01f};

    // Previous values
    float pre_ang_err_data_[3]    = {0.0f, 0.0f, 0.0f};
    float pre_angvel_err_data_[3] = {0.0f, 0.0f, 0.0f};
    
    // Previous filtered value
    float pre_filtered_dterm_data_[3]   = {0.0f, 0.0f, 0.0f};
    float pre_filtered_control_data_[3] = {0.0f, 0.0f, 0.0f};

    // Output data
    float ang_ref_data_[3]    = {0.0f, 0.0f, 0.0f};
    float angvel_ctl_data_[3] = {0.0f, 0.0f, 0.0f};
};

#endif  // #ifndef Control_h