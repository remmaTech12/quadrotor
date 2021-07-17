#ifndef PID_h
#define PID_h
#include "def_system.h"
#include "Arduino.h"

class PID {
   public:
    PID();

    void setup();
    void get_pid(float data[3]);
    void calculate_pid(float data[3]);

   private:
    // Gain: roll, pitch, yaw
    float Kp[3] = {20.0f, 20.0f, 20.0f};
    float Ki[3] = { 0.25f,  0.2f,  0.2f};
    float Kd[3] = { 15.0f,  15.0f, 15.0f};

    // Measured value: roll, pitch, yaw
    float rpy_p[3]   = {0.0f, 0.0f, 0.0f};
    float rpy_i[3]   = {0.0f, 0.0f, 0.0f};
    float rpy_d[3]   = {0.0f, 0.0f, 0.0f};
    float rpy_pre[3] = {0.0f, 0.0f, 0.0f};

    // Calculated value: roll, pitch, yaw
    float pid_rpy[3] = {0.0f, 0.0f, 0.0f};

    void calculate_id_term();
};

#endif  // #ifndef PID_h