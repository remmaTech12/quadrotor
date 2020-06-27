#ifndef PID_h
#define PID_h
#include "def_system.h"
#include "Arduino.h"

class PID {
   public:
    PID();

    void setup();
    void test_led();
    void set_rpy(float data[3]);
    void get_pid(float data[3]);
    void calculate_pid();

   private:
    int sampling_time_ms = 0;

    // roll, pitch, yaw
    float Kp[3] = {50.0f, 50.0f, 30.0f};
    float Ki[3] = { 0.0f,  0.0f,  0.0f};
    float Kd[3] = { 1.0f,  1.0f,  1.0f};

    float rpy_p[3] = {0.0f, 0.0f, 0.0f};
    float rpy_i[3] = {0.0f, 0.0f, 0.0f};
    float rpy_d[3] = {0.0f, 0.0f, 0.0f};
    float rpy_pre[3] = {0.0f, 0.0f, 0.0f};

    float pid_rpy[3] = {0.0f, 0.0f, 0.0f};

    void calculate_rpy();
};

#endif  // #ifndef PID_h