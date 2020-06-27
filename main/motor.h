#ifndef MOTOR_h
#define MOTOR_h
#include "arm.h"
#include "Arduino.h"
#include "def_system.h"

class Motor {
   public:
    Motor();

    void setup(Arm &arm);
    void control();
    void test_led(int cmd_data[4], float pid_rpy[3], Arm &arm);
    void stop_motor();

   private:
    Arm m_arm;
    float m_pid_rpy[3] = {0.0f, 0.0f, 0.0f};
    int m_cmd_data[4] = {0, 0, 0, 0};
    int m_test_pid[4] = {0, 0, 0, 0};

    void format_pid_data();
    void format_cmd_data();
};

#endif  // #ifndef Motor_h