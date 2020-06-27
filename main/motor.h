#ifndef MOTOR_h
#define MOTOR_h
#include "Arduino.h"
#include "def_system.h"

class Motor {
   public:
    Motor();

    void setup();
    void control();
    void test_led_pid(int cmd_data[4], float pid_rpy[3]);
    void test_led_cmd(int cmd_data[4]);

   private:
    float m_pid_rpy[3] = {0.0f, 0.0f, 0.0f};
    int m_cmd_data[4] = {0, 0, 0, 0};
    int m_test_pid[4] = {0, 0, 0, 0};

    void format_pid_data();
    void format_cmd_data();
};

#endif  // #ifndef Motor_h