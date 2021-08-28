#ifndef MOTOR_h
#define MOTOR_h
#include "arm.h"
#include "Arduino.h"
#include "def_system.h"

class Motor {
   public:
    Motor();

    void setup();
    //void test_control(int motor_val);
    //void test_count();
    void control(int cmd_data[4], float pid_rpy[3], Arm &arm);
    void stop_motor();

   private:
    float m_pid_data[3] = {0.0f, 0.0f, 0.0f};
    int m_recv_cmd[4] = {0, 0, 0, 0};
    int m_pid_cmd[4]  = {0, 0, 0, 0};
    int tcount = 0;
    int pre_button = LOW;

    //void format_cmd_data(int cmd_data[4]);
    //void format_pid_data(float pid_rpy[3]);
    void limit_command(int &cmd, int min, int max);
    //void debug_print(int data[4]);
    int calculate_thrust(double thrust_scale, int cmd_data[4]);
    void calculate_motor_control(float ctl_data[3], int motor_data[4]);
};

#endif  // #ifndef Motor_h