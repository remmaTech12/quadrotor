#include "motor.h"

Motor::Motor() {}

void Motor::setup() {
    //pinMode(MOTOR_M1, OUTPUT);
    //pinMode(MOTOR_E1, OUTPUT);
    //ledcSetup(5, 12800, 8);
    //ledcAttachPin(MOTOR_E1, 5);

    pinMode(LED_PIN1, OUTPUT);
    pinMode(LED_PIN2, OUTPUT);
    pinMode(LED_PIN3, OUTPUT);
    pinMode(LED_PIN4, OUTPUT);
    ledcSetup(0, 12800, 8);
    ledcAttachPin(LED_PIN1, 0);
    ledcSetup(1, 12800, 8);
    ledcAttachPin(LED_PIN2, 1);
    ledcSetup(2, 12800, 8);
    ledcAttachPin(LED_PIN3, 2);
    ledcSetup(3, 12800, 8);
    ledcAttachPin(LED_PIN4, 3);
}

void Motor::control() {
    digitalWrite(MOTOR_M1, LOW);
    ledcWrite(5, 255);
}

void Motor::limit_command(int &cmd, int min, int max) {
    if (cmd > max) { cmd = max; }
    if (cmd < min) { cmd = min; }
}

void Motor::stop_motor() {
    for (int i=0; i<4; i++) {
        ledcWrite(i, 0);
    }
}

void Motor::debug_print(int data[4]) {
    Serial.print("thrust: ");
    Serial.print(data[0]);
    Serial.print(", roll: ");
    Serial.print(data[3]);
    Serial.print(", pitch: ");
    Serial.print(data[2]);
    Serial.print(", yaw: ");
    Serial.println(data[1]);
}

void Motor::format_cmd_data(int cmd_data[4]) {
    int cmd_thrust = cmd_data[0];
    int cmd_roll   = cmd_data[3] - 127;
    int cmd_pitch  = cmd_data[2] - 127;
    int cmd_yaw    = cmd_data[1] - 127;

    m_recv_cmd[0] = + cmd_roll - cmd_pitch - cmd_yaw + cmd_thrust;
    m_recv_cmd[1] = + cmd_roll + cmd_pitch + cmd_yaw + cmd_thrust;
    m_recv_cmd[2] = - cmd_roll + cmd_pitch - cmd_yaw + cmd_thrust;
    m_recv_cmd[3] = - cmd_roll - cmd_pitch + cmd_yaw + cmd_thrust;

    for (int i = 0; i < 4; i++) {
        limit_command(m_recv_cmd[i], 0, 255);
    };
#ifdef DEBUG_RECV_COMMAND
    Serial.print("RECEIVE COMMAND: ");
    debug_print(m_recv_cmd);
#endif
}

void Motor::format_pid_data(float pid_data[3]) {
    int thrust = 0;
    m_pid_cmd[0] = - pid_data[0] + pid_data[1] + pid_data[2] + thrust;
    m_pid_cmd[1] = - pid_data[0] - pid_data[1] - pid_data[2] + thrust;
    m_pid_cmd[2] = + pid_data[0] - pid_data[1] + pid_data[2] + thrust;
    m_pid_cmd[3] = + pid_data[0] + pid_data[1] - pid_data[2] + thrust;

    for (int i=0; i<4; i++) {
        limit_command(m_pid_cmd[i], 0, PID_MAX);
        m_pid_cmd[i] = 255*m_pid_cmd[i]/PID_MAX;
    };
#ifdef DEBUG_PID_COMMAND
    Serial.print("PID COMMAND: ");
    debug_print(m_pid_cmd);
#endif
}

void Motor::test_led(int cmd_data[4], float pid_data[3], Arm &arm) {
    if (arm.get_arm_status() == false) { return; }

    format_cmd_data(cmd_data);
    format_pid_data(pid_data);

    float pid_ratio = 0.5;
    int motor_data[4] = {0, 0, 0, 0};

    for (int i=0; i<4; i++) {
        motor_data[i] = m_pid_cmd[i]*pid_ratio + m_recv_cmd[i]*(1.0f-pid_ratio);
        ledcWrite(i, motor_data[i]);
    }

#ifdef DEBUG_MOTOR_COMMAND
    Serial.print("MOTOR COMMAND: ");
    debug_print(motor_data);
#endif
}