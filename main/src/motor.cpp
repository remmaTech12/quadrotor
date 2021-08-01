#include "../include/motor.h"

Motor::Motor() {}

void Motor::setup() {
    pinMode(MOTOR_DIR1, OUTPUT);
    pinMode(MOTOR_DIR2, OUTPUT);
    pinMode(MOTOR_DIR3, OUTPUT);
    pinMode(MOTOR_DIR4, OUTPUT);
    digitalWrite(MOTOR_DIR1, HIGH);
    digitalWrite(MOTOR_DIR2, HIGH);
    digitalWrite(MOTOR_DIR3, LOW);
    digitalWrite(MOTOR_DIR4, LOW);

    pinMode(MOTOR_PWM1, OUTPUT);
    pinMode(MOTOR_PWM2, OUTPUT);
    pinMode(MOTOR_PWM3, OUTPUT);
    pinMode(MOTOR_PWM4, OUTPUT);
    ledcSetup(0, 12800, 8);
    ledcAttachPin(MOTOR_PWM1, 0);
    ledcSetup(1, 12800, 8);
    ledcAttachPin(MOTOR_PWM2, 1);
    ledcSetup(2, 12800, 8);
    ledcAttachPin(MOTOR_PWM3, 2);
    ledcSetup(3, 12800, 8);
    ledcAttachPin(MOTOR_PWM4, 3);
    for (int i = 0; i < 4; i++) {
        ledcWrite(i, 0);
    }
}

void Motor::test_control(int motor_val) {
    int test_motor_val = motor_val;
    if (digitalRead(EMERGENCY_SWITCH) == LOW) {
        ledcWrite(0, test_motor_val);
        ledcWrite(1, test_motor_val);
        ledcWrite(2, test_motor_val);
        ledcWrite(3, test_motor_val);
    } else {
        ledcWrite(0, 0);
        ledcWrite(1, 0);
        ledcWrite(2, 0);
        ledcWrite(3, 0);
    }
}

void Motor::test_count() {
    if (pre_button == HIGH && digitalRead(EMERGENCY_SWITCH) == LOW) {
        tcount += 5;
    }
    int test_motor_val = (tcount % 26) * 10;
    ledcWrite(0, test_motor_val);
    ledcWrite(1, test_motor_val);
    ledcWrite(2, test_motor_val);
    ledcWrite(3, test_motor_val);

    pre_button = digitalRead(EMERGENCY_SWITCH);
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
    int control_damper = 3;
    int cmd_thrust = cmd_data[0];
    int cmd_roll   = (cmd_data[3] - 127)/control_damper;
    int cmd_pitch  = (cmd_data[2] - 127)/control_damper;
    int cmd_yaw    = (cmd_data[1] - 127)/control_damper;

    m_recv_cmd[0] = + cmd_roll - cmd_pitch - cmd_yaw + cmd_thrust;
    m_recv_cmd[1] = + cmd_roll + cmd_pitch + cmd_yaw + cmd_thrust;
    m_recv_cmd[2] = - cmd_roll + cmd_pitch - cmd_yaw + cmd_thrust;
    m_recv_cmd[3] = - cmd_roll - cmd_pitch + cmd_yaw + cmd_thrust;

    for (int i = 0; i < 4; i++) {
        limit_command(m_recv_cmd[i], 0, LIMIT_MOTOR);
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
        m_pid_cmd[i] = LIMIT_MOTOR*m_pid_cmd[i]/PID_MAX;
    };
#ifdef DEBUG_PID_COMMAND
    Serial.print("PID COMMAND: ");
    debug_print(m_pid_cmd);
#endif
}

void Motor::control(int cmd_data[4], float ctl_data[3], Arm &arm) {
    if (arm.get_arm_status() == false) { 
        stop_motor();
        return;
    }

    int motor_data[4] = {0, 0, 0, 0};

    int cmd_thrust = cmd_data[0];
    limit_command(cmd_thrust, 0, LIMIT_MOTOR/2.0f);

    //format_cmd_data(cmd_data);
    //format_pid_data(pid_data);

    motor_data[0] = + ctl_data[0] - ctl_data[1] - ctl_data[2] + cmd_thrust;
    motor_data[1] = + ctl_data[0] + ctl_data[1] + ctl_data[2] + cmd_thrust;
    motor_data[2] = - ctl_data[0] + ctl_data[1] - ctl_data[2] + cmd_thrust;
    motor_data[3] = - ctl_data[0] - ctl_data[1] + ctl_data[2] + cmd_thrust;

    for (int i = 0; i < 4; i++) {
        limit_command(m_recv_cmd[i], 0, LIMIT_MOTOR);
    };

    //float pid_ratio = 0.45;

    for (int i=0; i<4; i++) {
        //motor_data[i] = m_pid_cmd[i]*pid_ratio + m_recv_cmd[i]*(1.0f-pid_ratio);
        ledcWrite(i, motor_data[i]);
    }

#ifdef DEBUG_MOTOR_COMMAND
    Serial.print("MOTOR COMMAND: ");
    debug_print(motor_data);
#endif
}