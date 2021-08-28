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

/*
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
*/

void Motor::limit_command(int &cmd, int min, int max) {
    if (cmd > max) { cmd = max; }
    if (cmd < min) { cmd = min; }
}

void Motor::stop_motor() {
    for (int i=0; i<4; i++) {
        ledcWrite(i, 0);
    }
}

/*
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
*/

void Motor::control(int cmd_data[4], float ctl_data[3], Arm &arm) {
    if (arm.get_arm_status() == false) { 
        stop_motor();
        return;
    }

    Serial.print("roll_ctrl: ");
    Serial.print(ctl_data[0]);
    Serial.print(", pitch_ctrl: ");
    Serial.print(ctl_data[1]);
    Serial.print(", yaw_ctrl: ");
    Serial.println(ctl_data[2]);

    int motor_data[4] = {0, 0, 0, 0};
    int cmd_thrust = 0;
    double thrust_scale = 0.65;

    cmd_thrust = calculate_thrust(thrust_scale, cmd_data);
    calculate_motor_control(ctl_data, motor_data);

    for (int i = 0; i < 4; i++) {
        double ctl_limit = LIMIT_MOTOR * (1.0f - thrust_scale);
        limit_command(motor_data[i], 0, ctl_limit);
        motor_data[i] += cmd_thrust;
        limit_command(motor_data[i], 0, LIMIT_MOTOR);
    };

    for (int i=0; i<4; i++) {
        ledcWrite(i, motor_data[i]);
    }

#ifdef DEBUG_MOTOR_COMMAND
    Serial.print("MOTOR COMMAND: ");
    debug_print(motor_data);
#endif
}

int Motor::calculate_thrust(double thrust_scale, int cmd_data[4]) {
    double kth = 40;
    if (cmd_data[0] < kth) {
        cmd_data[0] *= 3;
    } else {
        cmd_data[0] = ((LIMIT_MOTOR - 3*kth) / (LIMIT_MOTOR - kth)) * (cmd_data[0] - kth) + 3*kth;
    }
    int cmd_thrust = cmd_data[0]*thrust_scale;
    limit_command(cmd_thrust, 0, LIMIT_MOTOR*thrust_scale);

    return cmd_thrust;
}

void Motor::calculate_motor_control(float ctl_data[3], int motor_data[4]) {
    double offset_motor[4] = {22.0f, 0.0f, 23.0f, 32.0f};
    //double offset_motor[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    motor_data[0] = + ctl_data[0] - ctl_data[1] - ctl_data[2] + offset_motor[0];
    motor_data[1] = + ctl_data[0] + ctl_data[1] + ctl_data[2] + offset_motor[1];
    motor_data[2] = - ctl_data[0] + ctl_data[1] - ctl_data[2] + offset_motor[2];
    motor_data[3] = - ctl_data[0] - ctl_data[1] + ctl_data[2] + offset_motor[3];
}