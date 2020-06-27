#include "motor.h"

Motor::Motor() {}

void Motor::setup(Arm &arm) {
    m_arm = arm;
    //pinMode(MOTOR_M1, OUTPUT);
    //pinMode(MOTOR_E1, OUTPUT);
    ledcSetup(5, 12800, 8);
    ledcAttachPin(MOTOR_E1, 5);

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

void Motor::format_pid_data() {
    //int thrust = PID_MAX/2;
    int thrust = 0;
    m_test_pid[0] = - m_pid_rpy[0] + m_pid_rpy[1] + m_pid_rpy[2] + thrust;
    m_test_pid[1] = - m_pid_rpy[0] - m_pid_rpy[1] - m_pid_rpy[2] + thrust;
    m_test_pid[2] = + m_pid_rpy[0] - m_pid_rpy[1] + m_pid_rpy[2] + thrust;
    m_test_pid[3] = + m_pid_rpy[0] + m_pid_rpy[1] - m_pid_rpy[2] + thrust;

    Serial.println("PID_value");
    for (int i=0; i<4; i++) {
        if (m_test_pid[i] > PID_MAX) {
            m_test_pid[i] = PID_MAX;
        }
        if (m_test_pid[i] < 0) {
            m_test_pid[i] = 0;
        }

        m_test_pid[i] = 255*m_test_pid[i]/PID_MAX;
        #ifdef DEBUG_BUILD
            /*
            Serial.print(i);
            Serial.print(": ");
            Serial.print(m_test_pid[i]);
            if (i==4) Serial.print("\n");
            else Serial.println(", ");
            */
        #endif
    };
}

void Motor::format_cmd_data() {
    int cmd_thrust = m_cmd_data[0] * 2;
    int cmd_roll   = m_cmd_data[3] - 127;
    int cmd_pitch  = m_cmd_data[2] - 127;
    int cmd_yaw    = m_cmd_data[1] - 127;

    m_cmd_data[0] = + cmd_roll - cmd_pitch - cmd_yaw + cmd_thrust;
    m_cmd_data[1] = + cmd_roll + cmd_pitch + cmd_yaw + cmd_thrust;
    m_cmd_data[2] = - cmd_roll + cmd_pitch - cmd_yaw + cmd_thrust;
    m_cmd_data[3] = - cmd_roll - cmd_pitch + cmd_yaw + cmd_thrust;

    Serial.println("Command_value");
    for (int i=0; i<4; i++) {
        if (m_cmd_data[i] > 255) {
            m_cmd_data[i] = 255;
        }
        if (m_cmd_data[i] < 0) {
            m_cmd_data[i] = 0;
        }
        #ifdef DEBUG_BUILD
            /*
            Serial.print(i);
            Serial.print(": ");
            Serial.print(m_cmd_data[i]);
            if (i==4) Serial.print("\n");
            else Serial.println(", ");
            */
        #endif
    };
}

void Motor::stop_motor() {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    ledcWrite(2, 0);
    ledcWrite(3, 0);
}

void Motor::test_led(int cmd_data[4], float pid_rpy[3], Arm &arm) {
    if (arm.get_arm_status() == false) { return; }

    for (int i = 0; i < 4; i++) { m_cmd_data[i] = cmd_data[i]; }
    for (int i = 0; i < 3; i++) { m_pid_rpy[i] = pid_rpy[i]; }

    format_cmd_data();
    format_pid_data();

    ledcWrite(0, m_test_pid[0]*0.5 + m_cmd_data[0]*0.5);
    ledcWrite(1, m_test_pid[1]*0.5 + m_cmd_data[1]*0.5);
    ledcWrite(2, m_test_pid[2]*0.5 + m_cmd_data[2]*0.5);
    ledcWrite(3, m_test_pid[3]*0.5 + m_cmd_data[3]*0.5);
    /*
    Serial.println(m_test_pid[0]*0.5 + m_cmd_data[0]*0.5);
    Serial.println(m_test_pid[1]*0.5 + m_cmd_data[1]*0.5);
    Serial.println(m_test_pid[2]*0.5 + m_cmd_data[2]*0.5);
    Serial.println(m_test_pid[3]*0.5 + m_cmd_data[3]*0.5);
    */
}