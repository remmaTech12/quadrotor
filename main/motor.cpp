#include "motor.h"

Motor::Motor() {}

void Motor::setup() {
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
    int thrust = PID_MAX/2;
    m_test_pid[0] = - m_pid_rpy[0] + m_pid_rpy[1] + m_pid_rpy[2] + thrust;
    m_test_pid[1] = - m_pid_rpy[0] - m_pid_rpy[1] - m_pid_rpy[2] + thrust;
    m_test_pid[2] = + m_pid_rpy[0] - m_pid_rpy[1] + m_pid_rpy[2] + thrust;
    m_test_pid[3] = + m_pid_rpy[0] + m_pid_rpy[1] - m_pid_rpy[2] + thrust;

    for (int i=0; i<4; i++) {
        if (m_test_pid[i] > PID_MAX) {
            m_test_pid[i] = PID_MAX;
        }
        if (m_test_pid[i] < 0) {
            m_test_pid[i] = 0;
        }

        m_test_pid[i] = 255*m_test_pid[i]/PID_MAX;
        #ifdef DEBUG_BUILD
            Serial.print(i);
            Serial.print(": ");
            Serial.print(m_test_pid[i]);
        #endif
    };
    #ifdef DEBUG_BUILD
        Serial.print("\n");
    #endif
}

void Motor::test_led_pid(int cmd_data[4], float pid_rpy[3]) {
    for (int i = 0; i < 4; i++) { m_cmd_data[i] = cmd_data[i]; }
    for (int i = 0; i < 3; i++) { m_pid_rpy[i] = pid_rpy[i]; }
    //format_cmd_data();
    format_pid_data();

    ledcWrite(0, m_test_pid[0]);
    ledcWrite(1, m_test_pid[1]);
    ledcWrite(2, m_test_pid[2]);
    ledcWrite(3, m_test_pid[3]);
}

void Motor::test_led_cmd(int cmd_data[4]) {
    ledcWrite(0, cmd_data[0]);
    ledcWrite(1, cmd_data[1]);
    ledcWrite(2, cmd_data[2]);
    ledcWrite(3, cmd_data[3]);
}