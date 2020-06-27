#include "recv.h"
#include "imu_bmx055.h"
#include "pid.h"
#include "motor.h"
#include "def_system.h"

imu_bmx055 imu;
Receiver receiver;
PID pid;
Motor motor;

void setup() {
    Serial.begin(115200);

    imu.setup();
    pid.setup();
    receiver.setup();
    motor.setup();

    delay(300);
}

void loop() {
    int cmd_data[4];
    float rpy_data[3];
    float pid_data[3];

    receiver.update_data();
    receiver.get_command(cmd_data);

    imu.get_attitude_data(rpy_data);

    pid.set_rpy(rpy_data);
    pid.calculate_pid();
    pid.get_pid(pid_data);

    for (int i=0; i<4; i++) {
        Serial.println(cmd_data[i]);
    }

    motor.test_led_cmd(cmd_data);

    //delay(SAMPLING_TIME_MS);
}