#include "recv.h"
#include "imu_bmx055.h"
#include "pid.h"
#include "motor.h"
#include "def_system.h"
#include "emergency.h"

imu_bmx055 imu;
Receiver receiver;
PID pid;
Motor motor;
Arm arm;
Emergency emergency;

void setup() {
    Serial.begin(115200);

    imu.setup();
    receiver.setup();
    motor.setup();
    emergency.setup();

    delay(300);
}

void loop() {
    int cmd_data[4];
    float rpy_data[3];
    float pid_data[3];

    emergency.emergency_stop(arm, motor);

    receiver.update_data();
    receiver.get_command(cmd_data);
    receiver.set_arm_status(arm);
    receiver.emergency_stop(arm, motor);

    imu.get_attitude_data(rpy_data);

    pid.calculate_pid(rpy_data);
    pid.get_pid(pid_data);

    motor.test_led(cmd_data, pid_data, arm);

    delay(SAMPLING_TIME_MS);
}