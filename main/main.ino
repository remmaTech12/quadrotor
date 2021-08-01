#include "./include/recv.h"
#include "./include/imu_bmx055.h"
#include "./include/pid.h"
#include "./include/motor.h"
#include "./include/def_system.h"
#include "./include/emergency.h"
#include "./include/control.h"

imu_bmx055 imu;
Receiver receiver;
PID pid;
Motor motor;
Arm arm;
Emergency emergency;
Control control;

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
    float ang_data[3];
    float angvel_data[3];
    float ctl_data[3];

    emergency.emergency_stop(arm, motor);

    receiver.update_data();
    receiver.get_command(cmd_data);
    receiver.set_arm_status(arm);
    receiver.emergency_stop(arm, motor);

    imu.get_attitude_data(ang_data);
    imu.get_angvel_data(angvel_data);

    //pid.calculate_pid(rpy_data);
    //pid.get_pid(pid_data);
    control.calculate_pid_ang(cmd_data, ang_data);
    control.calculate_pid_angvel(angvel_data);

    motor.control(cmd_data, ctl_data, arm);
    //motor.test_control(128);
    //motor.test_count();

    delay(SAMPLING_TIME_MS);
}