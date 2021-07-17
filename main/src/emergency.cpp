#include "../include/def_system.h"
#include "../include/emergency.h"

Emergency::Emergency() {}

void Emergency::setup() {
    pinMode(EMERGENCY_SWITCH, INPUT_PULLUP);
}

void Emergency::emergency_stop(Arm &arm, Motor &motor) {
    if (digitalRead(EMERGENCY_SWITCH) == LOW) {
        arm.set_arm_status(false);
        motor.stop_motor();
    }
}