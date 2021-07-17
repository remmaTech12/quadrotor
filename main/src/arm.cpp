#include "../include/arm.h"

Arm::Arm() {}

void Arm::setup() {
}

void Arm::set_arm_status(bool armed) {
    m_armed = armed;
}

bool Arm::get_arm_status() {
    return m_armed;
}