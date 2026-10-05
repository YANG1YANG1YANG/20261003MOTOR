//
// Created by yangxiao on 2026/10/3.
//
#include "tools.h"


void motorHandler() {
    if (motor.has_feedback_ == true) {
        motor.canRxMsgCallback();
    }
}

void motorSetTorque(float current) {
    motor.setTxCurrent(current, 3);
}
