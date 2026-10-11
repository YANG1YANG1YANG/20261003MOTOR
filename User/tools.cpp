//
// Created by yangxiao on 2026/10/3.
//
#include "tools.h"


void motorHandler() {
    if (stop_flag == true) {
        motor.stop();
        return;
    }
    if (motor.has_data_ == true) {
        motor.handle();
    }
}

// void motorSetTorque(float current) {
//     motor.setTxCurrent(current, 3);
// }
