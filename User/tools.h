//
// Created by yangxiao on 2026/10/3.
//

#ifndef CAN_TOOLS_H
#define CAN_TOOLS_H

#ifdef __cplusplus

#include "Motor.hpp"
extern Motor motor;
extern bool stop_flag;

extern "C" {
#endif

void motorHandler(void);

// void motorSetTorque(float current);
#ifdef __cplusplus
}
#endif

#endif //CAN_TOOLS_H
