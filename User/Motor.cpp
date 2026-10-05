//
// Created by yangxiao on 2026/10/3.
//
#include "Motor.hpp"

Motor::Motor(float ratio) :
    ratio_(ratio) {}

void Motor::canRxMsgCallback() {
    processData();
    has_feedback_ = false;
}

void Motor::processData() {
    ecd_angle_ = static_cast<float>(rx_data_[0] << 8 | rx_data_[1]);
    if (received_ == false) {
        received_ = true;
        last_ecd_ = ecd_angle_;
    } else {
        ecd_angle_ = static_cast<float>(rx_data_[0] << 8 | rx_data_[1]);
        float delta_ecd = ecd_angle_ - last_ecd_;
        delta_ecd = wrapEncoderDelta(delta_ecd);
        angle_ += scale(delta_ecd, kEncoderRange, kAngleScale) / ratio_;
        last_ecd_ = ecd_angle_;
        speedRpm_ = static_cast<float>(static_cast<int16_t>(rx_data_[2] << 8 | rx_data_[3]));
    }
    int16_t current_raw = static_cast<int16_t>(rx_data_[4] << 8 | rx_data_[5]);
    currentA_ = scale(current_raw, kCurrentScale, kAmperesMax);
    tempC_ = static_cast<float>(rx_data_[6]);
}

// 出口：控制逻辑读取
float Motor::angle() {
    return angle_;
}

float Motor::speedRpm() {
    return speedRpm_;
} // 转子转速（RPM）
float Motor::currentAmps() {
    return currentA_;
} // 转矩电流（A）
float Motor::temperatureC() {
    return tempC_;
} // 温度（℃）

// 命令：控制逻辑调用，内部限幅
void Motor::setTxCurrent(float amperes, uint8_t motor_id) {
    uint8_t offset = motor_id - 1;
    amperes = limit(amperes, -kAmperesMax, kAmperesMax);
    int16_t current = static_cast<int16_t>(scale(amperes, kAmperesMax, kCurrentScale));
    tx_data_[0 + offset * 2] = static_cast<uint8_t>(current >> 8);
    tx_data_[1 + offset * 2] = static_cast<uint8_t>(current);
}

// 命令出口：TIM6 中断取用（HAL 形参非 const）
uint8_t* Motor::getTxData() {
    return tx_data_;
}

float Motor::limit(float value, float min, float max) {
    if (value < min) {
        value = min;
    } else if (value > max) {
        value = max;
    }
    return value;
}

float Motor::scale(float x, float in_scale, float out_scale) {
    return x / in_scale * out_scale;
}

float Motor::wrapEncoderDelta(float delta_ecd) {
    if (delta_ecd > kEncoderWrapThreshold)
        delta_ecd -= kEncoderRange;
    else if (delta_ecd < -kEncoderWrapThreshold)
        delta_ecd += kEncoderRange;
    return delta_ecd;
}

Motor motor(19.2f);
