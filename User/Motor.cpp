//
// Created by yangxiao on 2026/10/3.
//
#include "Motor.hpp"
#include "main.h"
#include "string.h"

Motor::Motor(float ratio,float load_mass, float load_dist):
    ratio_(ratio),spid{1.1,0,1.9,0,10,0.1},ppid{16,0,0.015,0,500,0.1},load_mass_(load_mass),load_dist_(load_dist) {}

void Motor::canRxMsgCallback() {
    processData();
}

void Motor::processData() {
    uint32_t saved_primask = __get_PRIMASK();
    __disable_irq();
    memcpy(rx_data_,rx_buff_,sizeof(rx_data_));
    has_data_ = false;
    __set_PRIMASK(saved_primask);
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
        current_speed_ = speedRpm_/ratio_;
    }
    // int16_t current_raw = static_cast<int16_t>(rx_data_[4] << 8 | rx_data_[5]);
    // currentA_ = scale(current_raw, kCurrentScale, kAmperesMax);
    // tempC_ = static_cast<float>(rx_data_[6]);
    torque_= static_cast<float>(rx_data_[4] << 8 | rx_data_[5]);
}


// 出口：控制逻辑读取
float Motor::angle() {
    return angle_;
}

float Motor::speedRpm() {
    return speedRpm_;
}
// float Motor::currentAmps() {
//     return currentA_;
// }
// float Motor::temperatureC() {
//     return tempC_;
// }


void Motor::setTxCurrent(float amperes) {
    uint8_t offset = motor_id_ - 1;
    amperes = limit(amperes, -kAmperesMax, kAmperesMax);
    int16_t current = static_cast<int16_t>(scale(amperes, kAmperesMax, kCurrentScale));
    uint32_t saved_primask = __get_PRIMASK();
    __disable_irq();
    tx_data_[0 + offset * 2] = static_cast<uint8_t>(current >> 8);
    tx_data_[1 + offset * 2] = static_cast<uint8_t>(current);
    __set_PRIMASK(saved_primask);
}

// 命令出口：TIM6 中断取用（HAL 形参非 const）
uint8_t* Motor::getTxData() {
    return tx_data_;
}

void Motor::handle() {
    canRxMsgCallback();
    switch (controlMethod_) {
        case POSITION_SPEED:
        //setPosition(target_angle_,feedforward_speed_,feedforward_intensity_);
        Position();
        break;
        case SPEED:
        //setSpeed(feedforward_speed_,feedforward_intensity_);
        Speed();
        break;
        case TORQUE:
        //setIntensity(intensity_);
        Intensity();
        break;
        default:
        break;
    }
    setTxCurrent(intensity_);
}

void Motor::calcGravityFeedforwardIntensity() {
    feedforward_intensity_=coeff_feedforwardintensity_*(sin(angle_*RAD2DEG))*9.8f*load_dist_*load_mass_;
}

void Motor::Position() {
    target_speed_=ppid.calc(target_angle_,angle_)+feedforward_speed_;
    calcGravityFeedforwardIntensity();
    intensity_=spid.calc(target_speed_,current_speed_)+feedforward_intensity_;
}

void Motor::Speed() {
    intensity_=spid.calc(target_speed_,current_speed_)+feedforward_intensity_;
}

void Motor::Intensity() {
    controlMethod_ = TORQUE;
}

void Motor::stop() {
    intensity_=0;
    ppid.reset();
    spid.reset();
    setTxCurrent(intensity_);
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

void Motor::setPosition(float target_position,float feedforward_speed,float feedforward_intensity) {
    target_angle_ = target_position;
    feedforward_speed_ = feedforward_speed;
    feedforward_intensity_ = feedforward_intensity;
    controlMethod_ = POSITION_SPEED;
}
void Motor::setSpeed(float target_speed, float feedforward_intensity) {
    target_speed_ = target_speed;
    feedforward_intensity_ = feedforward_intensity;
    controlMethod_ = SPEED;
}
void Motor::setIntensity(float intensity) {
    intensity_ = intensity;
    controlMethod_ = TORQUE;
}

Motor motor(36.0f,0.1f,0.065f);
