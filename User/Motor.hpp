//
// Created by yangxiao on 2026/10/3.
//

#ifndef CAN_MOTOR_H
#define CAN_MOTOR_H
#include "stdint.h"
#include "stdbool.h"
#include "pid.h"
#include "math.h"

#define RAD2DEG  (3.14159265358979323846f / 180.0f)

class Motor {
public:
    explicit Motor(float ratio,float load_mass, float load_dist);

    void canRxMsgCallback();

    float angle();
    float speedRpm();
    // float currentAmps();
    // float temperatureC();
    //bool hasFeedback();
    void setTxCurrent(float amperes);

    uint8_t* getTxData();

    uint8_t rx_data_[8] = {};
    uint8_t rx_buff_[8] = {};
    volatile bool has_feedback_ = false;
    volatile bool has_data_=false;

    PID spid,ppid;

    uint8_t motor_id_=3;

    float coeff_feedforwardintensity_ = 5.5556;

    typedef enum ControlMethod {
        POSITION_SPEED,
        SPEED,
        TORQUE,
    } ControlMethod_e;

    ControlMethod_e controlMethod_;
    void handle();
    void setPosition(float target_position,float feedforward_speed,float feedforward_intensity);
    void setSpeed(float target_speed, float feedforward_intensity);
    void setIntensity(float intensity);
    void stop();

private:
    // ---- 配置 ----
    const float ratio_; // 减速比

    // ---- 这一帧解出来的 ----
    float ecd_angle_ = 0; // 机械角度(0~8191)
    float speedRpm_ = 0;  // 转速(RPM)
    // float currentA_ = 0;  // 转矩电流(A)
    // float tempC_ = 0;     // 温度(℃)

    // ---- 连续角度 ----
    float angle_ = 0;       // 输出轴累计角度
    float last_ecd_ = 0;    // 上一帧角度，用来求差
    float torque_ = 0;
    bool received_ = false; // 第一帧特殊处理

    // float load_mass_ = 0.1f;
    // float load_dist_ = 0.065;

    float load_mass_ = 0;
    float load_dist_ = 0;

    float target_angle_,fdb_angle_;
    float target_speed_,current_speed_,feedforward_speed_;
    float feedforward_intensity_,intensity_;

    // ---- 待发送 ----
    uint8_t tx_data_[8] = {};

    void processData();

    float wrapEncoderDelta(float delta_ecd);

    float limit(float value, float min, float max);

    float scale(float x, float in_scale, float out_scale);

    static constexpr float kEncoderRange = 8192.0f;
    static constexpr float kEncoderWrapThreshold = 4096.0f;
    static constexpr float kAmperesMax = 10.0f;
    static constexpr float kCurrentScale = 10000.0f;
    static constexpr float kAngleScale = 360.0f;

    void calcGravityFeedforwardIntensity();
    void Position();
    void Speed();
    void Intensity();
};

#endif //CAN_MOTOR_H
