//
// Created by yangxiao on 2026/10/11.
//

#ifndef PID_PID_H
#define PID_PID_H
class PID {
public:
    PID(float kp,float ki,float kd,float i_max,float out_max,float d_filter_k);
    PID()=default;
    void reset();
    float calc(float ref,float fdb);
    float kp_;
    float ki_;
    float kd_;
    float d_filter_k_;
private:
    float integral_;
    float last_error_;
    float error_;
    float i_max_;
    float out_max_;
    float d_error_;
    float dout_now_;
    float dout_;
    float kout_;
    float iout_;
    float output_;
    float last_fdb_;
    float d_fdb_;
};
#endif //PID_PID_H
