//
// Created by yangxiao on 2026/10/3.
//
#include "can.h"
#include "can_user.h"
#include "Motor.hpp"

extern Motor motor;     // 减速比作为构造参数
/* UserCode/can_user.h */
// extern CAN_RxHeaderTypeDef rx_header;
// extern CAN_TxHeaderTypeDef tx_header;
// extern uint32_t can_tx_mailbox;
// extern CAN_FilterTypeDef can_filter_config;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    if (hcan->Instance==CAN1) {
        if (HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,motor.rx_data_)==HAL_OK){
            if (rx_header.StdId==0x203&&rx_header.IDE==CAN_ID_STD&&rx_header.RTR==CAN_RTR_DATA&&rx_header.DLC==8) {
                motor.has_feedback_=true;
            }
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance != TIM6) return;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
        HAL_CAN_AddTxMessage(&hcan1, &tx_header,
                             motor.getTxData(),
                             &can_tx_mailbox);
    }
}
