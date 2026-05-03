/*
 * sts_manager.c
 *
 *  Created on: May 3, 2026
 *      Author: miyab
 */


#include "sts_servo/sts_manager.h"
#include "sts_servo/sts_bus.h"
#include "usart.h"   // huartX
#include <stdbool.h>
#include <stdint.h>

/* =========================
   構成パラメータ
   ========================= */

#define STS_UART_HANDLE     huart1

#define STS_Q4_ID           11
#define STS_Q5_ID           12
#define STS_Q6_ID           13

#define STS_TX_TIMEOUT_MS   10
#define STS_RX_TIMEOUT_MS   10

#define STS_HOME_POS_Q4        2048
#define STS_HOME_POS_Q5        1024
#define STS_HOME_POS_Q6        2048

#define STS_HOME_TOLERANCE     10



typedef enum {
    STS_HOME_IDLE = 0,
    STS_HOME_MOVING,
    STS_HOME_DONE,
    STS_HOME_ERROR
} StsHomeState_t;

static StsHomeState_t sts_home_state = STS_HOME_IDLE;

static const uint16_t sts_home_pos[3] = {
    STS_HOME_POS_Q4,
    STS_HOME_POS_Q5,
    STS_HOME_POS_Q6
};

/* =========================
   実体
   ========================= */

static sts_bus_t sts_bus1;

sts3215_t sts_q4;
sts3215_t sts_q5;
sts3215_t sts_q6;

/* =========================
   初期化
   ========================= */

HAL_StatusTypeDef sts_manager_init(void)
{
    /* ---- bus 初期化 ---- */

    sts_bus_init(
        &sts_bus1,
        &STS_UART_HANDLE,
        STS_TX_TIMEOUT_MS,
        STS_RX_TIMEOUT_MS
    );

    /* ---- servo 初期化 ---- */

    sts3215_init(&sts_q4, &sts_bus1, STS_Q4_ID);
    sts3215_init(&sts_q5, &sts_bus1, STS_Q5_ID);
    sts3215_init(&sts_q6, &sts_bus1, STS_Q6_ID);

    /* ---- 通信確認（任意） ---- */

    if (sts3215_ping(&sts_q4) != STS_OK) {
        return HAL_ERROR;
    }

    if (sts3215_ping(&sts_q5) != STS_OK) {
        return HAL_ERROR;
    }

    if (sts3215_ping(&sts_q6) != STS_OK) {
        return HAL_ERROR;
    }

    /* ---- トルクON ---- */

    if (sts3215_set_torque_enable(&sts_q4, true) != STS_OK) {
        return HAL_ERROR;
    }

    if (sts3215_set_torque_enable(&sts_q5, true) != STS_OK) {
        return HAL_ERROR;
    }

    if (sts3215_set_torque_enable(&sts_q6, true) != STS_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* =========================
   停止
   ========================= */

HAL_StatusTypeDef sts_manager_stop_all(void)
{
    if (sts3215_stop(&sts_q4) != STS_OK)
        return HAL_ERROR;

    if (sts3215_stop(&sts_q5) != STS_OK)
        return HAL_ERROR;

    if (sts3215_stop(&sts_q6) != STS_OK)
        return HAL_ERROR;

    return HAL_OK;
}
/* =========================
   ３軸原点復帰
   ========================= */

HAL_StatusTypeDef sts_manager_start_homing_all(void)
{
    if (sts3215_set_goal_position(&sts_q4, sts_home_pos[0]) != STS_OK) {
        sts_home_state = STS_HOME_ERROR;
        return HAL_ERROR;
    }

    if (sts3215_set_goal_position(&sts_q5, sts_home_pos[1]) != STS_OK) {
        sts_home_state = STS_HOME_ERROR;
        return HAL_ERROR;
    }

    if (sts3215_set_goal_position(&sts_q6, sts_home_pos[2]) != STS_OK) {
        sts_home_state = STS_HOME_ERROR;
        return HAL_ERROR;
    }

    sts_home_state = STS_HOME_MOVING;
    return HAL_OK;
}
/* =========================
   原点復帰中の処理
   ========================= */

HAL_StatusTypeDef sts_manager_update_homing_all(void)
{
    bool reached[3];

    if (sts_home_state == STS_HOME_IDLE ||
        sts_home_state == STS_HOME_DONE) {
        return HAL_OK;
    }

    if (sts_home_state == STS_HOME_ERROR) {
        return HAL_ERROR;
    }

    if (sts3215_is_reached(&sts_q4,
                           sts_home_pos[0],
                           STS_HOME_TOLERANCE,
                           &reached[0]) != STS_OK) {
        sts_home_state = STS_HOME_ERROR;
        return HAL_ERROR;
    }

    if (sts3215_is_reached(&sts_q5,
                           sts_home_pos[1],
                           STS_HOME_TOLERANCE,
                           &reached[1]) != STS_OK) {
        sts_home_state = STS_HOME_ERROR;
        return HAL_ERROR;
    }

    if (sts3215_is_reached(&sts_q6,
                           sts_home_pos[2],
                           STS_HOME_TOLERANCE,
                           &reached[2]) != STS_OK) {
        sts_home_state = STS_HOME_ERROR;
        return HAL_ERROR;
    }

    if (reached[0] && reached[1] && reached[2]) {
        sts_home_state = STS_HOME_DONE;
    }

    return HAL_OK;
}
/* =========================
   ３軸あわせて目標到達確認
   ========================= */

HAL_StatusTypeDef sts_manager_is_homing_done(bool *done)
{
    if (done == NULL) {
        return HAL_ERROR;
    }

    *done = (sts_home_state == STS_HOME_DONE);
    return HAL_OK;
}
