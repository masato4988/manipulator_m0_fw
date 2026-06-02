/*
 * pc_interface.c
 *
 *  Created on: May 3, 2026
 *      Author: miyab
 */


#include "pc_interface.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "app.h"
#include "joint_mapper.h"

/* =========================
   User settings
   ========================= */

#define PC_RX_LINE_BUF_SIZE   128
#define PC_TX_BUF_SIZE        128
#define AXIS_NUM              6

/* =========================
   External functions
   既存の自分の関数名に合わせて変更してください
   ========================= */

/* 例：6軸の目標関節角度[rad]をセット */
extern HAL_StatusTypeDef joint_mapper_set_target_rad(float q1_target_rad,
													float q2_target_rad,
													float q3_target_rad,
													float q4_target_rad,
													float q5_target_rad,
													float q6_target_rad,
													float dq_max_rad_s,
													float ddq_max_rad_s2);

/* 例：原点復帰開始 */
extern HAL_StatusTypeDef app_start_homing_all(void);

/* 例：非常停止または通常停止 */
extern HAL_StatusTypeDef joint_mapper_stop_all(void);

/* 例：現在角度取得 */
extern HAL_StatusTypeDef joint_mapper_ger_rad_all(float q_rad[AXIS_NUM]);

/* 例：原点復帰済み判定 */
extern HAL_StatusTypeDef app_get_homing_seq_state(AppHomingSeqState_t *state);

/* 例：動作中判定 */
extern HAL_StatusTypeDef app_get_mode(AppMode_t *mode);


/* =========================
   Private variables
   ========================= */

static UART_HandleTypeDef *pc_huart = NULL;

static uint8_t rx_byte;

static char line_buf[PC_RX_LINE_BUF_SIZE];
static volatile uint16_t line_index = 0;
static volatile bool line_ready = false;

static char command_buf[PC_RX_LINE_BUF_SIZE];


/* =========================
   Private prototypes
   ========================= */

static void pc_send(const char *msg);
static void pc_sendf(const char *fmt, ...);

static void process_line(char *line);
static void trim_line(char *s);

//static void pc_send_joint_position(void);
static bool parse_move_deg(char *args);
//static bool parse_move_rad(char *args);
static bool parse_move_mrad(char *args);

/* =========================
   Public functions
   ========================= */

void pc_interface_init(UART_HandleTypeDef *huart)
{
    pc_huart = huart;
    line_index = 0;
    line_ready = false;

    HAL_UART_Receive_IT(pc_huart, &rx_byte, 1);

    pc_send("PC_IF READY\r\n");
}


void pc_interface_update(void)
{
    if (!line_ready) {
        return;
    }

    __disable_irq();
    strncpy(command_buf, line_buf, PC_RX_LINE_BUF_SIZE);
    command_buf[PC_RX_LINE_BUF_SIZE - 1] = '\0';
    line_ready = false;
    __enable_irq();

    process_line(command_buf);
}


void pc_interface_uart_rx_callback(UART_HandleTypeDef *huart)
{
    if (huart != pc_huart) {
        return;
    }

    char c = (char)rx_byte;

    if (c == '\r' || c == '\n') {
        if (line_index > 0) {
            line_buf[line_index] = '\0';
            line_ready = true;
            line_index = 0;
        }
    } else {
        if (line_index < PC_RX_LINE_BUF_SIZE - 1) {
            line_buf[line_index++] = c;
        } else {
            line_index = 0;
            line_ready = false;
        }
    }

    HAL_UART_Receive_IT(pc_huart, &rx_byte, 1);
}

#include <stdarg.h>

static void pc_send(const char *msg)
{
    if (pc_huart == NULL) {
        return;
    }

    HAL_UART_Transmit(pc_huart, (uint8_t *)msg, strlen(msg), 100);
}


static void pc_sendf(const char *fmt, ...)
{
    char tx_buf[PC_TX_BUF_SIZE];

    va_list args;
    va_start(args, fmt);
    vsnprintf(tx_buf, sizeof(tx_buf), fmt, args);
    va_end(args);

    pc_send(tx_buf);
}

static void process_line(char *line)
{
    AppHomingSeqState_t homing_state;
    AppMode_t app_mode;

    trim_line(line);

    if (strlen(line) == 0) {
        return;
    }

    /* =========================
       PC / ROS向け 短縮コマンド
       ========================= */

    // PING: 通信確認
    if (strcmp(line, "P") == 0 || strcmp(line, "PING") == 0) {
        pc_send("OK PONG\r\n");
        return;
    }

    // STATUS: 状態だけ返す
    if (strcmp(line, "S") == 0 || strcmp(line, "STATUS") == 0) {
        app_get_homing_seq_state(&homing_state);
        app_get_mode(&app_mode);

        if (app_mode == APP_MODE_ERROR) {
            pc_send("OK STATUS ERROR\r\n");
            return;
        }

        if (app_mode == APP_MODE_HOMING) {
            pc_send("OK STATUS HOMING\r\n");
            return;
        }

        if (app_mode == APP_MODE_JOINT_CONTROL) {
            pc_send("OK STATUS JOINT_CONTROL\r\n");
            return;
        }

        if (homing_state != APP_HOMING_SEQ_IDLE) {
            pc_send("OK STATUS NOT_HOMED\r\n");
            return;
        }

        pc_send("OK STATUS IDLE\r\n");
        return;
    }

    // Q: PC向け関節角取得 rad
    if (strcmp(line, "Q") == 0) {
        float q[AXIS_NUM];

        if (joint_mapper_get_rad_all(q) != HAL_OK) {
            pc_send("ERR GET_JOINT\r\n");
            return;
        }

        pc_sendf(
            "OK Q %ld %ld %ld %ld %ld %ld\r\n",
            (int32_t)(q[0] * 1000.0f),
            (int32_t)(q[1] * 1000.0f),
            (int32_t)(q[2] * 1000.0f),
            (int32_t)(q[3] * 1000.0f),
            (int32_t)(q[4] * 1000.0f),
            (int32_t)(q[5] * 1000.0f)
        );
        return;
    }

    // M: PC向け関節移動 rad
    // 例: M 0.000 0.500 -0.300 0.000 0.000 0.000
    if (strncmp(line, "M ", 2) == 0) {
        app_get_homing_seq_state(&homing_state);

        if (homing_state != APP_HOMING_SEQ_IDLE) {
            pc_send("ERR NOT_HOMED\r\n");
            return;
        }

        if (parse_move_mrad(line + 2)) {
            pc_send("OK MOVE\r\n");
        } else {
            pc_send("ERR MOVE_FORMAT\r\n");
        }
        return;
    }

    // H: ホーミング開始
    if (strcmp(line, "H") == 0 || strcmp(line, "HOME") == 0) {
        if (app_start_homing_all() == HAL_OK) {
            pc_send("OK HOME\r\n");
        } else {
            pc_send("ERR HOME\r\n");
        }
        return;
    }

    // X: 緊急停止/停止
    if (strcmp(line, "X") == 0 || strcmp(line, "STOP") == 0) {
        joint_mapper_stop_all();
        pc_send("OK STOP\r\n");
        return;
    }

    // RESET
    if (strcmp(line, "RESET") == 0) {
        app_init();
        pc_send("OK RESET\r\n");
        return;
    }

    /* =========================
       人間デバッグ向け 長いコマンド
       ========================= */

    // deg表示用
    if (strcmp(line, "GET_JOINT") == 0) {
        float q[AXIS_NUM];

        if (joint_mapper_get_rad_all(q) != HAL_OK) {
            pc_send("ERR GET_JOINT\r\n");
            return;
        }

        pc_sendf(
            "OK JOINT_DEG %.2f %.2f %.2f %.2f %.2f %.2f\r\n",
            q[0] * 57.2957795f,
            q[1] * 57.2957795f,
            q[2] * 57.2957795f,
            q[3] * 57.2957795f,
            q[4] * 57.2957795f,
            q[5] * 57.2957795f
        );
        return;
    }

    // deg指定移動 デバッグ用
    // 例: MOVE_DEG 0 30 -20 0 0 0
    if (strncmp(line, "MOVE_DEG ", 9) == 0) {
        app_get_homing_seq_state(&homing_state);

        if (homing_state != APP_HOMING_SEQ_DONE) {
            pc_send("ERR NOT_HOMED\r\n");
            return;
        }

        if (parse_move_deg(line + 9)) {
            pc_send("OK MOVE_DEG\r\n");
        } else {
            pc_send("ERR MOVE_DEG_FORMAT\r\n");
        }
        return;
    }

    // 互換用
    if (strcmp(line, "HELLO") == 0) {
        pc_send("OK HELLO\r\n");
        return;
    }

    pc_send("ERR UNKNOWN_CMD\r\n");
}
//static bool parse_move_rad(char *args)
//{
//    float q_rad[AXIS_NUM];
//
//    int count = sscanf(
//        args,
//        "%f %f %f %f %f %f",
//        &q_rad[0],
//        &q_rad[1],
//        &q_rad[2],
//        &q_rad[3],
//        &q_rad[4],
//        &q_rad[5]
//    );
//
//    if (count != AXIS_NUM) {
//        return false;
//    }
//
//    return app_pc_move_rad(
//        q_rad[0],
//        q_rad[1],
//        q_rad[2],
//        q_rad[3],
//        q_rad[4],
//        q_rad[5],
//        0.3f,   // dq_max_rad_s
//        1.0f    // ddq_max_rad_s2
//    ) == HAL_OK;
//}

static bool parse_move_mrad(char *args)
{
    int32_t q_mrad[AXIS_NUM];

    int count = sscanf(
        args,
        "%ld %ld %ld %ld %ld %ld",
        &q_mrad[0],
        &q_mrad[1],
        &q_mrad[2],
        &q_mrad[3],
        &q_mrad[4],
        &q_mrad[5]
    );

    if (count != AXIS_NUM) {
        return false;
    }

    return app_pc_move_rad(
        q_mrad[0] / 1000.0f,
        q_mrad[1] / 1000.0f,
        q_mrad[2] / 1000.0f,
        q_mrad[3] / 1000.0f,
        q_mrad[4] / 1000.0f,
        q_mrad[5] / 1000.0f,
        0.3f,
        1.0f
    ) == HAL_OK;
}

static bool parse_move_deg(char *args)
{
    float q_deg[AXIS_NUM];
    float q_rad[AXIS_NUM];

    int count = sscanf(
        args,
        "%f %f %f %f %f %f",
        &q_deg[0],
        &q_deg[1],
        &q_deg[2],
        &q_deg[3],
        &q_deg[4],
        &q_deg[5]
    );

    if (count != AXIS_NUM) {
        return false;
    }

    for (int i = 0; i < AXIS_NUM; i++) {
        q_rad[i] = q_deg[i] * 0.01745329252f;
    }

    return app_pc_move_rad(
        q_rad[0],
        q_rad[1],
        q_rad[2],
        q_rad[3],
        q_rad[4],
        q_rad[5],
        0.3f,   // dq_max_rad_s 仮
        1.0f    // ddq_max_rad_s2 仮
    ) == HAL_OK;
}

//static void pc_send_joint_position(void)
//{
//    float q[AXIS_NUM] = {0};
//
//    if (joint_mapper_get_rad_all(q) != HAL_OK) {
//        pc_send("ERR GET_Q\r\n");
//        return;
//    }
//
//    pc_sendf(
//        "Q %d %d %d %d %d %d\r\n",
//        (int)(q[0] * 57.2957795f * 100.0f),
//        (int)(q[1] * 57.2957795f * 100.0f),
//        (int)(q[2] * 57.2957795f * 100.0f),
//        (int)(q[3] * 57.2957795f * 100.0f),
//        (int)(q[4] * 57.2957795f * 100.0f),
//        (int)(q[5] * 57.2957795f * 100.0f)
//    );
//}


static void trim_line(char *s)
{
    char *p = s;

    while (isspace((unsigned char)*p)) {
        p++;
    }

    if (p != s) {
        memmove(s, p, strlen(p) + 1);
    }

    int len = strlen(s);
    while (len > 0 && isspace((unsigned char)s[len - 1])) {
        s[len - 1] = '\0';
        len--;
    }
}
