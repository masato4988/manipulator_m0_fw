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
static bool parse_move_deg(char *args);


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
	AppHomingSeqState_t state;
	AppMode_t mode;

    trim_line(line);

    if (strlen(line) == 0) {
        return;
    }

    if (strncmp(line, "MOVE ", 5) == 0) {
        if (app_get_homing_seq_state(&state) == HAL_OK) {
        	if(state != APP_HOMING_SEQ_IDLE){
				pc_send("ERR NOT_HOMED\r\n");
				return;
        	}
        }

        if (parse_move_deg(line + 5)) {
            pc_send("OK MOVE\r\n");
        } else {
            pc_send("ERR MOVE_FORMAT\r\n");
        }
        return;
    }

    if (strcmp(line, "HOME") == 0) {
        if (app_start_homing_all() == HAL_OK) {
            pc_send("OK HOME\r\n");
        } else {
            pc_send("ERR HOME\r\n");
        }
        return;
    }

    if (strcmp(line, "STOP") == 0) {
    	joint_mapper_stop_all();
        pc_send("OK STOP\r\n");
        return;
    }

    if (strcmp(line, "RESET") == 0) {
    	app_init();
        pc_send("OK RESET\r\n");
        return;
    }

    if (strcmp(line, "STATUS") == 0) {
        float q[AXIS_NUM];
        if(joint_mapper_get_rad_all(q) != HAL_OK){
        	pc_send("ERROR");
        	return;
        }
        app_get_homing_seq_state(&state);
        app_get_mode(&mode);

//        pc_sendf(
//            "STATUS homed=%d busy=%d q_deg=%.2f %.2f %.2f %.2f %.2f %.2f\r\n",
//			state,
//			mode,
//            q[0] * 57.2957795f,
//            q[1] * 57.2957795f,
//            q[2] * 57.2957795f,
//            q[3] * 57.2957795f,
//            q[4] * 57.2957795f,
//            q[5] * 57.2957795f
//        );
        pc_sendf(
            "STATUS AppHomingSeqState=%d AppMode=%d q_cdeg=%d %d %d %d %d %d\r\n",
            state,
            mode,
            (int)(q[0] * 57.2957795f * 100.0f),
            (int)(q[1] * 57.2957795f * 100.0f),
            (int)(q[2] * 57.2957795f * 100.0f),
            (int)(q[3] * 57.2957795f * 100.0f),
            (int)(q[4] * 57.2957795f * 100.0f),
            (int)(q[5] * 57.2957795f * 100.0f)
        );
        return;
    }

    pc_send("ERR UNKNOWN_CMD\r\n");
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
