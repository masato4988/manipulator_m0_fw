/*
 * pc_interface.h
 *
 *  Created on: May 3, 2026
 *      Author: miyab
 */

#ifndef INC_PC_INTERFACE_H_
#define INC_PC_INTERFACE_H_

#include "main.h"
#include <stdbool.h>

void pc_interface_init(UART_HandleTypeDef *huart);
void pc_interface_update(void);

/* UART受信割り込みから呼ぶ */
void pc_interface_uart_rx_callback(UART_HandleTypeDef *huart);


#endif /* INC_PC_INTERFACE_H_ */
