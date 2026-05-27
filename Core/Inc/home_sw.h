/*
 * home_sw.h
 *
 *  Created on: Mar 31, 2026
 *      Author: miyab
 */

#ifndef INC_HOME_SW_H_
#define INC_HOME_SW_H_

#include "main.h"
#include "stdbool.h"

typedef enum {
    SW_PRESSED = 1,
    SW_RELEASED = 0
} HomeSwitchState_t;

HAL_StatusTypeDef home_sw_read(uint8_t axis, HomeSwitchState_t *is_on);
//pressed:1/released:0


#endif /* INC_HOME_SW_H_ */
