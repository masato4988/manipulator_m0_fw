/*
 * home_sw.c
 *
 *  Created on: Mar 31, 2026
 *      Author: miyab
 */
#include "home_sw.h"
#include "system_config.h"
#include "main.h"

#define HOME_SW_PRESSED_LEVEL GPIO_PIN_SET

static HomeSwitchState_t gpio_to_pressed(GPIO_PinState pin)
{
	if(pin == HOME_SW_PRESSED_LEVEL){
		return SW_PRESSED;
	}else{
		return SW_RELEASED;
	}
}

HAL_StatusTypeDef home_sw_read(uint8_t axis, HomeSwitchState_t *is_pressed)
{
    GPIO_PinState pin;

    if (axis < 1 || axis > AXIS_COUNT) return HAL_ERROR;
    if (is_pressed == NULL) return HAL_ERROR;

    switch (axis) {
    case 1:
        pin = HAL_GPIO_ReadPin(SW_LIMIT_1_GPIO_Port, SW_LIMIT_1_Pin);
        break;

    case 2:
        pin = HAL_GPIO_ReadPin(SW_LIMIT_2_GPIO_Port, SW_LIMIT_2_Pin);
        break;

    case 3:
        pin = HAL_GPIO_ReadPin(SW_LIMIT_3_GPIO_Port, SW_LIMIT_3_Pin);
        break;

    default:
        return HAL_ERROR;
    }

    *is_pressed = gpio_to_pressed(pin);
    //pressed:1/released:0

    return HAL_OK;
}
