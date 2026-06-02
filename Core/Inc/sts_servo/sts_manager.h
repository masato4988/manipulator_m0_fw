/*
 * sts_manager.h
 *
 *  Created on: May 3, 2026
 *      Author: miyab
 */

#ifndef INC_STS_MANAGER_H_
#define INC_STS_MANAGER_H_


#include "stm32f4xx_hal.h"
#include "sts_servo/sts3215.h"

#define STS_NUM 3

HAL_StatusTypeDef sts_manager_init(void);
HAL_StatusTypeDef sts_manager_stop_all(void);

extern sts3215_t sts_q4;
extern sts3215_t sts_q5;
extern sts3215_t sts_q6;

HAL_StatusTypeDef sts_manager_set_goal_position_all(uint16_t q4,
                                                    uint16_t q5,
                                                    uint16_t q6);

HAL_StatusTypeDef sts_manager_update_motion_all(void);
bool sts_manager_is_busy_all(void);



HAL_StatusTypeDef sts_manager_start_homing_all(void);
HAL_StatusTypeDef sts_manager_update_homing_all(void);
HAL_StatusTypeDef sts_manager_is_homing_done(bool *done);

//HAL_StatusTypeDef sts_manager_get_position(uint8_t axis);
HAL_StatusTypeDef sts_manager_get_position_all(uint16_t position_raw[STS_NUM]);



#endif /* INC_STS_MANAGER_H_ */
