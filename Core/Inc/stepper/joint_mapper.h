/*
 * joint.h
 *
 *  Created on: Mar 30, 2026
 *      Author: miyab
 */

#ifndef INC_SYNC_MOTION_H_
#define INC_SYNC_MOTION_H_

#include "main.h"
#include <stdbool.h>

//#include "stm32f4xx_hal.h"

#define JOINT_MAPPER_JOINT_COUNT   6

HAL_StatusTypeDef joint_mapper_init(void);

//HAL_StatusTypeDef joint_mapper_set_target_rad(float q1_target_rad,
//                                               float q2_target_rad,
//                                               float q3_target_rad,
//                                               float dq_max_rad_s,
//                                               float ddq_max_rad_s2);

HAL_StatusTypeDef joint_mapper_set_sts_targets(float q4_rad,
                                               float q5_rad,
                                               float q6_rad);

HAL_StatusTypeDef joint_mapper_set_target_rad(float q1_target_rad,
                                              float q2_target_rad,
                                              float q3_target_rad,
                                              float q4_target_rad,
                                              float q5_target_rad,
                                              float q6_target_rad,
                                              float dq_max_rad_s,
                                              float ddq_max_rad_s2);

HAL_StatusTypeDef joint_mapper_stop_all(void);

#endif /* INC_SYNC_MOTION_H_ */
