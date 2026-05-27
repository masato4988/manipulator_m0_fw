/*
 * app.h
 *
 *  Created on: Mar 31, 2026
 *      Author: miyab
 */

#ifndef INC_APP_H_
#define INC_APP_H_


#include "main.h"
#include "stdbool.h"

typedef enum {
    APP_CONTROL_SOURCE_NONE = 0,
    APP_CONTROL_SOURCE_LOCAL,
    APP_CONTROL_SOURCE_PC
} AppControlSource_t;

typedef enum {
    APP_MODE_IDLE = 0,
    APP_MODE_SYNC_MOTION,
    APP_MODE_JOINT_CONTROL,
    APP_MODE_HOMING,
    APP_MODE_ERROR
} AppMode_t;

typedef enum {
    APP_HOMING_SEQ_IDLE = 0,
    APP_HOMING_SEQ_AXIS456_START,//4,5,6
    APP_HOMING_SEQ_AXIS456_WAIT,//4,5,6
    APP_HOMING_SEQ_AXIS1_START,//1
    APP_HOMING_SEQ_AXIS1_WAIT,//2
    APP_HOMING_SEQ_AXIS2_START,//3
    APP_HOMING_SEQ_AXIS2_WAIT,//4
    APP_HOMING_SEQ_AXIS3_START,//5
    APP_HOMING_SEQ_AXIS3_WAIT,//6
    APP_HOMING_SEQ_DONE,//7
    APP_HOMING_SEQ_ERROR//8
} AppHomingSeqState_t;

HAL_StatusTypeDef app_init(void);
HAL_StatusTypeDef app_update(void);

HAL_StatusTypeDef app_start_homing_all(void);

HAL_StatusTypeDef app_pc_move_rad(float q1_rad,
                                  float q2_rad,
                                  float q3_rad,
                                  float q4_rad,
                                  float q5_rad,
                                  float q6_rad,
                                  float dq_max_rad_s,
                                  float ddq_max_rad_s2);

HAL_StatusTypeDef app_set_mode_idle(void);
HAL_StatusTypeDef app_set_mode_joint_control(void);

HAL_StatusTypeDef app_set_control_source_pc(void);
//HAL_StatusTypeDef app_set_mode_sync_motion(void);

HAL_StatusTypeDef app_request_stop(void);

HAL_StatusTypeDef app_get_mode(AppMode_t *mode);
HAL_StatusTypeDef app_get_homing_seq_state(AppHomingSeqState_t *state);

//HAL_StatusTypeDef app_get_manipulator_is_homed(bool *status);
//HAL_StatusTypeDef app_get_manipulator_is_busy(bool *status);


#endif /* INC_APP_H_ */
