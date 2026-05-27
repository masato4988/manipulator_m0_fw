/*
 * app.c
 *
 *  Created on: Mar 31, 2026
 *      Author: miyab
 */


#include "app.h"

#include "stepper/stepper_hw.h"
#include "stepper/axis.h"
#include "joint_mapper.h"
#include "stepper/homing_control.h"
#include "sts_servo/sts_manager.h"
#include "pc_interface.h"

typedef struct {
	AppControlSource_t control_source;
    AppMode_t mode;
    AppHomingSeqState_t homing_seq_state;
    bool initialized;
} AppState_t;

static AppState_t g_app_state = {
	.control_source = APP_CONTROL_SOURCE_PC,
    .mode = APP_MODE_IDLE,
    .homing_seq_state = APP_HOMING_SEQ_IDLE,
    .initialized = false
};

static HAL_StatusTypeDef app_update_homing_sequence(void);
static void app_enter_error(void);

HAL_StatusTypeDef app_init(void)
{
    if (stepper_init() != HAL_OK) {
        app_enter_error();
        return HAL_ERROR;
    }

    if (axis_init() != HAL_OK) {
        app_enter_error();
        return HAL_ERROR;
    }

    if (joint_mapper_init() != HAL_OK) {
        app_enter_error();
        return HAL_ERROR;
    }

    if (homing_control_init() != HAL_OK) {
        app_enter_error();
        return HAL_ERROR;
    }
    if (sts_manager_init() != HAL_OK) {
        app_enter_error();
        return HAL_ERROR;
    }


    g_app_state.mode = APP_MODE_IDLE;
    g_app_state.homing_seq_state = APP_HOMING_SEQ_IDLE;
    g_app_state.initialized = true;

    return HAL_OK;
}

HAL_StatusTypeDef app_update(void)
{
    if (g_app_state.initialized == false) {
        return HAL_ERROR;
    }

    /*
     * PCは「操作元」なので、動作モードとは別に処理する。
     * 受信割り込みで溜まった1行コマンドをここで処理するだけ。
     */
    if (g_app_state.control_source == APP_CONTROL_SOURCE_PC) {
        pc_interface_update();
    }

    switch (g_app_state.mode) {
    case APP_MODE_IDLE:
        /*
         * 待機中。
         * PCコマンド受付は上で行っているので、ここでは何もしない。
         */
        break;

    case APP_MODE_SYNC_MOTION:
        /*
         * 将来、軌道再生などをここに入れる。
         */
        if (axis_update_all() != HAL_OK) {
            app_enter_error();
            return HAL_ERROR;
        }
        break;

    case APP_MODE_JOINT_CONTROL:
        /*
         * 関節目標に向けて動作中。
         */
        if (axis_update_all() != HAL_OK) {
            app_enter_error();
            return HAL_ERROR;
        }
        break;

    case APP_MODE_HOMING:
        /*
         * 原点復帰中。
         * PCから開始されたHOMINGでも、modeはHOMING。
         * control_sourceはPCのまま保持される。
         */
        if (app_update_homing_sequence() != HAL_OK) {
            app_enter_error();
            return HAL_ERROR;
        }

        if (homing_control_update() != HAL_OK) {
            app_enter_error();
            return HAL_ERROR;
        }
        break;

    case APP_MODE_ERROR:
        /*
         * 必要ならここで安全停止維持。
         * PCからSTATUSやRESETだけ受ける、なども可能。
         */
        break;

    default:
        app_enter_error();
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef app_start_homing_all(void)
{
    if (g_app_state.initialized == false) {
        return HAL_ERROR;
    }

    /* 通常動作中なら必要に応じて停止 */
    if (joint_mapper_stop_all() != HAL_OK) {
        app_enter_error();
        return HAL_ERROR;
    }

    homing_control_init();

    g_app_state.mode = APP_MODE_HOMING;
    g_app_state.homing_seq_state = APP_HOMING_SEQ_AXIS456_START;

    return HAL_OK;
}

HAL_StatusTypeDef app_pc_move_rad(float q1_rad,
                                  float q2_rad,
                                  float q3_rad,
                                  float q4_rad,
                                  float q5_rad,
                                  float q6_rad,
                                  float dq_max_rad_s,
                                  float ddq_max_rad_s2)
{
    if (g_app_state.initialized == false) {
        return HAL_ERROR;
    }

//    if (g_app_state.homing_seq_state != APP_HOMING_SEQ_IDLE) {
//        return HAL_ERROR;
//    }

    if (g_app_state.mode == APP_MODE_HOMING) {
        return HAL_BUSY;
    }

    if (g_app_state.mode == APP_MODE_ERROR) {
        return HAL_ERROR;
    }

    g_app_state.mode = APP_MODE_JOINT_CONTROL;

    return joint_mapper_set_target_rad(q1_rad,
                                       q2_rad,
                                       q3_rad,
                                       q4_rad,
                                       q5_rad,
                                       q6_rad,
                                       dq_max_rad_s,
                                       ddq_max_rad_s2);
}

HAL_StatusTypeDef app_set_mode_idle(void)
{
    if (g_app_state.initialized == false) {
        return HAL_ERROR;
    }

    g_app_state.mode = APP_MODE_IDLE;
    g_app_state.homing_seq_state = APP_HOMING_SEQ_IDLE;

    return HAL_OK;
}

HAL_StatusTypeDef app_set_mode_joint_control(void)
{
    if (g_app_state.initialized == false) {
        return HAL_ERROR;
    }

    if (g_app_state.mode == APP_MODE_HOMING) {
        return HAL_BUSY;
    }

    g_app_state.mode = APP_MODE_JOINT_CONTROL;
    return HAL_OK;
}

HAL_StatusTypeDef app_set_control_source_pc(void)
{
    if (g_app_state.initialized == false) {
        return HAL_ERROR;
    }

    if (g_app_state.mode == APP_MODE_HOMING) {
        return HAL_BUSY;
    }

    if (g_app_state.mode == APP_MODE_ERROR) {
        return HAL_ERROR;
    }

    g_app_state.control_source = APP_CONTROL_SOURCE_PC;
    return HAL_OK;
}

//HAL_StatusTypeDef app_set_mode_sync_motion(void)
//{
//    if (g_app_state.initialized == false) {
//        return HAL_ERROR;
//    }
//
//    if (g_app_state.mode == APP_MODE_HOMING) {
//        /* 原点復帰中は勝手に切り替えない方が安全 */
//        return HAL_BUSY;
//    }
//
//    g_app_state.mode = APP_MODE_SYNC_MOTION;
//    return HAL_OK;
//}

HAL_StatusTypeDef app_request_stop(void)
{
    if (g_app_state.initialized == false) {
        return HAL_ERROR;
    }

    if (joint_mapper_stop_all() != HAL_OK) {
        app_enter_error();
        return HAL_ERROR;
    }

    if (g_app_state.mode != APP_MODE_ERROR) {
        g_app_state.mode = APP_MODE_IDLE;
    }

    return HAL_OK;
}

HAL_StatusTypeDef app_get_mode(AppMode_t *mode)
{
    if (mode == NULL) {
        return HAL_ERROR;
    }

    *mode = g_app_state.mode;
    return HAL_OK;
}

HAL_StatusTypeDef app_get_homing_seq_state(AppHomingSeqState_t *state)
{
    if (state == NULL) {
        return HAL_ERROR;
    }

    *state = g_app_state.homing_seq_state;
    return HAL_OK;
}


//HAL_StatusTypeDef app_get_manipulator_is_homed(bool *status){
//	if(status == NULL){
//		return HAL_ERROR;
//	}
//}
//
//HAL_StatusTypeDef app_get_manipulator_is_busy(bool *status){
//
//}


static HAL_StatusTypeDef app_update_homing_sequence(void)
{
    bool done = false;

    switch (g_app_state.homing_seq_state) {

    case APP_HOMING_SEQ_IDLE:
        break;

    case APP_HOMING_SEQ_AXIS456_START:
    	if(sts_manager_start_homing_all() != HAL_OK){
    		return HAL_ERROR;
    	}
        g_app_state.homing_seq_state = APP_HOMING_SEQ_AXIS456_WAIT;
        break;

    case APP_HOMING_SEQ_AXIS456_WAIT:
    	if(sts_manager_update_homing_all() != HAL_OK){
    		return HAL_ERROR;
    	}
    	if(sts_manager_is_homing_done(&done) != HAL_OK){
    		return HAL_ERROR;
    	}
    	if(done){
    		done = false;
    		g_app_state.homing_seq_state = APP_HOMING_SEQ_AXIS1_START;
    	}
        break;

    case APP_HOMING_SEQ_AXIS1_START:
        if (homing_control_start(1) != HAL_OK) {
            g_app_state.homing_seq_state = APP_HOMING_SEQ_ERROR;
            return HAL_ERROR;
        }
        g_app_state.homing_seq_state = APP_HOMING_SEQ_AXIS1_WAIT;
        break;

    case APP_HOMING_SEQ_AXIS1_WAIT:
        if (homing_control_is_done(1, &done) != HAL_OK) {
            g_app_state.homing_seq_state = APP_HOMING_SEQ_ERROR;
            return HAL_ERROR;
        }
        if (done) {
            g_app_state.homing_seq_state = APP_HOMING_SEQ_AXIS2_START;
        }
        break;

    case APP_HOMING_SEQ_AXIS2_START:
        if (homing_control_start(2) != HAL_OK) {
            g_app_state.homing_seq_state = APP_HOMING_SEQ_ERROR;
            return HAL_ERROR;
        }
        g_app_state.homing_seq_state = APP_HOMING_SEQ_AXIS2_WAIT;
        break;

    case APP_HOMING_SEQ_AXIS2_WAIT:
        if (homing_control_is_done(2, &done) != HAL_OK) {
            g_app_state.homing_seq_state = APP_HOMING_SEQ_ERROR;
            return HAL_ERROR;
        }
        if (done) {
            g_app_state.homing_seq_state = APP_HOMING_SEQ_AXIS3_START;
        }
        break;

    case APP_HOMING_SEQ_AXIS3_START:
        if (homing_control_start(3) != HAL_OK) {
            g_app_state.homing_seq_state = APP_HOMING_SEQ_ERROR;
            return HAL_ERROR;
        }
        g_app_state.homing_seq_state = APP_HOMING_SEQ_AXIS3_WAIT;
        break;

    case APP_HOMING_SEQ_AXIS3_WAIT:
        if (homing_control_is_done(3, &done) != HAL_OK) {
            g_app_state.homing_seq_state = APP_HOMING_SEQ_ERROR;
            return HAL_ERROR;
        }
        if (done) {
            g_app_state.homing_seq_state = APP_HOMING_SEQ_DONE;
        }
        break;

    case APP_HOMING_SEQ_DONE:
        /* 原点復帰完了後はIDLEへ戻す。必要ならSYNC_MOTIONへ移行でもよい */
        g_app_state.mode = APP_MODE_IDLE;
        g_app_state.homing_seq_state = APP_HOMING_SEQ_IDLE;
        break;

    case APP_HOMING_SEQ_ERROR:
        app_enter_error();
        return HAL_ERROR;

    default:
        g_app_state.homing_seq_state = APP_HOMING_SEQ_ERROR;
        return HAL_ERROR;
    }

    return HAL_OK;
}

static void app_enter_error(void)
{
    g_app_state.mode = APP_MODE_ERROR;
    g_app_state.homing_seq_state = APP_HOMING_SEQ_ERROR;
}
