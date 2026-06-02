/*
 * axis_home.c
 *
 *  Created on: Mar 30, 2026
 *      Author: miyab
 */
#include "stepper/homing_control.h"
#include "home_sw.h"
#include "system_config.h"

//#define AXIS_COUNT 3

typedef struct {
    bool is_homed;
    bool request;
    bool done;
    bool error;

    HomingState_t state;
    uint32_t state_start_ms;
    int32_t backoff_start_step;
} HomingAxisState_t;

static HomingAxisState_t g_homing_axis[AXIS_COUNT];

/* ===== 設定値 ===== */
static const AxisDir_t HOME_DIR[AXIS_COUNT] = {
    DIR_NEGATIVE,
    DIR_NEGATIVE,
    DIR_POSITIVE
};

static const float HOME_FAST_RATE_HZ[AXIS_COUNT] = {
    1500.0f, 1500.0f, 1500.0f
};

static const float HOME_SLOW_RATE_HZ[AXIS_COUNT] = {
    100.0f, 100.0f, 100.0f
};

static const int32_t HOME_BACKOFF_STEP[AXIS_COUNT] = {
    100, 50, 50
};

static const int32_t HOME_SW_OFFSET_STEP[AXIS_COUNT] = {
    0, -5000, 16000
};

#define HOME_TIMEOUT_ESCAPE_MS       3000U
#define HOME_TIMEOUT_SEARCH_FAST_MS 25000U
#define HOME_TIMEOUT_BACKOFF_MS      3000U
#define HOME_TIMEOUT_SEARCH_SLOW_MS  5000U

static bool axis_is_valid(uint8_t axis)
{
    return (axis >= 1 && axis <= AXIS_COUNT);
}

static AxisDir_t reverse_dir(AxisDir_t dir)
{
    return (dir == DIR_POSITIVE) ? DIR_NEGATIVE : DIR_POSITIVE;
}

static void homing_change_state(uint8_t axis, HomingState_t state, uint32_t now_ms)
{
    g_homing_axis[axis - 1].state = state;
    g_homing_axis[axis - 1].state_start_ms = now_ms;

//    switch(state){
//    case HOMING_STATE_IDLE:
//    	printf("HOMING_STATE_IDLE\r\n");
//    	break;
//    case HOMING_STATE_START:			//1
//    	printf("HOMING_STATE_START\r\n");
//    	break;
//    case HOMING_STATE_ESCAPE:		//2
//    	printf("HOMING_STATE_ESCAPE\r\n");
//    	break;
//    case HOMING_STATE_SEARCH_FAST:	//3
//    	printf("HOMING_STATE_SEARCH_FAST\r\n");
//    	break;
//    case HOMING_STATE_BACKOFF:	    //4
//    	printf("HOMING_STATE_BACKOFF\r\n");
//    	break;
//    case HOMING_STATE_SEARCH_SLOW:	//5
//    	printf("HOMING_STATE_SEARCH_SLOW\r\n");
//    	break;
//    case HOMING_STATE_SET_ORIGIN:	//6
//    	printf("HOMING_STATE_SET_ORIGIN\r\n");
//    	break;
//    case HOMING_STATE_DONE:			//7
//    	printf("HOMING_STATE_DONE\r\n\n");
//    	break;
//    case HOMING_STATE_ERROR:			//8
//    	printf("HOMING_STATE_ERROR\r\n");
//    	break;
//    default:
//    	break;
//
//    }
}

static bool homing_is_timeout(uint8_t axis, uint32_t now_ms, uint32_t timeout_ms)
{
    return ((now_ms - g_homing_axis[axis - 1].state_start_ms) >= timeout_ms);
}

HAL_StatusTypeDef homing_control_init(void)
{
    uint8_t i;

    for (i = 0; i < AXIS_COUNT; i++) {
        g_homing_axis[i].is_homed = false;
        g_homing_axis[i].request = false;
        g_homing_axis[i].done = false;
        g_homing_axis[i].error = false;
        g_homing_axis[i].state = HOMING_STATE_IDLE;
        g_homing_axis[i].state_start_ms = 0U;
        g_homing_axis[i].backoff_start_step = 0;
    }

    return HAL_OK;
}

HAL_StatusTypeDef homing_control_start(uint8_t axis)
{
    if (!axis_is_valid(axis)) {
        return HAL_ERROR;
    }

    if (g_homing_axis[axis - 1].state != HOMING_STATE_IDLE &&
        g_homing_axis[axis - 1].state != HOMING_STATE_DONE &&
        g_homing_axis[axis - 1].state != HOMING_STATE_ERROR) {
        return HAL_BUSY;
    }

    g_homing_axis[axis - 1].request = true;
    g_homing_axis[axis - 1].done = false;
    g_homing_axis[axis - 1].error = false;
    g_homing_axis[axis - 1].is_homed = false;

    return HAL_OK;
}

HAL_StatusTypeDef homing_control_update(void)
{
    uint8_t axis;
    uint32_t now_ms = HAL_GetTick();

    HomeSwitchState_t sw_is_pressed;
    bool is_running;
    int32_t current_step;
    int32_t moved;
    HAL_StatusTypeDef ret;

    for (axis = 1; axis <= AXIS_COUNT; axis++) {

        switch (g_homing_axis[axis - 1].state) {

        case HOMING_STATE_IDLE:
            if (g_homing_axis[axis - 1].request) {
                g_homing_axis[axis - 1].request = false;
                homing_change_state(axis, HOMING_STATE_START, now_ms);
            }
            break;

        case HOMING_STATE_START:
            ret = home_sw_read(axis, &sw_is_pressed);
            if (ret != HAL_OK) {
                g_homing_axis[axis - 1].error = true;
                homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
                break;
            }

            if (sw_is_pressed == SW_PRESSED) {
                ret = set_dir(axis, reverse_dir(HOME_DIR[axis - 1]));
                if (ret != HAL_OK) {
                    g_homing_axis[axis - 1].error = true;
                    homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
                    break;
                }

                ret = step_timer_set_rate(axis, HOME_SLOW_RATE_HZ[axis - 1]);
                if (ret != HAL_OK) {
                    g_homing_axis[axis - 1].error = true;
                    homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
                    break;
                }

                homing_change_state(axis, HOMING_STATE_ESCAPE, now_ms);
            } else {
                ret = set_dir(axis, HOME_DIR[axis - 1]);
                if (ret != HAL_OK) {
                    g_homing_axis[axis - 1].error = true;
                    homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
                    break;
                }

                ret = step_timer_set_rate(axis, HOME_FAST_RATE_HZ[axis - 1]);
                if (ret != HAL_OK) {
                    g_homing_axis[axis - 1].error = true;
                    homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
                    break;
                }

                homing_change_state(axis, HOMING_STATE_SEARCH_FAST, now_ms);
            }
            break;

        case HOMING_STATE_ESCAPE:
            ret = home_sw_read(axis, &sw_is_pressed);
            if (ret != HAL_OK) {
                g_homing_axis[axis - 1].error = true;
                homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
                break;
            }

            if (sw_is_pressed == SW_RELEASED) {
                stepper_request_stop(axis);
                ret = stepper_is_running(axis, &is_running);
                if (ret != HAL_OK) {
                    g_homing_axis[axis - 1].error = true;
                    homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
                    break;
                }

                if (!is_running) {
                    ret = set_dir(axis, HOME_DIR[axis - 1]);
                    if (ret != HAL_OK) {
                        g_homing_axis[axis - 1].error = true;
                        homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
                        break;
                    }

                    ret = step_timer_set_rate(axis, HOME_FAST_RATE_HZ[axis - 1]);
                    if (ret != HAL_OK) {
                        g_homing_axis[axis - 1].error = true;
                        homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
                        break;
                    }

                    homing_change_state(axis, HOMING_STATE_SEARCH_FAST, now_ms);
                }
            } else if (homing_is_timeout(axis, now_ms, HOME_TIMEOUT_ESCAPE_MS)) {
                stepper_request_stop(axis);
                g_homing_axis[axis - 1].error = true;
                homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
            }
            break;

        case HOMING_STATE_SEARCH_FAST:
            ret = home_sw_read(axis, &sw_is_pressed);
            if (ret != HAL_OK) {
                g_homing_axis[axis - 1].error = true;
                homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
                break;
            }

            if (sw_is_pressed == SW_PRESSED) {
                ret = stepper_get_current_step(axis, &current_step);
                if (ret != HAL_OK) {
                    g_homing_axis[axis - 1].error = true;
                    homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
                    break;
                }

                g_homing_axis[axis - 1].backoff_start_step = current_step;

                stepper_request_stop(axis);
                ret = stepper_is_running(axis, &is_running);
                if (ret != HAL_OK) {
                    g_homing_axis[axis - 1].error = true;
                    homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
                    break;
                }

                if (!is_running) {
                    ret = set_dir(axis, reverse_dir(HOME_DIR[axis - 1]));
                    if (ret != HAL_OK) {
                        g_homing_axis[axis - 1].error = true;
                        homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
                        break;
                    }

                    ret = step_timer_set_rate(axis, HOME_SLOW_RATE_HZ[axis - 1]);
                    if (ret != HAL_OK) {
                        g_homing_axis[axis - 1].error = true;
                        homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
                        break;
                    }

                    homing_change_state(axis, HOMING_STATE_BACKOFF, now_ms);
                }
            } else if (homing_is_timeout(axis, now_ms, HOME_TIMEOUT_SEARCH_FAST_MS)) {
                stepper_request_stop(axis);
                g_homing_axis[axis - 1].error = true;
                homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
            }
            break;

        case HOMING_STATE_BACKOFF:
            ret = home_sw_read(axis, &sw_is_pressed);
            if (ret != HAL_OK) {
                g_homing_axis[axis - 1].error = true;
                homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
                break;
            }

            ret = stepper_get_current_step(axis, &current_step);
            if (ret != HAL_OK) {
                g_homing_axis[axis - 1].error = true;
                homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
                break;
            }

            moved = current_step - g_homing_axis[axis - 1].backoff_start_step;
            if (moved < 0) moved = -moved;

            if ((sw_is_pressed == SW_RELEASED) && (moved >= HOME_BACKOFF_STEP[axis - 1])) {
                stepper_request_stop(axis);
                ret = stepper_is_running(axis, &is_running);
                if (ret != HAL_OK) {
                    g_homing_axis[axis - 1].error = true;
                    homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
                    break;
                }

                if (!is_running) {
                    ret = set_dir(axis, HOME_DIR[axis - 1]);
                    if (ret != HAL_OK) {
                        g_homing_axis[axis - 1].error = true;
                        homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
                        break;
                    }

                    ret = step_timer_set_rate(axis, HOME_SLOW_RATE_HZ[axis - 1]);
                    if (ret != HAL_OK) {
                        g_homing_axis[axis - 1].error = true;
                        homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
                        break;
                    }

                    homing_change_state(axis, HOMING_STATE_SEARCH_SLOW, now_ms);
                }
            } else if (homing_is_timeout(axis, now_ms, HOME_TIMEOUT_BACKOFF_MS)) {
                stepper_request_stop(axis);
                g_homing_axis[axis - 1].error = true;
                homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
            }
            break;

        case HOMING_STATE_SEARCH_SLOW:
            ret = home_sw_read(axis, &sw_is_pressed);
            if (ret != HAL_OK) {
                g_homing_axis[axis - 1].error = true;
                homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
                break;
            }

            if (sw_is_pressed == SW_PRESSED) {
                stepper_request_stop(axis);
                ret = stepper_is_running(axis, &is_running);
                if (ret != HAL_OK) {
                    g_homing_axis[axis - 1].error = true;
                    homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
                    break;
                }

                if (!is_running) {
                    homing_change_state(axis, HOMING_STATE_SET_ORIGIN, now_ms);
                }
            } else if (homing_is_timeout(axis, now_ms, HOME_TIMEOUT_SEARCH_SLOW_MS)) {
                stepper_request_stop(axis);
                g_homing_axis[axis - 1].error = true;
                homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
            }
            break;

        case HOMING_STATE_SET_ORIGIN:
            ret = stepper_set_current_step(axis, HOME_SW_OFFSET_STEP[axis - 1]);
            if (ret != HAL_OK) {
                g_homing_axis[axis - 1].error = true;
                homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
                break;
            }

            g_homing_axis[axis - 1].is_homed = true;
            g_homing_axis[axis - 1].done = true;
            g_homing_axis[axis - 1].error = false;
            homing_change_state(axis, HOMING_STATE_DONE, now_ms);
            break;

        case HOMING_STATE_DONE:
            break;

        case HOMING_STATE_ERROR:
            stepper_request_stop(axis);
			stepper_is_running(axis, &is_running);
			if(!is_running){
				Error_Handler();
			}
            break;

        default:
            stepper_request_stop(axis);
            g_homing_axis[axis - 1].error = true;
            homing_change_state(axis, HOMING_STATE_ERROR, now_ms);
            break;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef homing_control_is_busy(uint8_t axis, bool *is_busy)
{
    HomingState_t state;

    if (!axis_is_valid(axis) || is_busy == NULL) return HAL_ERROR;

    state = g_homing_axis[axis - 1].state;
    *is_busy = !(state == HOMING_STATE_IDLE ||
                 state == HOMING_STATE_DONE ||
                 state == HOMING_STATE_ERROR);
    return HAL_OK;
}

HAL_StatusTypeDef homing_control_is_done(uint8_t axis, bool *is_done)
{
    if (!axis_is_valid(axis) || is_done == NULL) return HAL_ERROR;
    *is_done = g_homing_axis[axis - 1].done;
    return HAL_OK;
}

HAL_StatusTypeDef homing_control_is_homed(uint8_t axis, bool *is_homed)
{
    if (!axis_is_valid(axis) || is_homed == NULL) return HAL_ERROR;
    *is_homed = g_homing_axis[axis - 1].is_homed;
    return HAL_OK;
}

HAL_StatusTypeDef homing_control_get_state(uint8_t axis, HomingState_t *state)
{
    if (!axis_is_valid(axis) || state == NULL) return HAL_ERROR;
    *state = g_homing_axis[axis - 1].state;
    return HAL_OK;
}
