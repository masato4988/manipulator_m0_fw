/*

K_M1_TO_Q1 = 2π / 200 / 16 * (1/10)
           = 0.000196349540849362

K_M2_TO_Q2 = 2π / 200 / 16 * (40/160) * (16/64)
           = 0.000122718463030815

K_M3_TO_Q3 = 2π / 200 / 16 * (40/160) * (16/45)
           = 0.000174532925199433

K_M2_TO_Q3 = 2π / 200 / 16 * (16/45) * ((40/160) * (16/64))
           = 2π / 200 / 16 * (16/45) * (1/4) * (1/4)
           = 2π / 200 / 16 * (1/45)
           = 0.0000426332312998582

q1 = m1 * K_M1_TO_Q1
m1 = q1 / K_M1_TO_Q1

q2 = m2 * K_M2_TO_Q2
m2 = q2 / K_M2_TO_Q2

q3 = (m3 * K_M3_TO_Q3) - (m2 * K_M2_TO_Q3)
(m3 * K_M3_TO_Q3) = (m2 * K_M2_TO_Q3) + q3
m3 = (q3 + (m2 * K_M2_TO_Q3)) / K_M3_TO_Q3

*/


#include "joint_mapper.h"
#include "stepper/axis.h"
#include "stepper/stepper_hw.h"
#include "sts_servo/sts_manager.h"
#include "math_utils.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* --- 関節→モータ変換係数 [rad/step] --- */
#define K_M1_TO_Q1   (0.000196349540849362f)
#define K_M2_TO_Q2   (0.000122718463030815f)
#define K_M3_TO_Q3   (0.000174532925199433f)
#define K_M2_TO_Q3   (0.0000426332312998582f)

/* --- STS3215 設定 --- */
#define STS_Q4_ID    11
#define STS_Q5_ID    12
#define STS_Q6_ID    13

#define STS_POS_CENTER    (2048.0f)
#define STS_POS_PER_RAD   (2048.0f / (float)M_PI)
#define STS_POS_MIN       (0)
#define STS_POS_MAX       (4095)

#define DEG_TO_RAD(x) ((x) * 0.01745329252f)

typedef struct {
    float q_target_rad[JOINT_MAPPER_JOINT_COUNT];
    float dq_max_rad_s;
    float ddq_max_rad_s2;
    bool active;
} JointMapperState_t;

//typedef struct {
//    float q1_target_rad;
//    float q2_target_rad;
//    float q3_target_rad;
//    float dq_max_rad_s;
//    float ddq_max_rad_s2;
//    bool active;
//} JointMapperState_t;

static JointMapperState_t JointMapperState;

extern sts3215_t sts_q4;
extern sts3215_t sts_q5;
extern sts3215_t sts_q6;

static sts3215_t *sts_servo[3] = {
    &sts_q4,
    &sts_q5,
    &sts_q6
};

/* q4〜q6のゼロ点補正・向き補正 */
static const float sts_offset_rad[3] = {
    0.0f,
    0.0f,
    0.0f
};

static const float sts_sign[3] = {
    1.0f,
    1.0f,
    1.0f
};

//static const uint8_t sts_id[3] = {
//    STS_Q4_ID,
//    STS_Q5_ID,
//    STS_Q6_ID
//};
typedef struct {
    float min_rad;
    float max_rad;
} JointLimit_t;

static const JointLimit_t JointLimits[JOINT_MAPPER_JOINT_COUNT] = {
    { DEG_TO_RAD(-45.0f),  DEG_TO_RAD(90.0f)  }, // q1
    { DEG_TO_RAD(-30.0f),  DEG_TO_RAD(105.0f) }, // q2
    { DEG_TO_RAD(-150.0f), DEG_TO_RAD(170.0f) }, // q3
    { DEG_TO_RAD(-178.0f), DEG_TO_RAD(178.0f) }, // q4
    { DEG_TO_RAD(-90.0f),  DEG_TO_RAD(90.0f)  }, // q5
    { DEG_TO_RAD(-180.0f), DEG_TO_RAD(180.0f) }  // q6
};


/* ---------- 内部補助 ---------- */

static int32_t clamp_i32(int32_t x, int32_t min, int32_t max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

static HAL_StatusTypeDef joint_mapper_joint_to_motor_target_step(float q1_rad,
                                                                  float q2_rad,
                                                                  float q3_rad,
                                                                  int32_t *m1_step,
                                                                  int32_t *m2_step,
                                                                  int32_t *m3_step)
{
    float m3_f;

    if (m1_step == NULL || m2_step == NULL || m3_step == NULL) {
        return HAL_ERROR;
    }

    /*
     * q1 = m1 * K_M1_TO_Q1
     * q2 = m2 * K_M2_TO_Q2
     * q3 = m3 * K_M3_TO_Q3 - m2 * K_M2_TO_Q3
     */
    *m1_step = (int32_t)lroundf(q1_rad / K_M1_TO_Q1);
    *m2_step = (int32_t)lroundf(q2_rad / K_M2_TO_Q2);

    m3_f = (q3_rad + ((float)(*m2_step) * K_M2_TO_Q3)) / K_M3_TO_Q3;
    *m3_step = (int32_t)lroundf(m3_f);

    return HAL_OK;
}

static HAL_StatusTypeDef joint_mapper_joint_rate_to_motor_rate(float dq_max_rad_s,
                                                                float ddq_max_rad_s2,
                                                                float *m1_rate,
                                                                float *m2_rate,
                                                                float *m3_rate,
                                                                float *m1_acc,
                                                                float *m2_acc,
                                                                float *m3_acc)
{
    float dm2_rate;
    float dm2_acc;

    if (m1_rate == NULL || m2_rate == NULL || m3_rate == NULL ||
        m1_acc == NULL || m2_acc == NULL || m3_acc == NULL) {
        return HAL_ERROR;
    }

    /*
     * 各関節の共通最大速度/加速度を、
     * 各モータstep/s, step/s^2へ変換する。
     *
     * ここでは同期のための再配分はしない。
     */
    *m1_rate = fabsf(dq_max_rad_s / K_M1_TO_Q1);
    *m2_rate = fabsf(dq_max_rad_s / K_M2_TO_Q2);

    dm2_rate = *m2_rate;
    *m3_rate = fabsf((fabsf(dq_max_rad_s) + fabsf(K_M2_TO_Q3) * dm2_rate)
                     / K_M3_TO_Q3);

    *m1_acc = fabsf(ddq_max_rad_s2 / K_M1_TO_Q1);
    *m2_acc = fabsf(ddq_max_rad_s2 / K_M2_TO_Q2);

    dm2_acc = *m2_acc;
    *m3_acc = fabsf((fabsf(ddq_max_rad_s2) + fabsf(K_M2_TO_Q3) * dm2_acc)
                    / K_M3_TO_Q3);

    return HAL_OK;
}

static HAL_StatusTypeDef joint_mapper_sts_status_to_hal(sts_status_t status)
{
    if (status == STS_OK) {
        return HAL_OK;
    }

    return HAL_ERROR;
}

static HAL_StatusTypeDef joint_mapper_sts_rad_to_pos(uint8_t index,
                                                      float q_rad,
                                                      uint16_t *position)
{
    float servo_rad;
    int32_t raw_pos;

    if (index >= 3) {
        return HAL_ERROR;
    }

    if (position == NULL) {
        return HAL_ERROR;
    }

    servo_rad = sts_sign[index] * q_rad + sts_offset_rad[index];

    raw_pos = (int32_t)lroundf(STS_POS_CENTER + (servo_rad * STS_POS_PER_RAD));
    raw_pos = clamp_i32(raw_pos, STS_POS_MIN, STS_POS_MAX);

    *position = (uint16_t)raw_pos;

    return HAL_OK;
}

static HAL_StatusTypeDef joint_mapper_sts_pos_to_rad(uint8_t index,
														uint16_t position,
														float *q_rad)
{
    float servo_rad;

    if (index >= 3 || q_rad == NULL) {
        return HAL_ERROR;
    }

    servo_rad =
        ((float)((int32_t)position - STS_POS_CENTER))
        / STS_POS_PER_RAD;

    *q_rad =
        (servo_rad - sts_offset_rad[index])
        / sts_sign[index];

    return HAL_OK;
}



static HAL_StatusTypeDef joint_mapper_check_joint_limit_array(const float q_rad[JOINT_MAPPER_JOINT_COUNT])
{
	for (int i = 0; i < JOINT_MAPPER_JOINT_COUNT; i++) {

	    if (q_rad[i] < JointLimits[i].min_rad) {
	        return HAL_ERROR;
	    }

	    if (q_rad[i] > JointLimits[i].max_rad) {
	        return HAL_ERROR;
	    }
	}

    return HAL_OK;
}

/* ---------- 公開関数 ---------- */

HAL_StatusTypeDef joint_mapper_set_sts_targets(	float q4_rad,
												float q5_rad,
												float q6_rad)
{
    uint16_t pos[3];

    if (joint_mapper_sts_rad_to_pos(0, q4_rad, &pos[0]) != HAL_OK) {
        return HAL_ERROR;
    }

    if (joint_mapper_sts_rad_to_pos(1, q5_rad, &pos[1]) != HAL_OK) {
        return HAL_ERROR;
    }

    if (joint_mapper_sts_rad_to_pos(2, q6_rad, &pos[2]) != HAL_OK) {
        return HAL_ERROR;
    }

    if (sts_manager_set_goal_position_all( pos[0], pos[1], pos[2]) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef joint_mapper_init(void)
{
    uint8_t i;

    for (i = 0; i < JOINT_MAPPER_JOINT_COUNT; i++) {
        JointMapperState.q_target_rad[i] = 0.0f;
    }

    JointMapperState.dq_max_rad_s = 0.0f;
    JointMapperState.ddq_max_rad_s2 = 0.0f;
    JointMapperState.active = false;

    return HAL_OK;
}

HAL_StatusTypeDef joint_mapper_set_target_rad(float q1_target_rad,
                                               float q2_target_rad,
                                               float q3_target_rad,
                                               float q4_target_rad,
                                               float q5_target_rad,
                                               float q6_target_rad,
                                               float dq_max_rad_s,
                                               float ddq_max_rad_s2)
{
	float q_rad[JOINT_MAPPER_JOINT_COUNT];

	int32_t target_m1;
    int32_t target_m2;
    int32_t target_m3;

    float m1_rate;
    float m2_rate;
    float m3_rate;

    float m1_acc;
    float m2_acc;
    float m3_acc;

	if (dq_max_rad_s <= 0.0f) {
        return HAL_ERROR;
    }

    if (ddq_max_rad_s2 <= 0.0f) {
        return HAL_ERROR;
    }


	q_rad[0] = q1_target_rad;
	q_rad[1] = q2_target_rad;
	q_rad[2] = q3_target_rad;
	q_rad[3] = q4_target_rad;
	q_rad[4] = q5_target_rad;
	q_rad[5] = q6_target_rad;


    if (joint_mapper_check_joint_limit_array(q_rad) != HAL_OK) {
        return HAL_ERROR;
    }

    JointMapperState.q_target_rad[0] = q1_target_rad;
    JointMapperState.q_target_rad[1] = q2_target_rad;
    JointMapperState.q_target_rad[2] = q3_target_rad;
    JointMapperState.q_target_rad[3] = q4_target_rad;
    JointMapperState.q_target_rad[4] = q5_target_rad;
    JointMapperState.q_target_rad[5] = q6_target_rad;
    JointMapperState.dq_max_rad_s = dq_max_rad_s;
    JointMapperState.ddq_max_rad_s2 = ddq_max_rad_s2;
    JointMapperState.active = true;

    /*
     * stepper 3軸
     */
    if (joint_mapper_joint_to_motor_target_step(q1_target_rad,
                                                 q2_target_rad,
                                                 q3_target_rad,
                                                 &target_m1,
                                                 &target_m2,
                                                 &target_m3) != HAL_OK) {
        return HAL_ERROR;
    }

    if (joint_mapper_joint_rate_to_motor_rate(dq_max_rad_s,
                                               ddq_max_rad_s2,
                                               &m1_rate,
                                               &m2_rate,
                                               &m3_rate,
                                               &m1_acc,
                                               &m2_acc,
                                               &m3_acc) != HAL_OK) {
        return HAL_ERROR;
    }

    if (axis_set_motion_target(1, target_m1, m1_rate, m1_acc) != HAL_OK) {
        return HAL_ERROR;
    }

    if (axis_set_motion_target(2, target_m2, m2_rate, m2_acc) != HAL_OK) {
        return HAL_ERROR;
    }

    if (axis_set_motion_target(3, target_m3, m3_rate, m3_acc) != HAL_OK) {
        return HAL_ERROR;
    }

    /*
     * STS3215 3軸
     */
    if (joint_mapper_set_sts_targets(q4_target_rad,
                                     q5_target_rad,
                                     q6_target_rad) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef joint_mapper_stop_all(void)
{
    if (axis_stop(1) != HAL_OK) return HAL_ERROR;
    if (axis_stop(2) != HAL_OK) return HAL_ERROR;
    if (axis_stop(3) != HAL_OK) return HAL_ERROR;

    /*
     * ここは既存のSTS3215 API名に置き換える。
     * STS側にstopがない場合は、現在位置を読んでその位置を目標にする方法でもよい。
     */
    if (sts3215_stop(sts_servo[0]) != STS_OK) return HAL_ERROR;
    if (sts3215_stop(sts_servo[1]) != STS_OK) return HAL_ERROR;
    if (sts3215_stop(sts_servo[2]) != STS_OK) return HAL_ERROR;

    JointMapperState.active = false;
    JointMapperState.dq_max_rad_s = 0.0f;
    JointMapperState.ddq_max_rad_s2 = 0.0f;

    return HAL_OK;
}

HAL_StatusTypeDef joint_mapper_get_rad_all(float q_rad[JOINT_MAPPER_JOINT_COUNT]){
	//stepper
    int32_t step1;
    int32_t step2;
	int32_t step3;

    uint16_t position_raw[STS_NUM];

    if(stepper_get_current_step(1, &step1) != HAL_OK) return HAL_ERROR;
    if(stepper_get_current_step(2, &step2) != HAL_OK) return HAL_ERROR;
    if(stepper_get_current_step(3, &step3) != HAL_OK) return HAL_ERROR;

    q_rad[0] = step1 * K_M1_TO_Q1;
	q_rad[1] = step2 * K_M2_TO_Q2;
	q_rad[2] = (step3 * K_M3_TO_Q3) - (step2 * K_M2_TO_Q3);

	//sts
	if(sts_manager_get_position_all(position_raw) != HAL_OK) return HAL_ERROR;
//	q_rad[3] = ((float)position_raw[0] / STS3215_POSITION_RESOLUTION) * PI_2;
//	q_rad[4] = ((float)position_raw[1] / STS3215_POSITION_RESOLUTION) * PI_2;
//	q_rad[5] = ((float)position_raw[2] / STS3215_POSITION_RESOLUTION) * PI_2;

	if (joint_mapper_sts_pos_to_rad(0, position_raw[0], &q_rad[3]) != HAL_OK)
	    return HAL_ERROR;

	if (joint_mapper_sts_pos_to_rad(1, position_raw[1], &q_rad[4]) != HAL_OK)
	    return HAL_ERROR;

	if (joint_mapper_sts_pos_to_rad(2, position_raw[2], &q_rad[5]) != HAL_OK)
	    return HAL_ERROR;

	return HAL_OK;
}

bool joint_mapper_is_busy_all(void)
{
    bool is_busy_1 = false;
    bool is_busy_2 = false;
    bool is_busy_3 = false;

    if (axis_is_busy(1, &is_busy_1) != HAL_OK) return true;
    if (axis_is_busy(2, &is_busy_2) != HAL_OK) return true;
    if (axis_is_busy(3, &is_busy_3) != HAL_OK) return true;

    if (is_busy_1) return true;
    if (is_busy_2) return true;
    if (is_busy_3) return true;

    if (sts_manager_is_busy_all()) return true;

    return false;
}
