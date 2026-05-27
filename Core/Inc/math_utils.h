/*
 * math_utils.h
 *
 *  Created on: Mar 30, 2026
 *      Author: miyab
 */

#ifndef INC_MATH_UTILS_H_
#define INC_MATH_UTILS_H_


#include <stdint.h>

#define PI (3.14159265358979f)
#define PI_2 (6.28318530717958f)

#define DEG_TO_RAD_CONST 0.017453292519943295f
#define RAD_TO_DEG_CONST 57.29577951308232f

float deg_to_rad(float deg);
float rad_to_deg(float rad);


#endif /* INC_MATH_UTILS_H_ */
