#ifndef MATH_OPS_H
#define MATH_OPS_H

#define SQRT3 		1.73205080757f
#define SQRT3_2 	0.86602540378f
#define SQRT1_3		0.57735026919f
#define PI_F 		3.14159274101f
#define TWO_PI_F 	6.28318548203f
#define PI_OVER_2_F	1.57079632679f
#define LUT_MULT	81.4873308631f

#include "math.h"

float fast_fmaxf(float x, float y);
float fast_fminf(float x, float y);
float fmaxf3(float x, float y, float z);
float fminf3(float x, float y, float z);
//float roundf(float x);
void limit_norm(float *x, float *y, float limit);
void limit(float *x, float min, float max);
int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
float sin_lut(float theta);
float cos_lut(float theta);



#endif
