#ifndef __APPROX_MATH_H
#define __APPROX_MATH_H

#include <inttypes.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f10x.h"

	
extern s16 cos_val[91];
extern s16 cos_square_val[91];

s32 int_cos(s32);
s32 int_sin(s32);
s16 int_tan(s16 x);

s16 int_arc_tan2(s32 y, s32 x);
s16 int_arc_tan(s32 tan_val);
s16 int_arc_sin_square(s16 sin_val);
s16 int_arc_sin(s16 sin_val);
s32 my_int_arc_sin(s32 sin_val);
s32 my_int_arc_cos(s32 cos_val);
s16 int_sin_square(s16 x);
s16 int_cos_square(s16 x);
s16 my_angle(s16 x);
s16 Sqrt(s32 M);
u16 Sqrt_16(u32 W);
s32 Abs(s32 M);

#endif		/*  __APPROX_MATH_H */
