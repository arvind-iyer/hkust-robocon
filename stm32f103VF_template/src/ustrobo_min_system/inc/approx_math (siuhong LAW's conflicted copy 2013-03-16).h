#ifndef __APPROX_MATH_H
#define __APPROX_MATH_H

#include "stm32f10x.h"

extern s16 cos_val[91];

s32 int_sin(s32 a);
s32 int_cos(s32 a);
s16 int_tan(s16 a);

s16 int_arc_sin(s16 sin_val);
s16 int_arc_cos(s16 cos_val);
s16 int_arc_tan(s32 tan_val);
s16 int_arc_tan2(s32 y, s32 x);

s32 Abs(s32 v);
u16 Sqrt(s32 v);

#endif		/*  __APPROX_MATH_H */
