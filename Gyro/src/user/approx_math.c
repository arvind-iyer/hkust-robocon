#include "approx_math.h"

#define cos_mag_factor 10000

s16 tan_val[91] = //scale 100
{
0,2,3,5,7,9,11,12,14,16,18,19,21,23,25,27,29,31,32,34,36,38,40,42,
45,47,49,51,53,55,58,60,62,65,67,70,73,75,78,81,84,87,90,93,97,100,104,107,
111,115,119,123,128,133,138,143,148,154,160,166,173,180,188,196,205,214,225,236,248,261,275,290,
308,327,349,373,401,433,470,514,567,631,712,814,951,1143,1430,1908,2864,5729};

s16 cos_val[91] =
{
10000,9998,9994,9986,9976,9962,9945,9925,9903,9877,9848,9816,9781,9744,9703,9659,9613,9563,9511,
9455,9397,9336,9272,9205,9135,9063,8988,8910,8829,8746,8660,8572,8480,8387,8290,8192,8090,7986,
7880,7771,7660,7547,7431,7314,7193,7071,6947,6820,6691,6561,6428,6293,6157,6018,5878,5736,5592,
5446,5299,5150,5000,4848,4695,4540,4384,4226,4067,3907,3746,3584,3420,3256,3090,2924,2756,2588,
2419,2250,2079,1908,1736,1564,1392,1219,1045,872,698,523,349,175,0};

s32 int_sin(s32 x) {		//x = angle * 10

	while(x < 0)
		x += 3600;
	x = x % 3600;

	if (x > 2700) {
		return -int_sin(3600-x);
	} else if (x > 1800) {
		return -int_sin(x-1800);
	} else if (x > 900) {
		return int_sin(1800-x);
	} else {
		return cos_val[90-x/10] + x%10*cos_val[x/10]/570;
	}
}

s32 int_cos(s32 x) {		//x = angle * 10
	return int_sin(900-x);
}

s16 int_tan(s16 x) {
	u8 neg = 0;
	
	if (x < 0) {
		neg = 1;
		x = -x;
	}

	while (x < 0)
		x += 180;
	x = x % 180;
	
	if (x > 90) {
		return neg ? tan_val[180-x] : -tan_val[180-x];
	} else if (x < 90) {
		return neg ? -tan_val[x] : tan_val[x];
	} else {
		return -1;
	}
}

s16 int_arc_sin(s16 sin_val)//scaled by 10000
{
	s16 angle = 0;
	s32 pre_sin = 0;
	s32 cur_sin = 0;

	if (sin_val >= 0) {
		while (angle < 90) {
			pre_sin = cur_sin;
			cur_sin = int_sin(angle*10);
			if (sin_val <= cur_sin && sin_val >= pre_sin) break;
			angle++;
		}
	} else {
		while (angle > -90) {
			pre_sin = cur_sin;
			cur_sin = int_sin(angle*10);
			if (sin_val >= cur_sin && sin_val <= pre_sin) break;
			angle--;
		}
	}
	return angle;

}

s32 my_int_arc_sin(s32 sin_val)//scaled by 10000
{
	s32 angle=0;
	s32 head_sin= 0;
	s32 tail_sin = 10000;
	s32 mid_sin = 7071;
	s32 head = 0;
	s32 tail = 900;
	s32 mid = 450;
	
	if(sin_val == 0) return 0;

	else if (sin_val>0)
	{
		while(1)
		{
			if(sin_val<mid_sin)
			{
				tail_sin = mid_sin;
				tail = mid;
				mid = (head+mid)/2;
				mid_sin = int_sin(mid);
			}
			else if(sin_val>mid_sin)
			{
				head_sin = mid_sin;
				head = mid;
				mid = (tail+mid)/2;
				mid_sin = int_sin(mid);
			}
			else
			{
				angle = mid;
				break;
			}
			
			
			if(tail_sin == sin_val)
			{
				angle = tail;
				break;
			}
			
			else if(head_sin == sin_val)
			{
				angle = head;
				break;
			}
			else if(tail - head <2)
			{
				angle = mid;
				break;
			}
		}
		return angle;
	}
	
	else
	{
		sin_val = -sin_val;
		while(1)
		{
			if(sin_val<mid_sin)
			{
				tail_sin = mid_sin;
				tail = mid;
				mid = (head+mid)/2;
				mid_sin = int_sin(mid);
			}
			else if(sin_val>mid_sin)
			{
				head_sin = mid_sin;
				head = mid;
				mid = (tail+mid)/2;
				mid_sin = int_sin(mid);
			}
			else
			{
				angle = mid;
				break;
			}
			
			
			if(tail_sin == sin_val)
			{
				angle = tail;
				break;
			}
			
			else if(head_sin == sin_val)
			{
				angle = head;
				break;
			}
			else if(tail - head <2)
			{
				angle = mid;
				break;
			}
		}
		return -angle;
	}
}

s32 my_int_arc_cos(s32 cos_val)//scaled by 10000
{
	s32 angle=0;
	s32 head_cos= 10000;
	s32 tail_cos = 0;
	s32 mid_cos = 7071;
	s32 head = 0;
	s32 tail = 900;
	s32 mid = 450;
	
	if(cos_val == 0) return 900;

	else if (cos_val>0)
	{
		while(1)
		{
			if(cos_val>mid_cos)
			{
				tail_cos = mid_cos;
				tail = mid;
				mid = (head+mid)/2;
				mid_cos = int_cos(mid);
			}
			else if(cos_val<mid_cos)
			{
				head_cos = mid_cos;
				head = mid;
				mid = (tail+mid)/2;
				mid_cos = int_cos(mid);
			}
			else
			{
				angle = mid;
				break;
			}
			
			
			if(tail_cos == cos_val)
			{
				angle = tail;
				break;
			}
			
			else if(head_cos == cos_val)
			{
				angle = head;
				break;
			}
			else if(tail - head <2)
			{
				angle = mid;
				break;
			}
		}
		return angle;
	}
	
	else
	{
		cos_val = -cos_val;
		while(1)
		{
			if(cos_val>mid_cos)
			{
				tail_cos = mid_cos;
				tail = mid;
				mid = (head+mid)/2;
				mid_cos = int_cos(mid);
			}
			else if(cos_val<mid_cos)
			{
				head_cos = mid_cos;
				head = mid;
				mid = (tail+mid)/2;
				mid_cos = int_cos(mid);
			}
			else
			{
				angle = mid;
				break;
			}
			
			
			if(tail_cos == cos_val)
			{
				angle = tail;
				break;
			}
			
			else if(head_cos == cos_val)
			{
				angle = head;
				break;
			}
			else if(tail - head <2)
			{
				angle = mid;
				break;
			}
		}
		return 1800 - angle;
	}
}

s32 Abs(s32 M) {
	return M < 0 ? -M : M;
}

s16 int_arc_tan2(s32 y, s32 x) {

	if (x == 0) {
		if (y < 0)
			return -90;
		else if (y == 0)
			return 0;
		else
			return 90;
	} else if (y == 0) {
		return x < 0 ? 180 : 0;
	} else if (x < 0) {
		return 180+int_arc_tan((s32)100*y/x);
	} else {
		return int_arc_tan((s32)100*y/x);
	}
}


s16 int_arc_tan(s32 tan_val) {		//scaled by 100

	s16 angle = 0;
	s16 pre_tan = 0;
	s16 cur_tan = 0;

	if (tan_val >= 0) {
		while (angle < 90){
			pre_tan = cur_tan;
			cur_tan = int_tan(angle);
			if (tan_val <= cur_tan && tan_val >= pre_tan)break;
			angle++;
		}
	}
	else{
		while(angle>=-89){
			pre_tan = cur_tan;
			cur_tan = int_tan(angle);
			if (tan_val >= cur_tan && tan_val <= pre_tan)break;
			angle--;
		}
	}
	return angle;
}

s16 my_angle(s16 x) {	// make  -90< x < 90
    if (x < 0)
		while(x<0) x += 3600;
	else if(x>3599)
		while(x>3599) x -= 3600;
	return x;
}

s16 Sqrt(s32 W)
{
	u16 N = 0, i;
	u32 tmp, ttp;
	u32 M = Abs(W);
	if (M == 0)
		return 0;

	tmp = (M >> 30);
	M <<= 2;
	
	if (tmp > 1) {
		N++;
		tmp -= N;
	}

	for (i = 15; i > 0; i--) {
		N <<= 1;
		tmp <<= 2;
		tmp += (M >> 30);
		ttp = N;
		ttp = (ttp<<1)+1;
		M <<= 2;
		if (tmp >= ttp) {
			tmp -= ttp;
			N++;
		}
	}
	return N;
}

u16 Sqrt_16(u32 M) {
	u16 N = 0, i;
	u32 tmp, ttp;
	if (M == 0)
		return 0;

	tmp = (M >> 30);
	M <<= 2;
	if (tmp > 1) {
		N++;
		tmp -= N;
	}

	for (i = 15; i > 0; i--) {
		N <<= 1;
		tmp <<= 2;
		tmp += (M >> 30);
		ttp = N;
		ttp = (ttp<<1)+1;

		M <<= 2;
		if (tmp >= ttp) {
			tmp -= ttp;
			N++;
		}
	}  
	return N;
}
