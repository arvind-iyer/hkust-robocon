#include "log.h"

const s32 log_space = LOG_SPACE;

char* strList[log_space];// = {"log", "log", "log", "log", "log", "log"};
s32 valList[log_space];// = {1,2,3,4,5,6,};

void log(char* pstr, s32 val)
{
	//free(strList[log_space-1]);	// ???
	for (s32 i=log_space-1; i>=1; --i)
	{
		strList[i] = strList[i-1];
		valList[i] = valList[i-1];
	}
	strList[0] = pstr;
	valList[0] = val;
	
}

void log_update()
{
	for (s32 i=0; i<log_space; ++i)
	{
		tft_prints(0,9-i,"               ");
		tft_prints(0, 9-i, "%s %d", strList[i],valList[i]);
	}
	
}