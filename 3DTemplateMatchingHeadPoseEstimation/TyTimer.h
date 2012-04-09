#ifndef _TIMER_H_
#define _TIMER_H_

#include "main.h"


class TYTimer{
private:
	LARGE_INTEGER nFreq, nBefore, nAfter;
	DWORD dwTime;
	DWORD sum, count;
	char fpsString[30];
	
public:
	TYTimer():sum(0), count(0) {
		for(int i = 0 ; i < 30 ; i++)	fpsString[i] = '\0';
	}
	void timeInit();
	void timeReportMS();
	void timeReportFPS();
	void sprintFPS();
	char* FPSstring();
};



#endif