#include "TyTimer.h"

void TYTimer::timeInit(){
	dwTime = 0;
	memset(&nFreq,   0x00, sizeof nFreq);
	memset(&nBefore, 0x00, sizeof nBefore);
	memset(&nAfter,  0x00, sizeof nAfter);
	QueryPerformanceFrequency(&nFreq);
	QueryPerformanceCounter(&nBefore);
};
void TYTimer::timeReportMS(){
	QueryPerformanceCounter(&nAfter);
	dwTime =(DWORD)((nAfter.QuadPart-nBefore.QuadPart)*100000/nFreq.QuadPart);
	printf("%.2f ms\n", ((double)dwTime)/100.);
};
void TYTimer::timeReportFPS(){
	QueryPerformanceCounter(&nAfter);
	dwTime =(DWORD)((nAfter.QuadPart-nBefore.QuadPart)*1000/nFreq.QuadPart);
		
	printf("%.2f FPS\n", 1000./dwTime);
};
void TYTimer::sprintFPS(){
	/*------------- FPS Calculation after --------------*/
	QueryPerformanceCounter(&nAfter);
	dwTime =(DWORD)ceil(((nAfter.QuadPart - nBefore.QuadPart)*1000.0/(double)nFreq.QuadPart));
	count++;
	sum += dwTime;
	if(count > GL_PRINT_FPS_INTERVAL){
		sum /= (float)(count);
		sprintf(fpsString,"fps: %.2f \0", 1000/(float)sum);
		count = 0;	sum = 0;
	}
};
char* TYTimer::FPSstring(){
	return fpsString;
};
