#ifndef _CLNUI_LIB_
#define _CLNUI_LIB_
#define _CRT_SECURE_NO_WARNINGS


#include <CLNUIDevice.h>
#include "TyKinect.h"


#pragma comment( lib, "CLNUIDevice")

#define LED_OFF			0
#define LED_GREEN		1
#define LED_RED			2
#define LED_ORANGE		3
#define LED_G_FLICKER	4
#define LED_G_FLICKER1	5
#define LED_ORANGE_RED	6

class TYKinectMotorPlus : public TYKinect{
public:
	TYKinectMotorPlus(bool syncTwoVeiw = true, bool mirroring = true);
	bool motorUp();
	bool motorDown();
	bool setLed(int color);
private:
	CLNUIMotor	motor;
	PCHAR motor_serial;
	int motor_pos;
};

#endif