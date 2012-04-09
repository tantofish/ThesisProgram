#include "TyKinectMotorPlus.h"

TYKinectMotorPlus :: TYKinectMotorPlus(bool syncTwoVeiw , bool mirroring) : TYKinect(syncTwoVeiw, mirroring){
	int i = GetNUIDeviceCount();
	motor_serial = GetNUIDeviceSerial(i-1);
	motor = CreateNUIMotor(motor_serial);
}

bool TYKinectMotorPlus :: motorUp(){
	motor_pos += 2000;
	if(motor_pos > 8000) motor_pos = 8000;
	SetNUIMotorPosition(motor,motor_pos);
	return true;
}

bool TYKinectMotorPlus :: motorDown(){
	motor_pos -= 2000;
	if(motor_pos < -8000) motor_pos = -8000;
	SetNUIMotorPosition(motor,motor_pos);
	return true;
}

bool TYKinectMotorPlus :: setLed(int color){
	SetNUIMotorLED(motor, (BYTE)color);
	return true;
}

