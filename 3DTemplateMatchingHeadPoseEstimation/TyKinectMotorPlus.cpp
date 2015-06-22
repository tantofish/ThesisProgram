#include "TyKinectMotorPlus.h"

TYKinectMotorPlus :: TYKinectMotorPlus(bool syncTwoVeiw , bool mirroring, char* fileName){
	TYKinect(syncTwoVeiw, mirroring, fileName);
	device_count = GetNUIDeviceCount();
	motor_serial = GetNUIDeviceSerial(device_count-1);
	motor = CreateNUIMotor(motor_serial);
	motor_pos = 0;
}

bool TYKinectMotorPlus :: motorUp(){
	if(device_count != 0 && motor_serial != NULL){
		motor_pos += 2000;
		if(motor_pos > 8000) motor_pos = 8000;
		SetNUIMotorPosition(motor,motor_pos);
		return true;
	}else{
		return false;
	}
}

bool TYKinectMotorPlus :: motorDown(){
	if(device_count != 0 && motor_serial != NULL){
		motor_pos -= 2000;
		if(motor_pos < -8000) motor_pos = -8000;
		SetNUIMotorPosition(motor,motor_pos);
		return true;
	}else{
		return false;
	}
}

bool TYKinectMotorPlus :: setLed(int color){
	if(device_count != 0 && motor_serial != NULL){
		SetNUIMotorLED(motor, (BYTE)color);
		return true;
	}else{
		return false;
	}
}

