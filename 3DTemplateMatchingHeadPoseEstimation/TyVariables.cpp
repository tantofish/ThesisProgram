#include "TyVariables.h"

void TYGLVariables::reset(){
	uiPitch = 0;	uiYaw = 0;		uiRoll = 0;
	uiTransX = 0;	uiTransY = 0;	uiTransZ = 0;
	uiScale = 1;
}



TYSysVariables::TYSysVariables(){
	pose = Vec6f(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
	vecOM = Vec3f(0.f, 0.f, 0.f);
	isPause = false;
	doMatch = false;
}

void TYSysVariables::reset(){
	pose = Vec6f(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
	vecOM = Vec3f(0.f, 0.f, 0.f);
}

void TYSysVariables::turnOnOffMatch(){
	doMatch = !doMatch;
}

char* TYSysVariables::poseString(){
	sprintf(poseText, "Yaw:%2.2f, Pitch:%2.2f, Roll:%2.2f", pose[0], pose[1], pose[2]);
	return poseText;
}

void TYSysVariables::pause(){
	isPause = !isPause;
}