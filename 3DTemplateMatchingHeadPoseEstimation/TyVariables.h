#ifndef _VARIABLES_H_
#define _VARIABLES_H_

#include "main.h"


/*--------------------------------*/
/*  OpenGL Variables Pack for UI  */
/*--------------------------------*/
class TYGLVariables{
public:
	
	/*
	 * for user interface : viewing parameters
	 */
	float uiYaw, uiPitch, uiRoll;
	float uiTransX, uiTransY, uiTransZ;
	float uiScale;
	float rotateStep;
	float translateStep;
	float scaleStep;
	// mouse series
	int msBtn;
	int msSta;
	int msX, msY;

	GLint winW, winH;

	TYGLVariables() : 
	uiPitch(0.f), uiYaw(0.f), uiRoll(0.f), 
	uiTransX(0.f), uiTransY(0.f), uiTransZ(0.f),
	uiScale(1.f), rotateStep(5.f), translateStep(10.f), scaleStep(0.9f), 
	msgPrintCount(0)
	{}

	void reset();

	/* Message on the GL window Related Functions*/
	bool isShowingMsg();
	void setMessage(String &msg);
	const char* getMessage();
private:
	int	msgPrintCount;
	String message;
};

class TYSysVariables{
public:
	// current pose (yaw, pitch, roll, tx, ty, tz)
	Vec6f pose;
	// vector from ObservedNose to ModelNose (Xm-Xo, Ym-Yo, Zm-Zo)
	Vec3f vecOM;
	// flag for decide do matching process or not
	bool doMatch;
	bool isPause;
	char poseText[100];

	bool doFacialExpression;

	int sampleNum;

	/* reset all variables (except doMatch)*/
	void reset();
	/* pause and continue */
	void pause();
	/* turn on or off the matching process */
	void turnOnOffMatch();
	/* return the pose string (for putting text on the window) */
	char* poseString();


	int mpcNum;		// total model point cloud num
	int mpcShowIdx;	// model point cloud show index;
	void nextMPC();

	TYSysVariables();
};

#endif