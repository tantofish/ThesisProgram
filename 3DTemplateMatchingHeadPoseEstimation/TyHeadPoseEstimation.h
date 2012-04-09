#ifndef _HPE_H_
#define _HPE_H_
#define _CRT_SECURE_NO_WARNINGS

#include "main.h"




class TYGLVariables{
public:
	
	/*
	 * for user interface : viewing parameters
	 */
	float uiPitch;
	float uiYaw;
	float uiRoll;
	float uiTransX;
	float uiTransY;
	float uiTransZ;
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
	uiScale(1.f), rotateStep(5.f), translateStep(10.f), scaleStep(0.9f)
	{}
};

/* Program start here, setup opengl pipline*/
void ProgramStart();
/* Every Frame's Routine Jobs will be executed in this function */
void myDisplay();
/* trigger when gl window is reshaped*/
void myReshape(GLint w, GLint h);
/* keyboard function parser */
void myKey(GLubyte key, GLint x, GLint y);
/* mouse function */
void myMouse(int button, int state, int x, int y);
/* mouse motion function */
void myMotion(int x, int y);
/* Variables Initialization*/
void myInitial();
/* idle function*/
void myIdle();
/* OpenGL Light Setting*/
void myLightInit();
/* OpenGL Material Setting*/
void myMaterialInit();
/* FPS calculation */
void myFPS(bool isStart);
/* print text onto GL window*/
void myGlPutText(float x, float y, char* text, LPVOID font = GLUT_BITMAP_HELVETICA_18, float r = 1.f, float g = 0.f, float b = 0.f, float a = 0.5f);



#endif