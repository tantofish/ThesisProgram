#ifndef _HPE_H_
#define _HPE_H_
#define _CRT_SECURE_NO_WARNINGS

#include "main.h"


/* Program start here, setup opengl pipline*/
void ProgramStart();
/* Every Frame's Routine Jobs will be executed in this function */
void myDisplay();
/* trigger when gl window is reshaped*/
void myReshape(GLint w, GLint h);
/* keyboard function parser */
void myKey(GLubyte key, GLint x, GLint y);
/* This Mouse Callback Function is for OpenGL */
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
void myGlPutText(float x, float y, const char* text, LPVOID font = GLUT_BITMAP_HELVETICA_18, float r = 1.f, float g = 0.f, float b = 0.f, float a = 0.5f);
/* This Callback Function is for OpenCV */
void myMouseCallBack(int event, int x, int y, int flags, void* param);


#endif