#ifndef _MAIN_H_
#define _MAIN_H_
#define _CRT_SECURE_NO_WARNINGS

//-------------------------------------------------------------
// Includes
//-------------------------------------------------------------
#include <XnOS.h>
#include <XnCppWrapper.h>
using namespace xn;

#include <windows.h>
#include <process.h>
#include <GL\glew.h>
#include <GL\glut.h>

#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <vector>
#include <utility>
#include <iterator>
using namespace std;

#include <opencv\cv.h>
#include <opencv\cxcore.h>
#include <opencv\highgui.h>
using namespace cv;		//cv must be put after std, (I don't know why. tantofish.)

//-------------------------------------------------------------
// Definitions
//-------------------------------------------------------------
#define PI 3.141592653589793
#define toRad(x) ((x)*0.01745329251994)
#define toDeg(x) ((x)*57.2957795130823)
#define OUT_OF_BOUNDARY(x,y) (((x) < 0) || ((y) < 0) || ((x) > IMG_W) || ((y) > IMG_H))
/*-------------------------------*/
/*       OpenGL Parameters       */
/*-------------------------------*/
#define GL_WIN_SIZE_X 640
#define GL_WIN_SIZE_Y 480
#define GL_WIN_PIVOT_X	20
#define GL_WIN_PIVOT_Y	200
#define GL_PRINT_FPS_INTERVAL 30
/* projection parameters setting */
#define _FOVY	30.0
#define _ASPECT	1.0
#define _ZNEAR	0.01
#define _ZFAR	10000.0
/* camera parameters setting */
#define _EYEX		0.0
#define _EYEY		0.0
#define _EYEZ		0.0
#define _CENTERX	0.0
#define _CENTERY	0.0
#define _CENTERZ	1.0
#define _UPX		0.0
#define _UPY		1.0
#define _UPZ		0.0
/* glClearColor() Background Color */
#define _CC_R	0.2f
#define _CC_G	0.2f
#define _CC_B	0.2f
#define _CC_A	0.2f


/*-------------------------------*/
/*       Kinect Parameters       */
/*-------------------------------*/
#define D_WIN_NAME "Kinect Depth Map"
#define I_WIN_NAME "Kinect Color Image"

#define MAX_DEPTH 10000
#define RES_VGA		// Define QVGA Resolution or VGA Resolution
#ifdef RES_QVGA
#define IMG_W	320
#define IMG_H	240
#define IMG_FPS	30		//ASUS Xtion Can Support up to 60 FPS
#define IMG_W_HALF 160
#define IMG_H_HALF 120
#endif
#ifdef RES_VGA
#define IMG_W	640
#define IMG_H	480
#define IMG_FPS	30
#define IMG_W_HALF 320
#define IMG_H_HALF 240

#endif

#define OPMODE_NORMAL	0
#define OPMODE_CROP		1
#define OPMODE_SELECT	2

//#define DEVICE_KIENCT
#ifdef DEVICE_KINECT
	#define FOCAL_LEN 575.82f
#else
	#define FOCAL_LEN 570.34f
#endif

/*-------------------------------*/
/*   Model Loader Parameters     */
/*-------------------------------*/
//#define MODEL_FILENAME "models\\full_auto_2_16_27_44_57_72_avg1.2.txt"
#define MODEL_FILENAME "models\\data3_1_36_106_121.txt"
//#define MODEL_FILENAME "models\\1.txt"

#define DRAW_MODEL_NOSE
#define AVG_NOSE
//#define KNN_AVG_NOSE
//#define SHALLOWEST_NOSE
//#define DRAW_NOSE_SMOOTH_TERM

/*-------------------------------*/
/*   Cloud Matcher Parameters    */
/*-------------------------------*/
#define ROTATE_STEP		2.0f
#define TRANSLATE_STEP	2.0f

/*-------------------------------*/
/*   Sampler Parameters		     */
/*-------------------------------*/
#define NOSE_WINDOW_X		20
#define NOSE_WINDOW_Y		10
#define START_DELAY			15
#define	SAMPLE_AREA_WIDTH	30
#define	SAMPLE_AREA_HEIGHT	30

/*-------------------------------*/
/*  Self Defined Data Structure  */
/*-------------------------------*/
class TYPose{
public:
	float yaw, pitch, roll;
	float x, y, z;
	
	TYPose():yaw(0.f), pitch(0.f), roll(0.f), x(0.f), y(0.f), z(0.f){}
	
	void set(TYPose pose){	
		yaw = pose.yaw;		pitch = pose.pitch;		roll = pose.roll;
		x = pose.x;			y = pose.y;				z = pose.z;
	}
};

#endif