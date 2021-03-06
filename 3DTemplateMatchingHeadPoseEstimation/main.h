#ifndef _MAIN_H_
#define _MAIN_H_
#define _CRT_SECURE_NO_WARNINGS

//-------------------------------------------------------------
// Includes
//-------------------------------------------------------------

#include <flann\flann.hpp>

#include <XnOS.h>
#include <XnCppWrapper.h>
using namespace xn;


#include <windows.h>
#include <process.h>
#include <GL\glew.h>
#include <GL\glut.h>

#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <utility>
#include <iterator>

using namespace std;

#include <opencv\cxcore.h>
#include <opencv\highgui.h>
#include <opencv2\imgproc\imgproc.hpp>

using namespace cv;		//cv must be put after std, (I don't know why. tantofish.)



//-------------------------------------------------------------
// Definitions
//-------------------------------------------------------------
#define PI 3.141592653589793
#define toRad(x) ((x)*0.01745329251994)
#define toDeg(x) ((x)*57.2957795130823)
#define OUT_OF_BOUNDARY(x,y) (((x) < 0) || ((y) < 0) || ((x) > IMG_W) || ((y) > IMG_H))


//---------
// control
//---------


//#define USE_BIWI
//#define BIWI_READ_COLOR
//#define EDIT_BIWI

//#define OUTPUT_NOSE_POS
//#define OUTPUT_RESULT_PARAMS
//#define OUTPUT_ALL_WINDOW

//#define NORMAL_WHOLE_CLOUD
//#define NOSE_REST_CLOUD
//#define DRAW_NOSE_TIP_ON_2D
//#define DRAW_XYZ_AXIS
//#define DRAW_NOSE_SMOOTH_TERM
//#define PRINT_OPTIMIZATION_TIME

#define DRAW_VIRTUAL_MODEL
#define DRAW_SAMPLE_ON_2D
#define DRAW_NSW_ON_2D	// NSW: Nose Searching Window

#define MSG_SHOW_COUNT 45

/*---------------------------------*/
/* Facieal Exppression Recognition */
/*---------------------------------*/
//#define FACIAL_EXPRESSION


/*-------------------------------*/
/*   Model Loader Parameters     */
/*-------------------------------*/
//#define MODEL_FILENAME "models\\full_auto_2_16_27_44_57_72_avg1.2.txt"
//#define MODEL_FILENAME "models\\data3_1_36_106_121.txt"
#define MODEL_FILENAME "models\\1.txt"

//#define MODEL_FILENAME "models\\biwi_01.txt"
#define DB_SET 1

//#define DRAW_MODEL_NOSE
#define AVG_NOSE
//#define KNN_AVG_NOSE
//#define SHALLOWEST_NOSE

/*-----------------------*/
/*   Iterative Matcher   */
/*-----------------------*/
#define CVG_THRE_CNT 2
#define CVG_THRE_ROT 0.05f
#define CVG_THRE_EPP 2.0f
#define CVG_THRE_GoEPP 0.01f
//#else
#define ROTATE_STEP		2.0f
#define TRANSLATE_STEP	2.0f

/*-------------------------------*/
/* Smooth Filter (History Table) */
/*-------------------------------*/
#define SMOOTH_MODE_PRINTF

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
#define IMG_FPS	60		//ASUS Xtion Can Support up to 60 FPS
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

#define DEVICE_KIENCT
#ifdef DEVICE_KINECT
	#define FOCAL_LEN 575.82f
#else
	#define FOCAL_LEN 570.34f
#endif

/*-------------------------------*/
/*       OpenGL Parameters       */
/*-------------------------------*/
#define GL_WIN_SIZE_X 640
#define GL_WIN_SIZE_Y 480
#define GL_WIN_PIVOT_X	0
#define GL_WIN_PIVOT_Y	400
#define GL_PRINT_FPS_INTERVAL 30
/* projection parameters setting */
#define _FOVY	45.0
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
#define _CC_R	0.f
#define _CC_G	0.f
#define _CC_B	0.f
#define _CC_A	0.f



/*-------------------------------*/
/*   Sampler Parameters		     */
/*-------------------------------*/
#ifdef RES_QVGA
	#define NOSE_WINDOW_X		10
	#define NOSE_WINDOW_Y		5
	#define	SAMPLE_AREA_WIDTH	20
	#define	SAMPLE_AREA_HEIGHT	20
	#define	FE_SAMPLE_AREA_WIDTH	30
	#define	FE_SAMPLE_AREA_HEIGHT	30
	#define RES_FACTOR			2.f
#endif
#ifdef RES_VGA
	#define NOSE_WINDOW_X		20
	#define NOSE_WINDOW_Y		10
	#define	SAMPLE_AREA_WIDTH	40
	#define	SAMPLE_AREA_HEIGHT	40
	#define	FE_SAMPLE_AREA_WIDTH	60
	#define	FE_SAMPLE_AREA_HEIGHT	60
	#define RES_FACTOR			1.f
#endif
	#define START_DELAY			15
	#define SAMPLE_NUMBER		30

	#define SEGMENT_THRESH		50

/*-------------------*/
/*   Smooth Filter   */
/*-------------------*/
#define DYNAMIC_FRAME_WEIGHTINH
#define SMOOTH_FRAME_NUM	5
#define PRINT_FRAME_INTERVAL	30

/*-------------------------------*/
/*  Self Defined Data Structure  */
/*-------------------------------*/

/* Used for CV */
class TYMouse{
public:
	bool Lpressed;
	bool Rpressed;
	int xSt, ySt;	// x_started, y_started
	int x, y;
	TYMouse():x(0),y(0){ Lpressed = false; Rpressed = false;}
	void setCurrent(int u, int v){	x = u;	y = v;	}
	void setStarted(int u, int v){	xSt = u;	ySt = v;	}
	void reset(){x = 0; y = 0; xSt = 0; ySt = 0;}
	
};


#endif