#include "TyHeadPoseEstimation.h"
#include "TyTimer.h"
#include "TyKinectMotorPlus.h"
#include "TyModelLoader.h"
#include "TySampler.h"
#include "TyCloudMatcher.h"
#include "TyCloudDrawer.h"
#include "TyBiwiReader.h"
#include "TyVariables.h"
#include "TyDemoModel.h"
#include "TySmoothFilter.h"

TYGLVariables glVars;
TYSysVariables sysVars;

TYTimer myTimerGL;
//TYKinect kinect(true, true, "female.oni");
TYKinect kinect(true, true, "alice.oni");
TYBiwiDB biwi("C:\\Users\\Tantofish\\Master\\kinect_head_pose_db");
TYHeadModel myHead;
TYSampler sampler;
TYCloudMatcher matcher;

TYCloudDrawer modelDrawer;
TYCloudDrawer observeDrawer;


TYDemoModel dmModel;
TYSmoothFilter sfilter;

TYMouse mouse;


float nFrames = 0;
float under5cnt = 0;
float under10cnt = 0;
float under15cnt = 0;
float under20cnt = 0;

void ProgramStart(){
	printf("Head Motion Tracking Progeam Started!!! \n");
	glutInitDisplayMode(GLUT_RGB | GL_DOUBLE);
	// Window 1
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutInitWindowPosition(GL_WIN_PIVOT_X, GL_WIN_PIVOT_Y);
	int WindowNum1 = glutCreateWindow("Head Motion Tracking");
	myInitial();  
#ifndef EDIT_BIWI
	glutIdleFunc(myIdle);
#endif
	glutReshapeFunc(myReshape);
	glutDisplayFunc(myDisplay);
	glutKeyboardFunc(myKey);
	glutMouseFunc(myMouse);
	glutMotionFunc(myMotion);
	glutMainLoop();
}

int writeIdx = 1;

void writeOutAllWindows(){
#ifdef OUTPUT_ALL_WINDOW
	int f = biwi.frame();
	if( (f%15)!=0 || writeIdx > 30 ) return;
	char fName[50];

	int glW = glutGet(GLUT_WINDOW_WIDTH);
	int glH = glutGet(GLUT_WINDOW_HEIGHT);
	Mat src(glH, glW, CV_8UC3);
	Mat flipped;
	glReadPixels(0, 0, glW, glH, GL_BGR, GL_UNSIGNED_BYTE, (void*) src.data);
	cv::flip(src, flipped, 0);
	sprintf(fName, "./output/gl_%d.png", writeIdx);
	imwrite(fName, flipped);

	sprintf(fName, "./output/depth_%d.png", writeIdx);
	Mat dROI(biwi.DepthRGB, Rect(237, 159, 230, 220));
	imwrite(fName,dROI);
	sprintf(fName, "./output/color_%d.png", writeIdx);
	Mat cROI(biwi.ColorRGB, Rect(237, 159, 230, 220));
	imwrite(fName,cROI);

	
	writeIdx++;
#endif
}

inline void drawXYZAxis(){
#ifdef DRAW_XYZ_AXIS
	glPushMatrix(); 
		glTranslatef(0,-10,120); 
		glPushMatrix();
			glColor3f(1,0,0); 
			glRotatef(-90,0,1,0);
			gluCylinder(gluNewQuadric(),5,5,150,6,2); 
		glPopMatrix();
		glPushMatrix();
			glColor3f(0,1,0); 
			glRotatef(-90,1,0,0);
			gluCylinder(gluNewQuadric(),5,5,200,6,2); 
		glPopMatrix();
		glPushMatrix();
			glColor3f(0,0,1); 
			glRotatef(180,0,1,0);
			gluCylinder(gluNewQuadric(),5,5,200,6,2); 
		glPopMatrix();
	glPopMatrix();
#endif
}
inline void drawVirtualCharacter(){
	/*------------------------*/
	/* draw virtual character */
	/*------------------------*/
	float horiOffset = -100.f;

	glPushMatrix();
		/* Horizontal Offset */
		glTranslatef(horiOffset, 0.f, 0.f);
		/* User Interface Translation */
		glTranslatef(glVars.uiTransX, glVars.uiTransY, glVars.uiTransZ);
		/* Put Model to Point3f(myHead.nose) */
		glTranslatef(myHead.nose.x, myHead.nose.y, myHead.nose.z);
		/* Calibrate the rotate angle error caused by the horizontal offset */
		glRotatef((atan(horiOffset/myHead.nose.z))*180./PI,	  0, 1, 0);
		/* User Interface Rotation & Scale */
		glRotatef(glVars.uiPitch, 1, 0, 0);
		glRotatef(glVars.uiYaw  , 0, 1, 0);
		glRotatef(glVars.uiRoll , 0, 0, 1);
		glScalef(glVars.uiScale, glVars.uiScale, glVars.uiScale);
		/* Estimated Translation */
		glTranslatef(	-sysVars.pose[3]-sysVars.vecOM[0],
						-sysVars.pose[4]-sysVars.vecOM[1], 
						-sysVars.pose[5]-sysVars.vecOM[2]);	// x, y, z
		/* Estimated Rotation */
		glRotatef(-sysVars.pose[0], 0, 1, 0);	// yaw
		glRotatef(-sysVars.pose[1], 1, 0, 0);	// pitch
		glRotatef(-sysVars.pose[2], 0, 0, 1);	// roll

		glEnable(GL_LIGHTING);
		drawXYZAxis();
		dmModel.draw();
	glPopMatrix();
}
inline void drawSampleCloud(){
	/* Draw Sample Point Cloud */
	float horiOffset = 100.f;
	glPushMatrix();
		/* Horizontal Offset */
		glTranslatef(horiOffset, 0.f, 0.f);
		/* User Interface Translation */
		glTranslatef(glVars.uiTransX, glVars.uiTransY, glVars.uiTransZ);
		/* Put Model to Point3f(myHead.nose) */
		glTranslatef(myHead.nose.x, myHead.nose.y, myHead.nose.z);
		/* Calibrate the rotate angle error caused by the horizontal offset */
		glRotatef((atan(horiOffset/myHead.nose.z))*180./PI,	  0, 1, 0);
		/* User Interface Rotation & Scale */
		glRotatef(glVars.uiPitch, 1, 0, 0);
		glRotatef(glVars.uiYaw  , 0, 1, 0);
		glRotatef(glVars.uiRoll , 0, 0, 1);
		glScalef(glVars.uiScale, glVars.uiScale, glVars.uiScale);
		/* Put Model to Point3f(0,0,0) */
		glTranslatef(-myHead.nose.x, -myHead.nose.y, -myHead.nose.z);

		sampler.drawSamples3f();
		#ifdef USE_BIWI
		glPushMatrix();
			glColor3f(0, 1.0, 0);
			glTranslatef(biwi.nose.x, biwi.nose.y, biwi.nose.z);
			glutSolidSphere (5,5,5);	
		glPopMatrix();
		#endif
	glPopMatrix();
}
inline void drawHeadModelCloud(){
	glDisable(GL_LIGHTING);
	/* draw head model with the estimated motion parameters */
	glPushMatrix();
		float horiOffset = 100.f;
		/* Horizontal Offset */
		glTranslatef(horiOffset, 0.f, 0.f);
		/* User Interface Translation */
		glTranslatef(glVars.uiTransX, glVars.uiTransY, glVars.uiTransZ);
		/* Put Model to Point3f(myHead.nose) */
		glTranslatef(myHead.nose.x, myHead.nose.y, myHead.nose.z);
		/* Calibrate the rotate angle error caused by the horizontal offset */
		glRotatef((atan(horiOffset/myHead.nose.z))*180./PI,	  0, 1, 0);
		/* User Interface Rotation & Scale */
		glRotatef(glVars.uiPitch, 1, 0, 0);
		glRotatef(glVars.uiYaw  , 0, 1, 0);
		glRotatef(glVars.uiRoll , 0, 0, 1);
		glScalef(glVars.uiScale, glVars.uiScale, glVars.uiScale);
		/* Estimated Translation */
		glTranslatef(	-sysVars.pose[3]-sysVars.vecOM[0],
						-sysVars.pose[4]-sysVars.vecOM[1], 
						-sysVars.pose[5]-sysVars.vecOM[2]);	// x, y, z
		/* Estimated Rotation */
		glRotatef(-sysVars.pose[0], 0, 1, 0);	// yaw
		glRotatef(-sysVars.pose[1], 1, 0, 0);	// pitch
		glRotatef(-sysVars.pose[2], 0, 0, 1);	// roll
		/* Put Model to Point3f(0,0,0) */
		glTranslatef(-myHead.nose.x, -myHead.nose.y, -myHead.nose.z);

		myHead.drawNose();	// Nose Vertex
#ifdef EDIT_BIWI
		modelDrawer.getHistogram(biwi.point);
		modelDrawer.drawPointCloud(biwi.point);
#else
		modelDrawer.drawPointCloud(myHead.pointCloud);	// Head Point Cloud
#endif
	glPopMatrix();
}
inline void drawSamplePixels(){
#ifdef USE_BIWI
	sampler.drawSamples2i(biwi.DepthRGB);
	biwi.Show();
	int key = waitKey(1);
	biwi.RegisterKey(key);

	if(biwi.isFirstFrame){
		biwi.isFirstFrame = false;
		sysVars.pause();
	}
#else
	sampler.drawSamples2i(kinect.DepthRGB);
	kinect.Show(true, false);
	int key = waitKey(1);
	kinect.RegisterKey(key);

	if(kinect.isFirstFrame){
		kinect.isFirstFrame = false;
		//sysVars.pause();
	}
#endif
}

void myDisplay(){
	
	myTimerGL.timeInit();		// time calculation head
	
	cv::setMouseCallback(D_WIN_NAME, myMouseCallBack);

	////////////////////////////////////////
	if(matcher.isConverged){
	////////////////////////////////////////
	
	if(!sysVars.isPause){
#ifdef USE_BIWI
		/* Database Input: use biwi database */
		biwi.getNextFrame();
#ifdef EDIT_BIWI
		biwi.genPC();
#endif
#else
		/* Sensor Input: Kinect Grab One Frame Set <Depth, RGB> */
		XnStatus kinectStatus = kinect.GetMetaData();

		if(kinectStatus == XN_STATUS_OK){
			kinect.GetCvFormatImages();
		}
#endif

	
	/*
	 *	Program Body
	 */

	#ifdef USE_BIWI
		sampler.randomSampling(biwi.DepthRAW, sysVars.pose, sysVars.sampleNum);
	#else
		sampler.randomSampling(kinect.DepthRAW, sysVars.pose, sysVars.sampleNum);
	#endif

		/*  */
		if(sysVars.doMatch)	matcher.match(sysVars.pose, sysVars.vecOM, sampler.sample3f, sampler.nose3f);
	
		/* History Table : Using Smooth Filter */
		sysVars.pose = sfilter.smoothing(sysVars.pose);
	}

	////////////////////////////////////////
	}
	////////////////////////////////////////
	
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glClearColor(_CC_R, _CC_G, _CC_B, _CC_A);
	glPushMatrix();

	
	drawHeadModelCloud();
	drawSampleCloud();
	drawSamplePixels();
	writeOutAllWindows();
	
	//glPushMatrix();
	//	float horiOffset = 0.f;
	//	glTranslatef(glVars.uiTransX, glVars.uiTransY, glVars.uiTransZ);
	//	glTranslatef(horiOffset, 0.f, 0.f);
	//	glTranslatef(myHead.nose.x, myHead.nose.y, myHead.nose.z);
	//	glRotatef((atan(horiOffset/myHead.nose.z))*180./PI,	  0, 1, 0);
	//	glRotatef(glVars.uiPitch, 1, 0, 0);
	//	glRotatef(glVars.uiYaw  , 0, 1, 0);
	//	glRotatef(glVars.uiRoll , 0, 0, 1);
	//	glScalef(glVars.uiScale, glVars.uiScale, glVars.uiScale);
	//	glTranslatef(-myHead.nose.x, -myHead.nose.y, -myHead.nose.z);
	//	//observeDrawer.getHistogram(sampler.noseSmoothSet3f);
	//	//observeDrawer.drawPointCloudGreen(sampler.noseSmoothSet3f);
	//	//observeDrawer.getHistogram(sampler.irCloud);
	//	//observeDrawer.drawPointCloudRed(sampler.irCloud);
	//	//observeDrawer.getHistogram(sampler.restIrCloud);
	//	//observeDrawer.drawPointCloud(sampler.restIrCloud);
	//	glDisable(GL_LIGHTING);
	//	observeDrawer.getHistogram(sampler.wholeOrCloud);
	//	observeDrawer.drawPointCloud(sampler.wholeOrCloud);
	//glPopMatrix();
	
	glDisable(GL_LIGHTING);
	/* Put FPS String Onto The GL Window */
	myGlPutText(-0.95f, -0.95f, myTimerGL.FPSstring(), GLUT_BITMAP_HELVETICA_18, 1.0f, 0.0f, 0.0f, 1.0f);
	myGlPutText(-0.95f, 0.9f, sysVars.poseString(), GLUT_BITMAP_HELVETICA_18, 1.0f, 1.0f, 0.0f, 1.0f);
	if(glVars.isShowingMsg())
		myGlPutText(-0.8f, -0.8f, glVars.getMessage(), GLUT_BITMAP_HELVETICA_18, 0.8f, 0.8f, 0.8f, 1.0f);

	drawVirtualCharacter();

#ifdef USE_BIWI
	/* accuracy computing */
	float Ye = sysVars.pose[0];	float Ygt = biwi.pose[0];
	float Pe = sysVars.pose[1]; float Pgt = biwi.pose[1];
	float Re = -sysVars.pose[2]; float Rgt = biwi.pose[2];
	//printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n(%.2f,%.2f,%,2f)\n",Ye,Pe,Re);
	Point3f pp = (sampler.nose3f - biwi.nose);
	
#ifdef OUTPUT_NOSE_POS
	ofstream file("./output/nose.txt", ofstream::app);
	file << biwi.nose.x << '\t' << biwi.nose.y << '\t' << biwi.nose.z << endl;
	file.close();
#endif
#ifdef OUTPUT_RESULT_PARAMS
	ofstream file("./output/params.txt", ofstream::app);
	file << sysVars.pose[0] << '\t' << sysVars.pose[1] << '\t' << sysVars.pose[2] << '\t';
	file << biwi.pose[0] << '\t' << biwi.pose[1] << '\t' << biwi.pose[2] << endl;
	file.close();
#endif
	

	float noseErr = sqrt(pp.x*pp.x+pp.y*pp.y+pp.z*pp.z);
	float angularErr = sqrt((Ye-Ygt)*(Ye-Ygt) + (Re-Rgt)*(Re-Rgt) + (Pe-Pgt)*(Pe-Pgt));
	float angularErrNoRoll = sqrt((Ye-Ygt)*(Ye-Ygt)+(Pe-Pgt)*(Pe-Pgt));
	char noseErrMsg[100], angErrMsg[100], angErrNoRollMsg[100];
	sprintf(noseErrMsg,		 "Nose Position Error : %.2f (mm)", noseErr);
	sprintf(angErrMsg,		 "Angular Error : %.2f ", angularErr);
	sprintf(angErrNoRollMsg, "Angular Error (No Roll): %.2f ", angularErrNoRoll);
	myGlPutText(-0.95f, 0.85f, noseErrMsg, GLUT_BITMAP_HELVETICA_12, 0.0f, 1.0f, 1.0f, 1.0f);
	myGlPutText(-0.95f, 0.80f, angErrMsg, GLUT_BITMAP_HELVETICA_12, 0.0f, 1.0f, 1.0f, 1.0f);
	myGlPutText(-0.95f, 0.75f, angErrNoRollMsg, GLUT_BITMAP_HELVETICA_12, 0.0f, 1.0f, 1.0f, 1.0f);

	if(!sysVars.isPause){
		if(angularErrNoRoll < 5)	under5cnt++;
		if(angularErrNoRoll < 10)	under10cnt++;
		if(angularErrNoRoll < 15)	under15cnt++;
		if(angularErrNoRoll < 20)	under20cnt++;
		nFrames++;
		printf("Acc(%.2f, %.2f, %.2f, %.2f)\n", under5cnt/nFrames, under10cnt/nFrames, under15cnt/nFrames, under20cnt/nFrames);
		//printf("AErr = %.2f, AErrNoRoll = %.2f, NErr = %.2f \n", angularErr, angularErrNoRoll, noseErr);
		if(angularErr < 0 || angularErr > 1000)	system("pause");
	}
#endif

	glShadeModel(GL_SMOOTH);
	glUseProgram(0);

	glPopMatrix();
	glutSwapBuffers();
	myTimerGL.sprintFPS();		// time calculation tail
}

void materialInit()
{
	float matA[]={0.15,0.15,0.15,1};
	float matD[]={0.6,0.6,0.6,1};
	float matS[]={1,1,1,1};
	glEnable(GL_NORMALIZE);

	glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,30);
	glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,matA);
	glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,matD);
	glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,matS);

	glColorMaterial(GL_FRONT_AND_BACK,GL_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
};

void lightInit()
{
	float LightPos[4];
	int i = 0;
	float ambient[4],diffuse[4],specular[4];
	float cAtt,lAtt,qAtt;
	float r,alpha,beta;

	glEnable(GL_LIGHTING);

	for(int j=0;j<3;j++){
		ambient[j]=0.1;
		diffuse[j]=0.8;
		specular[j]=0.3;
	}
	ambient[3]=diffuse[3]=specular[3]=1;

	r=1000;
	cAtt=1;
	lAtt=0.00001;
	qAtt=0.000000001;
	alpha = 0;
	beta = 0;

	LightPos[0]=r*cos(toRad(alpha))*cos(toRad(beta));
	LightPos[1]=r*cos(toRad(alpha))*sin(toRad(beta));
	LightPos[2]=r*sin(toRad(alpha));
	LightPos[3]=1;

	LightPos[0]=200;
	LightPos[1]=200;
	LightPos[2]=0;
	LightPos[3]=1;

	glLightfv(GL_LIGHT0,GL_AMBIENT,ambient);
	glLightfv(GL_LIGHT0,GL_DIFFUSE,diffuse);
	glLightfv(GL_LIGHT0,GL_SPECULAR,specular);
	glLightfv(GL_LIGHT0,GL_POSITION,LightPos);

	glLightf(GL_LIGHT0,GL_CONSTANT_ATTENUATION,cAtt);
	glLightf(GL_LIGHT0,GL_LINEAR_ATTENUATION,lAtt);
	glLightf(GL_LIGHT0,GL_QUADRATIC_ATTENUATION,qAtt);

	glEnable(GL_LIGHT0);
	LightPos[0] *= -1;
	LightPos[1] *= -1;
	for(int j=0;j<3;j++){
		ambient[j]=0.3;
		diffuse[j]=0.3;
		specular[j]=0.3;
	}
	glLightfv(GL_LIGHT1,GL_AMBIENT,ambient);
	glLightfv(GL_LIGHT1,GL_DIFFUSE,diffuse);
	glLightfv(GL_LIGHT1,GL_SPECULAR,specular);
	glLightfv(GL_LIGHT1,GL_POSITION,LightPos);

	glLightf(GL_LIGHT1,GL_CONSTANT_ATTENUATION,cAtt);
	glLightf(GL_LIGHT1,GL_LINEAR_ATTENUATION,lAtt);
	glLightf(GL_LIGHT1,GL_QUADRATIC_ATTENUATION,qAtt);

	glEnable(GL_LIGHT1);
};

void myInitial(){
	glewInit();
	if(glewIsSupported("GL_VERSION_2_0")){
		cout << "Ready for OpenGL 2.0\n";
	}else{
		cout << "GLSL not supported\n";
		exit(1);
	}

	dmModel.loadSkull();
	dmModel.loadLee();
	dmModel.loadCSIE();
	dmModel.loadHead();
	
	

	/* Lighting Initial */
	lightInit();
	materialInit();

	/* Enable parameters  */
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	//glEnable(GL_BLEND);
	//glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	/*
	 * Project Specific Works' own initialization
	 */
#ifdef USE_BIWI
	biwi.Init(DB_SET,0);
	biwi.setCrop(500, 1500, 80, 560, 80, 400);
#else
	XnStatus status = kinect.Init();
#ifdef RES_QVGA
	kinect.setCrop(0, 2500, 40, 280, 40, 200);
#else
	kinect.setCrop(0, 2500, 80, 560, 80, 400);
#endif
#endif
	myHead.mReadModel(MODEL_FILENAME);
	myHead.findNose();
	modelDrawer.getHistogram(myHead.pointCloud);

	matcher.buildTree(myHead.pointCloud, myHead.center, myHead.nose);
}

void myReshape(GLint w, GLint h){

	/* projection parameters setting */
	glVars.winW = w;
	glVars.winH = h;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(_FOVY, _ASPECT*w/h, _ZNEAR, _ZFAR);
	glViewport(0,0,w,h);
	/* modelview parameters setting */
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	/* camera parameters setting */
	gluLookAt(_EYEX, _EYEY, _EYEZ, 
			  _CENTERX, _CENTERY, _CENTERZ,
			  _UPX, _UPY, _UPZ);
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	/* openGL window back ground clor */
	glClearColor(_CC_R, _CC_G, _CC_B, _CC_A);
}

void myGLimwrite(){
	int glW = glutGet(GLUT_WINDOW_WIDTH);
	int glH = glutGet(GLUT_WINDOW_HEIGHT);
	Mat src(glH, glW, CV_8UC3);
	Mat flipped;
	glReadPixels(0, 0, glW, glH, GL_BGR, GL_UNSIGNED_BYTE, (void*) src.data);
	cv::flip(src, flipped, 0);
	imwrite("./output/oout.png", flipped);
}

/* This Callback Function is for OpenCV */
void myMouseCallBack(int event, int x, int y, int flags, void* param){

	if(sysVars.doMatch){	
		// if real time tracking is on, assign nose window position
		if(event == CV_EVENT_LBUTTONDOWN || mouse.Lpressed){
			mouse.Lpressed = true;
			sampler.reset();	
			glVars.reset();
			sysVars.reset();
			sfilter.reset();
			sampler.nose2i.x = x;
			sampler.nose2i.y = y;
			sampler.startDelay = -1;
		}
		if(event == CV_EVENT_LBUTTONUP){ 
			mouse.Lpressed = false;
			mouse.reset();
		}
	}else{
		// if real time tracking is not on, assign nose window position
		/* Left Mouse */
		if(event == CV_EVENT_LBUTTONDOWN){
			mouse.Lpressed = true;
			mouse.setCurrent(x, y);
			mouse.setStarted(x, y);
		}
		if(mouse.Lpressed){
			mouse.setCurrent(x,y);

			kinect.cropL = (x < mouse.xSt)? ((x<0)? 0 : x) : ((mouse.xSt<0)? 0 : mouse.xSt);
			kinect.cropR = (x > mouse.xSt)? (x>(IMG_W-1))?(IMG_W-1):x	: (mouse.xSt>(IMG_W-1))?(IMG_W-1):mouse.xSt;
			kinect.cropT = (y < mouse.ySt)? ((y<0)? 0 : y) : ((mouse.ySt<0)? 0 : mouse.ySt);
			kinect.cropB = (y > mouse.ySt)? (y>(IMG_H-1))?(IMG_H-1):y	: (mouse.ySt>(IMG_H-1))?(IMG_H-1):mouse.ySt;
		}
		if(event == CV_EVENT_LBUTTONUP){ 
			mouse.Lpressed = false;
			mouse.reset();
		}
	}

	/* Right Mouse */
	if(event == CV_EVENT_RBUTTONDOWN){
		mouse.Rpressed = true;
		mouse.setCurrent(x, y);
		mouse.setStarted(x, y);
	}
	if(mouse.Rpressed){		

		kinect.cropL = ((int)kinect.cropL + x - mouse.x) > 0 ? (kinect.cropL + x - mouse.x) : 0;
		kinect.cropR = (kinect.cropR + x - mouse.x) < IMG_W ? (kinect.cropR + x - mouse.x) : IMG_W-1;
		kinect.cropT = ((int)kinect.cropT + y - mouse.y) > 0 ? (kinect.cropT + y - mouse.y) : 0;
		kinect.cropB = (kinect.cropB + y - mouse.y) < IMG_H ? (kinect.cropB + y - mouse.y) : IMG_H-1;

		mouse.setCurrent(x,y);
	}
	if(event == CV_EVENT_RBUTTONUP){ 
		mouse.Rpressed = false;
		mouse.reset();
	}
}

void myKey(GLubyte key, GLint x, GLint y){
	String msg;

	switch(key){
	case '[':
		sysVars.sampleNum += 5;
		break;
	case ']':
		sysVars.sampleNum -= 5;
		break;
	case 'f':	
		msg = sfilter.nextFilter();
		glVars.setMessage(msg);
		break;
	case '=':	
		sysVars.turnOnOffMatch();	
		break;
	case 'r':	
		sampler.reset();	
		glVars.reset();
		sysVars.reset();
		sfilter.reset();
		break;
	case 'n':	
		sampler.switchNoseSmooth();	
		break;
	case '/':	// biwi next set
		sampler.reset();	
		sysVars.reset();
		biwi.nextSet();
		
		sysVars.isPause = false;
		break;
	case 't':	// test : now is glImWrite
		//myGLimwrite();
		sfilter.switchRegRawInHTB();
		break;
	case 'm':	// change model
		dmModel.next();
		break;
	case ',':	// kinect mirror
		kinect.switchMirroring();
		break;
	/* SPACE: Pause this program */
	case 32:	
		sysVars.pause();	
		sfilter.reset();
		break;
	/* ESC: End this program */
	case 27:	exit(0);	break;

#ifdef EDIT_BIWI
	case 'a':	
		biwi.bOffset += 10;	
		biwi.genPC();	
		printf("bOffset = %d\n", biwi.bOffset);
		break;
	case 's':	
		biwi.bOffset -= 10;	
		biwi.genPC();	
		printf("bOffset = %d\n", biwi.bOffset);
		break;
	case 'z':	
		biwi.aOffset += 10;	
		biwi.genPC();	
		printf("aOffset = %d\n", biwi.aOffset);
		break;
	case 'x':	
		biwi.aOffset -= 10;	
		biwi.genPC();	
		printf("aOffset = %d\n", biwi.aOffset);
		break;
	case 'm':
		printf("write model");
		biwi.mWriteModel();
		break;
#endif
	
	}
	glutPostRedisplay();
}

/* This Callback Function is for OpenGL */
void myMouse(int button, int state, int x, int y){
	//printf("mouse function-->  button: %d, state: %d, x: %d, y: %d\n",button, state, x, y);
	glVars.msBtn = button;
	glVars.msSta = state;
	glVars.msX = x;
	glVars.msY = y;
	glutPostRedisplay();
}

void myMotion(int x, int y){
	//printf("motion function-->  x: %d, y:%d\n", x, y);
	
	int xdis = x - glVars.msX;
	int ydis = y - glVars.msY;

	switch(glVars.msBtn){
	case GLUT_LEFT_BUTTON:		// Rotation
		glVars.uiYaw	+= (float) xdis * 0.5f;
		glVars.uiPitch	-= (float) ydis * 0.5f;

		//printf("yaw :%.2f, pitch: %.2f\n", glVars.uiYaw, glVars.uiPitch);
		break;
	case GLUT_MIDDLE_BUTTON:	// Scale
		if(abs(xdis) > abs(ydis))
			glVars.uiRoll	+= (float) xdis * 0.5f;
		else
			glVars.uiScale  *= (float) 1+(ydis / 480.f);
		break;
	case GLUT_RIGHT_BUTTON:		// Translation
		glVars.uiTransX -= (float) xdis;
		glVars.uiTransY -= (float) ydis;
		//printf("tx :%.2f, ty: %.2f\n", glVars.uiTransX, glVars.uiTransY);
		break;
	}

	glVars.msX = x;
	glVars.msY = y;

	glutPostRedisplay();
}

void myIdle(){
	glutPostRedisplay();
}

void myGlPutText(float x, float y, const char* text, LPVOID font, float r, float g, float b, float a) 
{ 
	if(!text || !strlen(text)) return; 

	/* Projectoin and Modelview Matrix store then clear */
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	/* blending and lighting status storing then specify for text putting */
    bool blending = false; 
    if(glIsEnabled(GL_BLEND))	blending = true; 
	else	glEnable(GL_BLEND); 
	bool lighting = true; 
    if(glIsEnabled(GL_LIGHTING)) glDisable(GL_LIGHTING);
	else	lighting = false;
	
    /* Start to put text */
    glColor4f(r,g,b,a); 
    glRasterPos2f(x,y); 
    while (*text) { 
        glutBitmapCharacter(font, *text); 
        text++; 
    } 

	/* blending and lighting status resume */
    if(!blending) glDisable(GL_BLEND); 
	if(lighting) glEnable(GL_LIGHTING);
	

	/* Projectoin and Modelview Matrix restore */
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}
