#include "TyHeadPoseEstimation.h"
#include "TyTimer.h"
#include "TyKinectMotorPlus.h"
#include "TyModelLoader.h"
#include "TySampler.h"
#include "TyCloudMatcher.h"
#include "TyCloudDrawer.h"

TYGLVariables glVars;
TYTimer myTimerGL;
TYKinect kinect(true, true, "still.oni");
TYHeadModel myHead;
TYSampler sampler;
TYCloudMatcher matcher;

TYCloudDrawer modelDrawer;
TYCloudDrawer observeDrawer;

// πÍ≈Á•Œ
Vec6f pose(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
Vec3f vecOM(0.f, 0.f, 0.f);

bool doMatch = false;


void ProgramStart(){
	printf("Head Motion Tracking Progeam Started!!! \n");
	glutInitDisplayMode(GLUT_RGB | GL_DOUBLE);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutInitWindowPosition(GL_WIN_PIVOT_X, GL_WIN_PIVOT_Y);
	int WindowNum = glutCreateWindow("Head Motion Tracking");
	myInitial();  
	glutIdleFunc(myIdle);
	glutReshapeFunc(myReshape);
	glutDisplayFunc(myDisplay);
	glutKeyboardFunc(myKey);
	glutMouseFunc(myMouse);
	glutMotionFunc(myMotion);
	glutMainLoop();
}

void myDisplay(){
	myTimerGL.timeInit();		// time calculation head
	
	/* Sensor Input: Kinect Grab One Frame Set <Depth, RGB> */
	XnStatus kinectStatus = kinect.GetMetaData();

	if(kinectStatus == XN_STATUS_OK){
		kinect.GetCvFormatImages();
	}
	
	/* GL ModelView Matrix Reset */
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(_EYEX, _EYEY, _EYEZ, _CENTERX, _CENTERY, _CENTERZ, _UPX, _UPY, _UPZ);
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glClearColor(_CC_R, _CC_G, _CC_B, _CC_A);

	/* Put FPS String Onto The GL Window */
	myGlPutText(-0.95f, -0.95f, myTimerGL.FPSstring(), GLUT_BITMAP_HELVETICA_18, 1.0f, 0.0f, 0.0f, 1.0f);

	/* Viewing Deformations (Camera Move) */
	glTranslatef(glVars.uiTransX, glVars.uiTransY, glVars.uiTransZ);
	glTranslatef(myHead.center.x, myHead.center.y, myHead.center.z);		// Still need to set camera rotate pivot here
	glRotatef(glVars.uiPitch, 1, 0, 0);
	glRotatef(glVars.uiYaw  , 0, 1, 0);
	glRotatef(glVars.uiRoll , 0, 0, 1);
	glScalef(glVars.uiScale, glVars.uiScale, glVars.uiScale);
	glTranslatef(-myHead.center.x, -myHead.center.y, -myHead.center.z);

	
	/*
	 *	Program Body
	 */


	sampler.randomSampling(kinect.DepthRAW, pose, SAMPLE_NUMBER);
	if(doMatch)	matcher.match(pose, vecOM, sampler.sample3f, sampler.nose3f);
	

	glPushMatrix();
		glTranslatef(myHead.nose.x, myHead.nose.y, myHead.nose.z);		// Still need to set camera rotate pivot here
	
		glTranslatef(-pose[3]-vecOM[0], -pose[4]-vecOM[1], -pose[5]-vecOM[2]);	// x, y, z
		glRotatef(-pose[0], 0, 1, 0);	// yaw
		glRotatef(-pose[1], 1, 0, 0);	// pitch
		glRotatef(-pose[2], 0, 0, 1);	// roll
	
		glTranslatef(-myHead.nose.x, -myHead.nose.y, -myHead.nose.z);

		myHead.drawNose();
		modelDrawer.drawPointCloud(myHead.pointCloud);

	glPopMatrix();

	sampler.drawSamples2i(kinect.DepthRGB);
	sampler.drawSamples3f();

	kinect.Show(true, true);
	int key = waitKey(1);
	kinect.RegisterKey(key);

	glutSwapBuffers();
	myTimerGL.sprintFPS();		// time calculation tail
}

void myInitial(){
	/* projection parameters setting */
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(_FOVY, _ASPECT, _ZNEAR, _ZFAR);
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
	
	/* Lighting Initial */
	myLightInit();
	myMaterialInit();

	/* Enable parameters  */
	//glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	//glEnable(GL_BLEND);
	//glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	/*
	 * Project Specific Works' own initialization
	 */
	XnStatus status = kinect.Init();
	kinect.setCrop(0, 840, 80, 560, 80, 400);
	myHead.mReadModel(MODEL_FILENAME);
	myHead.findNose();
	modelDrawer.getHistogram(myHead.pointCloud);

	matcher.buildTree(myHead.pointCloud, myHead.center, myHead.nose);
}

void myReshape(GLint w, GLint h){
	glVars.winW = w;
	glVars.winH = h;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(_FOVY, _ASPECT*w/h, _ZNEAR, _ZFAR);
	glViewport(0,0,w,h);
}

void myKey(GLubyte key, GLint x, GLint y){
	//printf("key = %x, x = %d, y = %d\n",key,x,y);
	//mMaker.registerKey(key);
	//glutPostRedisplay();
	switch(key){
	case '=':	doMatch = true;	break;
	case 'r':	sampler.reset();	break;
	case 'n':	sampler.switchNoseSmooth();	break;
	}
}

void myMouse(int button, int state, int x, int y){
	//printf("mouse function-->  button: %d, state: %d, x: %d, y: %d\n",button, state, x, y);
	glVars.msBtn = button;
	glVars.msSta = state;
	glVars.msX = x;
	glVars.msY = y;
	//glutPostRedisplay();
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

	//glutPostRedisplay();
}

void myIdle(){
	glutPostRedisplay();
}

void myLightInit()
{
	float LightPos[4];
	int i = 0;
	float ambient[4],diffuse[4],specular[4];
	float cAtt,lAtt,qAtt;
	float r,alpha,beta;

	for(int j=0;j<3;j++){
		ambient[j]=0.8;
		diffuse[j]=0.8;
		specular[j]=0.3;
	}
	ambient[3]=diffuse[3]=specular[3]=1;

	r=1000;
	cAtt=1;
	lAtt=0.0000000001;
	qAtt=0.0000000000001;
	alpha = 0;
	beta = 0;

	LightPos[0]=r*cos(toRad(alpha))*cos(toRad(beta));
	LightPos[1]=r*cos(toRad(alpha))*sin(toRad(beta));
	LightPos[2]=r*sin(toRad(alpha));
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
	glLightfv(GL_LIGHT1,GL_AMBIENT,ambient);
	glLightfv(GL_LIGHT1,GL_DIFFUSE,diffuse);
	glLightfv(GL_LIGHT1,GL_SPECULAR,specular);
	glLightfv(GL_LIGHT1,GL_POSITION,LightPos);

	glLightf(GL_LIGHT1,GL_CONSTANT_ATTENUATION,cAtt);
	glLightf(GL_LIGHT1,GL_LINEAR_ATTENUATION,lAtt);
	glLightf(GL_LIGHT1,GL_QUADRATIC_ATTENUATION,qAtt);

	glEnable(GL_LIGHT1);
};

void myMaterialInit()
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

void myGlPutText(float x, float y, char* text, LPVOID font, float r, float g, float b, float a) 
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
