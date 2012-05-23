#include "TySampler.h"
//#include "TyTimer.h"

TYSampler::TYSampler(){
	nwX = NOSE_WINDOW_X;
	nwY = NOSE_WINDOW_Y;
	startDelay = START_DELAY;
	areaW = SAMPLE_AREA_WIDTH;
	areaH = SAMPLE_AREA_HEIGHT;
	noseSmoothTerm = true;
	a = 0;
	b = 0;
}

void TYSampler::setSampleArea(float width, float height){
	areaW = width;
	areaH = height;
}

bool TYSampler::randomSampling(const Mat &depthRAW, const Vec6f &pose, const int sampleNum){
	// Initialization
	sample2i.clear();
	sample3f.clear();
	orCloud.clear();
	irCloud.clear();
	pcIndex.clear();

	// Find the nose tip by inverse rotation 
	this->findNoseTip(depthRAW, pose);
	
	// define the depth value of the nose tip for further define the size of sampling area
	float z = (float) depthRAW.at<unsigned short>(nose2i.y, nose2i.x);
	if(z == 0)	return false;

	// define the sampling area
	a = (areaW / z) * FOCAL_LEN;
	b = (areaH / z) * FOCAL_LEN;
	areaAngle = -pose[2];	// roll angle
	float rad = (float) toRad(areaAngle);
	
	
	int randX, randY;
	int X, Y;
	float _f = 1.f/FOCAL_LEN;

	float sumX = 0.f, sumY = 0.f, sumZ = 0.f;
	for(int idx = 0 ; idx < sampleNum ; idx++){
		/* generate random number: */
		//float theta = (float) ( toRad(rand()%360) );
		//
		//randX = (int)( sqrtf((float)rand() / RAND_MAX ) * a * cosf(theta) );
		//randY = (int)( sqrtf((float)rand() / RAND_MAX ) * b * sinf(theta) );

		float theta = (float) ( toRad( ((float)idx*7/(float)sampleNum)*360 ) );
		
		randX = (int)( sqrtf( (float)idx/(float)sampleNum ) * a * cosf(theta) );
		randY = (int)( sqrtf( (float)idx/(float)sampleNum ) * b * sinf(theta) );


		X = (int) ( randX * cosf(rad) - randY * sinf(rad) );
		Y = (int) ( randX * sinf(rad) + randY * cosf(rad) );

		if(!OUT_OF_BOUNDARY(nose2i.x+X, nose2i.y+Y)){
			sample2i.push_back( Point(nose2i.x+X, nose2i.y+Y) );
			float pZ = depthRAW.at<unsigned short>(nose2i.y+Y, nose2i.x+X);
			if(pZ != 0){
				float pX = (IMG_W_HALF-(nose2i.x+X))*pZ*_f;
				float pY = (IMG_H_HALF-(nose2i.y+Y))*pZ*_f;
				sample3f.push_back( Point3f(pX,pY,pZ));
				sumX += pX;
				sumY += pY;
				sumZ += pZ;
			}
		}
	}

	return true;
}

void TYSampler::findNoseTip(const Mat &depthRAW, const Vec6f &pose){
	//TYTimer timer;	/* time report : 0.5 ms */
	
	/* Inverse Rotation */
	float nearest = FLT_MAX;
	int offset;
	float tmpX, tmpY, tmpZ;
	float X, Y, Z;
	
	float yaw	= toRad(pose[0]);
	float pitch = toRad(pose[1]);
	float roll	= toRad(pose[2]);

	int height = depthRAW.rows;
	int width = depthRAW.cols;
	int wStep16UC1 = depthRAW.step1()*2;

	float _f = 1.f/FOCAL_LEN;

	/* �ٯʤ@��crop���center���B�J */

	
	/* Reset */
	if(nose2i.x <=0 || nose2i.x>=width-1) startDelay = 10;


	/* startDelay > 0 means detect mode , simply find the nearest point */
	if(startDelay > 0){
		for (int y = 0 ; y < height ; y++) {
			offset = y*wStep16UC1;
			for (int x = 0 ; x < width ; x++) {
				unsigned short dVal = ((unsigned short*)(depthRAW.data + offset))[x];
				if((dVal < nearest) && (dVal != 0.f)){
					nearest = dVal;
					nose2i.x = x;
					nose2i.y = y;
				}
			}
		}
	}
	/* frameCounter <= 0 means track mode , find the nearest inversely rotated point */
	else{
		int lx, ly;
		lx = nose2i.x;
		ly = nose2i.y;

		orCloud.clear();
		irCloud.clear();
		pcIndex.clear();

		for (int y = ly - nwY ; y < (ly + nwY) ; y++) {
		//for(int y = 0 ; y < height ; y++){
			if(y < 0 || y >= height)	continue;
			offset = y*wStep16UC1;
			for (int x = lx - nwX ; x < (lx + nwX) ; x++) {
			//for(int x = 0 ; x < width ; x++){
				if(x < 0 || x >= width)	continue;
				
				int val = ((unsigned short*)(depthRAW.data + y*wStep16UC1))[x];
				if (val == 0)	continue;
			

				// get (x,y,z)
				Z = (float) val;
				X = (IMG_W_HALF-x)*Z*_f;
				Y = (IMG_H_HALF-y)*Z*_f;

				orCloud.push_back(Point3f(X,Y,Z));

				// align center to (0, 0 ,0)
				X = X - 0;
				Y = Y - 0;
				Z = Z - 800;

				// Simulate glRotatef(yaw, 0, 1, 0)
				tmpX =  X * cos(yaw) + Z * sin(yaw);
				tmpY =  Y ;
				tmpZ = -X * sin(yaw) + Z * cos(yaw);
				X = tmpX;	Y = tmpY;	Z = tmpZ;

				// Simulate glRotatef(pitch, 1, 0, 0)
				tmpX = X ;
				tmpY = Y * cos(pitch) - Z * sin(pitch);
				tmpZ = Y * sin(pitch) + Z * cos(pitch);
				X = tmpX;	Y = tmpY;	Z = tmpZ;

				// Simulate glRotatef(roll, 0, 0, 1)
				tmpX = X * cos(roll) - Y * sin(roll);
				tmpY = X * sin(roll) + Y * cos(roll);
				tmpZ = Z;
				X = tmpX;	Y = tmpY;	Z = tmpZ;
				

				X = X + 0;
				Y = Y + 0;
				Z = Z + 800;

				irCloud.push_back(Point3f(X,Y,Z));
				pcIndex.push_back(Point2i(x,y));

				if(Z < nearest){
					nearest = Z;
					nose2i.x = x;
					nose2i.y = y;
					nose3f = orCloud[orCloud.size()-1];
				}
			}
		}
	}
	
	if(noseSmoothTerm){
#ifdef DRAW_NOSE_SMOOTH_TERM
		noseSmoothSet2i.clear();
		noseSmoothSet3f.clear();
#endif
		/* nose pixel, nose vertex, smooth processing */
		Point2f sum2f(0.f, 0.f);
		Point3f sum3f(0.f, 0.f, 0.f);
		int pNum = 0;
		if(startDelay > 0 ){
			startDelay--;
			for (int y = 0 ; y < height ; y++) {
				offset = y*wStep16UC1;
				for (int x = 0 ; x < width ; x++) {
					unsigned short dVal = ((unsigned short*)(depthRAW.data + offset))[x];
					if((dVal < nearest+5) && (dVal != 0.f)){

						sum2f.x += x;
						sum2f.y += y;
					
						sum3f.x += (IMG_W_HALF - x) * ((float) dVal) * _f;
						sum3f.y += (IMG_H_HALF - y) * ((float) dVal) * _f;
						sum3f.z += (float) dVal;

						pNum++;

#ifdef DRAW_NOSE_SMOOTH_TERM
						noseSmoothSet2i.push_back(Point(x,y));
						noseSmoothSet3f.push_back(Point3f((IMG_W_HALF - x) * ((float) dVal) * _f,
							                              (IMG_H_HALF - y) * ((float) dVal) * _f,
														  (float) dVal));
#endif
					}
				}
			}
		}else{
			int nPoints = (unsigned int)irCloud.size();
			for(int i = 0 ; i < nPoints ; i++){
				if(irCloud[i].z < nearest + 5){
#ifdef DRAW_NOSE_SMOOTH_TERM
						noseSmoothSet2i.push_back(pcIndex[i]);
						noseSmoothSet3f.push_back(Point3f((IMG_W_HALF - pcIndex[i].x) * ((float) irCloud[i].z) * _f,
							                              (IMG_H_HALF - pcIndex[i].y) * ((float) irCloud[i].z) * _f,
														  (float) irCloud[i].z));
#endif
					sum2f.x += pcIndex[i].x;
					sum2f.y += pcIndex[i].y;

					sum3f.x += (IMG_W_HALF - pcIndex[i].x) * ((float) orCloud[i].z) * _f;
					sum3f.y += (IMG_H_HALF - pcIndex[i].y) * ((float) orCloud[i].z) * _f;
					sum3f.z += (float) orCloud[i].z;

					pNum++;
				}
			}
		}

		nose2i.x = (int)(sum2f.x / pNum);
		nose2i.y = (int)(sum2f.y / pNum);
	
		nose3f.x = sum3f.x / (float)pNum;
		nose3f.y = sum3f.y / (float)pNum;
		nose3f.z = sum3f.z / (float)pNum;
	}

	return;
}

void TYSampler::drawSamples2i(Mat &image){
	if(nose2i.x < 0 || nose2i.x > 640)	return;

	// Draw the Nose Searching Window on the "depthRGB" image
	Rect rect(nose2i.x - nwX, nose2i.y - nwY, 2*nwX, 2*nwY);
	rectangle(image, rect, CV_RGB(0,128,255));
	for(unsigned int i = 0 ; i < noseSmoothSet2i.size() ; i++){
		image.at<Vec3b>(noseSmoothSet2i[i].y, noseSmoothSet2i[i].x)[0] = 255;
		image.at<Vec3b>(noseSmoothSet2i[i].y, noseSmoothSet2i[i].x)[1] = 128;
		image.at<Vec3b>(noseSmoothSet2i[i].y, noseSmoothSet2i[i].x)[2] = 0;
	}

	// Draw the nose tip on the "depthRGB" image
	circle(image, nose2i,  2, CV_RGB(0,255,0));

	// Draw the sample area boundary on the "depthRGB" image
	ellipse(image, RotatedRect(nose2i, Size2f(2*a, 2*b), areaAngle), CV_RGB(255, 0, 0));

	// Draw the sample points on the "depthRGB" image
	for(unsigned int i = 0 ; i < sample2i.size() ; i++){
		image.at<Vec3b>(sample2i[i].y, sample2i[i].x)[0] = 0;
		image.at<Vec3b>(sample2i[i].y, sample2i[i].x)[1] = 0;
		image.at<Vec3b>(sample2i[i].y, sample2i[i].x)[2] = 255;
	}
}

void TYSampler::drawSamples3f(){
	glPointSize(3.0);
	glBegin(GL_POINTS);
		glColor3f(1.f, 0.f, 0.f);
		for(unsigned int i = 0 ; i < sample3f.size() ; i++)
			glVertex3f(sample3f[i].x, sample3f[i].y, sample3f[i].z);
	glEnd();
	glPointSize(1.0);

	/*glBegin(GL_POINTS);
		glColor3f(0.f, 0.f, 0.8f);
		for(unsigned int i = 0 ; i < irCloud.size() ; i++)
			glVertex3f(irCloud[i].x, irCloud[i].y, irCloud[i].z);
	glEnd();*/
	

	glPointSize(4.0);
	glBegin(GL_POINTS);
		glColor3f(0.f, 1.f, 0.f);
		glVertex3f(nose3f.x, nose3f.y, nose3f.z);
	glEnd();
	glPointSize(1.0);
}

void TYSampler::reset(){
	startDelay = 10;
}
void TYSampler::switchNoseSmooth(){
	noseSmoothTerm = !noseSmoothTerm;
}