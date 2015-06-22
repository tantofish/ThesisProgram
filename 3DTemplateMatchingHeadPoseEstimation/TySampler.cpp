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
	/* not random now, i made it kind of uniform for reason that sampling number is not big enough */
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
	if(sysVars->doFacialExpression)	areaH = areaW * 0.625f;
	b = (areaH / z) * FOCAL_LEN;

	feSample3f.clear();
	feSample2i.clear();
	feA = (FE_SAMPLE_AREA_WIDTH / z) * FOCAL_LEN;
	feB = (FE_SAMPLE_AREA_HEIGHT / z) * FOCAL_LEN;

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
		/*randX = (int)( (float)idx/(float)sampleNum  * a * cosf(theta) );
		randY = (int)( (float)idx/(float)sampleNum  * b * sinf(theta) );*/


		X = (int) ( randX * cosf(rad) - randY * sinf(rad) );
		Y = (int) ( randX * sinf(rad) + randY * cosf(rad) );

		if(!OUT_OF_BOUNDARY(nose2i.x+X, nose2i.y+Y)){
			sample2i.push_back( Point(nose2i.x+X, nose2i.y+Y) );
			float pZ = depthRAW.at<unsigned short>(nose2i.y+Y, nose2i.x+X);
			if(pZ != 0){
				float pX = (IMG_W_HALF-(nose2i.x+X))*pZ*_f*RES_FACTOR;
				float pY = (IMG_H_HALF-(nose2i.y+Y))*pZ*_f*RES_FACTOR;

				if(sqrt((nose3f.x-pX)*(nose3f.x-pX)+(nose3f.y-pY)*(nose3f.y-pY)+(nose3f.z-pZ)*(nose3f.z-pZ)) > SEGMENT_THRESH)	continue;

				sample3f.push_back( Point3f(pX,pY,pZ));
				sumX += pX;
				sumY += pY;
				sumZ += pZ;
			}
		}
	}

	if(sysVars->doFacialExpression){
		feSampleNum = (int)(sampleNum * 5.f);
		/* Sampling for Facial Expression Recognition */
		for(int idx = 0 ; idx < feSampleNum ; idx++){
			float theta = (float) ( toRad( ((float)idx*7/(float)feSampleNum)*360 ) );
		
			randX = (int)( sqrtf( (float)idx*0.8f/((float)feSampleNum) + 0.2) * feA * cosf(theta) );
			randY = (int)( sqrtf( (float)idx*0.8f/((float)feSampleNum) + 0.2) * feB * sinf(theta) );
	
			X = (int) ( randX * cosf(rad) - randY * sinf(rad) );
			Y = (int) ( randX * sinf(rad) + randY * cosf(rad) );

			if(!OUT_OF_BOUNDARY(nose2i.x+X, nose2i.y+Y)){
			
				float pZ = depthRAW.at<unsigned short>(nose2i.y+Y, nose2i.x+X);
				if(pZ != 0){
					float pX = (IMG_W_HALF-(nose2i.x+X))*pZ*_f*RES_FACTOR;
					float pY = (IMG_H_HALF-(nose2i.y+Y))*pZ*_f*RES_FACTOR;

					if(sqrt((nose3f.x-pX)*(nose3f.x-pX)+(nose3f.y-pY)*(nose3f.y-pY)+(nose3f.z-pZ)*(nose3f.z-pZ)) > 80)	continue;
					feSample2i.push_back( Point(nose2i.x+X, nose2i.y+Y) );
					feSample3f.push_back( Point3f(pX,pY,pZ));
				}
			}
		}
	}

	return true;
}

inline void TYSampler::picGenUse(const Mat &depthRAW, const Vec6f &pose){
#ifdef NORMAL_WHOLE_CLOUD
	float X, Y, Z;
	int height = depthRAW.rows;
	int width = depthRAW.cols;
	int wStep16UC1 = depthRAW.step1()*2;

	float _f = 1.f/FOCAL_LEN;

	wholeOrCloud.clear();
	for(int y = 0 ; y < height ; y++){
		for(int x = 0 ; x < width ; x++){
			int val = ((unsigned short*)(depthRAW.data + y*wStep16UC1))[x];
			if (val == 0)	continue;

			// get (x,y,z)
			Z = (float) val;
			X = (IMG_W_HALF-x)*Z*_f*RES_FACTOR;
			Y = (IMG_H_HALF-y)*Z*_f*RES_FACTOR;

			wholeOrCloud.push_back(Point3f(X,Y,Z));
		}
	}
#endif
#ifdef NOSE_REST_CLOUD
	float tmpX, tmpY, tmpZ;
	float X, Y, Z;
	
	float yaw	= toRad(pose[0]);
	float pitch = toRad(pose[1]);
	float roll	= toRad(pose[2]);

	int height = depthRAW.rows;
	int width = depthRAW.cols;
	int wStep16UC1 = depthRAW.step1()*2;

	float _f = 1.f/FOCAL_LEN;

	restCloud.clear();
	restIrCloud.clear();
	for(int y = 0 ; y < height ; y++){
		for(int x = 0 ; x < width ; x++){
			if( (y >= (ly - nwY)) && (y < (ly + nwY)) && (x >= (lx - nwX)) && (x < (lx + nwX)) )
				continue;
			int val = ((unsigned short*)(depthRAW.data + y*wStep16UC1))[x];
			if (val == 0)	continue;

			// get (x,y,z)
			Z = (float) val;
			X = (IMG_W_HALF-x)*Z*_f*RES_FACTOR;
			Y = (IMG_H_HALF-y)*Z*_f*RES_FACTOR;

			restCloud.push_back(Point3f(X,Y,Z));

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

			restIrCloud.push_back(Point3f(X,Y,Z));
		}
	}
#endif
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

	/* 還缺一個crop跟算center的步驟 */
	

	picGenUse(depthRAW, pose);

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

		for (int y = ly - nwY ; y < (ly + 2*nwY) ; y++) {
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
				X = (IMG_W_HALF-x)*Z*_f*RES_FACTOR;
				Y = (IMG_H_HALF-y)*Z*_f*RES_FACTOR;

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
					
						sum3f.x += (IMG_W_HALF - x) * ((float) dVal) * _f * RES_FACTOR;
						sum3f.y += (IMG_H_HALF - y) * ((float) dVal) * _f * RES_FACTOR;
						sum3f.z += (float) dVal;

						pNum++;

#ifdef DRAW_NOSE_SMOOTH_TERM
						noseSmoothSet2i.push_back(Point(x,y));
						noseSmoothSet3f.push_back(Point3f((IMG_W_HALF - x) * ((float) dVal) * _f *RES_FACTOR;,
							                              (IMG_H_HALF - y) * ((float) dVal) * _f *RES_FACTOR;,
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
						//noseSmoothSet3f.push_back(Point3f((IMG_W_HALF - pcIndex[i].x) * ((float) irCloud[i].z) * _f * RES_FACTOR;,
						//	                              (IMG_H_HALF - pcIndex[i].y) * ((float) irCloud[i].z) * _f * RES_FACTOR;,
						//								  (float) irCloud[i].z));
						noseSmoothSet3f.push_back(irCloud[i]);
						
#endif
					sum2f.x += pcIndex[i].x;
					sum2f.y += pcIndex[i].y;

					sum3f.x += (IMG_W_HALF - pcIndex[i].x) * ((float) orCloud[i].z) * _f * RES_FACTOR;
					sum3f.y += (IMG_H_HALF - pcIndex[i].y) * ((float) orCloud[i].z) * _f * RES_FACTOR;
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

#ifdef OUTPUT_NOSE_POS
	ofstream file("./output/nose.txt", ofstream::app);
	file << nose3f.x << '\t' << nose3f.y << '\t' << nose3f.z << '\t';
	file.close();
#endif

	return;
}

void TYSampler::drawSamples2i(Mat &image){
	if(nose2i.x < 0 || nose2i.x >= IMG_W)	return;

#ifdef DRAW_NSW_ON_2D
	// Draw the Nose Searching Window on the "depthRGB" image
	Rect rect(nose2i.x - nwX, nose2i.y - nwY, 2*nwX, 2*nwY);
	rectangle(image, rect, CV_RGB(0,0,255));
	for(unsigned int i = 0 ; i < noseSmoothSet2i.size() ; i++){
		image.at<Vec3b>(noseSmoothSet2i[i].y, noseSmoothSet2i[i].x)[0] = 0;
		image.at<Vec3b>(noseSmoothSet2i[i].y, noseSmoothSet2i[i].x)[1] = 255;
		image.at<Vec3b>(noseSmoothSet2i[i].y, noseSmoothSet2i[i].x)[2] = 0;
	}
#endif

#ifdef DRAW_NOSE_TIP_ON_2D
	// Draw the nose tip on the "depthRGB" image
	circle(image, nose2i,  3, CV_RGB(255,0,0), 2);
#endif
#ifdef DRAW_SAMPLE_ON_2D
	// Draw the sample area boundary on the "depthRGB" image
	ellipse(image, RotatedRect(nose2i, Size2f(2*a, 2*b), areaAngle), CV_RGB(255, 0, 0));

	// Draw the sample points on the "depthRGB" image
	for(unsigned int i = 0 ; i < sample2i.size() ; i++){
		image.at<Vec3b>(sample2i[i].y, sample2i[i].x)[0] = 0;
		image.at<Vec3b>(sample2i[i].y, sample2i[i].x)[1] = 0;
		image.at<Vec3b>(sample2i[i].y, sample2i[i].x)[2] = 255;
	}

	if(sysVars->doFacialExpression){
		// Draw the sample area boundary on the "depthRGB" image
		ellipse(image, RotatedRect(nose2i, Size2f(2*feA, 2*feB), areaAngle), CV_RGB(0, 255, 255));

		// Draw the sample points on the "depthRGB" image
		for(unsigned int i = 0 ; i < feSample2i.size() ; i++){
			image.at<Vec3b>(feSample2i[i].y, feSample2i[i].x)[0] = 255;
			image.at<Vec3b>(feSample2i[i].y, feSample2i[i].x)[1] = 255;
			image.at<Vec3b>(feSample2i[i].y, feSample2i[i].x)[2] = 0;
		}
	}

#endif
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
	
	/*glPushMatrix();
	glTranslatef(nose3f.x,nose3f.y,nose3f.z);
	glColor3f(0.f,0.f,1.f);
	glutSolidSphere(50,10,10);
	glPopMatrix();*/
}

void TYSampler::reset(){
	startDelay = 10;
}

void TYSampler::switchNoseSmooth(){
	noseSmoothTerm = !noseSmoothTerm;
}

void TYSampler::getSysVars(TYSysVariables *sysV){
	this->sysVars = sysV;
}