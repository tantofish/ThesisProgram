#include "TyModelLoader.h"
#include "TyTimer.h"

bool TYHeadModel::mReadModel(char* filename){
	Point3f max(-9999999999,-9999999999,-9999999999), min(9999999999,9999999999,9999999999);

	cout << "Loading model : " << filename << endl ;
	
	ifstream file(filename); 
	if(!file) { 
		cout << "Cannot open file.\n"; 
		return false; 
	}

	int nPoints;
	file >> nPoints;
	

	pointCloud.resize(nPoints);
	colorCloud.resize(nPoints);
	float x, y, z, r, g, b;
	float sx = 0.f, sy = 0.f, sz = 0.f;
	
	for(int i = 0 ; i < nPoints ; i++){
		file >> x >> y >> z >> r >> g >> b ;
		pointCloud[i].x = x;
		pointCloud[i].y = y;
		pointCloud[i].z = z;
		colorCloud[i][0] = r;
		colorCloud[i][1] = g;
		colorCloud[i][2] = b;
		sx += x;
		sy += y;
		sz += z;

		//cout << inPointCloud[i].x << " " << inPointCloud[i].y << " " << inPointCloud[i].z << endl;
	}
	center.x = sx/nPoints;
	center.y = sy/nPoints;
	center.z = sz/nPoints;

	

	file.close(); 
	//printf("# of points : %d  ", pointCloud.size());

	cout << "Done! # of vertex = " << nPoints << endl;

	return true;
}

void TYHeadModel::findNose(){

	/* find the point with the smallest z value, we regard it as nose tip */
	float shallowest = FLT_MAX;
	int index = -1;
	
	/* for sake of error range, we get an average of the N nearest points */

	for(unsigned int i = 0 ; i < pointCloud.size() ; i++){
		if(pointCloud[i].z < shallowest){
			shallowest = pointCloud[i].z;
			index = i;
		}
	}
	
	Point3f Sum(0.f, 0.f, 0.f);
	int count = 0;
	for(unsigned int i = 0 ; i < pointCloud.size() ; i++){
		if(pointCloud[i].z < shallowest+5){
			count++;
			Sum.x += pointCloud[i].x;
			Sum.y += pointCloud[i].y;
			Sum.z += pointCloud[i].z;
		}
	}
	averageNose.x = Sum.x / (float)count;
	averageNose.y = Sum.y / (float)count;
	averageNose.z = Sum.z / (float)count;
	printf("model average nose (%.2f,%.2f,%.2f)\n",averageNose.x,averageNose.y,averageNose.z);

	nose = averageNose;
}

void TYHeadModel::drawColorCloud(){
	/* Draw the Head */
	glBegin(GL_POINTS);
	for(unsigned int i = 0 ; i < pointCloud.size() ; i++){
		glColor4f(colorCloud[i][0], colorCloud[i][1], colorCloud[i][2], 0.5f);
		glVertex3f(pointCloud[i].x, pointCloud[i].y, pointCloud[i].z);
	}
	glEnd();
}

void TYHeadModel::drawNose(float r, float g, float b, float size){
#ifdef DRAW_MODEL_NOSE
	/* Mark the Nose */
#ifdef	KNN_AVG_NOSE 
	glPushMatrix();
		glColor3f(0.f, 0.f, 0.6f);
		glTranslatef(knnAvgNose.x, knnAvgNose.y, knnAvgNose.z);
		glutSolidSphere (size,10,10);	
	glPopMatrix();
#endif
#ifdef	SHALLOWEST_NOSE
	glPushMatrix();
		glColor3f(1.f, 0.f, 0.f);
		glTranslatef(shallowestNose.x, shallowestNose.y, shallowestNose.z);
		glutSolidSphere (size,10,10);	
	glPopMatrix();
#endif
#ifdef	AVG_NOSE
	glPushMatrix();
		glColor3f(r, g, b);
		glTranslatef(averageNose.x, averageNose.y, averageNose.z);
		glutSolidSphere (size,10,10);	
	glPopMatrix();
#endif
#endif
}

void TYHeadModel::mGetModelOnline(cv::Mat &DepthRAW, cv::Mat &ColorRGB){

	/* find the point with the smallest z value, we regard it as nose tip */
	float shallowest = FLT_MAX;
	int xIdx = -1;
	int yIdx = -1;
	int nPoints = 0;

	/* for sake of error range, we get an average of the N nearest points */
	for(int y = 0 ; y < IMG_H ; y++){
		for(int x = 0 ; x < IMG_W ; x++){
			float z = (float) DepthRAW.at<unsigned short>(y, x);
			if(z == 0.f) continue;
			nPoints ++;
			if(z < shallowest){
				shallowest = z;
				xIdx = x;
				yIdx = y;
			}
		}
	}

	/* Point Cloud Generate */
	pointCloud.resize(nPoints);
	colorCloud.resize(nPoints);
	Point3f AvgNose(0.f, 0.f, 0.f);
	Point3f Sum(0.f, 0.f, 0.f);
	float _f = 1.f/FOCAL_LEN;
	int idx = 0;
	int count = 0;

	for(int y = 0 ; y < IMG_H ; y++){
		for(int x = 0 ; x < IMG_W ; x++){
			float z = (float) DepthRAW.at<unsigned short>(y, x);
			if(z == 0.f) continue;
			
			pointCloud[idx].x = (IMG_W_HALF-x) * z * _f * RES_FACTOR;
			pointCloud[idx].y = (IMG_H_HALF-y) * z * _f * RES_FACTOR;
			pointCloud[idx].z = z;

			colorCloud[idx][0] = ColorRGB.at<Vec3b>(y, x)[2];
			colorCloud[idx][1] = ColorRGB.at<Vec3b>(y, x)[1];
			colorCloud[idx][2] = ColorRGB.at<Vec3b>(y, x)[0];

			Sum.x += pointCloud[idx].x;
			Sum.y += pointCloud[idx].y;
			Sum.z += pointCloud[idx].z;
		
			if(z < shallowest + 5){
				count++;
				AvgNose.x += pointCloud[idx].x;
				AvgNose.y += pointCloud[idx].y;
				AvgNose.z += pointCloud[idx].z;
			}
			idx ++;
		}
	}

	AvgNose.x /= (float)count;
	AvgNose.y /= (float)count;
	AvgNose.z /= (float)count;
	printf("model average nose (%.2f,%.2f,%.2f)\n", AvgNose.x, AvgNose.y, AvgNose.z);

	center.x = Sum.x / (float) nPoints;
	center.y = Sum.y / (float) nPoints;
	center.z = Sum.z / (float) nPoints;

	nose = AvgNose;
}

void TYHeadModel::mGetFacialExpressionOnline(cv::Mat &DepthRAW, cv::Mat &ColorRGB){

	vector<cv::Point3f> *pCloud = new vector<cv::Point3f>();
	vector<cv::Vec3f> *cCloud = new vector<cv::Vec3f>();

	/* find the point with the smallest z value, we regard it as nose tip */
	float shallowest = FLT_MAX;
	int xIdx = -1;
	int yIdx = -1;
	int nPoints = 0;

	/* for sake of error range, we get an average of the N nearest points */
	for(int y = 0 ; y < IMG_H ; y++){
		for(int x = 0 ; x < IMG_W ; x++){
			float z = (float) DepthRAW.at<unsigned short>(y, x);
			if(z == 0.f) continue;
			nPoints ++;
			if(z < shallowest){
				shallowest = z;
				xIdx = x;
				yIdx = y;
			}
		}
	}

	/* Point Cloud Generate */
	pCloud->resize(nPoints);
	cCloud->resize(nPoints);
	Point3f feAvgNose(0.f, 0.f, 0.f);
	Point3f feAvgCenter(0.f, 0.f, 0.f);
	float _f = 1.f/FOCAL_LEN;
	int idx = 0;
	int count = 0;

	for(int y = 0 ; y < IMG_H ; y++){
		for(int x = 0 ; x < IMG_W ; x++){
			float z = (float) DepthRAW.at<unsigned short>(y, x);
			if(z == 0.f) continue;
			
			(*pCloud)[idx].x = (IMG_W_HALF-x) * z * _f * RES_FACTOR;
			(*pCloud)[idx].y = (IMG_H_HALF-y) * z * _f * RES_FACTOR;
			(*pCloud)[idx].z = z;

			(*cCloud)[idx][0] = ColorRGB.at<Vec3b>(y, x)[2];
			(*cCloud)[idx][1] = ColorRGB.at<Vec3b>(y, x)[1];
			(*cCloud)[idx][2] = ColorRGB.at<Vec3b>(y, x)[0];

			feAvgCenter.x += (*pCloud)[idx].x;
			feAvgCenter.y += (*pCloud)[idx].y;
			feAvgCenter.z += (*pCloud)[idx].z;
		
			if(z < shallowest + 5){
				count++;
				feAvgNose.x += (*pCloud)[idx].x;
				feAvgNose.y += (*pCloud)[idx].y;
				feAvgNose.z += (*pCloud)[idx].z;
			}
			idx ++;
		}
	}

	feAvgNose.x /= (float)count;
	feAvgNose.y /= (float)count;
	feAvgNose.z /= (float)count;
	printf("model average nose (%.2f,%.2f,%.2f)\n", feAvgNose.x, feAvgNose.y, feAvgNose.z);

	feAvgCenter.x /= (float) nPoints;
	feAvgCenter.y /= (float) nPoints;
	feAvgCenter.z /= (float) nPoints;

	feCenter.push_back(feAvgCenter);
	feNose.push_back(feAvgNose);
	fePointCloud.push_back(pCloud);
	feColorCloud.push_back(cCloud);

	feIdx++;
}