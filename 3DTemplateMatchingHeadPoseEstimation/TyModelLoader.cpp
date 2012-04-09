#include "TyModelLoader.h"
#include "TyTimer.h"

bool TYHeadModel::mReadModel(char* filename){
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
	cv::flann::Index kdtree;
	cv::flann::KDTreeIndexParams indexParams;
	kdtree.build(cv::Mat(pointCloud).reshape(1), indexParams);

	/* find the point with the smallest z value, we regard it as nose tip */
	float shallowest = FLT_MAX;
	int index = -1;
	for(unsigned int i = 0 ; i < pointCloud.size() ; i++){
		if(pointCloud[i].z < shallowest){
			shallowest = pointCloud[i].z;
			index = i;
		}
	}


	/* for sake of error range, we get an average of the N nearest points */
	vector<float> query(3);
	query[0] = pointCloud[index].x;
	query[1] = pointCloud[index].y;
	query[2] = pointCloud[index].z;

	printf("model nose (%.2f,%.2f,%.2f)\n",query[0],query[1],query[2]);
	shallowestNose.x = query[0];
	shallowestNose.y = query[1];
	shallowestNose.z = query[2];

	int knn = 10;
	vector<int> indices(knn);
	vector<float> dists(knn);

	kdtree.knnSearch(query, indices, dists, knn, flann::SearchParams(32));
	
	knnAvgNose = Point3f(0.f, 0.f, 0.f);
	for(int i = 0 ; i < knn ; i++){
		knnAvgNose.x += pointCloud[indices[i]].x;
		knnAvgNose.y += pointCloud[indices[i]].y;
		knnAvgNose.z += pointCloud[indices[i]].z;
	}
	knnAvgNose.x /= (float)knn;
	knnAvgNose.y /= (float)knn;
	knnAvgNose.z /= (float)knn;
	printf("model nose average(%.2f,%.2f,%.2f)\n",nose.x,nose.y,nose.z);

	Point3f Sum(0.f, 0.f, 0.f);
	int count = 0;
	for(unsigned int i = 0 ; i < pointCloud.size() ; i++){
		if(pointCloud[i].z < shallowest+5){
			count++;
			Sum.x += pointCloud[i].x;
			Sum.y += pointCloud[i].y;
			Sum.z += pointCloud[i].z;
			glBegin(GL_POINTS);
				glColor3f(0.5f, 0.5f, 0.f);
				glVertex3f(pointCloud[i].x, pointCloud[i].y, pointCloud[i].z);
			glEnd();		
		}
	}
	averageNose.x = Sum.x / (float)count;
	averageNose.y = Sum.y / (float)count;
	averageNose.z = Sum.z / (float)count;

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