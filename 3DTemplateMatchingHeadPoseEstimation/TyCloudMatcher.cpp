#include "TyCloudMatcher.h"
#include "TyTimer.h"

/* Constructor */
TYCloudMatcher::TYCloudMatcher(){
	gradient.push_back( Vec6f( 0.f , 0.f , 0.f , 0.f , 0.f , 0.f ) );
	gradient.push_back( Vec6f( 1.f , 0.f , 0.f , 0.f , 0.f , 0.f ) );
	gradient.push_back( Vec6f( -1.f, 0.f , 0.f , 0.f , 0.f , 0.f ) );
	gradient.push_back( Vec6f( 0.f , 1.f , 0.f , 0.f , 0.f , 0.f ) );
	gradient.push_back( Vec6f( 0.f , -1.f, 0.f , 0.f , 0.f , 0.f ) );
	gradient.push_back( Vec6f( 0.f , 0.f , 1.f , 0.f , 0.f , 0.f ) );
	gradient.push_back( Vec6f( 0.f , 0.f , -1.f, 0.f , 0.f , 0.f ) );
	gradient.push_back( Vec6f( 0.f , 0.f , 0.f , 1.f , 0.f , 0.f ) );
	gradient.push_back( Vec6f( 0.f , 0.f , 0.f , -1.f, 0.f , 0.f ) );
	gradient.push_back( Vec6f( 0.f , 0.f , 0.f , 0.f , 1.f , 0.f ) );
	gradient.push_back( Vec6f( 0.f , 0.f , 0.f , 0.f , -1.f, 0.f ) );
	gradient.push_back( Vec6f( 0.f , 0.f , 0.f , 0.f , 0.f , 1.f ) );
	gradient.push_back( Vec6f( 0.f , 0.f , 0.f , 0.f , 0.f , -1.f) );

	rStep = ROTATE_STEP;
	tStep = TRANSLATE_STEP;
	cvgCtr = 0;
	isConverged = true;

	query = flann::Matrix<float>(qData, 1, 3);

};

void TYCloudMatcher::buildTree(vector<Point3f> &inCloud, Point3f &inCenter, Point3f &inNose){
	/*
	dstCloud.clear();
	dstCloud.insert(dstCloud.end(), inCloud.begin(), inCloud.end());
	dstCenter = inCenter;
	dstNose = inNose;

	TYTimer timer;
	timer.timeInit();
	cout << "Building Destination Tree... " ;
	
	kdtree.build(cv::Mat(dstCloud).reshape(1), cv::flann::KDTreeIndexParams());
	timer.timeReportMS();
	*/
	TYTimer timer;
	dstCenter = inCenter;
	dstNose = inNose;
	cout << "Uploading Point Cloud Data... " ;
	timer.timeInit();
	int nPoints = (int)inCloud.size();
	float *data = new float [nPoints*3];
	for(int i = 0 ; i < nPoints ; i++){
		data[i*3+0] = inCloud[i].x;
		data[i*3+1] = inCloud[i].y;
		data[i*3+2] = inCloud[i].z;
	}
	dataset = flann::Matrix<float>(data,nPoints,3);
	timer.timeReportMS();

	cout << "Building Destination KD Tree... " ;
	timer.timeInit();
	kdtree = new flann::Index<flann::L2<float> >(dataset, flann::KDTreeSingleIndexParams());
	kdtree->buildIndex();
	timer.timeReportMS();

}

void TYCloudMatcher::match(cv::Vec6f &pose, Vec3f &vecOM, const std::vector<cv::Point3f> &pCloud, const cv::Point3f &nose){
	if(!isConverged)	return;
	
	
	vecOM[0] = dstNose.x - nose.x;
	vecOM[1] = dstNose.y - nose.y;
	vecOM[2] = dstNose.z - nose.z;
	printf("vecOM(%.2f,%.2f,,%.2f,)\n",vecOM[0],vecOM[1],vecOM[2]);

	argCurrent = pose;
	argCurrent[3] += vecOM[0];
	argCurrent[4] += vecOM[1];
	argCurrent[5] += vecOM[2];
	//printf("Steepest Gradient Idx = %d\n", bestMatchIdx);
	isConverged = false;


	while(!isConverged){
		int index = this->bestDirection(pCloud, nose);
		
		if(index == 0){
			if( cvgCtr>=2 || rStep<=0.5f ){
				//printf(" matching converged! \n");
				cvgCtr = 0;
				rStep = ROTATE_STEP;
				tStep = TRANSLATE_STEP;
				isConverged = true;
			}
			cvgCtr++;
			rStep *= 0.5;
			tStep *= 0.5;
		}else{
			cvgCtr = 0;
		}

		//this->transformPointCloud(argCurrent, pCloud, center, 

		glutPostRedisplay();
	}

	argCurrent[3] -= vecOM[0];
	argCurrent[4] -= vecOM[1];
	argCurrent[5] -= vecOM[2];
	pose = argCurrent;
	printf("pose(%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)\n", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
};

int TYCloudMatcher::bestDirection(const std::vector<cv::Point3f> &pCloud, const cv::Point3f &center){
	Vec6f argPeek;

	float min = FLT_MAX;
	int idx = -1;
	float dist;

	/* Try 13 Directions, For Each Direction: */
	for(int i = 0 ; i < 13 ; i ++){
		/* Set up the peaking argument for the current direction */
		if(i > 6)		// i = [7,12] : is Translating direction 
			for(int j = 0 ; j < 6 ; j++)	argPeek[j] = argCurrent[j] + gradient[i][j] * tStep;
		else if(i > 0)	// i = [1, 6] : is Rotating direction 
			for(int j = 0 ; j < 6 ; j++)	argPeek[j] = argCurrent[j] + gradient[i][j] * rStep;
		else			// i = 0	  : is No Transforming direction 
			for(int j = 0 ; j < 6 ; j++)	argPeek[j] = argCurrent[j];

		/* Translate and rotate the source point cloud by the current argument */
		transformPointCloud(argPeek, pCloud, center, this->bufCloud, this->bufCenter); 
		
		/* Calculate Energy(Distance) */
		dist = energy(this->bufCloud);

		/* Keep the best one */
		if( dist < min ){
			min = dist;
			idx = i;
		}
	}
	
	float step = (idx>6) ? tStep:rStep;
		for(int i = 0 ; i < 6 ; i ++)
			argCurrent[i] = argCurrent[i] + gradient[idx][i]*step;


	//printf("	Step:(%.1f, %.1f, %.1f, %.1f, %.1f, %.1f), (idx, min) = (%d, %.2f) \n",	
	//	argCurrent[0], argCurrent[1], argCurrent[2], argCurrent[3], argCurrent[4], argCurrent[5], idx, min);	

	return idx;
};

float TYCloudMatcher::energy(const std::vector<cv::Point3f> &pCloud){
	
	
	float distSum = 0.f;
	
	//float distThresh = 10.f;
	TYTimer timer;
	//#pragma omp parallel for reduction(+:distSum)
	//for(int queryIdx = 0 ; queryIdx < pCloud.size() ; queryIdx++){
	//	
	//	vector<float> query(3);
	//	query[0] = pCloud[queryIdx].x;
	//	query[1] = pCloud[queryIdx].y;
	//	query[2] = pCloud[queryIdx].z;

	//	int knn = 1;
	//	vector<int> indices(knn);
	//	vector<float> dists(knn);

	//	//timer.timeInit();
	//	kdtree.knnSearch(query, indices, dists, knn, flann::SearchParams(32));
	//	//timer.timeReportMS();

	//	distSum = distSum + dists[0];
	//	
	//}

	int knn = 1;	
	flann::Matrix<int> indices(new int[query.rows*knn], query.rows, knn);
	flann::Matrix<float> dists(new float[query.rows*knn], query.rows, knn);

	for(unsigned int queryIdx = 0 ; queryIdx < pCloud.size() ; queryIdx++){
		
		query[0][0] = pCloud[queryIdx].x;
		query[0][1] = pCloud[queryIdx].y;
		query[0][2] = pCloud[queryIdx].z;

		//timer.timeInit();
		kdtree->knnSearch(query, indices, dists, knn, flann::SearchParams(32));
		//timer.timeReportMS();

		distSum = distSum + dists[0][0];
		
	}

	delete [] indices.ptr();
	delete [] dists.ptr();
	//printf("par[6]=(%.1f, %.1f, %.1f, %.1f, %.1f, %.1f)	inlier: %d\n", mtYaw, mtPitch, mtRoll, mtTransX, mtTransY, mtTransZ, inlierCount);
	//printf("inlier: %d, distSum: %.2f, nSamples: %d\n", inlierCount, distSum, nSamples);
	return distSum;
};

void TYCloudMatcher::transformPointCloud(const cv::Vec6f arguments,
									 const std::vector<cv::Point3f> &_pCloud, const cv::Point3f &_center,
									 std::vector<cv::Point3f> &_pCloudRes, cv::Point3f &_centerRes){
	
	
	if(_pCloudRes.size() != _pCloud.size())	_pCloudRes.resize(_pCloud.size());

	int nPoints = (int) _pCloud.size();
	float yaw   = toRad(arguments[0]);
	float pitch = toRad(arguments[1]);
	float roll  = toRad(arguments[2]);
	float tx	= arguments[3];
	float ty	= arguments[4];
	float tz	= arguments[5];

	float tmpX, tmpY, tmpZ;
	float X, Y, Z;
	
	for(int i = 0 ; i < nPoints ; i++){
		// align center to (0, 0 ,0)
		X = _pCloud[i].x - _center.x;
		Y = _pCloud[i].y - _center.y;
		Z = _pCloud[i].z - _center.z;

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

		// Simulate glTranslatef(tx, ty, yz)  +  align to original center
		_pCloudRes[i].x = tmpX + _center.x + tx;
		_pCloudRes[i].y = tmpY + _center.y + ty;
		_pCloudRes[i].z = tmpZ + _center.z + tz;
	}
	// assign the center which is returned as a result
	_centerRes.x = _center.x + tx;
	_centerRes.y = _center.y + ty;
	_centerRes.z = _center.z + tz;
}