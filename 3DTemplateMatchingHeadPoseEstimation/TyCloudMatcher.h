#ifndef _CLOUDMATCHER_H_
#define _CLOUDMATCHER_H_

#include "main.h"

class TYCloudMatcher{
private:
	std::vector<cv::Vec6f> gradient;
	cv::Vec6f argCurrent;
	//cv::Vec6f argPeek;
	float rStep;		// Rotate Step
	float tStep;		// Translate Step
	int cvgCtr;			// Converge Counter
	
	
	int cvgThreshC;		// Converge Threshold of Count
	float cvgThreshR;	// Converge Threshold of Rotate Step
	float cvgThreshEPP; // Converge Threshold of per Point Energy
	float cvgThreshGoEPP;// Converge Threshold of " Gradient of per Point Energy "

	float graOfPerPointEnergy;	//" Gradient of per Point Energy "
	float perPointEnergy;	// Total Energy divided by Point Num
	float preIterEnergy;	// previous iteration energy
	int preIterIdx;			// gradient index
	bool isPreIdxValid;		// is previous iteration gradient still work flag
	bool isFirstIter;		// is first iteration flag


	std::vector<cv::Point3f> bufCloud;
	cv::Point3f bufCenter;

	std::vector<cv::Point3f> dstCloud;
	cv::Point3f dstCenter;
	cv::Point3f dstNose;

	std::vector<cv::Point3f> srcCloud;
	cv::Point3f srcCenter;
	cv::Point3f srcNose;

	/* For Head Motion Tracking */
	vector<flann::Matrix<float>> dataset;
	vector<flann::Index<flann::L2<float> > *>kdtree;
	vector<cv::Point3f> feDstCenter;
	vector<cv::Point3f> feDstNose;

	float qData[3];
	flann::Matrix<float> query; 



	void transformPointCloud(const cv::Vec6f arguments,
							 const std::vector<cv::Point3f> &_pCloud,
							 const cv::Point3f &_center,
							 std::vector<cv::Point3f> &_pCloudRes, 
							 cv::Point3f &_centerRes);
	
	int bestDirection(const std::vector<cv::Point3f> &pCloud, const cv::Point3f &center);
	

	/* Match pCloud(sample point cloud) to kdtree[i](ith model point cloud) */
	float energy(const std::vector<cv::Point3f> &pCloud, int i = 0);

public:
	TYCloudMatcher();
	void buildTree(vector<Point3f> &inCloud, Point3f &inCenter, Point3f &inNose);
	void buildFETree(vector<Point3f> &inCloud, Point3f &inCenter, Point3f &inNose);


	void match(cv::Vec6f &pose, Vec3f &vecOM, const std::vector<cv::Point3f> &pCloud, const cv::Point3f &nose);
	int bestMatchedFE(cv::Vec6f &pose, Vec3f &vecOM, const std::vector<cv::Point3f> &pCloud, const cv::Point3f &nose);

	bool isConverged;	// Converge Flag
};


#endif