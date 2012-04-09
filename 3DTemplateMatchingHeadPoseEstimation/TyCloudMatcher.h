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
	bool isConverged;	// Converge Flag
	

	std::vector<cv::Point3f> bufCloud;
	cv::Point3f bufCenter;

	std::vector<cv::Point3f> dstCloud;
	cv::Point3f dstCenter;
	cv::Point3f dstNose;

	std::vector<cv::Point3f> srcCloud;
	cv::Point3f srcCenter;
	cv::Point3f srcNose;

	cv::flann::Index kdtree;


	void transformPointCloud(const cv::Vec6f arguments,
							 const std::vector<cv::Point3f> &_pCloud,
							 const cv::Point3f &_center,
							 std::vector<cv::Point3f> &_pCloudRes, 
							 cv::Point3f &_centerRes);
	
	int bestDirection(const std::vector<cv::Point3f> &pCloud, const cv::Point3f &center);
	float energy(const std::vector<cv::Point3f> &pCloud);

public:
	TYCloudMatcher();
	void buildTree(vector<Point3f> &inCloud, Point3f &inCenter, Point3f &inNose);
	void match(cv::Vec6f &pose, const std::vector<cv::Point3f> &pCloud, const cv::Point3f &nose);
	
};


#endif