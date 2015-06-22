#ifndef _MODEL_LOADER_H_
#define _MODEL_LOADER_H_
#define _CRT_SECURE_NO_WARNINGS

#include "main.h"

class TYHeadModel{
private:

	// in my opinion, averageNose is the best
	cv::Point3f shallowestNose;
	//cv::Point3f knnAvgNose;
	cv::Point3f averageNose;
public:
	TYHeadModel():outIndex(0),feIdx(0){}
	void findNose();
	void drawColorCloud();
	void drawNose(float r = 1.f, float g = 0.0f, float b = 0.f, float size = 3.f);
	bool mReadModel(char* filename);

	std::vector<cv::Point3f> pointCloud;	// user's head model point cloud (x, y, z)
	std::vector<cv::Vec3f>	 colorCloud;	// user's head model color cloud (r, g, b)
	cv::Point3f center;	// 3D position of all points' geometrical center
	cv::Point3f nose;	// 3D position of the nose tip

	int feIdx;
	vector< std::vector<cv::Point3f>* > fePointCloud;	// facial expression point cloud
	vector< std::vector<cv::Vec3f>* > feColorCloud;		// facial expression color cloud
	vector< cv::Point3f > feCenter;	// facial expression pc center
	vector< cv::Point3f > feNose;	// facial expressopm pc nose

	int outIndex;
	char outName[50];
	//void DM2PCwrite(cv::Mat &DepthRAW, cv::Mat &ColorRGB);	// Depth Map 2 Point Cloud Write
	void mGetModelOnline(cv::Mat &DepthRAW, cv::Mat &ColorRGB);
	void mGetFacialExpressionOnline(cv::Mat &DepthRAW, cv::Mat &ColorRGB);
};





#endif