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
	void findNose();
	void drawColorCloud();
	void drawNose(float r = 1.f, float g = 0.0f, float b = 0.f, float size = 3.f);
	bool mReadModel(char* filename);

	std::vector<cv::Point3f> pointCloud;
	std::vector<cv::Vec3f>	 colorCloud;

	//flann::Matrix<float> flannPCloud;

	cv::Point3f center;	// 3D position of all points' geometrical center
	cv::Point3f nose;	// 3D position of the nose tip

	void DM2PCwrite(cv::Mat &);	// Depth Map 2 Point Cloud Write
};





#endif