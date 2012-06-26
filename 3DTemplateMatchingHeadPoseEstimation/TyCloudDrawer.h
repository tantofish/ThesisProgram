#ifndef _TY_CLOUD_DRAWER_H
#define _TY_CLOUD_DRAWER_H
#define _CRT_SECURE_NO_WARNINGS

#include "main.h"

class TYCloudDrawer{
public:
	/*--------------------------*/
	/* Public Member Attributes */
	/*--------------------------*/
	float histogram[MAX_DEPTH];

	/*--------------------------*/
	/* Public Member Functions  */
	/*--------------------------*/
	bool getHistogram(vector<Point3f> &cloud);
	void drawPointCloud(vector<Point3f> &cloud);
	void drawPointCloudRed(vector<Point3f> &cloud);
	void drawPointCloudGreen(vector<Point3f> &cloud);
	void drawPointCloudBlue(vector<Point3f> &cloud);
private:
	/*---------------------------*/
	/* Private Member Attributes */
	/*---------------------------*/

	/*--------------------------*/
	/* Public Member Functions  */
	/*--------------------------*/
};

#endif