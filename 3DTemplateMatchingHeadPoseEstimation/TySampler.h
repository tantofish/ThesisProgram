#ifndef TYSAMPLER_H
#define TYSAMPLER_H
#define _CRT_SECURE_NO_WARNINGS

#include "main.h"
#include "TyVariables.h"

#include <cmath>
#include <ctime>

class TYSampler{
public:
	/* Public Member Attributes*/
	
	Point2i nose2i;
	Point3f nose3f;

	vector<Point3f> sample3f;
	vector<Point2i> sample2i;

	vector<Point3f> feSample3f;
	vector<Point2i> feSample2i;
	/* Public Member Functions */

	/* Constructor */
	TYSampler();
	/* Make randam sampling from the raw depth map. The given pose is used for inverse rotation */
	bool randomSampling(const Mat &depthRAW, const Vec6f &pose, const int sampleNum);
	/* Draw the sample points on the given 2d image (it should be the visualized depth map). OpenCV */
	void drawSamples2i(Mat &image);
	/* Draw the 3D sample points on OpenGL part */
	void drawSamples3f();
	/* Set the sampling area size (width and height in millimeter) */
	void setSampleArea(float width, float height);
	/* Register the System Varaibles Object */
	void getSysVars(TYSysVariables *sysV);
	/**/
	void reset();
	/**/
	void switchNoseSmooth();


	vector<Point3f>	orCloud;	// orginal point cloud
	vector<Point3f>	irCloud;	// inversely rotated point cloud
	vector<Point2i>	pcIndex;	// pixel index for point Cloud

	vector<Point3f> restCloud;	//(restOrClouf) :for picture generating use (useless in normal process)
	vector<Point3f> restIrCloud;//(restIrClouf) :for picture generating use (useless in normal process)
	vector<Point3f> wholeOrCloud;//(wholeOrCloud) :for picture generating use (useless in normal process)

	vector<Point2i> noseSmoothSet2i;
	vector<Point3f> noseSmoothSet3f;

	int startDelay;
private:
	/* Private Member Attributes*/
	int nwX;	// Nose Searching Window Width
	int nwY;	// Nose Searching Window Height
	

	float a;	// horizontal axis radius of the sampling area ellipse (in pixel)
	float b;	// vertical axis radius of the sampling area ellipse (in pixel)
	float areaW;	// horizontal axis radius of the sampling area ellipse (in millimeter)
	float areaH;	// vertical axis radius of the sampling area ellipse (in millimeter)
	float areaAngle;

	float feA;
	float feB;
	int feSampleNum;

	bool noseSmoothTerm;

	TYSysVariables *sysVars;

	/* Private Member Functions */
	void findNoseTip(const Mat &depthRAW, const Vec6f &pose);
	inline void picGenUse(const Mat &depthRAW, const Vec6f &pose);
};






#endif