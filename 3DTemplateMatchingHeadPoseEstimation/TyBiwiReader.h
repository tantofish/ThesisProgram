#ifndef _TY_BIWI_H_
#define _TY_BIWI_H_

#include "main.h"

class TYBiwiDB{
public:
	/* root path setting */
	TYBiwiDB(char* path = "./");
	/* Just Set sIndex and fIndex*/
	void Init(int sIndex = 1, int fIndex = 0);
	/* read next frame (D_RAW, D_RGB, Pose, C_RGB) */
	bool getNextFrame();
	/* show DepthRGB , and ColorRGB(if defined) */
	void Show();
	/* set cropping boundarys  */
	void setCrop(int n = 0, int f = 9999, int l = 0, int r = 639, int t = 0, int b = 479);
	/* key function implemented with OpenCV */
	void RegisterKey(int key);
	/* skip this set and jump to next set */
	void nextSet();
	/* generate point cloud */
	void genPC();
	/* write model*/
	bool mWriteModel();
	/* reutrn the current frame number */
	int frame(){return fIdx;}


	float currentSet();
	float currentFrame();
	float yawAngle();
	float pitchAngle();
	float rollAngle();

	//Depth : unsigned short, 1 channel
	Mat DepthRAW;

	//Image : unsigned char, 3 channels
	Mat DepthRGB;
	Mat ColorRGB;

	bool isFirstFrame;

	vector<Point3f>	point;
	vector<Vec3f> color;

	int bOffset;	// z - y > b crop
	int aOffset;	// z - y < a crop

	// head pose parameters
	Mat pm;
	Point3f	nose;
	float m[3][3];	// pose matrix given by the database label (considered as ground truth)
	float pose[3];	// decomposition the m[3][3] to yaw, pitch and roll
	

private:
	void RAW2RGB();
	float histogram[10000];
	bool loadDepthImageCompressed(cv::Mat &img, const char* fname );

	char rootPath[100];
	char fileName[100];
	char sIndex[3];	// Ex: when sIdx == 2,  sIndex == "02"
	char fIndex[6]; // Ex: when fIdx == 15, fIndex == "00015"

	int sIdx;	// set index
	int fIdx;	// frame index
	pair<int,int> range[25];
	Vec6f crop[25];

	

	// CROPPING
	unsigned int cropL, cropR, cropT, cropB, cropN, cropF;
	
	

};

#endif