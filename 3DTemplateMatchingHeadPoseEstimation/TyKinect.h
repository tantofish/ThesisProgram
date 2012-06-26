#ifndef TYKINECT_H
#define TYKINECT_H
#define _CRT_SECURE_NO_WARNINGS

#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
	fprintf(stderr,"%s failed: %s\n", what, xnGetStatusString(rc));	\
	}


#include "main.h"

struct TYColorPoint3D
{
	float X, Y, Z, R, G, B;
	float NX, NY, NZ;
	TYColorPoint3D(){}
	TYColorPoint3D( XnPoint3D pos, XnRGB24Pixel color ) : 
		X(pos.X), Y(pos.Y), Z(pos.Z), 
		R((float)color.nRed	/ 255),
		G((float)color.nGreen / 255),
		B((float)color.nBlue / 255)
	{ }
	TYColorPoint3D( XnPoint3D pos ) : 
		X(pos.X), Y(pos.Y), Z(pos.Z)
	{ }
};

class TYKinect{
public:
	/*-------------------------------------------*/
	/* Public Member Atributes                   */
	/*-------------------------------------------*/

	//Depth : unsigned short, 1 channel
	Mat DepthRAW;

	//Image : unsigned char, 3 channels
	Mat DepthRGB;
	Mat ColorRGB;

	const XnDepthPixel *oldDepthMap;
	const XnDepthPixel *pDepthMap;
	const XnRGB24Pixel *pImageMap;


	vector<TYColorPoint3D> pCloud;
	XnPoint3D center;

	int validPointNumber;


	/*-------------------------------------------*/
	/* Public Member Functions                   */
	/*-------------------------------------------*/
	TYKinect(bool syncTwoVeiw = true, bool mirroring = true, char *filename = "");
	~TYKinect();

	XnStatus Init(bool depthNode = true, bool imageNode = true);
	XnStatus Stop();
	XnStatus GetMetaData();			// Read a new frame (Every frame starts from here)
	XnStatus GeneratePointCloud();

	// Is implemented if CLNUI SDK have been setup
	virtual bool motorUp()	{	return false;	}
	virtual bool motorDown(){	return false;	}
	virtual bool setLed(int color)	{	return false;	}


	void GetCvFormatImages();
	/* key function implemented with OpenCV */
	void RegisterKey(int key);
	/* show depth/color images in OpenCV window */
	void Show(bool depth = true, bool image = true);
	/* save output images */
	void SaveImages();
	/* set cropping boundarys  */
	void setCrop(int n = -1, int f = -1, int l = -1, int r = -1, int t = -1, int b = -1);
	/* switch mirroring */
	void switchMirroring();

	// for thesis specific
	bool isFirstFrame;		// for pause at first frame use

	//CROPPING
	unsigned int cropL, cropR, cropT, cropB, cropN, cropF;
private:

	/*-------------------------------------------*/
	/* Private Member Attributes                  */
	/*-------------------------------------------*/

	// OpenNI Attributes
	XnMapOutputMode outputMode;
	int width;
	int height;
	int fps;
	bool antiFlickerCap;
	
	string oniFile;

	XnUInt64 focalLength;
	XnDouble pixelSize;

	Context context;
	DepthGenerator depth;
	DepthMetaData depthMD;
	ImageGenerator image;
	ImageMetaData imageMD;

	XnStatus status;	


	// tantofish
	String iWindowName;
	String dWindowName;
	bool showDepth;
	bool showImage;
	bool hasDepthNode;
	bool hasImageNode;
	bool isSyncTwoView;
	bool isMirroring;

	int opMode;	// Operation Mode

	

	//Globals
	unsigned int g_pDepthHist[MAX_DEPTH];
	unsigned int g_nViewState ;

	int wStep16UC1, wStep8UC3;

	/*for save out images*/
	int  outputIdx;
	char outputName[50];


	/*-------------------------------------------*/
	/* Private Member Functions                  */
	/*-------------------------------------------*/
	bool tyDepthHistogram();
	bool tyHistoAccuProb();
	static void cvMouse( int event, int x, int y, int flags, void* param );
};


#endif