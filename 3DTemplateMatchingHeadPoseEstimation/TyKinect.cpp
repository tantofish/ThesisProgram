#include "tyKinect.h"

//#define TIME
//#define PrintFocalLength
#ifdef TIME
	#include "TyTimer.h"
	TYTimer timer;
#endif

TYKinect :: TYKinect(bool sync, bool mirr, char *filename){

	DepthRAW.create(IMG_H, IMG_W, CV_16UC1);
	DepthRGB.create(IMG_H, IMG_W, CV_8UC3);
	ColorRGB.create(IMG_H, IMG_W, CV_8UC3);

	/* Global variables initial */
	width	= IMG_W;
	height	= IMG_H;
	fps		= IMG_FPS;
	outputMode.nXRes = width;
	outputMode.nYRes = height;
	outputMode.nFPS	 = fps;
	hasImageNode = false;
	hasDepthNode = false;
	showDepth	 = false;
	showImage	 = false;
	antiFlickerCap = false;
	isFirstFrame = true;
	iWindowName  = String(I_WIN_NAME);
	dWindowName  = String(D_WIN_NAME);
	isSyncTwoView = sync;
	isMirroring = mirr;
	opMode = OPMODE_NORMAL;
	cropL = 0; cropR = (width - 1); cropT = 0; cropB = (height - 1); cropN = 0, cropF = 10000;
	//g_nViewState = DEFAULT_DISPLAY_MODE;
	oniFile.assign("oni/");
	oniFile.append(filename);


	outputIdx = 0;
	for(int i = 0 ; i < 30 ; i ++)	outputName[i] = '\0';
}
TYKinect :: ~TYKinect(){
	DepthRAW.release();
	DepthRGB.release();
	ColorRGB.release();
}
XnStatus TYKinect :: Init(bool depthNode, bool imageNode){	//Kinect Routines (Copied From OpenNI Sameple : Simple Viewer)
	cout << "Connecting to kinect..."		<<	endl;

	status = context.Init();
	CHECK_RC(status, "context init");
	if(status != XN_STATUS_OK)	return status;

	/* >>>>> Acquisition from device*/

	/* Depth Node Initialization */
	if(depthNode){
		status = depth.Create(context);
		CHECK_RC(status, "create depth generator");
		if(status == XN_STATUS_OK){
			status = depth.SetMapOutputMode(outputMode);	//640, 480, 30fps
			CHECK_RC(status, "set output mode");
			if(status == XN_STATUS_OK)	hasDepthNode = true;
		}
	}
	/* Depth Node Initialization */
	if(imageNode){
		status = image.Create(context);
		CHECK_RC(status, "create image generator");
		if(status == XN_STATUS_OK){
			status = image.SetMapOutputMode(outputMode);	//640, 480, 30fps
			CHECK_RC(status, "set output mode");
			if(status == XN_STATUS_OK)	hasImageNode = true;
		}
	}
	

	/* >>>>> Acquisition from file*/

	if(!(hasDepthNode||hasImageNode)){
		EnumerationErrors errors;
		printf("Trying to create node from oni file\n");
		//status = xnContextOpenFileRecording( (XnContext*) &context,  oniFile.c_str());
		status = context.OpenFileRecording(oniFile.c_str());
 		CHECK_RC(status,"Open input file");
		
		if (status == XN_STATUS_NO_NODE_PRESENT)
		{
			XnChar strError[1024];
			errors.ToString(strError, 1024);
			printf("%s\n", strError);
			return status;
		}
		else if (status != XN_STATUS_OK)
		{
			printf("Open failed: %s\n", xnGetStatusString(status));
			return status;
		}
	
		status = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth);
		CHECK_RC(status, "Find depth generator");
		if(status == XN_STATUS_OK)	hasDepthNode = true;

		status = context.FindExistingNode(XN_NODE_TYPE_IMAGE, image);
		CHECK_RC(status, "Find image generator");
		if(status == XN_STATUS_OK)	hasImageNode = true;

		if(status != XN_STATUS_OK)
		{
			cout << "Initial Failed!! " << endl;
			return status;
		}
	}
	
	if(isSyncTwoView && hasImageNode && hasDepthNode)	depth.GetAlternativeViewPointCap().SetViewPoint(image);
	if(isMirroring)		context.SetGlobalMirror(!context.GetGlobalMirror());

	status = context.StartGeneratingAll();
	CHECK_RC(status, "context start generating all");

	return status;
}
XnStatus TYKinect :: Stop(){
	if(hasDepthNode && showDepth){
		destroyWindow(dWindowName);
		showDepth = false;
	}
	if(hasImageNode && showImage){
		destroyWindow(iWindowName);
		showImage = false;
	}
	if(hasDepthNode){	
		depth.Release();
		hasDepthNode = false;
	}
	if(hasImageNode){
		image.Release();
		hasImageNode = false;
	}
	context.Release();
	return XN_STATUS_OK;
}
XnStatus TYKinect :: GetMetaData(){
	//Kinect Grab Frame Routine, Every Frame Starts From Here
	
	//cout << "is new data available? " << depth.IsNewDataAvailable() << endl;
	
	//status = context.WaitAnyUpdateAll();
	status = context.WaitOneUpdateAll(depth);
	
	if (status != XN_STATUS_OK)
	{
		fprintf(stderr,"Read from device failed: %s\n", xnGetStatusString(status));
		return status;
	}

	if(hasDepthNode){
		depth.GetMetaData(depthMD);
		pDepthMap = depthMD.Data();
	}
	
	if(hasImageNode){
		image.GetMetaData(imageMD);
		pImageMap = (XnRGB24Pixel*) imageMD.Data();
	}
	
	return status;
}
XnStatus TYKinect :: GeneratePointCloud(){
	
	int width   = depthMD.FullXRes();
	int height  = depthMD.FullYRes();
	int nPoints = width * height;

	
	XnPoint3D* pDepthPointSet = new XnPoint3D[ nPoints ];

	int i, j, offset, idx;

	for( j = 0; j < height ; ++j )
	{
		offset = j * width;
		for( i = 0; i < width; ++i )
		{
			idx = offset + i;
			pDepthPointSet[idx].X = (XnFloat)(width - i);
			pDepthPointSet[idx].Y = (XnFloat)(j);
			pDepthPointSet[idx].Z = pDepthMap[idx];
		}
	}

	// reverse project points to real world
	XnPoint3D* p3DPointSet = new XnPoint3D[ nPoints ];
	
	depth.ConvertProjectiveToRealWorld( nPoints, pDepthPointSet, p3DPointSet );
	
	
	// build point cloud
	float sumX = 0.f, sumY = 0.f, sumZ = 0.f;
	int pCount = 0;
	pCloud.clear();



	if(hasImageNode){
		for( j = 0 ; j < height ; j++){
			offset = j * width;
			for( i = 0 ; i < width ; i++){
				idx = offset + i;
				// skip the depth 0 points
				if( p3DPointSet[idx].Z > 0){
					sumX += p3DPointSet[idx].X;
					sumY += p3DPointSet[idx].Y;
					sumZ += p3DPointSet[idx].Z;
					pCount++;

#ifdef PrintFocalLength// show the focal length
#ifdef RES_QVGA
					printf("f = %f\n", p3DPointSet[idx].Z * (120-j)/ p3DPointSet[idx].Y);
#else
					printf("f = %f\n", p3DPointSet[idx].Z * (240-j)/ p3DPointSet[idx].Y);
#endif
#endif
				}
				
				pCloud.push_back( TYColorPoint3D( p3DPointSet[idx], pImageMap[idx] ) );
			}
		}
	}else{
		for( j = 0 ; j < height ; j++){
			offset = j * width;
			for( i = 0 ; i < width ; i++){
				idx = offset + i;
				// skip the depth 0 points
				if( p3DPointSet[idx].Z > 0){
					sumX += p3DPointSet[idx].X;
					sumY += p3DPointSet[idx].Y;
					sumZ += p3DPointSet[idx].Z;
					pCount++;
				}	
				pCloud.push_back( TYColorPoint3D( p3DPointSet[idx] ) );
			}
		}
	}

	
	validPointNumber = pCount;

	center.X = sumX / (float)pCount;
	center.Y = sumY / (float)pCount;
	center.Z = sumZ / (float)pCount;
	
	
	delete [] pDepthPointSet;
	delete [] p3DPointSet;
	
	return 0;
}
void TYKinect :: Show(bool depth, bool image){
	showDepth = depth;
	showImage = image;
	if(hasDepthNode && showDepth)	imshow(dWindowName, DepthRGB);
	if(hasImageNode && showImage)	imshow(iWindowName, ColorRGB);
}
bool TYKinect :: tyDepthHistogram(){
	//Accumulated Probabilistic Histogram, normalized to 256, for visualization

	for (int nIndex = 0 ; nIndex < MAX_DEPTH ; nIndex++) {
		g_pDepthHist[nIndex] = 0;
	}


	for (int y = 0 ; y < height ; y++) {
		for (int x = 0 ; x < width ; x++) {
			int val = ((unsigned short*)(DepthRAW.data + y*wStep16UC1))[x];
			if (val != 0){
				g_pDepthHist[val]++;
			}
		}
	}

	for (int nIndex=1 ; nIndex < MAX_DEPTH; nIndex++) {
		g_pDepthHist[nIndex] += g_pDepthHist[nIndex-1];
	}

	unsigned int nNumberOfPoints = g_pDepthHist[MAX_DEPTH-1];

	if(nNumberOfPoints) {
		for (int nIndex = 1 ; nIndex < MAX_DEPTH ; nIndex++) {
			g_pDepthHist[nIndex] = (unsigned int)(256 * (1.0f - ((float)g_pDepthHist[nIndex] / (float)nNumberOfPoints)));
		}
	}

	return true;
}
void TYKinect :: GetCvFormatImages(){
	/* Get Raw RGB Image */
	if(hasImageNode){
		Mat ColorTmp = Mat(height, width, CV_8UC3);
		ColorTmp.data =  (uchar*) imageMD.Data();
		cvtColor(ColorTmp, ColorRGB, CV_BGR2RGB);
	}
	
	/* depth RGB image visualization (using histogram) */
	if(hasDepthNode){
		wStep16UC1 = DepthRAW.step1()*2;
		wStep8UC3  = DepthRGB.step1();

		DepthRAW.setTo(cv::Scalar(0));
		/* Get Raw Depth Map */
		const XnDepthPixel *pDepth = pDepthMap;
		pDepth += cropT * width;
		for (XnUInt y = cropT; y <= cropB; y++){
			pDepth += cropL;
			for (XnUInt x = cropL; x <= cropR; x++, ++pDepth){
				if(*pDepth < cropF && *pDepth > cropN)
				((unsigned short*)(DepthRAW.data + y*wStep16UC1))[x] = *pDepth;
			}
			pDepth += width-1 - cropR;
		}

		/* Calculate Accumulated Probability density function*/
		this->tyDepthHistogram();

		/* Color Assign */
		int offset;
		DepthRGB.setTo(cv::Scalar(0,0,0));
		for (int y = 0 ; y < height ; y++) {
			offset = y * wStep8UC3;
			for (int x = 0 ; x < width ; x++) {
				int val = ((unsigned short*)(DepthRAW.data + y*wStep16UC1))[x];
				if (val != 0){
					int n = g_pDepthHist[val];
					((uchar*)(DepthRGB.data + offset))[x*3+0] = 255;
					((uchar*)(DepthRGB.data + offset))[x*3+1] = n;
					((uchar*)(DepthRGB.data + offset))[x*3+2] = n;
				}/*else{
					((uchar*)(DepthRGB.data + offset))[x*3+0] = 0;
					((uchar*)(DepthRGB.data + offset))[x*3+1] = 0;
					((uchar*)(DepthRGB.data + offset))[x*3+2] = 0;
				}*/
			}
		}
	}

	cv::rectangle(DepthRGB, Rect(cropL, cropT, cropR-cropL+1, cropB-cropT+1), CV_RGB(255,0,0));
}
void TYKinect :: SaveImages(){
	sprintf(outputName,"output/%d_depth.png",outputIdx);
	imwrite(outputName, DepthRGB);
	sprintf(outputName,"output/%d_color.png",outputIdx);
	imwrite(outputName, ColorRGB);
	sprintf(outputName,"output/%d_raw.txt",outputIdx);

	FILE *outfile;
	errno_t err = fopen_s(&outfile, outputName,"w");
	if(err != 0){
		printf("cannot open file!!\n");
		exit(0);
	}
	for (int y = 0 ; y < height ; y++) {
		for (int x = 0 ; x < width ; x++) {
			int val = ((unsigned short*)(DepthRAW.data + y*wStep16UC1))[x];
			fprintf(outfile, "%d ", val);
		}
		fprintf(outfile, "\n");
	}
	fclose(outfile);

	printf("Image set %d saved. \n",outputIdx++);
}
void TYKinect :: setCrop(int n, int f, int l, int r, int t, int b){
	if(l != -1) cropL = l;
	if(r != -1) cropR = r;
	if(t != -1) cropT = t;
	if(b != -1) cropB = b;
	if(n != -1) cropN = n;
	if(f != -1) cropF = f;
}
void TYKinect :: RegisterKey(int key){
	
	switch(opMode){
	case OPMODE_NORMAL:
		if(key==27){
			Stop();
		}else if(key == 'm'){
			opMode = OPMODE_SELECT;
		}else if(key == 'q'){
			motorUp();
		}else if(key == 'w'){
			motorDown();
		}else if(key == 'l'){
			setLed(0);
		}else if(key == 's'){
			CreateDirectory("output",NULL);
			SaveImages();
		}
		break;
	case OPMODE_CROP:
		if(key==27){
			Stop();
		}else if(key == 'm'){
			opMode = OPMODE_SELECT;
		}else if(key == 'A'){
			if((cropL-2) >= 0) cropL -= 2;
		}else if(key == 'a'){
			if((cropL+2) < cropR) cropL += 2;
		}else if(key == 'D'){
			if((cropR+2) <= (unsigned)(width - 1)) cropR += 2;
		}else if(key == 'd'){
			if((cropR-2) > cropL) cropR -= 2;
		}else if(key == 'W'){
			if((cropT-2) >= 0) cropT -= 2;
		}else if(key == 'w'){
			if((cropT+2) < cropB) cropT += 2;
		}else if(key == 'S'){
			if((cropB+2) <= (unsigned)(height - 1)) cropB += 2;
		}else if(key == 's'){
			if((cropB-2) > cropT) cropB -= 2;
		}else if(key == 'Z'){
			if((cropN-10) > 0) cropN -= 10;
		}else if(key == 'z'){
			if((cropN+10) < cropF) cropN += 10;
		}else if(key == 'X'){
			if((cropF+10) < 10000) cropF += 10;
		}else if(key == 'x'){
			if((cropF-10) > cropN) cropF -= 10;
		}
		if(key == 'w' || key == 'a' || key == 's' || key == 'd' || key == 'z' || key == 'x' ||
		   key == 'W' || key == 'A' || key == 'S' || key == 'D' || key == 'Z' || key == 'X' )
		printf("Crop(N, F, L, R, T, B)=(%d,%d,%d,%d,%d,%d)\n",cropN,cropF,cropL,cropR,cropT,cropB);
		break;
	case OPMODE_SELECT:
		if(key == 'n')		opMode = OPMODE_NORMAL;
		else if(key == 'c')	opMode = OPMODE_CROP;
		
		break;
	}
	
}
void TYKinect :: switchMirroring(){
	context.SetGlobalMirror(!context.GetGlobalMirror());
}