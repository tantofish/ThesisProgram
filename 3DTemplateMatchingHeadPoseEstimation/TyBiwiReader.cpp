#include "TyBiwiReader.h"

TYBiwiDB::TYBiwiDB(char* path){
	// index boundary of each data set
	range[1].first = 3;		range[1].second = 501;
	range[2].first = 3;		range[2].second = 513;
	range[3].first = 3;		range[3].second = 674;
	range[4].first = 3;		range[4].second = 746;
	range[5].first = 3;		range[5].second = 948;
	range[6].first = 0;		range[6].second = 364;
	range[7].first = 3;		range[7].second = 747;
	range[8].first = 3;		range[8].second = 774;
	range[9].first = 3;		range[9].second = 884;
	range[10].first = 3;	range[10].second = 728;
	range[11].first = 3;	range[11].second = 574;
	range[12].first = 3;	range[12].second = 734;
	range[13].first = 3;	range[13].second = 487;
	range[14].first = 3;	range[14].second = 799;
	range[15].first = 0;	range[15].second = 582;
	range[16].first = 0;	range[16].second = 853;
	range[17].first = 0;	range[17].second = 351;
	range[18].first = 0;	range[18].second = 529;
	range[19].first = 0;	range[19].second = 281;
	range[20].first = 0;	range[20].second = 100;
	range[21].first = 0;	range[21].second = 218;
	range[22].first = 0;	range[22].second = 664;
	range[23].first = 0;	range[23].second = 512;
	range[24].first = 0;	range[24].second = 424;

	crop[1] = Vec6f(500, 1000, 86, 560, 80, 380);
	crop[2] = Vec6f(500, 1000, 86, 560, 80, 374);
	crop[3] = Vec6f(500, 1040, 86, 560, 80, 356);
	crop[4] = Vec6f(600, 1200, 86, 560, 80, 328);
	crop[5]= Vec6f(600, 1200, 86, 560, 80, 362);
	crop[6] = Vec6f(500, 1500, 80, 560, 80, 392);
	crop[7] = Vec6f(500, 1500, 80, 560, 80, 332);
	crop[8] = Vec6f(500, 1500, 80, 560, 80, 340);
	crop[9] = Vec6f(500, 1500, 80, 560, 80, 364);
	crop[10] = Vec6f(500, 1500, 80, 560, 80, 366);
	crop[11] = Vec6f(500, 1500, 80, 560, 80, 292);
	crop[12] = Vec6f(500, 1500, 80, 560, 80, 326);
	crop[13] = Vec6f(500, 1500, 80, 560, 80, 326);
	crop[14] = Vec6f(500, 1500, 80, 560, 80, 362);
	crop[15] = Vec6f(500, 1500, 80, 560, 80, 396);
	crop[16] = Vec6f(500, 1500, 80, 560, 80, 348);
	crop[17] = Vec6f(500, 1500, 80, 438, 80, 384);
	crop[18] = Vec6f(500, 1500, 80, 510, 80, 394);
	crop[19] = Vec6f(500, 1500, 80, 560, 80, 368);
	crop[20] = Vec6f(500, 1500, 80, 560, 80, 350);
	crop[21] = Vec6f(500, 1500, 80, 560, 80, 382);
	crop[22] = Vec6f(500, 1500, 80, 560, 80, 354);
	crop[23] = Vec6f(500, 1500, 80, 560, 80, 392);
	crop[24] = Vec6f(500, 1500, 80, 560, 80, 354);


	// Mat initial
	DepthRAW.create(480, 640, CV_16UC1);
	DepthRGB.create(480, 640, CV_8UC3);
	ColorRGB.create(480, 640, CV_8UC3);

	// root path setting
	strcpy(rootPath, path);

	// Variables Initial
	sIdx = 1;
	fIdx = 0;

	isFirstFrame = true;

	bOffset = 0;
	aOffset = -100;
}

void TYBiwiDB::Init(int sIndex, int fIndex){
	sIdx = sIndex; 
	fIdx = fIndex;
	if (fIdx < range[sIdx].first)	fIdx = range[sIdx].first;
}

bool TYBiwiDB::loadDepthImageCompressed(cv::Mat &img, const char* fname ){
	
	//now read the depth image
	FILE* pFile = fopen(fname, "rb");
	if(!pFile){
		//std::cerr << "could not open file " << fname << std::endl;
		return false;
	}

	int im_width = 0;
	int im_height = 0;
	bool success = true;

	success &= ( fread(&im_width,sizeof(int),1,pFile) == 1 ); // read width of depthmap
	success &= ( fread(&im_height,sizeof(int),1,pFile) == 1 ); // read height of depthmap

	
	//unsigned short* depth_img = new int16_t[im_width*im_height];
	img.create(im_height, im_width, CV_16U);

	int numempty;
	int numfull;
	int p = 0;
	unsigned short x, y, d;

	while(p < im_width*im_height ){

		success &= ( fread( &numempty,sizeof(int),1,pFile) == 1 );

		for(int i = 0; i < numempty; i++)
			((unsigned short*)(img.data))[p+i] = 0;

		success &= ( fread( &numfull,sizeof(int), 1, pFile) == 1 );
		success &= ( fread( &((unsigned short*)(img.data))[ p + numempty ], sizeof(unsigned short), numfull, pFile) == (unsigned int) numfull );
		
		for(int myI = p + numempty ; myI < p + numempty + numfull ; myI++){
			x = myI%640;
			y = myI/640;
			d = ((unsigned short*)(img.data))[ myI ];
			//if ( d < cropN || d > cropF || x < cropL || x > cropR || y < cropT || y > cropB )
			if ( d < crop[sIdx][0] || d > crop[sIdx][1] || x < crop[sIdx][2] || x > crop[sIdx][3] || y < crop[sIdx][4] || y > crop[sIdx][5] )
				((unsigned short*)(img.data))[ myI ] = 0;
		}
		
		p += numempty+numfull;

	}

	fclose(pFile);

	if(success){
		//printf("read success\n");
		return true;
	}
	else{
		printf("read fail\n");
		return false;
	}
}

bool loadPose(float *pose, const char *fname, Mat &pm, Point3f &nose){
	FILE* pFile = fopen(fname, "rb");
	if(!pFile){
		std::cerr << "could not open file " << fname << std::endl;
		return false;
	}
	
	pm.create(3, 3, CV_32F);
	fscanf(pFile, "%f %f %f %f %f %f %f %f %f %f %f %f",
		(((float*)(pm.data))+0), (((float*)(pm.data))+1), (((float*)(pm.data))+2),
		(((float*)(pm.data))+3), (((float*)(pm.data))+4), (((float*)(pm.data))+5),
		(((float*)(pm.data))+6), (((float*)(pm.data))+7), (((float*)(pm.data))+8),
		&(nose.x), &(nose.y), &(nose.z));

	nose.x *= -1.f;
	nose.y *= -1.f;
	Mat p;	p.create(3, 1, CV_32F);
	p.at<float>(0,0) = 0;
	p.at<float>(1,0) = 0;
	p.at<float>(2,0) = -1;
	Mat rp;	p.create(3, 1, CV_32F);	// inversely rotated point
	Mat pm_inv = pm.inv();
	cv::gemm(pm_inv, p, 1, NULL, 0, rp);
	
	nose.x += rp.at<float>(0,0)*55;
	nose.y += rp.at<float>(1,0)*55;
	nose.z += rp.at<float>(2,0)*55;

	pose[1] = -asin(pm.at<float>(1,2));
	pose[0] = asin(pm.at<float>(0,2)/cos(pose[1]));
	pose[2] = acos(pm.at<float>(1,1)/cos(pose[1]));

	/*
	float m[3][3];
	fscanf(pFile, "%f %f %f %f %f %f %f %f %f",
		&(m[0][0]), &(m[0][1]), &(m[0][2]),
		&(m[1][0]), &(m[1][1]), &(m[1][2]),
		&(m[2][0]), &(m[2][1]), &(m[2][2]));

	pose[1] = -asin(m[1][2]);
	pose[0] = asin(m[0][2]/cos(pose[1]));
	pose[2] = acos(m[1][1]/cos(pose[1]));
	*/


	pose[0] = pose[0]*180/3.1415926;
	pose[1] = pose[1]*180/3.1415926;
	pose[2] = pose[2]*180/3.1415926;

	//printf("(%f, %f, %f)\n", pose[0], pose[1], pose[2]);
	fclose(pFile);
	return true;
}

void TYBiwiDB::RAW2RGB(){
	// depth RGB image visualization (using histogram)

	/*---------------------*/
	/* Histogram Establish */
	/*---------------------*/
	for (int i = 0 ; i < 10000 ; i++)	histogram[i] = 0.f;

	for (int p = 0 ; p < 480*640; p++){
		int val = ((unsigned short*)(DepthRAW.data))[p];
		if (val > 10000 || val < 1){
			((unsigned short*)(DepthRAW.data))[p] = 0;
		}else{
			histogram[val] += 1.f;
		}
	}

	// accumulate histogram
	for (int i=1 ; i < 10000; i++) {
		histogram[i] += histogram[i-1];
	}

	int nPoints = histogram[9999];

	if(nPoints) {
		float factor = 1.f/(float)nPoints;
		for (int i = 1 ; i < 10000 ; i++) {
			histogram[i] = (unsigned int)(256 * (1.0f - (histogram[i] * factor)));
		}
	}



	
	/*---------------------*/
	/* Depth 2 RGB Mapping */
	/*---------------------*/
	int wStep16UC1 = DepthRAW.step1()*2;
	int wStep8UC3  = DepthRGB.step1();

	int offset;
	for (int y = 0 ; y < 480 ; y++) {
		offset = y * wStep8UC3;
		for (int x = 0 ; x < 640 ; x++) {
			int val = ((unsigned short*)(DepthRAW.data + y*wStep16UC1))[x];
			if (val != 0){
				int n = histogram[val];
				((uchar*)(DepthRGB.data + offset))[x*3+0] = 255;
				((uchar*)(DepthRGB.data + offset))[x*3+1] = n;
				((uchar*)(DepthRGB.data + offset))[x*3+2] = n;
			}else{
				((uchar*)(DepthRGB.data + offset))[x*3+0] = 0;
				((uchar*)(DepthRGB.data + offset))[x*3+1] = 0;
				((uchar*)(DepthRGB.data + offset))[x*3+2] = 0;
			}
		}
	}
}

bool TYBiwiDB::getNextFrame(){
	if(sIdx > 24 || sIdx < 1){
		cout << "Biwi.getNextFrame Err: sIdx = " << sIdx << endl;
		return false;
	}
	
	// set index parsing (int -> char*)
	if		( sIdx > 9 )	sprintf(sIndex,"%d", sIdx);
	else if	( sIdx > 0 )	sprintf(sIndex,"0%d", sIdx);

	// frame index parsing (int -> char*)
	if		( fIdx > 9999)	sprintf(fIndex,"%d", fIdx);
	else if	( fIdx > 999)	sprintf(fIndex,"0%d", fIdx);
	else if	( fIdx > 99)	sprintf(fIndex,"00%d", fIdx);
	else if	( fIdx > 9)		sprintf(fIndex,"000%d", fIdx);
	else if	( fIdx > 0)		sprintf(fIndex,"0000%d", fIdx);
	else if	( fIdx ==0)		sprintf(fIndex,"00000");


	// read depth image
	sprintf(fileName, "%s\\%s\\frame_%s_depth.bin", rootPath, sIndex, fIndex);
	
	//sprintf(name,"db\\%s\\frame_%s_depth.bin", sIndex, fIndex);
	if(!loadDepthImageCompressed(DepthRAW, fileName)){
		cout << "Biwi.getNextFrame Err: Load Depth Img Failed, (s,f) = (" << sIdx << "," << fIdx << ")" << endl;
		return false;
	}

	// visualize the DepthRAW image into displayable format
	this->RAW2RGB();
	
	// read pose parameters
	sprintf(fileName, "%s\\%s\\frame_%s_pose.txt", rootPath, sIndex, fIndex);
	if(!loadPose(pose, fileName, pm, nose)){
		cout << "Biwi.getNextFrame Err: Load Pose Failed, (s,f) = (" << sIdx << "," << fIdx << ")" << endl;
		return false;
	}
	

#ifdef BIWI_READ_COLOR
	// read color image
	sprintf(fileName, "%s\\%s\\frame_%s_rgb.png", rootPath, sIndex, fIndex);
	ColorRGB = cv::imread(fileName);
#endif

	// set next iteration set index and frame index
	if(fIdx < range[sIdx].second){
		fIdx++;
	}
	else{
		this->nextSet();
	}
}

void TYBiwiDB::Show(){
	char text[100];

	sprintf(text, "set#%s  frame#%s  ", sIndex, fIndex);
	cv::putText(DepthRGB, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(255,255,0), 1);
	sprintf(text, "Biwi result : Yaw:%2.2f, Pitch:%2.2f, Roll:%2.2f  ", pose[0], pose[1], pose[2]);
	cv::putText(DepthRGB, text, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(255,255,0), 1);
	
	

	imshow("Biwi DB depth",DepthRGB);
	
#ifdef BIWI_READ_COLOR
	imshow("Biwi DB color",ColorRGB);
#endif
}

void TYBiwiDB::setCrop(int n, int f, int l, int r, int t, int b){
	cropL = l;	cropR = r;	cropT = t;	cropB = b;
	cropN = n;	cropF = f;
}

void TYBiwiDB::nextSet(){
	if(sIdx < 24)	
		sIdx++;
	else	
		sIdx=1;
	fIdx = range[sIdx].first;

	this->isFirstFrame = true;
}

void TYBiwiDB::RegisterKey(int key){
	
	if(key==27){
		exit(0);
	}else if(key == 'A'){
		if((cropL-2) >= 0) cropL -= 2;
	}else if(key == 'a'){
		if((cropL+2) < cropR) cropL += 2;
	}else if(key == 'D'){
		if((cropR+2) <= (unsigned int)639) cropR += 2;
	}else if(key == 'd'){
		if((cropR-2) > cropL) cropR -= 2;
	}else if(key == 'W'){
		if((cropT-2) >= 0) cropT -= 2;
	}else if(key == 'w'){
		if((cropT+2) < cropB) cropT += 2;
	}else if(key == 'S'){
		if((cropB+2) <= (unsigned int)479) cropB += 2;
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
	}else if(key == 'l'){ //dump the crop parameters
		
		ofstream out(".\\output\\croplog.txt", ofstream::app); 
		
		if(!out) { 
			cout << "Cannot open file.\n"; 
		}else{
			out << sIndex << " ("
				<< cropN << ", "
				<< cropF << ", "
				<< cropL << ", "
				<< cropR << ", "
				<< cropT << ", "
				<< cropB << ");" <<endl;
		}
		out.close(); 
	}
	else if(key == 'n'){
		this->nextSet();
	}

	if(key == 'w' || key == 'a' || key == 's' || key == 'd' || key == 'z' || key == 'x' ||
		key == 'W' || key == 'A' || key == 'S' || key == 'D' || key == 'Z' || key == 'X' )
	printf("Crop(N, F, L, R, T, B)=(%d,%d,%d,%d,%d,%d)\n",cropN,cropF,cropL,cropR,cropT,cropB);

}

void TYBiwiDB::genPC(){
	float _f = 1.f/FOCAL_LEN;
	Mat p;	p.create(3, 1, CV_32F);
	Mat rp;	p.create(3, 1, CV_32F);	// inversely rotated point
	Mat pm_inv; pm_inv = pm.inv();
	float r, g, b;
	unsigned short dVal;

	point.clear();
	color.clear();

	sprintf(fileName, "%s\\%s\\frame_%s_rgb.png", rootPath, sIndex, fIndex);
	ColorRGB = imread(fileName);

	Point3f center(0,0,0);
	float nps = 0;
	for(int j = 0 ; j < 480 ; j++){
		for(int i = 0 ; i < 640 ; i++){
			dVal = DepthRAW.at<unsigned short>(j, i);
			if(dVal == 0)	continue;
			center.z += (float) dVal;
			center.x += ((float) (320-i))*dVal*_f;
			center.y += ((float) (240-j))*dVal*_f;
			nps+=1;
		}
	}
	center.x /= nps;
	center.y /= nps;
	center.z /= nps;

	for(int j = 0 ; j < 480 ; j++){
		for(int i = 0 ; i < 640 ; i++){
			dVal = DepthRAW.at<unsigned short>(j, i);
			if(dVal == 0)	continue;
			p.at<float>(2, 0) = (float) dVal				-center.z;
			p.at<float>(0, 0) = ((float) (320-i))*dVal*_f	-center.x;
			p.at<float>(1, 0) = ((float) (240-j))*dVal*_f	-center.y;
			
			cv::gemm(pm, p, 1, NULL, 0, rp);

			
			//point.push_back( Point3f(x,y,z) );
			if( (rp.at<float>(2, 0) - rp.at<float>(1, 0)) > bOffset)	
				continue;
			if( (rp.at<float>(2, 0) - rp.at<float>(1, 0)) < aOffset)	
				continue;
			

			point.push_back( (Point3f(rp)+center) );

			b = ColorRGB.at<Vec3b>(j, i)[0]/255.f;
			g = ColorRGB.at<Vec3b>(j, i)[1]/255.f;
			r = ColorRGB.at<Vec3b>(j, i)[2]/255.f;
			color.push_back( Vec3f(r,g,b) );

			
		}
	}
}

bool TYBiwiDB::mWriteModel(){
	
	char f_name[50];

	sprintf(f_name, ".\\models\\biwi_%s.txt", sIndex);
	ofstream out(f_name); 
	if(!out) { 
		cout << "Cannot open file.\n"; 
		return false; 
	}

	cout << "Writing output model \"" << f_name << "\"  ... ";
	int nPoints = (int) point.size();
	out << nPoints << endl;

	for(int i = 0 ; i < nPoints ; i++){
		out << point[i].x << " "
			<< point[i].y << " "
			<< point[i].z << " "
			<< color[i][0]<< " "
			<< color[i][1]<< " "
			<< color[i][2]<< endl;
	}
	out.close(); 

	cout << "Done!" << endl;
	return true;
}