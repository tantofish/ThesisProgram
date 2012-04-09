#include "TyCloudDrawer.h"
#include <cmath>
#ifdef DRAW_TIME
#include "TyTimer.h"
TYTimer ttt;
#endif

bool TYCloudDrawer::getHistogram(vector<Point3f> &cloud){
#ifdef DRAW_TIME
	ttt.timeInit();
#endif
	// Accumulated Probabilistic Histogram, normalized to [0.0 ~ 1.0], for visualization

	// Clear the Array
	for(int i = 0 ; i < MAX_DEPTH ; i++)	histogram[i] = 0.f;

	// Get the number of points (cloud size)
	unsigned int nPoints = cloud.size();
	
	// exception Message
	if(nPoints == 0){
		fprintf(stderr, "TYCloudDrawer.getHistogram() exception: Empty Cloud \n");
		return false;
	}

	// Histogram Computing
	for (int i = 0 ; i < nPoints ; i++){
		int val = (int)cloud[i].z;
		

		if(val > 0 && val < MAX_DEPTH)	histogram[val]++;
		else	fprintf(stderr, "TYCloudDrawer.getHistogram() exception: Point out of boundary (%d) \n", val);
	}
	
	// Histogram Accumulating
	for(int i=1 ; i < MAX_DEPTH; i++)	histogram[i] += histogram[i-1];
	
	// The Number of Points counted in the histogram (most of the time, it's the same as nPoints)
	float nNumberOfPoints = histogram[MAX_DEPTH-1];

	// not empty histogram
	if(nNumberOfPoints > 0.f) {
		float factor = 1.f/nNumberOfPoints;
		for(int i = 1 ; i < MAX_DEPTH ; i++) {
			histogram[i] = 1.0f - histogram[i] * factor;
		}
	}
#ifdef DRAW_TIME
	printf("hist ");
	ttt.timeReportMS();
#endif
	return true;
}

void TYCloudDrawer::drawPointCloud(vector<Point3f> &cloud){
#ifdef DRAW_TIME
	ttt.timeInit();
#endif
	unsigned int nPoints = cloud.size();
	glBegin(GL_POINTS);
	for (int i = 0 ; i < nPoints ; i++){
		int val = (int)cloud[i].z;
		
			float n = histogram[val] * 0.8;
			
			glColor3f(n, n, n);
			glVertex3f(cloud[i].x, cloud[i].y, cloud[i].z);
		
	}
	glEnd();
#ifdef DRAW_TIME
	printf("draw ");
	ttt.timeReportMS();
#endif
}