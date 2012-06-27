#ifndef _SMOOTH_FILTER_H_
#define _SMOOTH_FILTER_H_
#define _CRT_SECURE_NO_WARNINGS

#include "main.h"

class TYSmoothFilter{
private:
	/*--------------------*/
	/* private attributes */
	/*--------------------*/
	int print_count;
	int start_count;

	int fIndex;		// filter mode : dynamic avg, simple avg, raw
	Vec6f HTB[30];	// History table
	
	Vec6f prePose;	// Previous Pose Output (Smoothed)
	Vec6f curPose;	// Current  Pose Output (Smoothed)

	int smoothFrameNum;		// use how many frame average to smooth
	int tbHead;		// table head
	int tbTail;		// table tail

	bool regRawInHTB;	// register raw pose (non smoothed) in HTB to smooth
	/*--------------------*/
	/* private functions  */
	/*--------------------*/


public:
	
	/*  public attributes */
	
	/*  public functions  */
	
	/* Constructor, fIndex 0 : Dynamic W Avg, 1: Simple Avg, 2: No Filter */
	TYSmoothFilter(int filterIndex = 0);

	/* Black box, eat "poseIn", return smoothed pose */
	Vec6f smoothing(Vec6f &poseIn);

	/* return current smoothed pose */
	Vec6f getCurrentPose();

	/* return raw pose at frame (head+i)*/
	Vec6f getHTB(int i=0);

	/* reset */
	void reset();
	
	/* switch between put non-smoothed or smoothed pose into HTB */
	void switchRegRawInHTB();

	String nextFilter();
	void toString();
	void putText(char *angle_str);
};

#endif