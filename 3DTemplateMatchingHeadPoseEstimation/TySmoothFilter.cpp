#include "TySmoothFilter.h"


TYSmoothFilter::TYSmoothFilter(int filterIndex){
	smoothFrameNum = SMOOTH_FRAME_NUM;
	tbHead = -1;
	print_count = 0;
	start_count = 0;
	fIndex = filterIndex;
	regRawInHTB = true;

	prePose = Vec6f(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
	curPose = Vec6f(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
	for(int i = 0 ; i < smoothFrameNum ; i++)	HTB[i] = Vec6f(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);

}

void TYSmoothFilter::reset(){
	tbHead = -1;
	print_count = 0;
	start_count = 0;
	prePose = Vec6f(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
	curPose = Vec6f(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
	for(int i = 0 ; i < smoothFrameNum ; i++)	HTB[i] = Vec6f(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
}

String TYSmoothFilter::nextFilter(){
	fIndex = (fIndex + 1) % 3;

#ifdef SMOOTH_MODE_PRINTF
#endif

	switch(fIndex){
	case 0:
		return String("Smooth Filter Mode: Dynamic Weighted Average Filter.\n");
		break;
	case 1:
		return String("Smooth Filter Mode: Simple Average Filter.\n");
		break;
	case 2:
		return String("Smooth Filter Mode: No Filter.\n");
		break;
	default:
		return String("\n");
		break;
	}

}

Vec6f TYSmoothFilter::getCurrentPose(){
	return curPose;
}

Vec6f TYSmoothFilter::getHTB(int i){
	return HTB[tbHead+i];
}

Vec6f TYSmoothFilter::smoothing(Vec6f &poseIn){

	/*---------------------*/
	/* Smooth Processing   */
	/*---------------------*/
	tbHead = (tbHead + 1) % smoothFrameNum;
	HTB[tbHead] = poseIn;


	int ps = 6;	// parameters to be smoothed: 6 => (y,p,r,x,y,z) or 3 => (y,p,r)

	float factor = 0;	// sum of weights
	Vec6f sum(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
	Vec6f dist;		// pose(current) - pose(i)
	float distance;	// L2 Distance of pose(current) and pose(i)

	/* fIndex : filter mode index */
	if(fIndex == 0 && start_count > smoothFrameNum){	
	/* Dynamic Weighting Filter */

		/* Find the max distance pose from the HTB to the current pose */
		float maxDist = -100000.f;
		for(int i = 1 ; i < smoothFrameNum ; i++){
			distance = 0.f;
			for(int j = 0 ; j < ps ; j++){
				dist[j] = abs(HTB[tbHead][j] - HTB[(tbHead + i) % smoothFrameNum][j]  );
				distance += dist[j]*dist[j];
			}
			distance = sqrt(distance);
			if(maxDist < distance)	maxDist = distance;
		}
	
		float H = 0.1f;
	
		if(maxDist < 9.f){
			for(int i = 0 ; i < smoothFrameNum ; i++){
				for(int j = 0 ; j < ps ; j++){
					sum[j] += HTB[(tbHead + i) % smoothFrameNum][j];
				}
				factor += 1.f;
			}
		}else{
			//printf("DIstance: %.2f ",maxDist);
			for(int i = 0 ; i < smoothFrameNum ; i++){
				/* the key element --dynamic weight-- of this filter */
				float w = pow(2, -i * H * maxDist);
				for(int j = 0 ; j < ps ; j++){
					sum[j] += HTB[(tbHead + i) % smoothFrameNum][j] * w;
				}
				factor += w;
				//printf("%.2f ",w);
			}
			//printf("\n");
		}
	}else if(fIndex == 1 && start_count > smoothFrameNum ){
		/* Simple Average Filter */
		for(int i = 0 ; i < smoothFrameNum ; i++){
			for(int j = 0 ; j < ps ; j++){
				sum[j] += HTB[(tbHead + i) % smoothFrameNum][j];
			}
			factor += 1.f;
		}
	}else if(fIndex == 2 || start_count <= smoothFrameNum){
		/* No Filter */
		start_count++;
		sum = HTB[tbHead];
		factor = 1.f;
	}

	

	for(int j = 0 ; j < ps ; j++)
		curPose[j] = sum[j]	/ factor;

	for(int j = ps ; j < 6 ; j++)
		curPose[j] = HTB[tbHead][j];

	if(!regRawInHTB){
		for(int j = 0 ; j < 6 ; j++)
		HTB[tbHead][j] = curPose[j];
	}

	return curPose;
}
void TYSmoothFilter::toString(){
	print_count++;
	if(print_count > PRINT_FRAME_INTERVAL){
		printf("Yaw:%.1f, Pitch:%.1f, Roll:%.1f\n", curPose[0], curPose[1], curPose[2]);
		print_count = 0;
	}
}
void TYSmoothFilter::putText(char *angle_str){
	print_count++;
	if(print_count > 3){
		sprintf(angle_str, "Yaw:%.1f, Pitch:%.1f, Roll:%.1f\0", curPose[0], curPose[1], curPose[2]);
		print_count = 0;
	}
}
void TYSmoothFilter::switchRegRawInHTB(){
	regRawInHTB = !regRawInHTB;
#ifdef SMOOTH_MODE_PRINTF
	if(regRawInHTB)
		printf("TYSmoothFilter: regRawInHTB = true: Put Non-Smoothed Poses into HTB \n");
	else
		printf("TYSmoothFilter: regRawInHTB = false: Put Smoothed Poses into HTB \n");
#endif
}