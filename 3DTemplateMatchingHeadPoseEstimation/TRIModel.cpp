#include "TRIModel.h"


bool TRIModel::loadFromFile(const char* fileName){
	char tmp_string[100] = "";
	double max[3]={0.0, 0.0, 0.0};
	double min[3]={0.0, 0.0, 0.0};

	FILE* inFile = fopen(fileName, "r");
	if(!inFile){
		cout << "Can not open object File \"" << fileName << "\" !" << endl;
		return false;
	}

	
	/*float tx;
	float ty;
	float tz= -1000000;*/

	cout <<"Loading \"" << fileName << "\" !" << endl;
	while(fscanf(inFile,"%s",tmp_string) != EOF){
		double tmp_double[6];
		int tmp_int[6];
		fscanf(inFile,"%d %d %d %d %d %d",&tmp_int[0],&tmp_int[1], &tmp_int[2], &tmp_int[3], &tmp_int[4], &tmp_int[5]);
		Triangle tmp_triangle(tmp_int);
		for(int i = 0; i < 3; i++){
			fscanf(inFile,"%lf %lf %lf %lf %lf %lf",&tmp_double[0],&tmp_double[1], &tmp_double[2], &tmp_double[3], &tmp_double[4], &tmp_double[5]);
			/*if(tmp_double[0] > tz){
				tz = tmp_double[2];
				tx = tmp_double[0];
				ty = tmp_double[1];
			}*/

			for(int j = 0; j < 3; j++){
				if(tmp_double[j] < min[j]){
					min[j] = tmp_double[j];
				}
				if(tmp_double[j] > max[j]){
					max[j] = tmp_double[j];
				}
			}
			tmp_triangle.loadVertex(i, tmp_double);
		}
		triangleList.push_back(tmp_triangle);
	}
	for(int i = 0; i < 3; i++){
		center[i] = (min[i] + max[i]) / 2;
	}

	/*printf("nose!!!!!!!!!! ( %f, %f, %f)", tx, ty, tz);*/
	return true;
}
void TRIModel::copy(TRIModel * t){
	for(int i = 0; i < 3; i++){
		center[i] = t->center[i];
	}
	triangleList = t->triangleList;
}

TRIModel::TRIModel(){
}

TRIModel::~TRIModel(){
}