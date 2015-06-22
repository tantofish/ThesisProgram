#include "ObjRead.h"
#define _CRT_SECURE_NO_WARNINGS

bool ObjRead::loadFromFile(const char* fileName)
{
	int count = 0;
	char tmp_string[100] = "";
	float max[3]={0.0, 0.0, 0.0};
	float min[3]={0.0, 0.0, 0.0};
	float tmp_center[3]={0.0, 0.0, 0.0};
	eye_depth = 0;

	int numvertex=0;

	FILE* inFile = fopen(fileName, "r");
	if(!inFile)
	{
		cout << "Can not open object File \"" << fileName << "\" !" << endl;
		return false;
	}

	cout <<"Loading \"" << fileName << "\" !" << endl;
	while(fscanf(inFile,"%s",tmp_string) != EOF)
	{
		count++;

		int tmp_int[12];
		float tmp_float[3]={0.0, 0.0, 0.0};
		char tmp_char;

		LeeVertex tmp_vertex;
		LeeNormal tmp_normal;
		LeeTexture tmp_texture;
		LeeQuadrangle tmp_quadrangle;
		
		if (strcmp(tmp_string, "v") == 0) {
			fscanf(inFile,"%f %f %f", &tmp_float[0], &tmp_float[1], &tmp_float[2]);
			tmp_vertex.loadVertex(tmp_float);

			numvertex++;

			for(int j = 0; j < 3; j++)
			{
				if(tmp_float[j] < min[j])
					min[j] = tmp_float[j];
				if(tmp_float[j] > max[j])
					max[j] = tmp_float[j];

				tmp_center[j] = tmp_center[j] + tmp_float[j];
			}
			vertwxList.push_back(tmp_vertex);
		}
		else if (strcmp(tmp_string, "g") == 0) {
			fgets(tmp_string, sizeof(tmp_string), inFile);
		}
		else if (strcmp(tmp_string, "vn") == 0) {
			//for (int i = 0; i < 3; i++)
			//	fscanf(inFile, "%s", tmp_string);
			fscanf(inFile, "%f %f %f", &tmp_float[0], &tmp_float[1], &tmp_float[2]);
			tmp_normal.loadNormal(tmp_float);
			normalList.push_back(tmp_normal);
		}
		else if (strcmp(tmp_string, "vt") == 0) {
			//fscanf(inFile,"%f %f %f", &tmp_float[0], &tmp_float[1], &tmp_float[2]);
			fscanf(inFile,"%f %f", &tmp_float[0], &tmp_float[1]);
			tmp_texture.loadTexture(tmp_float);
			textureList.push_back(tmp_texture);
		}
		else if (strcmp(tmp_string, "f") == 0) {
			fscanf(inFile,"%d %c %d %c %d 	%d %c %d %c %d   %d %c %d %c %d	  %d %c %d %c %d", 
					&tmp_int[0], &tmp_char, &tmp_int[1], &tmp_char, &tmp_int[2],
					&tmp_int[3], &tmp_char, &tmp_int[4], &tmp_char, &tmp_int[5],
					&tmp_int[6], &tmp_char, &tmp_int[7], &tmp_char, &tmp_int[8],
					&tmp_int[9], &tmp_char, &tmp_int[10], &tmp_char, &tmp_int[11]);
			tmp_quadrangle.loadQuadrangle(tmp_int);
			quadrangleList.push_back(tmp_quadrangle);
		}
		else if(strcmp(tmp_string, "s") == 0){
			fgets(tmp_string, sizeof(tmp_string), inFile);
		}else{
		}
	}

	for(int i = 0; i < 3; i++){

		center[i] = tmp_center[i] / (float)numvertex;

		if(max[i] - min[i] > eye_depth)
			eye_depth = max[i] - min[i];
	}

	eye_depth = eye_depth * (float)1.6;
	return true;
}
void ObjRead::copy(ObjRead * t)
{
	for(int i = 0; i < 3; i++)
	{
		center[i] = t->center[i];
	}
	vertwxList = t->vertwxList;
	textureList = t->textureList;
	normalList = t->normalList;
	quadrangleList = t->quadrangleList;
}

ObjRead::ObjRead(){
}

ObjRead::~ObjRead(){
}
