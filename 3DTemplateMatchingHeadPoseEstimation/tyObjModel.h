#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <vector>

#pragma once

using namespace std;

class Vertex
{
public:
	float vertex[3];
	void load(float *in)
	{
		for(int i = 0; i < 3; i++)
			vertex[i] = in[i];
	}	
};
class Normal
{
public:
	float normal[3];
	void load(float *in)
	{
		for(int i = 0; i < 3; i++)
			normal[i] = in[i];
	}	
};
class Texture
{
public:
	float texture[2];
	void load(float *in)
	{
		for(int i = 0; i < 2; i++)
			texture[i] = in[i];
	}
};
class Quadrangle
{
public:
	int quadrangle_vertex[4];
	int quadrangle_texture[4];
	int quadrangle_normal[4];
	
	void loadQuadrangle(int in[12])
	{
		for(int i = 0; i < 4; i++)
		{
			quadrangle_vertex[i] = in[3*i];
			quadrangle_texture[i] = in[3*i+1];
			quadrangle_normal[i] = in[3*i+2];
		}
	}
};

//class Quadrangle{
//public:
//	float vertex[4][3];
//	float normal[4][3];
//	float texture[4][2];
//
//	void loadVertex(int index, float in[3]){
//		for(int i = 0; i < 3; i++){
//			vertex[index][i] = in[i];
//		}
//	}
//	void loadNormal(int index, float in[3]){
//		for(int i = 0; i < 3; i++){
//			normal[index][i] = in[i];
//		}
//	}
//	void loadTexture(int index, float in[2]){
//		for(int i = 0; i < 3; i++){
//			normal[index][i] = in[i];
//		}
//	}
//};
class ObjTriangle{
public:
	float vertex[3][3];
	float normal[3][3];
	float texture[3][2];


	void loadVertex(int index, float in[3]){
		for(int i = 0; i < 3; i++){
			vertex[index][i] = in[i];
		}
	}
	void loadNormal(int index, float in[3]){
		for(int i = 0; i < 3; i++){
			normal[index][i] = in[i];
		}
	}
	void loadTexture(int index, float in[2]){
		for(int i = 0; i < 3; i++){
			normal[index][i] = in[i];
		}
	}
};
class TYObjModel
{
public:
	//attributes:
	vector<ObjTriangle>   tList;
	vector<Quadrangle> qList;
	float center[3];

	//functions:
	bool loadFromFile(const char* fileName);
	void copy(TYObjModel *);
	void toString();
	TYObjModel();
	~TYObjModel();
};