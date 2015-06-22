#include <iostream>
#include <vector>
#pragma once

using namespace std;

class LeeVertex
{
public:
	float vertex[3];
	
	void loadVertex(float in[3])
	{
		for(int i = 0; i < 3; i++)
			vertex[i] = in[i];
	}	
};

class LeeTexture
{
public:
	float texture[3];
	
	void loadTexture(float in[3])
	{
		for(int i = 0; i < 3; i++)
			texture[i] = in[i];
	}
};

class LeeNormal
{
public:
	float normal[3];

	void loadNormal(float in[3])
	{
		for(int i = 0; i < 3; i++)
			normal[i] = in[i];
	}
};

class LeeQuadrangle
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

class ObjRead
{
public:
	vector<LeeVertex>	 vertwxList;
	vector<LeeNormal>   normalList;
	vector<LeeTexture>  textureList;
	vector<LeeQuadrangle> quadrangleList;
	
	float center[3];
	float eye_depth;

	bool loadFromFile(const char* fileName);

	void copy(ObjRead *);
	ObjRead();
	~ObjRead();

};