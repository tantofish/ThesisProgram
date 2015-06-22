#include <iostream>
#include <vector>
#define _CRT_SECURE_NO_WARNINGS

#pragma once

using namespace std;

class Triangle{
public:
	double vertex[3][3];
	double normal[3][3];
	int foreColor[3];
	int backColor[3];
	Triangle(int in[6]){
		for(int i = 0; i < 3; i++){
			foreColor[i] = in[i];
		}
		for(int i = 3; i < 6; i++){
			backColor[i - 3] = in[i];
		}
	}
	void loadVertex(int index, double in[6]){
		for(int i = 0; i < 3; i++){
			vertex[index][i] = in[i];
		}
		for(int i = 3; i < 6; i++){
			normal[index][i - 3] = in[i];
		}
	}
	void swap(Triangle& t){
		for(int i = 0; i < 3; i++){
			for(int j = 0; j < 3; j++){
				double tmp = this->vertex[i][j];
				this->vertex[i][j] = t.vertex[i][j];
				t.vertex[i][j] = tmp;
				tmp = this->normal[i][j];
				this->normal[i][j] = t.normal[i][j];
				t.normal[i][j] = tmp;
			}
			int tmp = this->foreColor[i];
			this->foreColor[i] = t.foreColor[i];
			t.foreColor[i] = tmp;
			tmp = this->backColor[i];
			this->backColor[i] = t.backColor[i];
			t.backColor[i] = tmp;
		}
	}
};
class TRIModel{
	

public:
	vector<Triangle> triangleList;
	double center[3];

	bool loadFromFile(const char* fileName);

	void copy(TRIModel *);
	TRIModel();
	~TRIModel();

};