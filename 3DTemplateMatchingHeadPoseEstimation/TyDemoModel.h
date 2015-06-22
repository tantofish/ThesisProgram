#ifndef _DEMO_MODEL_H_
#define _DEMO_MODEL_H_



#include <iostream>
using namespace std;


#include "main.h"
#include "Model_3DS.h"
#include "TRIModel.h"
#include "GL/glut.h"
#include "tyObjModel.h"
#include "ObjectDisplay.h"

// I'm running out of time, so that this code is writed with no flexibility and freedom.


class TYDemoModel{
public:
	TYDemoModel(){
		currentModel = 0;
	}

	Model_3DS alfred;
	Model_3DS skull;
	TRIModel csie;
	TYObjModel head;


	bool loadCSIE();
	bool loadAlfred();
	bool loadSkull();
	bool loadHead();
	bool loadLee();

	void next();
	void draw();

	int currentModel;

	ObjectDisplay LeeHead;
	ObjectDisplay Leye;
	ObjectDisplay Reye;
	ObjectDisplay hair;
	ObjectDisplay features;
};

#endif