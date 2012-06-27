#ifndef _LIGHT_H_
#define _LIGHT_H_
#define GLUT_DISABLE_ATEXIT_HACK

#include "main.h"
#include "GL/glut.h"
#include <vector>
#include <math.h>
#include <iostream>

using namespace std;



class myLight{
private:
	int ID;
	float alpha, beta, r;
	float cAtt, lAtt, qAtt;
	float position[4];
	float ambient[4], diffuse[4], specular[4];
	bool On;

public:
	myLight(){
		//lightID = lightCount++;
		On = false;
		alpha = beta = 0.0f;
		r = 700;
		cAtt=1;
		lAtt=0.00001;
		qAtt=0.000000001;

		for(int i = 0 ; i < 4 ; i ++){
			ambient[i] = 0.1f;
			diffuse[i] = 1.0f;
			specular[i] = 1.0f;
		}
	}

	void SetID(int para){ ID = para; }
	void AdjPosition(float alpha, float beta, float radius){ 
		this->alpha += alpha;	if(this->alpha > 360) this->alpha -= 360;
		this->beta += beta; if(this->beta > 360) this->beta -= 360;
		this->r += radius; 
	}
	void SetAmbient(float r, float g, float b, float a){ ambient[0] = r; ambient[1] = g; ambient[2] = b; ambient[3] = a; }
	void SetDiffuse(float r, float g, float b, float a){ diffuse[0] = r; diffuse[1] = g; diffuse[2] = b; diffuse[3] = a; }
	void SetSpecular(float r, float g, float b, float a){ specular[0] = r; specular[1] = g; specular[2] = b; specular[3] = a; }

	/*0x4000 = GL_LIGHT0*/
	/*0x4001 = GL_LIGHT1*/
	/*0x4002 = GL_LIGHT2*/
	void SetGLParameters(){
		//position[0]=r * cos( toRad(alpha) ) * cos( toRad(beta) );
		//position[1]=r * cos( toRad(alpha) ) * sin( toRad(beta) );
		//position[2]=r * sin( toRad(alpha) );
		position[0]= alpha;
		position[1]= beta;
		position[2]= r;
		position[3]=1;

		glLightfv(0x4000+ID, GL_AMBIENT,ambient);
		glLightfv(0x4000+ID, GL_DIFFUSE,diffuse);
		glLightfv(0x4000+ID, GL_SPECULAR,specular);
		glLightfv(0x4000+ID, GL_POSITION,position);

		glLightf(0x4000+ID, GL_CONSTANT_ATTENUATION,cAtt);
		glLightf(0x4000+ID, GL_LINEAR_ATTENUATION,lAtt);
		glLightf(0x4000+ID, GL_QUADRATIC_ATTENUATION,qAtt);
	}
	
	void turnOnOff(){	
		if(On){
			On = false;	
			glDisable(0x4000+ID);
		}else{
			On = true;	
			glEnable(0x4000+ID);
		}
	}
	bool isOn(){ return On; }
	void toString(){
		cout << "**********Light[" << ID << "] toString***********" << endl;
		cout << "    alpha = " << alpha << ", beta = " << beta << ", radius = " << r << endl;
	}
};

class TYLight{
public:
	vector<myLight> lights;
	TYLight(){
		lights.push_back(myLight());
		lights.push_back(myLight());
		lights.push_back(myLight());
	}
	void ltInit(){
		lights[0].SetID(0);
		lights[0].AdjPosition(400, 400, 400);
		lights[0].SetGLParameters();
		lights[0].turnOnOff();

		lights[1].SetID(1);
		lights[1].AdjPosition(-200, -200, 1500);
		lights[1].SetGLParameters();
		lights[1].turnOnOff();

		lights[2].SetID(2);
		lights[2].AdjPosition(-200, -200, -200);
		lights[2].SetGLParameters();
		lights[2].turnOnOff();
	}

	void mtInit(){
		float matA[]={0.15,0.15,0.15,1};
		float matD[]={0.6,0.6,0.6,1};
		float matS[]={1,1,1,1};
		glEnable(GL_NORMALIZE);

		glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,30);
		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,matA);
		glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,matD);
		glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,matS);

		glColorMaterial(GL_FRONT_AND_BACK,GL_DIFFUSE);
		glEnable(GL_COLOR_MATERIAL);
	}

};

#endif