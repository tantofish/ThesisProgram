#include "TyDemoModel.h"


bool TYDemoModel::loadAlfred(){
	alfred.Load("./model_show/alfred/alfred.3ds", 1);
	alfred.rot.y += 90;

	return true;
}

bool TYDemoModel::loadSkull(){
	skull.Load("./model_show/skull/skull.3ds", 0);
	skull.rot.y += 180;

	return true;
}

bool TYDemoModel::loadCSIE(){
	return csie.loadFromFile("./model_show/tri/csie.tri");
}

bool TYDemoModel::loadHead(){
	return head.loadFromFile("./model_show/head/head.obj");
}

bool TYDemoModel::loadLee(){
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	LeeHead.init(	"model_show/lee/combine_all.obj", 
					"model_show/lee/try_combine.png", 
					"model_show/lee/DrLee_Head_00_op_normal.png",
					"model_show/lee/DrLee_Head_00_op_Sp_1.png",
					"model_show/lee/DrLee_Head_00_op_normal_difr.png",
					"model_show/lee/DrLee_Head_00_op_normal_difg.png",
					"model_show/lee/DrLee_Head_00_op_normal_difb.png");
	LeeHead.initShader("model_show/lee/head.vs", "model_show/lee/head.fs");
	glPopAttrib();
	return true;
}

void TYDemoModel::draw(){
	float scale = 0.5f;
	
	
	if(currentModel == 0){
		glUseProgram(0);
		glPushMatrix();
		scale = 800.f;
		glRotatef(180, 0, 1, 0);
		glScalef(scale, scale, scale);
		glTranslatef(-0.f, -1.728f, -0.1256f);
		glColor3f(0.5, 0.5, 0.5);
		glBegin(GL_TRIANGLES);
		for(unsigned int i = 0 ; i < head.tList.size() ; i++){
			/* Here's some dirty work, we have some trouble dealing with "head.obj" in FLAT shading */
			if(i > 637 && i < 700){
				continue;
			}
			if(i > 1505 && i < 1560){
				continue;
			}
			/* The above code just filtered some triangles out , nothing's spacial. */

			glNormal3f(head.tList[i].normal[0][0],
				head.tList[i].normal[0][1],
				head.tList[i].normal[0][2]);
			glVertex3f(head.tList[i].vertex[0][0],
				head.tList[i].vertex[0][1],
				head.tList[i].vertex[0][2]);
			glNormal3f(head.tList[i].normal[1][0],
				head.tList[i].normal[1][1],
				head.tList[i].normal[1][2]);
			glVertex3f(head.tList[i].vertex[1][0],
				head.tList[i].vertex[1][1],
				head.tList[i].vertex[1][2]);
			glNormal3f(head.tList[i].normal[2][0],
				head.tList[i].normal[2][1],
				head.tList[i].normal[2][2]);
			glVertex3f(head.tList[i].vertex[2][0],
				head.tList[i].vertex[2][1],
				head.tList[i].vertex[2][2]);
		}
		glEnd();
		//glColor4f(currentColor[0],currentColor[1],currentColor[2],currentColor[3]);
		glPopMatrix();
	}else if(currentModel == 1){
		// Lee	
		LeeHead.drawObject();
	}else if(currentModel ==2){
		// CSIE
		glDisable(GL_LIGHTING);
		glPushMatrix();
		glRotatef(180, 0.0f, 0.0f, 1.0f);
		glRotatef(90, 1.0f, 0.0f, 0.0f);
		glScalef(scale*0.5,scale,scale);
		glTranslatef(-csie.center[0], -csie.center[1], -csie.center[2]);
		glBegin(GL_TRIANGLES);
		for(unsigned int i = 0 ; i < csie.triangleList.size() ; i++){
			GLfloat r = (float) csie.triangleList[i].foreColor[0]/255.f;
			GLfloat g = (float) csie.triangleList[i].foreColor[1]/255.f;
			GLfloat b = (float) csie.triangleList[i].foreColor[2]/255.f;
			glColor3f(r, g, b);
		
			for(int j = 0 ; j < 3 ; j++){
				GLfloat nx = (float) csie.triangleList[i].normal[j][0];
				GLfloat ny = (float) csie.triangleList[i].normal[j][1];
				GLfloat nz = (float) csie.triangleList[i].normal[j][2];
				glNormal3f(nx, ny, nz);

				GLfloat x = (float) csie.triangleList[i].vertex[j][0];
				GLfloat y = (float) csie.triangleList[i].vertex[j][1];
				GLfloat z = (float) csie.triangleList[i].vertex[j][2];
				glVertex3f(x, y, z);
			}
		}
		glEnd();
		glPopMatrix();
		glEnable(GL_LIGHTING);
	}else if(currentModel == 3){
		// SKULL
		glColor4f(0.8,0.8,0.8,1.0);
		glEnable(GL_LIGHTING);
		glUseProgram(0);
		skull.Draw();	
	}
}

void TYDemoModel::next(){
	currentModel = (currentModel+1)%4;
	//currentModel = 1;
}