#include "main.h"
#include "ObjRead.h"
#include "textfile.h"

#pragma once
using namespace std;

class Shader
{
	public:
	GLuint vShader,fShader,prog;
	Shader()
	{
	}
	~Shader()
	{
	}
	void load(const char *vspath, const char *fspath)
	{
		char *vsSource,*fsSource;
		vsSource=textFileRead(vspath);
		fsSource=textFileRead(fspath);
		vShader=glCreateShader(GL_VERTEX_SHADER);
		fShader=glCreateShader(GL_FRAGMENT_SHADER);
		const char* vsSc=vsSource;
		const char* fsSc=fsSource;
		glShaderSource(vShader,1,&vsSc,NULL);
		glShaderSource(fShader,1,&fsSc,NULL);
		free(vsSource);
		free(fsSource);
		glCompileShader(vShader);
		glCompileShader(fShader);
		prog=glCreateProgram();
		glAttachShader(prog,vShader);
		glAttachShader(prog,fShader);
		glLinkProgram(prog);
	}
	void useProgram()
	{
		glUseProgram(prog);
	}

};

class ObjectDisplay
{
	/*
	an ObjectDisplay represent a model, we will read a .obj file to get the information 
	like vertex, texture coordinate...etc and read some picture via openCV as the texture of this model.
	there are two mode in this display object, 
	one would use some texture pictures as normal map
	the other would use model geometry to calculate normal direction
	*/
private:
	//object data
	string objName;//obj file name
	vector <string> textureName;//input texture name, including normal texture
	bool HeadOrNot;//in some function, we need this variable to decide what kind of action should be taken
	bool transparent;//for some special material like glass
	int alpha;//one alpha value for all object

	Shader ObjectShader;//GLSL shader for the object

	//geometry data array
	GLdouble *vertex;
	GLdouble *texCoord;
	GLdouble *normal;
	//GLuint XXX: the same number as normal and if we want to pass this variable to GLSL,
	//then it should not be a structure such as array, matrix...
	GLuint texColor, texDif_Rnor, texDif_Gnor, texDif_Bnor, texSp, texSp_nor;

	bool loadModel();
	bool loadTexture();
	bool loadTexture(string picAlphaName);

	//function to set up data
	void setModelArray(GLdouble *& target, const ObjRead * source, char mode);
	void setOneTexture(const char * fileName, GLuint * textureId);
	void ConvertFromIplImageToRGBA(unsigned char * destination, const IplImage * source, int width, int height);
	void BindTexture(GLuint * textureId, const GLvoid * pixels, GLsizei width, GLsizei height);
	void activeForGLSL(GLchar *texName, GLuint * textureId, Shader *S, int num);

public:
	float center[3];// the center coord. of this model
	//openGL transformation
	GLfloat translation[3];
	GLfloat rotation[3];
	GLfloat scale;

	int quadNum;//number of quadrilateral of the model

	void init(string fileName, string picName, string picAlphaName);
	void init(string fileName, string picName, int translucent);
	void init(string fileName, string picName);
	void init(string fileName, 
		      string picName1, string picName2, string picName3, string picName4, string picName5, string picName6);
	void initShader(string vertexShader, string fragmentShader);
	void drawObject();

	//this is for play animation, every time we draw a new frame, 
	//we use this function to copy the vertex array which already calculated by "animation part" to display array
	void setVertexArray(GLdouble *& source);
	void loadCurrentVertex(GLdouble *& target);
	//in case "animation" need read obj file to get vertex data
	void readVertexArray(const string objfile, GLdouble *& target);
	//rotate the eyes make the "head" vivid.
	void decideAngle();
	void setRotate(GLfloat xAxis, GLfloat yAxis);

	ObjectDisplay();
	~ObjectDisplay();
};