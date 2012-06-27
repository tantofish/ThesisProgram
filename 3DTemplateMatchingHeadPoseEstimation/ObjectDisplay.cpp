#include "ObjectDisplay.h"

void ObjectDisplay::init(string fileName, string picName, string picAlphaName)
{//for the object with alpha value reading from a picture
	HeadOrNot = false;
	objName.assign(fileName);
	textureName.push_back(picName);

	//we will use alpha value when loading texture
	transparent = true;

	loadModel();
	loadTexture(picAlphaName);
}
void ObjectDisplay::init(string fileName, string picName, int translucent)
{//for the object with translucent value (such as glasses lenses)
	HeadOrNot = false;
	objName.assign(fileName);
	textureName.push_back(picName);

	//we will use alpha value when loading texture
	transparent = true;
	alpha = translucent;

	loadModel();
	loadTexture();
}
void ObjectDisplay::init(string fileName, string picName)
{//for the object without translucent and using one picture as texture
	HeadOrNot = false;
	objName.assign(fileName);
	textureName.push_back(picName);

	//we will use alpha value when loading texture
	transparent = false;
	alpha = 255;

	loadModel();
	loadTexture();
}
void ObjectDisplay::init(string fileName, 
						 string color, string sp_normal, string sp, 
						 string dif_R, string dif_G, string dif_B)
{//special for the Head model which all normals are from texture pictures.
	HeadOrNot = true;

	objName.assign(fileName);

	textureName.push_back(color);//texture color
	textureName.push_back(sp_normal);//specular, normal map
	textureName.push_back(sp);//specular
	textureName.push_back(dif_R);//diffuse R channel, normal map
	textureName.push_back(dif_G);//diffuse G channel, normal map
	textureName.push_back(dif_B);//diffuse B channel, normal map

	//we will use alpha value when loading texture
	transparent = false;
	alpha = 255;

	loadModel();
	loadTexture();
}
void ObjectDisplay::initShader(string vertexShader, string fragmentShader)
{
	ObjectShader.load(vertexShader.c_str(), fragmentShader.c_str());
}
void ObjectDisplay::drawObject()
{
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glEnable( GL_LIGHTING);
	glEnable( GL_TEXTURE_2D );
	glEnable( GL_DEPTH_TEST );
	//glDisable(GL_LIGHT1);

	if( transparent ){
		glEnable( GL_BLEND );
		glEnable( GL_ALPHA_TEST );
		glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );//GL_ONE_MINUS_SRC_ALPHA
		glDepthMask(GL_FALSE);
		//glDisable( GL_CULL_FACE );
	}

	ObjectShader.useProgram();

	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glEnableClientState(GL_VERTEX_ARRAY);

	activeForGLSL("texColor", &texColor, &ObjectShader, 0);
	activeForGLSL("texSp_nor", &texSp_nor, &ObjectShader, 1);
	activeForGLSL("texSp", &texSp, &ObjectShader, 2);
	activeForGLSL("texDif_Rnor", &texDif_Rnor, &ObjectShader, 3);
	activeForGLSL("texDif_Gnor", &texDif_Gnor, &ObjectShader, 4);
	activeForGLSL("texDif_Bnor", &texDif_Bnor, &ObjectShader, 5);
	

	glTranslatef(center[0], center[1], center[2]+50);
	
	glTranslatef(translation[0], translation[1], translation[2]);
	glRotatef(rotation[1]+180, 0.0, 1.0, 0.0);
	glRotatef(rotation[0], 1.0, 0.0, 0.0);
	glScalef(scale*8, scale*8, scale*8);

	glTranslatef(-center[0], -center[1], -center[2]);

	glTexCoordPointer(2, GL_DOUBLE, 0, texCoord);
	glVertexPointer(3, GL_DOUBLE, 0, vertex);
	glDrawArrays(GL_QUADS, 0, quadNum*4);

	//we use GL_QUADS to draw the model, 
	//because we initial everything(vertex array, readfromfile...) in "quad" mode.

    if( transparent ){
		glDisable(GL_BLEND);
		glDisable(GL_ALPHA_TEST);
		glDepthMask(GL_TRUE);
		//glEnable( GL_CULL_FACE );
	}

	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);
	//glDisableClientState(GL_NORMAL_ARRAY);

	glPopAttrib();
	glPopMatrix();
}
bool ObjectDisplay::loadTexture()
{
	if(HeadOrNot){
		setOneTexture(textureName.at(0).c_str(), &texColor);
		setOneTexture(textureName.at(1).c_str(), &texSp_nor);
		setOneTexture(textureName.at(2).c_str(), &texSp);
		setOneTexture(textureName.at(3).c_str(), &texDif_Rnor);
		setOneTexture(textureName.at(4).c_str(), &texDif_Gnor);
		setOneTexture(textureName.at(5).c_str(), &texDif_Bnor);
	}else{
		setOneTexture(textureName.at(0).c_str(), &texColor);
	}
	return true;
}
bool ObjectDisplay::loadTexture(string picAlphaName)
{
	cout <<"Loading \"" << picAlphaName << "\" !" << endl;
	IplImage * alphaImage;
	alphaImage = cvLoadImage(picAlphaName.c_str(), 0);

	//do setOneTexture but change some step in ConvertFromIplImageToRGBA
	IplImage * image;
	image = cvLoadImage(textureName.at(0).c_str(), 1);
	unsigned char * pixels = new unsigned char [4 * image->width * image->height];
	//do ConvertFromIplImageToRGBA with alpha value loading from picture
	for(int i = 0; i < image->height; i++) 
	{
		const char * src = image->imageData + i*image->widthStep;
		for(int j = 0; j < image->width; j++)
		{
			pixels[(image->height-i-1)*image->width*4 + j*4 + 3] = alphaImage->imageData[i*alphaImage->widthStep+(j)];

			for(int k = 0; k < 3; k++)
				pixels[(image->height-i-1)*image->width*4 + j*4 + k] = static_cast<unsigned char>(src[j*3 + (2-k)]);
		}
	}
	//
	BindTexture(&texColor, pixels, image->width, image->height);
	delete pixels;
	cvReleaseImage(&image);
	cvReleaseImage(&alphaImage);

	return true;
}
void ObjectDisplay::setOneTexture(const char * fileName, GLuint * textureId)
{
	cout <<"Loading \"" << fileName << "\" !" << endl;
	// 1. Load image via the OpenCV lib.
	IplImage * image;
	image = cvLoadImage(fileName, 1);
	// 2. Convert the result of step 1 to OpenGL format.
	unsigned char * pixels = new unsigned char [4 * image->width * image->height];
	ConvertFromIplImageToRGBA(pixels, image, image->width, image->height);
	// 3. Bind the texture to OpenGL
	BindTexture(textureId, pixels, image->width, image->height);
	// 4. Release the data array and IplImage.
	delete pixels;
	cvReleaseImage(&image);
}
void ObjectDisplay::ConvertFromIplImageToRGBA(unsigned char * destination,
                          const IplImage * source, int width, int height)
{
	// Initialize all elements in desination to value alpha,
	// this is very important since that the source data
	// do not contains the alpha channel information. so we indicate it by a default value
	for(int m = 0; m < width*height*4; m++)
		destination[m] = (unsigned char)alpha;
	// Copy the source data to into destination,
	// be sure to convert from BGR format to RGBA.
	for(int i = 0; i < height; i++) 
	{
		// Forward to the beginning of row i.
		const char * src = source->imageData + i*source->widthStep;
		for(int j = 0; j < width; j++)
			for(int k = 0; k < 3; k++)	// color channel
				destination[(height-i-1)*width*4 + j*4 + k] = static_cast<unsigned char>(src[j*3 + (2-k)]);
	}
}
void ObjectDisplay::BindTexture(GLuint * textureId, const GLvoid * pixels,
						 GLsizei width, GLsizei height)
{
	glGenTextures(1, textureId);
	glBindTexture(GL_TEXTURE_2D, *textureId);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	//glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);//GL_MODULATE
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); 
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);

}
void ObjectDisplay::activeForGLSL(GLchar *texName, GLuint *textureId, Shader *S, int num)
{
	glActiveTexture(GL_TEXTURE0 + num);   
	int location = glGetUniformLocation(S->prog, texName);  
	glUniform1i(location, num);  
	glBindTexture(GL_TEXTURE_2D, *textureId);
}
bool ObjectDisplay::loadModel()
{
	ObjRead *inModel = new ObjRead[1];
	inModel[0].loadFromFile( objName.c_str() );//read model obj file

	quadNum = (int) inModel[0].quadrangleList.size();//set up some property for the model

	for(int i=0; i<3; i++)
		center[i] = inModel[0].center[i];

	if(HeadOrNot){
		setModelArray(vertex, &inModel[0] , 'v');
		setModelArray(texCoord, &inModel[0] , 't');
	}
	else{
		setModelArray(vertex, &inModel[0] , 'v');
		setModelArray(normal, &inModel[0] , 'n');
		setModelArray(texCoord, &inModel[0] , 't');
	}

	delete [] inModel;
	return true;
}
void ObjectDisplay::setModelArray(GLdouble *& target, const ObjRead * source, char mode)
{
	// set node_array('v'), normal_array('n'), texture_array('t') from ObjRead file
	int tmp=0;
	switch(mode){
	case 'v':
		target = new GLdouble[quadNum * 12];
		for(int i=0; i<quadNum; i++){
			for(int j=0; j<4; j++){
				for(int k=0; k<3; k++){
					target[tmp] = source->vertwxList.at(source->quadrangleList.at(i).quadrangle_vertex[j] - 1).vertex[k];
					tmp++;
				}
			}
		}
		break;
	case 'n':
		target = new GLdouble[quadNum * 12];
		for(int i=0; i<quadNum; i++){
			for(int j=0; j<4; j++){
				for(int k=0; k<3; k++){
					target[tmp] = source->normalList.at(source->quadrangleList.at(i).quadrangle_normal[j] - 1).normal[k];
					tmp++;
				}
			}
		}
		break;
	case 't':
		target = new GLdouble[quadNum * 8];
		for(int i=0; i<quadNum; i++){
			for(int j=0; j<4; j++){
				for(int k=0; k<2; k++){
					target[tmp] = source->textureList.at(source->quadrangleList.at(i).quadrangle_texture[j] - 1).texture[k];
					tmp++;
				}
			}
		}
		break;
	default:
		break;
	}

}
void ObjectDisplay::setVertexArray(GLdouble *& source)
{
	//load outside array to vertex, display the result
	memmove(vertex, source, sizeof(GLdouble)*quadNum*12);
}
void ObjectDisplay::decideAngle()
{
	//rotation[]: 0:up and down, 1:left and right
	__int8 actionOrNot;

	srand( clock() );
	actionOrNot = ( rand() % 100 ) + 1;
	if( (actionOrNot%87) == 0 )
	{
		if( rotation[1] == 5.0 ){			rotation[1] += 5.0;}
		else if( rotation[1] == 10.0 ){		rotation[1] -= 5.0;}
		else if( rotation[1] == -5.0){		rotation[1] -= 5.0;}
		else if( rotation[1] == -10.0 ){	rotation[1] += 5.0;}
		else{								rotation[1] += 5.0;}
	}

	if( (actionOrNot%99) == 0 )
	{
		if( rotation[0] == 0.0 ){			rotation[0] += 3.0;}
		else if( rotation[0] == 3.0 ){		rotation[0] -= 3.0;}
		else if( rotation[0] == -6.0){		rotation[0] -= 3.0;}
		else if( rotation[0] == -9.0 ){		rotation[0] += 3.0;}
		else{								rotation[0] -= 3.0;}
	}
}
void ObjectDisplay::setRotate(GLfloat xAxis, GLfloat yAxis)
{
	rotation[0] = xAxis;
	rotation[1] = yAxis;
}
void ObjectDisplay::readVertexArray(const string objfile, GLdouble *& target)
{
	ObjRead *inModel = new ObjRead[1];
	inModel[0].loadFromFile( objfile.c_str() );//read model obj file
	quadNum = (int) inModel[0].quadrangleList.size();
	setModelArray(target, &inModel[0], 'v');
	delete [] inModel;
}
void ObjectDisplay::loadCurrentVertex(GLdouble *& target)
{
	//load vertex array to target, usually target is some array outside this class
	memmove(target, vertex, sizeof(GLdouble)*quadNum*12);
}
ObjectDisplay::ObjectDisplay(){
	//initial value of model data
	quadNum = 0;
	scale = 1.0;
	for(int i=0; i<3; i++)
	{
		center[i] = 0.0;
		translation[i] = 0.0;
		rotation[i] = 0.0;
	}
}
ObjectDisplay::~ObjectDisplay(){
}