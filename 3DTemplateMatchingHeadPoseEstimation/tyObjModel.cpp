#include "tyObjModel.h"


bool TYObjModel::loadFromFile(const char* fileName)
{
	//--------------------------------------------------
	// local(tmp) variables
	//--------------------------------------------------
	char tmp[100]	= {0};
	char buf[5][50] = {0};
	char trash[10]	= {0};

	float max[3]  = {-100000.0, -100000.0, -100000.0};
	float min[3]  = { 100000.0,  100000.0,  100000.0};
	float tmpf[3] = {      0.0,       0.0,       0.0};

	int nIdx[4];
	int vIdx[4];
	int tIdx[4];

	vector<Vertex> vList;		Vertex	v;
	vector<Normal> nList;		Normal	n;
	vector<Texture> texList;	Texture	tex;
	
	//--------------------------------------------------
	// open the obj file
	//--------------------------------------------------
	FILE *fp;
	errno_t err = fopen_s( &fp, fileName, "r" );
	if(err)
	{
		cout << "Can not open object File \"" << fileName << "\" !" << endl;
		return false;
	}
	cout <<"Loading \"" << fileName << "\" !" << endl;

	
	//--------------------------------------------------
	// parse the obj file
	// "%[^\n]\n" is the regular expression for read 1 row
	//--------------------------------------------------
	/*float tx;
	float ty;
	float tz= -1000000;*/

	while(fscanf(fp,"%[^\n]\n",tmp) != EOF){
		// if the current reading row is "v x y z"
		if (strncmp(tmp,"v ",2) == 0)
		{
			sscanf(tmp, "%s %f %f %f", trash, &tmpf[0], &tmpf[1], &tmpf[2]);
			v.load(tmpf);
			vList.push_back(v);


			/*if(tmpf[2] > tz){
				tz = tmpf[2];
				tx = tmpf[0];
				ty = tmpf[1];
			}*/

			// keep min and max, for center caculating
			for(int i = 0; i < 3; i++){
				if(tmpf[i] < min[i])
					min[i] = tmpf[i];
				if(tmpf[i] > max[i])
					max[i] = tmpf[i];
			}
		}
		// if the current reading row is "vt u v"
		else if(strncmp(tmp,"vt ",3) == 0)
		{
			sscanf(tmp, "%s %f %f", trash, &tmpf[0], &tmpf[1]);
			tex.load(tmpf);
			texList.push_back(tex);
		}
		// if the current reading row is "vn x y z"
		else if(strncmp(tmp,"vn ",3) == 0)
		{
			sscanf(tmp, "%s %f %f %f", trash, &tmpf[0], &tmpf[1], &tmpf[2]);
			n.load(tmpf);
			nList.push_back(n);
		}
		// if the current reading row is "f v1[/vt1][/vn1] v2[/vt2][/vn2] v3[/vt3][/vn3]"
		// (grouping triangle's 3 vertice,normals,textures together )
		else if(strncmp(tmp,"f ",2) == 0)	
		{
			int vNum;			//vertex Number: 3 if isTriangle; 4 if isQuadrangle
			int nInfo;			//vertex info Number: 1 if only vertex; 3 if got vt; 5 if got vn
			char slash;			//just a buffer

			ObjTriangle t;			//current triangle: for push_back to vecture
			//Quadrangle q;		//current quadrangle: for push_back to vecture
			
			vNum = sscanf(tmp,"%s %s %s %s %s", buf[0], buf[1], buf[2], buf[3], buf[4]) - 1;
			//--------------------------------------------------------------
			// sscanf returns the number of successfully scanned variables
			//--------------------------------------------------------------

			if(!(vNum==3 || vNum==4)){
				printf("obj file format not supported!\n");
				return false;
			}

			for(int i = 0 ; i < vNum ; i++){
				nInfo = sscanf(buf[i+1],"%d %c %d %c %d", &vIdx[i], &slash, &tIdx[i], &slash, &nIdx[i]);
				
				if(!(nInfo==1 || nInfo==3 || nInfo==5)){
					printf("obj file format not supported!\n");
					return false;
				}

				if(vNum == 3){
					if(nInfo >= 1)	t.loadVertex (i, vList[ vIdx[i] -1].vertex );
					if(nInfo >= 3)	t.loadTexture(i, texList[tIdx[i]-1].texture);
					if(nInfo >= 5)	t.loadNormal (i, nList[ nIdx[i] -1].normal );
				}else{
					//if(nInfo >= 1)	q.loadVertex (i, vList[ vIdx[i] -1].vertex );
					//if(nInfo >= 3)	q.loadTexture(i, texList[tIdx[i]-1].texture);
					//if(nInfo >= 5)	q.loadNormal (i, nList[ nIdx[i] -1].normal );
					printf("Quadrangle has been deprecated!\n");
				}
			}

			if(vNum == 3){
				tList.push_back(t);
			}else{
				printf("Quadrangle has been deprecated!\n");//qList.push_back(q);
			}

		}
		else if(strncmp(tmp,"g ",2) == 0)
		{
			//Do Nothing
		}
	}

	//printf("nose!!!!!!!!!! ( %f, %f, %f)", tx, ty, tz);

	//set model center
	for(int i = 0; i < 3; i++){
		center[i] = (min[i] + max[i]) / 2;
		//printf("min[%d], max[%d] = %f, %f\n", i, i, min[i], max[i]);
	}

	/*-------------------------------------*/
	/* only for head.obj , (it's the nose) */
	/*						!!!!!!!!!!!!!! */
	/*-------------------------------------*/
	//center[0] = 0;		
	//center[1] = 1.728f;
	//center[2] = 0.1256f;
	/*-------------------------------------*/
	/* only for head.obj , (it's the nose) */
	/*						!!!!!!!!!!!!!! */
	/*-------------------------------------*/


	printf("Load complete:	%d triangles, %d quadrangles.\n", tList.size(), qList.size());
	
	return true;
}
void TYObjModel::toString(){
	printf("triangles:\n");
	for(unsigned int i = 0 ; i < tList.size(); i++){
		printf("(%f, %f, %f) (%f, %f, %f) (%f, %f, %f)\n",tList[i].vertex[0][0]
														 ,tList[i].vertex[0][1]
														 ,tList[i].vertex[0][2]
														 ,tList[i].vertex[1][0]
														 ,tList[i].vertex[1][1]
														 ,tList[i].vertex[1][2]
														 ,tList[i].vertex[2][0]
														 ,tList[i].vertex[2][1]
														 ,tList[i].vertex[2][2]);
	}
	printf("quadrangles:\n");
	/*for(unsigned int i = 0 ; i < qList.size(); i++){
		printf("(%f, %f, %f) (%f, %f, %f) (%f, %f, %f) (%f, %f, %f)\n",qList[i].vertex[0][0]
																	  ,qList[i].vertex[0][1]
																	  ,qList[i].vertex[0][2]
																	  ,qList[i].vertex[1][0]
																	  ,qList[i].vertex[1][1]
																	  ,qList[i].vertex[1][2]
																	  ,qList[i].vertex[2][0]
																	  ,qList[i].vertex[2][1]
																	  ,qList[i].vertex[2][2]
																	  ,qList[i].vertex[3][0]
																	  ,qList[i].vertex[3][1]
																	  ,qList[i].vertex[3][2]);
	}*/
}
void TYObjModel::copy(TYObjModel *obj)
{
	for(int i = 0; i < 3; i++)
	{
		center[i] = obj->center[i];
	}
	tList = obj->tList;
	qList = obj->qList;
}
TYObjModel::TYObjModel(){
}
TYObjModel::~TYObjModel(){
}