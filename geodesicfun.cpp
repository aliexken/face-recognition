#include "stdafx.h"
#include "geodesicFun.h"
#include "math.h"
#include <iostream>
#include "../Glm/glmVector.h"
using namespace std;

double pai_f = 3.1415927;

//³õÊ¼»¯
geodesicCom::geodesicCom(GLMmodel* pModel){	
	fflush(stdout);	
	this->pModel = pModel;
	const char* meshFile = 0;
	int sourceVertexID = -1;
	int targetVertexID = -1;
	int pathToVertexIDs[2] = { -1, -1 };
	bool printDistances = false;
	double tolerance = -1.0;
	if (geodesic_Global.meshNVertices() >= 0){
		geodesic_Global.meshClear();
	}
	this->loadGLMmodel(pModel, geodesic_Global);
}

//»­³ö²âµØÏß£¨Á¬Ðøµã£©
void geodesicCom::geodesicVectorDraw(vector<double> vV)
{

	glPushMatrix();
	glColor3f(0, 1, 0);
	glLineWidth(6);   //ÉèÖÃÏß¿íÎªÏñËØ
	glPointSize(4);
	glBegin(GL_LINES);

	for (int i = 1; i < vV.size()/3; i++){
		glVertex3f(vV.at(3 * (i - 1)), vV.at(3 * (i - 1) + 1), vV.at(3 * (i - 1) + 2));
		glVertex3f(vV.at(3 * i), vV.at(3 * i + 1), vV.at(3 * i + 2));
	}

	glEnd();
	glPopMatrix();
}

//»­³ö²âµØÏß
void geodesicCom::geodesicPointDraw(int s, int v, double r, double g, double b)
{
	
	s = s - 1;
	v = v - 1;
	vector<double> vV = this->geodesicPath(s, v);
	glPushMatrix();
	glColor3f(r, g, b);
	glLineWidth(8);   //ÉèÖÃÏß¿íÎªÏñËØ
	glPointSize(4);
	glBegin(GL_LINES);
	for (int i = 3; i < vV.size(); i = i + 3){
		glVertex3f(vV.at(i - 3), vV.at(i - 2), vV.at(i - 1));
		glVertex3f(vV.at(i), vV.at(i + 1), vV.at(i + 2));
	}
	glEnd();
	glPopMatrix();
}

//ÔØÈëÄ£ÐÍ
void geodesicCom::loadGLMmodel(GLMmodel* pModel, Geodesic& g){
	vX.clear();
	vY.clear();
	vZ.clear();
	for (int i = 1; i <= pModel->numvertices; i++){
		g.meshAddVertex(pModel->vertices[3 * i + 0],
			pModel->vertices[3 * i + 1], pModel->vertices[3 * i + 2]);
		vX.push_back(pModel->vertices[3 * i + 0]);
		vY.push_back(pModel->vertices[3 * i + 1]);
		vZ.push_back(pModel->vertices[3 * i + 2]);
	}
	for (int i = 0; i < pModel->numtriangles; i++){
		g.meshAddFace(pModel->triangles[(i)].vindices[0] - 1,
			pModel->triangles[(i)].vindices[1] - 1,
			pModel->triangles[(i)].vindices[2] - 1);
	}

}

//¼ÆËãÁ½¸ö¶¨µãÖ®¼äµÄ²âµØ¾àÀë2£¨·µ»Ø¾àÀë£©
double geodesicCom::geodesicPathLen(int s, int v){
	//s = s - 1;
	//v = v - 1;
	vector<double> vV;
	
	vV = this->geodesicPath(s, v);		
	if (vV.size() == 1){
		printf("²âµØÏß³¤¶È¼ÆËãerror£¡\n");
		return -1;
	}
	else{
		double length = 0;
		for (int i = 3; i < vV.size(); i = i + 3){
			double x1 = vV.at(i - 3);
			double y1 = vV.at(i - 2);
			double z1 = vV.at(i - 1);
			double x2 = vV.at(i);
			double y2 = vV.at(i + 1);
			double z2 = vV.at(i + 2);
			length = length + sqrt((x1 - x2)*(x1 - x2) +
				(y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
		}
		return length;	
	}
	
	
}

vector<double> geodesicCom::geodesicPath_throw_exception(int s, int t)throw(exception&, bad_alloc&){
	vector<double> gec;
	Geodesic geodesic_Global_now;
	if (geodesic_Global_now.meshNVertices() >= 0){
		geodesic_Global_now.meshClear();
	}
	this->loadGLMmodel(pModel, geodesic_Global_now);
	geodesic_Global_now.findPathBetweenTwoVertices(s, t);
	while (!geodesic_Global_now.pathEmpty()) {
		int vs, vt;
		double b;
		geodesic_Global_now.pathPopNextStep(vs, vt, b);
		//printf("%f,%f,%f", xx[vs - 1] * (1 - b), xx[vt - 1] * b);
		double g1 = vX.at(vs) * (1 - b) + vX.at(vt) * b;
		double g2 = vY.at(vs) * (1 - b) + vY.at(vt) * b;
		double g3 = vZ.at(vs) * (1 - b) + vZ.at(vt) * b;
		gec.push_back(g1);
		gec.push_back(g2);
		gec.push_back(g3);
	}

	return gec;
}

vector<double> geodesicCom::geodesicPath_process_exception(int s, int t){
	vector<double> result;	
	
	try{
		result = geodesicPath_throw_exception(s, t);
		return result;
	}	
	catch (...){
		vector<double> result2;
		result2.push_back(-1);		
		return result2;
	}	
}

//¼ÆËãÁ½¸ö¶¨µãÖ®¼äµÄ²âµØ¾àÀë1£¨·µ»ØÒ»ÏµÁÐµãµÄ×ø±ê£©
vector<double> geodesicCom::geodesicPath(int s, int t){
	s = s - 1;
	t = t - 1;
	vector<double> result;
	result = geodesicPath_process_exception(s, t);	
	if (result.size() != 1){
		return result;
	}
	else{			
		vector<double> result2;
		result2 = geodesicPath_process_exception(t, s);
		if (result2.size() != 1){
			vector<double> result3;
			for (int i = result2.size() / 3 - 1; i >= 0; i--){
				result3.push_back(result2.at(3 * i));
				result3.push_back(result2.at(3 * i + 1));
				result3.push_back(result2.at(3 * i + 2));
			}
			printf("ÐÞÕý³É¹¦£¡\n");
			return result3;
		}
		else{
			vector<double> result3;
			result3.push_back(-1);
			printf("ÐÞÕýÊ§°Ü£¡\n");
			return result3;		
		}		
	}	
}

//¼ÆËãÔ´µãµ½Õû¸ömodel¶¥µãµÄ²âµØ¾àÀë(È«¾Ö)
vector<double> geodesicCom::geodesicDistances(int s, double tolerance){	
	vector<double> gecDis;

	Geodesic geodesic_Global_now;
	if (geodesic_Global_now.meshNVertices() >= 0){
		geodesic_Global_now.meshClear();
	}
	this->loadGLMmodel(pModel, geodesic_Global_now);

	if (tolerance == -1.0)
		geodesic_Global_now.computeExactGeodesics(s);
	else
		geodesic_Global_now.computeApproxGeodesics(s, tolerance);

	const int n = geodesic_Global_now.meshNVertices();
	for (int i = 0; i < n; ++i){
		gecDis.push_back(geodesic_Global_now.distanceAtVertex(i));
	}	

	return gecDis;

}

//¼ÆËãµÈ²âµØ´ø
vector<int> geodesicCom::geodesicISOstripe(double radius_low, double radius, vector<double> dis){
	vector<int> ISOstripe;
	for (int i = 0; i < dis.size(); i++){
		if (dis.at(i) < radius&&dis.at(i) >= radius_low){
			ISOstripe.push_back(i + 1);
		}
	}
	return ISOstripe;
}
vector<vector<int>> geodesicCom::geodesicISOstripes(int noseMiddle, int eyebrowMiddle){
	//noseMiddle = noseMiddle - 1;
	//eyebrowMiddle = eyebrowMiddle - 1;
	vector<vector<int>> result;

	double normalLength = this->geodesicPathLen(noseMiddle, eyebrowMiddle);
	normalLength = normalLength / 3;
	vector<double> dis = this->geodesicDistances(noseMiddle - 1, -1);
	//Éè¶¨4¸öãÐÖµ
	vector<double> Threshold;
	Threshold.push_back(normalLength);
	Threshold.push_back(normalLength * 2);
	Threshold.push_back(normalLength * 3);
	Threshold.push_back(normalLength * 4);
	Threshold.push_back(normalLength * 4.5);//Ã»ÓÐÒâÒå£¬ÓÃÓÚ×îÍâ±ß½ç¶¨Î»

	vector<int> ISOstripe_start = geodesicISOstripe(0, Threshold.at(0), dis);
	result.push_back(ISOstripe_start);

	for (int i = 1; i < Threshold.size(); i++){
		vector<int> ISOstripe = geodesicISOstripe(Threshold.at(i-1),Threshold.at(i), dis);
		result.push_back(ISOstripe);
	}
	return result;
}

//¼ÆËã²âµØ´øµÄ±ß½ç
vector<vector<int>> geodesicCom::geodesicBorderStripe(vector<vector<int>> Stripes, vector<vector<int>> neiborPoint){
	
	vector<vector<int>> geodesicBorder;
	for (int i = 0; i < Stripes.size(); i++){
		//cout << "´¦ÀíµÚ" << i << "¸ö±ß½ç¡£" << endl;
		vector<int> border_i;
		if (i + 1 < Stripes.size()){
			vector<int> inside = Stripes.at(i);
			vector<int> outside = Stripes.at(i+1);
			for (int j = 0; j < inside.size(); j++){
				int index = inside.at(j);
				vector<int> indexNeobor = neiborPoint.at(index);
				if (pointexist(indexNeobor, outside)){
					border_i.push_back(index);				
				}
			}
			geodesicBorder.push_back(border_i);
		}
		//geodesicBorder.push_back(border_i);
		
	}
	return geodesicBorder;
}
vector<vector<vector<int>>> geodesicCom::geodesicBorderStripe_neibor(vector<vector<int>> borders, vector<vector<int>> neiborPoint){
	vector<vector<vector<int>>> border_neibor;
	for (int i = 0; i < borders.size(); i++){
		vector<int> borders_i = borders.at(i);
		vector<vector<int>> borders_i_neibor;
		for (int j = 0; j < borders_i.size(); j++){
			int index_j = borders_i.at(j);
			vector<int> neibor_j = neiborPoint.at(index_j);
			vector<int> neibor_j_new;
			for (int k = 0; k < neibor_j.size(); k++){
				if (pointexist(neibor_j.at(k), borders_i)){
					neibor_j_new.push_back(neibor_j.at(k));
				}
			}
			borders_i_neibor.push_back(neibor_j_new);
		}
		border_neibor.push_back(borders_i_neibor);
	}
	return border_neibor;
}

//¼ÆËã±ÇÁºµÄÂ·¾¶
vector<vector<double>> geodesicCom::geodesicNosePath(vector<int> points,string fileName){
	
	vector<double> nosePath;
	vector<double> nosePathY0;//ÓënosePath´¹Ö±µÄÈýÌõ²âµØÏßµÄ×ø±ê
	vector<double> nosePathY2;
	vector<double> nosePathY4;

	nosePath = geodesicPath(points.at(0), points.at(1));
	double nosePathLength = 0;
	int pointNum = nosePath.size() / 3;
	for (int i = 1; i < pointNum; i++){
		double x1 = nosePath.at(3 * (i - 1));
		double y1 = nosePath.at(3 * (i - 1) + 1);
		double z1 = nosePath.at(3 * (i - 1) + 2);
		double x2 = nosePath.at(3 * i);
		double y2 = nosePath.at(3 * i + 1);
		double z2 = nosePath.at(3 * i + 2);
		double length = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
		nosePathLength = nosePathLength + length;
	}
	
	double length4 = nosePathLength* 2.0 / 3.0;
	double length2 = nosePathLength / 3.0;	
	double minlength4diff = 1000;
	double minlength2diff = 1000;
	double x_2, y_2, z_2;
	double x_4, y_4, z_4;

	nosePathLength = 0;

	for (int i = 1; i < pointNum; i++){
		double x1 = nosePath.at(3 * (i - 1));
		double y1 = nosePath.at(3 * (i - 1) + 1);
		double z1 = nosePath.at(3 * (i - 1) + 2);
		double x2 = nosePath.at(3 * i);
		double y2 = nosePath.at(3 * i + 1);
		double z2 = nosePath.at(3 * i + 2);
		double length = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
		nosePathLength = nosePathLength + length;
		if (abs(nosePathLength - length4) < minlength4diff){
			minlength4diff = abs(nosePathLength - length4);
			x_4 = x2;
			y_4 = y2;
			z_4 = z2;
		}
		if (abs(nosePathLength - length2) < minlength2diff){
			minlength2diff = abs(nosePathLength - length2);
			x_2 = x2;
			y_2 = y2;
			z_2 = z2;
		}

	}
	int point2 = glmIndexPoint(this->pModel, x_2, y_2, z_2);
	int point4 = glmIndexPoint(this->pModel, x_4, y_4, z_4);
	nosePathY0 = geodesicPath(points.at(2), points.at(0));
	nosePathY2 = geodesicPath(points.at(3), point2);
	nosePathY4 = geodesicPath(points.at(4), point4);
	
	vector<vector<double>> nosePathThree;
	nosePathThree.push_back(nosePathY0);
	nosePathThree.push_back(nosePathY2);
	nosePathThree.push_back(nosePathY4);

	ofstream f1(fileName, ios::app);//´ò¿ªÎÄ¼þ

	f1 << nosePathThree.size() << endl;
	for (int i = 0; i < nosePathThree.size(); i++){
		vector<double> nosePathThree_i = nosePathThree.at(i);
		f1 << nosePathThree_i.size() / 3 << endl;
		for (int j = 0; j < nosePathThree_i.size(); j++){
			f1 << nosePathThree_i.at(j) << " ";
		}
		f1 << endl;
	}
	f1.close();
	return nosePathThree;
}


//»­³öµÈ²âµØ´ø
void geodesicCom::geodesicStripsDraw(vector<vector<int>> pointDraw){

	geodesicPointDraw(pointDraw.at(0), 1, 0, 0);
	geodesicPointDraw(pointDraw.at(1), 0, 1, 0);
	geodesicPointDraw(pointDraw.at(2), 0, 0, 1);
	geodesicPointDraw(pointDraw.at(3), 1, 0, 1);

}
void geodesicCom::geodesicPointDraw(vector<int> pointDraw,double r, double g, double b){
	glPushMatrix();
	glColor3f(r, g, b);

	glPointSize(4);
	glBegin(GL_POINTS);
	for (int i = 0; i < pointDraw.size(); i++){
		glVertex3f(pModel->vertices[3 * pointDraw.at(i)],
			pModel->vertices[3 * pointDraw.at(i) + 1],
			pModel->vertices[3 * pointDraw.at(i) + 2]);
	}
	glEnd();
	glPopMatrix();
}

//¶ÔÍâµÄÕûºÏ½Ó¿Ú
vector<vector<int>> geodesicCom::geodesicBorderCompute(int noseMiddle, int eyeBrow, vector<vector<int>> neiborPoint, string filenameStore){
	vector<vector<int>> geodesicPoint = geodesicISOstripes(noseMiddle, eyeBrow);//Ê×ÏÈÉú³ÉµÈ²âµØ´øÊý¾Ý
	vector<vector<int>> geodesicBorder = geodesicBorderStripe(geodesicPoint, neiborPoint);//Ñ°ÕÒ±ß½ç
	//»ñµÃÁÙ½Ó¹ØÏµ
	vector<vector<vector<int>>> geodesicBorder_neibor = geodesicBorderStripe_neibor(geodesicBorder, neiborPoint);
	geodesicBorderStripeWrite(geodesicBorder, filenameStore, noseMiddle, eyeBrow, geodesicBorder_neibor);
	return geodesicBorder;
}

//Ò»Ð©»ù±¾µÄ¹¦ÄÜº¯Êý£ºÅÐ¶¨½ÚµãÊÇ·ñ´æÔÚÓÚ½á¹¹ÖÐ
bool geodesicCom::pointexist(int point, vector<int> data){

	for (int i = 0; i < data.size(); i++){
		if (point == data.at(i)){
			return true;
		}
	}
	return false;
}
int geodesicCom::pointexist_return(int point, vector<int> data){
	for (int i = 0; i < data.size(); i++){
		if (point == data.at(i)){
			return i;
		}
	}
	return -1;
}
bool geodesicCom::pointexist(vector<int> point, vector<int> data){
	for (int i = 0; i < point.size(); i++){
		int index = point.at(i);
		for (int j = 0; j < data.size(); j++){
			if (index == data.at(j)){
				return true;
			}
		}	    
	}
	return false;
}


//´æ´¢
void geodesicCom::geodesicBorderStripeWrite(vector<vector<int>> circlePoint, string fileName, int noseMiddle, int eyeBrow, vector<vector<vector<int>>> neibor){
	
	//Ê×ÏÈ»ñµÃ²¢Ð´ÈëÁ½¸ö¶¨Î»µãµÄÐÅÏ¢
	double x1 = pModel->vertices[3 * noseMiddle];
	double y1 = pModel->vertices[3 * noseMiddle + 1];
	double z1 = pModel->vertices[3 * noseMiddle + 2];
	double x2 = pModel->vertices[3 * eyeBrow];
	double y2 = pModel->vertices[3 * eyeBrow + 1];
	double z2 = pModel->vertices[3 * eyeBrow + 2];

	ofstream f1(fileName, ios::app);//´ò¿ªÎÄ¼þ

	f1 << circlePoint.size() << endl;
	f1 << x1 << " " << y1 << " " << z1 << endl;
	f1 << x2 << " " << y2 << " " << z2 << endl;

	for (int i = 0; i < circlePoint.size(); i++){
		vector<int> circlePoint_i = circlePoint.at(i);
		f1 << circlePoint_i.size() << endl;
		for (int j = 0; j < circlePoint_i.size(); j++){
			int index = circlePoint_i.at(j);
			double xi = pModel->vertices[3 * index];
			double yi = pModel->vertices[3 * index + 1];
			double zi = pModel->vertices[3 * index + 2];
			f1 << xi << " " << yi << " " << zi << endl;
		}
	}

	//¿ªÊ¼Ð´ÁÙ½Ó¹ØÏµÊý¾Ý¿â
	f1 << neibor.size() << endl;
	for (int i = 0; i < neibor.size(); i++){
		vector<vector<int>> neibor_i = neibor.at(i);//µÚiÌõµÈ²âµØ»·µÄÁÙ½ÓÊý¾Ý¿â
		f1 << neibor_i.size()<<endl;
		for (int j = 0; j < neibor_i.size(); j++){
			vector<int> neibor_i_j = neibor_i.at(j);
			f1 << neibor_i_j.size()<<" ";
			for (int k = 0; k < neibor_i_j.size(); k++){
				int neibor_point = pointexist_return(neibor_i_j.at(k), circlePoint.at(i));
				f1 << neibor_point <<" ";
			}
			f1 << endl;
		}
	}


	f1.close();

}
