#include "../stdafx.h"
#include "../Glm/glm.h"
#include "../Glm/glmVector.h"
#include "../Geodesic/geodesicFun.h"
#include "../Method_3_SRV/Method_3_SRV3D.h"
#include <cv.h>
using namespace std;
#pragma once

class obRCurves{
public:
	string fileName;
	vector<int> gBC_index;//´æ´¢ÐòÁÐ»¯µÄ²âµØ»·µã
	vector<CvPoint3D32f> noseCordinate;//¾Ö²¿×ø±êÏµ
	vector<vector<vector<double>>> noseBridgeCurves;//´æ´¢ÐòÁÐ»¯µÄ±ÇÁºÏß
	vector<vector<int>> noseBridgeCurvesIndex;
	vector<int> noseBridgeIndex;//¼ÇÂ¼±ÇÁºµãµÄIndex
	vector<int> stripesIndex;//¼ÇÂ¼µÈ²âµØ»·µÄIndex
	vector<srv_object3D> NoseShapeSRV3D;//¼ÆËãnoseShapeSRV3D	
	vector<vector<int>> pointIndex;

private:

	int curvesNum;//±ÇÁºÏßµÄÌõÊý
	int noseMiddle;
	int eyesbrow;
	double normalLength;
	double muliti_nL = 1;//¶ÔnormalLengthÊ©¼ÓÓ°ÏìµÄ±¶Êý
	GLMmodel* pModel;
	//´æ´¢Èý½ÇÐÎÉ¨Ãè¹ØÁªµãÊý¾Ý½á¹¹
	vector<vector <int>> neiborPoint;

public:

	obRCurves(string fileNameObj, int cNum, int nMiddle, int eMiddle);
	obRCurves(string filePath_gbc);
	double obNoseSRV3D_CompareResult(obRCurves ob2);
	vector<double> obNoseSRV3D_CompareResult2(obRCurves ob2);//·µ»ØÃ¿¸ö±ÇÁºÏßµÄ±È½Ï½á¹û¡£
	vector<vector<vector<double>>> obNoseSRV3D_pathData(obRCurves ob2, double l);

private:

	void pointIndex_Stripes();//ÇóµÈ²âµØ´øÄÚ²¿µÄµã¼¯ºÏ;

	void obNoseSRV3D_gbc_compute();//¼ÆËãµÈ²âµØ»·
	vector<int> obNoseSRV3D_gbc_border(vector<int> outsideStripe, vector<int> insideStripe);
	vector<int> obNoseSRV3D_gbc_border_order(vector<int> stripeBorder);
	int obNoseSRV3D_gbc_border_order_startPoint(vector<int> stripeBorder);


	//¼ÆËã±ÇÁºÇúÏß
	vector<double> obNoseSRV3D_pointIndex(double length, double noseLength, vector<double> nosePoint);//ÔÚ±ÇÁº²âµØÏßÉÏÕÒµ½ÏàÓ¦µÄµã.
	int obNoseSRV3D_closePoint(double yTheroid, int positive, vector<CvPoint3D32f> border);//¼ÆËã×î½Ó½üyÌõ¼þÏÂµÄµã
	void obNoseSRV3D_geodesicNosePath();

	//¼ÆËã¾Ö²¿×ø±ê
	void obNoseSRV3D_noseCordinate();
	vector<CvPoint3D32f> obNoseSRV3D_stripedRegular();



	//¼ÆËãNoseShapeSRV3D
	void obNoseSRV3D_NoseShapeSRV3D();


	//¹¦ÄÜº¯Êý
	int obNoseSRV3D_pointexist_return(int point, vector<int> data);

};
