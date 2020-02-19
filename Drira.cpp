#include "stdafx.h"
#include "Method_4_RadialCurves.h"


obRCurves::obRCurves(string fileNameObj, int cNum, int nMiddle, int eMiddle){
	fileName = fileNameObj;
	curvesNum = cNum;
	noseMiddle = nMiddle;
	eyesbrow = eMiddle;

	if (pModel)
	{
		glmDelete(pModel);
		pModel = NULL;
	}
	char fileNameChar[128];
	strcpy(fileNameChar, fileNameObj.c_str());
	// Load the new obj model
	pModel = glmReadOBJ(fileNameChar);
	// Generate normal for the model
	glmFacetNormals(pModel);
	geodesicCom geodesicTool(pModel);
	normalLength = geodesicTool.geodesicPathLen(noseMiddle, eyesbrow);
	//normalLength = normalLength * 4 / 3;
	normalLength = normalLength * muliti_nL;
	cout << "ÁÙ±ß½á¹¹¼ÆËã" << endl;
	neiborPoint = glmPointNeibor(pModel);
	cout << "¼ÆËã²âµØ±ß½ç" << endl;
	obNoseSRV3D_gbc_compute();
	cout << "¼ÆËã±ÇÁºÏß" << endl;
	obNoseSRV3D_geodesicNosePath();
	cout << "¼ÆËã¾Ö²¿×ø±ê" << endl;
	obNoseSRV3D_noseCordinate();
	cout << "SRV3D¼ÆËã" << endl;
	obNoseSRV3D_NoseShapeSRV3D();
}

obRCurves::obRCurves(string filePath_gbc){

}

double obRCurves::obNoseSRV3D_CompareResult(obRCurves ob2){

	double sum = 0;
	for (int i = 0; i < NoseShapeSRV3D.size(); i++){
		srv_object3D obs1 = NoseShapeSRV3D.at(i);
		srv_object3D obs2 = ob2.NoseShapeSRV3D.at(i);
		double result = obs1.srv_object3D_Compare(obs2);
		sum = sum + result;
	}
	return sum;
}

vector<double> obRCurves::obNoseSRV3D_CompareResult2(obRCurves ob2){//·µ»ØÃ¿¸ö±ÇÁºÏßµÄ±È½Ï½á¹û¡£

	vector<double> resultV;

	for (int i = 0; i < NoseShapeSRV3D.size(); i++){
		srv_object3D obs1 = NoseShapeSRV3D.at(i);
		srv_object3D obs2 = ob2.NoseShapeSRV3D.at(i);
		double result = obs1.srv_object3D_Compare(obs2);
		resultV.push_back(result);
	}

	return resultV;

}

vector<vector<vector<double>>> obRCurves::obNoseSRV3D_pathData(obRCurves ob2, double l){

	vector<vector<vector<double>>> result;

	int n1 = noseBridgeIndex.at(0);
	int n2 = noseBridgeIndex.at(1);
	double x_n1 = pModel->vertices[3 * n1];
	double y_n1 = pModel->vertices[3 * n1 + 1];
	double z_n1 = pModel->vertices[3 * n1 + 2];
	double x_n2 = pModel->vertices[3 * n2];
	double y_n2 = pModel->vertices[3 * n2 + 1];
	double z_n2 = pModel->vertices[3 * n2 + 2];

	double mlutiple = sqrt((x_n1 - x_n2)*(x_n1 - x_n2) +
		(y_n1 - y_n2)*(y_n1 - y_n2) + (z_n1 - x_n2)*(z_n1 - x_n2));

	mlutiple = (mlutiple * (curvesNum + 1)) * 4 / 3;

	for (int i = 0; i < NoseShapeSRV3D.size(); i++){
		int nosepoint_i = i / 2;
		int point_index = noseBridgeIndex.at(nosepoint_i);
		double x_i = pModel->vertices[3 * point_index];
		double y_i = pModel->vertices[3 * point_index + 1];
		double z_i = pModel->vertices[3 * point_index + 2];
		vector<double> point_v;
		point_v.push_back(x_i);
		point_v.push_back(y_i);
		point_v.push_back(z_i);
		srv_object3D obs1 = NoseShapeSRV3D.at(i);
		srv_object3D obs2 = ob2.NoseShapeSRV3D.at(i);
		vector<vector<double>> result_i = obs1.srv_object3D_pathCompute(obs2, l, point_v, mlutiple);
		result.push_back(result_i);
	}
	return result;
}

#pragma region ÄÚ²¿º¯Êý

void obRCurves::obNoseSRV3D_gbc_compute(){

	geodesicCom geodesicTool(pModel);
	vector<double> disNoseToAll = geodesicTool.geodesicDistances(noseMiddle - 1, -1);
	double tnormal = normalLength * 5 / 4;
	double t2normal = normalLength * 3 / 4;
	vector<int> outStripes;
	vector<int> inStripes;
	vector<int> pointIndex_i;
	for (int i = 0; i < disNoseToAll.size(); i++){
		double disNoseToAll_i = disNoseToAll.at(i);

		if (disNoseToAll_i <= normalLength){
			pointIndex_i.push_back(i + 1);
		}

		if (disNoseToAll_i>normalLength&&disNoseToAll_i <= tnormal){
			outStripes.push_back(i + 1);
		}
		else if (disNoseToAll_i <= normalLength&&disNoseToAll_i > t2normal){
			inStripes.push_back(i + 1);
		}
		else{
			continue;
		}


	}
	pointIndex.push_back(pointIndex_i);

	gBC_index = obNoseSRV3D_gbc_border(outStripes, inStripes);
}


vector<int> obRCurves::obNoseSRV3D_gbc_border(vector<int> outsideStripe, vector<int> insideStripe){
	vector<int> stripesBorderResult;

	for (int i = 0; i < insideStripe.size(); i++){
		int index_i = insideStripe.at(i);
		vector<int> inside_i = neiborPoint.at(index_i);
		for (int j = 0; j < inside_i.size(); j++){
			int index_j = inside_i.at(j);
			if (obNoseSRV3D_pointexist_return(index_j, stripesBorderResult) != -1){
				continue;
			}
			else{
				if (obNoseSRV3D_pointexist_return(index_j, outsideStripe) != -1){
					stripesBorderResult.push_back(index_j);
				}
				else{
					continue;
				}
			}
		}
	}
	vector<int> order_result = obNoseSRV3D_gbc_border_order(stripesBorderResult);
	return order_result;
}

vector<int> obRCurves::obNoseSRV3D_gbc_border_order(vector<int> stripeBorder){
	vector<int> CircleRegular;
	int startPoint = obNoseSRV3D_gbc_border_order_startPoint(stripeBorder);
	//int startP2 = geodesicCircleRegular_startPoint_0729(nosePoint, noseMiddle, circlePoint);
	//printf("startPoint = %d\n", startPoint);
	//printf("startP2 = %d\n", startP2);
	double sx = pModel->vertices[3 * startPoint];
	double sy = pModel->vertices[3 * startPoint + 1];
	double sz = pModel->vertices[3 * startPoint + 2];
	//ÕÒµ½³õÊ¼µã£¬Í¨¹ýÏàÁÚµãÊý¾Ý½á¹¹¿ªÊ¼ÐòÁÐ»¯CircleRegular
	//1.Ê×ÏÈÈ·¶¨·½Ïò
	vector<int> spv = neiborPoint.at(startPoint);
	int secondPoint;
	double tx_min = 10000;
	for (int i = 0; i < spv.size(); i++){
		if (obNoseSRV3D_pointexist_return(spv.at(i), stripeBorder) != -1){
			double tx = pModel->vertices[3 * spv.at(i)];
			if (tx < tx_min){
				secondPoint = spv.at(i);
				tx_min = tx;
			}
		}
	}

	//printf("´ú±í·½ÏòµÄµÚ¶þ¸öµãÎª%d\n", secondPoint);
	CircleRegular.push_back(startPoint);
	CircleRegular.push_back(secondPoint);
	//2.¿ªÊ¼ÐòÁÐ»¯
	int nowActivePoint = secondPoint;
	bool loopjud = true;


	while (loopjud){
		//cout << "nowActivePoint=" << nowActivePoint << endl;
		//if (nowActivePoint == 525){
		//cout << "breakpoint!" << endl;

		//}
		//printf("»î¶¯µãµÄ±êºÅÎª£º%d\n", nowActivePoint);
		bool continuejud = true;//ÅÐ¶ÏÊÇ·ñ
		vector<int> nAP = neiborPoint.at(nowActivePoint);
		for (int i = 0; i < nAP.size(); i++){
			int tNow = nAP.at(i);
			//cout <<"tNow:"<< nAP.at(i) << endl;
			if (tNow == startPoint&&nowActivePoint != secondPoint){
				loopjud = false;
				continuejud = false;
				break;
			}
			if (obNoseSRV3D_pointexist_return(tNow, stripeBorder) != -1 &&
				obNoseSRV3D_pointexist_return(tNow, CircleRegular) == -1){
				nowActivePoint = tNow;
				CircleRegular.push_back(tNow);
				//printf("µ±Ç°ÕÒµ½µÄµãÎª%d\n", tNow);
				continuejud = false;
				break;
			}
		}
		if (continuejud){

			bool conjud = false;
			vector<int> CircleRegular_RA;
			CircleRegular_RA.assign(CircleRegular.begin(), CircleRegular.end() - 1);
			for (int i = 0; i < nAP.size(); i++){
				if (obNoseSRV3D_pointexist_return(nAP.at(i), stripeBorder) != -1){
					continue;
				}
				else{
					vector<int> nAPt = neiborPoint.at(nAP.at(i));
					for (int j = 0; j < nAPt.size(); j++){
						int nAPt_now = nAPt.at(j);
						if (obNoseSRV3D_pointexist_return(nAPt_now, CircleRegular_RA) == -1
							&& obNoseSRV3D_pointexist_return(nAPt_now, stripeBorder) != -1 && nAPt_now != nowActivePoint){//&& nAPt_now != nowActivePoint
							conjud = true;
						}
					}
					if (conjud){
						nowActivePoint = nAP.at(i);
						CircleRegular.push_back(nAP.at(i));
						break;
					}
				}

			}
		}
	}

	return CircleRegular;
}

int obRCurves::obNoseSRV3D_gbc_border_order_startPoint(vector<int> stripeBorder){

	int nosePoint = noseMiddle;

	//Ê×ÏÈ»ñÈ¡¶ÔÓÚµãµÄÏà¹Ø×ø±ê
	double npx = pModel->vertices[3 * nosePoint];
	double npy = pModel->vertices[3 * nosePoint + 1];
	//double npz = pModel->vertices[3 * nosePoint + 2];
	double nmx = pModel->vertices[3 * eyesbrow];
	double nmy = pModel->vertices[3 * eyesbrow + 1];

	//printf("±Ç¼âµã×ø±ê£ºx=%f,y=%f\n", npx, npy);
	//printf("±ÇÁºµã×ø±ê£ºx=%f,y=%f\n", nmx, nmy);

	double ax = nmx - npx;
	double ay = nmy - npy;
	double cosMax = -1;
	int startPoint = stripeBorder.at(0);

	for (int i = 0; i < stripeBorder.size(); i++){
		double nix = pModel->vertices[3 * stripeBorder.at(i)];
		double niy = pModel->vertices[3 * stripeBorder.at(i) + 1];

		//if (circlePoint.at(i) == 7303 || circlePoint.at(i) == 3128){
		//printf("%dµÄ×ø±êÎªx = %f,y = %f\n", circlePoint.at(i), nix, niy);
		//}

		if (niy < npy){
			continue;
		}
		else{
			double bx = nix - npx;
			double by = niy - npy;
			double cos = (ax*bx + ay*by) / (sqrt(ax*ax + ay*ay)*sqrt(bx*bx + by*by));

			if (cos > cosMax){
				cosMax = cos;
				startPoint = stripeBorder.at(i);
			}
		}
	}
	return startPoint;

}

//¼ÆËã±ÇÁº¶ÔÓ¦µã

vector<double> obRCurves::obNoseSRV3D_pointIndex(double length, double noseLength, vector<double> nosePath){
	vector<double> t;
	if (length > noseLength){
		cout << "error!±ÇÁº²âµØÏß¼ÆËã¶ÔÓ¦µãÊ§°Ü" << endl;

		return t;
	}
	double nosePathLength = 0;
	int pointNum = nosePath.size() / 3;
	for (int i = 1; i < pointNum; i++){
		double x1 = nosePath.at(3 * (i - 1));
		double y1 = nosePath.at(3 * (i - 1) + 1);
		double z1 = nosePath.at(3 * (i - 1) + 2);
		double x2 = nosePath.at(3 * i);
		double y2 = nosePath.at(3 * i + 1);
		double z2 = nosePath.at(3 * i + 2);
		double length_i = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
		nosePathLength = nosePathLength + length_i;
		if (nosePathLength >= length){
			double weightLength_1 = (nosePathLength - length) / length_i;
			double weightLength_2 = 1 - weightLength_1;
			if (weightLength_1 > weightLength_2){
				vector<double> point;
				point.push_back(x1);
				point.push_back(y1);
				point.push_back(z1);
				return point;
			}
			else{
				vector<double> point;
				point.push_back(x2);
				point.push_back(y2);
				point.push_back(z2);
				return point;
			}
		}
	}

	cout << "error!Ã»ÓÐ»ñµÃÏàÓ¦µÄµãÐÅÏ¢£¡" << endl;
	return t;


}

int obRCurves::obNoseSRV3D_closePoint(double yTheroid, int positive, vector<CvPoint3D32f> border){//¼ÆËã×î½Ó½üyÌõ¼þÏÂµÄµã

	double minValue = 10000;
	int pointIndex;

	for (int i = 0; i < border.size(); i++){

		double x_i = border.at(i).x;
		double y_i = border.at(i).y;

		if (y_i < 0){
			continue;
		}
		else if (positive>0 && x_i < 0){
			continue;
		}
		else if (positive < 0 && x_i > 0){
			continue;
		}
		else{
			if (abs(y_i - yTheroid) < minValue){
				minValue = abs(y_i - yTheroid);
				pointIndex = i;
			}
		}
	}

	return gBC_index.at(pointIndex);

}

void obRCurves::obNoseSRV3D_geodesicNosePath(){

	vector<double> nosePath;
	geodesicCom geodesicTool(pModel);
	nosePath = geodesicTool.geodesicPath(noseMiddle, gBC_index.at(0));
	double nosePathLength = geodesicTool.geodesicPathLen(noseMiddle, gBC_index.at(0));

	//¼ÆËã±ÇÁº¶ÔÓ¦µãµÄ×ø±ê
	vector<int> length_i;
	double length_unit = nosePathLength / (curvesNum + 1);
	double length_i_active = 0;
	for (int i = 0; i < curvesNum; i++){
		length_i_active = length_i_active + length_unit;
		length_i.push_back(length_i_active);
	}

	for (int i = 0; i < length_i.size(); i++){
		vector<double> noseBridgePoint_i = obNoseSRV3D_pointIndex(length_i.at(i), nosePathLength, nosePath);
		int pointIndex = glmIndexPoint(pModel, noseBridgePoint_i.at(0), noseBridgePoint_i.at(1), noseBridgePoint_i.at(2));
		noseBridgeIndex.push_back(pointIndex);
	}


	//¼ÆËãµÈ²âµØ´ø¶ÔÓ¦µãµÄ×ø±ê


	vector<CvPoint3D32f> border_regular = obNoseSRV3D_stripedRegular();
	double y_end = border_regular.at(0).y;
	double y_start = 0;
	double y_step = y_end / (curvesNum + 1);
	for (int i = 0; i < curvesNum; i++){
		y_start = y_start + y_step;
		int pointIndex_zheng = obNoseSRV3D_closePoint(y_start, 1, border_regular);
		int pointIndex_fu = obNoseSRV3D_closePoint(y_start, -1, border_regular);
		stripesIndex.push_back(pointIndex_zheng);
		stripesIndex.push_back(pointIndex_fu);
	}

	for (int i = 0; i < noseBridgeIndex.size(); i++){
		geodesicCom geodesicTool2(pModel);
		vector<double> path1 = geodesicTool2.geodesicPath(noseMiddle, stripesIndex.at(2 * i));
		vector<double> path2 = geodesicTool2.geodesicPath(noseMiddle, stripesIndex.at(2 * i + 1));
		vector<vector<double>> stripes_1;
		vector<int> noseBridgeCurvesIndex_1;
		int pointNum = path1.size() / 3;
		for (int j = 0; j < pointNum; j++){
			double x1 = path1.at(3 * j);
			double y1 = path1.at(3 * j + 1);
			double z1 = path1.at(3 * j + 2);
			vector<double> point_i;
			point_i.push_back(x1);
			point_i.push_back(y1);
			point_i.push_back(z1);
			stripes_1.push_back(point_i);
			int point_index = glmIndexPoint(pModel, x1, y1, z1);
			noseBridgeCurvesIndex_1.push_back(point_index);
		}



		vector<vector<double>> stripes_2;
		vector<int> noseBridgeCurvesIndex_2;
		int pointNum2 = path2.size() / 3;
		for (int j = 0; j < pointNum2; j++){
			double x1 = path2.at(3 * j);
			double y1 = path2.at(3 * j + 1);
			double z1 = path2.at(3 * j + 2);
			vector<double> point_i;
			point_i.push_back(x1);
			point_i.push_back(y1);
			point_i.push_back(z1);
			stripes_2.push_back(point_i);
			int point_index = glmIndexPoint(pModel, x1, y1, z1);
			noseBridgeCurvesIndex_2.push_back(point_index);
		}

		noseBridgeCurves.push_back(stripes_1);
		noseBridgeCurves.push_back(stripes_2);

		noseBridgeCurvesIndex.push_back(noseBridgeCurvesIndex_1);
		noseBridgeCurvesIndex.push_back(noseBridgeCurvesIndex_2);
	}
}

//Çó¾Ö²¿×ø±ê
void obRCurves::obNoseSRV3D_noseCordinate(){
	//Ê×ÏÈ¼ÆËã¹æ·¶»¯µÄ²âµØ»·
	vector<CvPoint3D32f> border_regular = obNoseSRV3D_stripedRegular();
	//ÕÒµ½ÏàÓ¦µÄËÄ¸öµã
	int point1, point2, point3, point4;
	int p1_i, p2_i, p3_i, p4_i;
	double p2 = 1000;
	double p3 = 1000;
	double p4 = 1000;

	point1 = 0;
	for (int i = 0; i < border_regular.size(); i++){
		double x_i = border_regular.at(i).x;
		double y_i = border_regular.at(i).y;
		if (x_i < 0 && abs(y_i) < p2){
			point2 = i;
			p2 = abs(y_i);
		}
		if (y_i < 0 && abs(x_i) < p3){
			point3 = i;
			p3 = abs(x_i);
		}
		if (x_i > 0 && abs(y_i) < p4){
			point4 = i;
			p4 = abs(y_i);
		}
	}
	p1_i = gBC_index.at(point1);
	p2_i = gBC_index.at(point2);
	p3_i = gBC_index.at(point3);
	p4_i = gBC_index.at(point4);

	CvPoint3D32f vectorX;//¾Ö²¿×ø±êÏµµÄx·½Ïò
	vectorX.x = pModel->vertices[3 * p4_i] - pModel->vertices[3 * p2_i];
	vectorX.y = pModel->vertices[3 * p4_i + 1] - pModel->vertices[3 * p2_i + 1];
	vectorX.z = pModel->vertices[3 * p4_i + 2] - pModel->vertices[3 * p2_i + 2];

	CvPoint3D32f vectorY_wei;//ÒòÎª²»ÄÜÈ·¶¨¾Ö²¿×ø±êÏµyÏòÁ¿£¬ÄÇÃ´Ê×ÏÈ¼ÆËãÒ»¸ö½Ó½üyµÄÏòÁ¿
	vectorY_wei.x = pModel->vertices[3 * p1_i] - pModel->vertices[3 * p3_i];
	vectorY_wei.y = pModel->vertices[3 * p1_i + 1] - pModel->vertices[3 * p3_i + 1];
	vectorY_wei.z = pModel->vertices[3 * p1_i + 2] - pModel->vertices[3 * p3_i + 2];

	CvPoint3D32f vectorZ;//Çó¾Ö²¿×ø±êzµÄ·½Ïò£¬²æ³ËÉÏÃæÁ½¸öÏòÁ¿
	vectorZ.x = vectorX.y*vectorY_wei.z - vectorY_wei.y*vectorX.z;
	vectorZ.y = vectorY_wei.x*vectorX.z - vectorX.x*vectorY_wei.z;
	vectorZ.z = vectorX.x*vectorY_wei.y - vectorY_wei.x*vectorX.y;

	CvPoint3D32f vectorY;//ÓÉx,zÇó³öy
	vectorY.x = vectorX.y*vectorZ.z - vectorZ.y*vectorX.z;
	vectorY.y = vectorZ.x*vectorX.z - vectorX.x*vectorZ.z;
	vectorY.z = vectorX.x*vectorZ.y - vectorZ.x*vectorX.y;

	double normalX = sqrt(vectorX.x*vectorX.x + vectorX.y*vectorX.y + vectorX.z*vectorX.z);
	double normalY = sqrt(vectorY.x*vectorY.x + vectorY.y*vectorY.y + vectorY.z*vectorY.z);
	double normalZ = sqrt(vectorZ.x*vectorZ.x + vectorZ.y*vectorZ.y + vectorZ.z*vectorZ.z);

	vectorX.x = vectorX.x / normalX;
	vectorX.y = vectorX.y / normalX;
	vectorX.z = vectorX.z / normalX;

	vectorY.x = vectorY.x / normalY;
	vectorY.y = vectorY.y / normalY;
	vectorY.z = vectorY.z / normalY;

	vectorZ.x = vectorZ.x / normalZ;
	vectorZ.y = vectorZ.y / normalZ;
	vectorZ.z = vectorZ.z / normalZ;

	noseCordinate.push_back(vectorX);
	noseCordinate.push_back(vectorY);
	noseCordinate.push_back(vectorZ);
}

vector<CvPoint3D32f> obRCurves::obNoseSRV3D_stripedRegular(){

	vector<CvPoint3D32f> border;
	for (int i = 0; i < gBC_index.size(); i++){
		int index_i = gBC_index.at(i);
		double xi = pModel->vertices[3 * index_i];
		double yi = pModel->vertices[3 * index_i + 1];
		double zi = pModel->vertices[3 * index_i + 2];
		CvPoint3D32f point_i;
		point_i.x = xi;
		point_i.y = yi;
		point_i.z = zi;
		border.push_back(point_i);
	}


	//Ê×ÏÈ»ñÈ¡Á½¸ö±ê¶¨µãµÄ×ø±ê
	double x_noseP = pModel->vertices[3 * noseMiddle];
	double y_noseP = pModel->vertices[3 * noseMiddle + 1];
	double x_noseM = border.at(0).x - x_noseP;
	double y_noseM = border.at(0).y - y_noseP;

	//Íê³ÉÆ½ÒÆÓëÐý×ª
	double x_noseM_new = 0;
	double y_noseM_new = sqrt(x_noseM*x_noseM + y_noseM*y_noseM);
	double lengthNormal = y_noseM_new;
	double lengthMulti = 6 / y_noseM_new;
	double cos_angle = (y_noseM_new / x_noseM) / (x_noseM / y_noseM + y_noseM / x_noseM);
	double sin_angle = (y_noseM_new / y_noseM) / (x_noseM / y_noseM + y_noseM / x_noseM);


	vector<CvPoint3D32f> border_i_regular;
	for (int j = 0; j < border.size(); j++){
		CvPoint3D32f ij_regular;
		//Íê³ÉÆ½ÒÆ
		ij_regular.x = border.at(j).x - x_noseP;
		ij_regular.y = border.at(j).y - y_noseP;
		ij_regular.z = 0;
		//Íê³ÉÐý×ª
		double x_new = ij_regular.x*cos_angle - ij_regular.y*sin_angle;
		double y_new = ij_regular.x*sin_angle + ij_regular.y*cos_angle;
		x_new = x_new * lengthMulti;
		y_new = y_new * lengthMulti;
		ij_regular.x = x_new;
		ij_regular.y = y_new;
		border_i_regular.push_back(ij_regular);
	}
	return border_i_regular;
}

void obRCurves::obNoseSRV3D_NoseShapeSRV3D(){

	for (int i = 0; i < noseBridgeCurves.size(); i++){
		vector<vector<double>> noseBridgeCurves_i = noseBridgeCurves.at(i);
		srv_object3D obs1(noseBridgeCurves_i, 20, noseCordinate);
		NoseShapeSRV3D.push_back(obs1);
	}

}


//¹¦ÄÜº¯Êý
int obRCurves::obNoseSRV3D_pointexist_return(int point, vector<int> data){
	for (int i = 0; i < data.size(); i++){
		if (point == data.at(i)){
			return i;
		}
	}
	return -1;

}

#pragma endregion
