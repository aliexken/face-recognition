#include "stdafx.h"
#include "../Geodesic/geodesicFun.h"
#include "Method_1_Berriti.h"
#include<iostream> 
#include<string> 
using namespace std;

objBerriti::objBerriti(string fileNameObj_o, int circleNum_o, int noseMiddle_o, int eyebrowsMiddle_o,int database_i){//³õÊ¼»¯
	cout << "objBerriti:objBerriti³õÊ¼»¯¿ªÊ¼£º" << endl;
	dataBaseIndex = database_i;
	fileNameObj = fileNameObj_o;//³õÊ¼»¯
	circleNum = circleNum_o;//³õÊ¼»¯
	noseMiddle = noseMiddle_o;//³õÊ¼»¯
	eyebrowsMiddle = eyebrowsMiddle_o;//³õÊ¼»¯
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
	normalLength = geodesicTool.geodesicPathLen(noseMiddle, eyebrowsMiddle);
	normalLength = normalLength * 4 / 3;
	
	objBerriti_CirclePoint();
	objBerriti_3DWWComputation();
	objBerriti_WriteRecord();
}

objBerriti::objBerriti(string fileNameCircle, int dataBase_i){//³õÊ¼»¯
	//vector<vector<vector<double>>> points;
	dataBaseIndex = dataBase_i;
	fstream out1;
	out1.open(fileNameCircle, ios::_Nocreate);
	if (!out1){
		cout << "read file error!(file not exist)" << endl;	
	}
	//circle3DWWResult
	out1 >> circleNum;

	for (int i = 0; i < circleNum; i++){
		vector<double> circle3DWWResult_i;
		int circleNum_i;
		out1 >> circleNum_i;		
		for (int j = 0; j < circleNum_i; j++){
			double circle3DWWResult_ij;			
			out1 >> circle3DWWResult_ij;
			circle3DWWResult_i.push_back(circle3DWWResult_ij);
		}	
		circle3DWWResult.push_back(circle3DWWResult_i);
		
	}
	out1.close();
}

void objBerriti::objBerriti_CirclePoint(){
	geodesicCom geodesicTool(pModel);
	
	vector<double> dis = geodesicTool.geodesicDistances(noseMiddle - 1, -1);
	vector<int> disInt;
	for (int i = 0; i < dis.size(); i++){
		if (dis.at(i) > normalLength || dis.at(i) < 0){
			continue;		
		}
		else{
			disInt.push_back(i);		
		}	
	}

	//vector<vector<int>> edagePointWhole;
	for (int i = 0; i < circleNum; i++){
		cout << "objBerriti_CirclePoint:µÚ" << i << "Ìõ²âµØ»·ÕýÔÚ¼ÆËãÖÐ..." << endl;
		vector<int> edagePoint;
		double circle_s = normalLength*i / circleNum;
		double circle_e = normalLength*(i+1) / circleNum;
		for (int j = 0; j < disInt.size(); j++){
			int index_j = disInt.at(j);
			double index_j_length = dis.at(index_j);
			if (index_j_length <= circle_e&&index_j_length >= circle_s){
				edagePoint.push_back(index_j + 1);			
			}			
		}
		pointsIndex.push_back(edagePoint);
	}
	for (int i = 0; i < pointsIndex.size(); i++){
		vector<int> edagePoint_i = pointsIndex.at(i);
		vector<vector<double>> circle_i;
		for (int j = 0; j < edagePoint_i.size(); j++){	
			double x_j = pModel->vertices[3 * edagePoint_i.at(j)];
			double y_j = pModel->vertices[3 * edagePoint_i.at(j) + 1];
			double z_j = pModel->vertices[3 * edagePoint_i.at(j) + 2];
			vector<double> circle_ij;
			circle_ij.push_back(x_j);
			circle_ij.push_back(y_j);
			circle_ij.push_back(z_j);
			circle_i.push_back(circle_ij);
		}	
		points.push_back(circle_i);
	}
}

double objBerriti::objBerriti_threhold(int trangleNum){

	
	int b1 = pModel->triangles[(trangleNum)].vindices[0];
	int b2 = pModel->triangles[(trangleNum)].vindices[1];
	int b3 = pModel->triangles[(trangleNum)].vindices[2];

	double x1 = pModel->vertices[3 * b1];
	double y1 = pModel->vertices[3 * b1 + 1];
	double z1 = pModel->vertices[3 * b1 + 2];

	double x2 = pModel->vertices[3 * b2];
	double y2 = pModel->vertices[3 * b2 + 1];
	double z2 = pModel->vertices[3 * b2 + 2];

	double x3 = pModel->vertices[3 * b3];
	double y3 = pModel->vertices[3 * b3 + 1];
	double z3 = pModel->vertices[3 * b3 + 2];

	double b1l = sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1) + (z2 - z1)*(z2 - z1));
	double b2l = sqrt((x3 - x1)*(x3 - x1) + (y3 - y1)*(y3 - y1) + (z3 - z1)*(z3 - z1));
	double b3l = sqrt((x3 - x2)*(x3 - x2) + (y3 - y2)*(y3 - y2) + (z3 - z2)*(z3 - z2));

	double threhold = (b1l + b2l + b3l) / 3;

	return threhold;

}

void objBerriti::objBerriti_3DWWComputation(){
	//¼ÆËã²âµØ»·3D×ß²éÊý¾Ý
	//Ê×ÏÈ¼ÆËãÒ»ÏÂthrehold£¬È¡³öÈý¸öÈý½ÇÐÎËãÆ½¾ùÖµ
	
	double threhold1 = objBerriti_threhold(pModel->numtriangles / 2);
	double threhold2 = objBerriti_threhold(pModel->numtriangles / 3);
	double threhold3 = objBerriti_threhold(pModel->numtriangles / 4);
	double threhold = 0;

	if (threhold1 >= threhold2&&threhold1 >= threhold3){
		threhold = threhold1;	
	}
	else if (threhold2 >= threhold3){
		threhold = threhold2;	
	}
	else{
		threhold = threhold3;	
	}
	
	for (int i = 0; i < circleNum-1; i++){
		//È¡³öÁ½Ìõ²âµØÏß
		vector<vector<double>> stripe1 = points.at(i);
		vector<vector<double>> stripe2 = points.at(i + 1);
		//Çó³ö0LA,1HA,2DA,3LB,4HB,5DB,6LAB,7HAB,8DAB

		int sunPoint = stripe1.size() * stripe2.size();//¼ÇÂ¼ËùÓÐµã¶ÔµÄ×ÜÊý
		vector<double> Kijk = objBerriti_Kijk_Compute(stripe1, stripe2);
		double kH0 = Kijk[6] * Kijk[1] * Kijk[4] * Kijk[2] * Kijk[5];
		double kV0 = Kijk[0] * Kijk[3] * Kijk[7] * Kijk[2] * Kijk[5];
		double kD0 = Kijk[0] * Kijk[3] * Kijk[1] * Kijk[4] * Kijk[8];
		double kHV0 = Kijk[6] * Kijk[7] * Kijk[2] * Kijk[5];		
		double kHD0 = Kijk[6] * Kijk[1] * Kijk[4] * Kijk[8];
		double kVD0 = Kijk[0] * Kijk[3] * Kijk[7] * Kijk[8];

		double wH = 0; double wH0 = 0;
		double wV = 0; double wV0 = 0;
		double wD = 0; double wD0 = 0;
		double wXY = 0; double wXY0 = 0;
		double wXZ = 0; double wXZ0 = 0;
		double wYZ = 0; double wYZ0 = 0;
		vector<long double> num_3DWW;
		
		/*
		for (int i = 0; i < stripe1.size(); i++){
			//È¡³öµÚÒ»¸ö×ø±ê
			
			double x1 = stripe1.at(i).at(0);
			double y1 = stripe1.at(i).at(1);
			double z1 = stripe1.at(i).at(2);
			for (int j = 0; j < stripe2.size(); j++){
				//È¡³öµÚ¶þ¸ö×ø±ê				
				double x2 = stripe2.at(i).at(0);
				double y2 = stripe2.at(i).at(1);
				double z2 = stripe2.at(i).at(2);
				double xDiff = x2 - x1;
				double yDiff = y2 - y1;
				double zDiff = z2 - z1;
				int xi;
				int yi;
				int zi;
				if (xDiff > threhold){
					xi = 0;
				}
				else if (xDiff <= threhold&&xDiff >= -threhold){
					xi = 1;
				}
				else{
					xi = 2;
				}

				if (yDiff > threhold){
					yi = 0;
				}
				else if (yDiff <= threhold&&yDiff >= -threhold){
					yi = 1;
				}
				else{
					yi = 2;
				}

				if (zDiff > threhold){
					zi = 0;
				}
				else if (zDiff <= threhold&&zDiff >= -threhold){
					zi = 1;
				}
				else{
					zi = 2;
				}
				int signNum = xi + 3 * yi + 3 * 3 * zi;
				num_3DWW[signNum]++;
				//¼ÇÂ¼ÏàÓ¦µÄ±ê×¼·ûºÅ
			}
		}
		*/
		num_3DWW = objBerriti_num_3DWW(threhold, stripe1, stripe2);
		
		wH = (num_3DWW[0] + num_3DWW[6] + num_3DWW[18] + num_3DWW[24]) / (double)sunPoint;
		wV = (num_3DWW[2] + num_3DWW[0] + num_3DWW[20] + num_3DWW[18]) / (double)sunPoint;
		wD = (num_3DWW[0] + num_3DWW[6] + num_3DWW[2] + num_3DWW[8]) / (double)sunPoint;
		wXY = (num_3DWW[8] + num_3DWW[0] + num_3DWW[26] + num_3DWW[18]) / (double)sunPoint;
		wXZ = (num_3DWW[26] + num_3DWW[20] + num_3DWW[0] + num_3DWW[6]) / (double)sunPoint;
		wYZ = (num_3DWW[2] + num_3DWW[0] + num_3DWW[26] + num_3DWW[24]) / (double)sunPoint;
		//Kijk6,7,8 6LAB,7HAB,8DAB
		//wH0 = (double)(num_3DWW[1] + num_3DWW[7] + num_3DWW[19] + num_3DWW[25]) / (double)sunPoint*(Kijk[6] / Kijk[7]);
		//wV0 = (double)(num_3DWW[3] + num_3DWW[5] + num_3DWW[23] + num_3DWW[21]) / (double)sunPoint*(Kijk[7] / Kijk[8]);
		//wD0 = (double)(num_3DWW[9] + num_3DWW[15] + num_3DWW[11] + num_3DWW[17]) / (double)sunPoint*(Kijk[6] / Kijk[8]);
		//wXY0 = (double)(num_3DWW[4] + num_3DWW[22]) / (double)sunPoint*(Kijk[6] / Kijk[7]);
		//wXZ0 = (double)(num_3DWW[10] + num_3DWW[16]) / (double)sunPoint*(Kijk[6] / Kijk[8]);
		//wYZ0 = (double)(num_3DWW[12] + num_3DWW[14]) / (double)sunPoint*(Kijk[7] / Kijk[8]);

		wH0 = (double)(num_3DWW[1] + num_3DWW[7] + num_3DWW[19] + num_3DWW[25]) / kH0;
		wV0 = (double)(num_3DWW[3] + num_3DWW[5] + num_3DWW[23] + num_3DWW[21]) / kV0;
		wD0 = (double)(num_3DWW[9] + num_3DWW[15] + num_3DWW[11] + num_3DWW[17]) / kD0;
		wXY0 = (double)(num_3DWW[4] + num_3DWW[22]) / kHV0;
		wXZ0 = (double)(num_3DWW[10] + num_3DWW[16]) / kHD0;
		wYZ0 = (double)(num_3DWW[12] + num_3DWW[14]) / kVD0;

		vector<double> result;
		result.push_back(wH);
		result.push_back(wV);
		result.push_back(wD);
		result.push_back(wXY);
		result.push_back(wXZ);
		result.push_back(wYZ);

		result.push_back(wH0);
		result.push_back(wV0);
		result.push_back(wD0);
		result.push_back(wXY0);
		result.push_back(wXZ0);
		result.push_back(wYZ0);

		circle3DWWResult.push_back(result);	
	}	
}

void objBerriti::objBerriti_WriteRecord(){
	//Ê×ÏÈ»ñµÃÎÄ¼þÃû
    //D:\facedata\face_data_experiment_new\Data_Texas3D_obj\004\Clean_0005_004_20050913101316_Range.png.obj
	
	string fileNameWhole;
	
	if (dataBaseIndex == 1){
		string fileName1 = "D:\\facedata\\face_data_experiment_new\\Method_1_Berriti\\Texas3D\\";
		int numFile = fileNameObj.find("Clean_", 1);
		string fileName2 = fileNameObj.substr(numFile, 39);
		//cout << fileName2;
		string fileName3 = ".txt";
		fileNameWhole = fileName1 + fileName2 + fileName3;	
	}
	else if (dataBaseIndex == 2){
	
	
	}
	else if (dataBaseIndex == 3){
		string fileName1 = "D:\\facedata\\face_data_experiment_new\\Method_1_Berriti\\Gavab\\";
		int numFile = fileNameObj.find("cara", 1);
		int end = fileNameObj.find(".obj", 1);
		string fileName2 = fileNameObj.substr(numFile, end - numFile);
		//cout << fileName2;
		string fileName3 = ".txt";
		fileNameWhole = fileName1 + fileName2 + fileName3;	
	}

	else{
		cout << "Ð´ÎÄ¼þ³ö´í£¡" << endl;
		return;	
	}
	
	ofstream f1(fileNameWhole, ios::_Noreplace);
	if (!f1.is_open()) {
		cout << "cannot open file.(file exsit!)" << endl;	
	}
	else{

		f1 << circle3DWWResult.size() << endl;
		for (int i = 0; i < circle3DWWResult.size(); i++){
			vector<double> circle3DWWResult_i = circle3DWWResult.at(i);
			f1 << circle3DWWResult_i.size() << endl;
			for (int j = 0; j < circle3DWWResult_i.size(); j++){
				f1 << circle3DWWResult_i.at(j) << " ";
			}
			f1 << endl;
		}
		f1.close();
	
	}
	
}

double objBerriti::objBerriti_FinalResult_2Obj(objBerriti obj2){

	if (circle3DWWResult.size() != obj2.circle3DWWResult.size()){
		cout << "objBerriti_FinalResult_2Obj error!(µÈ²âµØ»·Êý¾Ý²»Ò»ÖÂ£¡)"<<endl;
		return -1;	
	}

	vector<double> result;

	for (int i = 0; i < circle3DWWResult.size(); i++){
		vector<double> ob1_i = circle3DWWResult.at(i);
		vector<double> ob2_i = obj2.circle3DWWResult.at(i);
		double d = abs(ob1_i.at(0) - ob2_i.at(0)) / 12 + abs(ob1_i.at(1) - ob2_i.at(1)) / 12 +
			abs(ob1_i.at(2) - ob2_i.at(2)) / 6 + abs(ob1_i.at(3) - ob2_i.at(3)) / 12 +
			abs(ob1_i.at(4) - ob2_i.at(4)) / 12 + abs(ob1_i.at(5) - ob2_i.at(5)) / 6 +
			abs(ob1_i.at(6) - ob2_i.at(6)) / 18 + abs(ob1_i.at(7) - ob2_i.at(7)) / 18 +
			abs(ob1_i.at(8) - ob2_i.at(8)) / 18 + abs(ob1_i.at(9) - ob2_i.at(9)) / 18 +
			abs(ob1_i.at(10) - ob2_i.at(10)) / 18 + abs(ob1_i.at(11) - ob2_i.at(11)) / 18;
		result.push_back(d);
	}
	
	double finalResult = 0;
	for (int i = 0; i < result.size(); i++){
		finalResult = result.at(i) + finalResult;	
	}
	return finalResult;

}

vector<double> objBerriti::objBerriti_Kijk_Compute(vector<vector<double>> s1, vector<vector<double>> s2){
	
	//Çó³öLA,LB,HA,HB,DA,DB,LAB,HAB,DAB;
    //L:x
	//H:y
	//D:z
    //Ê×ÏÈÇó³öÁ½¸öµÈ²âµØ´øµÄÏàÓ¦Öµ
	double s1max_x = -1000;
	double s1max_y = -1000;
	double s1max_z = -1000;
	double s1min_x = 1000;
	double s1min_y = 1000;
	double s1min_z = 1000;	

	for (int i = 0; i < s1.size(); i++){
		vector<double> s1_point = s1.at(i);
		int s1_x = s1_point.at(0);
		int s1_y = s1_point.at(1);
		int s1_z = s1_point.at(2);
		if (s1_x > s1max_x){
			s1max_x = s1_x;		
		}
		if (s1_y > s1max_y){
			s1max_y = s1_y;		
		}
		if (s1_z > s1max_z){
			s1max_z = s1_z;
		}
		if (s1_x < s1min_x){
			s1min_x = s1_x;
		}
		if (s1_y < s1min_y){
			s1min_y = s1_y;
		}
		if (s1_z < s1min_z){
			s1min_z = s1_z;
		}
	}

	double LA = s1max_x - s1min_x;
	double HA = s1max_y - s1min_y;
	double DA = s1max_z - s1min_z;

	double s2max_x = -10000;
	double s2max_y = -10000;
	double s2max_z = -10000;
	double s2min_x = 10000;
	double s2min_y = 10000;
	double s2min_z = 10000;

	for (int i = 0; i < s2.size(); i++){
		vector<double> s2_point = s2.at(i);
		int s2_x = s2_point.at(0);
		int s2_y = s2_point.at(1);
		int s2_z = s2_point.at(2);
		if (s2_x > s1max_x){
			s2max_x = s2_x;
		}
		if (s2_y > s2max_y){
			s2max_y = s2_y;
		}
		if (s2_z > s2max_z){
			s2max_z = s2_z;
		}
		if (s2_x < s2min_x){
			s2min_x = s2_x;
		}
		if (s2_y < s2min_y){
			s2min_y = s2_y;
		}
		if (s2_z < s2min_z){
			s2min_z = s2_z;
		}
	}

	double LB = s2max_x - s2min_x;
	double HB = s2max_y - s2min_y;
	double DB = s2max_z - s2min_z;

	double s12max_x;
	double s12max_y;
	double s12max_z;	

	if (s2max_x > s1max_x){
		s12max_x = s2max_x;
	}
	else{
		s12max_x = s1max_x;
	}

	if (s2max_y > s1max_y){
		s12max_y = s2max_y;
	}
	else{
		s12max_y = s1max_y;
	}

	if (s2max_z > s1max_z){
		s12max_z = s2max_z;
	}
	else{
		s12max_z = s1max_z;
	}

	double s12min_x;
	double s12min_y;
	double s12min_z;

	if (s2min_x < s1min_x){
		s12min_x = s2min_x;
	}
	else{
		s12min_x = s1min_x;
	}
	if (s2min_y < s1min_y){
		s12min_y = s2min_y;
	}
	else{
		s12min_y = s1min_y;
	}
	if (s2min_z < s1min_z){
		s12min_z = s2min_z;
	}
	else{
		s12min_z = s1min_z;
	}

	double LAB = s12max_x - s12min_x;
	double HAB = s12max_y - s12min_y;
	double DAB = s12max_z - s12min_z;

	vector<double> Kijk;
	Kijk.push_back(LA);
	Kijk.push_back(HA);
	Kijk.push_back(DA);
	Kijk.push_back(LB);
	Kijk.push_back(HB);
	Kijk.push_back(DB);
	Kijk.push_back(LAB);
	Kijk.push_back(HAB);
	Kijk.push_back(DAB);

	return Kijk;
}

vector<long double> objBerriti::objBerriti_num_3DWW(double threhold, vector<vector<double>> stripe1, vector<vector<double>> stripe2){

	int xNum[3] = { 0, 0, 0 };
	int yNum[3] = { 0, 0, 0 };
	int zNum[3] = { 0, 0, 0 };
	/*
	
	
	for (int i = 0; i < stripe1.size(); i++){
		//È¡³öµÚÒ»¸ö×ø±ê

		double x1 = stripe1.at(i).at(0);
		double y1 = stripe1.at(i).at(1);
		double z1 = stripe1.at(i).at(2);
		for (int j = 0; j < stripe2.size(); j++){
			//È¡³öµÚ¶þ¸ö×ø±ê				
			double x2 = stripe2.at(i).at(0);
			double y2 = stripe2.at(i).at(1);
			double z2 = stripe2.at(i).at(2);
			double xDiff = x2 - x1;
			double yDiff = y2 - y1;
			double zDiff = z2 - z1;
			int xi;
			int yi;
			int zi;
			if (xDiff > threhold){
				xNum[0]++;
			}
			else if (xDiff <= threhold&&xDiff >= -threhold){
				xNum[1]++;
			}
			else{
				xNum[2]++;
			}

			if (yDiff > threhold){
				yNum[0]++;
			}
			else if (yDiff <= threhold&&yDiff >= -threhold){
				yNum[1]++;
			}
			else{
				yNum[2]++;
			}

			if (zDiff > threhold){
				zNum[0]++;
			}
			else if (zDiff <= threhold&&zDiff >= -threhold){
				zNum[1]++;
			}
			else{
				zNum[2]++;
			}
		}
	}

	vector<long long> num_3DWW;
	for (int i = 0; i < 27; i++){
		int zi = i / 9;
		int yi = (i - zi * 9) / 3;
		int xi = i % 3;
		long long xN = xNum[xi];
		long long yN = yNum[yi];
		long long zN = zNum[zi];


		long long i_index = xN*yN*zN;
		cout << xNum[xi] << "," << yNum[yi] << "," << zNum[zi] << ":" << (long long)xNum[xi] * (long long)yNum[yi] * (long long)zNum[zi]<< endl;
		num_3DWW.push_back(i_index);
	}
	return num_3DWW;
	*/
	vector<long double> num_3DWW;
	for (int i = 0; i < 27; i++){
		num_3DWW.push_back(0);
	
	}
	for (int i = 0; i < stripe1.size(); i++){
		//È¡³öµÚÒ»¸ö×ø±ê

		double x1 = stripe1.at(i).at(0);
		double y1 = stripe1.at(i).at(1);
		double z1 = stripe1.at(i).at(2);
		for (int j = 0; j < stripe2.size(); j++){
			//È¡³öµÚ¶þ¸ö×ø±ê				
			double x2 = stripe2.at(i).at(0);
			double y2 = stripe2.at(i).at(1);
			double z2 = stripe2.at(i).at(2);
			double xDiff = x2 - x1;//LAB
			double yDiff = y2 - y1;//HAB
			double zDiff = z2 - z1;//DAB
			int xi;
			int yi;
			int zi;
			if (xDiff > threhold){
				xi = 0;
			}
			else if (xDiff <= threhold&&xDiff >= -threhold){
				xi = 1;
			}
			else{
				xi = 2;
			}

			if (yDiff > threhold){
				yi = 0;
			}
			else if (yDiff <= threhold&&yDiff >= -threhold){
				yi = 1;
			}
			else{
				yi = 2;
			}

			if (zDiff > threhold){
				zi = 0;
			}
			else if (zDiff <= threhold&&zDiff >= -threhold){
				zi = 1;
			}
			else{
				zi = 2;
			}
			int signNum = xi + 3 * yi + 3 * 3 * zi;
			if (xi != 1 && yi != 1 && zi != 1){
				num_3DWW[signNum] = num_3DWW[signNum]+1;
			}
			else{
				if (xi == 1 && yi != 1 && zi != 1){
					num_3DWW[signNum] = num_3DWW[signNum] + abs(xDiff);
					int signNum1 = 0 + 3 * yi + 3 * 3 * zi;
					int signNum2 = 2 + 3 * yi + 3 * 3 * zi;
					num_3DWW[signNum1] = num_3DWW[signNum1] + 0.5;
					num_3DWW[signNum2] = num_3DWW[signNum2] + 0.5;
				}
				else if (xi != 1 && yi == 1 && zi != 1){
					num_3DWW[signNum] = num_3DWW[signNum] + abs(yDiff);
					int signNum1 = xi + 3 * 0 + 3 * 3 * zi;
					int signNum2 = xi + 3 * 2 + 3 * 3 * zi;
					num_3DWW[signNum1] = num_3DWW[signNum1] + 0.5;
					num_3DWW[signNum2] = num_3DWW[signNum2] + 0.5;
				
				}
				else if (xi != 1 && yi != 1 && zi == 1){
					num_3DWW[signNum] = num_3DWW[signNum] + abs(zDiff);
					int signNum1 = xi + 3 * yi + 3 * 3 * 0;
					int signNum2 = xi + 3 * yi + 3 * 3 * 2;
					num_3DWW[signNum1] = num_3DWW[signNum1] + 0.5;
					num_3DWW[signNum2] = num_3DWW[signNum2] + 0.5;
				}
				else if (xi == 1 && yi == 1 && zi != 1){
					num_3DWW[signNum] = num_3DWW[signNum] + abs(xDiff*yDiff);
					int signNum1 = 0 + 3 * 0 + 3 * 3 * zi;
					int signNum2 = 0 + 3 * 2 + 3 * 3 * zi;
					int signNum3 = 2 + 3 * 0 + 3 * 3 * zi;
					int signNum4 = 2 + 3 * 2 + 3 * 3 * zi;
					num_3DWW[signNum1] = num_3DWW[signNum1] + 0.25;
					num_3DWW[signNum2] = num_3DWW[signNum2] + 0.25;
					num_3DWW[signNum3] = num_3DWW[signNum3] + 0.25;
					num_3DWW[signNum4] = num_3DWW[signNum4] + 0.25;
				
				}
				else if (xi == 1 && yi != 1 && zi == 1){
					num_3DWW[signNum] = num_3DWW[signNum] + abs(xDiff*zDiff);
					int signNum1 = 0 + 3 * yi + 3 * 3 * 0;
					int signNum2 = 0 + 3 * yi + 3 * 3 * 2;
					int signNum3 = 2 + 3 * yi + 3 * 3 * 0;
					int signNum4 = 2 + 3 * yi + 3 * 3 * 2;
					num_3DWW[signNum1] = num_3DWW[signNum1] + 0.25;
					num_3DWW[signNum2] = num_3DWW[signNum2] + 0.25;
					num_3DWW[signNum3] = num_3DWW[signNum3] + 0.25;
					num_3DWW[signNum4] = num_3DWW[signNum4] + 0.25;

				}
				else if (xi != 1 && yi == 1 && zi == 1){
					num_3DWW[signNum] = num_3DWW[signNum] + abs(yDiff*zDiff);
					int signNum1 = xi + 3 * 0 + 3 * 3 * 0;
					int signNum2 = xi + 3 * 0 + 3 * 3 * 2;
					int signNum3 = xi + 3 * 2 + 3 * 3 * 0;
					int signNum4 = xi + 3 * 2 + 3 * 3 * 2;
					num_3DWW[signNum1] = num_3DWW[signNum1] + 0.25;
					num_3DWW[signNum2] = num_3DWW[signNum2] + 0.25;
					num_3DWW[signNum3] = num_3DWW[signNum3] + 0.25;
					num_3DWW[signNum4] = num_3DWW[signNum4] + 0.25;

				}
				else{
					num_3DWW[signNum] = num_3DWW[signNum] + 1;
					int signNum1 = 0 + 3 * 0 + 3 * 3 * 0;
					int signNum2 = 0 + 3 * 0 + 3 * 3 * 2;
					int signNum3 = 0 + 3 * 2 + 3 * 3 * 0;
					int signNum4 = 0 + 3 * 2 + 3 * 3 * 2;
					int signNum5 = 2 + 3 * 0 + 3 * 3 * 0;
					int signNum6 = 2 + 3 * 0 + 3 * 3 * 2;
					int signNum7 = 2 + 3 * 2 + 3 * 3 * 0;
					int signNum8 = 2 + 3 * 2 + 3 * 3 * 2;
					num_3DWW[signNum1] = num_3DWW[signNum1] + 0.125;
					num_3DWW[signNum2] = num_3DWW[signNum2] + 0.125;
					num_3DWW[signNum3] = num_3DWW[signNum3] + 0.125;
					num_3DWW[signNum4] = num_3DWW[signNum4] + 0.125;
					num_3DWW[signNum5] = num_3DWW[signNum5] + 0.125;
					num_3DWW[signNum6] = num_3DWW[signNum6] + 0.125;
					num_3DWW[signNum7] = num_3DWW[signNum7] + 0.125;
					num_3DWW[signNum8] = num_3DWW[signNum8] + 0.125;
				}
				
			
			}
			
			//¼ÇÂ¼ÏàÓ¦µÄ±ê×¼·ûºÅ
		}
	}
	return num_3DWW;
}
