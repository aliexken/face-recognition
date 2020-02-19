#include "../stdafx.h"
#include "../Glm/glm.h"
#include "../Glm/glmVector.h"
#include "../Geodesic/geodesicFun.h"


class objBerriti{
private:
	int dataBaseIndex;
	int circleNum;
	int noseMiddle;
	int eyebrowsMiddle;	
	double normalLength;
	//geodesicCom geodesicTool;
	GLMmodel* pModel;
public:
	string fileNameObj;
	vector<vector<vector<double>>> points;
	vector<vector<int>> pointsIndex;
	vector<vector<double>> circle3DWWResult;

public:
	objBerriti(string fileNameObj, int circleNum, int noseMiddle,int eyebrowsMiddle,int database_i);//³õÊ¼»¯
	objBerriti(string fileNameCircle, int database_i);//³õÊ¼»¯
	void objBerriti_CirclePoint();	
	double objBerriti_threhold(int trangleNum);
	
	void objBerriti_3DWWComputation();
	void objBerriti_WriteRecord();
	double objBerriti_FinalResult_2Obj(objBerriti obj2);

private:
	vector<double> objBerriti_Kijk_Compute(vector<vector<double>> s1, vector<vector<double>> s2);
	vector<long double> objBerriti_num_3DWW(double threhold, vector<vector<double>> s1, vector<vector<double>> s2);
};
