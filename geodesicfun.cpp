#include "../stdafx.h"
#include <iostream>
#include <fstream>
#include <Geodesic.h>
#include "../Glm/glm.h"
#include <vector>
#pragma once
using namespace std;


class geodesicCom{
public:
	vector<double> vX;
	vector<double> vY;
	vector<double> vZ;	
	vector<int> tempDraw;//¼ÇÂ¼ÔÝÊ±´æ´¢µÄÊý¾Ý
	//double tempArea;//¼ÇÂ¼ÔÝÊ±´æ´¢µÄÃæ»ý
	//vector<double> globalGeodiscDistance;//ÖÐµãµ½È«¾ÖµÄ²âµØ¾àÀë
	Geodesic geodesic_Global;
	GLMmodel* pModel;
public:
	geodesicCom(GLMmodel* pModel);
	void geodesicVectorDraw(vector<double> vV);
	void geodesicPointDraw(int s, int v, double r, double g, double b);
	void geodesicPointDraw2(double middle, double radius, double r, double g, double b);
	void geodesicPointDraw3(vector<int> pointDraw, double r, double g, double b);
	void loadGLMmodel(GLMmodel* pModel, Geodesic& g);
	double geodesicPathLen(int s, int v);
	vector<double> geodesicPath_throw_exception(int s, int t)throw(exception&, bad_alloc&);
	vector<double> geodesicPath_process_exception(int s, int t);
	vector<double> geodesicPath(int s, int t);
	vector<double> geodesicDistances(int s, double tolerance);//¼ÆËãÈ«¾ÖµÄ²âµØ¾àÀë
	vector<int> geodesicMap(int s, GLMmodel* pModel,double dis);
	
	//¼ÆËãµÈ²âµØ´ø
	vector<int> geodesicISOstripe(double radius_low, double radius, vector<double> dis);
	vector<vector<int>> geodesicISOstripes(int noseMiddle, int eyebrowMiddle);

	//¼ÆËãµÈ²âµØ´øµÄ±ß½ç,ÒÔ¼°±ß½çµãµÄÏàÁÚ¹ØÏµ½á¹¹.
	vector<vector<int>> geodesicBorderStripe(vector<vector<int>> Stripes, vector<vector<int>> neiborPoint);
	vector<vector<vector<int>>> geodesicBorderStripe_neibor(vector<vector<int>> borders, vector<vector<int>> neiborPoint);

	//¼ÆËã±ÇÁºµÄÂ·¾¶
	vector<vector<double>> geodesicNosePath(vector<int> points, string filename);//pointsµÄË³ÐòÒÀ´ÎÊÇ±Ç¼âµã£¬Ã¼ÐÄµã£¬Èý¸öÍâÈ¦µÄ±ê¶¨µã


	//¶ÔÍâµÄÕûºÏ½Ó¿Ú
	vector<vector<int>> geodesicBorderCompute(int noseMiddle, int eyeBrow, vector<vector<int>> neiborPoint, string filenameStore);


	//»­³öµÈ²âµØ´ø
	void geodesicStripsDraw(vector<vector<int>> pointDraw);
	void geodesicPointDraw(vector<int> pointDraw,double r, double g, double b);

	//Ò»Ð©»ù±¾µÄ¹¦ÄÜº¯Êý
	bool pointexist(vector<int> point, vector<int> data);
	int pointexist_return(int point, vector<int> data);
	bool pointexist(int point, vector<int> data);
	
	//´æ´¢Êý¾Ý±ß½ç
	void geodesicBorderStripeWrite(vector<vector<int>> circlePoint, string fileName, int noseMiddle, int eyeBrow, vector<vector<vector<int>>> neibor);
};






