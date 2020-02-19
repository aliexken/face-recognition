#include <OpenGL/glut.h>
#include <vector>
using namespace std;

//»ñµÃÄ£ÐÍµÄ°üÎ§ºÐ×Ó
vector<int> glmBox();

//¼ÆËã×ø±êÖáµÄ±ß½çµã
int glmNosePoint(GLMmodel* model);

//¼ÆËãÏàÁÚµãµÄÊý¾Ý½á¹¹
vector<vector <int>> glmPointNeibor(GLMmodel* model);//¼ÆËãµãÔÆÖÐÃ¿Ò»µãµÄÏàÁÚµã

bool glmPointNeibor_pointexist(int point, vector<int> data);//Á´±íÈ¥ÖØ

//Ìá¹©Ä³Ò»µã×ø±ê£¬»ñµÃ¸Ã×ø±êµÄindex
int glmIndexPoint(GLMmodel* model,double x,double y,double z);

int glmIndexPoint(GLMmodel* model, double x, double y);

//Çó²âµØµØ´øµÄÈý½ÇÃæÆ¬±íÊ¾
vector<vector<double>> glmTriangle(GLMmodel* model, vector<vector<int>> pointsIndex);
