#pragma  once
#include <vector>
using namespace std;

struct VC_Point
{
	double Score;
	double z;
	int R;
	int G;
	int B;
	double x;
	double y;
	int cam[11];
};

class myData
{
private:
	double minx,maxx,miny,maxy;


public:

	double RTs[10][12];
	double rt2_original_v[12];
	double rt2_original_v1[12];
	vector <vector <VC_Point>> VC_Points;


	myData();

	void InitRTs();
	void InitVC_Points();
	void computeTrueRTs();
	void computeCenter(double a[3]);
	void computVC_Score(double mid_x,double mid_y,double mid_z);

	void Generate_DepthP();

	double scoreFunction(double color[10][3]);

	void Filter();
	void Filter2();


	void C_C();
};