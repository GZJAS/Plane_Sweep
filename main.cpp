#include <opencv2\opencv.hpp>
#include <iostream>
#include <string>
#include <cstdlib>
#include "CoordinateChange.h"
#include "myData.h"
#include <fstream>
#include <sstream>
using namespace cv;
using namespace std;


int main()
{
	myData data;
	data.InitRTs();
	data.InitVC_Points();
	data.computeTrueRTs();

	fstream in;
	in.open("2.txt",ios::out);
	string strLine;
	double x,y,z;
	//int line_num=1;
	//while (getline(in,strLine))
	//{
	//	istringstream is(strLine);
	//	is >> x >> y >> z;
	//	if (x<1.5 && x >-1.5 && y<1.5 && y>-1.5 && z<-2.6 && z>-3.5)
	//	{
	//		data.computVC_Score(x,y,z);
	//		cout<<line_num<<endl;
	//		line_num++;
	//	}
	//}
	data.computVC_Score(0,0,-3.8);



	//data.computeTrueRTs();


	//data.computVC_Score(-4,-1);

	int point_num=0;

	for (int i = 0 ; i<data.VC_Points.size() ; i++)
	{
		for (int j = 0 ; j<data.VC_Points[i].size(); j++)
			if (data.VC_Points[i][j].Score != -1)
			{
				point_num++;
			}
	}

	cout<<"point_num = "<<point_num<<endl;


	for (int i = 0 ; i<data.VC_Points.size() ; i++)
	{
		for (int j = 0 ; j<data.VC_Points[i].size(); j++)
			if (data.VC_Points[i][j].Score != -1)
			{
				in<<data.VC_Points[i][j].x<<" "<<data.VC_Points[i][j].y<<" "<<data.VC_Points[i][j].z<<" "<<data.VC_Points[i][j].R<<" "<<data.VC_Points[i][j].G<<" "<<data.VC_Points[i][j].B<<endl;
			}
	}
	in.close();


	data.Filter();
	data.Filter2();
	//data.Filter();

	data.C_C();

	point_num = 0;

	for (int i = 0 ; i<data.VC_Points.size() ; i++)
	{
		for (int j = 0 ; j<data.VC_Points[i].size(); j++)
			if (data.VC_Points[i][j].Score != -1)
			{
				point_num++;
			}
	}

	cout<<"point_num = "<<point_num<<endl;



	fstream f;
	f.open("1.txt",ios::out);
	for (int i = 0 ; i<data.VC_Points.size() ; i++)
	{
		for (int j = 0 ; j<data.VC_Points[i].size(); j++)
			if (data.VC_Points[i][j].Score != -1)
		{
			f<<data.VC_Points[i][j].x<<" "<<data.VC_Points[i][j].y<<" "<<data.VC_Points[i][j].z<<" "<<data.VC_Points[i][j].R<<" "<<data.VC_Points[i][j].G<<" "<<data.VC_Points[i][j].B<<endl;
		}
	}
	f.close();

	system("pause");
	return 0;
}