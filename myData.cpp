#include "myData.h"
#include <fstream>
#include <string>
#include <cmath>
#include "CoordinateChange.h"

using namespace cv;




myData::myData()
{
	minx = -2.0;
	maxx =2.0;
	miny = -2.0;
	maxy = 2.0;
}

void PrintMat(CvMat * A)
{
	int i,j;
	for(i=0;i<A->rows;i++)
	{
		printf("\n");
		for(j=0;j<A->cols;j++)
			//if(CV_MAT_DEPTH(A->type)==CV_64F)
			printf("%6.2lf",cvGetReal2D(A,i,j));   

	}
}


void myData::InitRTs()
{
	fstream read;
	int i ;
	stringstream ss;
	string i_str;
	string path;
	string strLine;

	for (i=1 ; i<=10 ; i++)
	{
		ss<<i;
		ss>>i_str;
		ss.clear();
		ss.str("");
		if (i<10)
		{
			path = "F:/三维重建/kermit/data8/0000"+ i_str + "_P.txt";

		}
		else if (i<100)
		{
			path = "F:/三维重建/kermit/data8/000"+ i_str + "_P.txt";

		}
		//else
		//{
		//	path = "F:/三维重建/data8/data/00"+ i_str + "_P.txt";

		//}
		read.open(path);
		getline(read,strLine);
		int n = 0;
		while(getline(read,strLine))
		{
			istringstream is(strLine);
			is >> RTs[i-1][n] >> RTs[i-1][n+1] >> RTs[i-1][n+2] >> RTs[i-1][n+3];
			n += 4;
		}

		read.close();

		for (int i=0;i<12;i++)
		{
			rt2_original_v[i]=RTs[2][i];
			rt2_original_v1[i]=RTs[2][i];
		}
	}
}

void myData::InitVC_Points()
{
	int VC_size = 0;
	VC_size = (maxy-miny)*(1/0.01);
	VC_Points.resize(VC_size);
	for (int i=0;i<VC_size;i++)
	{
		VC_Points[i].resize(VC_size);
	}
	for (int x=0 ; x<VC_size ; x++)
	{
		for (int y=0 ; y<VC_size ; y++)
		{
			VC_Point p;
			p.B = 0;
			p.G = 0;
			p.R = 0;
			p.Score = -1;
			p.z = 0;
			p.x = 0;
			p.y = 0;
			for (int i=0;i<11;i++)
			{
				p.cam[i]=0;
			}
			VC_Points[x][y]=p;
		}
	}

}

void myData::computeTrueRTs()
{
	CvMat *rt2 = cvCreateMat( 3, 4, CV_64FC1);
	CvMat *rt2_original = cvCreateMat( 3, 4, CV_64FC1);
	double b[] ={-480, 0 ,319.5,
		0 ,480,239.5,
		0 , 0 , 1};


	cvInitMatHeader(rt2_original,3,4,CV_64FC1,rt2_original_v);

	CvMat *k = cvCreateMat( 3, 3, CV_64FC1);
	CvMat *k_inv = cvCreateMat( 3, 3, CV_64FC1);

	cvInitMatHeader(k,3,3,CV_64FC1,b);


	cvInvert(k, k_inv, CV_LU);

	cvMatMulAdd( k_inv, rt2_original, 0, rt2_original);

	CvMat *fe = cvCreateMat( 3, 3, CV_64FC1);
	double fe_value[]={-1,0,0,
		0,-1,0,
		0,0,-1};
	cvInitMatHeader(fe,3,3,CV_64FC1,fe_value);


	cvMatMulAdd(fe,rt2_original,0,rt2_original);


	double x,y,z;
	x=CV_MAT_ELEM(*rt2_original,double,0,3);
	y=CV_MAT_ELEM(*rt2_original,double,1,3);
	z=CV_MAT_ELEM(*rt2_original,double,2,3);

	double t_value[]={-x,
		-y,
		-z};

	CvMat *t = cvCreateMat( 3, 1, CV_64FC1);
	cvInitMatHeader(t,3,1,CV_64FC1,t_value);



	CvMat *r = cvCreateMat(3,3,CV_64FC1);
	double r_value[]={CV_MAT_ELEM(*rt2_original,double,0,0),CV_MAT_ELEM(*rt2_original,double,0,1),CV_MAT_ELEM(*rt2_original,double,0,2),
		CV_MAT_ELEM(*rt2_original,double,1,0),CV_MAT_ELEM(*rt2_original,double,1,1),CV_MAT_ELEM(*rt2_original,double,1,2),
		CV_MAT_ELEM(*rt2_original,double,2,0),CV_MAT_ELEM(*rt2_original,double,2,1),CV_MAT_ELEM(*rt2_original,double,2,2)};
	cvInitMatHeader(r,3,3,CV_64FC1,r_value);

	CvMat *r_inv = cvCreateMat(3,3,CV_64FC1);
	cvInvert(r, r_inv, CV_LU);

	cvMatMulAdd(r_inv,t,0,t);

	CvMat *change = cvCreateMat(4,4,CV_64FC1);
	double change_value[]=
	{CV_MAT_ELEM(*r_inv,double,0,0),CV_MAT_ELEM(*r_inv,double,0,1),CV_MAT_ELEM(*r_inv,double,0,2),CV_MAT_ELEM(*t,double,0,0),
	CV_MAT_ELEM(*r_inv,double,1,0),CV_MAT_ELEM(*r_inv,double,1,1),CV_MAT_ELEM(*r_inv,double,1,2),CV_MAT_ELEM(*t,double,1,0),
	CV_MAT_ELEM(*r_inv,double,2,0),CV_MAT_ELEM(*r_inv,double,2,1),CV_MAT_ELEM(*r_inv,double,2,2),CV_MAT_ELEM(*t,double,2,0),
	0        ,               0              ,                0             ,                  1         
	};
	cvInitMatHeader(change,4,4,CV_64FC1,change_value);



	for (int i=0 ; i<10 ; i++)
	{

		CvMat *rt1 = cvCreateMat( 3, 4, CV_64FC1);
		cvInitMatHeader(rt1,3,4,CV_64FC1,RTs[i]);

		cvMatMulAdd( rt1, change, 0, rt1);



	}




}

void myData::computVC_Score(double mid_x,double mid_y,double mid_z)
{
	Mat img0;
	Mat img1;
	Mat img2;
	Mat img3;
	Mat img4;
	Mat img5;
	Mat img6;
	Mat img7;
	Mat img8;
	Mat img9;

	img0 = imread("F:/三维重建/kermit/data8/00001.jpg");
	img1 = imread("F:/三维重建/kermit/data8/00002.jpg");
	img2 = imread("F:/三维重建/kermit/data8/00003.jpg");
	img3 = imread("F:/三维重建/kermit/data8/00004.jpg");
	img4 = imread("F:/三维重建/kermit/data8/00005.jpg");
	img5 = imread("F:/三维重建/kermit/data8/00006.jpg");
	img6 = imread("F:/三维重建/kermit/data8/00007.jpg");
	img7 = imread("F:/三维重建/kermit/data8/00008.jpg");
	img8 = imread("F:/三维重建/kermit/data8/00009.jpg");
	img9 = imread("F:/三维重建/kermit/data8/00010.jpg");



	int min_i = (mid_x-2.0-minx)/0.01;
	int min_j = (mid_y-2.0-miny)/0.01;

	int max_i = (mid_x+2.0-minx)/0.01;
	int max_j = (mid_y+2.0-miny)/0.01;

	if (min_i<0)
	{
		min_i = 0;
	}
	if (min_j<0)
	{
		min_j = 0;
	}
	if (max_i>=VC_Points.size())
	{
		max_i = VC_Points.size()-1;
	}
	if (max_j>=VC_Points.size())
	{
		max_j = VC_Points.size()-1;
	}



	for (double Plane_z=mid_z-1;Plane_z<=mid_z+1;Plane_z+=0.01)
	{

		double now_color[10][3];//存储当前点投影在每个图片上后的颜色


		stringstream ss;


		CvMat *rt = cvCreateMat( 3, 4, CV_64FC1);
		CvMat *three_d_point = cvCreateMat( 4, 1, CV_64FC1);
		CvMat *two_d_point = cvCreateMat(3,1,CV_64FC1);



		for (int now_i = min_i ; now_i<=max_i ; now_i++)
		{
			for (int now_j=min_j ; now_j<=max_j ; now_j++)
			{
				double now_x = minx+now_i*0.01;
				double now_y = miny+now_j*0.01;
				for (int ii = 0 ; ii<10 ; ii++)
				{
					for ( int jj = 0; jj<3 ;jj++)
					{
						now_color[ii][jj] = -1;
					}
				}

				double b[] = {now_x, now_y,Plane_z,1};
				//CvMat *three_d_point = cvCreateMat( 4, 1, CV_64FC1);
				cvInitMatHeader( three_d_point, 4, 1, CV_64FC1, b);
				//PrintMat(three_d_point);


				for (int i=1 ; i<=10 ; i++)
				{
					//CvMat *rt = cvCreateMat( 3, 4, CV_64FC1);

					cvInitMatHeader(rt,3,4,CV_64FC1,RTs[i-1]);
					//PrintMat(rt);

					double b[] = {now_x, now_y,Plane_z,1};
					//CvMat *three_d_point = cvCreateMat( 4, 1, CV_64FC1);
					cvInitMatHeader( three_d_point, 4, 1, CV_64FC1, b);
					//PrintMat(three_d_point);

					//CvMat *two_d_point = cvCreateMat(3,1,CV_64FC1);

					cvMatMulAdd( rt, three_d_point, 0, two_d_point);
					double x,y,z;
					z = CV_MAT_ELEM(* two_d_point,double,2,0);
					x = CV_MAT_ELEM(* two_d_point,double,0,0);
					y = CV_MAT_ELEM(* two_d_point,double,1,0);
					//PrintMat(two_d_point);


					x = x/z+0.5;
					y = y/z+0.5  ;
					z = 1;

					if (x<640 && x>=0 && y<480 && y>=0)
					{

						int r_color,g_color,b_color;
						if (i == 1)
						{
							r_color = img0.at<Vec3b>(y,x)[2];
							g_color = img0.at<Vec3b>(y,x)[1];
							b_color = img0.at<Vec3b>(y,x)[0];
						}
						else if (i == 2)
						{
							r_color = img1.at<Vec3b>(y,x)[2];
							g_color = img1.at<Vec3b>(y,x)[1];
							b_color = img1.at<Vec3b>(y,x)[0];
						}
						else if(i==3)
						{
							r_color = img2.at<Vec3b>(y,x)[2];
							g_color = img2.at<Vec3b>(y,x)[1];
							b_color = img2.at<Vec3b>(y,x)[0];
						}
						else if(i==4)
						{
							r_color = img3.at<Vec3b>(y,x)[2];
							g_color = img3.at<Vec3b>(y,x)[1];
							b_color = img3.at<Vec3b>(y,x)[0];
						}
						else if(i==5)
						{
							r_color = img4.at<Vec3b>(y,x)[2];
							g_color = img4.at<Vec3b>(y,x)[1];
							b_color = img4.at<Vec3b>(y,x)[0];
						}
						else if(i==6)
						{
							r_color = img5.at<Vec3b>(y,x)[2];
							g_color = img5.at<Vec3b>(y,x)[1];
							b_color = img5.at<Vec3b>(y,x)[0];
						}
						else if(i==7)
						{
							r_color = img6.at<Vec3b>(y,x)[2];
							g_color = img6.at<Vec3b>(y,x)[1];
							b_color = img6.at<Vec3b>(y,x)[0];
						}
						else if(i==8)
						{
							r_color = img7.at<Vec3b>(y,x)[2];
							g_color = img7.at<Vec3b>(y,x)[1];
							b_color = img7.at<Vec3b>(y,x)[0];
						}
						else if(i==9)
						{
							r_color = img8.at<Vec3b>(y,x)[2];
							g_color = img8.at<Vec3b>(y,x)[1];
							b_color = img8.at<Vec3b>(y,x)[0];
						}
						else if(i==10)
						{
							r_color = img9.at<Vec3b>(y,x)[2];
							g_color = img9.at<Vec3b>(y,x)[1];
							b_color = img9.at<Vec3b>(y,x)[0];
						}
						now_color[i-1][0]=r_color;
						now_color[i-1][1]=g_color;
						now_color[i-1][2]=b_color;


					}
				}
				double ave_r=0;
				double ave_g=0;
				double ave_b=0;
				double score=0;
				int color_num=0;
				for (int iii = 0 ; iii<10 ; iii++)
				{
					if (now_color[iii][0]!=-1)
					{
						color_num++;
						ave_r = ave_r+now_color[iii][0];
						ave_g = ave_g+now_color[iii][1];
						ave_b = ave_b+now_color[iii][2];
					}
				}
				if (color_num > 3)
				{
					ave_r = ave_r/color_num;
					ave_g = ave_g/color_num;
					ave_b = ave_b/color_num;


					for (int iii = 0 ; iii<10 ; iii++)
					{
						if (now_color[iii][0]!=-1)
						{
							score = score+pow(now_color[iii][0]-ave_r,2)+pow(now_color[iii][1]-ave_g,2)+pow(now_color[iii][2]-ave_b,2);
						}
					}
					score = score/color_num;
					if ((VC_Points[now_i][now_j].Score==-1 || score < VC_Points[now_i][now_j].Score)&&color_num>2)
					{
						VC_Points[now_i][now_j].R = ave_r;
						VC_Points[now_i][now_j].G = ave_g;
						VC_Points[now_i][now_j].B = ave_b;
						VC_Points[now_i][now_j].Score = score;
						VC_Points[now_i][now_j].z = Plane_z;
						VC_Points[now_i][now_j].x = now_x;
						VC_Points[now_i][now_j].y = now_y;
						//for (int iiii=0;iiii<11;iiii++)
						//{
						//	if (now_color[iiii][0]!=-1)
						//	{
						//		VC_Points[now_i][now_j].cam[iiii]=1;
						//	}
						//	else VC_Points[now_i][now_j].cam[iiii]=0;
						//}

					}

				}

			}
		}

		cout<<Plane_z<<endl;
	}




}

double myData::scoreFunction(double color[10][3])
{
	double score;
	int color_num;
	double ave_r,ave_g,ave_b;
	ave_b=ave_g=ave_r=0.0;
	for (int iii = 0 ; iii<10 ; iii++)
	{
		if (color[iii][0]!=-1)
		{
			color_num++;
			ave_r = ave_r+color[iii][0];
			ave_g = ave_g+color[iii][1];
			ave_b = ave_b+color[iii][2];
		}
	}
	ave_r = ave_r/color_num;
	ave_g = ave_g/color_num;
	ave_b = ave_b/color_num;
	for (int iii = 0 ; iii<10 ; iii++)
	{
		if (color[iii][0]!=-1)
		{
			score = score+pow(color[iii][0]-ave_r,2)+pow(color[iii][1]-ave_g,2)+pow(color[iii][2]-ave_b,2);
		}
	}
	score = score/color_num;
	return score;
}


void myData ::Filter()
{
	double threshold = 0.15;

 	for (int x=1 ; x<VC_Points.size()-1 ; x++)
	{
		for (int y=1 ; y<VC_Points.size()-1 ; y++)
		{
			if (VC_Points[x][y].Score!=-1)
			{
				vector<double> z;
				double total_z = 0;
				for (int i=-1;i<2;i++)
				{
					for (int j=-1;j<2;j++)
					{
						if (i!=0&&j!=0)
						{
							if (VC_Points[x+i][y+j].Score!=-1)
							{
								z.push_back(VC_Points[x+i][y+j].z);
								total_z = total_z+VC_Points[x+i][y+j].z;
							}
						}
					}
				}

				if (z.size()>0)
				{
					double mean;
					double median;

					mean = total_z /z.size();
					for (int i=0 ; i<z.size()-1 ; i++)
					{
						for (int j=0 ; j<z.size()-i-1 ; j++)
						{
							if (z[j]>z[j+1])
							{
								double temp;
								temp = z[j];
								z[j]=z[j+1];
								z[j+1]=temp;
							}
						}
					}
					if (z.size()%2 == 0)
						median = (z[z.size()/2-1]+z[z.size()/2])/2;
					else
						median = z[z.size()/2];



					if (fabs(VC_Points[x][y].z-mean)>threshold || fabs(VC_Points[x][y].z-median)>threshold)
					{
						VC_Points[x][y].Score = -1;
					}

				}

			}
		}
	}

	for (int i=0;i<VC_Points.size();i++)
	{
		VC_Points[0][i].Score=-1;
		VC_Points[VC_Points.size()-1][i].Score=-1;
		VC_Points[i][0].Score=-1;
		VC_Points[i][VC_Points.size()-1].Score=-1;
	}
}



void myData::Filter2()
{

	Mat img0;
	Mat img1;
	Mat img2;
	Mat img3;
	Mat img4;
	Mat img5;
	Mat img6;
	Mat img7;
	Mat img8;
	Mat img9;

	img0 = imread("F:/三维重建/kermit/data8/00001.jpg");
	img1 = imread("F:/三维重建/kermit/data8/00002.jpg");
	img2 = imread("F:/三维重建/kermit/data8/00003.jpg");
	img3 = imread("F:/三维重建/kermit/data8/00004.jpg");
	img4 = imread("F:/三维重建/kermit/data8/00005.jpg");
	img5 = imread("F:/三维重建/kermit/data8/00006.jpg");
	img6 = imread("F:/三维重建/kermit/data8/00007.jpg");
	img7 = imread("F:/三维重建/kermit/data8/00008.jpg");
	img8 = imread("F:/三维重建/kermit/data8/00009.jpg");
	img9 = imread("F:/三维重建/kermit/data8/00010.jpg");



	CvMat *rt = cvCreateMat( 3, 4, CV_64FC1);
	CvMat *three_d_point = cvCreateMat( 4, 1, CV_64FC1);
	CvMat *two_d_point = cvCreateMat(3,1,CV_64FC1);

	Mat rec0(5,5,CV_8U);
	Mat rec1(5,5,CV_8U);
	Mat rec2(5,5,CV_8U);
	Mat rec3(5,5,CV_8U);
	Mat rec4(5,5,CV_8U);
	Mat rec5(5,5,CV_8U);
	Mat rec6(5,5,CV_8U);
	Mat rec7(5,5,CV_8U);
	Mat rec8(5,5,CV_8U);
	Mat rec9(5,5,CV_8U);


	for (int x=1 ; x<VC_Points.size()-1 ; x++)
	{
		for (int y=1 ; y<VC_Points.size()-1 ; y++)
		{
			if (VC_Points[x][y].Score!=-1)
			{
				double score[9];
				double score_avg=0;
				for (int i=0 ; i<9 ; i++)
				{
					score[i]=0;
				}
				int alpha,beta;


				cvInitMatHeader(rt,3,4,CV_64FC1,RTs[2]);
				double b[] = {VC_Points[x][y].x, VC_Points[x][y].y,VC_Points[x][y].z,1};
				cvInitMatHeader( three_d_point, 4, 1, CV_64FC1, b);

				cvMatMulAdd( rt, three_d_point, 0, two_d_point);
				double x2d,y2d,z2d;
				z2d = CV_MAT_ELEM(* two_d_point,double,2,0);
				x2d = CV_MAT_ELEM(* two_d_point,double,0,0);
				y2d = CV_MAT_ELEM(* two_d_point,double,1,0);
				//PrintMat(two_d_point);


				x2d = x2d/z2d;
				y2d = y2d/z2d;
				z2d = 1;

				if (x2d<640 && x2d>0 && y2d<480 && y2d>0)
				{
					Point2f p;
					p.x = x2d;
					p.y = y2d;

					getRectSubPix( img2, Size(5,5), p,rec2 );


					for (int i=1 ; i<=10; i++)
					{
						//CvMat *rt = cvCreateMat( 3, 4, CV_64FC1);
						if (i!=3)
						{
							cvInitMatHeader(rt,3,4,CV_64FC1,RTs[i-1]);


							cvMatMulAdd( rt, three_d_point, 0, two_d_point);
							double x2d,y2d,z2d;
							z2d = CV_MAT_ELEM(* two_d_point,double,2,0);
							x2d = CV_MAT_ELEM(* two_d_point,double,0,0);
							y2d = CV_MAT_ELEM(* two_d_point,double,1,0);
							//PrintMat(two_d_point);


							x2d = x2d/z2d;
							y2d = y2d/z2d;
							z2d = 1;

							Point2f p;
							p.x = x2d;
							p.y = y2d;

							if (x2d<640 && x2d>0 && y2d<480 && y2d>0)
							{
								switch(i)
								{
								case 1:
									getRectSubPix( img0, Size(5,5), p,rec0 );
									score[0]=-1;
									break;
								case 2:
									getRectSubPix( img1, Size(5,5), p,rec1 );
									score[1]=-1;
									break;
								case 4:
									getRectSubPix( img3, Size(5,5), p,rec3 );
									score[2]=-1;
									break;
								case 5:
									getRectSubPix( img4, Size(5,5), p,rec4 );
									score[3]=-1;
									break;
								case 6:
									getRectSubPix( img5, Size(5,5), p,rec5 );
									score[4]=-1;
									break;
								case 7:
									getRectSubPix( img6, Size(5,5), p,rec6 );
									score[5]=-1;
									break;
								case 8:
									getRectSubPix( img7, Size(5,5), p,rec7 );
									score[6]=-1;
									break;
								case 9:
									getRectSubPix( img8, Size(5,5), p,rec8 );
									score[7]=-1;
									break;
								case 10:
									getRectSubPix( img9, Size(5,5), p,rec9 );
									score[8]=-1;
									break;
								default:
									break;
								}

							}
						}

					}

					for(int i=0 ; i<9 ; i++)
					{
						Mat res(1,1,CV_32F);
						switch(i)
						{
						case 0:
							if (score[0]==-1)
							{
								matchTemplate( rec2,rec0, res, CV_TM_CCOEFF_NORMED);
								score[0] = ((float *)(res.data))[0];
							}
							break;
						case 1:
							if (score[1]==-1)
							{
								matchTemplate( rec2,rec1, res, CV_TM_CCOEFF_NORMED);
								score[1] = ((float *)(res.data))[0];
							}
							break;
						case 2:
							if (score[2]==-1)
							{
								matchTemplate( rec2,rec3, res, CV_TM_CCOEFF_NORMED);
								score[2] = ((float *)(res.data))[0];
							}
							break;
						case 3:
							if (score[3]==-1)
							{
								matchTemplate( rec2,rec4, res, CV_TM_CCOEFF_NORMED);
								score[3] = ((float *)(res.data))[0];
							}
							break;
						case 4:
							if (score[4]==-1)
							{
								matchTemplate( rec2,rec5, res, CV_TM_CCOEFF_NORMED);
								score[4] = ((float *)(res.data))[0];
							}
							break;
						case 5:
							if (score[5]==-1)
							{
								matchTemplate( rec2,rec6, res, CV_TM_CCOEFF_NORMED);
								score[5] = ((float *)(res.data))[0];
							}
							break;
						case 6:
							if (score[6]==-1)
							{
								matchTemplate( rec2,rec7, res, CV_TM_CCOEFF_NORMED);
								score[6] = ((float *)(res.data))[0];
							}
							break;
						case 7:
							if (score[7]==-1)
							{
								matchTemplate( rec2,rec8, res, CV_TM_CCOEFF_NORMED);
								score[7] = ((float *)(res.data))[0];
							}
							break;
						case 8:
							if (score[8]==-1)
							{
								matchTemplate( rec2,rec9, res, CV_TM_CCOEFF_NORMED);
								score[8] = ((float *)(res.data))[0];
							}
							break;
						default:
							break;
						}
					}



					alpha = beta = 0;
					for (int i=0 ; i<9 ; i++)
					{
						if (score[i] >= 0.3)
						{
							alpha++;
							score_avg = score_avg+score[i];
						}
						if (score[i] >= 0.7)
						{
							beta++;
						}
					}

					if (alpha>0)
					{
						score_avg = score_avg/alpha;
					}

				}
				if (beta<2 || score_avg<0.5)
				{
					VC_Points[x][y].Score=-1;
				}
			}
		}
	}


}

void myData::C_C()
{
	CvMat *rt2 = cvCreateMat( 3, 4, CV_64FC1);
	CvMat *rt2_original = cvCreateMat( 3, 4, CV_64FC1);
	double b[] ={-480, 0 ,319.5,
		0 ,480,239.5,
		0 , 0 , 1};


	cvInitMatHeader(rt2_original,3,4,CV_64FC1,rt2_original_v1);

	CvMat *k = cvCreateMat( 3, 3, CV_64FC1);
	CvMat *k_inv = cvCreateMat( 3, 3, CV_64FC1);

	cvInitMatHeader(k,3,3,CV_64FC1,b);


	cvInvert(k, k_inv, CV_LU);

	cvMatMulAdd( k_inv, rt2_original, 0, rt2_original);

	CvMat *fe = cvCreateMat( 3, 3, CV_64FC1);
	double fe_value[]={-1,0,0,
		0,-1,0,
		0,0,-1};
	cvInitMatHeader(fe,3,3,CV_64FC1,fe_value);


	cvMatMulAdd(fe,rt2_original,0,rt2_original);


	double x,y,z;
	x=CV_MAT_ELEM(*rt2_original,double,0,3);
	y=CV_MAT_ELEM(*rt2_original,double,1,3);
	z=CV_MAT_ELEM(*rt2_original,double,2,3);

	double t_value[]={-x,
		-y,
		-z};

	CvMat *t = cvCreateMat( 3, 1, CV_64FC1);
	cvInitMatHeader(t,3,1,CV_64FC1,t_value);



	CvMat *r = cvCreateMat(3,3,CV_64FC1);
	double r_value[]={CV_MAT_ELEM(*rt2_original,double,0,0),CV_MAT_ELEM(*rt2_original,double,0,1),CV_MAT_ELEM(*rt2_original,double,0,2),
		CV_MAT_ELEM(*rt2_original,double,1,0),CV_MAT_ELEM(*rt2_original,double,1,1),CV_MAT_ELEM(*rt2_original,double,1,2),
		CV_MAT_ELEM(*rt2_original,double,2,0),CV_MAT_ELEM(*rt2_original,double,2,1),CV_MAT_ELEM(*rt2_original,double,2,2)};
	cvInitMatHeader(r,3,3,CV_64FC1,r_value);

	CvMat *r_inv = cvCreateMat(3,3,CV_64FC1);
	cvInvert(r, r_inv, CV_LU);

	cvMatMulAdd(r_inv,t,0,t);

	CvMat *three_dp_real = cvCreateMat(3,1,CV_64FC1);
	CvMat *three_dp_ref = cvCreateMat(3,1,CV_64FC1);

	for (int i=0;i<VC_Points.size();i++)
	{
		for (int j=0;j<VC_Points.size();j++)
		{
			if (VC_Points[i][j].Score!=-1)
			{
				double real_value[]={VC_Points[i][j].x,VC_Points[i][j].y,VC_Points[i][j].z};

				cvInitMatHeader(three_dp_real,3,1,CV_64FC1,real_value);

				cvMatMulAdd(r_inv,three_dp_real,0,three_dp_real);
				cvAdd(three_dp_real,t,three_dp_ref);

				VC_Points[i][j].x=CV_MAT_ELEM(*three_dp_ref,double,0,0);
				VC_Points[i][j].y=CV_MAT_ELEM(*three_dp_ref,double,1,0);
				VC_Points[i][j].z=CV_MAT_ELEM(*three_dp_ref,double,2,0);
			}
		}
	}
}