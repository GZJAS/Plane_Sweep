#include "CoordinateChange.h"


void Print_Mat(CvMat *mat)
{
	int i,j;
	for(i=0;i<mat->rows;i++)
	{
		printf("\n");
		for(j=0;j<mat->cols;j++)
			//if(CV_MAT_DEPTH(A->type)==CV_64F)
			printf("%6.2lf",cvGetReal2D(mat,i,j));   

	}
	printf("\n");
}


CvMat *change(CvMat *original,int rti)
{
	CvMat *mat_k,*mat_k_inv,*mat_rt,*mat_inv_e;
	mat_rt = cvCreateMat(3,4,CV_64FC1);
	mat_inv_e = cvCreateMat(4,4,CV_64FC1);

	double _e[] = {-1,0,0,0,
		0,-1,0,0,
		0,0,-1,0,
		0,0,0,-1};	
	cvInitMatHeader(mat_inv_e,4,4,CV_64FC1,_e);

	mat_k =  cvCreateMat( 3, 3, CV_64FC1);

	double _k[] = {-800,0,599.5,0,800,399.5,0,0,1};
	cvInitMatHeader( mat_k, 3, 3, CV_64FC1, _k);



	mat_k_inv = cvCreateMat( 3, 3, CV_64FC1);
	cvInvert(mat_k, mat_k_inv, CV_SVD);




	cvmSet(mat_k,0,0,800);



	cvMatMulAdd( mat_k_inv, original, 0, mat_rt);

	Print_Mat(mat_rt);


	cvMatMulAdd( mat_rt, mat_inv_e, 0, mat_rt);


	Print_Mat(mat_rt);





	//if (rti == 1)
	//{
	//	CvMat* ad, *mul ,*e ,*inv_rt ,*rt_change , *rt_e;
	//	ad = cvCreateMat(3,4,CV_64FC1);
	//	mul = cvCreateMat(4,4,CV_64FC1);
	//	e = cvCreateMat(4,4,CV_64FC1);
	//	inv_rt = cvCreateMat(4,3,CV_64FC1);
	//	rt_change = cvCreateMat(4,4,CV_64FC1);
	//	rt_e = cvCreateMat(3,4,CV_64FC1);

	//	double _add[] = {0,0,0,0.12,
	//		0,0,0,-0.43,
	//		0,0,0,-0.17};
	//	cvInitMatHeader(ad,3,4,CV_64FC1,_add);

	//	double _rt_e[] = {1,0,0,0,
	//	                  0,1,0,0,
	//	                  0,0,1,0};

	//	cvInitMatHeader(rt_e,3,4,CV_64FC1,_rt_e);


	//	double _e[] ={1,0,0,0,
	//	              0,1,0,0,
	//	              0,0,1,0,
	//	              0,0,0,1};

	//	cvInitMatHeader(e,4,4,CV_64FC1,_e);

	//	cvMatMulAdd(mat_rt,e,ad,mat_rt);

	//	cout<<"mat_rt:";

	//	Print_Mat(mat_rt);


	//	cvInvert(mat_rt, inv_rt, CV_SVD);

	//	cout<<"inv_rt:";

	//	Print_Mat(inv_rt);


	//	cvMatMulAdd(inv_rt,rt_e,0,rt_change);

	//	cout<<"rt_change:";
	//	Print_Mat(rt_change);

	//    CvMat *test;
	//	test = cvCreateMat(3,4,CV_64FC1);

	//	cvMatMulAdd(mat_rt,rt_change,0,test);

	//	cout<<"test:";
	//	Print_Mat(test);


	//}
	//CvMat *rt_change,*rt_add,*rt_e;
	//rt_change = cvCreateMat(4,4,CV_64FC1);
	//rt_add = cvCreateMat(3,4,CV_64FC1);
	//rt_e = cvCreateMat(4,4,CV_64FC1);

	//double _change[] = {1.64,-0.24,-0.52,0,
	//                    0.27,1.59,-0.02,0,
	//                    -0.1,0.42,1.08,0,
	//                    0,-0.01,0,0};

	//cvInitMatHeader(rt_change,4,4,CV_64FC1,_change);

	//double _add[] = {0,0,0,0.12,
	//			     0,0,0,-0.43,
	//			     0,0,0,-0.17};
	//cvInitMatHeader(rt_add,3,4,CV_64FC1,_add);

	//double _rt_e[] ={1,0,0,0,
	//	0,1,0,0,
	//	0,0,1,0,
	//	0,0,0,1};

	//cvInitMatHeader(rt_e,4,4,CV_64FC1,_rt_e);

	//cvMatMulAdd(mat_rt,rt_e,rt_add,mat_rt);

	//cvMatMulAdd(mat_rt,rt_change,0,mat_rt);

	//Print_Mat(mat_rt);

	cvMatMulAdd(mat_k,mat_rt,0,original);


	return original;

}