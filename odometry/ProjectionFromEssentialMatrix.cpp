/**
 * This file implements projection computation from Essential matrix
 * and takes care of scaling by getting the affine transformation of
 * projected 3d points.	*/

#include <vector>
#include <iostream>
#include <string>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"

Mat Kmat;
Mat_<double> LinearLSTriangulation(
Point3d u,//homogenous image point (u,v,1)
Matx34d P,//camera 1 matrix
Point3d u1,//homogenous image point in 2nd camera
Matx34d P1//camera 2 matrix
)
{
	//build A matrix
	Matx43d A(u.x*P(2,0)-P(0,0),u.x*P(2,1)-P(0,1),u.x*P(2,2)-P(0,2),
			  u.y*P(2,0)-P(1,0),u.y*P(2,1)-P(1,1),u.y*P(2,2)-P(1,2),
			  u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),u1.x*P1(2,2)-P1(0,2),
			  u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),u1.y*P1(2,2)-P1(1,2)
	);
	//build B vector
	Matx41d B(-(u.x*P(2,3)-P(0,3)),
			  -(u.y*P(2,3)-P(1,3)),
			  -(u1.x*P1(2,3)-P1(0,3)),
			  -(u1.y*P1(2,3)-P1(1,3)));

	//solve for X
	Mat_<double> X;
	Mat Amat(A);Mat Bmat(B);
	solve(Amat,Bmat,X,DECOMP_SVD);

	return X;
}

double TriangulatePoints(
const vector<Point2d>& pt_set1,
const vector<Point2d>& pt_set2,
const Mat&Kinv,
const Matx34d& P,
const Matx34d& P1,
vector<Point3d>& pointcloud)
{
	vector<double> reproj_error;
	int pts_size = pt_set1.size();
	for (unsigned int i=0; i<pts_size; i++) {
		//convert to normalized homogeneous coordinates
		Point2f kp = pt_set1[i];
		Point3d u(kp.x,kp.y,1.0);
		Mat_<double> um = Kinv * Mat_<double>(u);
		u = um.at<Point3d>(0);
		Point2f kp1 = pt_set2[i];
		Point3d u1(kp1.x,kp1.y,1.0);
		Mat_<double> um1 = Kinv * Mat_<double>(u1);
		u1 = um1.at<Point3d>(0);

		//triangulate
		Mat_<double> X = LinearLSTriangulation(u,P,u1,P1);

		//calculate reprojection error
		Mat Pmat = Mat(P1);
		cerr<<Kmat.cols<<" "<<Kmat.rows<<"\n";
		cerr<<X.cols<<" "<<X.rows<<"\n";
		cerr<<Kmat<<" "<<X<<" "<<Pmat<<"\n";
		Mat<double>
		Mat_<double> xPt_img = Kmat * Mat(P1) * X;
		Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2));
		reproj_error.push_back(norm(xPt_img_-kp1));

		//store 3D point
		pointcloud.push_back(Point3d(X(0),X(1),X(2)));
	}

	//return mean reprojection error
	Scalar me = mean(reproj_error);
	return me[0];
}

bool CheckCoherentRotation(cv::Mat_<double>& R) {
	if(fabsf(determinant(R))-1.0 > 1e-07) {
		cerr<<"det(R) != +-1.0, this is not a rotation matrix"<<endl;
		return false;
    }
	return true;
}

Matx34d getCameraMotion(vector<Point2f> points[2], vector<uchar> status, Matx33d K, vector<Point3d> prevPoints3d){
  vector<Point2f>imgpts1, imgpts2;
  for( unsigned int i = 0; i<status.size(); i++ )
    {
      if(status[i]){
	// queryIdx is the "left" image
	imgpts1.push_back(points[0][i]);
	// trainIdx is the "right" image
	imgpts2.push_back(points[1][i]);
      }
    }
  vector<uchar> statusInlier(imgpts1.size());
  Mat F;
  F = findFundamentalMat(imgpts1, imgpts2,FM_RANSAC, 0.1, 0.99,statusInlier);
  Kmat = Mat(K);
  Mat_<double> E = Kmat.t() * F * Kmat; //according to HZ (9.12)

  SVD svd(E);
  Matx33d W(0,-1,0,//HZ 9.13
	        1,0,0,
	        0,0,1);
  Mat_<double> R = svd.u * Mat(W) * svd.vt; //HZ 9.19
  Mat_<double> t = svd.u.col(2); //u3
  Matx34d P1(  R(0,0),R(0,1), R(0,2), t(0),
	       R(1,0),R(1,1), R(1,2), t(1),
	       R(2,0),R(2,1), R(2,2), t(2));
  if(!CheckCoherentRotation(R))
	  return Mat();

  Matx34d P(1,0,0,0,
		  	0,1,0,0,
		  	0,0,1,0);
  vector<Point3d> pointcloud;
  vector<Point2d> points0_good,points1_good;
  for(int i=0;i<status.size();i++)
	  if(status[i]){
		  points0_good.push_back(points[0][i]);
		  points1_good.push_back(points[1][i]);
	  }
  cerr<<"Starting points triangulation\n";
  TriangulatePoints(points0_good,points1_good, Kmat.inv(), P, P1, pointcloud);

  Mat trans3d;Mat inliers;
  cerr<<"Estimating affine transform between point clouds\n";
  estimateAffine3D(points0_good,points1_good,trans3d,inliers,3,0.99);
  prevPoints3d = pointcloud;

  //we are only worried about scaling.
  double vx = trans3d.at<double>(0,0); double vy = trans3d.at<double>(1,1);double vz = trans3d.at<double>(2,2);

  //change the projection matrix accordingly.
  P1.val[3]/=vx;P1.val[7]/=vy;P1.val[11]/=vz;

  cerr<<"Projection from fundamental mat\n";
  cerr<<P1<<endl;

  return P;
}
