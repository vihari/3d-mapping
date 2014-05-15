/*
 * CameraTrack.cpp
 * Got away with all matrix representations in Mat and replaced with Eigen Mat's
 *
 *  Created on: May 5, 2014
 *      Author: viharipiratla
 */

#include "CameraTrack.h"
#include ""
#include <iostream>

using namespace cv;
namespace std {

  CameraTrack::CameraTrack(Mat flowImage, vector<Point2f> features) {
	p.resize(3,4);
    this->featurePoints = features;
    mux = 0,sigmax=0;
    muy=0,sigmay=0;
    muz=0,sigmaz=0;
    orientation_file = fopen("data/SensorData/ORIENTATION.txt", "r+");

    for(size_t i=0;i<features.size();i++){
      float x = features[i].x;
      float y = features[i].y;
      mux += x;
      muy += y;
      muz += (float)flowImage.data[(int)(y*flowImage.cols+x)];
    }
    mux /= features.size();
    muy /= features.size();
    muz /= features.size();

    for(size_t i=0;i<features.size();i++){
      float x = features[i].x;
      float y = features[i].y;
      sigmax += pow(mux-x,2);
      sigmay += pow(mux-y,2);
      sigmaz += pow(muz-(float)flowImage.data[(int)(y*flowImage.cols+x)],2);
    }
    sigmax /= features.size();
    sigmay /= features.size();
    sigmaz /= features.size();

    cerr<<mux<<" "<<sigmax<<endl;
    cerr<<muy<<" "<<sigmay<<endl;
    cerr<<muz<<" "<<sigmaz<<endl;
    //mux=0,muy=0,muz=0;
    //sigmax=1;sigmay=1,sigmaz=1;
    for(size_t i=0;i<features.size();i++){
      float x = features[i].x;
      float y = features[i].y;
      float z = (float)flowImage.data[(int)(y*flowImage.cols+x)];;
      this->features.push_back(Point3f((x-mux)/sigmax,(y-muy)/sigmay,(z-muz)/sigmaz));
    }
    
    float initialP[] = {1,0,0,0,0,1,0,0,0,0,1,0};
    //p = Mat(3,4,CV_64FC1,&initialP);
    p << 1,0,0,0,0,1,0,0,0,0,1,0;
    cout<<"Initialised!"<<endl;
    cout<<this->features;
  }

  Point3f readFileTill(float time) {
    float timestamp = 0;
    float a, p, r;
    while (timestamp < time) {
      fscanf(orientation_file, "%f;%f;%f;%f\n", &timestamp, &a, &p, &r);
      fseek(orientation_file, 32 * 4, SEEK_CUR);
      printf("%f %f %f %f\n", timestamp, a, p, r);
    }
    Point3f pt;
    pt.x = a;
    pt.y = p;
    pt.z = r;
    return pt;
  }

  string CameraTrack::type2str(int type) {
     string r;

     uchar depth = type & CV_MAT_DEPTH_MASK;
     uchar chans = 1 + (type >> CV_CN_SHIFT);

     switch ( depth ) {
       case CV_8U:  r = "8U"; break;
       case CV_8S:  r = "8S"; break;
       case CV_16U: r = "16U"; break;
       case CV_16S: r = "16S"; break;
       case CV_32S: r = "32S"; break;
       case CV_32F: r = "32F"; break;
       case CV_64F: r = "64F"; break;
       default:     r = "User"; break;
     }

     r += "C";
     r += (chans+'0');

     return r;
   }

  void CameraTrack::UpdatePosition(vector<Point2f> prevFeatures,vector<Point2f> trackedFeatures,vector<uchar> status){
    cerr<<"Call to update position"<<endl;
    //Mat A(4,features.size(),CV_64F);
    //Mat B(3,features.size(),CV_64F);
    cerr<<features.size()<<"\n";
    Eigen::MatrixXf A(4,features.size());
    Eigen::MatrixXf B(3,features.size());
    cerr<<"l 90\n";
    cerr<<p.rows()<<" "<<p.cols()<<"\n";
    float a = p(2,0); float b = p(2,1); float c = p(2,2);float d = p(2,3);
    cerr<<"l 61"<<endl;
    //cout<<features;
    cerr<<"The type of the matrix P is: ";
    //cerr<<type2str(p.type())<<"\n";
    for(size_t i=0;i<features.size();i++){
		  //if(status[i]){
		/*A.at<double>(0,i) = features[i].x;
		A.at<double>(1,i) = features[i].y;
		A.at<double>(2,i) = features[i].z;
		A.at<double>(3,i) = 1;*/
    	A(0,i) = features[i].x;
    	A(1,i) = features[i].y;
    	A(2,i) = features[i].z;
    	A(3,i) = 1;

		double wi = a*features[i].x+b*features[i].y+c*features[i].z+d;
		wi=1;
		/*B.at<double>(0,i) = wi*(trackedFeatures[i].x-featurePoints[i].x);
		B.at<double>(1,i) = wi*(trackedFeatures[i].y-featurePoints[i].y);
		B.at<double>(2,i) = 0;*/
		B(0,i) = wi*(trackedFeatures[i].x-prevFeatures[i].x);
		B(1,i) = wi*(trackedFeatures[i].y-prevFeatures[i].y);
		B(2,i) = wi*sqrt(pow((trackedFeatures[i].x-prevFeatures[i].x),2)+pow((trackedFeatures[i].y-prevFeatures[i].y),2));//0;
	//}
    }
    cerr<<"l 75"<<endl;

    Mat tmp(4,features.size(),CV_64FC1);
    for(int i=0;i<4;i++)
    	for(int j=0;j<features.size();j++)
    		tmp.at<double>(i,j) = A(i,j);
    Mat AinvM = tmp.inv(DECOMP_SVD);
    Eigen::MatrixXf Ainv(features.size(),4);
    for(int i=0;i<features.size();i++)
        	for(int j=0;j<4;j++)
        		Ainv(i,j) = AinvM.at<double>(i,j);
    p += B*Ainv;
    Eigen::MatrixXf identity = Ainv*A;
    cerr<<"*********Identity******\n";
    cerr<<identity.determinant()<<endl;
    cerr<<"*********P******\n";
    cerr<<(B*Ainv)<<endl;
    cerr<<"*********B******\n";
    cerr<<B<<endl;
    //cout<<p;

    featurePoints = trackedFeatures;
    //A.release();
    //B.release();
    cerr<<"Updated position!"<<endl;
  }

  void CameraTrack::ReplaceFeatures(vector<Point2f> newFeatures){
    cout<<"Replacing Features\n";
     featurePoints = newFeatures;
    //flush all features
    features.clear();
    Mat pmat(3,4,CV_64FC1);
    for(int i=0;i<3;i++)
    	for(int j=0;j<4;j++)
    		pmat.at<double>(i,j) = p(i,j);
    Mat pinv = pmat.inv(DECOMP_SVD);
    print(pmat*pinv);
    for(size_t i=0;i<newFeatures.size();i++){
      float data[] = {newFeatures[i].x,newFeatures[i].y,1};
      Mat homo2d(3,1,CV_64FC1,&data);
      Mat homo3d = pinv*homo2d;
      float w = 1;
      if(homo3d.data[homo3d.cols*3]!=0)
    	  w = 1/homo3d.data[homo3d.cols*3];
      features.push_back(Point3f((w*homo3d.at<double>(0,0)-mux)/sigmax,(w*homo3d.at<double>(1,0)-muy)/sigmay,(w*homo3d.at<double>(2,0)-muz)/sigmaz));
    }
  }

  Eigen::MatrixXf CameraTrack::GetProjectionMatrix(){
    return this->p;
  }

  void CameraTrack::print(Mat proj){
    cout<<proj;
  }

  CameraTrack::~CameraTrack() {
    // TODO Auto-generated destructor stub
  }
} /* namespace std */
