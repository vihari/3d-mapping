/*
 * CameraTrack.cpp
 *
 *  Created on: May 5, 2014
 *      Author: viharipiratla
 */

#include "CameraTrack.h"

using namespace cv;
namespace std {

CameraTrack::CameraTrack(Mat flowImage, vector<Point2f> features) {
	this->featurePoints = features;
	mux = 0,sigmax=0;
	muy=0,sigmay=0;
	muz=0,sigmaz=0;

	for(size_t i=0;i<features.size();i++){
		float x = features[i].x;
		float y = features[i].y;
		mux += x;
		muy += y;
		muz += flowImage.data[(int)(y*flowImage.cols+x)];
	}
	mux /= features.size();
	muy /= features.size();
	muz /= features.size();

	for(size_t i=0;i<features.size();i++){
		float x = features[i].x;
		float y = features[i].y;
		sigmax += pow(mux-x,2);
		sigmay += pow(mux-y,2);
		sigmaz += pow(muz-flowImage.data[(int)(y*flowImage.cols+x)],2);
	}
	sigmax /= features.size();
	sigmay /= features.size();
	sigmaz /= features.size();

	cerr<<mux<<" "<<sigmax<<endl;
	cerr<<muy<<" "<<sigmay<<endl;
	cerr<<muz<<" "<<sigmaz<<endl;
	for(size_t i=0;i<features.size();i++){
			float x = features[i].x;
			float y = features[i].y;
			float z = flowImage.data[(int)(y*flowImage.cols+x)];
			this->features.push_back(Point3f((x-mux)/sigmax,(y-muy)/sigmay,(z-muz)/sigmaz));
	}

	float initialP[] = {1,0,0,0,0,1,0,0,0,0,1,0};
	p = Mat(3,4,CV_64FC1,&initialP);
	cout<<"Initialised!"<<endl;
}

void CameraTrack::UpdatePosition(vector<Point2f> trackedFeatures,vector<uchar> status){
	cerr<<"Call to update position"<<endl;
	Mat A(4,features.size(),CV_64F);
	Mat B(3,features.size(),CV_64F);
	float a = p.data[p.cols*3+0]; float b = p.data[p.cols*3+1]; float c = p.data[p.cols*3+2];float d = p.data[p.cols*3+3];
	cerr<<"l 61"<<endl;
	for(size_t i=0;i<features.size();i++){
		//if(status[i]){
			A.data[A.cols*0+i] = features[i].x;
			A.data[A.cols*1+i] = features[i].y;
			A.data[A.cols*2+i] = features[i].z;
			A.data[A.cols*3+i] = 1;

			float wi = a*features[i].x+b*features[i].y+c*features[i].z+d;
			B.data[B.cols*0+i] = wi*(trackedFeatures[i].x-featurePoints[i].x);
			B.data[B.cols*0+i] = wi*(trackedFeatures[i].y-featurePoints[i].y);
			B.data[B.cols*0+i] = 0;
		//}
	}
	cerr<<"l 75"<<endl;
	Mat Ainv = A.inv(DECOMP_SVD);
	cerr<<B.cols<<" "<<B.rows<<endl;
	cerr<<Ainv.cols<<" "<<Ainv.rows<<endl;
	p = p+(B*(Ainv));
	print(p);

	featurePoints = trackedFeatures;
	cerr<<"Updated position!"<<endl;
}

void CameraTrack::ReplaceFeatures(vector<Point2f> newFeatures){
	featurePoints = newFeatures;
	//flush all features
	features.clear();
	Mat pinv = p.inv(DECOMP_SVD);
	print(p*pinv);
	for(size_t i=0;i<newFeatures.size();i++){
		float data[] = {newFeatures[i].x,newFeatures[i].y,1};
		Mat homo2d(3,1,CV_64FC1,&data);
		Mat homo3d = pinv*homo2d;
		float w = 1;
		if(homo3d.data[homo3d.cols*3]!=0)
			w = 1/homo3d.data[homo3d.cols*3];
		features.push_back(Point3f((w*homo3d.data[homo3d.cols*0]-mux)/sigmax,(w*homo3d.data[homo3d.cols*1]-muy)/sigmay,(w*homo3d.data[homo3d.cols*2]-muz)/sigmaz));
	}
}

Mat CameraTrack::GetProjectionMatrix(){
	return p;
}

void CameraTrack::print(Mat proj){
	cout<<proj;
}

CameraTrack::~CameraTrack() {
	// TODO Auto-generated destructor stub
}

} /* namespace std */
