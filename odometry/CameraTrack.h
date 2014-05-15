/*
 * CameraTrack.h
 *
 *  Created on: May 5, 2014
 *      Author: viharipiratla
 */

#ifndef CAMERATRACK_H_
#define CAMERATRACK_H_
#include <vector>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include <Eigen/Dense>


using namespace cv;
namespace std {

class CameraTrack {
public:
	//The current projection matrix
	Eigen::MatrixXf p;
	vector<Point3f> features;
	vector<Point2f> featurePoints;
	FILE* orientation_file;

	//Normalisation parameters for X,Y,Z
	float mux,sigmax;
	float muy,sigmay;
	float muz,sigmaz;

	/**
	 * Initialise with the dense flow image(need not be dense) and interst points*/
	CameraTrack(Mat flowImage, vector<Point2f> features);
	/**
	 * update features and track teh poistion with tracked features*/
	void UpdatePosition(vector<Point2f> prevFeatures,vector<Point2f> trackedFeatures, vector<uchar> status);
	/**
	 * Features are no longer visible and replace them with these!*/
	void ReplaceFeatures(vector<Point2f> newFeatures);
	/**
	 * @return the current projection matrix*/
	Eigen::MatrixXf GetProjectionMatrix();

	 /*
	  * Reads the file to a desired time
	  * and returns the value at the end of particular time.
	  */
	Point3f readFileTill(float time);

	string type2str(int type);

	/**
	 * Prints some interesting stuff about projection matrix.*/
	void print(Mat proj);

	virtual ~CameraTrack();
};
}
#endif /* CAMERATRACK_H_ */
