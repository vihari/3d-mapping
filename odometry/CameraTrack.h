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

using namespace cv;
namespace std {

class CameraTrack {
public:
	//The current projection matrix
	Mat p;
	vector<Point3f> features;
	vector<Point2f> featurePoints;
	//Normalisation parameters for X,Y,Z
	float mux,sigmax;
	float muy,sigmay;
	float muz,sigmaz;

	/**
	 * Initialise with the dense flow image(need not be dense) and interst points*/
	CameraTrack(Mat flowImage, vector<Point2f> features);
	/**
	 * update features and track teh poistion with tracked features*/
	void UpdatePosition(vector<Point2f> trackedFeatures, vector<uchar> status);
	/**
	 * Features are no longer visible and replace them with these!*/
	void ReplaceFeatures(vector<Point2f> newFeatures);
	/**
	 * @return the current projection matrix*/
	Mat GetProjectionMatrix();
	/**
	 * Prints some interesting stuff about projection matrix.*/
	void print(Mat proj);

	virtual ~CameraTrack();
};

}

#endif /* CAMERATRACK_H_ */
