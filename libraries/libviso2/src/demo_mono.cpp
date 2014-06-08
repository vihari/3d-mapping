/*
Copyright 2012. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

/*
  Documented C++ sample code of mono visual odometry (modify to your needs)
  To run this demonstration, download the Karlsruhe dataset sequence
  '2010_03_09_drive_0019' from: www.cvlibs.net!
  Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019
*/

#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>

#include <viso_mono.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;

/*void drawLine(cv::Mat img, cv::Point start, cv::Point end){
  int thickeness=2;
  int linetype=8;
  line(img,start,end,Scalar(255,255,255),thickness,linetype);
  }*/

int main (int argc, char** argv) {

  //just give it the folder and specify the format of image files
  if (argc<3) {
    cerr << "Usage: ./viso2 path/to/sequence/ image-%03d.pgm" << endl;
    return 1;
  }

  // sequence directory
  string dir = argv[1];
  string format = argv[2];

  VisualOdometryMono::parameters param;
  
  // calibration parameters for sequence 2010_03_09_drive_0019
  //TODO: fill it with appropriate calib params.
  //param.calib.f  = 645.24; // focal length in pixels
  //param.calib.cu = 635.96; // principal point (u-coordinate) in pixels
  //param.calib.cv = 194.13; // principal point (v-coordinate) in pixels
  //param.base     = 0.5707; // baseline in meters
  
  //imp params for mono
  param.calib.f  = 614.21994668037830; // focal length in pixels
  param.calib.cu = 3.4558123240636974e+02; // principal point (u-coordinate) in pixels
  param.calib.cv = 2.5058513263943294e+02; // principal point (v-coordinate) in pixels
  param.height = 0.25;
  param.pitch = -0.26;
  param.motion_threshold = 100;
  VisualOdometryMono viso(param);
  cv::Mat cameraMatrix = (cv::Mat_<double>(3,3)<<612.91994668037830, 0, 345.58123240636974,0,614.00708530052441, 250.58513263943294,0, 0, 1);
  std::vector <double> distCoeffs;
  distCoeffs.push_back(2.3848783769193385e-01);
  distCoeffs.push_back(-1.4566591819736021e+00);
  distCoeffs.push_back(-1.9239952001727575e-03);
  distCoeffs.push_back(6.3304014166893946e-04);
  distCoeffs.push_back(2.5757547632849924e+00);
  Matrix pose = Matrix::eye(4);
    
  for (int32_t i=1; i<238; i++) {
    
    // input file names
    char base_name[256]; 
    sprintf(base_name,format.c_str(),i);
    string img_file_name  = dir + "/" +base_name;
        
    // catch image read/write errors here
    try {

      // load left and right input image
      cv::Mat img = cv::imread(img_file_name), imgNew = cv::imread(img_file_name);
      cerr<<"Read image from: "<<img_file_name<<"\n";
      cv::undistort(img, imgNew, cameraMatrix, distCoeffs);         
      uint8_t* img_data  = imgNew.data;
      
      // status
      cerr << "Processing: Frame: " << i;
      
      int32_t width = img.cols; int32_t height = img.rows;
      //      cout<<"Rows: "<<height<<" Cols: "<<width<<endl;
      // compute visual odometry
      //TODO: without teh -1's below throws EXEC_BAD_ACCESS l:149 filter.cpp
      int32_t dims[] = {width-1,height-1,width-1};
      if (viso.process(img_data,dims)) {
            
        // output some statistics
        double num_matches = viso.getNumberOfMatches();
        double num_inliers = viso.getNumberOfInliers();
        cerr << ", Matches: " << num_matches;
	double prop = num_inliers/num_matches;
	
        cerr << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
	cerr<<base_name<<endl;
        cerr << pose << endl << endl;

	// on success, update current pose
        pose = pose * Matrix::inv(viso.getMotion());

	cout<<pose.val[0][3] <<" "<<pose.val[1][3] <<" "<< pose.val[2][3]<<"\n";	
	//	updateGraphic(pose);
      } else {
	cerr << " ... failed!" << endl;
      }

      // release uint8_t buffers
      //free(img_data);

    // catch image read errors here
    } catch (...) {
      cerr << "ERROR: Couldn't read input files!" << endl;
      return 1;
    }
  }
  
  // output
  cerr << "Demo complete! Exiting ..." << endl;

  // exit
  return 0;
}
