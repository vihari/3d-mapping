/*
  @author: Vihari Piratla
  @Date: 1st December 2013
  This function tracks the detected object across frames:
  1. Initilises features in the mask defined.
  2. These features are tracked with lk and 
  3. affine transformation is calculated.
  4. This affine transformation is used to redefine the image.
  5. The mask is redefined and expanded or contracted based on the color 
     based patch propagation. 
*/
#include <vector>
#include <iostream>
#include <string>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

#define MIN_INDEX 1
#define MAX_INDEX 200

using namespace cv;
using namespace std; 

const char pattern[] = "../data/frames/image-%03d.png";
FILE* orientation_file;

/*
  Reads the file to a desired time,
  note that one can only move in chronological 
  order and cannot go back
*/
Point3f readFileTill(float time){
  float timestamp = 0;
  float a,p,r;
  while( timestamp < time)
    fscanf(orientation_file, "%f;%f;%f;%f", &timestamp,&a,&p,&r);
  Point3f pt; pt.x = a; pt.y = p; pt.z = r;
  return pt;
}

Mat MultiplyAffineTransformation(Mat a, Mat b){
  Mat result = Mat::zeros( 2, 3, float);
  result[0][2] = a[0][2] + b[0][2];
  result[1][2] = a[1][2] + b[1][2];
  result[0][0] = a[0][0]*b[0][0] + a[0][1]*b[1][0];
  result[0][1] = a[0][0]*b[0][1] + a[0][1]*b[1][0];
  result[1][0] = a[1][0]*b[0][0] + a[1][1]*b[1][1];
  result[1][1] = a[1][0]*b[0][1] + a[1][1]*b[1][1];
  return (result);
}

/*
  Input is the mask image that masks the object.
*/
int object_track(Mat mask){
  orientation_file = fopen("../data/SensorData/ORIENTATION.txt"/,"r+");
  //read in the initial orientation
  Point3f initialOrientation = readFileTill(0.01);

  //imwrite("mask.png", maskImage);
  Mat maskImage;
  //sometimes when features have to be re-initialised 
  Mat prevMaskImage;
  threshold(mask, maskImage, 250, 255, 0);
  Mat initialMask = maskImage.clone();
  //this is cummulative transformation.
  Mat transformation = Mat::zeros(2, 3, float);
  transformation[0][0] = 1; transformation[1][1] = 1;

  
  TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
  Size subPixWinSize(10,10), winsize(31,31);
  const int MAX_COUNT = 500;

  //contains the tracked features
  vector<Point2f> points[2];
  Mat currImage, prevImage, currGray, prevGray;
  char imageName[] = "../data/frames/image-001.png";//filePrefix + MIN_INDEX + ".png";
  currImage = imread(imageName, 1);
  cvtColor(currImage, currGray, CV_BGR2GRAY);
 
  //currGray = currGray & maskImage;
  //initialise
  goodFeaturesToTrack(currGray, points[1], MAX_COUNT, 0.01, 1, maskImage, 3, 0, 0.04);
 
  cornerSubPix(currGray, points[1], subPixWinSize, Size(-1,-1), termcrit);
  for(int i = 0; i < points[1].size(); i++)
    circle(currImage, points[1][i], 1, Scalar(0,255,0), -1, 8);
  
  namedWindow("Mask", 0);
  imshow("Mask", currImage); 
  waitKey();

  for(int i=MIN_INDEX; i<MAX_INDEX; i += 2){
    points[0] = points[1];
    prevImage = currImage.clone();
    prevGray = currGray.clone();
    sprintf(imageName, pattern, i);
    cout<<imageName<<endl;
    currImage = imread(imageName, 1);
    cvtColor(currImage, currGray, CV_BGR2GRAY);
    
    //Re-initialise the features after 20 frames
    if((i - MIN_INDEX)%10 == 0){
      goodFeaturesToTrack(currGray, points[1], MAX_COUNT, 0.01, 1, maskImage, 3, 0, 0.04);
      cornerSubPix(currGray, points[1], subPixWinSize, Size(-1,-1), termcrit);
      
      goodFeaturesToTrack(prevGray, points[0], MAX_COUNT, 0.01, 1, prevMaskImage, 3, 0, 0.04);
      cornerSubPix(prevGray, points[0], subPixWinSize, Size(-1,-1), termcrit);
    }

    //status and err in tracking a point
    vector<uchar> status; 
    vector<float> err;
    calcOpticalFlowPyrLK(prevGray, currGray, points[0], points[1], 
			 status, err, winsize, 3, termcrit, 0, 0.001);
    
    //prune badly tracked points
    int k = 0;
    for(int j = 0; j < points[1].size(); j++){
      if( !status[j] )
	continue;
      
      points[0][k] = points[0][j];
      points[1][k++] = points[1][j];
      circle(currImage, points[1][j], 1, Scalar(0,255,0), 0.5, 8);
    }
    points[0].resize(k);
    points[1].resize(k);
    
    Mat t = estimateRigidTransform(points[0], points[1], false);
    cout << "**********************" << endl;
    cout << t << endl;
    
    Mat tmp = Mat::zeros(maskImage.rows, maskImage.cols, maskImage.type());
    //transforming the mask
    warpAffine( maskImage, tmp, t, tmp.size() );
    transformation = MultiplyAffineTranforms(transformation, t);
    
    prevMaskImage = maskImage.clone();
    maskImage = tmp.clone();
    Mat mask3c;
    Mat tmp2[] = {maskImage,maskImage,maskImage};
    merge(tmp2, 3, mask3c);
    
    tmp = currImage & mask3c;
    imshow("Mask", tmp);
    waitKey();
    
    //once in few frames we try to reset the mask with respect to the initial mask 
  }
}

int main(){
  Mat mask = imread("mask.png", 0);
  object_track(mask);
}
