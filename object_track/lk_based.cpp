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
  
  Bug revision: 10th Jan 2013.
  Bugs: 
  1. Dead reckoning error, the mask seems to be strained and deforms badly with time.
     Possible fixes: 
     1. Set the mask right by taking reading from sensor readings, read pitch, roll and try to set the mask right.:
        This does not seem to work, one can check that on their own by setting GLOBAL_OPTIMISE macro to the frequency of frames.
	Possible reasons are pitch and roll measurements are not sensitive enough and also proper transformation based on pitch, yaw and roll is not used.
     2. Other possible approach can be rectify mask after few frames or after every frame. Try expanding mask or contract until there is an edge, 
        this seems to be lucrative solution at the expense of extra computation. 
*/
#include <vector>
#include <iostream>
#include <string>
#include <cmath>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

#define _USE_MATH_DEFINES
#define MIN_INDEX 1
#define MAX_INDEX 200
#define FRAME_RATE 25
#define GLOBAL_OPTIMISE 3

using namespace cv;
using namespace std; 

const char pattern[] = "../data/frames/image-%03d.png";
FILE* orientation_file;

/*
  Reads the file to a desired time
  and returns the value at the end of particular time.
*/
Point3f readFileTill(float time){
  float timestamp = 0;
  float a,p,r;
  while( timestamp < time ){
    fscanf(orientation_file, "%f;%f;%f;%f\n", &timestamp, &a, &p, &r);
    fseek( orientation_file, 32*4, SEEK_CUR );
    printf("%f %f %f %f\n", timestamp, a, p, r);
  }  
  Point3f pt; pt.x = a; pt.y = p; pt.z = r;
  return pt;
}

/*
  Input: two 2*3 matrices of type CV_64F
  Output: matrix that results from fusion of input matrices. that is 
          addition of translation components and multiplication of rotation parts.
*/
Mat MultiplyAffineTransformation(Mat a, Mat b){
  Mat result = Mat::zeros( 2, 3, CV_64F);
  Mat x = Mat::zeros(2, 2, CV_64F);
  Mat y = Mat::zeros(2, 2, CV_64F);
  Mat r;
  for(int i=0; i<2; i++)
    for(int j=0; j<2; j++){
      x.at<double>(i,j) = a.at<double>(i,j);
      y.at<double>(i,j) = b.at<double>(i,j);
    }
  
  r = x * y;

  //cout << "a:" <<endl;
  //cout << a << endl; cout << b << endl;
  //cout << x << endl; cout << y << endl;

  result.at<double>(0,2) = a.at<double>(0,2) + b.at<double>(0,2);
  result.at<double>(1,2) = a.at<double>(1,2) + b.at<double>(1,2);
  //result.at<double>(2,0) = a.at<double>(2,0) + b.at<double>(2,0);
  //result.at<double>(2,1) = a.at<double>(2,1) + b.at<double>(2,1);
  for(int i=0;i<2; i++)
    for(int j=0; j<2; j++)
      result.at<double>(i,j) = r.at<double>(i,j);
  
  cout << "Result:" << endl;
  cout << result << endl;
  return (result);
}

/*
  Input is the mask image that masks the object.
  It tries to track the object in the mask.
  Output: no sense.
*/
int object_track(Mat mask){
  orientation_file = fopen("../data/SensorData/ORIENTATION.txt","r+");
  //read in the initial orientation
  Point3f initialOrientation = readFileTill(0.01);
  //To maintain time
  float time = 0;
  
  //imwrite("mask.png", maskImage);
  Mat maskImage;
  //sometimes when features have to be re-initialised 
  Mat prevMaskImage;
  threshold(mask, maskImage, 250, 255, 0);
  Mat initialMask = maskImage.clone();
  
  //this is cummulative transformation.
  Mat transformation = Mat::zeros(2, 3, CV_64F);
  transformation.at<double>(0,0) = 1; transformation.at<double>(1,1) = 1;

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
    time += 2*1/FRAME_RATE;
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
    double x = t.at<double>(0,0); double y = t.at<double>(1,0);
    printf("norm: %f\n", x*x+y*y);

    Mat tmp = Mat::zeros(maskImage.rows, maskImage.cols, maskImage.type());
    
    transformation = MultiplyAffineTransformation(transformation, t);
    
    //once in few frames we try to reset the mask with respect to the initial mask 
    if( (i != MIN_INDEX) && (i - MIN_INDEX)%GLOBAL_OPTIMISE == 0 ){
      Point3f pt = readFileTill(time);
      float angle = pt.z - initialOrientation.z;
      angle *= M_PI/180;
      transformation.at<double>(0,0) = cos(angle);transformation.at<double>(1,1) = cos(angle);
      transformation.at<double>(1,0) = -sin(angle);
      transformation.at<double>(0,1) = sin(angle);
      cout << "Global transformation:" << endl;
      cout << transformation << endl;
      warpAffine( initialMask, tmp, transformation, tmp.size() );
    }
    else{
      //transforming the mask
      warpAffine( maskImage, tmp, t, tmp.size() );
    }
    
    prevMaskImage = maskImage.clone();
    maskImage = tmp.clone();
    Mat mask3c;
    Mat tmp2[] = {maskImage,maskImage,maskImage};
    merge(tmp2, 3, mask3c);
    
    tmp = currImage & mask3c;
    imshow("Mask", tmp);
    waitKey();
  }
}

/*Main for the sake of debugging else should be called after segmetation of the object finishes.*/
int main(){
  Mat mask = imread("mask.png", 0);
  object_track(mask);
}
