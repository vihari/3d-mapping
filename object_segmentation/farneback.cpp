/*
 * @file farneback.cpp
 * @brief Segments image from dense optical flow, edge detections and grab cut.
 * @author Vihari Piratla
 */
#include "cv.h"
#include "highgui.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include <iostream>

#include "CannyDetector.cpp"
#include "../grabcut/grabcut.cpp"
//#include "../object_track/lk_based.cpp"
#define max(a,b) a>b?a:b
#define TOLERANCE 0.1
#define DEBUG false
#define MASK_DEFINE false

using namespace cv;
using namespace std;

int histSize = 256;
Mat flow_vector;
float max_distance = -1;
float lmean;
//foreground model and background model; outputs of grabcut, used as inputs to segmentation in successive frames.
Mat fgdModel;
Mat bgdModel;
Mat frame1, frame2;

void GrabCut(Rect);

void CallbackROI(int event, int x, int y, int flags, void* param){
  switch(event){
  case CV_EVENT_MOUSEMOVE:
    {
      if(g_press == true){
	rect.width = x - rect.x;
	rect.height = y - rect.y;
      }
    }break;
  case CV_EVENT_LBUTTONDOWN:
    {
      g_press = true;
      rect.x = x;
      rect.y = y;
      rect.height = rect.width = 0;
    }break;
  case CV_EVENT_LBUTTONUP:
    {
      g_press = false;
      if(rect.width < 0){
	rect.x += rect.width;
	rect.width *= -1;
      }
      if(rect.height <0){
	rect.y += rect.height;
	rect.height *= -1;
      }

      Mat tmp = frame1.clone();
      rectangle(tmp, Point(rect.x, rect.y), Point(rect.x+rect.width, rect.y+rect.height), CV_RGB(100,0,100), 1/*CV_FILLED*/);
      imshow("Original Image", tmp);
      Point pt1 = Point(rect.x, rect.y);
      Point pt2 = Point(rect.x+rect.width, rect.y+rect.height);
      
      //Now we call the function to segment the object
      GrabCut(rect);
    }break;
  }
}

void CallbackPrintData(int event, int x, int y, int flags, void* userdata)
{
  if  ( event == EVENT_LBUTTONDOWN )
    {
      printf(" %d , %d , %f \n", x, y, flow_vector.at<float>(y,x));
    }
}

void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
                    double scale, const Scalar& color)
{
  for(int y = 0; y < cflowmap.rows; y += step)
    for(int x = 0; x < cflowmap.cols; x += step)
      {
	const Point2f& fxy = flow.at<Point2f>(y, x);
	line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
	     color);
	circle(cflowmap, Point(x,y), 2, color, -1);
      }
}

void getFlowVector(Mat& flow, Mat& hist){
  flow_vector = Mat(flow.rows, flow.cols, CV_32FC1);
  float sum = 0;
  //  float max_distance = -1;
  for(int y=0; y < flow.rows; y++)
    for(int x=0; x < flow.cols; x++){
      const Point2f& fxy = flow.at<Point2f>(y,x);
      float distance = sqrt(fxy.x*fxy.x+fxy.y*fxy.y);
      flow_vector.at<float>(y,x) = distance;
      max_distance = max(max_distance, distance);
      sum += distance; 
    }
  
  lmean = sum/(flow.rows*flow.cols);
  printf("max_distance %f\n",max_distance);
  float range[] = { 0, max_distance} ;
  const float* histRange = { range };

  bool uniform = true; bool accumulate = false;

  /// Compute the histograms
  calcHist( &flow_vector, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );
  namedWindow("Flow vector",1);
  imshow("Flow vector",flow_vector);
  waitKey(-1);
}

void print_help(){
  printf("*****Invalid number of arguements*****\n");
  printf("*****[exe] <Image1> <Image2>\n");
}

/*
  @function: GrabCut(Point, Point);
  @brief: two points that define the ROI of the mask for grabcut segmentation
*/
void GrabCut(Rect rect){
  Mat tmp, mask; 
  Mat result = cv::Mat::ones(frame1.size(), CV_8U) * cv::GC_BGD;
  Mat white = cv::Mat::ones(frame1.size(), CV_8U) * 255;

  //cv::Rect rectangle(50,70,frame1.cols-150,frame1.rows-180);
  grabCut( frame1, result, rect, bgdModel, fgdModel, 20, GC_INIT_WITH_RECT );
  cout<<bgdModel<<endl;
  cout<<fgdModel<<endl;
  //frame1 = frame1 & mask;
  cout << rect<< endl;
  
  compare(result,GC_FGD,tmp,CMP_EQ);

  //Generate output image
  Mat foreground(frame1.size(),CV_8UC3,cv::Scalar(0,0,0));
  frame1.copyTo(foreground, tmp);
  white.copyTo(mask, tmp);
  compare(result,GC_PR_FGD,tmp,CMP_EQ);  
  frame1.copyTo(foreground, tmp);
  white.copyTo(mask, tmp);

  // draw rectangle on original image
  rectangle(frame1, rect, cv::Scalar(255,255,255),1);
  cv::namedWindow("Image");
  cv::imshow("Image",frame1);

  // display result
  namedWindow("Segmented Image");
  imshow("Segmented Image",foreground);

  waitKey();
  imwrite("mask.png", mask);
}

void GrabCut(Mat result){
  Mat tmp;
  /*  compare(result,GC_FGD,tmp,CMP_EQ);
  //compare(result,GC_FGD,tmp,CMP_EQ);
  
  //Generate output image
  Mat mask(frame2.size(),CV_8UC3,cv::Scalar(0,0,0));
  frame2.copyTo(mask, tmp); // bg pixels not copied
  compare(result,GC_PR_FGD,tmp,CMP_EQ);
  frame2.copyTo(mask, tmp);
  */

  /*result = cv::Mat::ones(frame2.size(), CV_8U) * cv::GC_BGD;

  Rect area;
  area.x=10;  area.y=10;
  area.width=250; area.height=250;
  rectangle(result, area , cv::Scalar(cv::GC_PR_FGD),-1,8,0);

  //Fill a smaller rectangle with the foreground value.
  area.x=50;  area.y=50;
  area.width=20;  area.height=20;
  rectangle(result, area , cv::Scalar(cv::GC_FGD),-1,8,0);
  */

  cv::Rect rect;
  grabCut( frame2, result, rect, bgdModel, fgdModel, 5, GC_INIT_WITH_MASK );
  cout<<bgdModel<<endl;
  cout<<fgdModel<<endl;
      
  Mat foreground(frame2.size(),CV_8UC3,cv::Scalar(0,0,0));
  compare(result,GC_PR_FGD,tmp,CMP_EQ);
  frame2.copyTo(foreground,tmp);
  compare(result,GC_FGD,tmp,CMP_EQ);
  frame2.copyTo(foreground,tmp);
  
  // display result
  namedWindow("Segmented Image");
  imshow("Segmented Image", foreground);
  
  //namedWindow("Initial Mask");
  //  imshow("Initial Mask", mask);

  waitKey();
}


int main(int argc, char** argv)
{
  //VideoCapture cap(0);
    
  //if( !cap.isOpened() )
  //return -1;
    
  Mat prevgray, gray, flow, cflow, hist;
  namedWindow("flow", 0);
    
  if(argc < 3){
    print_help();
    return 0;
  }

  //cap >> frame;
  frame1 = imread(argv[1],1);
  frame2 = imread(argv[2],1);
  
  namedWindow("first", 0);
  namedWindow("second", 0);
  
  if(DEBUG){
    imshow("first", frame1);
    imshow("second", frame2);
  }  
  
  cvtColor(frame1, gray, CV_BGR2GRAY);
  cvtColor(frame2, prevgray, CV_BGR2GRAY);

  if( prevgray.data )
    {
      calcOpticalFlowFarneback(prevgray, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
      cvtColor(prevgray, cflow, CV_GRAY2BGR);
      getFlowVector(flow, hist);
      drawOptFlowMap(flow, cflow, 16, 1.5, CV_RGB(0, 255, 0));
      imshow("flow", cflow);
    }

  imwrite("flow.png",flow_vector);
  // Draw the histograms for B, G and R
  int hist_w = 512; int hist_h = 400;
  int bin_w = cvRound( (double) hist_w/histSize );

  Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

  /// Normalize the result to [ 0, histImage.rows ]
  normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

  /// Draw for each channel
  bool set = false;
  float peak;
  for( int i = histSize-1; i >= 0 ; i-- ){
    line( histImage, Point( bin_w*(i+1), hist_h - cvRound(hist.at<float>(i+1)) ) , Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ), Scalar( 255, 0, 0), 2, 8, 0  );
    if((!set) && (i>0)){
      float f_l = hist.at<float>(i-1);
      float f_r = hist.at<float>(i+1);
      float f = hist.at<float>(i);
      if((f_l<f) && (f_r<f)){
	set = true;
	peak = (((float)i)/histSize)*max_distance;
      }
    }
  }
  printf("First peak: %f\n",peak);

  //Threshold calculation for histogram
  float thresh = lmean;
  float new_thresh = lmean+2*TOLERANCE;
  while((new_thresh-thresh)>TOLERANCE){
    float mean1 = 0, mean2 = 0;
    int count1 = 0,count2 = 0;
    thresh = new_thresh;
    for(int y=0; y<flow_vector.rows; y++)
      for(int x=0; x<flow_vector.cols; x++){
	float val = flow_vector.at<float>(y,x);
	if(val < thresh){
	  mean1 += val;
	  count1++;
	}
	else{
	  mean2 += val;
	  count2++;
	}
      }
    mean1 /= count1;
    mean2 /= count2;
    new_thresh = (mean1+mean2)/2;
  }
  
  //thresh = 8.5;
  printf("Threshold: %f\n",thresh);

  for(int y=0; y<flow_vector.rows; y++)
    for(int x=0; x<flow_vector.cols; x++){
      float val = flow_vector.at<float>(y,x);
      if(val < thresh)
	flow_vector.at<float>(y,x) = 0;
      //else
      //flow_vector.at<float>(y,x) = val*(max_distance-thresh)/(max_distance*10);
    }
  
  if(DEBUG){
    namedWindow("hist", 1);
    imshow("hist", histImage);
    imshow("second", flow_vector);
    
    setMouseCallback("second", CallbackPrintData, NULL);
  }

  Mat mask = Mat(frame1.size(), frame1.type());

  if(MASK_DEFINE){
    //This function is available in cannydetector.cpp
    mask = EdgeDetector(flow_vector);
    GrabCut(mask);
  }
  else{
    namedWindow("Original Image", 1);
    imshow("Original Image", frame1);
    setMouseCallback("Original Image", CallbackROI, NULL);
    waitKey();
  }

  /*Mat mask3c;
  Mat tmp[] = {mask,mask,mask};

  merge(tmp, 3, mask3c);
  
  frame1 = frame2 & mask3c;
  imshow("first", frame1);
  //imshow("second", mask3c);
  waitKey();*/

  //object_track(mask);

  std::swap(prevgray, gray);

  return 0;
}
