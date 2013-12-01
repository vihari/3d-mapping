/**
 * @file CannyDetector_Demo.cpp
 * @brief Sample code showing how to detect edges using the Canny Detector
 * @author OpenCV team
 */

#include "cv.h"
#include "highgui.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <vector>

#define THRESHOLD 0.0
using namespace cv;

/// Global variables

Mat src, src_gray;
Mat dst, detected_edges;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
const char* window_name = "Edge Map";
Rect rect;
bool g_press = false;

Point pt1,pt2;

/*
  @@args: the first 3 arguments are for rectangle specification and value to be filled in the rectangle.
*/
void fill(Point pt, int width, int height, int value){
  for(int y = pt.y; y < (pt.y+height); y++)
    for(int x = pt.x; x < (pt.x+width); x++)
      dst.at<unsigned char>(y, x) = value;
}

/*
  @@args: rectangle specification
  Returns true if the mean intensity value inside the rectangle > THRESHOLD value.
  THRESHOLD value should be chosen so as to be robust to some error.
*/
bool EdgeExists(Point pt, int width, int height){
  float value = 0.0;
  for(int y = pt.y; y < pt.y+height; y++)
    for(int x = pt.x; x < pt.x+width; x++)
      value += dst.at<unsigned char>(y,x); 
  
  value /= (float)(width*height);
  if(value > THRESHOLD)
    return true;
  else
    return false;
}

/*
  @@args: are the left-top and right-bottom pts of rectangle.
  We adopt a bfs style search here:
  we push in to the array new pts to right and bottom to be searched
  a rectangle is labeled either as a 0 or 1 depending on whether it has an edge.
*/
void SegmentObject(Point pt1, Point pt2){
  int width = pt2.x - pt1.x;
  int height = pt2.y - pt1.y;
  
  //The user must have just clicked on the image.
  //if((width==0) || (height==0))
  //  return;

  std::vector<Point> regions;
  regions.push_back(pt1);
  while(!regions.empty()){
    Point pt = regions.back();
    //printf("%d %d\n",pt.x,pt.y);
    regions.pop_back();
    
    if(!EdgeExists(pt,width,height)){
      if(((pt.x+width) < dst.cols) && ((pt.x+width) > 0)){
	Point new_pt = Point(pt.x+width, pt.y);
	regions.push_back(new_pt);
      }
      
      if(((pt.y+height) < dst.rows) && ((pt.y+height) > 0)){
	Point new_pt = Point(pt.x, pt.y+height);
	regions.push_back(new_pt);
      }
      fill(pt, width, height, 255);
    }
    else{
      //fill with max_values
    }
    //fill(pt, width, height, 255);
  }
  imshow(window_name, dst);
}

void CallbackFunc(int event, int x, int y, int flags, void* param){
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

      ////By this statement, I want to fill the final rectangle and hold it in the window. I examined it for several times and I did not find mistakes. I just want to know the reason.
      Mat tmp = dst.clone();
      rectangle(tmp, Point(rect.x, rect.y), Point(rect.x+rect.width, rect.y+rect.height), CV_RGB(100,0,100), 1/*CV_FILLED*/);
      imshow(window_name, tmp);
      Point pt1 = Point(rect.x, rect.y);
      Point pt2 = Point(rect.x+rect.width, rect.y+rect.height);
      
      //Now we call the function to segment the object
      SegmentObject(pt1, pt2);
    }break;
  }
}

/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */
static void CannyThreshold(int, void*){
  /// Reduce noise with a kernel 3x3
  blur( src_gray, detected_edges, Size(3,3) );
  
  /// Canny detector
  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
  
  /// Using Canny's output as a mask, we display our result
  dst = Scalar::all(0);

  src_gray.copyTo( dst, detected_edges);

  imshow( window_name, dst );
}


/**
 * @function main
 */
Mat EdgeDetector( Mat flow_map ){
  
  cv::normalize(flow_map, src_gray, 0, 255, NORM_MINMAX, CV_8UC1);
  //src_gray = flow_map.clone();
  
  if( !src_gray.data )
    { return flow_map; }

  /// Create a matrix of the same type and size as src (for dst)
  dst.create( src_gray.size(), src_gray.type() );
  
  /// Convert the image to grayscale
  //cvtColor( src, src_gray, COLOR_BGR2GRAY );

  /// Create a window
  namedWindow( window_name, WINDOW_AUTOSIZE );

  /// Create a Trackbar for user to enter threshold
  createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );

  /// Show the image
  CannyThreshold(0, 0);

  setMouseCallback(window_name, CallbackFunc, NULL);

  if(g_press) {
    rectangle(dst, Point(rect.x, rect.y), Point(rect.x+rect.width, rect.y+rect.height), CV_RGB(50,50,50));
  }
  imshow(window_name, dst);
  
  int threshold_value = 250;
  int threshold_type = 0;
  int const max_BINARY_value = 255;
  Mat mask;
  //threshold( dst, mask, threshold_value, max_BINARY_value, threshold_type );
        
  // Wait until user exit program by pressing a key
  waitKey(0);

  return dst;
}
