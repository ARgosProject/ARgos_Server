#define _USE_MATH_DEFINES
#include <math.h>
#include <cstdio>
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opticalFlow.hpp"

using namespace std;
using namespace cv;

OpticalFlow::OpticalFlow(){
  
  needToInit = true;
  
  /* Shi-Tomasi Parameters */
  maxCorners = 200;
  qualityLevel = 0.01;
  minDistance = 10;
  blockSize = 3;
  useHarrisDetector = false;
  k = 0.04;                         //Free parameter of the Harris detector  
  
  /* Corner SubPix parameters */
  termcrit =  TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
  subPixWinSize = Size(10,10);
  
  /* Lucas-Kanade Parameters */
  winSize = Size(31,31);
  maxLevel = 3;
  flags = 0;
  minEigThreshold = 0.001;
  
  /*Debug */
  line_thickness = 1;
  line_color = CV_RGB(255,0,0);
}

OpticalFlow::~OpticalFlow(){}

void OpticalFlow::track(const cv::Mat &currentFrame){
  currentFrame.copyTo(image);
  //prevPoints.clear();
  //currentPoints.clear();

  if (currentFrame.type() == CV_8UC3) 
    cv::cvtColor(currentFrame, greyFrame, CV_BGR2GRAY);
  else     
    greyFrame = currentFrame;
  
  if(needToInit){
    goodFeaturesToTrack(greyFrame, currentPoints, maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k);
    cornerSubPix(greyFrame, currentPoints, subPixWinSize, Size(-1,-1), termcrit);
    needToInit =  false;
    //for(int i = 0; i < maxCorners; i++)
    // circle(image, prevPoints[i], 3, Scalar(255,0,0), -1, 8);
  }
  else if(!prevPoints.empty()){
    vector<uchar> status;
    vector<float> err;
    if(prevGrey.empty()) greyFrame.copyTo(prevGrey);
    calcOpticalFlowPyrLK(prevGrey, greyFrame, prevPoints, currentPoints, status, err, winSize, maxLevel, termcrit, flags, minEigThreshold);
    
    size_t i, j;
    for(i = j = 0; i < currentPoints.size(); i++){
      /* If Pyramidal Lucas Kanade didn't really find the feature, skip it. */
      if (status[i] == 0)
	continue;
      
      currentPoints[j++] = currentPoints[i];
      circle(image, currentPoints[i], 3, Scalar(0,255,0), -1, 8);
    
      /* Let's make the flow field look nice with arrows. */
      /* The arrows will be a bit too short for a nice visualization because of the high framerate
       * (ie: there's not much motion between the frames).  So let's lengthen them by a factor of 3.
       */
      
      Point2f p,q;
      //Point on previous frame 
      p.x = currentPoints[i].x;
      p.y = currentPoints[i].y;
      // same Point on current frame
      q.x = prevPoints[i].x;
      q.y = prevPoints[i].y;
      
      float angle;           angle = atan2(p.y - q.y, p.x - q.x);
      float hypotenuse;      hypotenuse = sqrt(square(p.y - q.y) + square(p.x - q.x));
      
      //fenergy += hypotenuse * hypotenuse;
      if (hypotenuse > 0.0){
	//                      {
	/* Here we lengthen the arrow by a factor of three. */
	q.x = (p.x - 3 * hypotenuse * cos(angle));
	q.y = (p.y - 3 * hypotenuse * sin(angle));
	
	/* Now we draw the main line of the arrow. */
	/* "image" is the frame to draw on.
	 * "p" is the point where the line begins.
	 * "q" is the point where the line stops.
	 * "CV_AA" means antialiased drawing.
	 * "0" means no fractional bits in the center cooridinate or radius.
	 */
	line(image, p, q, line_color, line_thickness, CV_AA, 0 );
	
	/* Now draw the tips of the arrow.  I do some scaling so that the
	 * tips look proportional to the main line of the arrow.
	 */                      
	p.x = (q.x + 9 * cos(angle + M_PI / 18));
	p.y = (q.y + 9 * sin(angle + M_PI / 18));
	line(image, p, q, line_color, line_thickness, CV_AA, 0 );
	
	p.x = (q.x + 9 * cos(angle - M_PI / 18));
	p.y = (q.y + 9 * sin(angle - M_PI / 18));
	line(image, p, q, line_color, line_thickness, CV_AA, 0 );   
      }
    }
    currentPoints.resize(j);
  }
  
  if (currentPoints.size() < 50)
    needToInit = true;
  
  std::swap(currentPoints, prevPoints);
  cv::swap(prevGrey,greyFrame);
}
  

    
    
    


