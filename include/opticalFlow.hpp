#ifndef _ARGOS_OPTICALFLOW_H
#define _ARGOS_OPTICALFLOW_H
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdio>
#include <iostream>

using namespace std;
using namespace cv;


/**
 * \brief Main class for paper detection
 */
class OpticalFlow{
  
public:
  /**
   * Default constructor 
   */
  OpticalFlow();
  
  /**
   * Default destructor
   */
  ~OpticalFlow();
  
 
  //squares a number
  inline static float square(int a){
    return a * a;
  }
   
  void track(const cv::Mat &currentFrame);
  
  const cv::Mat & getImage(){
    return image;
  }
  



private:
  Mat greyFrame;
  Mat prevGrey;
  Mat image;
  
  bool needToInit;

  vector<Point2f> prevPoints;
  vector<Point2f> currentPoints;
  
  /* Shi-Tomasi Parameters */
  int maxCorners;
  double qualityLevel;
  double minDistance;
  int blockSize;
  bool useHarrisDetector;
  double k;                         //Free parameter of the Harris detector  
  
  /* Corner SubPix parameters */
  TermCriteria termcrit;
  Size subPixWinSize;
  
  /* Lucas-Kanade Parameters */
  Size winSize;
  int maxLevel;
  int flags;
  double minEigThreshold;
  
  /*Debug */
  
  int line_thickness;
  Scalar line_color;
};

#endif
