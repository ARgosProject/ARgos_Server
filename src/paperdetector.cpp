
//#include <opencv/highgui.h>
//#include <iostream>
//#include <fstream>

#include <valarray>
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <iostream>
#include "paperdetector.hpp"

using namespace std;
using namespace cv;

PaperDetector::PaperDetector(){
  thresMethod = ADPT_THRES;
  thresParam1 = thresParam2 = 7;
  minContourValue = 0.1;
  maxContourValue = 0.8;
  contourAreaValue = 5000;
}


PaperDetector::~PaperDetector(){}

/************************************
 *
 * Main detection function. Performs all steps
 *
 *
 ************************************/
void PaperDetector::detect(const cv::Mat& currentFrame,vector<Paper>& detectedPapers, CameraProjectorSystem& cameraProjector,cv::Size paperSizeMeters, bool setYPerperdicular,  bool screenExtrinsics) throw (cv::Exception){
  //clear input data
  detectedPapers.clear();
  currentFrame.copyTo(debug);
  
  //-Convert to greyScale (it must be a 3 channel image) -------------------------------
  if (currentFrame.type() == CV_8UC3) 
    cv::cvtColor(currentFrame, greyFrame, CV_BGR2GRAY);
  else     
    greyFrame = currentFrame;
  
  //-Thresholding image ----------------------------------------------------------------
  //cv::threshold(greyFrame, outThres, 127, 255, 0);
  //cv::threshold(greyFrame, outThres, 200.0, 255.0, THRESH_BINARY);
  thresHold(thresMethod, greyFrame, outThres, thresParam1, thresParam2);
  //cv::dilate(outThres, outThres, cv::Mat(), cv::Point(-1,-1));
  outThres.copyTo(thres2);

  //find all rectangles in the thresholdes image
  vector<PaperCandidate> PaperCandidates;
  vector<PaperCandidate> OutPaperCandidates;
  
  //- Find Contours--------------------------------------------------------------------  
  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;
  //cv::findContours(thres2, contours, hierarchy, CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
  cv::findContours(thres2, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
  
  // Find Rectangles ----------------------------------------------------
  //findRectangles(contours, PaperCandidates);
  
  // Convex Hull contours------------------------------------------------
  convexHullContours(contours, PaperCandidates);
  //for (size_t i = 0; i < contours.size(); i++)
  //  drawContour(debug,contours[i],Scalar(255,0,225));
  
  
  // Sort the points in anti-clockwise order
  valarray<bool> swapped(false,PaperCandidates.size());  //used later
  for (unsigned int i=0;i<PaperCandidates.size();i++ ){
    //trace a line between the first and second point.
    //if the thrid point is at the right side, then the points are anti-clockwise
    double dx1 = PaperCandidates[i][1].x - PaperCandidates[i][0].x;
    double dy1 =  PaperCandidates[i][1].y - PaperCandidates[i][0].y;
    double dx2 = PaperCandidates[i][2].x - PaperCandidates[i][0].x;
    double dy2 = PaperCandidates[i][2].y - PaperCandidates[i][0].y;
    double o = ( dx1*dy2 )- ( dy1*dx2 );
    
    if ( o  < 0.0 ){		 //if the third point is in the left side, then sort in anti-clockwise order
      swap (PaperCandidates[i][1],PaperCandidates[i][3] );
      swapped[i]=true;
      //sort the contour points
      //reverse(MarkerCanditates[i].contour.begin(),MarkerCanditates[i].contour.end());//????
      
    }
  }
  
  //deletes the one with smaller perimeter
  vector<bool> toRemove (PaperCandidates.size(), false);
  for (int i=0;i<int ( PaperCandidates.size() )-1; i++){
    if (!toRemove[i+1]){
      if (perimeter(PaperCandidates[i]) > perimeter(PaperCandidates[i+1])) 
	toRemove[i+1]=true;
      else toRemove[i]=true;
    }
  }
  //remove the papers marker
  removeElements ( PaperCandidates, toRemove );

  // Calculate Extrinsics------------------------------------------------
  ///detect the position of detected markers if desired
  if (cameraProjector.isValid()  && (paperSizeMeters.width > 0 && paperSizeMeters.height > 0)) {
    for(unsigned int i=0; i < PaperCandidates.size(); i++)
      PaperCandidates[i].calculateExtrinsics(paperSizeMeters, cameraProjector, setYPerperdicular, screenExtrinsics);
  }
  
  // Return vector
  for (unsigned int i=0;i<PaperCandidates.size();i++ ){
    detectedPapers.push_back(PaperCandidates[i]);
  }
}

/************************************
 *
 *
 *
 *
 ************************************/

void PaperDetector::convexHullContours(vector<vector<cv::Point> > &contours, vector<PaperCandidate> &PaperCandidates){
  vector<cv::Point> hull;
  //calculate the  min_max contour sizes
  //int minSize = minContourValue * std::max(currentFrame.cols,currentFrame.rows) * 4;
  //int maxSize = maxContourValue * std::max(currentFrame.cols,currentFrame.rows) * 4;
  
  for (size_t i = 0; i < contours.size(); i++){
    //drawContour(debug, contours[i], CV_RGB(0,255,0));
    //    if(minSize < contours[i].size()  && contours[i].size() < maxSize){
    if (cv::contourArea(contours[i]) > contourAreaValue){
      cv::convexHull(contours[i],hull);
      cv::approxPolyDP(hull, hull, double(0.1*arcLength(hull,true)),true);
      drawContour(debug, hull, CV_RGB(255,0,0));
      if(hull.size() == 4){
	drawApproxCurve(debug, hull, CV_RGB(0,0,255));   //blue
	PaperCandidates.push_back(PaperCandidate());
	PaperCandidates.back().idx = i;
	for (int j=0; j<4; j++ ){
	  PaperCandidates.back().push_back(Point2f(hull[j].x,hull[j].y));
	}
	//cout<<"ContourAdded"<<endl;
      }
    }
  }
}

void PaperDetector::findRectangles(vector<vector<cv::Point> > &contours, vector<PaperCandidate> &PaperCandidates){
  vector<cv::Point>  approxCurve;
  //calculate the  min_max contour sizes
  int minSize = minContourValue * 640 * 4;
  int maxSize = maxContourValue * 640 * 4;
  
  for(unsigned int i=0; i<contours.size(); i++){
    if(minSize < (int)contours[i].size() && (int)contours[i].size() < maxSize){
      //approximate to a poligon
      cv::approxPolyDP(contours[i],approxCurve, double(contours[i].size())*0.02, true);
      //drawApproxCurve(debug, approxCurve, Scalar(255,0,255));
      //check that the poligon has 4 points
      if (approxCurve.size() == 4){
	//drawContour(debug,contours[i],Scalar(255,0,225));
	//and is convex
	if (cv::isContourConvex(Mat(approxCurve))){
	  //drawApproxCurve(debug,approxCurve,Scalar(255,0,255));
	  //ensure that the   distace between consecutive points is large enough
	  float minDist = 1e10;
	  for (int j=0; j<4; j++){
	    //float d = std::sqrt((float)(approxCurve[j].x-approxCurve[(j+1)%4].x) * 
	    //(approxCurve[j].x-approxCurve[(j+1)%4].x) +
	    //			  (approxCurve[j].y-approxCurve[(j+1)%4].y) *
	    //			  (approxCurve[j].y-approxCurve[(j+1)%4].y));
	    //norm(Mat(approxCurve[i]),Mat(approxCurve[(i+1)%4]));
	    //if (d < minDist) minDist = d;
	    //---------------------
	    cv::Point side = approxCurve[i] - approxCurve[(i+1)%4];
	    float squaredSideLength = side.dot(side);
	    minDist = std::min(minDist, squaredSideLength);
	  }
	  //check that distance is not very small
	  if (minDist > 10){
	    //add the points
	    //cout<<"ADDED"<<endl;
	    drawApproxCurve(debug,approxCurve,CV_RGB(243,249,21));  // yellow
	    
	    PaperCandidates.push_back ( PaperCandidate() );
	    PaperCandidates.back().idx = i;
	    for ( int j=0;j<4;j++ ){
	      PaperCandidates.back().push_back ( Point2f ( approxCurve[j].x,approxCurve[j].y ) );
	    }
	  }
	  
	  double maxCosine = 0;
	  for(int j = 2; j < 5; j++){
	    double cosine = fabs(angle(approxCurve[j%4], approxCurve[j-2], approxCurve[j-1]));
	    maxCosine = std::max(maxCosine, cosine);
	  }
	  
	  if(maxCosine < 0.3 )
	    drawApproxCurve(debug,approxCurve,CV_RGB(255,0,0));   //red
	}
      }
    }
  }
}

void PaperDetector::thresHold(int method, const Mat &grey, Mat &out, double param1, double param2) throw (cv::Exception){
  if (param1 == -1) param1 = thresParam1;
  if (param2 == -1) param2 = thresParam2;
  
  if (grey.type() != CV_8UC1)     
    throw cv::Exception (9001,"grey.type() != CV_8UC1", "thresHold", __FILE__, __LINE__);
  
  switch (method){
  case FIXED_THRES:
    cv::threshold (grey, out, param1, 255, CV_THRESH_BINARY_INV);
    break;
  case ADPT_THRES://currently, this is the best method
    //ensure that _thresParam1%2==1
    if (param1 < 3) param1 = 3;
    else if (((int) param1) %2 != 1) param1 = (int)(param1 + 1);
    cv::adaptiveThreshold(grey, out, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, param1, param2);
    break;
  case CANNY:
    cv::Canny (grey, out, param1, param2);
    break;
  }
}

/************************************
 *
 *
 *
 *
 ************************************/
void PaperDetector::drawAllContours ( Mat input, std::vector<std::vector<cv::Point> > &contours ){
  drawContours ( input,  contours, -1,Scalar ( 255,0,255 ) );
}

/************************************
 *
 *
 *
 *
 ************************************/
void PaperDetector:: drawContour ( Mat &in,vector<Point>  &contour,Scalar color ){
  for ( unsigned int i=0;i<contour.size();i++ ){
    cv::rectangle ( in,contour[i],contour[i],color );
  }
}

void  PaperDetector:: drawApproxCurve ( Mat &in,vector<Point>  &contour,Scalar color ){
  for ( unsigned int i=0;i<contour.size();i++ ){
    cv::line ( in,contour[i],contour[ ( i+1 ) %contour.size() ],color );
  }
}
/************************************
 *
 *
 *
 *
 ************************************/

void PaperDetector::draw ( Mat out,const vector<Paper> &markers ){
  for ( unsigned int i=0;i<markers.size();i++ ){
    cv::line ( out,markers[i][0],markers[i][1],cvScalar ( 255,0,0 ),2,CV_AA );
    cv::line ( out,markers[i][1],markers[i][2],cvScalar ( 255,0,0 ),2,CV_AA );
    cv::line ( out,markers[i][2],markers[i][3],cvScalar ( 255,0,0 ),2,CV_AA );
    cv::line ( out,markers[i][3],markers[i][0],cvScalar ( 255,0,0 ),2,CV_AA );
  }
}

void PaperDetector::setMinMaxSize(float min ,float max )throw(cv::Exception){
  if (min<=0 || min>1) throw cv::Exception(1," min parameter out of range","PaperDetector::setMinMaxSize",__FILE__,__LINE__);
  if (max<=0 || max>1) throw cv::Exception(1," max parameter out of range","PaperDetector::setMinMaxSize",__FILE__,__LINE__);
  if (min>max) throw cv::Exception(1," min>max","PaperDetector::setMinMaxSize",__FILE__,__LINE__);
  minContourValue = min;
  maxContourValue = max; 
}

double PaperDetector::angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 ) {
  double dx1 = pt1.x - pt0.x;
  double dy1 = pt1.y - pt0.y;
  double dx2 = pt2.x - pt0.x;
  double dy2 = pt2.y - pt0.y;
  return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}


int PaperDetector::perimeter(vector<Point2f> &a) {
  int sum=0;
  for (unsigned int i=0; i<a.size(); i++){
    int i2 = (i+1) % a.size();
    sum+= sqrt ( ( a[i].x-a[i2].x ) * ( a[i].x-a[i2].x ) + ( a[i].y-a[i2].y ) * ( a[i].y-a[i2].y ) ) ;
  }
  return sum;
}
