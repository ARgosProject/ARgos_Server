#ifndef _ARGOS_PAPERDETECTOR_H
#define _ARGOS_PAPERDETECTOR_H
#include "paper.hpp"
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <iostream>

using namespace std;

//Represent a candidate to be a sheet of paper
class PaperCandidate:public Paper{
public:
  vector<cv::Point> contour;  //all the points of its contour
  int idx;                    //index position in the global contour list
  
  PaperCandidate(){}
  
  PaperCandidate(const Paper &M): Paper(M){} 
  
  PaperCandidate(const  PaperCandidate &M): Paper(M){
    contour = M.contour;
    idx = M.idx;
  }

  PaperCandidate(cv::Size S): Paper(S){}
  
  //PaperCandidate& operator=(const PaperCandidate& P){
  //  (*(Paper*)this)=(*(Paper*)&P);
  //  contour = P.contour;
  //  idx = P.idx;
  //}

};

/**
 * \brief Main class for paper detection
 */
class PaperDetector{
  
 public:
  
  /**
   * Default constructor 
   */
  PaperDetector();
  
  /**
   * Default destructor
   */
  ~PaperDetector();
  
  /**
   * Detects the sheet of papers in the image passed
   *
   * If you provide information about the camera parameters and the size of the marker, then, the extrinsics of the markers are detected
   *
   * @param currentFrame input color image
   * @param detectedPapers output vector with the paper detected
   */
  
  void detect (const cv::Mat& currentFrame, std::vector<Paper>& detectedPapers, CameraProjectorSystem& camProjector,
	       cv::Size paperSizeMeters, bool setYPerperdicular, bool screenExtrinsics) throw (cv::Exception);
  
  /**
   *  This set the type of thresholding methods available
   */
  enum ThresholdMethods {FIXED_THRES,ADPT_THRES,CANNY};
  
  /**
   * Sets the threshold method
   */
  void setThresholdMethod(ThresholdMethods m) {
    thresMethod = m;
  }
  
  /**
   * Returns the current threshold method
   */
  ThresholdMethods getThresholdMethod()const {
    return thresMethod;
  }
  /**
   * Set the parameters of the threshold method
   * We are currently using the Adptive threshold ee opencv doc of adaptiveThreshold for more info
   *   @param param1: blockSize of the pixel neighborhood that is used to calculate a threshold value for the pixel
   *   @param param2: The constant subtracted from the mean or weighted mean
   */
  void setThresholdParams(double param1,double param2) {
    thresParam1 = param1;
    thresParam2 = param2;
  }
  
  /**
   * Set the parameters of the threshold method
   * We are currently using the Adptive threshold ee opencv doc of adaptiveThreshold for more info
   *   @param param1: blockSize of the pixel neighborhood that is used to calculate a threshold value for the pixel
   *   @param param2: The constant subtracted from the mean or weighted mean
   */
  void getThresholdParams(double &param1,double &param2)const {
    param1 = thresParam1;
    param2 = thresParam2;
  }
  
  
  /**
   * Returns a reference to the internal image thresholded. It is for visualization purposes and to adjust manually
   * the parameters
   */
  const cv::Mat  & getThresFrame(){
    return outThres;
  }
  
  const cv::Mat &  getDebugFrame(){
    return debug;
  }

  const cv::Mat & getWarpVector(){
    return warpVector[0];
  }
  
  /**
   * Specifies the min and max sizes of the markers as a fraction of the image size. By size we mean the maximum
   * of cols and rows.
   * @param min size of the contour to consider a possible marker as valid (0,1]
   * @param max size of the contour to consider a possible marker as valid [0,1)
   */
  void setMinMaxSize(float min=0.03,float max=0.5)throw(cv::Exception);
  
  void setContourArea(float area=5000.0)throw(cv::Exception);
    
  /**
   * Reads the min and max sizes employed
   * @param min output size of the contour to consider a possible marker as valid (0,1]
   * @param max output size of the contour to consider a possible marker as valid [0,1)
   */
  void getMinMaxSize(float &min,float &max){min=minContourValue; max=maxContourValue;}

  void getContourArea(float &area){area=contourAreaValue;}
  
  /**
   * Thesholds the passed image with the specified method.
   */
  void thresHold(int method, const Mat &grey, Mat &out, double param1, double param2) throw (cv::Exception);
  
  /**
   * Detection of candidates to be markers, i.e., rectangles.
   * This function returns in candidates all the rectangles found in a thresolded image
   */
  void detectRectangles(const cv::Mat &thresImg, vector<std::vector<cv::Point2f> > & candidates);
  
  /**
   * Returns a list candidates to be markers (rectangles), for which no valid id was found after calling detectRectangles
   */
  //const vector<std::vector<cv::Point2f> > &getCandidates() {
  //  return _candidates;
  //}
  
  /**
   * Refine PaperCandidate Corner using LINES method
   * @param candidate candidate to refine corners
   */
  void refineCandidateLines(PaperCandidate &candidate);    
    
  
 private:
  
  // Images
  cv::Mat currentFrame;		            // current frame
  cv::Mat greyFrame;                        // grey frame
  cv::Mat debug;                            // debug frame
  cv::Mat outThres,thres2;                  // threshold frame
  vector<cv::Mat> warpVector;               // warp Vector
  

  //----------------------- Filters------------------------------------------------------------
  // Threshold -------------------------------------------------------------------------------
  ThresholdMethods thresMethod;             //Current threshold method
  double thresParam1, thresParam2;          //Threshold parameters
  

  // ----------------------- Detection methods------------------------------------------------
  // Find Rectangles
  //vectr of candidates to be markers. This is a vector with a set of rectangles that have no valid id
  //vector<std::vector<cv::Point2f> > _candidates;

  // Convex Hull Contours
  float minContourValue, maxContourValue;   //Minimum and maximum size of a contour lenght
  float contourAreaValue;
  
  /**
   * Detection of candidates to be sheets of paper, i.e., rectangles.
   * This function returns in candidates all the rectangles found in a thresolded image
   */
  void findRectangles(vector<vector<cv::Point> > &contours, vector<PaperCandidate> &PaperCandidates);
  void convexHullContours(vector<vector<cv::Point> > &contours, vector<PaperCandidate> &PaperCandidates);



  //-Utils-----------------------------------------------------------------------------------------
  
  /**
   */
  bool isInto(cv::Mat &contour,vector<cv::Point2f> &b);
  
  /**
   */
  int perimeter(vector<cv::Point2f> &a);

  double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);

  bool warp(cv::Mat &in, cv::Mat &out, cv::Size size, vector<Point2f> points) throw ( cv::Exception );
  /**
   * Given a vector vinout with elements and a boolean vector indicating the lements from it to remove, 
   * this function remove the elements
   * @param vinout
   * @param toRemove
   */
  template<typename T>
    void removeElements(vector<T> & vinout,const vector<bool> &toRemove){
    //remove the invalid ones by setting the valid in the positions left by the invalids
    size_t indexValid=0;
    for (size_t i=0;i<toRemove.size();i++) {
      if (!toRemove[i]) {
	if (indexValid!=i) vinout[indexValid]=vinout[i];
	indexValid++;
      }
    }
    vinout.resize(indexValid);
  }
  
  //-Graphical debug
  void drawApproxCurve(cv::Mat &in,vector<Point> &contour,cv::Scalar color);
  void drawContour(cv::Mat &in,vector<Point>  &contour,cv::Scalar color);
  void drawAllContours(cv::Mat input, vector<vector<cv::Point> > &contours);
  void draw(cv::Mat out,const vector<Paper> &markers );
  
};

  
#endif
