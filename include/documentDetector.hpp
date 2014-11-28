#ifndef _ARGOS_DOCUMENTDETECTOR_H
#define _ARGOS_DOCUMENTDETECTOR_H
#include "paper.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp> 
#include <cstdio>
#include <iostream>

using namespace std;

class DocumentDetector{
  
public:
  
  /**
   * Default constructor 
   */
  DocumentDetector();
  
  /**
   * Default destructor
   */
  ~DocumentDetector();
  
  void detect(const cv::Mat& trainFrame, vector<Paper>& detectedPapers);
  cv::Mat& getPaperContent(){
    return paperContent;
  }
  
private:
 
  //Create Feature Algorithm  
  cv::Ptr<FeatureDetector> featureDetector;             
  cv::Ptr<DescriptorExtractor> descriptorExtractor;
  cv::Ptr<DescriptorMatcher> descriptorMatcher;
  
  // Read query Images
  vector<cv::Mat> queryImages;
  vector<string> queryImagesNames;
  
  
  //Extracting keypoints from query images
  vector<vector<KeyPoint> > queryKeypoints;
  
  //Compute descriptor from query images
  vector<cv::Mat> queryDescriptors;
  
  
  vector<cv::KeyPoint> trainKeypoints;
  cv::Mat trainDescriptors;
  
  //vector<DMatch> matches;
  vector<vector<DMatch > > matches;
  
  cv::Mat paperContent;

  
  bool createDetectorDescriptorMatcher();

  void readQueryFilenames(const string& filename, string& dirName, vector<string>& queryFilenames);

  bool readImages( const string& queryFilename, vector <Mat>& queryImages, vector<string>& queryImageNames );

  bool warp(const cv::Mat& in, cv::Mat& out, Size size, vector<Point2f> points ) throw ( cv::Exception );



};


#endif
