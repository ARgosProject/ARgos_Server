/**
   @file DocumentDetector.h
   @brief Detect the content of a paper
   @author Manuel Hervas
   @date 06/2014
*/






#ifndef _ARGOS_DOCUMENTDETECTOR_H
#define _ARGOS_DOCUMENTDETECTOR_H
#include "Paper.h"
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <cstdio>
#include <iostream>
// Singleton
#include "Singleton.h"

using namespace std;

namespace argosServer{

  /**
   * Detect the content of a paper
   */

  class DocumentDetector: public Singleton<DocumentDetector> {

  public:

    /**
     * Default constructor
     */
    DocumentDetector();

    /**
     * Default destructor
     */
    ~DocumentDetector();

    /**
     * Set the directory where images can be found
     * @param path The directory path
     */
    void setImagesPath(const string& path);

    /**
     * Identify the content of a vector of papers
     * @param trainFrame Camera frame
     * @param detectedPapers Vector of paper previously detected
     */
    void detect(const cv::Mat& trainFrame, vector<Paper>& detectedPapers);

    /**
     * Returns a image with content of paper (warpping corrected)
     */
    cv::Mat& getPaperContent(){
      return paperContent;
    }

  private:

    //Create Feature Algorithm
    cv::Ptr<FeatureDetector> featureDetector;              ///< Feature detector
    cv::Ptr<DescriptorExtractor> descriptorExtractor;      ///< Descriptor extractor
    cv::Ptr<DescriptorMatcher> descriptorMatcher;          ///< Descriptor matcher

    // Read query Images
    vector<cv::Mat> queryImages;                           ///< Vector of images to match
    vector<string> queryImagesNames;                       ///< file names with images to match

    string imagesPath;                                     ///> The directory where images can be found

    //Extracting keypoints from query images
    vector<vector<KeyPoint> > queryKeypoints;              ///< Key Points in query images

    //Compute descriptor from query images
    vector<cv::Mat> queryDescriptors;                      ///< Descriptors in query images


    vector<cv::KeyPoint> trainKeypoints;                   ///< Key Points in current frame
    cv::Mat trainDescriptors;                              ///< Descriptors in current frame

    //vector<DMatch> matches;
    vector<vector<DMatch > > matches;                      ///< vector of descriptors matchers

    cv::Mat paperContent;                                  ///< image with paper content

    /**
     * Creates a feature algorithm ( feature detector,  descriptor extractor and  descriptor matcher)
     */
    bool createDetectorDescriptorMatcher();

    /**
     * Extracs each query filename of configuration file
     */
    void readQueryFilenames(const string& filename, string& dirName, vector<string>& queryFilenames);

    /**
     * Reads query filenames
     */
    bool readImages();

    /**
     * Returns a image with content of paper (warpping corrected)
     */
    bool warp(const cv::Mat& in, cv::Mat& out, Size size, vector<Point2f> points ) throw ( cv::Exception );

  };

}
#endif
