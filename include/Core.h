/**
   @file Core.h
   @brief Server side logic (main)
   @author Manuel Hervas
   @date 06/2014
*/
#ifndef SERVER_CORE_H_
#define SERVER_CORE_H_

// OpenCV library
#include <opencv2/opencv.hpp>
// Detection stuff
#include "PaperDetector.h"
#include "Paper.h"
#include "CameraProjectorSystem.h"
#include "DrawCV.h"
//#include "OpticalFlow.hpp"
#include "DocumentDetector.h"

// Singleton
#include "Singleton.h"

namespace argosServer {
  /**
   * Server side main logic
   */
  class Core: public Singleton<Core> {

  public:
    /**
     * Default Constructor
     */
    Core();

    /**
     * Default Destructor
     */
    virtual ~Core();

    /**
     * Finds and indentify papers and then return a cv::Mat to send
     */
    //vector<Paper> processCvMat(cv::Mat&);

    /**
     * Finds and indentify papers and then return a vector of Papers
     */
    vector<Paper>&  update(cv::Mat&, bool&);



    void checkRegion(cv::Mat& currentFrame,vector<Paper>& detectedPapers);
    bool warp (const cv::Mat& in, cv::Mat& out, Size size, vector<Point2f> points ) throw ( cv::Exception );

    cv::Mat calculate3DPointFrom2D(const cv::Point& ps, const cv::Mat& Rs, const cv::Mat& ts, const cv::Mat& cameraMatrix);

  private:
    cv::Mat currentFrame;                 ///<  Current frame
    cv::Mat projectorFrame;               ///<  Projector openCV frame
    cv::Mat background;                     ///<  Background and logos
    CameraProjectorSystem cameraProjector;  ///<  Intrinsics and extrinsics camera-projector parameters

    vector<Paper> paperList;                ///<  Vector of detected papers
    Size paperSize;                         ///<  Paper size in cm


    cv::Mat video;                          ///<  Video image test
    cv::Mat bg_out;                         ///<  Background image test
    cv::Mat M;                              ///<  Background image test

    vector<string> invoices;
    vector<int> invoicesIndex;
    string lastSearch;
    bool isPreviousPaperDetected;

    int numFrames;
    int numInvoices;
    int previousNumInvoices;

    bool initVideoConference;
    vector<cv::Point> projectionLimits;
  };
}

#endif
