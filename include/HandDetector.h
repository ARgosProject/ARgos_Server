/**
   @file HandDetector.h
   @brief
   @author Manuel Hervas
   @date 07/2014
*/
#ifndef _ARGOS_HANDDETECTOR_H_
#define _ARGOS_HANDDETECTOR_H_

#include <opencv2/opencv.hpp>
#include <iostream>
// Singleton
#include "Singleton.h"

#define SAMPLES 7

using namespace std;

/**
 * ARgos server side
 */
namespace argosServer{

  class HandDetector: public Singleton<HandDetector> {

  public:
    /**
     * Default constructor
     */
    HandDetector();

    /**
     * Default destructor
     */
    ~HandDetector();

    void detectFinger(const cv::Mat& currentFrame, cv::Point& fingerPosition);


    cv::Mat GetSkin(cv::Mat const &src);

    bool R1(int R, int G, int B);
    bool R2(float Y, float Cr, float Cb);
    bool R3(float H, float S, float V);
    void getconvexhull(vector<cv::Point>& contour, cv::Mat& image, vector<cv::Point> &fingerTips);
    void fingertip(vector<cv::Point> &contour, cv::Mat& image, cv::Point armcenter);
    int findBiggestContour(vector<vector<cv::Point> > contours);
    void drawApproxCurve(cv::Mat &in, vector<cv::Point> &contour,cv::Scalar color);
    float distanceP2P(cv::Point a, cv::Point b);
    float getAngle(cv::Point s, cv::Point f, cv::Point e);
    //void getFingertip(vector<cv::Point> &contour, vector<cv::Vec4i> &convexityDefects, cv::Mat& image);

    int getMedian(vector<int> values);
    void getAvgColor(cv::Mat& in, cv::Rect& roi, int avg[3]);
    void normalizeColors();
    void produceBinaries(cv::Mat& image);

  private:
    cv::Mat hsvFrame;                  // HSV Input frame
    std::vector<cv::Mat> hsvChannels;  // Separate HSV channels in images
    cv::Mat imageHLS;
    cv::Mat output;
    string configWindow;
    vector<cv::Rect> roi;

    int Hlow;
    int Htop;
    int Slow;
    int Stop;
    int Vlow;
    int Vtop;

    int avgColor[SAMPLES][3];
    int c_lower[SAMPLES][3];
    int c_upper[SAMPLES][3];
  };
}

#endif
