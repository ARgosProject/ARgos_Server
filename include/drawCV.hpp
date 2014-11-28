#ifndef _argosToolkit_DrawUtils_H_
#define _argosToolkit_DrawUtils_H_

#include <opencv2/opencv.hpp>
#include <cstdio>
#include <iostream>
#include "paper.hpp"
#include "CameraProjectorSystem.hpp"
/**\brief A set of functions to draw in opencv images
 */

class drawCV{
  
public:
  
  static void draw3dAxis(cv::Mat& image, Paper& p, CameraProjectorSystem& cameraProjector);
  static void draw3dCube(cv::Mat& image, Paper& p, CameraProjectorSystem& cameraProjector);
  static void draw3dPaper(cv::Mat& image, Paper& paper, CameraProjectorSystem& cameraProjector);
  static void projectPaper(cv::Mat& image, Paper& paper, CameraProjectorSystem& cameraProjector, cv::Mat& video,  cv::Mat& out);
  static void projectPaper0(cv::Mat& image, Paper& paper, CameraProjectorSystem& cameraProjector, int index);
  static void cameraPaper(cv::Mat& image, Paper& paper, CameraProjectorSystem& cameraProjector, const string& text);
  static void drawBoardMarks(cv::Mat& image, int thickness, int size);
  static void drawInvoice(cv::Mat& image, Paper& paper, CameraProjectorSystem& cameraProjector);

private:
  static void drawContour(cv::Mat &in, vector<cv::Point2f>& contour, cv::Scalar color);
};
#endif

