#ifndef _ARGOS_CAMERAPROJECTORSYSTEM_H
#define _ARGOS_CAMERAPROJECTORSYSTEM_H

#include <opencv2/opencv.hpp>
#include "CameraModel.hpp"

using namespace std;

class CameraProjectorSystem{
  
public:
  CameraProjectorSystem();

  void load(string cameraConfig = "calibrationCamera.yml",
	    string projectorConfig  = "calibrationProjector.yml",
	    string extrinsicsConfig = "CameraProjectorExtrinsics.yml");
  
  void saveExtrinsics(string filename) const;
  void loadExtrinsics(string filename);
  
  bool isValid() const;
 
  vector<cv::Point2f> getProjected(const vector<cv::Point3f> & objectPoints,
				   const cv::Mat & rotObjToCam = cv::Mat::zeros(3, 1, CV_64F),
				   const cv::Mat & transObjToCam = cv::Mat::zeros(3, 1, CV_64F));
  
  CameraModel& getCamera() { return camera; }
  CameraModel& getProjector() {return projector; }
  
  const cv::Mat& getCamToProjRotation() {return rotCamToProj;}
  const cv::Mat& getCamToProjTranslation() {return transCamToProj;}
  
protected:
  
  CameraModel camera;
  CameraModel projector;
  
  cv::Mat rotCamToProj;
  cv::Mat transCamToProj;
};

#endif
