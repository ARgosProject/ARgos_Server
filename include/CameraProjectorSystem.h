/**
   @file CameraProjetorSystem.h
   @brief Defines the camera-projector system and transformation matrix of them 
   @author Manuel Hervas
   @date 06/2014
*/
#ifndef _ARGOS_CAMERAPROJECTORSYSTEM_H
#define _ARGOS_CAMERAPROJECTORSYSTEM_H

#include <opencv2/opencv.hpp>
#include "CameraModel.h"

using namespace std;

namespace argosServer{
  /**
   *  Defines the camera-projector system and transformation matrix of them 
   */

  class CameraProjectorSystem{
  
  public:

    /**
     * Default constructor
     */
    CameraProjectorSystem();

    /**
     * Reads intrinsics and extrinsics parameters files 
     * @param cameraConfig Camera intrinsics parameters 
     * @param projectorConfig Projector intrinsics parameters 
     * @param extrinsicsConfig Transformation matrix camera-projector (Extrinsics parameters camera-projector)
     */
    void load(string cameraConfig = "calibrationCamera.yml",
	      string projectorConfig  = "calibrationProjector.yml",
	      string extrinsicsConfig = "CameraProjectorExtrinsics.yml");
    /**
     * Writes extrinsics parameters to a file
     * @param filename Path and name file
     */ 
    void saveExtrinsics(string filename) const;

    /**
     * Reads extrinsics parameters from a file
     * @param filename Path and name file
     */ 
    void loadExtrinsics(string filename);
    
    /**
     * Checks whether the object has valid parameters
     */ 
    bool isValid() const;
    
    /**
     * Combines rotation-and-shift transformations object-camera-projector and computes projections of 3D points to the image plane 
     * @param objectPoints Points of the object
     * @param rotObjToCam  Rotation transformation object-camera
     * @param transObjToCam Shift transformation object-camera
     */ 
    vector<cv::Point2f> getProjected(const vector<cv::Point3f> & objectPoints,
				     const cv::Mat & rotObjToCam = cv::Mat::zeros(3, 1, CV_64F),
				     const cv::Mat & transObjToCam = cv::Mat::zeros(3, 1, CV_64F));
  
    /**
     * Returns a reference to camera 
     */
    CameraModel& getCamera() { return camera; }

    /**
     * Returns a reference to projector 
     */
    CameraModel& getProjector() {return projector; }
    
    /**
     * Returns rotation transformation camera-projector
     */
    const cv::Mat& getCamToProjRotation() {return rotCamToProj;}
    
    /**
     * Returns shift transformation camera-projector
     */
    const cv::Mat& getCamToProjTranslation() {return transCamToProj;}
  
  protected:
  
    CameraModel camera;        ///< Camera intrinsics parameters 
    CameraModel projector;     ///< Projector intrinsics parameters 
  
    cv::Mat rotCamToProj;      ///< Rotation transformation camera-projector
    cv::Mat transCamToProj;    ///< Shift transformation camera-projector
  };
}
#endif
