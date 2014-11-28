#include "CameraProjectorSystem.hpp"

CameraProjectorSystem::CameraProjectorSystem(){
  camera = CameraModel();
  projector = CameraModel();
  rotCamToProj = cv::Mat();
  transCamToProj = cv::Mat();
}

void CameraProjectorSystem::load(string cameraConfig, string projectorConfig, string extrinsicsConfig){
  camera.loadFile(cameraConfig);
  projector.loadFile(projectorConfig);
  loadExtrinsics(extrinsicsConfig);
}

void CameraProjectorSystem::saveExtrinsics(string filename) const {
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  fs << "Rotation_Vector" << rotCamToProj;
  fs << "Translation_Vector" << transCamToProj;
}

void CameraProjectorSystem::loadExtrinsics(string filename) {
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  fs["Rotation_Vector"] >> rotCamToProj;
  fs["Translation_Vector"] >> transCamToProj;
}

bool CameraProjectorSystem::isValid() const {
  return camera.isValid() && projector.isValid() && rotCamToProj.rows != 0 && rotCamToProj.cols != 0 && transCamToProj.rows !=0 && transCamToProj.cols != 0;
}

vector<cv::Point2f> CameraProjectorSystem::getProjected(const vector<cv::Point3f>& objectPoints, const cv::Mat& rotObjToCam, const cv::Mat& transObjToCam){
  
  cv::Mat rotObjToProj, transObjToProj;
  
  cv::composeRT(rotObjToCam,  transObjToCam,
		rotCamToProj, transCamToProj,
		rotObjToProj, transObjToProj);
  
  vector<cv::Point2f> out;
  cv::projectPoints(cv::Mat(objectPoints),
		rotObjToProj, transObjToProj,
		projector.getDistortedIntrinsics().getCameraMatrix(),
		projector.getDistCoeffs(),
		out);
  return out;
}
    


