#include <opencv2/opencv.hpp>
#include <iostream>
#include "ActionsManager.h"
#include "Log.h"

using namespace std;
using namespace cv;

namespace argosServer{
  
  ActionsManger::ActionsManger(){
  }

  ActionsManger::~ActionsManger(){}


  
  ActionsManger::loadFile(string filename){}
  /*
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    cv::Size imageSize, sensorSize;
    cv::Mat cameraMatrix, cameraMatrixF;
    
    fs["cameraMatrix"] >> cameraMatrix;
    fs["imageSize_width"] >> imageSize.width;
    fs["imageSize_height"] >> imageSize.height;
    fs["sensorSize_width"] >> sensorSize.width;
    fs["sensorSize_height"] >> sensorSize.height;
    fs["distCoeffs"] >> distCoeffs;
    
    if (cameraMatrix.cols == 0 || cameraMatrix.rows == 0) 
      throw cv::Exception(9007,"File :"+filename+" does not contains valid camera matrix","CameraModel::loadFile",__FILE__,__LINE__);
      
    if (imageSize.width == -1 || imageSize.height == 0) 
      throw cv::Exception(9007,"File :"+filename+" does not contains valid camera dimensions","CameraModel::loadFile",__FILE__,__LINE__);
    
    if (cameraMatrix.type()!=CV_32FC1) cameraMatrix.convertTo(cameraMatrixF,CV_32FC1);
    else cameraMatrixF = cameraMatrix;
    
    if (distCoeffs.total() < 5) 
      throw cv::Exception(9007,"File :"+filename+" does not contains valid distortion_coefficients","CameraModel::loadFile",__FILE__,__LINE__);
    
    distortedIntrinsics.setup(cameraMatrixF, imageSize, sensorSize);
    updateUndistortion();
   }    
  */
  
  
  ActionsManger:: saveFile(string filename){
    //if(!isValid())
    //  cout << "CameraModel::saveFile() failed, because camera does not contains valid parameters."<< endl;
    //else{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs << "actions" << "[";
    for( int i = 0; i < actionsList.size(); i++ ){
      fs << "id" << actionsList[i].id;
      fs << "description" << actionsList[i].description;
      fs << "region" <<actionsList[i].region;
      fs << "actionType" << actionsList[i].actionType;
      fs << "scriptName" << actionsList[i].scriptName;
    }
    fs << "]";
  }
  //}
  
  
  vector<Action> getActions(int id){}
  
}
