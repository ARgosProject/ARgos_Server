#ifndef _ARGOS_CAMERAMODEL_H
#define _ARGOS_CAMERAMODEL_H

#include <opencv2/opencv.hpp>

using namespace std;

/*
 * Class Intrinsics
 * 
 */ 
class Intrinsics{
public:
  Intrinsics();
  Intrinsics(cv::Mat cameraMatrix, cv::Size imageSize);
  Intrinsics(const Intrinsics& intrinsics);
  
  void setup(cv::Mat cameraMatrix, cv::Size imageSize);  
  void setup(cv::Mat cameraMatrix, cv::Size imageSize, cv::Size sensorSize);  
 
  void setImageSize(cv::Size imgSize);
  
  cv::Mat getCameraMatrix() const;
  cv::Size getImageSize() const;
  cv::Size getSensorSize() const;
  cv::Point2d getFov() const;
  double getFocalLength() const;
  double getAspectRatio() const;
  cv::Point2d getPrincipalPoint() const;

  void resize(cv::Size size)throw(cv::Exception);
  //void loadProjectionMatrix(float nearDist = 10., float farDist = 10000., cv::Point2d viewportOffset = cv::Point2d(0, 0)) const;
  
  bool isValid() const {
    return cameraMatrix.rows != 0 && cameraMatrix.cols != 0  && imageSize.width != -1 && imageSize.height != -1;
  }

private:
  cv::Mat cameraMatrix;             // 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1) 
  cv::Size imageSize;               // size of the image
  cv::Size sensorSize;               // size of the image
  cv::Point2d fov;                  // field of view
  double focalLength, aspectRatio;
  cv::Point2d principalPoint;
};

/*
* Class CameraModel
* 
*/ 
class CameraModel{
  
public:
  /** Empty constructor
   */
  CameraModel();
  
  /**Creates the object from the info passed
   * @param cameraMatrix 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
   * @param distCoeffs 4x1 matrix (k1,k2,p1,p2)
   * @param imageSize image size
   */
  CameraModel(cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Size imageSize);

  /**
   * Copy constructor
   */
  CameraModel(const CameraModel& camera);

  /**
   * Indicates whether this object is valid
   */
  bool isValid() const {
    return distortedIntrinsics.isValid() && undistortedIntrinsics.isValid() && distCoeffs.rows !=0 && distCoeffs.cols != 0;
  }
  
  /*
   *Assign operator
   */
  //CameraModel& operator=(const CameraModel &camera);
   
  cv::Mat getDistCoeffs() const;
  
  void setIntrinsics(Intrinsics& distortedIntrinsics, cv::Mat& distortionCoefficients);
  
  const Intrinsics& getDistortedIntrinsics() const;
  const Intrinsics& getUndistortedIntrinsics() const;
  

  cv::Mat getDistortedCamMatrix() const{
    return distortedIntrinsics.getCameraMatrix();
  }
  cv::Mat getUndistortedCamMatrix() const{
    return undistortedIntrinsics.getCameraMatrix();
  }
  //cv::Size getImageSize() const;
  //cv::Size getSensorSize() const;
  //cv::Point2d getFov() const;
  //double getFocalLength() const;
  //double getAspectRatio() const;
  //cv::Point2d getPrincipalPoint() const;

  /**
   * getTransformation
   * 
   */
  //bool getTransformation(Calibration& dst, cv::Mat& rotation, cv::Mat& translation);
  
  /**
   * loadProjectionMatrix
   * 
   */
  //void loadProjectionMatrix(float nearDist = 10., float farDist = 10000., cv::Point2d viewportOffset = cv::Point2d(0, 0)) const;
  
  /**
   * Returns the location of the camera in the reference system given by the rotation and translation vectors passed
   * NOT TESTED
   */
  static cv::Point3f getCameraLocation(cv::Mat Rvec,cv::Mat Tvec);
  
  /**
   * Given the intrinsic camera parameters returns the GL_PROJECTION matrix for opengl.
   * Please NOTE that when using OpenGL, it is assumed no camera distorsion! So, if it is not true, you should have undistor image
   *
   * @param orgImgSize size of the original image
   * @param size of the image/window where to render (can be different from the real camera image). Please not that it must be related to CamMatrix
   * @param proj_matrix output projection matrix to give to opengl
   * @param gnear,gfar: visible rendering range
   * @param invert: indicates if the output projection matrix has to yield a horizontally inverted image because image data has not been stored in the order of glDrawPixels: bottom-to-top.
   */
  void glGetProjectionMatrix( cv::Size orgImgSize, cv::Size size,float proj_matrix[16],float gnear,float gfar,bool invert=false) throw (cv::Exception);
    
  /**
   * setup camera for an Ogre project.
   * 	Use:
   * ...
   * Ogre::Matrix4 PM(proj_matrix[0], proj_matrix[1], ... , proj_matrix[15]);
   * yourCamera->setCustomProjectionMatrix(true, PM);
   * yourCamera->setCustomViewMatrix(true, Ogre::Matrix4::IDENTITY);
   * ...
   * As in OpenGL, it assumes no camera distorsion
   */
  void OgreGetProjectionMatrix(cv::Size orgImgSize, cv::Size size, float proj_matrix[16], float gnear, float gfar, bool invert=false) throw (cv::Exception);
    
  /**
   * Saves this to a file
   */
  void saveFile(string filename) const throw(cv::Exception);
  
  /**
   * Reads from a XML/YAML file generated with the camera model
   */
  void loadFile(string filename) throw(cv::Exception); 

  void glGetProjectionMatrix2(float x0,float y0,float width,float height,float znear,float zfar, float proj_matrix[16]);
  
  
private:
  Intrinsics distortedIntrinsics;   // Principal parameters
  Intrinsics undistortedIntrinsics; // Undistorted intrinsics parameters for OPENGL
  cv::Mat distCoeffs;               // 4x1 matrix distorsion coefficients (k1,k2,p1,p2)
  
  cv::Mat __undistortMapX, __undistortMapY;

  void argConvGLcpara2( float cparam[3][4], int width, int height, float gnear, float gfar, float m[16], bool invert)throw(cv::Exception);
  int arParamDecompMat( float source[3][4], float cpara[3][4], float trans[3][4] )throw(cv::Exception);
  float norm( float a, float b, float c );
  float dot( float a1, float a2, float a3, float b1, float b2, float b3 );
  
  void undistort(cv::Mat& img, int interpolationMode = cv::INTER_NEAREST);
  void undistort(const cv::Mat& src, cv::Mat& dst, int interpolationMode = cv::INTER_NEAREST);
  void updateUndistortion();



};
#endif
