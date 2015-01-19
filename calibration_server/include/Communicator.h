#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include <boost/asio.hpp>
#include <opencv2/opencv.hpp>
#include "CameraProjectorCalibration.hpp"

/**
 * Protocolo de recepción de datos:
 *
 *   Tipo  Tamaño     Datos
 * +-------------------------------
 * | int  | int  | unsigned char* |
 * +-------------------------------
 */


using namespace std;
using namespace cv;

using boost::asio::ip::tcp;

class Communicator {
public:
  enum Type {
    VECTOR_I   = 0,
    MATRIX_16F = 1,
    CV_MAT     = 2,
  };
  
public:
  struct StreamType {
    Type type;
    int size;
    std::vector<unsigned char> data;
  };
  
public:
  explicit Communicator(unsigned short port);
  ~Communicator();
  
  void waitForConnections();
  
  void receive();
  void proccessMatrix16f(StreamType& st);
  void proccessVectori(StreamType& st);
  void proccessCvMat(StreamType& st);
  
  int send() const;
  void addMatrix16f(const float* matrix);
  void addVectori(const std::vector<int>& vector);
  void addCvMat(const cv::Mat& mat);
  void addVectorCvMat(const std::vector<cv::Mat>& mats);

 
private:
  boost::asio::io_service _ioService;
  tcp::socket* _tcpSocket;
  unsigned short _port;
  std::vector<unsigned char> _buff;

  
protected:
  
  // calibration status
  enum CalibState {CAMERA, PROJECTOR_STATIC, PROJECTOR_DYNAMIC, DEMO_AR};
  CalibState currState;
  
  // Extrinsics parameters CAMERA-PROJECTOR
  CameraProjectorCalibration camProjCalib;
  
  cv::Mat cameraFrame;        // Camera input
  cv::Mat projectorFrame;      //Projector output
  cv::Mat debugFrame;          //debug Frame
  cv::Mat processedImg;
  cv::Mat snapShot;
  
  bool bProjectorRefreshLock;
  // Threshold parameters
  int circleDetectionThreshold;

  // Application
  float diffMinBetweenFrames;
  float timeMinBetweenCaptures;
  string currStateString;

  //Boards parameters
  int numBoardsFinalCamera;
  int numBoardsFinalProjector;
  int numBoardsBeforeCleaning;
  int numBoardsBeforeDynamicProjection;
  float maxReprojErrorCamera;
  float maxReprojErrorProjectorStatic;
  float maxReprojErrorProjectorDynamic;
  
  // board holding movement
  
  cv::Mat previous;
  cv::Mat diff;
  float diffMean;
  double lastTime;
  
  //---------------------------------------------------------------------------------
  void update();
  void draw();
  void initializeCamera(int dev);
  void setState(CalibState state);
  string getCurrentStateString();
  bool calibrateCamera(const cv::Mat& cameraFrame);
  bool calibrateProjector(const cv::Mat& cameraFrame);
  void processImageForCircleDetection(const cv::Mat& cameraFrame);
  bool timeMinBetweenFrames();
  void drawCameraImagePoints();
  void drawProjectorPattern();
  void drawProjectorFinePattern();
  void toGreyscale(const cv::Mat& inputFrame, cv::Mat& cameraFrame);
  bool calculateDemo(cv::Mat& cameraFrame);
  //--------------------------------------------------------------------------------
  
};

#endif
