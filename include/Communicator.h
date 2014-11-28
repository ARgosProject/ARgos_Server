#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <boost/asio.hpp>
#include <opencv2/opencv.hpp>

// Detection stuff
#include "paperdetector.hpp"
#include "paper.hpp"
#include "CameraProjectorSystem.hpp"
#include "drawCV.hpp"
//#include "opticalFlow.hpp"
#include "documentDetector.hpp"

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
using boost::asio::ip::udp;

class Communicator {
public:
  enum Type {
    VECTOR_I      =  0,
    MATRIX_16F    =  1,
    CV_MAT        =  2,
    VIDEO_STREAM  =  3,
    PAPER         =  4,
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

  void startVideoStream(StreamType st);
  size_t sendCvMatVideoStream(const cv::Mat& mat, udp::socket& udpSocket, const udp::endpoint& udpEndpoint);

  int send() const;
  void addInt(int val);
  void addMatrix16f(const float* matrix);
  void addVectori(const std::vector<int>& vector);
  void addCvMat(const cv::Mat& mat);
  void addPaper(Paper& paper);
  void addVectorCvMat(const std::vector<cv::Mat>& mats);
  void addVectorPapers(std::vector<Paper>& papers);

  void drawChessBoard(cv::Mat& currentFrame, cv::Mat& projectorFrame, CameraProjectorSystem& cameraProjector);

private:
  boost::asio::io_service _ioService;
  tcp::socket* _tcpSocket;
  unsigned short _port;
  std::vector<unsigned char> _buff;

  std::shared_ptr<std::thread> _videoThread;
  std::mutex _mutex;
  std::atomic<bool> _threadDone;
  std::condition_variable _condReceive;
  bool _receive;

  // I/O Mat
private:
  cv::Mat currentFrame;		// current frame
  cv::Mat projectorFrame;	// projector openCV frame
  cv::Mat background;           // background and logos
  cv::Mat video;           // background and logos
  CameraProjectorSystem cameraProjector;
  cv::Mat bg_out;
  cv::Mat M;
  // Register and tracking stuff
private:
  PaperDetector paperDetector;
  vector<Paper> paperList;

  Size paperSize;

  //Optical Flow
  //OpticalFlow of;

  // Document detector stuff
private:
  DocumentDetector* documentDetector;
  vector<string> invoices;
  vector<int> invoicesIndex;
  string lastSearch;
  bool isPreviousPaperDetected;

  int numFrames;
  int numInvoices;
  int previousNumInvoices;

  bool initVideoConference;

};

#endif
