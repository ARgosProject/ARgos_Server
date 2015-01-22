#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <boost/asio.hpp>
#include <opencv2/opencv.hpp>
#include "Paper.h"
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

namespace argosServer{
  /**
   * A class used to communicate with the client It provides bidirectional communication with the client, allowing to receive and send some data structures
   */
  class Communicator {
  public:
    enum Type {
      SKIP          = -1,

      VECTOR_I      =  0,
      MATRIX_16F    =  1,
      CV_MAT        =  2,
      PAPER         =  3,
      VIDEO_STREAM  =  4
    };

  public:
    struct StreamType {
      Type type;
      int size;
      std::vector<unsigned char> data;
    };

  public:
    explicit Communicator(unsigned short port, const char* iface);
    ~Communicator();

    void waitForConnections();
    std::string getIpFromInterface(const std::string& iface);

    void readStreamTypeFromSocket(tcp::socket &socket, StreamType &st);
    void receive();
    void processMatrix16f(StreamType& st);
    void processVectori(StreamType& st);
    void processCvMat(StreamType& st);

    void startVideoStream(StreamType st);
    size_t sendCvMatVideoStream(const cv::Mat& mat, udp::socket& udpSocket, const udp::endpoint& udpEndpoint);

    int send() const;
    void addInt(int val);
    void addMatrix16f(const float* matrix);
    void addVectori(const std::vector<int>& vector);

    void addSkip();
    void addCvMat(const cv::Mat& mat);
    void addPaper(Paper& paper);
    void addVectorCvMat(const std::vector<cv::Mat>& mats);
    void addVectorPapers(std::vector<Paper>& papers);


  private:
    boost::asio::io_service _ioService;
    tcp::socket* _tcpSocket;
    unsigned short _port;
    std::string _iface;
    std::vector<unsigned char> _buff;

    std::shared_ptr<std::thread> _videoThread;
    std::mutex _mutex;
    std::atomic<bool> _threadDone;
    std::condition_variable _condReceive;
    bool _receive;

    // I/O Mat
  private:
    cv::Mat currentFrame;		// current frame
    //cv::Mat projectorFrame;	// projector openCV frame
    vector<Paper> paperList;
    bool initVideoConference;
    //vector<int> invoicesIndex;
  };
}
#endif
