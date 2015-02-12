#include "Communicator.h"
#include "Paper.h"
#include "Log.h"
#include "Core.h"
#include "ScriptManager.h"
#include "Script.h"
#include "ScriptSentence.h"
#include "ScriptFunction.h"
#include "ConfigManager.h"

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <ifaddrs.h>
#include <iomanip>

namespace argosServer{

  Communicator::Communicator(unsigned short port, const char* iface)
    : _port(port), _iface(iface), _threadDone(true), _receive(false), _initVideoConference(false) {
    _tcpSocket = new tcp::socket(_ioService);

    Core::getInstance();
  }

  Communicator::~Communicator() {
    if(_tcpSocket)
      delete _tcpSocket;
  }

  void Communicator::processCvMat(StreamType& st) {
    Log::success("New cv::Mat received. Size: " + std::to_string(st.size));

    //cv::imdecode(st.data, cv::IMREAD_GRAYSCALE, &currentFrame);             // Decode cv::Mat
    cv::imdecode(st.data, CV_LOAD_IMAGE_COLOR, &currentFrame);             // Decode cv::Mat
    //projectorFrame = cv::Scalar::all(0);   // Clear last output frame
    //projectorFrame = Core::getInstance().processCvMat(currentFrame);
    //cvtColor(projectorFrame,projectorFrame,CV_BGR2RGB);
    //addCvMat(projectorFrame);

    paperList = Core::getInstance().update(currentFrame, _initVideoConference);

    if(paperList.empty())
      addSkip();
    else
      addPaper(paperList.back());
  }

  void Communicator::run() {
    tcp::acceptor a(_ioService, tcp::endpoint(tcp::v4(), _port));

    while(1) {
      Log::info("Server listening at " + getIpFromInterface(_iface) + ":" + std::to_string(_port));
      a.accept(*_tcpSocket);
      Log::success("New connection from " + _tcpSocket->remote_endpoint().address().to_string());
      receive();
    }
  }

  std::string Communicator::getIpFromInterface(const std::string& iface) {
    struct ifaddrs *ifaddr, *ifa;
    int s;
    char host[NI_MAXHOST];
    std::string ip("0.0.0.0");

    if(getifaddrs(&ifaddr) == -1) {
      Log::error("getifaddrs");
      return ip;
    }

    for(ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
      if(ifa->ifa_addr == NULL)
        continue;

      s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);

      if((iface == ifa->ifa_name) && (ifa->ifa_addr->sa_family == AF_INET)) {
        if(s != 0) {
          Log::error("getnameinfo() falló: " + std::string(gai_strerror(s)));
          return ip;
        }

        ip = std::string(host);
      }
    }

    if(ip == "0.0.0.0") {
      Log::error("Network interface " + iface + " not found.");
    }

    freeifaddrs(ifaddr);

    return ip;
  }

  void Communicator::readStreamTypeFromSocket(tcp::socket &socket, StreamType &st) {
    unsigned char type_buf[sizeof(int)];
    unsigned char size_buf[sizeof(int)];
    unsigned char* data_buf;

    boost::asio::read(socket, boost::asio::buffer(&type_buf, sizeof(int)));  // Type
    memcpy(&st.type, &type_buf, sizeof(int));
    boost::asio::read(socket, boost::asio::buffer(&size_buf, sizeof(int))); // Size
    memcpy(&st.size, &size_buf, sizeof(int));

    data_buf = new unsigned char[st.size];
    boost::asio::read(socket, boost::asio::buffer(data_buf, st.size)); // Data
    st.data.insert(st.data.end(), &data_buf[0], &data_buf[st.size]);
    delete [] data_buf;
  }

  void Communicator::receive() {
    while(1) {
      try {
        StreamType st;
        readStreamTypeFromSocket(*_tcpSocket, st);

        switch(st.type) {
        case Type::VECTOR_I:
          processVectori(st);
          break;
        case Type::MATRIX_16F:
          processMatrix16f(st);
          break;
        case Type::CV_MAT:
          processCvMat(st);
          _receive = true;
          _condReceive.notify_one();
          break;
        default:
          break;
        }

        if(!_buff.empty()) {
          send();
          _buff.clear();
        }
      }
      catch(boost::system::system_error const& e) {
        Log::error("Connection lost with the client. " + std::string(e.what()));
        _tcpSocket->close();
        break;
      }
    }
  }

  void Communicator::initVideoConference(const std::string& port) {
    _initVideoConference = true;

    if(_threadDone) {
      Log::video("New videostream started.");
      _threadDone = false;
      _videoThread.reset();
      _videoThread = std::make_shared<std::thread>(&Communicator::startVideoStream, this, port);
      _videoThread->detach();
    }
  }

  void Communicator::startVideoStream(const std::string& port) {
    const std::string ip = _tcpSocket->remote_endpoint().address().to_string();
    Log::video("Sending video to " + ip + ":" + port);

    const string videoStream = "Video Stream";

    boost::asio::io_service ioService;
    udp::socket udpSocket(ioService, udp::endpoint(udp::v4(), 0));
    udp::resolver udpResolver(ioService);
    udp::resolver::query query(udp::v4(), ip, port);
    udp::endpoint udpEndpoint = *udpResolver.resolve(query);

    cv::VideoCapture cam(0);
    cam.set(CV_CAP_PROP_FORMAT, CV_8UC1);
    cam.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    cam.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
    //cam.set(CV_CAP_PROP_CONTRAST, 55);
    //cam.set(CV_CAP_PROP_SATURATION, 55);
    //cam.set(CV_CAP_PROP_GAIN, 55);

    if(!cam.isOpened()) {
      Log::error("No camera detected.");
      return;
    }

    cv::Mat frame;

    while(_initVideoConference) {
      // Frame receive
      _receive = false;

      std::unique_lock<std::mutex> lock(_mutex);
      while(!_receive) {
        _condReceive.wait(lock);
      }

      imshow(videoStream, currentFrame);

      // Frame grab
      if(!cam.grab()) continue;
      if(!cam.retrieve(frame) || frame.empty()) continue;
      if(cv::waitKey(30) >= 0) break;

      // Frame send
      cv::flip(frame, frame, 0);
      cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
      size_t bytes = sendCvMatVideoStream(frame, udpSocket, udpEndpoint);
      if(bytes > 0) {
        Log::info("Frame: " + std::to_string(frame.cols) + "x" + std::to_string(frame.rows)
                  + ", " + std::to_string(bytes) + " bytes.");
      }
    }

    destroyWindow(videoStream);
    _threadDone = true;
  }

  size_t Communicator::sendCvMatVideoStream(const cv::Mat& mat, udp::socket& udpSocket, const udp::endpoint& udpEndpoint) {
    size_t bytes = 0;
    std::vector<unsigned char> output;
    std::vector<unsigned char> mat_buff;
    std::vector<int> params;
    int type = Type::VIDEO_STREAM;

    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(80);
    cv::imencode(".jpg", mat, mat_buff, params);

    unsigned char packet_type[sizeof(int)];
    memcpy(packet_type, &type, sizeof(int));

    int length = mat_buff.size();
    unsigned char packet_size[sizeof(int)];
    memcpy(packet_size, &length, sizeof(int));

    // Tipo
    output.insert(output.begin(), &packet_type[0], &packet_type[sizeof(int)]);
    bytes += udpSocket.send_to(boost::asio::buffer(output, output.size()), udpEndpoint);
    output.clear();

    // Tamaño
    output.insert(output.begin(), &packet_size[0], &packet_size[sizeof(int)]);
    bytes += udpSocket.send_to(boost::asio::buffer(output, output.size()), udpEndpoint);
    output.clear();

    // Datos
    bytes += udpSocket.send_to(boost::asio::buffer(mat_buff, mat_buff.size()), udpEndpoint);

    return bytes;
  }

  void Communicator::processMatrix16f(StreamType& st) {
    Log::success("New 16 floats matrix received. Size: " + std::to_string(st.size));

    int num_floats = st.size/sizeof(float);
    float tmp[num_floats];
    memcpy(&tmp[0], &st.data[0], sizeof(float)*(num_floats));

    // Procesar matriz (tmp) aquí
  }

  void Communicator::processVectori(StreamType& st) {
    Log::success("New std::vector<int> received. Size: " + std::to_string(st.size));

    int num_ints = st.size/sizeof(int);
    std::vector<int> tmp(num_ints);
    memcpy(&tmp[0], &st.data[0], sizeof(int)*(num_ints));

    // Procesar vector (tmp) aquí
  }

  int Communicator::send() const {
    int buff_size = _buff.size();
    Log::info("Sending " + std::to_string(buff_size) + " bytes...");
    int bytes = boost::asio::write(*_tcpSocket, boost::asio::buffer(_buff, buff_size));
    Log::success(std::to_string(bytes) + " bytes sent.");

    return bytes;
  }

  size_t Communicator::addInt(int val) {
    unsigned char val_chars[sizeof(int)];
    memcpy(val_chars, &val, sizeof(int));
    _buff.insert(_buff.end(), &val_chars[0], &val_chars[sizeof(int)]);

    return sizeof(int);
  }

  size_t Communicator::addFloat(float val) {
    unsigned char val_chars[sizeof(float)];
    memcpy(val_chars, &val, sizeof(float));
    _buff.insert(_buff.end(), &val_chars[0], &val_chars[sizeof(float)]);

    return sizeof(float);
  }

  size_t Communicator::addChars(const char* chars, int num_chars) {
    int size = num_chars * sizeof(char);
    unsigned char val_chars[size];
    memcpy(val_chars, chars, size);
    _buff.insert(_buff.end(), &val_chars[0], &val_chars[size]);

    return size;
  }

  size_t Communicator::addMatrix16f(const float* matrix) {
    int size = 16 * sizeof(float);
    unsigned char sMatrix[size];
    memcpy(sMatrix, &matrix[0], size);
    _buff.insert(_buff.end(), &sMatrix[0], &sMatrix[size]);

    return size;
  }

  void Communicator::addVectori(const std::vector<int>& vector) {
    int size = vector.size() * sizeof(int);
    unsigned char sVector[size];
    int type = Type::VECTOR_I;
    memcpy(sVector, &vector[0], size);

    addInt(type);
    addInt(size);

    _buff.insert(_buff.end(), &sVector[0], &sVector[size]);
  }

  void Communicator::addSkip() {
    addInt(static_cast<int>(Type::SKIP));
  }

  void Communicator::addCvMat(const cv::Mat& mat) {
    std::vector<unsigned char> mat_buff;
    std::vector<int> params;
    int type = Type::CV_MAT;

    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(80);
    cv::imencode(".jpg", mat, mat_buff, params);

    addInt(type);
    addInt(mat_buff.size());

    _buff.insert(_buff.end(), mat_buff.begin(), mat_buff.end());
  }

  void Communicator::addPaper(Paper& paper) {
    // Type
    int type = Type::PAPER;

    // Id
    int id = paper.getId();

    // Matrix
    float modelview_matrix[16];
    paper.glGetModelViewMatrix(modelview_matrix);
    int size = 16 * sizeof(float);
    unsigned char sMatrix[size];
    memcpy(sMatrix, modelview_matrix, size);
    Log::matrix(modelview_matrix, Log::Colour::FG_DARK_GRAY);

    cv::Point& point = paper.getFingerPoint();

    size = 0;
    addInt(type);                            // Type
    addInt(-1);                              // Size placeholder
    size += addInt(id);                      // Paper Id
    size += addMatrix16f(modelview_matrix);  // Paper Model-View matrix
    size += addInt(point.x);
    size += addInt(point.y);

    // Script
    // Check whether there is a script associated
    bool found = false;
    auto& scripts = ConfigManager::getScriptsList();
    for(auto& script_pair : scripts) {
      if(script_pair.first == id) {
        found = true;
        break;
      }
    }

    if(found) {
      Script& script = ScriptManager::getInstance().getScript(id);
      size += addScript(script, id);         // Paper script
    }
    else {
      size += addInt(0);
    }

    // Real size
    unsigned char val_chars[sizeof(int)];
    memcpy(val_chars, &size, sizeof(int));
    for(size_t i = 0; i < sizeof(int); ++i) {
      _buff[sizeof(int) + i] = val_chars[i];
    }
  }

  size_t Communicator::addScript(Script& script, int id) {
    static int oldId = -2;
    int size = 0;

    std::vector<ScriptSentence>& sentences = script.getSentences();
    int num_sentences = sentences.size();

    if(id != oldId) {
      size += addInt(num_sentences);
    }
    else {
      size += addInt(0);
    }

    for(auto& sentence : sentences) {
      sentence.execute(script, ScriptManager::getInstance(), *this);
      size += std::stoi(sentence.getScriptFunction()->getProperty("bytes"));
    }

    oldId = id;

    return size;
  }

  void Communicator::addVectorCvMat(const std::vector<cv::Mat>& mats) {
    int size = mats.size();

    addInt(size);

    for(int i = 0; i < size; ++i) {
      addCvMat(mats[i]);
    }
  }

  void Communicator::addVectorPapers(std::vector<Paper>& papers) {
    int size = papers.size();
    int type = Type::PAPER;

    // Type         - Int
    // Size         - Int
    // Papers {
    //   id         - Int
    //   matrix     - 16 floats
    // }

    addInt(type);
    addInt(size);

    for(int i = 0; i < size; ++i) {
      addPaper(papers[i]);
    }
  }
}
