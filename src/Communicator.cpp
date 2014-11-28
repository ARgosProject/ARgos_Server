#include "Communicator.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include "Log.h"

Communicator::Communicator(unsigned short port) : _port(port), _threadDone(true), _receive(false) {
  _tcpSocket = new tcp::socket(_ioService);


  cout << "Loading camera & projector parameters... " << endl;
  //- Camera Parameters ----
  cameraProjector.load("calibrationCamera.yml", "calibrationProjector.yml", "CameraProjectorExtrinsics.yml");
  if(!cameraProjector.isValid()){
    cout << "! Camera or projector parameters is not set, need to run the calibrator tool" << endl;
    exit(1);
  }

  video = imread("video.jpg", CV_LOAD_IMAGE_COLOR );
  background = imread("background_distorted.jpg", CV_LOAD_IMAGE_COLOR );


  //M = (Mat_<double>(3,3) << 0.920522217764549, -0.05717832171634402, 25.53594017028823,
  //   -0.001733162838494719, -1.019753859872951, 485.4448242187501,
  //   9.872223504399846e-06, -0.0001829466936106208, 1);

  //drawCV::drawBoardMarks(background, 8, 40);    // Draw projections limits
  //bg_out = cv::Mat(480, 640,CV_8UC3);
  //cv::warpPerspective ( background, bg_out,  M, Size(640,480),cv::INTER_NEAREST );



  projectorFrame.create(600, 800,CV_8UC3);
  background.copyTo(projectorFrame);
  //bg_out.copyTo(projectorFrame);

  //paperSize = Size(27.3, 43.0);     // Projection Size
  //paperSize = Size(27.1, 42.7);     // Projection Size
  paperSize = Size(21.0, 29.7);     // Paper Size A4
  //paperSize = Size(14.8, 21.0);     // Paper Size A5

  lastSearch = "NONE";
  isPreviousPaperDetected = false;
  documentDetector =  new DocumentDetector();
  numFrames = 0;
  previousNumInvoices = 0;
  initVideoConference = false;
}

Communicator::~Communicator() {
  if(_tcpSocket)
    delete _tcpSocket;
}

void Communicator::waitForConnections() {
  tcp::acceptor a(_ioService, tcp::endpoint(tcp::v4(), _port));

  while(1) {
    Logger::Log::info("Servidor ARgos escuchando en puerto " + std::to_string(_port));
    a.accept(*_tcpSocket);
    Logger::Log::success("Nueva conexión de " + _tcpSocket->remote_endpoint().address().to_string());
    receive();
  }
}

void Communicator::receive() {
  while(1) {
    StreamType st;
    unsigned char type_buf[sizeof(int)];
    unsigned char size_buf[sizeof(int)];
    unsigned char* data_buf;

    try {
      boost::asio::read(*_tcpSocket, boost::asio::buffer(&type_buf, sizeof(int)));  // Type
      memcpy(&st.type, &type_buf, sizeof(int));
      boost::asio::read(*_tcpSocket, boost::asio::buffer(&size_buf, sizeof(int))); // Size
      memcpy(&st.size, &size_buf, sizeof(int));

      data_buf = new unsigned char[st.size];
      boost::asio::read(*_tcpSocket, boost::asio::buffer(data_buf, st.size)); // Data
      st.data.insert(st.data.end(), &data_buf[0], &data_buf[st.size]);
      delete [] data_buf;

      switch(st.type) {
      case Type::VECTOR_I:
        proccessVectori(st);
        break;
      case Type::MATRIX_16F:
        proccessMatrix16f(st);
        break;
      case Type::CV_MAT:
        proccessCvMat(st);
        _receive = true;
        _condReceive.notify_one();
        break;
      default:
        break;
      }

      if(initVideoConference) {
        if(_threadDone) {
          Logger::Log::video("Nueva videoconferencia iniciada");
          _threadDone = false;
          _videoThread.reset();
          _videoThread = std::make_shared<std::thread>(&Communicator::startVideoStream, this, st);
          _videoThread->detach();
        }
      }

      if(!_buff.empty()) {
        send();
        _buff.clear();
      }
    }
    catch(boost::system::system_error const& e) {
      Logger::Log::error("Se perdió la conexión del cliente. " + std::string(e.what()));
      _tcpSocket->close();
      break;
    }
  }
}

void Communicator::startVideoStream(StreamType st) {
  const std::string ip = _tcpSocket->remote_endpoint().address().to_string();
  const std::string port("9999");
  Logger::Log::video("Enviando video a " + ip + ":" + port);

  boost::asio::io_service ioService;
  udp::socket udpSocket(ioService, udp::endpoint(udp::v4(), 0));
  udp::resolver udpResolver(ioService);
  udp::resolver::query query(udp::v4(), ip, port);
  udp::endpoint udpEndpoint = *udpResolver.resolve(query);

  cv::VideoCapture cam(0);
  cam.set(CV_CAP_PROP_FORMAT, CV_8UC1);
  cam.set(CV_CAP_PROP_FRAME_WIDTH, 320);
  cam.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
  cam.set(CV_CAP_PROP_CONTRAST, 55);
  cam.set(CV_CAP_PROP_SATURATION, 55);
  cam.set(CV_CAP_PROP_GAIN, 55);

  if(!cam.isOpened()) {
    Logger::Log::error("No se detectó ninguna cámara.");
    return;
  }

  cv::Mat frame;
  while(initVideoConference) {
    // Frame grab
    if(!cam.grab()) continue;
    if(!cam.retrieve(frame) || frame.empty()) continue;
    if(cv::waitKey(30) >= 0) break;

    // Frame send
    cv::flip(frame, frame, 0);
    cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
    size_t bytes = sendCvMatVideoStream(frame, udpSocket, udpEndpoint);
    if(bytes > 0) {
      Logger::Log::info("Frame: " + std::to_string(frame.cols) + "x" + std::to_string(frame.rows)
                        + ", " + std::to_string(bytes) + " bytes.");
    }

    // Frame receive
    std::unique_lock<std::mutex> lock(_mutex);
    while(!_receive) {
      _condReceive.wait(lock);
    }
    imshow("Video Stream", currentFrame);

    _receive = false;
  }

  destroyWindow("Video Stream");
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

void Communicator::proccessMatrix16f(StreamType& st) {
  Logger::Log::success("Nueva matriz de 16 float recibida. Size: " + std::to_string(st.size));

  int num_floats = st.size/sizeof(float);
  float tmp[num_floats];
  memcpy(&tmp[0], &st.data[0], sizeof(float)*(num_floats));

  // Procesar matriz (tmp) aquí

}

void Communicator::proccessVectori(StreamType& st) {
  Logger::Log::success("Nuevo std::vector<int> recibido. Size: " + std::to_string(st.size));

  int num_ints = st.size/sizeof(int);
  std::vector<int> tmp(num_ints);
  memcpy(&tmp[0], &st.data[0], sizeof(int)*(num_ints));

  // Procesar vector (tmp) aquí

}

void Communicator::proccessCvMat(StreamType& st) {
  Logger::Log::success("Nueva cv::Mat recibida. Size: " + std::to_string(st.size));

  cv::imdecode(st.data, cv::IMREAD_GRAYSCALE, &currentFrame);             // Decode cv::Mat

  numFrames++;
  //bg_out.copyTo(projectorFrame);
  background.copyTo(projectorFrame);
  //projectorFrame = cv::Scalar::all(0);   // Clear last output frame
  //currentFrame.copyTo(projectorFrame);

  //Chessboard Test
  //drawChessBoard(currentFrame, projectorFrame, cameraProjector);

  //drawCV::drawBoardMarks(projectorFrame, 8, 40);    // Draw projections limits


  cv::Mat out(480, 640,CV_8UC3);
  cv::Mat def(480, 640,CV_8UC3);
  //Detection of papers in the image passed
  paperDetector.detect(currentFrame,paperList, cameraProjector, paperSize, false, false);   // Projector
  //paperDetector.detect(currentFrame,paperList, cameraProjector, paperSize, false, true);      // Camera

  numInvoices = paperList.size();
  /*
    if (initVideoConference){
    if ( paperList.size() == 0)
    initVideoConference = false;
    }

    else{
  */
  if (paperList.size() == 0 ){
    //cout << "Detecting NONE" << endl;
    isPreviousPaperDetected = false;
    previousNumInvoices = 0;
    numFrames = 0;
  }
  else{
    if (!isPreviousPaperDetected || (numInvoices != previousNumInvoices) || numFrames > 30){
      isPreviousPaperDetected = true;
      numFrames = 0;
      previousNumInvoices = numInvoices;

      documentDetector->detect(currentFrame,paperList);

      invoicesIndex.clear();
      //cout <<  "invoicesIndex clear "<< endl;
      for (unsigned int i=0; i<paperList.size(); i++){
        //if (paperList[i].getId() == 999) initVideoConference = true;
        invoicesIndex.push_back(paperList[i].getId());
        //cout <<  "invoicesIndex " << i << ":"<< invoicesIndex[i]<< endl;
      }
    }
  }
  //}

  // Send Mat to Raspberry pi

  //cvtColor(projectorFrame,projectorFrame,CV_BGR2RGB);
  //addCvMat(projectorFrame);

  //double alpha = 1;
  //double beta = 1;
  //double gamma = 0.0; //offset


  //addWeighted(projectorFrame,alpha,out,beta,gamma,def);

  //imshow("Out", def);
  //cv::waitKey(1);

  //cvtColor(def,def,CV_BGR2RGB);
  //addCvMat(def);

  //cvtColor(projectorFrame,projectorFrame,CV_BGR2RGB);
  //addCvMat(projectorFrame);
  //addVectorPapers(paperList);

  //if(!paperList.empty())
  addPaper(paperList.back());
}

int Communicator::send() const {
  int buff_size = _buff.size();
  Logger::Log::info("Intentando enviar " + std::to_string(buff_size) + " bytes...");
  int bytes = boost::asio::write(*_tcpSocket, boost::asio::buffer(_buff, buff_size));
  Logger::Log::success(std::to_string(bytes) + " bytes enviados.");

  return bytes;
}

void Communicator::addInt(int val) {
  unsigned char val_chars[sizeof(int)];
  memcpy(val_chars, &val, sizeof(int));
  _buff.insert(_buff.end(), &val_chars[0], &val_chars[sizeof(int)]);
}

void Communicator::addMatrix16f(const float* matrix) {
  int size = 16 * sizeof(float);
  unsigned char sMatrix[size];
  /*int type = Type::MATRIX_16F;

    addInt(type);
    addInt(size);
  */
  memcpy(sMatrix, matrix, size);
  _buff.insert(_buff.end(), &sMatrix[0], &sMatrix[size]);                // Datos
}

void Communicator::addVectori(const std::vector<int>& vector) {
  int size = vector.size() * sizeof(int);
  unsigned char sVector[size];
  int type = Type::VECTOR_I;
  memcpy(sVector, &vector[0], size);

  addInt(type);
  addInt(size);

  _buff.insert(_buff.end(), &sVector[0], &sVector[size]);                // Datos
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

  _buff.insert(_buff.end(), mat_buff.begin(), mat_buff.end());           // Datos
}

void Communicator::addPaper(Paper& paper) {
  float modelview_matrix[16];
  paper.glGetModelViewMatrix(modelview_matrix);

  int size = (1 * sizeof(int)) + (16 * sizeof(float));
  unsigned char sMatrix[size];
  int type = Type::PAPER;
  memcpy(sMatrix, modelview_matrix, size);

  Logger::Log::matrix(modelview_matrix);

  addInt(type);
  addInt(size);
  addInt(invoicesIndex.back());
  addMatrix16f(modelview_matrix);
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

void Communicator::drawChessBoard(cv::Mat& currentFrame, cv::Mat& projectorFrame, CameraProjectorSystem& cameraProjector){
  vector<cv::Point2f> chessImgPts;
  vector<cv::Point3f> chessObjPts;
  bool bPrintedPatternFound = cv::findChessboardCorners(currentFrame, cv::Size(8, 5), chessImgPts, CV_CALIB_CB_ADAPTIVE_THRESH);
  if(bPrintedPatternFound) {
    //cv::cornerSubPix(currentFrame, chessImgPts, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ));
    for(int i = 0; i < 5; i++)
      for(int j = 0; j < 8; j++)
        chessObjPts.push_back(cv::Point3f(float(j * 3.0), float(i * 3.0), 0));

    CameraModel& camera = cameraProjector.getCamera();
    CameraModel& projector = cameraProjector.getProjector();

    cv::Mat camMatrix = camera.getDistortedCamMatrix();
    cv::Mat distCoeffs = camera.getDistCoeffs();
    cv::Mat boardRot,boardTrans;
    cv::solvePnP(chessObjPts, chessImgPts, camMatrix, distCoeffs, boardRot, boardTrans);

    cv::Mat rotObjToProj, transObjToProj;
    cv::composeRT(boardRot, boardTrans,
                  cameraProjector.getCamToProjRotation(), cameraProjector.getCamToProjTranslation(),
                  rotObjToProj, transObjToProj);

    vector<cv::Point3f> auxObjectPoints;
    for(int i = 0; i < 5; i++)
      for(int j = 0; j < 8; j++)
        auxObjectPoints.push_back(cv::Point3f(float(j * 3.0), float(i * 3.0), 0));

    vector<cv::Point2f> out;
    cv::projectPoints(cv::Mat(auxObjectPoints), rotObjToProj, transObjToProj, projector.getDistortedCamMatrix(), projector.getDistCoeffs(), out);

    if(out.size() > 0)
      for(size_t i = 0; i < out.size(); i++)
        cv::circle(projectorFrame, out[i], 5,  CV_RGB(255,255,255), -1, 8);

    //cout << "rotObjToProj:" << rotObjToProj << endl;
    //cout << "transObjToProj:" << transObjToProj << endl;
  }
}
