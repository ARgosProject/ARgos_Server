#include "Communicator.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#include "Log.h"

Communicator::Communicator(unsigned short port) : _port(port) {
  _tcpSocket = new tcp::socket(_ioService);


  // Threshold parameters
  circleDetectionThreshold = 170;

  // Application
  diffMinBetweenFrames = 4.0;    // maximum amount of movement between successive frames (must be smaller in order to add a board)
  timeMinBetweenCaptures = 2.0;  // minimum time between snapshots (seconds)
  
  bProjectorRefreshLock = true;

  //Boards parameters
  numBoardsFinalCamera = 20;
  numBoardsFinalProjector = 20;
  numBoardsBeforeCleaning = 3;
  numBoardsBeforeDynamicProjection = 5;
  maxReprojErrorCamera = 0.20;
  maxReprojErrorProjectorStatic = 0.25;
  maxReprojErrorProjectorDynamic = 0.43;

  projectorFrame.create(600, 800, CV_8UC1);
 
  //setState(CAMERA);
  setState(PROJECTOR_STATIC);
  //setState(DEMO_AR); 
  


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
    //std::thread(&Communicator::listenToCvMats, this).detach();
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
	break;
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
  //Logger::Log::success("Nueva cv::Mat recibida. Size: " + std::to_string(st.size));
 
  cv::imdecode(st.data, cv::IMREAD_GRAYSCALE, &cameraFrame);             // Decode cv::Mat

  update();
  draw();


  // Send Mat to Raspberry pi
  addCvMat(projectorFrame);
 
}

int Communicator::send() const {
  int buff_size = _buff.size();
  //Logger::Log::info("Intentando enviar " + std::to_string(buff_size) + " bytes...");
  int bytes = boost::asio::write(*_tcpSocket, boost::asio::buffer(_buff, buff_size));
  //Logger::Log::success(std::to_string(bytes) + " bytes enviados.");

  return bytes;
}

void Communicator::addMatrix16f(const float* matrix) {
  int size = 16 * sizeof(float);
  unsigned char sMatrix[size];
  int type = Type::MATRIX_16F;
  memcpy(sMatrix, matrix, size);

  unsigned char packet_type[sizeof(int)];
  memcpy(packet_type, &type, sizeof(int));

  unsigned char packet_size[sizeof(int)];
  memcpy(packet_size, &size, sizeof(int));

  _buff.insert(_buff.end(), &packet_type[0], &packet_type[sizeof(int)]); // Tipo
  _buff.insert(_buff.end(), &packet_size[0], &packet_size[sizeof(int)]); // Tamaño
  _buff.insert(_buff.end(), &sMatrix[0], &sMatrix[size]);                // Datos
}

void Communicator::addVectori(const std::vector<int>& vector) {
  int size = vector.size() * sizeof(int);
  unsigned char sVector[size];
  int type = Type::VECTOR_I;
  memcpy(sVector, &vector[0], size);

  unsigned char packet_type[sizeof(int)];
  memcpy(packet_type, &type, sizeof(int));

  unsigned char packet_size[sizeof(int)];
  memcpy(packet_size, &size, sizeof(int));

  _buff.insert(_buff.end(), &packet_type[0], &packet_type[sizeof(int)]); // Tipo
  _buff.insert(_buff.end(), &packet_size[0], &packet_size[sizeof(int)]); // Tamaño
  _buff.insert(_buff.end(), &sVector[0], &sVector[size]);                // Datos
}

void Communicator::addCvMat(const cv::Mat& mat) {
  std::vector<unsigned char> mat_buff;
  std::vector<int> params;
  int type = Type::CV_MAT;

  params.push_back(cv::IMWRITE_JPEG_QUALITY);
  params.push_back(80);
  cv::imencode(".jpg", mat, mat_buff, params);

  unsigned char packet_type[sizeof(int)];
  memcpy(packet_type, &type, sizeof(int));

  int length = mat_buff.size();
  unsigned char packet_size[sizeof(int)];
  memcpy(packet_size, &length, sizeof(int));

  _buff.insert(_buff.end(), &packet_type[0], &packet_type[sizeof(int)]); // Tipo
  _buff.insert(_buff.end(), &packet_size[0], &packet_size[sizeof(int)]); // Tamaño
  _buff.insert(_buff.end(), mat_buff.begin(), mat_buff.end());           // Datos
}

void Communicator::addVectorCvMat(const std::vector<cv::Mat>& mats) {
  int size = mats.size();

  unsigned char num_mats[sizeof(int)];
  memcpy(num_mats, &size, sizeof(int));

  _buff.insert(_buff.end(), &num_mats[0], &num_mats[sizeof(int)]);
  for(int i = 0; i < size; ++i) {
    addCvMat(mats[i]);
  }
}


void Communicator::update(){
  
  try{
    projectorFrame = cv::Scalar::all(0);
    //cameraFrame.copyTo(debugFrame);
    
    switch (currState) {
    case CAMERA:
      if(!timeMinBetweenFrames()) return;
      calibrateCamera(cameraFrame);
      lastTime = cv::getTickCount();
      break;
      
    case PROJECTOR_STATIC:
      if(!timeMinBetweenFrames()) return;
      calibrateProjector(cameraFrame);
      lastTime = cv::getTickCount();
      break;
      
    case PROJECTOR_DYNAMIC:
      if(bProjectorRefreshLock){
	if(camProjCalib.setDynamicProjectorImagePoints(cameraFrame)){
	  cout << "Updating projection points ..." << endl;
	  if(!timeMinBetweenFrames()) return;
	  bProjectorRefreshLock = false;
	}
      }
      else{
	calibrateProjector(cameraFrame);
	bProjectorRefreshLock = true;
	lastTime = cv::getTickCount();
      }
      break;      
    case DEMO_AR:
      calculateDemo(cameraFrame);
      break;
    default: break;
    }
  }catch (cv::Exception e){
    cout << endl << "Exception caught: " << e.msg << endl;
  }  
}

/**
 * 
 * DRAW
 *
 */
void Communicator::draw(){
 
  switch (currState) {
  case CAMERA:
    break;
  case PROJECTOR_STATIC:
  case PROJECTOR_DYNAMIC:
    drawProjectorPattern();
    break;
  case DEMO_AR:
    drawProjectorFinePattern();
  default:
    break;
  }
}


bool Communicator::calibrateCamera(const cv::Mat& cameraFrame){
  
  CameraCalibration& calibrationCamera = camProjCalib.getCalibrationCamera();
  
  bool foundPattern = calibrationCamera.add(cameraFrame);
  
  if(foundPattern){
    drawCameraImagePoints();
    //cv::imshow(snapShotWindow, snapShot); 
    
    calibrationCamera.calibrate();
    
    if(calibrationCamera.size() >= numBoardsBeforeCleaning) {
      calibrationCamera.clean(maxReprojErrorCamera);
      
      if(calibrationCamera.getReprojectionError(calibrationCamera.size()-1) > maxReprojErrorCamera) {
	cout << "Board found, but reproj. error is too high, skipping" << endl;
	return false;
      }
    }
    
    if (calibrationCamera.size() >= numBoardsFinalCamera){
      calibrationCamera.save("calibrationCamera.xml");
      cout << "Camera calibration finished & saved to calibrationCamera.xml" << endl;
      setState(PROJECTOR_STATIC);
    }
  } else cout << "Could not find board" << endl;
  
  return foundPattern;
}


bool Communicator::calibrateProjector(const cv::Mat& cameraFrame){
  
  //CameraCalibration& calibrationCamera = camProjCalib.getCalibrationCamera();
  ProjectorCalibration& calibrationProjector = camProjCalib.getCalibrationProjector();
  
  processImageForCircleDetection(cameraFrame);
  
  if(camProjCalib.addProjected(cameraFrame, processedImg)){
   
    waitKey(1);
    cout << "Calibrating projector" << endl;
      
    calibrationProjector.calibrate();
    
    if(calibrationProjector.size() >= numBoardsBeforeCleaning) {
      
      cout << "Cleaning" << endl;
      
      int numBoardRemoved;
      if(currState == PROJECTOR_STATIC)
	numBoardRemoved = camProjCalib.cleanStereo(maxReprojErrorProjectorStatic);
      else	
	numBoardRemoved = camProjCalib.cleanStereo(maxReprojErrorProjectorDynamic);
      
      cout << numBoardRemoved << " boards removed" << endl;
      
      if(currState == PROJECTOR_DYNAMIC && calibrationProjector.size() < numBoardsBeforeDynamicProjection) {
	cout << "Too many boards removed, restarting to PROJECTOR_STATIC" << endl;
	setState(PROJECTOR_STATIC);
	return false;
      }
    }
    
    if (calibrationProjector.size() > 0){
      cout << "Performing stereo-calibration" << endl;
      camProjCalib.stereoCalibrate();
      cout << "Done" << endl;
    }

    if(currState == PROJECTOR_STATIC) {
      
      if( calibrationProjector.size() < numBoardsBeforeDynamicProjection) 
	cout << numBoardsBeforeDynamicProjection - calibrationProjector.size() << " boards to go before dynamic projection" << endl;
      else {
	setState(PROJECTOR_DYNAMIC);
	lastTime = cv::getTickCount();
      }
    }
    else{
      if( calibrationProjector.size() < numBoardsFinalProjector) 
	cout << numBoardsFinalProjector - calibrationProjector.size() << " boards to go to completion" << endl;
      else {
	calibrationProjector.save("calibrationProjector.xml");
	cout << "Projector calibration finished & saved to calibrationProjector.xml" << endl;
        
	camProjCalib.saveExtrinsics("CameraProjectorExtrinsics.xml");
	cout << "Stereo Calibration finished & saved to CameraProjectorExtrinsics.xml" << endl;
        
	cout << "Congrats, you made it ;)" << endl;
	setState(DEMO_AR);
      }
    }
    return true;
  } 
  return false;
}


void Communicator::processImageForCircleDetection(const cv::Mat& cameraFrame){    
  if(cameraFrame.type() != CV_8UC1) 
    cv::cvtColor(cameraFrame, processedImg, CV_RGB2GRAY);
  else 
    cameraFrame.copyTo(processedImg);   
  
  cv::threshold(processedImg, processedImg, circleDetectionThreshold, 255, cv::THRESH_BINARY_INV);
}


bool Communicator::timeMinBetweenFrames(){
  double timeDiff = ((double)cv::getTickCount() - lastTime) /cv::getTickFrequency(); //time in second
  //cout << "seconds elapsed :" << timeDiff <<endl;
  return timeMinBetweenCaptures < timeDiff;
}


void Communicator::drawCameraImagePoints(){
  vector<vector<cv::Point2f> > imagePoints = camProjCalib.getCalibrationCamera().getImagePoints();
  if(imagePoints.size() <= 0) return;
  for(size_t i = 0; i < imagePoints.back().size(); i++)
    cv::circle(snapShot, imagePoints.back()[i], 3,  CV_RGB(255,0,0), -1, 8);
}


void Communicator::drawProjectorPattern(){
  vector<cv::Point2f> patternPoints = camProjCalib.getCalibrationProjector().getCandidateImagePoints();
  if(patternPoints.size() <= 0) return;
  for(size_t i = 0; i < patternPoints.size(); i++){
    // cout <<  patternPoints[i] << endl;
    cv::circle(projectorFrame, patternPoints[i], 18,  CV_RGB(255,255,255), -1, 8);
  }
}

void Communicator::drawProjectorFinePattern(){
  vector<cv::Point2f> patternPoints = camProjCalib.getCalibrationProjector().getCandidateImagePoints();
  if(patternPoints.size() <= 0) return;
  for(size_t i = 0; i < patternPoints.size(); i++){
    // cout <<  patternPoints[i] << endl;
    cv::circle(projectorFrame, patternPoints[i], 6,  CV_RGB(255,255,255), -1, 8);
  }
}

// argosCalibrator States
void Communicator::setState(CalibState state){
  
  CameraCalibration& calibrationCamera = camProjCalib.getCalibrationCamera();
  ProjectorCalibration& calibrationProjector = camProjCalib.getCalibrationProjector();
  
  switch (state){
  case CAMERA:
    camProjCalib.resetBoards();
    break;
  case PROJECTOR_STATIC:
    calibrationCamera.load("calibrationCamera.xml",false);
    camProjCalib.resetBoards();
    calibrationCamera.setupCandidateObjectPoints();
    calibrationProjector.setStaticCandidateImagePoints();
    break;
  case PROJECTOR_DYNAMIC:
    break;
  case DEMO_AR:
    camProjCalib.load("calibrationCamera.xml", "calibrationProjector.xml", "CameraProjectorExtrinsics.xml");
    calibrationCamera.setupCandidateObjectPoints();
    break;
  default:
    break;
  }
  
  currState = state;
  currStateString = getCurrentStateString();
  cout << "Set state : " << getCurrentStateString() << endl;
}

string Communicator::getCurrentStateString(){
  string name;
  switch (currState){
  case CAMERA:            name = "CAMERA"; break;
  case PROJECTOR_STATIC:  name = "PROJECTOR_STATIC"; break;
  case PROJECTOR_DYNAMIC: name = "PROJECTOR_DYNAMIC"; break;
  case DEMO_AR:           name = "DEMO_AR"; break;
  default: break;
  }
  return name;
}

void Communicator::toGreyscale(const cv::Mat& inputFrame, cv::Mat& cameraFrame){
  //-Convert to greyScale (it must be a 3 channel image)
  if (inputFrame.type() == CV_8UC3) 
    cv::cvtColor(inputFrame, cameraFrame, CV_BGR2GRAY);
  else     
    cameraFrame = inputFrame;
}

bool Communicator::calculateDemo(cv::Mat& cameraFrame){
  
  CameraCalibration& calibrationCamera = camProjCalib.getCalibrationCamera();
  ProjectorCalibration& calibrationProjector  = camProjCalib.getCalibrationProjector();
  
  vector<cv::Point2f> chessImgPts;
  bool bPrintedPatternFound = calibrationCamera.findBoard(cameraFrame, chessImgPts, true);
  
  if(bPrintedPatternFound) {
    
    cv::Mat boardRot;
    cv::Mat boardTrans;
    calibrationCamera.computeCandidateBoardPose(chessImgPts, boardRot, boardTrans);
    //const auto & camCandObjPts = calibrationCamera.getCandidateObjectPoints();        
    //vector<cv::Point3f> camCandObjPts = calibrationCamera.getCandidateObjectPoints();
    //cv::Point3f axisX = camCandObjPts[1] - camCandObjPts[0];
    //cv::Point3f axisY = camCandObjPts[calibrationCamera.getPatternSize().width] - camCandObjPts[0];
    //cv::Point3f pos   = camCandObjPts[0] - axisY * (calibrationCamera.getPatternSize().width-2);
            
    vector<cv::Point3f> auxObjectPoints;
    //for(int i = 0; i < calibrationCamera.getPatternSize().height; i++)
    //  for(int j = 0; j < calibrationCamera.getPatternSize().width; j++)
    //auxObjectPoints.push_back(cv::Point3f(float(j * calibrationCamera.getSquareSize()), float(i * calibrationCamera.getSquareSize()), 0));
    auxObjectPoints.push_back(calibrationCamera.getCandidateObjectPoints()[0]);
    auxObjectPoints.push_back(calibrationCamera.getCandidateObjectPoints()[calibrationCamera.getPatternSize().width-1]);
    auxObjectPoints.push_back(calibrationCamera.getCandidateObjectPoints()[calibrationCamera.getPatternSize().width*calibrationCamera.getPatternSize().height-1]);
    auxObjectPoints.push_back(calibrationCamera.getCandidateObjectPoints()[calibrationCamera.getPatternSize().width*(calibrationCamera.getPatternSize().height-1)]);
    
         
    cv::Mat rotObjToProj, transObjToProj;
    
    cv::composeRT(boardRot,  boardTrans,
		  camProjCalib.getCamToProjRotation(), camProjCalib.getCamToProjTranslation(),
		  rotObjToProj, transObjToProj);

    vector<cv::Point2f> out;
    projectPoints(cv::Mat(auxObjectPoints),
		  rotObjToProj, transObjToProj,
		  calibrationProjector.getDistortedIntrinsics().getCameraMatrix(),
		  calibrationProjector.getDistCoeffs(),
		  out);
    
    calibrationProjector.setCandidateImagePoints(out);
  }
  
  return bPrintedPatternFound;
}


