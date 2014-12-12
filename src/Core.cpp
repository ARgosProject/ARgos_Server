#include "Core.h"
#include "Log.h"

namespace argosServer{

  Core::Core(){
    // Read configuration file
    Log::info("Reading configuration file... ");
    //Configuration::getInstance()->readFile();
  
    //- Camera & Projector Parameters ----
    Log::info("Loading camera & projector parameters... ");
    cameraProjector.load("calibrationCamera.yml", "calibrationProjector.yml", "CameraProjectorExtrinsics.yml");
    if(!cameraProjector.isValid()){
      Log::error("Camera or projector parameters is not set, need to run the calibrator tool");
      exit(1);  // throw "Camera parameters is not set";
    }
  
    // background
    //background = imread("background_distorted.jpg", CV_LOAD_IMAGE_COLOR );
  
    //frame.create(240, 320,CV_8UC3);
    //video = imread("video.jpg", CV_LOAD_IMAGE_COLOR );
    //M = (Mat_<double>(3,3) << 0.920522217764549, -0.05717832171634402, 25.53594017028823,
    //   -0.001733162838494719, -1.019753859872951, 485.4448242187501,
    //   9.872223504399846e-06, -0.0001829466936106208, 1);
    //drawCV::drawBoardMarks(background, 8, 40);    // Draw projections limits
    //bg_out = cv::Mat(480, 640,CV_8UC3);
    //cv::warpPerspective ( background, bg_out,  M, Size(640,480),cv::INTER_NEAREST );
  
  
    //projectorFrame.create(600, 800,CV_8UC3);
    //background.copyTo(projectorFrame);
    //bg_out.copyTo(projectorFrame);
  
  
    //Paper size : {A4,A5}  
    //paperSize = Configuration::getInstance()->getPaperSize();
    //paperSize = Size(27.3, 43.0);     // Projection Size
    //paperSize = Size(27.1, 42.7);     // Projection Size
    paperSize = Size(21.0, 29.7);     // Paper Size A4
    //paperSize = Size(14.8, 21.0);     // Paper Size A5
  
  
    // Paper detector initilization
    PaperDetector::getInstance();
  
    // Document detector initialization
    DocumentDetector::getInstance();
    
    
    lastSearch = "NONE";
    isPreviousPaperDetected = false;
    numFrames = 0;
    previousNumInvoices = 0;
    initVideoConference = false;
  }

  Core::~Core(){}


  vector<Paper> Core::processCvMat(cv::Mat& currentFrame){
    
    numFrames++;
    //bg_out.copyTo(projectorFrame);
    //background.copyTo(projectorFrame);
    //projectorFrame = cv::Scalar::all(0);   // Clear last output frame
    //currentFrame.copyTo(projectorFrame);
    
    //Chessboard Test
    //drawChessBoard(currentFrame, projectorFrame, cameraProjector);
    
    //drawCV::drawBoardMarks(projectorFrame, 8, 40);    // Draw projections limits
    
    
    //cv::Mat out(480, 640,CV_8UC3);
    //cv::Mat def(480, 640,CV_8UC3);
    
    //Detection of papers in the image passed
    PaperDetector::getInstance().detect(currentFrame,paperList, cameraProjector, paperSize, false, false);   // Projector
    //PaperDetector::getInstance()->detect(currentFrame,paperList, cameraProjector, paperSize, true, true);    // Camera
    
    numInvoices = paperList.size();
    
    //if (initVideoConference){
    //if ( paperList.size() == 0)
    // initVideoConference = false;
    //}
    
    //else{
    if (paperList.size() == 0 ){
      //cout << "Detecting NONE" << endl;
      isPreviousPaperDetected = false;
      previousNumInvoices = 0;
      numFrames = 0;
    }
    else{
      if (!isPreviousPaperDetected || (numInvoices != previousNumInvoices)){// || numFrames > 30){
	isPreviousPaperDetected = true;
	numFrames = 0;
	previousNumInvoices = numInvoices;
	
	DocumentDetector::getInstance().detect(currentFrame,paperList);
	
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
    //for (unsigned int i=0; i<paperList.size(); i++){
    //cout <<  "invoicesIndex " << i << ":"<< invoicesIndex[i]<< endl;
    //if  (invoicesIndex[i] == 999){
    //cvtColor(frame,frame,CV_BGR2RGB);
    //DrawCV::projectPaper(projectorFrame,paperList[i],cameraProjector,frame,out);
    
    //DrawCV::projectPaper0(projectorFrame,paperList[i],cameraProjector,0);
    //}
    //else
    //	DrawCV::projectPaper0(projectorFrame,paperList[i],cameraProjector,invoicesIndex[i]);
    //DrawCV::projectPaper(projectorFrame,paperList[i],cameraProjector,lastSearch);
    //DrawCV::drawInvoice(projectorFrame,paperList[i],cameraProjector);
    //DrawCV::draw3dPaper(projectorFrame,paperList[i],cameraProjector);
    //DrawCV::cameraPaper(projectorFrame,paperList[i],cameraProjector, lastSearch);
    //}
    //}
  
    // Send Mat to Raspberry pi
    
    //cvtColor(projectorFrame,projectorFrame,CV_BGR2RGB);
    //addCvMat(projectorFrame);
    
  
    
    //imshow("Out", def);
    //cv::waitKey(1);
  
    //cvtColor(def,def,CV_BGR2RGB);
    //addCvMat(def);
    return paperList;
    
  }
  
  
  vector<Paper>& Core::processOpenGL(cv::Mat& currentFrame, bool &initVideoConference){
    
    numFrames++;
    
    //Detection of papers in the image passed
    PaperDetector::getInstance().detect(currentFrame,paperList, cameraProjector, paperSize, false, false);   // Projector
    //PaperDetector::getInstance()->detect(currentFrame,paperList, cameraProjector, paperSize, true, true);    // Camera
    
    numInvoices = paperList.size();
    
    if (initVideoConference){
      if ( paperList.size() == 0)
	initVideoConference = false;
    }
    
    else{
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
	  
	  DocumentDetector::getInstance().detect(currentFrame,paperList);
	  
	  invoicesIndex.clear();
	  //cout <<  "invoicesIndex clear "<< endl;
	  for (unsigned int i=0; i<paperList.size(); i++){
	    initVideoConference = (paperList[i].getId() == 999);
	    invoicesIndex.push_back(paperList[i].getId());
	    //cout <<  "invoicesIndex " << i << ":"<< invoicesIndex[i]<< endl;
	  }
	}
      }
    }
    
    for (unsigned int i=0; i<paperList.size(); i++){
      Log::info(std::to_string(invoicesIndex[i]));
      paperList[i].setId(invoicesIndex[i]);
    }
    return paperList;
    
  }
} 
