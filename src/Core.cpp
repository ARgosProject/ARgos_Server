#include "Core.h"
#include "Log.h"
#include "HandDetector.h"
#include "ScriptManager.h"
#include "ConfigManager.h"
#include "DrawCV.h"

namespace argosServer{

  Core::Core(){
    //- Read configuration file ----
    Log::info("Reading configuration file... ");
    ConfigManager::loadConfiguration("data/config.xml");

    //- Camera & Projector Parameters ----
    Log::info("Loading camera & projector parameters... ");
    cameraProjector.setCalibrationsPath("data/calibrations/");
    cameraProjector.load(ConfigManager::getCameraCalibration(),
                         ConfigManager::getProjectorCalibration(),
                         ConfigManager::getExtrinsics());

    if(!cameraProjector.isValid()){
      Log::error("Camera or projector parameters is not set, need to run the calibrator tool");
      exit(1);  // throw "Camera parameters is not set";
    }
    else
      Log::success("Camera & projector parameters... OK");


    //- Script loading
    ScriptManager::getInstance().loadScripts("data/scripts/");

    // Paper detector initilization
    PaperDetector::getInstance();

    // Document detector initialization
    DocumentDetector::getInstance().setImagesPath("data/templates/");
    DocumentDetector::getInstance().configure();

    // Hand and finger detector initialization
    //HandDetector::getInstance();

    lastSearch = "NONE";
    isPreviousPaperDetected = false;
    numFrames = 0;
    previousNumInvoices = 0;
    initVideoConference = false;
  }

  Core::~Core(){}

  vector<Paper>& Core::update(cv::Mat& currentFrame, bool &initVideoConference){

    numFrames++;

    //Detection of papers in the image passed
    PaperDetector::getInstance().detect(currentFrame, paperList, cameraProjector,
                                        ConfigManager::getPaperSize(), ConfigManager::getOutputDisplay());
    if(!paperList.empty())
      Log::paper(paperList.back());

    numInvoices = paperList.size();


    //Finger Detection ---------
    //cv::Point fingerPoint;
    //HandDetector::getInstance().detectFinger(currentFrame,fingerPoint);
    //if(fingerPoint.x >= 0 && fingerPoint.y >= 0)
    //PaperCandidates[i].calculateExtrinsics(paperSizeMeters, cameraProjector, setYPerperdicular, screenExtrinsics);


    //ScriptManager::getInstance().update();

    // CheckRegion TEST
    //checkRegion(currentFrame,paperList);


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
        if (!isPreviousPaperDetected || (numInvoices != previousNumInvoices)){// || (numFrames > 30)){
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


    /*
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
      //initVideoConference = (paperList[i].getId() == 999);
      invoicesIndex.push_back(paperList[i].getId());
      //cout <<  "invoicesIndex " << i << ":"<< invoicesIndex[i]<< endl;
      }
      }
      }

      for (unsigned int i=0; i<paperList.size(); i++){
      Log::info(std::to_string(invoicesIndex[i]));
      paperList[i].setId(invoicesIndex[i]);
      }
    */
    return paperList;

  }



  void Core::checkRegion(cv::Mat& currentFrame,vector<Paper>& detectedPapers){
    cv::Mat paperExtraction;
    cv::Mat signature;
    if (detectedPapers.size() == 1)
      warp(currentFrame,paperExtraction,Size(600,425), detectedPapers.back());

    getRectSubPix(paperExtraction, Size(101,50), Point2f(416.16,313.13), signature);

    imshow("extraction", paperExtraction);
    imshow("signature", signature);
    cv::waitKey(1);

  }




  bool Core::warp (const cv::Mat& in, cv::Mat& out, Size size, vector<Point2f> points ) throw ( cv::Exception ){
    if ( points.size() !=4 )  throw cv::Exception ( 9001,"point.size()!=4","MarkerDetector::warp",__FILE__,__LINE__ );
    //obtain the perspective transform
    Point2f  pointsRes[4],pointsIn[4];

    for ( int i=0;i<4;i++ ) pointsIn[i]=points[i];

    pointsRes[0]= ( Point2f ( 0,0 ) );
    pointsRes[1]= Point2f ( size.width-1,0 );
    pointsRes[2]= Point2f ( size.width-1,size.height-1 );
    pointsRes[3]= Point2f ( 0,size.height-1 );

    Mat M=getPerspectiveTransform ( pointsIn,pointsRes );
    cv::warpPerspective ( in, out,  M, size,cv::INTER_NEAREST );
    return true;
  }

}
