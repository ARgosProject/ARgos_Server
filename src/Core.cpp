#include "Core.h"
#include "Log.h"
#include "HandDetector.h"
#include "ScriptManager.h"
#include "ConfigManager.h"
#include "DrawCV.h"

namespace argosServer {

  Core::Core() {
    //- Read configuration file ----
    Log::info("Reading configuration file... ");
    ConfigManager::loadConfiguration("data/config.xml");

    //- Camera & Projector Parameters ----
    Log::info("Loading camera & projector parameters... ");
    cameraProjector.setCalibrationsPath("data/calibrations/");
    cameraProjector.load(ConfigManager::getCameraCalibration(),
                         ConfigManager::getProjectorCalibration(),
                         ConfigManager::getExtrinsics());

    if(!cameraProjector.isValid()) {
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
    HandDetector::getInstance();

    lastSearch = "NONE";
    isPreviousPaperDetected = false;
    numFrames = 0;
    previousNumInvoices = 0;
    initVideoConference = false;

    projectionLimits.push_back(cv::Point(266,102));
    projectionLimits.push_back(cv::Point(554,100));
    projectionLimits.push_back(cv::Point(565,338));
    projectionLimits.push_back(cv::Point(250,338));
  }

  Core::~Core() {
  }

  vector<Paper>& Core::update(cv::Mat& currentFrame, bool &initVideoConference) {

    numFrames++;

    //Detection of papers in the image passed
    PaperDetector::getInstance().detect(currentFrame, paperList, cameraProjector,
                                        ConfigManager::getPaperSize(), ConfigManager::getOutputDisplay());
    //imshow("Screen", currentFrame);
    //cv::waitKey(1);

    Paper paperDetected;
    if(!paperList.empty()) {
      paperDetected = paperList.back();
      Log::paper(paperDetected);
    }
    numInvoices = paperList.size();


    //Finger Detection ---------
    cv::Mat fingerMat;

    cv::Point fingerPoint(0,0);
    cv::Point2f fingerPoint3D;
    //cv::Point3f point3f;
    //vector<cv::Point2f> out;

    HandDetector::getInstance().detectFinger(currentFrame,fingerPoint);
    cout << "fingerPoint:" << fingerPoint << endl;

    if(pointPolygonTest(projectionLimits,fingerPoint,false) >= 0) {
      //CameraModel& camera = cameraProjector.getCamera();
      CameraModel& projector  = cameraProjector.getProjector();

      fingerMat = calculate3DPointFrom2D(fingerPoint, paperDetected.getRotVec(),paperDetected.getTransVec(),
                                          projector.getDistortedIntrinsics().getCameraMatrix());

      //cout << "fingerMat:" << fingerMat << endl;
      fingerPoint3D = cv::Point2f(fingerMat.at<float>(0,0), fingerMat.at<float>(0,1));
      fingerPoint3D = fingerPoint3D * (-1);
      //point3f =  cv::Point3f((fingerMat.at<float>(0,0)), fingerMat.at<float>(0,1), fingerMat.at<float>(0,2));
      cout << "finger3D:" << fingerPoint3D << endl;

      /*
        vector<cv::Point3f> objectPoints;
        objectPoints.push_back(point3f);

        cv::projectPoints(objectPoints,
        cameraProjector.getCamToProjRotation(), cameraProjector.getCamToProjTranslation(),
                        projector.getDistortedIntrinsics().getCameraMatrix(),
                        projector.getDistCoeffs(),
                        out);
      */
    }
    //DrawCV::draw3DAxisInPoint(currentFrame, paperDetected, cameraProjector,fingerPoint3D2);
    //cout << "out: " << out << endl;
    //cv::imshow("Test", currentFrame);
    //cv::waitKey(1);
    //}

    //ScriptManager::getInstance().update();

    // CheckRegion TEST
    //checkRegion(currentFrame,paperList);


    if (initVideoConference) {
      if (paperList.size() == 0)
        initVideoConference = false;
    }
    else {
      if (paperList.empty()) {
        //cout << "Detecting NONE" << endl;
        isPreviousPaperDetected = false;
        previousNumInvoices = 0;
        numFrames = 0;
      }
      else {
        if (!isPreviousPaperDetected || (numInvoices != previousNumInvoices)) {// || (numFrames > 30)) {
          isPreviousPaperDetected = true;
          numFrames = 0;
          previousNumInvoices = numInvoices;

          DocumentDetector::getInstance().detect(currentFrame,paperList);

          invoicesIndex.clear();
          //cout <<  "invoicesIndex clear "<< endl;
          for (unsigned int i=0; i < paperList.size(); i++) {
            initVideoConference = (paperList[i].getId() == 999);
            invoicesIndex.push_back(paperList[i].getId());
            //cout <<  "invoicesIndex " << i << ":"<< invoicesIndex[i]<< endl;
          }
        }
      }
    }

    for (unsigned int i=0; i < paperList.size(); i++) {
      Log::info(std::to_string(invoicesIndex[i]));
      paperList[i].setId(invoicesIndex[i]);
      paperList[i].setFingerPoint(fingerPoint3D);
    }
    //paperList.clear();

    return paperList;

  }

  void Core::checkRegion(cv::Mat& currentFrame, vector<Paper>& detectedPapers) {
    cv::Mat paperExtraction;
    cv::Mat signature;
    if (detectedPapers.size() == 1)
      warp(currentFrame,paperExtraction,Size(600,425), detectedPapers.back());

    getRectSubPix(paperExtraction, Size(101,50), Point2f(416.16,313.13), signature);

    //imshow("extraction", paperExtraction);
    //imshow("signature", signature);
    //cv::waitKey(1);

  }


  bool Core::warp (const cv::Mat& in, cv::Mat& out, Size size, vector<Point2f> points) throw (cv::Exception) {
    if (points.size() !=4)  throw cv::Exception (9001,"point.size() != 4", "PaperDetector::warp", __FILE__, __LINE__ );
    //obtain the perspective transform
    Point2f  pointsRes[4], pointsIn[4];

    for (int i=0; i<4; i++) pointsIn[i] = points[i];

    pointsRes[0] = (Point2f (0,0));
    pointsRes[1] = Point2f(size.width-1,0);
    pointsRes[2] = Point2f(size.width-1,size.height-1);
    pointsRes[3] = Point2f(0,size.height-1);

    Mat M = getPerspectiveTransform (pointsIn, pointsRes);
    cv::warpPerspective (in, out, M, size, cv::INTER_NEAREST);
    return true;
  }


  cv::Mat Core::calculate3DPointFrom2D(const cv::Point& ps, const cv::Mat& Rs, const cv::Mat& ts, const cv::Mat& camMatrix) {
    Mat_<float> cameraMatrix(camMatrix);
    cv::Mat rotationMatrix(3,3,cv::DataType<float>::type);
    cv::Rodrigues(Rs,rotationMatrix);

    cv::Mat uvPoint = cv::Mat::ones(3,1,cv::DataType<float>::type); //u,v,1
    uvPoint.at<float>(0,0) = ps.x;
    uvPoint.at<float>(1,0) = ps.y;

    cv::Mat tempMat, tempMat2;
    float s;
    tempMat = rotationMatrix.inv() * cameraMatrix.inv() * uvPoint;
    tempMat2 = rotationMatrix.inv() * ts;
    s = 0 + tempMat2.at<float>(2,0); // 0 represents the height Zconst
    s /= tempMat.at<float>(2,0);
    cv::Mat point3d =  rotationMatrix.inv() * (s * cameraMatrix.inv() * uvPoint - ts);
    //cout << "Mat" << point3d << endl;
    return point3d;
  }
}
