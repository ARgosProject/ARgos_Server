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
    HandDetector::getInstance();

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
    
    Paper paperDetected;
    if(!paperList.empty()){
      paperDetected = paperList.back();
      Log::paper(paperDetected);
    }
    numInvoices = paperList.size();
    
    
    //Finger Detection ---------
    cv::Point fingerPoint;
    HandDetector::getInstance().detectFinger(currentFrame,fingerPoint);
    
    cout << "fingerPoint:" << fingerPoint << endl;
    
    

    
    //CameraModel& camera = cameraProjector.getCamera();
    //CameraModel& projector  = cameraProjector.getProjector();
    
    //triangulate(fingerPoint, paperDetected.getRotVec(),paperDetected.getTransVec(), 
    //		camera.getDistortedIntrinsics().getCameraMatrix());
      
    //vector<cv::Point2f> out;
    
    //cv::projectPoints(cv::Mat(ObjPoints),
    //		      cameraProjector.getRotObjToProj(), cameraProjector.getTransObjToProj(),
    //     	      projector.getDistortedIntrinsics().getCameraMatrix(),
    //		      projector.getDistCoeffs(),
    //		      out);
    
    //calibrationProjector.setCandidateImagePoints(out);

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
      paperList[i].setFingerPoint(fingerPoint);
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
  
  /*
  cv::Point3f triangulatePoint(const vector<cv::Point2f>& ps, const vector<cv::Mat>& Rs, 
			       const vector<cv::Mat>& ts, const cv::Mat& cameraMatrix){
    Mat_<double> K(cameraMatrix);
    
    if( ps.size() > 2 ){
      Mat_<double> L(ps.size()*3, 4), U, evalues;
      Mat_<double> P(3,4), Rt(3,4), Rt_part1=Rt.colRange(0,3), Rt_part2=Rt.colRange(3,4);
      
      for( size_t i = 0; i < ps.size(); i++ ){
	double x = ps[i].x, y = ps[i].y;
	Rs[i].convertTo(Rt_part1, Rt_part1.type());
	ts[i].convertTo(Rt_part2, Rt_part2.type());
	P = K*Rt;
      
	for( int k = 0; k < 4; k++ ){
	  L(i*3, k) = x*P(2,k) - P(0,k);
	  L(i*3+1, k) = y*P(2,k) - P(1,k);
	  L(i*3+2, k) = x*P(1,k) - y*P(0,k);
	}
      }
      
      cv::eigen(L.t()*L, evalues, U);
      CV_Assert(evalues(0,0) >= evalues(3,0));
      
      double W = fabs(U(3,3)) > FLT_EPSILON ? 1./U(3,3) : 0;
      return cv::Point3f((float)(U(3,0)*W), (float)(U(3,1)*W), (float)(U(3,2)*W));
    }
    
    else{
      cv::Mat_<float> iK = K.inv();
      cv::Mat_<float> R1t = cv::Mat_<float>(Rs[0]).t();
      cv::Mat_<float> R2t = cv::Mat_<float>(Rs[1]).t();
      cv::Mat_<float> m1 = (cv::Mat_<float>(3,1) << ps[0].x, ps[0].y, 1);
      cv::Mat_<float> m2 = (cv::Mat_<float>(3,1) << ps[1].x, ps[1].y, 1);
      cv::Mat_<float> K1 = R1t*(iK*m1), K2 = R2t*(iK*m2);
      cv::Mat_<float> B1 = -R1t*cv::Mat_<float>(ts[0]);
      cv::Mat_<float> B2 = -R2t*cv::Mat_<float>(ts[1]);
      
      return findRayIntersection(*K1.ptr<cv::Point3f>(), *B1.ptr<cv::Point3f>(),
				 *K2.ptr<cv::Point3f>(), *B2.ptr<cv::Point3f>());
    }
  }
  
  
  cv::Point3f findRayIntersection(cv::Point3f k1, cv::Point3f b1, cv::Point3f k2, cv::Point3f b2){    
    float a[4], b[2], x[2];
    a[0] = k1.dot(k1);
    a[1] = a[2] = -k1.dot(k2);
    a[3] = k2.dot(k2);
    b[0] = k1.dot(b2 - b1);
    b[1] = k2.dot(b1 - b2);
    cv::Mat_<float> A(2, 2, a), B(2, 1, b), X(2, 1, x);
    cv::solve(A, B, X);
  
    float s1 = X.at<float>(0, 0);
    float s2 = X.at<float>(1, 0);
    return (k1*s1 + b1 + k2*s2 + b2)*0.5f;
  }
  */

  void Core::calculate3DPointFrom2D(const cv::Point& ps, const cv::Mat& Rs, const cv::Mat& ts, const cv::Mat& cameraMatrix){
      
    cv::Mat uvPoint = cv::Mat::ones(3,1,cv::DataType<double>::type); //u,v,1
    uvPoint.at<double>(0,0) = ps.x; //got this point using mouse callback
    uvPoint.at<double>(1,0) = ps.y;
    cv::Mat tempMat, tempMat2;
    double s;
    tempMat = Rs.inv() * cameraMatrix.inv() * uvPoint;
    tempMat2 = Rs.inv() * ts;
    //s = 285 + tempMat2.at<double>(2,0); //285 represents the height Zconst
    s = tempMat2.at<double>(2,0); //285 represents the height Zconst
    cout << "s0: " << s << endl; 
    s /= tempMat.at<double>(2,0);
    cout << "s: " << s << endl;
    cout << "P = " << Rs.inv() * (s * cameraMatrix.inv() * uvPoint - ts) << std::endl;
    
  }
}

