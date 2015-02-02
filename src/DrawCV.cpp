#include "DrawCV.h"
using namespace cv;

namespace argosServer{


  /********************************************************************************************
   *  SCREEN DISPLAY
   ********************************************************************************************/
  
  void DrawCV::draw3DAxis(cv::Mat& image, Paper& paper, CameraProjectorSystem& cameraProjector){
    
    CameraModel& camera = cameraProjector.getCamera();
    
    float size = paper.getPaperSize().width / 2;
    
    Mat objectPoints (4,3,CV_32FC1);
    objectPoints.at<float>(0,0) = 0;
    objectPoints.at<float>(0,1) = 0;
    objectPoints.at<float>(0,2) = 0;

    objectPoints.at<float>(1,0) = size;
    objectPoints.at<float>(1,1) = 0;
    objectPoints.at<float>(1,2) = 0;
    
    objectPoints.at<float>(2,0) = 0;
    objectPoints.at<float>(2,1) = size;
    objectPoints.at<float>(2,2) = 0;

    objectPoints.at<float>(3,0) = 0;
    objectPoints.at<float>(3,1) = 0;
    objectPoints.at<float>(3,2) = size;
    
    vector<Point2f> imagePoints;
    cv::projectPoints(objectPoints, paper.getRotVec(), paper.getTransVec(), camera.getDistortedCamMatrix(), camera.getDistCoeffs(), imagePoints);
    //draw lines of different colours
    cv::line(image, imagePoints[0], imagePoints[1], Scalar(0,0,255,255), 1, CV_AA);
    cv::line(image, imagePoints[0], imagePoints[2], Scalar(0,255,0,255), 1, CV_AA);
    cv::line(image, imagePoints[0], imagePoints[3], Scalar(255,0,0,255), 1, CV_AA);
    putText(image,"x", imagePoints[1],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255,255), 2);
    putText(image,"y", imagePoints[2],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,255,0,255), 2);
    putText(image,"z", imagePoints[3],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,0,0,255), 2);
  }

 
  void DrawCV::draw3DCube(cv::Mat& image, Paper& paper, CameraProjectorSystem& cameraProjector){

    Mat objectPoints (8,3,CV_32FC1);
  
    CameraModel& camera = cameraProjector.getCamera();

    double halfSize = paper.getPaperSize().width / 4;

    objectPoints.at<float>(0,0) = -halfSize;
    objectPoints.at<float>(0,1) = 0;
    objectPoints.at<float>(0,2) = -halfSize;
    objectPoints.at<float>(1,0) = halfSize;
    objectPoints.at<float>(1,1) = 0;
    objectPoints.at<float>(1,2) = -halfSize;
    objectPoints.at<float>(2,0) = halfSize;
    objectPoints.at<float>(2,1) = 0;
    objectPoints.at<float>(2,2) = halfSize;
    objectPoints.at<float>(3,0) = -halfSize;
    objectPoints.at<float>(3,1) = 0;
    objectPoints.at<float>(3,2) = halfSize;
  
    objectPoints.at<float>(4,0) = -halfSize;
    objectPoints.at<float>(4,1) = 2 * halfSize;
    objectPoints.at<float>(4,2) = -halfSize;
    objectPoints.at<float>(5,0) = halfSize;
    objectPoints.at<float>(5,1) = 2 * halfSize;
    objectPoints.at<float>(5,2) = -halfSize;
    objectPoints.at<float>(6,0) = halfSize;
    objectPoints.at<float>(6,1) = 2 * halfSize;
    objectPoints.at<float>(6,2) = halfSize;
    objectPoints.at<float>(7,0) = -halfSize;
    objectPoints.at<float>(7,1) = 2 * halfSize;
    objectPoints.at<float>(7,2) = halfSize;

    vector<Point2f> imagePoints;
    cv::projectPoints(objectPoints, paper.getRotVec(), paper.getTransVec(), camera.getDistortedCamMatrix(),  camera.getDistCoeffs(), imagePoints);
    //draw lines of different colours
    for (int i=0;i<4;i++)
      cv::line(image, imagePoints[i], imagePoints[(i+1)%4], Scalar(0,0,255,255),1,CV_AA);

    for (int i=0;i<4;i++)
      cv::line(image, imagePoints[i+4], imagePoints[4+(i+1)%4], Scalar(0,0,255,255),1,CV_AA);
    
    for (int i=0;i<4;i++)
      cv::line(image, imagePoints[i], imagePoints[i+4], Scalar(0,0,255,255),1,CV_AA);
    
  }
  
  void DrawCV::draw3DPaper(cv::Mat& image, Paper& paper, CameraProjectorSystem& cameraProjector){

    Mat objectPoints (8,3,CV_32FC1);
  
    CameraModel& camera = cameraProjector.getCamera();
  
    double halfSize = paper.getPaperSize().width / 6;
  
    double halfWidthSize  =  paper.getPaperSize().width / 2.;
    double halfHeightSize =  paper.getPaperSize().height / 2.;
  
    objectPoints.at<float>(0,0) = -halfWidthSize;
    objectPoints.at<float>(0,1) = 0;
    objectPoints.at<float>(0,2) = -halfHeightSize;
    objectPoints.at<float>(1,0) = halfWidthSize;
    objectPoints.at<float>(1,1) = 0;
    objectPoints.at<float>(1,2) = -halfHeightSize;
    objectPoints.at<float>(2,0) = halfWidthSize;
    objectPoints.at<float>(2,1) = 0;
    objectPoints.at<float>(2,2) = halfHeightSize;
    objectPoints.at<float>(3,0) = -halfWidthSize;
    objectPoints.at<float>(3,1) = 0;
    objectPoints.at<float>(3,2) = halfHeightSize;
  
    objectPoints.at<float>(4,0) = -halfWidthSize;
    objectPoints.at<float>(4,1) = 2 * halfSize;
    objectPoints.at<float>(4,2) = -halfHeightSize;
    objectPoints.at<float>(5,0) = halfWidthSize;
    objectPoints.at<float>(5,1) = 2 * halfSize;
    objectPoints.at<float>(5,2) = -halfHeightSize;
    objectPoints.at<float>(6,0) = halfWidthSize;
    objectPoints.at<float>(6,1) = 2 * halfSize;
    objectPoints.at<float>(6,2) = halfHeightSize;
    objectPoints.at<float>(7,0) = -halfWidthSize;
    objectPoints.at<float>(7,1) = 2 * halfSize;
    objectPoints.at<float>(7,2) = halfHeightSize;

    vector<Point2f> imagePoints;
    cv::projectPoints(objectPoints, paper.getRotVec(), paper.getTransVec(), camera.getDistortedCamMatrix(),  camera.getDistCoeffs(), imagePoints);
    //draw lines of different colours
    for (int i=0;i<4;i++)
      cv::line(image, imagePoints[i], imagePoints[(i+1)%4], Scalar(0,0,255,255),1,CV_AA);

    for (int i=0;i<4;i++)
      cv::line(image, imagePoints[i+4], imagePoints[4+(i+1)%4], Scalar(0,0,255,255),1,CV_AA);

    for (int i=0;i<4;i++)
      cv::line(image, imagePoints[i], imagePoints[i+4], Scalar(0,0,255,255),1,CV_AA);

  }

  void DrawCV::cameraPaper(cv::Mat& image, Paper& paper, CameraProjectorSystem& cameraProjector, const string& text){
  
    CameraModel& camera = cameraProjector.getCamera();
  
    double halfWidthSize  =  paper.getPaperSize().width / 2.;
    double halfHeightSize =  paper.getPaperSize().height / 2.;
  
    cv::Mat objPoints(4,3,CV_32FC1);
  
    objPoints.at<float>(0,0)=-halfWidthSize;
    objPoints.at<float>(0,1)=0;
    objPoints.at<float>(0,2)=-halfHeightSize;
    objPoints.at<float>(1,0)=-halfWidthSize;
    objPoints.at<float>(1,1)=0;
    objPoints.at<float>(1,2)=halfHeightSize;
    objPoints.at<float>(2,0)=halfWidthSize;
    objPoints.at<float>(2,1)=0;
    objPoints.at<float>(2,2)=halfHeightSize;
    objPoints.at<float>(3,0)=halfWidthSize;
    objPoints.at<float>(3,1)=0;
    objPoints.at<float>(3,2)=-halfHeightSize;

    vector<cv::Point2f> imagePoints;

    cv::projectPoints(objPoints, paper.getRotVec(), paper.getTransVec(), camera.getDistortedCamMatrix(), camera.getDistCoeffs(), imagePoints);
  
    vector<cv::Point> imageIntPoints;
    for (size_t i=0; i<imagePoints.size(); i++)
      imageIntPoints.push_back(imagePoints[i]);

  
    if(imagePoints.size() <= 0) return;
    //drawContour(image, imagePoints, Scalar(255,0,255));
    fillConvexPoly(image, &imageIntPoints[0], 4 , Scalar(255,0,255), 8, 0);
 
    //determine the centroid
    cv::Point centroid(0,0);
    for (int i=0; i<4; i++){
      centroid.x+=imagePoints[i].x;
      centroid.y+=imagePoints[i].y;
    }
    centroid.x/= 4.;
    centroid.y/= 4.;
    putText(image, text, centroid ,FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255),2);
  

    // for(size_t i = 0; i < imagePoints.size(); i++)
    //	  cv::circle(image, imagePoints[i], 10,  CV_RGB(255,255,255), -1, 8);
  
  }

  /********************************************************************************************
   * PROJECTOR  DISPLAY
   ********************************************************************************************/
  
  void DrawCV::projectPaper(cv::Mat& image, Paper& paper, CameraProjectorSystem& cameraProjector, cv::Mat& video,  cv::Mat& out){
  
    CameraModel& projector = cameraProjector.getProjector();

    double halfWidthSize  =  paper.getPaperSize().width / 2.;
    double halfHeightSize =  paper.getPaperSize().height / 2.;
  
    cv::Mat objPoints(4,3,CV_32FC1);

    objPoints.at<float>(0,0)=-halfWidthSize;
    objPoints.at<float>(0,1)=0;
    objPoints.at<float>(0,2)=-halfHeightSize;
    objPoints.at<float>(1,0)=-halfWidthSize;
    objPoints.at<float>(1,1)=0;
    objPoints.at<float>(1,2)=halfHeightSize;
    objPoints.at<float>(2,0)=halfWidthSize;
    objPoints.at<float>(2,1)=0;
    objPoints.at<float>(2,2)=halfHeightSize;
    objPoints.at<float>(3,0)=halfWidthSize;
    objPoints.at<float>(3,1)=0;
    objPoints.at<float>(3,2)=-halfHeightSize;

    vector<cv::Point2f> imagePoints;

    cv::projectPoints(objPoints, paper.getRotVec(), paper.getTransVec(), projector.getDistortedCamMatrix(), projector.getDistCoeffs(), imagePoints);
  
    vector<cv::Point> imageIntPoints;
    for (size_t i=0; i<imagePoints.size(); i++)
      imageIntPoints.push_back(imagePoints[i]);
  
    if(imagePoints.size() <= 0) return;
    if(imagePoints.size() !=4 ) return;
  
    //obtain the perspective transform
    Point2f  pointsRes[4],pointsIn[4];
  
    pointsIn[0]= ( Point2f ( 0,0 ) );
    pointsIn[1]= Point2f ( video.cols-1,0);
    pointsIn[2]= Point2f ( video.cols-1,video.rows-1 );
    pointsIn[3]= Point2f ( 0,video.rows-1);

    for ( int i=0;i<4;i++ ) {
      pointsRes[i] = imagePoints[i];
      //cout << "pointsRes: " << pointsRes[i] <<endl;
    }
    Mat M=getPerspectiveTransform ( pointsIn,pointsRes );
    ///cout << "M: " << M << endl;
    cv::Mat out1(480, 640,CV_8UC3);
    cv::warpPerspective ( video, out1,  M, Size(640,480),cv::INTER_NEAREST );

    double alpha = 1;
    double beta = 1;
    double gamma = 0.0; //offset 
  
    addWeighted(image,alpha,out1,beta,gamma,image);
 
  }

  void DrawCV::projectPaper0(cv::Mat& image, Paper& paper, CameraProjectorSystem& cameraProjector, int index){
  
    CameraModel& projector = cameraProjector.getProjector();

    double halfWidthSize  =  paper.getPaperSize().width / 2.;
    double halfHeightSize =  paper.getPaperSize().height / 2.;

  
    vector<string> invoices = {"NONE","Archivar en Carpeta Roja","Dar la vuelta a la factura","Validar Importe"};
    vector<string> providers = {" ","Proveedor: NEOBIZ","Proveedor: SINOVO","Proveedor: ACTIVE"};

    cv::Mat objPoints(4,3,CV_32FC1);

    objPoints.at<float>(0,0)=-halfWidthSize;
    objPoints.at<float>(0,1)=0;
    objPoints.at<float>(0,2)=-halfHeightSize;
    objPoints.at<float>(1,0)=-halfWidthSize;
    objPoints.at<float>(1,1)=0;
    objPoints.at<float>(1,2)=halfHeightSize;
    objPoints.at<float>(2,0)=halfWidthSize;
    objPoints.at<float>(2,1)=0;
    objPoints.at<float>(2,2)=halfHeightSize;
    objPoints.at<float>(3,0)=halfWidthSize;
    objPoints.at<float>(3,1)=0;
    objPoints.at<float>(3,2)=-halfHeightSize;

    vector<cv::Point2f> imagePoints;

    cv::projectPoints(objPoints, paper.getRotVec(), paper.getTransVec(), projector.getDistortedCamMatrix(), projector.getDistCoeffs(), imagePoints);
  
    vector<cv::Point> imageIntPoints;
    for (size_t i=0; i<imagePoints.size(); i++)
      imageIntPoints.push_back(imagePoints[i]);
  
  
    if(imagePoints.size() <= 0) return;
    // Fill Rectangle
    //fillConvexPoly(image, &imageIntPoints[0], 4 ,  CV_RGB(150,150,150), 8, 0);
  
    //determine the centroid
    cv::Point centroid(0,0);
    for (int i=0; i<4; i++){
      centroid.x+=imagePoints[i].x;
      centroid.y+=imagePoints[i].y;
    }
    centroid.x/= 4.;
    centroid.y/= 4.;
 
    if (index == 2){
      putText(image, providers[index], cv::Point(centroid.x-105,centroid.y-25) ,FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(160,160,160),2);
      putText(image, "[AYUDA]", cv::Point(centroid.x-80,centroid.y) ,FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(160,160,160),2);
      putText(image, invoices[index],  cv::Point(centroid.x-105,centroid.y+25) ,FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(160,160,160),2);
    }
    else{
      putText(image, providers[index], cv::Point(centroid.x-105,centroid.y-25) ,FONT_HERSHEY_SIMPLEX, 0.5,CV_RGB(160,160,160),2);
      putText(image, invoices[index],  cv::Point(centroid.x-105,centroid.y) ,FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(160,160,160),2);
    }
    // for(size_t i = 0; i < imagePoints.size(); i++)
    //	  cv::circle(image, imagePoints[i], 10,  CV_RGB(255,255,255), -1, 8);
  
    // Contour
    drawContour(image, imagePoints, CV_RGB(90,90,90));
  
    // Circles in corners
    //for(size_t i = 0; i < imagePoints.size(); i++)
    //cv::circle(image, imagePoints[i], 3,  CV_RGB(150,150,150), -1, 8);
  
  }











  void DrawCV::drawInvoice(cv::Mat& image, Paper& paper, CameraProjectorSystem& cameraProjector){
  
    CameraModel& projector = cameraProjector.getProjector();

    double halfWidthSize  =  paper.getPaperSize().width / 2.;
    double halfHeightSize =  paper.getPaperSize().height / 2.;
  
    cv::Mat objPoints(5,3,CV_32FC1);

    objPoints.at<float>(0,0)=-halfWidthSize;
    objPoints.at<float>(0,1)=0;
    objPoints.at<float>(0,2)=-halfHeightSize;
    objPoints.at<float>(1,0)=-halfWidthSize;
    objPoints.at<float>(1,1)=0;
    objPoints.at<float>(1,2)=halfHeightSize;
    objPoints.at<float>(2,0)=halfWidthSize;
    objPoints.at<float>(2,1)=0;
    objPoints.at<float>(2,2)=halfHeightSize;
    objPoints.at<float>(3,0)=halfWidthSize;
    objPoints.at<float>(3,1)=0;
    objPoints.at<float>(3,2)=-halfHeightSize;

    objPoints.at<float>(4,0)=(paper.getPaperSize().width / 4.);
    objPoints.at<float>(4,1)=0;
    objPoints.at<float>(4,2)=(paper.getPaperSize().height / 4.);

    vector<cv::Point2f> imagePoints;
    cv::projectPoints(objPoints, paper.getRotVec(), paper.getTransVec(), projector.getDistortedCamMatrix(), projector.getDistCoeffs(), imagePoints);
  
    if(imagePoints.size() <= 0) return;
 
    // Fill invoice
    vector<cv::Point> imageIntPoints;
    for (size_t i=0; i<imagePoints.size(); i++)
      imageIntPoints.push_back(imagePoints[i]);
  
    fillConvexPoly(image, &imageIntPoints[0], 4 , CV_RGB(0,255,255), 8, 0);
  
    putText(image, "Archivar en Carperta ROJA",imagePoints[4], FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(0,0,0),2);


    // Contour
    //drawContour(image, imagePoints, CV_RGB(255,255,255));
  
    // Circles in corners
    //for(size_t i = 0; i < imagePoints.size(); i++)
    //  cv::circle(image, imagePoints[i], 3,  CV_RGB(255,255,255), -1, 8);
  }



  void DrawCV::drawContour(cv::Mat &in, vector<cv::Point2f>& contour, cv::Scalar color){  
  
    unsigned int size = 20;
    int thickness = 4;

    cv::Point lTemp1;
    cv::Point lTemp2;
  
    cv::Point lTemp3;
    cv::Point lTemp4;

  
    for(unsigned int i = 0; i<contour.size(); i++){
      LineIterator iterator(in,contour[i],contour[(i+1)%contour.size()]);
      for(unsigned int j = 0; j < iterator.count; j++, iterator++){
	if ( j == 0 )   
	  lTemp1 =  iterator.pos();
      
	if ( j == size) 
	  lTemp2 =  iterator.pos();
      
	if ( j == iterator.count - size) 
	  lTemp3 =  iterator.pos();
      
	if ( j == (iterator.count - 1)) 
	  lTemp4 =  iterator.pos();
      }
      line(in,lTemp1,lTemp2,color,thickness,8);
      line(in,lTemp3,lTemp4,color,thickness,8);
    }
  }


  /* Draw Dotted contour
     void DrawCV::drawContour(cv::Mat &in, vector<cv::Point2f>& contour, cv::Scalar color){  
  
     int lenghOfDots = 40;
     int thickness = 2;

     cv::Point lTemp1;
     cv::Point lTemp2;
  
     for(unsigned int i = 0; i<contour.size(); i++){
     LineIterator iterator(in,contour[i],contour[(i+1)%contour.size()]);

     for(int j = 0; j < iterator.count; j++, iterator++){
     if ( j%lenghOfDots == 0 )  {
     lTemp1 =  iterator.pos();
     //	cout << "Point1 "<< j << iterator.pos() << endl;
     }
     if ( j%lenghOfDots == lenghOfDots/2) { 
     //cout << "Point2 " << j << iterator.pos() << endl;
     lTemp2 =  iterator.pos();
     //cout << "draw line" << endl;
     line(in,lTemp1,lTemp2,color,thickness,8);
     }
     }
     }
     }
  */



  /*
    void DrawCV::drawContour(cv::Mat &in, vector<cv::Point2f>& contour, cv::Scalar color){  
    for ( unsigned int i=0;i<contour.size();i++ )
    cv::line (in,contour[i],contour[(i+1)%contour.size()],color,3);
    }
  */

  void DrawCV::drawBoardMarks(cv::Mat& image, int thickness, int size){
  
    int width =  image.cols;
    int height = image.rows;
  
    //Top Left mark
    vector<cv::Point> topLeftMark;
    topLeftMark.push_back(cv::Point(0,0));
    topLeftMark.push_back(cv::Point(size,0));
    topLeftMark.push_back(cv::Point(size,thickness));
    topLeftMark.push_back(cv::Point(thickness,thickness));
    topLeftMark.push_back(cv::Point(thickness,size));
    topLeftMark.push_back(cv::Point(0,size));

    cv::fillConvexPoly(image, &topLeftMark[0], 6 , CV_RGB(160,160,160), 8, 0);

    //Top right mark
    vector<cv::Point> topRightMark;
    topRightMark.push_back(cv::Point(width-size,0));
    topRightMark.push_back(cv::Point(width,0));
    topRightMark.push_back(cv::Point(width,size));
    topRightMark.push_back(cv::Point(width-thickness,size));
    topRightMark.push_back(cv::Point(width-thickness,thickness));
    topRightMark.push_back(cv::Point(width-size,thickness));

    cv::fillConvexPoly(image, &topRightMark[0], 6 , CV_RGB(160,160,160), 8, 0);

    //Bottom left mark
    vector<cv::Point> bottomLeftMark;
    bottomLeftMark.push_back(cv::Point(0,height-size));
    bottomLeftMark.push_back(cv::Point(thickness,height-size));
    bottomLeftMark.push_back(cv::Point(thickness,height-thickness));
    bottomLeftMark.push_back(cv::Point(size,height-thickness));
    bottomLeftMark.push_back(cv::Point(size,height));
    bottomLeftMark.push_back(cv::Point(0,height));

    cv::fillConvexPoly(image, &bottomLeftMark[0], 6 , CV_RGB(160,160,160), 8, 0);

    //Bottom right mark
    vector<cv::Point> bottomRightMark;
    bottomRightMark.push_back(cv::Point(width-thickness,height-size));
    bottomRightMark.push_back(cv::Point(width,height-size));
    bottomRightMark.push_back(cv::Point(width,height));
    bottomRightMark.push_back(cv::Point(width-size,height));
    bottomRightMark.push_back(cv::Point(width-size,height-thickness));
    bottomRightMark.push_back(cv::Point(width-thickness,height-thickness));

    cv::fillConvexPoly(image, &bottomRightMark[0], 6 , CV_RGB(160,160,160), 8, 0);
  
    //putText(image, "Proyecto ARgos - Catedra Indra-UCLM", cv::Point(285,460),FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(160,160,160),2);
 
    /*  
    //Left column
    vector<cv::Point> leftColumn;
    leftColumn.push_back(cv::Point(0,0));
    leftColumn.push_back(cv::Point(0,480));
    leftColumn.push_back(cv::Point(33,480));
    cv::fillConvexPoly(image, &leftColumn[0], 3 , CV_RGB(0,255,255), 8, 0);
  
    //Right column
    vector<cv::Point> rightColumn;
    rightColumn.push_back(cv::Point(640,0));
    rightColumn.push_back(cv::Point(640,480));
    rightColumn.push_back(cv::Point(570,480));
    cv::fillConvexPoly(image, &rightColumn[0], 3 , CV_RGB(0,255,255), 8, 0);
    */


  }

}



