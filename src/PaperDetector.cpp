#include <valarray>
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <iostream>
#include "PaperDetector.h"
#include "Log.h"

using namespace std;
using namespace cv;

namespace argosServer{

  PaperDetector::PaperDetector() {
    thresMethod = CANNY;
    // With balck panel
    //thresParam1 = 100;
    //thresParam2 = 60;
    thresParam1 = 50;
    thresParam2 = 255;

    minContourValue = 0.1;
    maxContourValue = 0.8;
    minContourAreaValue = 41000;
    maxContourAreaValue = 49000;

    //historicWeights = {0.020,0.030,0.040,0.060,0.070,0.080,0.090,0.110,0.240,0.260};
    historicWeights = {0.05,0.05,0.10,0.30,0.50};


    typeThres = 2;
    max_Type = 2;
    max_Level = 255;

    //configWindow = "Configuration";
    //cv::namedWindow(configWindow, CV_WINDOW_AUTOSIZE);  // configuration window

    // Trackbar Threshold
    trackbarType = "Type:\n 0:Fixed\n1:Adapt\n2:Canny";
    trackbarParm1 = "Parameter 1";
    trackbarParm2 = "Parameter 2";

    cv::createTrackbar(trackbarType, configWindow, &typeThres, max_Type, NULL);
    cv::createTrackbar(trackbarParm1, configWindow, &thresParam1, max_Level, NULL);
    cv::createTrackbar(trackbarParm2, configWindow, &thresParam2, max_Level, NULL);
  }

  PaperDetector::~PaperDetector() {}

  /************************************
   *
   * Main detection function. Performs all steps
   *
   *
   ************************************/
  void PaperDetector::detect(const cv::Mat& currentFrame,vector<Paper>& detectedPapers, CameraProjectorSystem& cameraProjector,cv::Size paperSizeMeters, bool screenExtrinsics, bool setYPerperdicular) throw (cv::Exception) {
    //clear input data
    detectedPapers.clear();
    currentFrame.copyTo(debug);
    //cv::cvtColor(debug, debug, CV_GRAY2BGR);

    //- Convert to greyScale  -------------------------------
    if (currentFrame.type() == CV_8UC3)
      cv::cvtColor(currentFrame, greyFrame, CV_BGR2GRAY);
    else
      greyFrame = currentFrame;

    cv::Mat thres0;
    greyFrame.copyTo(thres0);
    //- Thresholding image ----------------------------------------------------------------
    //cv::threshold(greyFrame, outThres, 127, 255, 0);
    //cv::threshold(greyFrame, outThres, 200.0, 255.0, THRESH_BINARY);
    //cv::threshold (greyFrame, outThres, 165, 255, CV_THRESH_BINARY);
    //thresHold(2, greyFrame, outThres, 50, 255);
    thresHold(2, greyFrame, outThres, thresParam1, thresParam2);
    //thresHold(thresMethod, greyFrame, outThres, thresParam1, thresParam2);
    Mat element = getStructuringElement( MORPH_RECT,
                                         Size(2,2));
    cv::dilate(outThres, outThres, element);
    outThres.copyTo(thres2);
    outThres.copyTo(thres0);
    //imshow("Threshold", outThres);
    //waitKey(1);

    //find all rectangles in the thresholdes image
    vector<PaperCandidate> PaperCandidates;
    vector<PaperCandidate> OutPaperCandidates;

    //- Find Contours--------------------------------------------------------------------
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    //cv::findContours(thres2, contours, hierarchy, CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
    //cv::findContours(thres2, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    cv::findContours(thres2, contours, hierarchy, CV_RETR_EXTERNAL , CV_CHAIN_APPROX_NONE);
    //cv::findContours(thres2, contours, hierarchy, CV_RETR_EXTERNAL , CV_CHAIN_APPROX_SIMPLE);
    //cv::findContours(thres2, contours, hierarchy, CV_RETR_EXTERNAL , CV_CHAIN_APPROX_TC89_L1);

    // Find Rectangles ----------------------------------------------------
    //findRectangles(contours, PaperCandidates);

    // Find Rectangles - Convex Hull contours ------------------------------------------------
    convexHullContours(contours, PaperCandidates);
    //for (size_t i = 0; i < contours.size(); i++)
    //  drawContour(debug,contours[i], CV_RGB(255,0,255));
    //imshow("debug", debug);

    if (PaperCandidates.size() == 0) {
      Log::info("HoughSpace");
      // Hough Line Transform
      houghLineTransform(thres0, PaperCandidates);
    }

    //cv::waitKey(1);


    // Sort the points in anti-clockwise order
    valarray<bool> swapped(false,PaperCandidates.size());  //used later
    for (unsigned int i=0;i<PaperCandidates.size();i++) {
      //trace a line between the first and second point.
      //if the thrid point is at the right side, then the points are anti-clockwise
      double dx1 = PaperCandidates[i][1].x - PaperCandidates[i][0].x;
      double dy1 = PaperCandidates[i][1].y - PaperCandidates[i][0].y;
      double dx2 = PaperCandidates[i][2].x - PaperCandidates[i][0].x;
      double dy2 = PaperCandidates[i][2].y - PaperCandidates[i][0].y;
      double o = ( dx1*dy2 )- ( dy1*dx2 );

      if (o  < 0.0) {     //if the third point is in the left side, then sort in anti-clockwise order
        swap (PaperCandidates[i][1],PaperCandidates[i][3] );
        swapped[i]=true;
        //sort the contour points
        //reverse(MarkerCanditates[i].contour.begin(),MarkerCanditates[i].contour.end());//????

      }
    }

    //deletes the one with smaller perimeter
    vector<bool> toRemove (PaperCandidates.size(), false);
    for (int i=0;i<int ( PaperCandidates.size() )-1; i++) {
      if (!toRemove[i+1]) {
        if (perimeter(PaperCandidates[i]) > perimeter(PaperCandidates[i+1]))
          toRemove[i+1]=true;
        else toRemove[i]=true;
      }
    }
    //remove the papers marker
    removeElements (PaperCandidates, toRemove);

    // Calculate Extrinsics------------------------------------------------
    ///detect the position of detected papers
    if (cameraProjector.isValid()  && (paperSizeMeters.width > 0 && paperSizeMeters.height > 0)) {
      for(unsigned int i=0; i < PaperCandidates.size(); i++)
        PaperCandidates[i].calculateExtrinsics(paperSizeMeters, cameraProjector, setYPerperdicular, screenExtrinsics);
    }

    // Perception Historic
    for (unsigned int i=0; i<PaperCandidates.size(); i++) {
      if (PaperCandidates.size() == 1) {
        if (historicRot.size() == 0 && historicTrans.size() == 0) {
          historicRot.clear();
          historicRot.clear();
          //cout << "historicRot.size() " << historicRot.size() << endl;
          //cout << "historicTrans.size() " << historicTrans.size() << endl;
          for(int j=0;j<5;j++) {
            historicRot.push_back(PaperCandidates[i].getRotVec());
            historicTrans.push_back(PaperCandidates[i].getTransVec());
          }
        }
        //cout << "historicRot.size() " << historicRot.size() << endl;
        //cout << "historicTrans.size() " << historicTrans.size() << endl;

        historicRot.pop_front();
        historicTrans.pop_front();

        historicRot.push_back(PaperCandidates[i].getRotVec());
        historicTrans.push_back(PaperCandidates[i].getTransVec());

        cv:: Mat avgRotHist = calculateAverage(historicRot);
        //cout << "avgRotHist: " << avgRotHist << endl;

        cv:: Mat avgTransHist= calculateAverage(historicTrans);
        //cout << "avgTransHist: " << avgTransHist << endl;

        cv::Mat absR;
        cv::Mat absT;
        cv::absdiff(avgRotHist,PaperCandidates[i].getRotVec(),absR);
        cv::absdiff(avgTransHist,PaperCandidates[i].getTransVec(),absT);
        //cout << "absR.at<float>(0): " << absR.at<float>(0) << endl;
        //cout << "absR.at<float>(1): " << absR.at<float>(1) << endl;
        //cout << "absR.at<float>(2): " << absR.at<float>(2) << endl;
        //cout << "absT.at<float>(0): " << absT.at<float>(0) << endl;
        //cout << "absT.at<float>(1): " << absT.at<float>(1) << endl;
        //cout << "absT.at<float>(2): " << absT.at<float>(2) << endl;
        if(absR.at<float>(0) < 0.04 && absR.at<float>(1) < 0.04 && absR.at<float>(2) < 0.04 &&
           absT.at<float>(0) < 0.04 && absT.at<float>(1) < 0.04 && absT.at<float>(2) < 0.04) {
          PaperCandidates[i].setRotVec(avgRotHist);
          PaperCandidates[i].setTransVec(avgTransHist);
        }
        else {
          historicRot.clear();
          historicTrans.clear();
          //cout << "cleaning historic" << endl;
        }

      }
      //cout << "PaperCandidates " << PaperCandidates[i] << endl;

      // Return vector
      detectedPapers.push_back(PaperCandidates[i]);
    }
  }

  cv::Mat PaperDetector::calculateAverage(std::deque<cv::Mat> &matrix) {
    cv::Mat avg(1,3,CV_32FC1,Scalar::all(0.0));
    for(unsigned int i=0; i<matrix.size(); i++) {
      avg = avg + (matrix[i] * historicWeights[i]);
    }
    //cout << "calculateAverage " << endl;
    return avg;
  }

  /************************************
   *
   *
   *
   *
   ************************************/

  void PaperDetector::convexHullContours(vector<vector<cv::Point> > &contours, vector<PaperCandidate> &PaperCandidates) {
    vector<cv::Point> hull;
    //calculate the  min_max contour sizes
    //int minSize = minContourValue * std::max(currentFrame.cols,currentFrame.rows) * 4;
    //int maxSize = maxContourValue * std::max(currentFrame.cols,currentFrame.rows) * 4;

    for (size_t i = 0; i < contours.size(); i++) {
      //drawContour(debug, contours[i], CV_RGB(0,255,0));
      //waitKey(1);
      // if(minSize < contours[i].size()  && contours[i].size() < maxSize){
      //cout << "contour Area: " << cv::contourArea(contours[i]);
      if ((minContourAreaValue < cv::contourArea(contours[i])) && (cv::contourArea(contours[i]) < maxContourAreaValue)) {
        cv::convexHull(contours[i],hull);
        //drawContour(debug, hull, CV_RGB(255,0,0));
        //imshow("debug", debug);
        //waitKey(1);
        cv::approxPolyDP(hull, hull, double(0.1*arcLength(hull,true)),true);

        if(hull.size() == 4) {
          //cout << "contour Area: " << cv::contourArea(contours[i]);
          //drawApproxCurve(debug, hull, CV_RGB(255,0,0));   //red
          //imshow("debug", debug);
          //waitKey(1);
          PaperCandidates.push_back(PaperCandidate());
          PaperCandidates.back().idx = i;
          for (int j=0; j<4; j++) {
            PaperCandidates.back().push_back(Point2f(hull[j].x,hull[j].y));
          }
          //cout<<"ContourAdded"<<endl;
        }
      }
    }
  }

  void PaperDetector::findRectangles(vector<vector<cv::Point> > &contours, vector<PaperCandidate> &PaperCandidates) {
    vector<cv::Point>  approxCurve;
    //calculate the  min_max contour sizes
    int minSize = minContourValue * 640 * 4;
    int maxSize = maxContourValue * 640 * 4;

    for(unsigned int i=0; i<contours.size(); i++) {
      if(minSize < (int)contours[i].size() && (int)contours[i].size() < maxSize) {
        //approximate to a poligon
        cv::approxPolyDP(contours[i],approxCurve, double(contours[i].size())*0.02, true);
        //drawApproxCurve(debug, approxCurve, Scalar(255,0,255));
        //check that the poligon has 4 points
        if (approxCurve.size() == 4) {
          //drawContour(debug,contours[i],Scalar(255,0,225));
          //and is convex
          if (cv::isContourConvex(Mat(approxCurve))) {
            //drawApproxCurve(debug,approxCurve,Scalar(255,0,255));
            //ensure that the   distace between consecutive points is large enough
            float minDist = 1e10;
            for (int j=0; j<4; j++) {
              //float d = std::sqrt((float)(approxCurve[j].x-approxCurve[(j+1)%4].x) *
              //(approxCurve[j].x-approxCurve[(j+1)%4].x) +
              //        (approxCurve[j].y-approxCurve[(j+1)%4].y) *
              //        (approxCurve[j].y-approxCurve[(j+1)%4].y));
              //norm(Mat(approxCurve[i]),Mat(approxCurve[(i+1)%4]));
              //if (d < minDist) minDist = d;
              //---------------------
              cv::Point side = approxCurve[i] - approxCurve[(i+1)%4];
              float squaredSideLength = side.dot(side);
              minDist = std::min(minDist, squaredSideLength);
            }
            //check that distance is not very small
            if (minDist > 10) {
              //add the points
              //cout<<"ADDED"<<endl;
              drawApproxCurve(debug,approxCurve,CV_RGB(243,249,21));  // yellow

              PaperCandidates.push_back ( PaperCandidate() );
              PaperCandidates.back().idx = i;
              for (int j=0; j<4; j++) {
                PaperCandidates.back().push_back (Point2f(approxCurve[j].x,approxCurve[j].y));
              }
            }

            double maxCosine = 0;
            for(int j = 2; j < 5; j++) {
              double cosine = fabs(angle(approxCurve[j%4], approxCurve[j-2], approxCurve[j-1]));
              maxCosine = std::max(maxCosine, cosine);
            }

            if(maxCosine < 0.3)
              drawApproxCurve(debug,approxCurve,CV_RGB(255,0,0));   //red
          }
        }
      }
    }
  }

  void PaperDetector::thresHold(int method, const Mat& grey, Mat& out, double param1, double param2) throw (cv::Exception) {
    if (param1 == -1) param1 = thresParam1;
    if (param2 == -1) param2 = thresParam2;

    if (grey.type() != CV_8UC1)
      throw cv::Exception (9001,"grey.type() != CV_8UC1", "thresHold", __FILE__, __LINE__);

    switch (method) {
    case FIXED_THRES:
      cv::threshold (grey, out, param1, 255, CV_THRESH_BINARY_INV);
      break;
    case ADPT_THRES:
      //ensure that _thresParam1%2==1
      if (param1 < 3) param1 = 3;
      else if (((int) param1) %2 != 1) param1 = (int)(param1 + 1);
      cv::adaptiveThreshold(grey, out, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, param1, param2);
      break;
    case CANNY:
      cv::Canny (grey, out, param1, param2);
      break;
    }
  }

  /************************************
   *
   *
   *
   *
   ************************************/
  void PaperDetector::drawAllContours ( Mat input, std::vector<std::vector<cv::Point> > &contours ) {
    drawContours ( input,  contours, -1,Scalar ( 255,0,255 ) );
  }

  /************************************
   *
   *
   *
   *
   ************************************/
  void PaperDetector:: drawContour ( Mat &in,vector<Point>  &contour,Scalar color ) {
    for ( unsigned int i=0;i<contour.size();i++ ){
      cv::rectangle ( in,contour[i],contour[i],color );
    }
  }

  void  PaperDetector:: drawApproxCurve ( Mat &in,vector<Point>  &contour,Scalar color ) {
    for ( unsigned int i=0;i<contour.size();i++ ){
      cv::line ( in,contour[i],contour[ ( i+1 ) %contour.size() ],color );
    }
  }
  void  PaperDetector:: drawApproxCurve ( Mat &in,vector<Point2f>  &contour,Scalar color ) {
    for ( unsigned int i=0;i<contour.size();i++ ){
      cv::line ( in,contour[i],contour[ ( i+1 ) %contour.size() ],color );
    }
  }


  /************************************
   *
   *
   *
   *
   ************************************/

  void PaperDetector::draw ( Mat out,const vector<Paper> &markers ) {
    for ( unsigned int i=0;i<markers.size();i++ ){
      cv::line ( out,markers[i][0],markers[i][1],cvScalar ( 255,0,0 ),2,CV_AA );
      cv::line ( out,markers[i][1],markers[i][2],cvScalar ( 255,0,0 ),2,CV_AA );
      cv::line ( out,markers[i][2],markers[i][3],cvScalar ( 255,0,0 ),2,CV_AA );
      cv::line ( out,markers[i][3],markers[i][0],cvScalar ( 255,0,0 ),2,CV_AA );
    }
  }

  void PaperDetector::setMinMaxSize(float min ,float max )throw(cv::Exception) {
    if (min<=0 || min>1) throw cv::Exception(1," min parameter out of range","PaperDetector::setMinMaxSize",__FILE__,__LINE__);
    if (max<=0 || max>1) throw cv::Exception(1," max parameter out of range","PaperDetector::setMinMaxSize",__FILE__,__LINE__);
    if (min>max) throw cv::Exception(1," min>max","PaperDetector::setMinMaxSize",__FILE__,__LINE__);
    minContourValue = min;
    maxContourValue = max;
  }

  double PaperDetector::angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 ) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
  }


  int PaperDetector::perimeter(vector<Point2f> &a) {
    int sum=0;
    for (unsigned int i=0; i<a.size(); i++) {
      int i2 = (i+1) % a.size();
      sum+= sqrt ( ( a[i].x-a[i2].x ) * ( a[i].x-a[i2].x ) + ( a[i].y-a[i2].y ) * ( a[i].y-a[i2].y ) ) ;
    }
    return sum;
  }

  // Hough Line Transform

  // the long nasty wikipedia line-intersection equation...bleh...
  vector<Point> PaperDetector::lineToPointPair(Vec2i line) {
    vector<Point> points;

    float r = line[0], t = line[1];
    double cos_t = cos(t), sin_t = sin(t);
    double x0 = r*cos_t, y0 = r*sin_t;
    double alpha = 1000;

    points.push_back(Point2f(x0 + alpha*(-sin_t), y0 + alpha*cos_t));
    points.push_back(Point2f(x0 - alpha*(-sin_t), y0 - alpha*cos_t));

    return points;
  }

  bool PaperDetector::acceptLinePair(cv::Vec2f line1, cv::Vec2f line2, float minTheta) {
    float theta1 = line1[1], theta2 = line2[1];

    if(theta1 < minTheta)
      theta1 += CV_PI; // dealing with 0 and 180 ambiguities...

    if(theta2 < minTheta)
      theta2 += CV_PI; // dealing with 0 and 180 ambiguities...

    return abs(theta1 - theta2) > minTheta;
  }

  Point PaperDetector::computeIntersect(cv::Vec4i a, cv::Vec4i b) {
    Point o1(a[0],a[1]);
    Point p1(a[2],a[3]);
    Point o2(b[0],b[1]);
    Point p2(b[2],b[3]);

    Point x = o2 - o1;
    Point d1 = p1 - o1;
    Point d2 = p2 - o2;

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < 100)
      //cout << "cross: " << cross<< endl;
      //if (abs(cross) < /*EPS*/1e-8)
      return cv::Point(-1, -1);

    double t1 = (x.x * d2.y - x.y * d2.x) / cross;
    Point intersect = o1 + d1 * t1;

    return intersect;
  }

  void PaperDetector::sortCorners(std::vector<cv::Point2f>& corners, cv::Point2f center) {
    std::vector<cv::Point2f> top, bot;

    for (size_t i = 0; i < corners.size(); i++) {
      if (corners[i].y < center.y)
        top.push_back(corners[i]);
      else
        bot.push_back(corners[i]);
    }

    cv::Point2f tl = top[0].x > top[1].x ? top[1] : top[0];
    cv::Point2f tr = top[0].x > top[1].x ? top[0] : top[1];
    cv::Point2f bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
    cv::Point2f br = bot[0].x > bot[1].x ? bot[0] : bot[1];

    corners.clear();
    corners.push_back(tl);
    corners.push_back(tr);
    corners.push_back(br);
    corners.push_back(bl);
  }


  void PaperDetector::houghLineTransform(cv::Mat& image, vector<PaperCandidate> &PaperCandidates) {
    cv::Mat  image_c;
    //cv::Mat  image_d;
    //cv::Mat dst = image.clone();

    //cvtColor(dst, dst, CV_GRAY2BGR);
    //cv::Mat dst2(600,800,CV_8UC3);
    //dst2 = Scalar::all(0);

    vector<cv::Point> projectionLimits;
    projectionLimits.push_back(cv::Point(266,102));
    projectionLimits.push_back(cv::Point(554,100));
    projectionLimits.push_back(cv::Point(565,338));
    projectionLimits.push_back(cv::Point(250,338));

    //drawApproxCurve (dst, projectionLimits, CV_RGB(0,0,255));

    //drawApproxCurve (dst2, projectionLimits, CV_RGB(0,0,255));

    //cv::Mat  image_d;

    //cv::Point center(0,0);
    //cv::adaptiveThreshold(image, image_c, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 7, 7);
    //cv::threshold (image, image_c, thresParam1, 255, CV_THRESH_BINARY);
    //thresHold(typeThres,image, image_c, thresParam1, thresParam2);
    //cv::blur( image, image_d, Size(3,3),Point(-1,-1));
    cv::Canny(image, image_c, 25, 180);
    cv::Mat element = getStructuringElement( MORPH_RECT, Size(2.5,2.5), Point(0,0));
    cv::dilate(image_c, image_c, element);
    //cv::imshow("thres", image_c);

    std::vector<cv::Vec4i> lines;
    //cv::HoughLinesP(image_c, lines, 1, CV_PI/180, 70, 3, 10);
    cv::HoughLinesP(image_c, lines, 1, CV_PI/180,40,40, 8);

    std::vector<cv::Vec4i> lines2;
    for (size_t i = 0; i < lines.size(); i++) {
      cv::Vec4i v = lines[i];
      if ((pointPolygonTest(projectionLimits,cv::Point(v[0], v[1]),false) >= 0) &&      // line is into projection limits
          (pointPolygonTest(projectionLimits,cv::Point(v[2], v[3]),false) >= 0) &&
          (length(v) > 45)) {                                                          // line is longer than a threshold value
        //cv::line(dst, cv::Point(v[0], v[1]), cv::Point(v[2], v[3]), CV_RGB(0,0,255),3);
        lines2.push_back(v);
      }
    }

    std::vector<cv::Vec4i> lines3;
    for (size_t i = 0; i < lines2.size(); i++) {
      for (size_t j = i+1; j < lines2.size(); j++) {
        cv::Point pt = computeIntersect(lines2[i], lines2[j]);
        if (pt.x < 0 && pt.y < 0) {
          //cv::line(dst, cv::Point(lines2[i][0], lines2[i][1]), cv::Point(lines2[i][2], lines2[i][3]), CV_RGB(255,0,0),3);
          //cv::line(dst, cv::Point(lines2[j][0], lines2[j][1]), cv::Point(lines2[j][2], lines2[j][3]), CV_RGB(0,255,0),3);
          lines3.push_back(lines2[i]);
        }
      }
    }


    std::vector<cv::Vec4i> lines4;
    for (size_t i = 0; i < lines3.size(); i++) {
      for (size_t j = i+1; j < lines3.size(); j++) {
        float angle = angleBetween2Lines(lines3[i], lines3[j]);
        //cout << "angle: " << angle << endl;
        if (4.6 < angle && angle < 4.8) {
          //cv::line(dst, cv::Point(lines3[i][0], lines3[i][1]), cv::Point(lines3[i][2], lines3[i][3]), CV_RGB(255,0,0),3);
          //cv::line(dst, cv::Point(lines2[j][0], lines2[j][1]), cv::Point(lines2[j][2], lines2[j][3]), CV_RGB(0,255,0),3);
          //cv::Vec4i v = lines3[i];
          //cv::line(dst, cv::Point(v[0], v[1]), cv::Point(v[2], v[3]), CV_RGB(255,0,0),3);
          lines4.push_back(lines3[i]);
        }
      }
    }


    vector<cv::Point> corners;
    for (size_t i = 0; i < lines4.size(); i++) {
      cv::Vec4i v = lines4[i];
      if(length(v) > 45) {
        //cv::line(dst, cv::Point(v[0], v[1]), cv::Point(v[2], v[3]), CV_RGB(0,255,0),3);
        corners.push_back(cv::Point(v[0], v[1]));
        corners.push_back(cv::Point(v[2], v[3]));
      }
    }

    //for (size_t i = 0; i < corners.size(); i++) {
    // circle(dst2, corners[i], 3, CV_RGB(255,0,0), -1);
    //}
    /*
    cv::Mat dst3(600,800,CV_8UC1);
    cv::Mat dst4(600,800,CV_8UC3);
    dst4 = Scalar::all(0);


    dst2.copyTo(dst3);
    //- Find Contours--------------------------------------------------------------------
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    //cv::findContours(thres2, contours, hierarchy, CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
    //cv::findContours(thres2, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    cv::findContours(dst3, contours, hierarchy, CV_RETR_EXTERNAL , CV_CHAIN_APPROX_NONE);
    //cv::findContours(thres2, contours, hierarchy, CV_RETR_EXTERNAL , CV_CHAIN_APPROX_SIMPLE);
    //cv::findContours(thres2, contours, hierarchy, CV_RETR_EXTERNAL , CV_CHAIN_APPROX_TC89_L1);


    for (size_t i = 0; i < contours.size(); i++)
      drawContour(dst4,contours[i], CV_RGB(255,0,0));

    //vector<cv::Point>  approxCurve;
    //for(unsigned int i=0; i<contours.size(); i++) {
      //if(minSize < (int)contours[i].size() && (int)contours[i].size() < maxSize) {
      //approximate to a poligon
      //cv::approxPolyDP(contours[i], approxCurve, double(contours[i].size())*0.02, true);
      //drawApproxCurve(dst, approxCurve, Scalar(255,0,255));
      //check that the poligon has 4 points
    //}

    std::vector<cv::Point> corners;
    for (size_t i = 0; i < lines3.size(); i++) {
      for (size_t j = i+1; j < lines3.size(); j++) {
        float angle = angleBetween2Lines(lines3[i], lines3[j]);
        if (1.04 < angle && angle < 1.74) {
          cv::Point pt = computeIntersect(lines3[i], lines3[j]);
          if (pt.x >= 0 && pt.y >= 0 && (pointPolygonTest(projectionLimits, pt, false) >= 0)){
            circle(dst, pt, 3, CV_RGB(255,0,0));
            corners.push_back(pt);
          }
        }
      }
    }
    */

    if (corners.size() > 0) {
      vector<cv::Point> hull;
      cv::convexHull(corners, hull, true);
      std::vector<cv::Point> approx;
      cv::approxPolyDP(hull, approx, double(0.1*arcLength(hull, true)), true);
      //drawApproxCurve(dst2, approx, CV_RGB(0,255,0));

      if(approx.size() == 4 &&
         (minContourAreaValue < cv::contourArea(approx)) && (cv::contourArea(approx) < maxContourAreaValue)) {
        //drawApproxCurve(dst2, approx, CV_RGB(0,255,0));
        //drawApproxCurve(debug, approx , CV_RGB(255,0,0));   //blue
        //imshow("debug", debug);
        PaperCandidates.push_back(PaperCandidate());
        PaperCandidates.back().idx = 0;
        PaperCandidates.back().push_back(Point2f(approx[2].x,approx[2].y));
        PaperCandidates.back().push_back(Point2f(approx[1].x,approx[1].y));
        PaperCandidates.back().push_back(Point2f(approx[0].x,approx[0].y));
        PaperCandidates.back().push_back(Point2f(approx[3].x,approx[3].y));

      }
    }
    /*
    //std::vector<cv::Point2f> approx;
    //cv::approxPolyDP(cv::Mat(corners), approx, cv::arcLength(cv::Mat(corners), true) * 0.01, true);
    //drawApproxCurve(dst2, approx, Scalar(255,255,255));
    //if (approx.size() != 4) {
    //  std::cout << "The object is not quadrilateral!" << std::endl;
    //  return;
    //}

    // Get mass center
    //for (size_t i = 0; i < corners.size(); i++)
    //  center += corners[i];
    //center *= (1. / corners.size());

    //sortCorners(corners, center);
    //for (size_t i = 0; i < corners.size(); i++)
    //cv::circle(dst2, corners[i], 3, CV_RGB(0,0,255), 2);
    */

    //cv::imshow("image", dst);
    //cv::imshow("image2", dst2);
    //cv::imshow("image4", dst4);
    //cv::imshow("quadrilateral", quad);
    //waitKey(1);
  }

  bool PaperDetector::isInto (Mat &contour, vector<Point2f> &b) {
    for (unsigned int i=0; i<b.size(); i++)
      if (pointPolygonTest(contour, b[i], false) >0) return true;
    return false;
  }


  // Length of a line (between two points)
  float PaperDetector::length(cv::Vec4i v) {
    float dx =  v[2] - v[0];
    float dy =  v[3] - v[1];
    return abs(sqrt((dx * dx)  + (dy * dy)));
  }


  float PaperDetector::angleBetween2Lines(cv::Vec4i line1, cv::Vec4i line2) {
    /* The atan2 function eases the pain of dealing with atan.
       It is declared as double atan2(double y, double x) and converts rectangular coordinates (x,y) to the angle
       theta from the polar coordinates (r,theta)
    */
    float angle1 = atan2(line1[1] - line1[3],     // Y1 - Y2
                         line1[0] - line1[2]);    // X1 - X2
    float angle2 = atan2(line2[1] - line2[3],     // Y1 - Y2
                         line2[0] - line2[2]);    // X1 - X2
    return abs(angle1-angle2);
  }


}
