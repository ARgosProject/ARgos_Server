#include <valarray>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "HandDetector.h"
#include "Log.h"
//#include "opencv2/contrib/contrib.hpp"
#define SAMPLES 7

using namespace std;
using namespace cv;

namespace argosServer {

  HandDetector::HandDetector() {

    configWindow = "Configuration";

    cv::namedWindow(configWindow, CV_WINDOW_AUTOSIZE);       // configuration window

    for(int i=0; i<SAMPLES; i++) {
      c_lower[i][0]=7;
      c_upper[i][0]=12;
      c_lower[i][1]=30;
      c_upper[i][1]=40;
      c_lower[i][2]=80;
      c_upper[i][2]=80;
    }

    Hlow = 0;
    Htop = 25;
    Slow = 80;
    Stop = 255;
    Vlow = 0;
    Vtop = 255;

    createTrackbar("H Low",configWindow,&Hlow,255);
    createTrackbar("H High",configWindow,&Htop,255);
    createTrackbar("S Low",configWindow,&Slow,255);
    createTrackbar("S High",configWindow,&Stop,255);
    createTrackbar("V Low",configWindow,&Vlow,255);
    createTrackbar("V High",configWindow,&Vtop,255);
    /*
    createTrackbar("H Low",configWindow,&c_lower[0][0],255);
    createTrackbar("H High",configWindow,&c_upper[0][0],255);
    createTrackbar("L Low",configWindow,&c_lower[0][1],255);
    createTrackbar("L High",configWindow,&c_upper[0][1],255);
    createTrackbar("S Low",configWindow,&c_lower[0][2],255);
    createTrackbar("S High",configWindow,&c_upper[0][2],255);

    cv::Size roiSize(5,5);
    roi.push_back(cv::Rect(320,287, roiSize.width, roiSize.height));
    roi.push_back(cv::Rect(367,288, roiSize.width, roiSize.height));
    roi.push_back(cv::Rect(442,295, roiSize.width, roiSize.height));
    roi.push_back(cv::Rect(331,330, roiSize.width, roiSize.height));
    roi.push_back(cv::Rect(373,333, roiSize.width, roiSize.height));
    roi.push_back(cv::Rect(431,330, roiSize.width, roiSize.height));
    roi.push_back(cv::Rect(379,310, roiSize.width, roiSize.height));
    */
  }

  HandDetector::~HandDetector() {
  }

  int HandDetector::getMedian(vector<int> values){
    int median;
    size_t size = values.size();
    sort(values.begin(), values.end());
    if (size % 2 == 0)
      median = values[size / 2 - 1];
    else
      median = values[size / 2];
    return median;
  }

  void HandDetector::getAvgColor(cv::Mat& in, cv::Rect& roi, int avg[3]) {
    cv::Mat roiMat(in, roi);

    vector<int> hm;
    vector<int> sm;
    vector<int> lm;
    // generate vectors
    for(int i=0; i < roiMat.rows-1; i++) {
      for(int j=0; j < roiMat.cols-1; j++) {
        hm.push_back(roiMat.data[roiMat.channels() * (roiMat.cols*i + j) + 0]);
        sm.push_back(roiMat.data[roiMat.channels() * (roiMat.cols*i + j) + 1]);
        lm.push_back(roiMat.data[roiMat.channels() * (roiMat.cols*i + j) + 2]);
      }
    }

    avg[0] = getMedian(hm);
    avg[1] = getMedian(sm);
    avg[2] = getMedian(lm);
  }

  void HandDetector::normalizeColors() {
    // copy all boundries read from trackbar to all of the different boundries
    for(int i=1; i<SAMPLES; i++) {
      for(int j=0; j<3; j++) {
        c_lower[i][j] = c_lower[0][j];
        c_upper[i][j] = c_upper[0][j];
      }
    }
    // normalize all boundries so that threshold is whithin 0-255
    for(int i=0; i<SAMPLES; i++) {
      if((avgColor[i][0] - c_lower[i][0]) < 0)
        c_lower[i][0] = avgColor[i][0];

      if((avgColor[i][1] - c_lower[i][1]) < 0)
        c_lower[i][1] = avgColor[i][1];

      if((avgColor[i][2] - c_lower[i][2]) < 0)
        c_lower[i][2] = avgColor[i][2];

      if((avgColor[i][0] + c_upper[i][0]) > 255)
        c_upper[i][0] = 255-avgColor[i][0];

      if((avgColor[i][1] + c_upper[i][1]) > 255)
        c_upper[i][1] = 255-avgColor[i][1];

      if((avgColor[i][2] + c_upper[i][2]) > 255)
        c_upper[i][2] = 255-avgColor[i][2];
    }
  }

  void HandDetector::produceBinaries(cv::Mat& image) {
    cv::Scalar lowerBound;
    cv::Scalar upperBound;
    cv::Mat composedImage;
    vector<cv::Mat> thresholdImage;

    for(int i=0; i<SAMPLES; i++) {
      normalizeColors();
      lowerBound = cv::Scalar(avgColor[i][0] - c_lower[i][0], avgColor[i][1] - c_lower[i][1], avgColor[i][2] - c_lower[i][2]);
      upperBound = cv::Scalar(avgColor[i][0] + c_upper[i][0], avgColor[i][1] + c_upper[i][1], avgColor[i][2] + c_upper[i][2]);

      thresholdImage.push_back(cv::Mat(image.rows, image.cols, CV_8U));
      cv::inRange(image, lowerBound, upperBound, thresholdImage[i]);
    }

    thresholdImage[0].copyTo(composedImage);

    for(int i=1; i<SAMPLES; i++)
      composedImage += thresholdImage[i];

    cv::medianBlur(composedImage, composedImage, 7);
    cv::imshow("Median", composedImage);
  }

  void HandDetector::detectFinger(const cv::Mat& currentFrame, cv::Point& fingerPosition) {
    vector<cv::Point> handLimits;
    handLimits.push_back(cv::Point(262,0));
    handLimits.push_back(cv::Point(655,0));
    handLimits.push_back(cv::Point(659,353));
    handLimits.push_back(cv::Point(237,352));

    currentFrame.copyTo(output);

    //cv::cvtColor(currentFrame, imageHLS, CV_BGR2HLS);

    cv::cvtColor(currentFrame, imageHLS, CV_BGR2HSV);

    Mat H, S,Seq, L, Heq, Hx, Sx, outImage;
    vector<Mat> channels(3);
    // split img:
    split(imageHLS, channels);
    // get the channels (dont forget they follow BGR order in OpenCV)
    H = channels[0];
    S = channels[1];
    L = channels[2];

    equalizeHist(H, Heq);
    cv::Mat element = getStructuringElement(MORPH_ELLIPSE, Size(7,7));
    cv::erode(Heq, Heq, element);

    bilateralFilter(Heq, Hx, 7, 10, 7);
    bilateralFilter(S, Sx, 7, 24, 7);

    channels[0] = Hx;
    channels[1] = Sx;

    merge(channels, outImage);

    cv::Scalar lower;
    cv::Scalar upper;

    cv::Mat thresholdImage;

    lower = cv::Scalar(Hlow,Slow,Vlow);
    upper = cv::Scalar(Htop,Stop,Vtop);


    cv::inRange(outImage, lower, upper, thresholdImage);

    cv::Mat kernel2 = getStructuringElement(MORPH_ELLIPSE, cv::Size(3,3));

    morphologyEx(thresholdImage, thresholdImage, MORPH_OPEN, kernel2, cv::Point (-1,-1), 3);
    morphologyEx(thresholdImage, thresholdImage, MORPH_CLOSE, kernel2, cv::Point (-1,-1), 3);

    //for(int i=0; i < SAMPLES; i++) {
    //cv::rectangle(output, roi[i], CV_RGB(0,255,0));
    //getAvgColor(imageHLS, roi[i], avgColor[i]);
      //cout << "avgColor: " <<  avgColor[i][0] << " " << avgColor[i][1] << " " << avgColor[i][2] << endl;
    //}
    //cout << endl;
    //cv::blur(imageHLS,imageHLS,Size(3,3));

    //produceBinaries(imageHLS);

    //blur(currentFrame, hsvFrame, Size(5,5));

    /*

    cv::Mat skin = GetSkin(currentFrame);

    cv::Mat detection, mask;

    currentFrame.copyTo(detection);

    int dilation_size = 4;
    int erosion_size = 4;

    //erode(skin, skin, cv::Mat());
    //dilate(skin,skin, cv::Mat());

    erode(skin, skin, getStructuringElement(MORPH_RECT,
                                            Size(2*erosion_size+1, 2*erosion_size+1),
                                            Point(erosion_size, erosion_size)));

    dilate(skin, skin, getStructuringElement(MORPH_RECT,
                                             Size(2*dilation_size+1, 2*dilation_size+1),
                                             Point(dilation_size, dilation_size)));
    */
    cv::Mat detection, thres;


    cv::Mat mask(cv::Size(800,600),CV_8UC1,Scalar::all(0));
    for(int x = 0; x < mask.cols; x++){
      for(int y = 0; y < mask.rows; y++){
        if (pointPolygonTest(handLimits, cv::Point(x,y), false) > 0)
          mask.at<uchar>(y,x) = 255;
      }
    }

    thresholdImage.copyTo(thres,mask);
    thres.copyTo(detection);
    cvtColor(detection, detection, CV_GRAY2BGR);
    //imshow("mask" , mask);
    //imshow("Hx" , thresholdImage);


    vector<cv::Point> fingerTips;

    //- Find Contours--------------------------------------------------------------------
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    findContours(thres, contours, hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    int indexMaxContour = findBiggestContour(contours);

    if (indexMaxContour != -1 && contours[indexMaxContour].size() > 10 ) {
      //cv::Point armcenter;
      // cv::RotatedRect contourcenter;
      drawContours(detection,contours,indexMaxContour,CV_RGB(255,0,0),3);
      //contourcenter =  minAreaRect(contours[indexMaxContour]);

      //Point2f vertices[4];
      //contourcenter.points(vertices);
      //for (int i = 0; i < 4; i++)
      //line(detection, vertices[i], vertices[(i+1)%4], CV_RGB(0,0,255),2);


      //armcenter.x = contourcenter.center.x;
      //armcenter.y = contourcenter.center.y;
      //circle(detection,armcenter,10,CV_RGB(255,255,255),-1,8,0);

      getconvexhull(contours[indexMaxContour], detection, fingerTips);


      for (vector<Point>::iterator c = fingerTips.begin(); c < fingerTips.end(); c++) {
        cv::Point v=(*c);
        circle(detection,v,5,CV_RGB(0,255,0),-1,CV_AA,0);
      }

      if (fingerTips.empty() )
        fingerPosition = cv::Point(-1,-1);
      else
        fingerPosition =  fingerTips.back();
    }
    imshow("dt" , detection);
    //imshow("original" , output);
    //imshow("out" , out);
    //imshow("H" , Hx);
    //imshow("x",thresholdImage);
    //imshow("Seq" , Sx);
    //imshow("L" , L);
    //imshow("mask" , mask);
    waitKey(1);

  }

  bool HandDetector::R1(int R, int G, int B) {
    //bool e1 = (R>95) && (G>40) && (B>20) && ((max(R,max(G,B)) - min(R, min(G,B)))>15) && (abs(R-G)>15) && (R>G) && (R>B);
    //bool e2 = (R>220) && (G>210) && (B>170) && (abs(R-G)<=15) && (R>B) && (G>B);
    bool e3 = (R>70) && (G>60) && (B>40) && (abs(R-G)<=20) && (R>B) && (G>B);
    return e3; //(e1||e2);
  }

  bool HandDetector::R2(float Y, float Cr, float Cb) {
    bool e3 = Cr <= 1.5862*Cb+20;
    bool e4 = Cr >= 0.3448*Cb+76.2069;
    bool e5 = Cr >= -4.5652*Cb+234.5652;
    bool e6 = Cr <= -1.15*Cb+301.75;
    bool e7 = Cr <= -2.2857*Cb+432.85;
    return (e3 && e4 && e5 && e6 && e7);
  }

  bool HandDetector::R3(float H, float S, float V) {
    //return (H<25) || (H > 230);
    return (H<31) && (H > 25);
  }

  cv::Mat HandDetector::GetSkin(cv::Mat const &src) {
    // allocate the result matrix
    //cv::Mat dst = src.clone();
    cv::Mat mask(600,800,CV_8UC1,Scalar::all(255));

    cv::Vec3b cwhite = cv::Vec3b::all(255);
    //cv::Vec3b cblack = cv::Vec3b::all(0);

    cv::Mat src_ycrcb, src_hsv, src_hsl;
;
    // OpenCV scales the YCrCb components, so that they cover the whole value range of [0,255], so there's
    // no need to scale the values:
    cvtColor(src, src_ycrcb, CV_BGR2YCrCb);
    // OpenCV scales the Hue Channel to [0,180] for 8bit images, so make sure we are operating on
    // the full spectrum from [0,360] by using floating  point precision:
    src.convertTo(src_hsv, CV_32FC3);
    cvtColor(src_hsv, src_hsv, CV_BGR2HSV);
    // Now scale the values between [0,255]:
    normalize(src_hsv, src_hsv, 0.0, 255.0, NORM_MINMAX, CV_32FC3);

    src.convertTo(src_hsl, CV_32FC3);
    cvtColor(src_hsl, src_hsl, CV_BGR2HLS);
    // Now scale the values between [0,255]:
    normalize(src_hsl, src_hsl, 0.0, 255.0, NORM_MINMAX, CV_32FC3);


    for(int i = 0; i < src.rows; i++) {
      for(int j = 0; j < src.cols; j++) {

        Vec3b pix_bgr = src.ptr<Vec3b>(i)[j];
        int B = pix_bgr.val[0];
        int G = pix_bgr.val[1];
        int R = pix_bgr.val[2];
        // apply rgb rule
        bool a = R1(R,G,B);

        Vec3b pix_ycrcb = src_ycrcb.ptr<Vec3b>(i)[j];
        int Y = pix_ycrcb.val[0];
        int Cr = pix_ycrcb.val[1];
        int Cb = pix_ycrcb.val[2];
        // apply ycrcb rule
        bool b = R2(Y,Cr,Cb);

        Vec3f pix_hsv = src_hsv.ptr<Vec3f>(i)[j];
        float H = pix_hsv.val[0];
        float S = pix_hsv.val[1];
        float V = pix_hsv.val[2];
        // apply hsv rule
        bool c = R3(H,S,V);

        if(!(a && b && c))
          //mask.at<uchar>(i,j) = 0;
          mask.ptr<Vec3b>(i)[j] = cwhite;
      }
    }
    return mask;
  }

  void HandDetector::getconvexhull(vector<cv::Point> &contour, cv::Mat& image, vector<cv::Point> &fingerTips) {

    vector<cv::Point> palm;
    vector<int> hullIndex;
    vector<cv::Point> hull;

    cv::RotatedRect contourRect;
    contourRect =  minAreaRect(contour);

    //int tolerance =  contourRect.size.height/5;
    //float angleTolerance = 95;

    drawApproxCurve(image, hull, Scalar(255,255,255));
    cv::convexHull(contour,hullIndex,false, false);
    cv::convexHull(contour,hull,false, true);
    //std::vector<cv::Point> approx;
    //cv::approxPolyDP(hull, approx, double(0.1*arcLength(hull,true)),true);
    drawApproxCurve(image, hull, CV_RGB(0,0,255));
    /*
    std::vector<cv::Vec4i> convexityDefects;
    std::vector<cv::Vec4i> newDefects;

    cv::convexityDefects(contour, hullIndex, convexityDefects);

    for (size_t i = 0; i < convexityDefects.size(); i++) {
      cv::Vec4i defect = convexityDefects[i];

      cv::Point start(contour[defect[0]]);
      cv::Point end(contour[defect[1]]);
      cv::Point farthest(contour[defect[2]]);
      //float depth = defect[3]/256.0;

      //if(depth < 50){
      //	circle(image,start,5,CV_RGB(0,255,0),-1,CV_AA,0);
      //circle(image,end,5,CV_RGB(0,0,255),-1,CV_AA,0);
      //}
      //if(depth > 10) {
      //circle(image,farthest,5,CV_RGB(0,0,255),-1,CV_AA,0);
      //palm.push_back(farthest);
      // }

      if(distanceP2P(start,farthest) > tolerance &&
         distanceP2P(end, farthest) > tolerance &&
         getAngle(start, farthest, end ) < angleTolerance) {

        //circle(image,farthest,5,CV_RGB(0,0,255),-1,CV_AA,0);

        newDefects.push_back(defect);
      }
    }

    for (size_t i = 0; i < newDefects.size(); i++) {
      cv::Vec4i defect = newDefects[i];

      cv::Point start(contour[defect[0]]);
      cv::Point end(contour[defect[1]]);
      cv::Point farthest(contour[defect[2]]);
      //float depth = defect[3]/256.0;

      if(i == 0) {
        fingerTips.push_back(start);
        circle(image,start,5,CV_RGB(0,255,0),-1,CV_AA,0);
      }
      fingerTips.push_back(end);
      circle(image,end,5,CV_RGB(0,255,0),-1,CV_AA,0);
    }
    */
    //if(fingerTips.size()==0) {
      //int xTol= contourRect.size.height/5;
    cv::Point maxP;
    maxP.y = 0;

    for (vector<Point>::iterator c = contour.begin(); c < contour.end(); c++) {
      cv::Point v=(*c);
      if(v.y > maxP.y){
        maxP = v;
        //cout << minP.x<<endl;
      }
    }
      /*
        int n=0;
        d=hullP[cIdx].begin();
        for (vector<Point>::iterator c = hull.begin(); c < hull.end(); c++){
        cv::Point v=(*d);
        cout<<"x " << v.x << " y "<<  v.y << " highestpY " << highestP.y<< "xtol "<<xtol<<endl;

        if(v.y<highestP.y+xTol && v.y!=highestP.y && v.x!=highestP.x)
        n++;
        }

        if(n==0) {
        fingerTips.push_back(highestP);
        }
      */
    fingerTips.push_back(maxP);
    //}

  }

  //cv::Point2f mincirclecenter;
  //cv::RotatedRect palmcenter;
  //cv::Point mincirclecenter2, center;
  //float radius;
  /*
    if(palm.size() > 1) {
    minEnclosingCircle(palm,mincirclecenter,radius);
    mincirclecenter2.x = cvRound(mincirclecenter.x);
    mincirclecenter2.y = cvRound(mincirclecenter.y);
    circle(image,mincirclecenter2,5,CV_RGB(0,255,0),4,8,0);
    circle(image,mincirclecenter2,10,CV_RGB(255,128,255),4,8,0);
    palmcenter =  minAreaRect(palm);
    center.x = std::round(palmcenter.center.x);
    center.y = std::round(palmcenter.center.y);
    ellipse(image,palmcenter,CV_RGB(128,128,255),2,CV_AA);
    circle(image,center,10,CV_RGB(128,128,255),-1,8,0);
    }
  */

  void HandDetector::fingertip(vector<cv::Point> &contour, cv::Mat& image,cv:: Point armcenter ) {
    int dotproduct,i;
    float length1,length2,angle,minangle,length;
    cv::Point vector1,vector2,min,minp1,minp2;
    vector<cv::Point> fingertip;
    cv::Point p1,p2,p;
    vector<int> tiplocation;
    int count = 0;
    bool signal = false;
    vector<cv::Point>fingerseq;
    //
    // p1, p, p2 forms a triangle. we calculate the angle of it to decide if it might be a fingertip or not.
    //
    for(size_t i=0; i<contour.size(); i++) {
      p1 = contour[i];
      p  = contour[(i+20) % contour.size()];
      p2 = contour[(i+40)  % contour.size()];
      vector1.x = p.x - p1.x;
      vector1.y = p.y - p1.y;
      vector2.x = p.x - p2.x;
      vector2.y = p.y - p2.y;
      dotproduct = (vector1.x*vector2.x) + (vector1.y*vector2.y);
      length1 = sqrtf((vector1.x*vector1.x)+(vector1.y*vector1.y));
      length2 = sqrtf((vector2.x*vector2.x)+(vector2.y*vector2.y));
      angle = fabs(dotproduct/(length1*length2));

      if(angle < 0.2) {
        //cvCircle(frame,*p,4,CV_RGB(0,255,255),-1,8,0);

        if(!signal) {
          signal = true;
          min.x = p.x;
          min.y = p.y;
          minp1.x = p1.x;
          minp1.y = p1.y;
          minp2.x = p2.x;
          minp2.y = p2.y;
          minangle = angle;
        }
        else {
          if(angle <= minangle) {
            min.x = p.x;
            min.y = p.y;
            minp1.x = p1.x;
            minp1.y = p1.y;
            minp2.x = p2.x;
            minp2.y = p2.y;
            minangle = angle;
          }
        }
      }
      else { //else start
        if(signal) {
          signal = false;
          cv::Point l1,l2,l3;
          l1.x = min.x - armcenter.x;
          l1.y = min.y - armcenter.y;

          l2.x = minp1.x - armcenter.x;
          l2.y = minp1.y - armcenter.y;

          l3.x = minp2.x - armcenter.x;
          l3.y = minp2.y - armcenter.y;

          length = sqrtf((l1.x*l1.x)+(l1.y*l1.y));
          length1 = sqrtf((l2.x*l2.x)+(l2.y*l2.y));
          length2 = sqrtf((l3.x*l3.x)+(l3.y*l3.y));

          if(length > length1 && length > length2) {
            //cvCircle(frame,min,6,CV_RGB(0,255,0),-1,8,0);
            fingertip[count] = min;
            tiplocation[count] = i+20;
            count = count + 1;
          }
          else if(length < length1 && length < length2) {
            //cvCircle(frame,min,8,CV_RGB(0,0,255),-1,8,0);
            //cvCircle(virtualhand,min,8,CV_RGB(255,255,255),-1,8,0);
            //palm.push_back(min);
            //fingertip[count] = min;
            //tiplocation[count] = i+20;
            //count = count + 1;
          }
        }
      }//else end
    }//for end

    for(i=0; i<count; i++) {
      if((tiplocation[i] - tiplocation[i-1]) > 40) {
        if(fingertip[i].x >= 630  || fingertip[i].y >= 470) {
          circle(image, fingertip[i], 6, CV_RGB(50,200,250), -1, 8, 0);
        }
        else {
          //cvCircle(frame,fingertip[i],6,CV_RGB(0,255,0),-1,8,0);
          //cvCircle(virtualhand,fingertip[i],6,CV_RGB(0,255,0),-1,8,0);
          //cvLine(virtualhand,fingertip[i],armcenter,CV_RGB(255,0,0),3,CV_AA,0);
          fingerseq.push_back(fingertip[i]);
        }
      }
    }
    //cvClearSeq(fingerseq);
  }

  void  HandDetector::drawApproxCurve(Mat &in, vector<Point>  &contour, Scalar color ) {
    for (unsigned int i=0; i<contour.size(); i++) {
      cv::line (in, contour[i], contour[(i+1) % contour.size()], color, 3);
    }
  }

  //find BiggestContour
  int HandDetector::findBiggestContour(vector<vector<cv::Point> > contours) {
    int indexOfBiggestContour = -1;
    size_t sizeOfBiggestContour = 0;
    for (size_t i = 0; i < contours.size(); i++) {
      if(contours[i].size() > sizeOfBiggestContour) {
        sizeOfBiggestContour = contours[i].size();
        indexOfBiggestContour = i;
      }
    }
    return indexOfBiggestContour;
  }

  float HandDetector::distanceP2P(cv::Point a, cv::Point b) {
    float d= sqrt(fabs( pow(a.x-b.x,2) + pow(a.y-b.y,2) )) ;
    return d;
  }

  float HandDetector::getAngle(cv::Point s, cv::Point f, cv::Point e) {
    float l1 = distanceP2P(f,s);
    float l2 = distanceP2P(f,e);
    float dot = (s.x-f.x) * (e.x-f.x) + (s.y-f.y) * (e.y-f.y);
    float angle = acos(dot / (l1*l2));
    angle = angle * 180/CV_PI;
    return angle;
  }
  /*
  bool HandDetector::isInto (cv::Mat &contour, vector<cv::Point2f> &b) {
    for (unsigned int i=0; i<b.size(); i++)
      if (pointPolygonTest(contour, b[i], false) >0) return true;
    return false;
  }
  */




}
