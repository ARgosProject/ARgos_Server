#include <opencv2/opencv.hpp>
#include <cstdio>
#include <iostream>
#include <fstream>
#include "DocumentDetector.h"
#include "ConfigManager.h"
#include "Log.h"

using namespace std;
using namespace cv;

namespace argosServer{

  DocumentDetector::DocumentDetector(){
    // Create Feature Detectors and Descriptors
    createDetectorDescriptorMatcher();

    queryImagesNames = ConfigManager::getDescriptorsList();
  }

  DocumentDetector::~DocumentDetector(){}

  void DocumentDetector::setImagesPath(const string& path) {
    imagesPath = path;
  }

  void DocumentDetector::configure() {
    // Read query images
    readImages();

    //Extracting keypoints from query images
    featureDetector->detect( queryImages, queryKeypoints );
    Log::success("Extracting keypoints from images... [OK]");

    //Compute descriptor from query images

    descriptorExtractor->compute( queryImages, queryKeypoints, queryDescriptors );
    Log::success("Computing descriptors for keypoints... [OK]");
    /*
      FileStorage fs("Descriptors.yml", FileStorage::WRITE);
      for(size_t i=0; i< queryDescriptors.size(); i++){
      cout <<  queryDescriptors.at(i).rows << " descriptors extracted from query image " << i << endl;
      write(fs, "descriptors", queryDescriptors.at(i));
      cout << " Descriptors " << i << " saved to file" << endl;
      }
      fs.release();
      cout << ">" << endl;
    */
  }

  /**
   * Main detection function.
   */
  void DocumentDetector::detect(const cv::Mat& trainFrame, vector<Paper>& detectedPapers){

    double  distance_0_1;
    double  distance_1_2;
    double  distance_2_3;
    double  distance_3_0;

    //imshow("trainFrame",trainFrame );
    //waitKey(1);

    for (unsigned int i=0; i<detectedPapers.size(); i++){

      distance_0_1 = cv::norm( (detectedPapers[i])[0] - (detectedPapers[i])[1] );
      distance_1_2 = cv::norm( (detectedPapers[i])[1] - (detectedPapers[i])[2] );
      distance_2_3 = cv::norm( (detectedPapers[i])[2] - (detectedPapers[i])[3] );
      distance_3_0 = cv::norm( (detectedPapers[i])[3] - (detectedPapers[i])[0] );

      //cout << "distance_0_1: "<<  distance_0_1 << endl;
      //cout << "distance_1_2: "<<  distance_1_2 << endl;
      //cout << "distance_2_3: "<<  distance_2_3 << endl;
      //cout << "distance_3_0: "<<  distance_3_0 << endl;

      //cout << "distance_01-23: "<<  (distance_0_1 + distance_2_3) << endl;
      //cout << "distance_12-30: "<<  (distance_1_2 + distance_3_0) << endl;


      if( (distance_0_1 + distance_2_3) > (distance_1_2 + distance_3_0) )
        warp(trainFrame,paperContent,Size(600,420), detectedPapers[i]);
      //warp(trainFrame,paperContent,Size(278,180), detectedPapers[i]);
      else
        warp(trainFrame,paperContent,Size(420,600), detectedPapers[i]);
      //warp(trainFrame,paperContent,Size(180,278), detectedPapers[i]);

      //imshow("paperContent", paperContent);
      //waitKey(1);



      // extract feautures and compute descriptors of current frame
      featureDetector->detect(paperContent, trainKeypoints );
      descriptorExtractor->compute(paperContent, trainKeypoints, trainDescriptors);
      vector<vector<DMatch> > elements_matches;

      if (trainDescriptors.empty() || trainDescriptors.rows == 1){
        //cout <<  "trainDescriptors empty "<< endl;
        //cout <<  "trainDescriptors rows =  "<<  trainDescriptors.rows << endl;
        detectedPapers[i].setId(999);
        //cout <<  "detectedPapers["<< i <<"]="<< detectedPapers[i].getId()<< endl;
      }
      else{
        if (trainDescriptors.type() != 0 ){
          for(size_t j=0; j < queryDescriptors.size(); j++){
            vector<DMatch> good_matches;
            //cout <<  "trainDescriptors rows =  "<<  trainDescriptors.rows << endl;
            descriptorMatcher->knnMatch(queryDescriptors[j], trainDescriptors, matches, 2);
            for(unsigned int k=0; k<matches.size(); k++){
              // Apply NNDR
              if(matches.at(k).at(0).distance <= 0.8 * matches.at(k).at(1).distance)
                good_matches.push_back(matches[k][0]);
            }
            //cout <<  "Good Matches: " << good_matches.size() << endl;
            elements_matches.push_back(good_matches);
          }
        }
        int max_index = -1;
        unsigned int max_descriptors = 0;
        for (size_t m=0;  m < elements_matches.size(); m++){
          if (elements_matches.at(m).size() > max_descriptors){
            max_descriptors = elements_matches.at(m).size();
            max_index = m;
          }
        }

        if(max_index == -1){
          //cout << "error index -1" << endl;
          detectedPapers[i].setId(-1);
        }
        else{
          //cout <<  "Detecting: " << queryImagesNames.at(max_index) << endl;
          detectedPapers[i].setId(max_index);
        }
      }
    }

  }


  bool DocumentDetector::createDetectorDescriptorMatcher(){
    Log::info("Creating feature detector, descriptor extractor and descriptor matcher ...");
    featureDetector = new cv::SurfFeatureDetector(4000,4);
    descriptorExtractor = new cv::SurfDescriptorExtractor();
    descriptorMatcher = cv::Ptr<cv::DescriptorMatcher>(new cv::FlannBasedMatcher(cv::Ptr<cv::flann::IndexParams>(new cv::flann::KDTreeIndexParams())));

    bool isCreated = !( featureDetector.empty() || descriptorExtractor.empty() || descriptorMatcher.empty() );
    if( !isCreated )
      Log::error("Can not create feature detector or descriptor extractor or descriptor matcher of given types.");

    return isCreated;
  }

  /*
    void DocumentDetector::readQueryFilenames(const string& filename, string& dirName, vector<string>& queryFilenames){
    queryFilenames.clear();

    ifstream file( filename.c_str() );
    if ( !file.is_open() )
    return;

    size_t pos = filename.rfind('\\');
    char dlmtr = '\\';
    if (pos == String::npos){
    pos = filename.rfind('/');
    dlmtr = '/';
    }
    dirName = pos == string::npos ? "" : filename.substr(0, pos) + dlmtr;

    while( !file.eof() ){
    string str; getline( file, str );
    if( str.empty() ) break;
    queryFilenames.push_back(str);
    }
    file.close();
    }
  */

  bool DocumentDetector::readImages(){
    /*
      cout << "< Reading the images..." << endl;
      string queryDirName;
      readQueryFilenames(queryFilename, queryDirName, queryImageNames);

      if(queryImageNames.empty() ){
      cout << "Query image filenames can not be read." << endl << ">" << endl;
      return false;
      }
    */


    int readImageCount = 0;
    for( size_t i = 0; i < queryImagesNames.size(); i++ ){
      string filename = queryImagesNames[i];
      Mat img = imread(imagesPath + filename, CV_LOAD_IMAGE_GRAYSCALE );
      if( img.empty() )
        Log::error("Query image " + imagesPath + filename + " can not be read.");
      else
        readImageCount++;
      queryImages.push_back(img);
    }

    if(!readImageCount){
      Log::error("All query images can not be read.");
      return false;
    }
    else
      Log::success(to_string(readImageCount) + " query images were read.");


    return true;
  }

  bool DocumentDetector::warp (const cv::Mat& in, cv::Mat& out, Size size, vector<Point2f> points ) throw ( cv::Exception ){
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
