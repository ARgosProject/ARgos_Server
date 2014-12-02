#include <opencv2/opencv.hpp>
#include <cstdio>
#include <iostream>
#include <fstream>
#include "DocumentDetector.h"

using namespace std;
using namespace cv;

namespace argosServer{
  
  DocumentDetector::DocumentDetector(){
    // Default Configuration
    const string fileWithQueryImages = "./queryImages.txt";
    
    // Create Feature Detectors and Descriptors
    createDetectorDescriptorMatcher();
    
    // Read query images
    readImages(fileWithQueryImages, queryImages, queryImagesNames);
    
    //Extracting keypoints from query images
    cout << endl << "< Extracting keypoints from images..." << endl;  
    featureDetector->detect( queryImages, queryKeypoints );
    cout << ">" << endl;
    
    //Compute descriptor from query images
    cout << "< Computing descriptors for keypoints..." << endl;
    descriptorExtractor->compute( queryImages, queryKeypoints, queryDescriptors );
    cout << ">" << endl;
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
  DocumentDetector::~DocumentDetector(){}


  /**
   *
   * Main detection function. Performs all steps
   *
   *
   */
  void DocumentDetector::detect(const cv::Mat& trainFrame, vector<Paper>& detectedPapers){  
    
    for (unsigned int i=0; i<detectedPapers.size(); i++){ 
      //if (detectedPapers[i].getTransVec().at<float>(0) > 0)
      warp(trainFrame,paperContent,Size(600,420), detectedPapers[i]);
      //else
      //warp(trainFrame,paperContent,Size(420,600), detectedPapers[i]);
      //imshow("trainFrame",paperContent);
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
	  detectedPapers[i].setId(0);
	}
	else{
	  //cout <<  "Detecting: " << queryImagesNames.at(max_index) << endl;
	  detectedPapers[i].setId(max_index+1);
	}
      }
    }
  }


  bool DocumentDetector::createDetectorDescriptorMatcher(){
    cout << "< Creating feature detector, descriptor extractor and descriptor matcher ..." << endl;
    featureDetector = new cv::SurfFeatureDetector(2000,4);
    descriptorExtractor = new cv::SurfDescriptorExtractor();
    descriptorMatcher = cv::Ptr<cv::DescriptorMatcher>(new cv::FlannBasedMatcher(cv::Ptr<cv::flann::IndexParams>(new cv::flann::KDTreeIndexParams())));
  
    cout << ">" << endl;
  
    bool isCreated = !( featureDetector.empty() || descriptorExtractor.empty() || descriptorMatcher.empty() );
    if( !isCreated )
      cout << "Can not create feature detector or descriptor extractor or descriptor matcher of given types." << endl << ">" << endl;
  
    return isCreated;
  }

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

  bool DocumentDetector::readImages( const string& queryFilename, vector <Mat>& queryImages, vector<string>& queryImageNames ){
  
    cout << "< Reading the images..." << endl;
    string queryDirName;
    readQueryFilenames(queryFilename, queryDirName, queryImageNames);

    if(queryImageNames.empty() ){
      cout << "Query image filenames can not be read." << endl << ">" << endl;
      return false;
    }
  
    int readImageCount = 0;
    for( size_t i = 0; i < queryImageNames.size(); i++ ){
      string filename = queryDirName + queryImageNames[i];
      Mat img = imread( filename, CV_LOAD_IMAGE_GRAYSCALE );
      if( img.empty() )
	cout << "Query image " << filename << " can not be read." << endl;
      else
	readImageCount++;
      queryImages.push_back( img );
    }
  
    if(!readImageCount){
      cout << "All query images can not be read." << endl << ">" << endl;
      return false;
    }
    else
      cout << readImageCount << " query images were read." << endl;
    cout << ">" << endl;
  
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
