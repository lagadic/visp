/****************************************************************************
 *
 * $Id: $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.GPL at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Ferns based plane detection.
 *
 * Authors:
 * Romain Tallonneau
 *
 *****************************************************************************/


#include <visp/vpConfig.h>

#if (VISP_HAVE_OPENCV_VERSION >= 0x020000) // Require opencv >= 2.0.0

#include <visp/vpPlanarObjectDetector.h>
#include <visp/vpImageConvert.h>
#include <visp/vpException.h>
#include <cv.h>
#include <vector>
#include <cvaux.h>
#include <iostream>
#include <visp/vpImagePoint.h>
#include <visp/vpDisplay.h>
#include <visp/vpColor.h>
#include <visp/vpImageTools.h>


/*!

  Basic constructor

*/
vpPlanarObjectDetector::vpPlanarObjectDetector(): gen(0, 256, 5, true, 0.6, 1.5, -CV_PI/2, CV_PI/2, -CV_PI/2, CV_PI/2)
{
  hasLearn = false;
  nbClassfier = 100;
  ClassifierSize = 11;
  nbPoints = 100;
  model = NULL;
  blurImage = true;
  radiusBlur = 7;
  sigmaBlur = 1;
  init();
}

/*!

  Basic constructor, load data from a file
  
  \param dataFile : the name of the file
  \param objectName : the name of the object to load

*/
vpPlanarObjectDetector::vpPlanarObjectDetector(const std::string& dataFile, const std::string& objectName): 
    gen(0, 256, 5, true, 0.6, 1.5, -CV_PI/2, CV_PI/2, -CV_PI/2, CV_PI/2)
{
  nbClassfier = 100;
  ClassifierSize = 11;
  nbPoints = 100;
  model = NULL;
  blurImage = true;
  radiusBlur = 7;
  sigmaBlur = 1;
  init();
  this->load(dataFile, objectName);
  hasLearn = true;
}

/*!
  initialise some variables (the values are set from the openCV default values)
*/
void
vpPlanarObjectDetector::init()
{
  patchSize = 32;
  radius = 7;
  threshold = 20;
  nbOctave = 2;
  nbView = 2000;
  dist = 2;  
  nbMinPoint = 10;
}

/*!

  Basic destructor

*/
vpPlanarObjectDetector::~vpPlanarObjectDetector() 
{
  if(model != NULL){
    cvReleaseImage(&model);
    model = NULL;
  }
}

/*!
  Compute the rectangular ROI from at least 4 points and set the region of 
  interest on the current image.
  
  \param ip : the list of image point
  \param nbpt : the number of point
*/
void 
vpPlanarObjectDetector::computeRoi(vpImagePoint* ip, const int nbpt)
{
  if(nbpt < 3){
    throw vpException(vpException::badValue, "Not enough point");
  }
  
  std::vector < vpImagePoint > ptsx;
  ptsx.resize(nbpt);
  std::vector < vpImagePoint > ptsy;
  ptsy.resize(nbpt);
  for(int i=0; i<nbpt; i++){
    ptsx[i] = ptsy[i] = ip[i];
  }
  
  for(int i=0; i<nbpt; i++){
    for(int j=0; j<nbpt-1; j++){
      if(ptsx[j].get_j() > ptsx[j+1].get_j()){
        double tmp = ptsx[j+1].get_j();
        ptsx[j+1].set_j(ptsx[j].get_j());
        ptsx[j].set_j(tmp);
      }
    }
  }
  for(int i=0; i<nbpt; i++){
    for(int j=0; j<nbpt-1; j++){
      if(ptsy[j].get_i() > ptsy[j+1].get_i()){
        double tmp = ptsy[j+1].get_i();
        ptsy[j+1].set_i(ptsy[j].get_i());
        ptsy[j].set_i(tmp);
      }
    }
  }
  
  
   vpImagePoint tl(ptsx[1].get_j(), ptsy[1].get_i());
   vpImagePoint br(ptsx[nbpt-2].get_j(), ptsy[nbpt-2].get_i());
  
  setRoi(tl, br);
  
}

/*!
  Set several parameters of the detector.

  \param _threshold : threshold to detect points of interests (between 2 and 20)
  \param _nbView : number of view used to train the classifier (between 1000 and 10000)
  \param _dist : the minimum distance between two points (in pixel)
  \param _nbClassfier : the number of classifier (between 30 and 100)
  \param _ClassifierSize : the size of each classifier (between 8 and 15)
  \param _nbOctave : the number of octave ( subsampling of the image )
  \param _patchSize : the size of a patch for the point recognition (usually 32)
  \param _radius : the radius for the recognition
  \param _nbPoints : the maximum number of points 
  
*/
void 
vpPlanarObjectDetector::setDetectorParameters(int _threshold, int _nbView, int _dist, int _nbClassfier, int _ClassifierSize, int _nbOctave, int _patchSize, int _radius, int _nbPoints)
{
  threshold = _threshold;
  nbView = _nbView;
  dist = _dist;
  nbClassfier = _nbClassfier;
  ClassifierSize = _ClassifierSize;
  nbOctave = _nbOctave;
  patchSize = _patchSize;
  radius = _radius;
  nbPoints = _nbPoints;
}


/*!
  train the classifier from the entire image (it is therefore assumed that the image is planar)
  
  Depending on the parameters, the training can take up to several minutes.
  
  \param I : the image use to train the classifier
  
  \return the number of reference points
*/
int
vpPlanarObjectDetector:: buildReference(const vpImage<unsigned char> &I)
{
  this->setImage(I);
  
  train();

  return objKeypoints.size();
}


/*!
  train the classifier a region of the entire image 
  
  \param I : the image use to train the classifier
  \param iP : the top left corner of the rectangle defining the region of interest (ROI)
  \param height : the height of the ROI
  \param width : the width of the ROI
  
  \return the number of reference points
*/
int
vpPlanarObjectDetector::buildReference(const vpImage<unsigned char> &I,
		       vpImagePoint &iP,
		       unsigned int height, unsigned int width)
{
  if((iP.get_i()+height) >= I.getHeight()
     || (iP.get_j()+width) >= I.getWidth()) {
    vpTRACE("Bad size for the subimage");
    throw(vpException(vpImageException::notInTheImage ,
		      "Bad size for the subimage"));
  }
  
//  vpImagePoint br(iP.get_i() + height, iP.get_j() + width);
//  this->setImage(I);
//  this->setRoi(iP, br);
  
    vpImage<unsigned char> subImage;

  vpImageTools::createSubImage(I,
			       (unsigned int)iP.get_i(),
			       (unsigned int)iP.get_j(),
			       height, width, subImage);
  this->setImage(subImage);
  
  modelROI_Ref.x = (int)iP.get_u();
  modelROI_Ref.y = (int)iP.get_v();
  modelROI_Ref.width = width;
  modelROI_Ref.height = height;  
  
  train();

  return objKeypoints.size(); 
}



/*!
  train the classifier a region of the entire image 
  
  \param I : the image use to train the classifier
  \param rectangle : the rectangle defining the region of interest (ROI)
  
  \return the number of reference points
*/
int 
vpPlanarObjectDetector::buildReference(const vpImage<unsigned char> &I,
		       const vpRect rectangle)
{
  vpImagePoint iP;
  iP.set_i(rectangle.getTop());
  iP.set_j(rectangle.getLeft());
  return (this->buildReference(I, iP,
			       (unsigned int)rectangle.getHeight(),
			       (unsigned int)rectangle.getWidth()));
}


/*! 
  train the classifier using the image from the class (the model variable)
*/
void
vpPlanarObjectDetector::train()
{
  
    // initialise detector
  cv::LDetector d(radius, threshold, nbOctave, nbView, patchSize, dist);
  
    //blur 
  cv::Mat obj = (cv::Mat)model;
  
  if(this->getBlurSetting()){
    cv::GaussianBlur(obj, obj, cv::Size(getBlurSize(), getBlurSize()), getBlurSigma(), getBlurSigma());
  }
  
    // build pyramid 
  std::vector<cv::Mat> objpyr;
  cv::buildPyramid(obj, objpyr, d.nOctaves-1);
  
    // patch generator
  std::vector<cv::KeyPoint> imgKeypoints;
  
    // getPoints
  d.getMostStable2D(obj, objKeypoints, 100, gen);
  
  ldetector = d;
  
    // train classifier
  cv::PlanarObjectDetector::train(objpyr, /*objKeypoints*/100, this->patchSize, 100, 11, 10000, ldetector, gen);
  
    // set flag
  hasLearn = true;
}


/*!
  Compute the points of interest in the current image and try to recognise them 
  using the trained classifier. The matched pairs can be found with the 
  getMatchedPointByRef function. The homography between the two planar 
  surfaces is also computed.
  
  \param I : The gray scaled image where the points are computed.
  
  \return true if the surface has been found.
*/
bool 
vpPlanarObjectDetector::matchPoint(const vpImage<unsigned char> &I)
{
  if(!hasLearn){
    vpERROR_TRACE("The object has not been learned. ");
    throw vpException(vpException::notInitialized , "object is not learned, load database or build the reference ");
  }

  setImage(I);
  // resize image  
//  cv::resize(_image, image, Size(), 1./imgscale, 1./imgscale, INTER_CUBIC);
  cv::Mat img = this->model;
  
  if(this->getBlurSetting()){
    cv::GaussianBlur(img, img, cv::Size(this->getBlurSize(), this->getBlurSize()), this->getBlurSigma(), this->getBlurSigma());
  }
  
  std::vector<cv::Mat> imgPyr;
  cv::buildPyramid(img, imgPyr, ldetector.nOctaves-1);
  
    //matching
  cv::Mat H;  
  objKeypoints = getModelPoints();
  
  ldetector(imgPyr, imgKeypoints, 500);

  bool found = this->operator()(imgPyr, imgKeypoints, H, dst_corners, &pairs); 
  if(pairs.size() < 2 * nbMinPoint){
    return false;
  }  
  double sum = 0;
  if(found){
    double* ptr = (double*)H.data;
    
    for(int i=0; i<9; i++){
      sum += *ptr;
      this->homography[(int)(i/3)][i%3] = *(ptr++);
    }
  }
  else{
    this->homography.setIdentity();
  }
  
  if(sum <= DBL_EPSILON){
    return false;
  }

  return found;
}


/*!
  Compute the points of interest in the specified region of the current image 
  and try to recognise them using the trained classifier. The matched pairs can 
  be found with the getMatchedPointByRef function. The homography between the 
  two planar surfaces is also computed.
  
  \param I : The gray scaled image where the points are computed.
  \param iP : the top left corner of the rectangle defining the ROI
  \param height : the height of the ROI
  \param width : the width of the ROI
  
  \return true if the surface has been found.
*/
bool 
vpPlanarObjectDetector::matchPoint(const vpImage<unsigned char> &I,
	   vpImagePoint &iP, unsigned int height, unsigned int width)
{
  if((iP.get_i()+height) >= I.getHeight()
     || (iP.get_j()+width) >= I.getWidth()) {
    vpTRACE("Bad size for the subimage");
    throw(vpException(vpImageException::notInTheImage ,
		      "Bad size for the subimage"));
  }

  vpImage<unsigned char> subImage;

  vpImageTools::createSubImage(I,
			       (unsigned int)iP.get_i(),
			       (unsigned int)iP.get_j(),
			       height, width, subImage);

  return this->matchPoint(subImage);
}

/*!
  Compute the points of interest in the specified region of the current image 
  and try to recognise them using the trained classifier. The matched pairs can 
  be found with the getMatchedPointByRef function. The homography between the 
  two planar surfaces is also computed.
  
  \param I : The gray scaled image where the points are computed.
  \param rectangle : the rectangle defining the ROI
  
  \return true if the surface has been found.
*/
bool 
vpPlanarObjectDetector::matchPoint(const vpImage<unsigned char> &I, const vpRect rectangle)
{
  vpImagePoint iP;
  iP.set_i(rectangle.getTop());
  iP.set_j(rectangle.getLeft());
  return (this->matchPoint(I, iP,
			   (unsigned int)rectangle.getHeight(),
			   (unsigned int)rectangle.getWidth()));
}


/*!
  Display the result of the matching. The plane is displayed in red and the 
  points of interests detected in the image are shown by a red dot surrounded by 
  a green circle (the radius of the circle depends on the octave at which it has 
  been detected).
  
  \param I : the gray scaled image for the display
  \param displayKpts : the flag to display keypoints in addition to the surface
*/
void 
vpPlanarObjectDetector::display(vpImage<unsigned char> &I, bool displayKpts)
{
  for(unsigned int i=0; i<dst_corners.size(); i++){
    vpImagePoint ip1(
      dst_corners[i].y, 
      dst_corners[i].x);
    vpImagePoint ip2(
      dst_corners[(i+1)%dst_corners.size()].y, 
      dst_corners[(i+1)%dst_corners.size()].x);
    vpDisplay::displayLine(I, ip1, ip2, vpColor::red) ;
  }
  
  if(displayKpts){
    for(unsigned int i=0; i<imgKeypoints.size(); i++){
      vpImagePoint ip1(
        imgKeypoints[i].pt.y, 
        imgKeypoints[i].pt.x);
      vpDisplay::displayCircle(I, ip1, 2, vpColor::red, true, 1);
      vpDisplay::displayCircle(I, ip1, (1 << imgKeypoints[i].octave)*15, vpColor::green, false, 1);
    }
  }
}

/*!
  This function displays the matched reference points and the matched
  points computed in the current image. The reference points are
  displayed in the image Ireference and the matched points coming from
  the current image are displayed in the image Icurrent. It is
  possible to set Ireference and Icurrent with the same image when
  calling the method.

  \param Iref : The image where the matched refrence points are
  displayed.

  \param Icurrent : The image where the matched points computed in the
  current image are displayed.
*/
void 
vpPlanarObjectDetector::display(vpImage<unsigned char> &Iref,
		 vpImage<unsigned char> &Icurrent)
{
  display(Icurrent, false);
  for (int i = 0; i < (int)pairs.size(); i+=2){
    vpImagePoint ptRef, ptCur;
    ptRef.set_uv(modelPoints[pairs[i]].pt.x, modelPoints[pairs[i]].pt.y);
    ptCur.set_uv(imgKeypoints[pairs[i+1]].pt.x, imgKeypoints[pairs[i+1]].pt.y);
    vpDisplay::displayCross (Iref, ptRef, 3, vpColor::red);
    vpDisplay::displayCross (Icurrent,  ptCur, 3, vpColor::green);
  }
}


/*!
  \brief load the Fern classifier
  
  Load and initialize the Fern classifier and load the 3D and 2D keypoints. It 
  can take up to sevral seconds.
  
  \param dataFilename : The name of the data filename (very large text file). It can have any file extension.
  \param objName : The name of the object.
*/
void
vpPlanarObjectDetector::load(const std::string& dataFilename, const std::string& objName)
{
  std::cout << "> Load data for the planar object detector..." << std::endl;

  cv::FileStorage fs(dataFilename, cv::FileStorage::READ);
  read(fs.getFirstTopLevelNode());
  
  if(fs.getFirstTopLevelNode().name() != objName){
    vpERROR_TRACE(" The name in the database does not correspond to the name in the parameters.");
  }
  
  cv::LDetector d(radius, threshold, nbOctave, nbView, patchSize, dist);
  ldetector = d;
  hasLearn = true;
}


/*!
  \brief record the Ferns classifier in the text file

  \param objectName : The name of the object to store in the data file
  \param dataFile : The name of the data filename (very large text file). It can have any file extension.
*/
void
vpPlanarObjectDetector::recordDetector(const std::string& objectName, const std::string& dataFile )
{
  cv::FileStorage fs(dataFile, cv::FileStorage::WRITE);
  write(fs, objectName);
}

/*!
  \brief set the current image

  \param I : the current image
*/
void
vpPlanarObjectDetector::setImage(const vpImage<unsigned char>& I)
{
  if(model != NULL){
    cvResetImageROI(model);
    cvReleaseImage(&model);
    model = NULL;
  }
  
  vpImageConvert::convert(I,model);
  if(model == NULL){
    std::cout << "!> conversion failed" << std::endl;
    throw vpException(vpException::notInitialized , "conversion failed");
  }
}

/*!
  \brief Set the region of interest (ROI) in the current image

  \param tl : the top left image point of the ROI
  \param br : the bottom right image point of the ROI
*/
void
vpPlanarObjectDetector::setRoi(vpImagePoint tl, vpImagePoint br)
{
  CvRect rect = cvRect((int)tl.get_u(), (int)tl.get_v(), (int)(br.get_u()-tl.get_u()), (int)(br.get_v()-tl.get_v()));
  if(model == NULL){
    std::cout << "!> image not set" << std::endl;
    throw vpException(vpException::notInitialized , "image not set in setRoi");
  }

  cvSetImageROI(model, rect);

    
  modelROI.x = (int)tl.get_u();
  modelROI.y = (int)tl.get_v();
  modelROI.width = (int)(br.get_u()-tl.get_u());
  modelROI.height = (int)(br.get_v()-tl.get_v());
  
  modelROI_Ref.x = (int)tl.get_u();
  modelROI_Ref.y = (int)tl.get_v();
  modelROI_Ref.width = (int)(br.get_u()-tl.get_u());
  modelROI_Ref.height = (int)(br.get_v()-tl.get_v());

}



/*!
  get the reference point given by the index
  
  \param index : the number of the point
  
  \return the reference image point
*/
vpImagePoint 
vpPlanarObjectDetector::getRefPoint(const int index)
{
  if(index <0 || index >= (int)modelPoints.size()){
    vpTRACE("Index of the reference point out of range");
    throw(vpException(vpException::fatalError,"Index of the refrence point out of range"));
  }
  vpImagePoint pt;
  pt.set_u(modelPoints[index].pt.x);
  pt.set_v(modelPoints[index].pt.y);
  return pt;
}

void 
vpPlanarObjectDetector::getReferencePoint(const int index, vpImagePoint &imP)
{
  if(index <0 || index >= (int)modelPoints.size()){
    vpTRACE("Index of the reference point out of range");
    throw(vpException(vpException::fatalError,"Index of the refrence point out of range"));
  }

  imP.set_u(modelPoints[index].pt.x);
  imP.set_v(modelPoints[index].pt.y);
}

/*!
  get the current image point given by the index
  
  \param index : the number of the point
  
  \return the current image point
*/
vpImagePoint 
vpPlanarObjectDetector::getCurPoint(const int index)
{
  if(index <0 || index >= (int)imgKeypoints.size()){
    vpTRACE("Index of the reference point out of range");
    throw(vpException(vpException::fatalError,"Index of the refrence point out of range"));
  }
  vpImagePoint pt;
  pt.set_u(imgKeypoints[index].pt.x);
  pt.set_v(imgKeypoints[index].pt.y);
  return pt;
}

void 
vpPlanarObjectDetector::getCurPoint(const int index, vpImagePoint & ip)
{
  if(index <0 || index >= (int)imgKeypoints.size()){
    vpTRACE("Index of the reference point out of range");
    throw(vpException(vpException::fatalError,"Index of the refrence point out of range"));
  }
  vpImagePoint pt;
  ip.set_u(imgKeypoints[index].pt.x);
  ip.set_v(imgKeypoints[index].pt.y);
}

/*!
  The image is blurred before being processed. This solution can lead to a 
  better recognition rate.
  
  \param _blur : the new option for the blur
  \param _sigma : the sigma for the blur
  \param _size : the size for the blur
*/
void 
vpPlanarObjectDetector::setBlurSettings(const bool _blur, int _sigma, int _size)
{ 
  blurImage = _blur; 
  radiusBlur = _size; 
  sigmaBlur = _sigma; 
}

/*!
  Get the point in the current image matching with the one given by index
  
  \param indexRef : the index of the point in the reference image points vector
  
  \return the index of the matched point in the current image or -1 if there is 
    no matching
*/
int 
vpPlanarObjectDetector::getMatchedPointByRef(const int indexRef)
{
  for(unsigned int i=0; i< pairs.size(); i+=2){
    if(pairs[i] == indexRef){
      return pairs[i+1];
    }
  }
  return -1;
}

/*!
  return the last positions of the detected corners
  
  \return the vectors of corners' postions
*/
std::vector<vpImagePoint> 
vpPlanarObjectDetector::getDetectedCorners() const{
  vpImagePoint ip;
  std::vector <vpImagePoint> corners;
  corners.clear();
  for(unsigned int i=0; i<dst_corners.size(); i++){
    ip.set_uv( dst_corners[i].x, dst_corners[i].y);
    corners.push_back(ip);
  }

  return corners;
}

#endif

