/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Planar surface detection tool.
 *
 * Authors:
 * Romain Tallonneau
 *
 *****************************************************************************/

#include <visp3/vision/vpPlanarObjectDetector.h>

#if (VISP_HAVE_OPENCV_VERSION >= 0x020000) &&                                                                          \
    (VISP_HAVE_OPENCV_VERSION < 0x030000) // Require opencv >= 2.0.0 and < 3.0.0

#include <visp3/core/vpColor.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpImageTools.h>

#include <iostream>
#include <vector>

/*!

  Basic constructor

*/
vpPlanarObjectDetector::vpPlanarObjectDetector()
  : fern(), homography(), H(), dst_corners(), isCorrect(false), ref_corners(), modelROI(), currentImagePoints(),
    refImagePoints(), minNbMatching(10)
{
}

/*!

  Basic constructor, load data from a file.

  \param _dataFile : the name of the file.
  \param _objectName : the name of the object to load.

*/
vpPlanarObjectDetector::vpPlanarObjectDetector(const std::string &_dataFile, const std::string &_objectName)
  : fern(), homography(), H(), dst_corners(), isCorrect(false), ref_corners(), modelROI(), currentImagePoints(),
    refImagePoints(), minNbMatching(10)
{
  load(_dataFile, _objectName);
}

/*!
  Initialise stuff. For the moment does nothing.
*/
void vpPlanarObjectDetector::init() {}

/*!

  Basic destructor

*/
vpPlanarObjectDetector::~vpPlanarObjectDetector() {}

/*!
  Compute the rectangular ROI from at least 4 points and set the region of
  interest on the current image.

  \param ip : the list of image point.
  \param nbpt : the number of point.
*/
void vpPlanarObjectDetector::computeRoi(vpImagePoint *ip, const unsigned int nbpt)
{
  if (nbpt < 3) {
    throw vpException(vpException::badValue, "Not enough point to compute the region of interest.");
  }

  std::vector<vpImagePoint> ptsx(nbpt);
  std::vector<vpImagePoint> ptsy(nbpt);
  for (unsigned int i = 0; i < nbpt; i++) {
    ptsx[i] = ptsy[i] = ip[i];
  }

  for (unsigned int i = 0; i < nbpt; i++) {
    for (unsigned int j = 0; j < nbpt - 1; j++) {
      if (ptsx[j].get_j() > ptsx[j + 1].get_j()) {
        double tmp = ptsx[j + 1].get_j();
        ptsx[j + 1].set_j(ptsx[j].get_j());
        ptsx[j].set_j(tmp);
      }
    }
  }
  for (unsigned int i = 0; i < nbpt; i++) {
    for (unsigned int j = 0; j < nbpt - 1; j++) {
      if (ptsy[j].get_i() > ptsy[j + 1].get_i()) {
        double tmp = ptsy[j + 1].get_i();
        ptsy[j + 1].set_i(ptsy[j].get_i());
        ptsy[j].set_i(tmp);
      }
    }
  }
}

/*!
  Train the classifier from the entire image (it is therefore assumed that the
  image is planar).

  Depending on the parameters, the training can take up to several minutes.

  \param _I : The image use to train the classifier.

  \return The number of reference points.
*/
unsigned int vpPlanarObjectDetector::buildReference(const vpImage<unsigned char> &_I)
{
  modelROI.x = 0;
  modelROI.y = 0;
  modelROI.width = (int)_I.getWidth();
  modelROI.height = (int)_I.getHeight();

  initialiseRefCorners(modelROI);

  return fern.buildReference(_I);
}

/*!
  Train the classifier on a region of the entire image. The region is a
  rectangle defined by its top left corner, its height and its width. The
  parameters of this rectangle must be given in pixel. It also includes the
  training of the fern classifier.

  \param _I : The image use to train the classifier.
  \param _iP : The top left corner of the rectangle defining the region of
  interest (ROI). \param _height : The height of the ROI. \param _width : The
  width of the ROI.

  \return the number of reference points
*/
unsigned int vpPlanarObjectDetector::buildReference(const vpImage<unsigned char> &_I, const vpImagePoint &_iP,
                                                    unsigned int _height, unsigned int _width)
{
  unsigned int res = fern.buildReference(_I, _iP, _height, _width);
  modelROI.x = (int)_iP.get_u();
  modelROI.y = (int)_iP.get_v();
  modelROI.width = (int)_width;
  modelROI.height = (int)_height;

  initialiseRefCorners(modelROI);

  return res;
}

/*!
  Train the classifier on a region of the entire image. The region is a
  rectangle. The parameters of this rectangle must be given in pixel. It also
  includes the training of the fern classifier.

  \param _I : The image use to train the classifier.
  \param _rectangle : The rectangle defining the region of interest (ROI).

  \return The number of reference points.
*/
unsigned int vpPlanarObjectDetector::buildReference(const vpImage<unsigned char> &_I, const vpRect &_rectangle)
{
  unsigned int res = fern.buildReference(_I, _rectangle);

  vpImagePoint iP = _rectangle.getTopLeft();

  modelROI.x = (int)iP.get_u();
  modelROI.y = (int)iP.get_v();
  modelROI.width = (int)_rectangle.getWidth();
  modelROI.height = (int)_rectangle.getHeight();

  initialiseRefCorners(modelROI);

  return res;
}

/*!
  Compute the points of interest in the current image and try to recognise
  them using the trained classifier. The matched pairs can be found with the
  getMatchedPointByRef function. The homography between the two planar
  surfaces is also computed.

  \param I : The gray scaled image where the points are computed.

  \return True if the surface has been found.
*/
bool vpPlanarObjectDetector::matchPoint(const vpImage<unsigned char> &I)
{
  fern.matchPoint(I);

  /* compute homography */
  std::vector<cv::Point2f> refPts = fern.getRefPt();
  std::vector<cv::Point2f> curPts = fern.getCurPt();

  for (unsigned int i = 0; i < refPts.size(); ++i) {
    refPts[i].x += modelROI.x;
    refPts[i].y += modelROI.y;
  }
  for (unsigned int i = 0; i < curPts.size(); ++i) {
    curPts[i].x += modelROI.x;
    curPts[i].y += modelROI.y;
  }

  if (curPts.size() < 4) {
    for (unsigned int i = 0; i < 3; i += 1) {
      for (unsigned int j = 0; j < 3; j += 1) {
        if (i == j) {
          homography[i][j] = 1;
        } else {
          homography[i][j] = 0;
        }
      }
    }
    return false;
  }

  /* part of code from OpenCV planarObjectDetector */
  std::vector<unsigned char> mask;
  H = cv::findHomography(cv::Mat(refPts), cv::Mat(curPts), mask, cv::RANSAC, 10);

  if (H.data) {
    const cv::Mat_<double> &H_tmp = H;
    dst_corners.resize(4);
    for (unsigned int i = 0; i < 4; i++) {
      cv::Point2f pt = ref_corners[i];

      double w = 1. / (H_tmp(2, 0) * pt.x + H_tmp(2, 1) * pt.y + H_tmp(2, 2));
      dst_corners[i] = cv::Point2f((float)((H_tmp(0, 0) * pt.x + H_tmp(0, 1) * pt.y + H_tmp(0, 2)) * w),
                                   (float)((H_tmp(1, 0) * pt.x + H_tmp(1, 1) * pt.y + H_tmp(1, 2)) * w));
    }

    double *ptr = (double *)H_tmp.data;
    for (unsigned int i = 0; i < 9; i++) {
      this->homography[(unsigned int)(i / 3)][i % 3] = *(ptr++);
    }
    isCorrect = true;
  } else {
    isCorrect = false;
  }

  currentImagePoints.resize(0);
  refImagePoints.resize(0);
  for (unsigned int i = 0; i < mask.size(); i += 1) {
    if (mask[i] != 0) {
      vpImagePoint ip;
      ip.set_i(curPts[i].y);
      ip.set_j(curPts[i].x);
      currentImagePoints.push_back(ip);
      ip.set_i(refPts[i].y);
      ip.set_j(refPts[i].x);
      refImagePoints.push_back(ip);
    }
  }

  if (currentImagePoints.size() < minNbMatching) {
    isCorrect = false;
  }

  return isCorrect;
}

/*!
  Compute the points of interest in the specified region of the current image
  and try to recognise them using the trained classifier. The matched pairs
  can be found with the getMatchedPointByRef function. The homography between
  the two planar surfaces is also computed.

  \param I : The gray scaled image where the points are computed.
  \param iP : the top left corner of the rectangle defining the ROI
  \param height : the height of the ROI
  \param width : the width of the ROI

  \return true if the surface has been found.
*/
bool vpPlanarObjectDetector::matchPoint(const vpImage<unsigned char> &I, const vpImagePoint &iP,
                                        const unsigned int height, const unsigned int width)
{
  if ((iP.get_i() + height) >= I.getHeight() || (iP.get_j() + width) >= I.getWidth()) {
    vpTRACE("Bad size for the subimage");
    throw(vpException(vpImageException::notInTheImage, "Bad size for the subimage"));
  }

  vpImage<unsigned char> subImage;

  vpImageTools::crop(I, (unsigned int)iP.get_i(), (unsigned int)iP.get_j(), height, width, subImage);

  return this->matchPoint(subImage);
}

/*!
  Compute the points of interest in the specified region of the current image
  and try to recognise them using the trained classifier. The matched pairs
  can be found with the getMatchedPointByRef function. The homography between
  the two planar surfaces is also computed.

  \param I : The gray scaled image where the points are computed.
  \param rectangle : The rectangle defining the ROI.

  \return True if the surface has been found.
*/
bool vpPlanarObjectDetector::matchPoint(const vpImage<unsigned char> &I, const vpRect &rectangle)
{
  vpImagePoint iP;
  iP.set_i(rectangle.getTop());
  iP.set_j(rectangle.getLeft());
  return (this->matchPoint(I, iP, (unsigned int)rectangle.getHeight(), (unsigned int)rectangle.getWidth()));
}

/*!
  Display the result of the matching. The plane is displayed in red and the
  points of interests detected in the image are shown by a red dot surrounded
  by a green circle (the radius of the circle depends on the octave at which
  it has been detected).

  \param I : The gray scaled image for the display.
  \param displayKpts : The flag to display keypoints in addition to the
  surface.
*/
void vpPlanarObjectDetector::display(vpImage<unsigned char> &I, bool displayKpts)
{
  for (unsigned int i = 0; i < dst_corners.size(); i++) {
    vpImagePoint ip1(dst_corners[i].y - modelROI.y, dst_corners[i].x - modelROI.x);
    vpImagePoint ip2(dst_corners[(i + 1) % dst_corners.size()].y - modelROI.y,
                     dst_corners[(i + 1) % dst_corners.size()].x - modelROI.x);
    vpDisplay::displayLine(I, ip1, ip2, vpColor::red);
  }

  if (displayKpts) {
    for (unsigned int i = 0; i < currentImagePoints.size(); ++i) {
      vpImagePoint ip(currentImagePoints[i].get_i() - modelROI.y, currentImagePoints[i].get_j() - modelROI.x);
      vpDisplay::displayCross(I, ip, 5, vpColor::red);
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

  \param Iref : The image where the matched reference points are
  displayed.

  \param Icurrent : The image where the matched points computed in the
  current image are displayed.

  \param displayKpts : The flag to display keypoints in addition to the
  surface.
*/
void vpPlanarObjectDetector::display(vpImage<unsigned char> &Iref, vpImage<unsigned char> &Icurrent, bool displayKpts)
{
  display(Icurrent, displayKpts);

  if (displayKpts) {
    for (unsigned int i = 0; i < refImagePoints.size(); ++i) {
      vpDisplay::displayCross(Iref, refImagePoints[i], 5, vpColor::green);
    }
  }
}

/*!
  \brief Load the Fern classifier.

  Load and initialize the Fern classifier and load the 3D and 2D keypoints. It
  can take up to sevral seconds.

  \param dataFilename : The name of the data filename (very large text file).
  It can have any file extension.
  \param objName : The name of the object.
*/
void vpPlanarObjectDetector::load(const std::string &dataFilename, const std::string &objName)
{
  fern.load(dataFilename, objName);
  modelROI = fern.getModelROI();
  initialiseRefCorners(modelROI);
}

/*!
  \brief Record the Ferns classifier in the text file.

  \param objectName : The name of the object to store in the data file.
  \param dataFile : The name of the data filename (very large text file).
  It can have any file extension.
*/
void vpPlanarObjectDetector::recordDetector(const std::string &objectName, const std::string &dataFile)
{
  fern.record(objectName, dataFile);
}

/*!
  Return the last positions of the detected corners.

  \return The vectors of corners' postions.
*/
std::vector<vpImagePoint> vpPlanarObjectDetector::getDetectedCorners() const
{
  vpImagePoint ip;
  std::vector<vpImagePoint> corners;
  corners.clear();
  for (unsigned int i = 0; i < dst_corners.size(); i++) {
    ip.set_uv(dst_corners[i].x, dst_corners[i].y);
    corners.push_back(ip);
  }

  return corners;
}

/*!
  Initialise the internal reference corners from the rectangle.

  \param _modelROI : The rectangle defining the region of interest.
*/
void vpPlanarObjectDetector::initialiseRefCorners(const cv::Rect &_modelROI)
{
  cv::Point2f ip;

  ip.y = (float)_modelROI.y;
  ip.x = (float)_modelROI.x;
  ref_corners.push_back(ip);

  ip.y = (float)(_modelROI.y + _modelROI.height);
  ip.x = (float)_modelROI.x;
  ref_corners.push_back(ip);

  ip.y = (float)(_modelROI.y + _modelROI.height);
  ip.x = (float)(_modelROI.x + _modelROI.width);
  ref_corners.push_back(ip);

  ip.y = (float)_modelROI.y;
  ip.x = (float)(_modelROI.x + _modelROI.width);
  ref_corners.push_back(ip);
}

void vpPlanarObjectDetector::getReferencePoint(unsigned int _i, vpImagePoint &_imPoint)
{
  if (_i >= refImagePoints.size()) {
    throw vpException(vpException::fatalError, "index out of bound in getMatchedPoints.");
  }
  _imPoint = refImagePoints[_i];
}

void vpPlanarObjectDetector::getMatchedPoints(const unsigned int _index, vpImagePoint &_referencePoint,
                                              vpImagePoint &_currentPoint)
{
  //  fern.getMatchedPoints(_index, _referencePoint, _currentPoint);
  if (_index >= currentImagePoints.size()) {
    throw vpException(vpException::fatalError, "index out of bound in getMatchedPoints.");
  }

  _referencePoint = refImagePoints[_index];
  _currentPoint = currentImagePoints[_index];
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning:
// libvisp_vision.a(vpPlanarObjectDetector.cpp.o) has no symbols
void dummy_vpPlanarObjectDetector(){};
#endif
