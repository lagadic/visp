/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 * Wrapper for the KLT (Kanade-Lucas-Tomasi) feature tracker implemented
 * with opencv.
 *
*****************************************************************************/

/*!
  \file vpKltOpencv.cpp

  \brief Wrapper for the KLT (Kanade-Lucas-Tomasi) feature tracker
  implemented with opencv.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)

#include <string>

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/klt/vpKltOpencv.h>

/*!
  Default constructor.
 */
vpKltOpencv::vpKltOpencv()
  : m_gray(), m_prevGray(), m_points_id(), m_maxCount(500), m_termcrit(), m_winSize(10), m_qualityLevel(0.01),
    m_minDistance(15), m_minEigThreshold(1e-4), m_harris_k(0.04), m_blockSize(3), m_useHarrisDetector(1),
    m_pyrMaxLevel(3), m_next_points_id(0), m_initial_guess(false)
{
  m_termcrit = cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
}

/*!
  Copy constructor.
 */
vpKltOpencv::vpKltOpencv(const vpKltOpencv &copy)
  : m_gray(), m_prevGray(), m_points_id(), m_maxCount(500), m_termcrit(), m_winSize(10), m_qualityLevel(0.01),
    m_minDistance(15), m_minEigThreshold(1e-4), m_harris_k(0.04), m_blockSize(3), m_useHarrisDetector(1),
    m_pyrMaxLevel(3), m_next_points_id(0), m_initial_guess(false)
{
  *this = copy;
}

/*!
  Copy operator.
 */
vpKltOpencv &vpKltOpencv::operator=(const vpKltOpencv &copy)
{
  m_gray = copy.m_gray;
  m_prevGray = copy.m_prevGray;
  m_points[0] = copy.m_points[0];
  m_points[1] = copy.m_points[1];
  m_points_id = copy.m_points_id;
  m_maxCount = copy.m_maxCount;
  m_termcrit = copy.m_termcrit;
  m_winSize = copy.m_winSize;
  m_qualityLevel = copy.m_qualityLevel;
  m_minDistance = copy.m_minDistance;
  m_minEigThreshold = copy.m_minEigThreshold;
  m_harris_k = copy.m_harris_k;
  m_blockSize = copy.m_blockSize;
  m_useHarrisDetector = copy.m_useHarrisDetector;
  m_pyrMaxLevel = copy.m_pyrMaxLevel;
  m_next_points_id = copy.m_next_points_id;
  m_initial_guess = copy.m_initial_guess;

  return *this;
}

vpKltOpencv::~vpKltOpencv() {}

/*!
  Initialise the tracking by extracting KLT keypoints on the provided image.

  \param I : Grey level image used as input. This image should have only 1 channel.
  \param mask : Image mask used to restrict the keypoint detection
  area. If mask is NULL, all the image will be considered.

  \exception vpTrackingException::initializationError : If the image I is not
  initialized, or if the image or the mask have bad coding format.
*/
void vpKltOpencv::initTracking(const cv::Mat &I, const cv::Mat &mask)
{
  m_next_points_id = 0;

  // cvtColor(I, m_gray, cv::COLOR_BGR2GRAY);
  I.copyTo(m_gray);

  for (size_t i = 0; i < 2; i++) {
    m_points[i].clear();
  }

  m_points_id.clear();

  cv::goodFeaturesToTrack(m_gray, m_points[1], m_maxCount, m_qualityLevel, m_minDistance, mask, m_blockSize, false,
                          m_harris_k);

  if (m_points[1].size() > 0) {
    cv::cornerSubPix(m_gray, m_points[1], cv::Size(m_winSize, m_winSize), cv::Size(-1, -1), m_termcrit);

    for (size_t i = 0; i < m_points[1].size(); i++)
      m_points_id.push_back(m_next_points_id++);
  }
}

/*!
   Track KLT keypoints using the iterative Lucas-Kanade method with pyramids.

   \param I : Input image.
 */
void vpKltOpencv::track(const cv::Mat &I)
{
  if (m_points[1].size() == 0)
    throw vpTrackingException(vpTrackingException::fatalError, "Not enough key points to track.");

  std::vector<float> err;
  int flags = 0;

  cv::swap(m_prevGray, m_gray);

  if (m_initial_guess) {
    flags |= cv::OPTFLOW_USE_INITIAL_FLOW;
    m_initial_guess = false;
  } else {
    std::swap(m_points[1], m_points[0]);
  }

  // cvtColor(I, m_gray, cv::COLOR_BGR2GRAY);
  I.copyTo(m_gray);

  if (m_prevGray.empty()) {
    m_gray.copyTo(m_prevGray);
  }

  std::vector<uchar> status;

  cv::calcOpticalFlowPyrLK(m_prevGray, m_gray, m_points[0], m_points[1], status, err, cv::Size(m_winSize, m_winSize),
                           m_pyrMaxLevel, m_termcrit, flags, m_minEigThreshold);

  // Remove points that are lost
  for (int i = (int)status.size() - 1; i >= 0; i--) {
    if (status[(size_t)i] == 0) { // point is lost
      m_points[0].erase(m_points[0].begin() + i);
      m_points[1].erase(m_points[1].begin() + i);
      m_points_id.erase(m_points_id.begin() + i);
    }
  }
}

/*!

  Get the 'index'th feature image coordinates.  Beware that
  getFeature(i,...) may not represent the same feature before and
  after a tracking iteration (if a feature is lost, features are
  shifted in the array).

  \param index : Index of feature.
  \param id : id of the feature.
  \param x : x coordinate.
  \param y : y coordinate.

*/
void vpKltOpencv::getFeature(const int &index, long &id, float &x, float &y) const
{
  if ((size_t)index >= m_points[1].size()) {
    throw(vpException(vpException::badValue, "Feature [%d] doesn't exist", index));
  }

  x = m_points[1][(size_t)index].x;
  y = m_points[1][(size_t)index].y;
  id = m_points_id[(size_t)index];
}

/*!
  Display features position and id.

  \param I : Image used as background. Display should be initialized on it.
  \param color : Color used to display the features.
  \param thickness : Thickness of the drawings.
  */
void vpKltOpencv::display(const vpImage<unsigned char> &I, const vpColor &color, unsigned int thickness)
{
  vpKltOpencv::display(I, m_points[1], m_points_id, color, thickness);
}

/*!

  Display features list.

  \param I : The image used as background.

  \param features : Vector of features.

  \param color : Color used to display the points.

  \param thickness : Thickness of the points.
*/
void vpKltOpencv::display(const vpImage<unsigned char> &I, const std::vector<cv::Point2f> &features,
                          const vpColor &color, unsigned int thickness)
{
  vpImagePoint ip;
  for (size_t i = 0; i < features.size(); i++) {
    ip.set_u(vpMath::round(features[i].x));
    ip.set_v(vpMath::round(features[i].y));
    vpDisplay::displayCross(I, ip, 10 + thickness, color, thickness);
  }
}

/*!

  Display features list.

  \param I : The image used as background.

  \param features : Vector of features.

  \param color : Color used to display the points.

  \param thickness : Thickness of the points.
*/
void vpKltOpencv::display(const vpImage<vpRGBa> &I, const std::vector<cv::Point2f> &features, const vpColor &color,
                          unsigned int thickness)
{
  vpImagePoint ip;
  for (size_t i = 0; i < features.size(); i++) {
    ip.set_u(vpMath::round(features[i].x));
    ip.set_v(vpMath::round(features[i].y));
    vpDisplay::displayCross(I, ip, 10 + thickness, color, thickness);
  }
}

/*!

  Display features list with ids.

  \param I : The image used as background.

  \param features : Vector of features.

  \param featuresid : Vector of ids corresponding to the features.

  \param color : Color used to display the points.

  \param thickness : Thickness of the points
*/
void vpKltOpencv::display(const vpImage<unsigned char> &I, const std::vector<cv::Point2f> &features,
                          const std::vector<long> &featuresid, const vpColor &color, unsigned int thickness)
{
  vpImagePoint ip;
  for (size_t i = 0; i < features.size(); i++) {
    ip.set_u(vpMath::round(features[i].x));
    ip.set_v(vpMath::round(features[i].y));
    vpDisplay::displayCross(I, ip, 10, color, thickness);

    std::ostringstream id;
    id << featuresid[i];
    ip.set_u(vpMath::round(features[i].x + 5));
    vpDisplay::displayText(I, ip, id.str(), color);
  }
}

/*!

  Display features list with ids.

  \param I : The image used as background.

  \param features : Vector of features.

  \param featuresid : Vector of ids corresponding to the features.

  \param color : Color used to display the points.

  \param thickness : Thickness of the points
*/
void vpKltOpencv::display(const vpImage<vpRGBa> &I, const std::vector<cv::Point2f> &features,
                          const std::vector<long> &featuresid, const vpColor &color, unsigned int thickness)
{
  vpImagePoint ip;
  for (size_t i = 0; i < features.size(); i++) {
    ip.set_u(vpMath::round(features[i].x));
    ip.set_v(vpMath::round(features[i].y));
    vpDisplay::displayCross(I, ip, 10, color, thickness);

    std::ostringstream id;
    id << featuresid[i];
    ip.set_u(vpMath::round(features[i].x + 5));
    vpDisplay::displayText(I, ip, id.str(), color);
  }
}

/*!
  Set the maximum number of features to track in the image.

  \param maxCount : Maximum number of features to detect and track. Default
  value is set to 500.
*/
void vpKltOpencv::setMaxFeatures(int maxCount) { m_maxCount = maxCount; }

/*!
  Set the window size used to refine the corner locations.

  \param winSize : Half of the side length of the search window. Default value
  is set to 10. For example, if \e winSize=5 , then a 5*2+1 \f$\times\f$ 5*2+1
  = 11 \f$\times\f$ 11 search window is used.
*/
void vpKltOpencv::setWindowSize(int winSize) { m_winSize = winSize; }

/*!
  Set the parameter characterizing the minimal accepted quality of image
  corners.

  \param qualityLevel : Quality level parameter. Default value is set to 0.01.
  The parameter value is multiplied by the best corner quality measure, which
  is the minimal eigenvalue or the Harris function response. The corners with
  the quality measure less than the product are rejected. For example, if the
  best corner has the quality measure = 1500, and the qualityLevel=0.01, then
  all the corners with the quality measure less than 15 are rejected.
 */
void vpKltOpencv::setQuality(double qualityLevel) { m_qualityLevel = qualityLevel; }

/*!
  Set the free parameter of the Harris detector.

  \param harris_k : Free parameter of the Harris detector. Default value is
  set to 0.04.
*/
void vpKltOpencv::setHarrisFreeParameter(double harris_k) { m_harris_k = harris_k; }

/*!
  Set the parameter indicating whether to use a Harris detector or
  the minimal eigenvalue of gradient matrices for corner detection.
  \param useHarrisDetector : If 1 (default value), use the Harris detector. If
  0 use the eigenvalue.
*/
void vpKltOpencv::setUseHarris(int useHarrisDetector) { m_useHarrisDetector = useHarrisDetector; }

/*!
  Set the minimal Euclidean distance between detected corners during
  initialization.

  \param minDistance : Minimal possible Euclidean distance between the
  detected corners. Default value is set to 15.
*/
void vpKltOpencv::setMinDistance(double minDistance) { m_minDistance = minDistance; }

/*!
  Set the minimal eigen value threshold used to reject a point during the
  tracking. \param minEigThreshold : Minimal eigen value threshold. Default
  value is set to 1e-4.
*/
void vpKltOpencv::setMinEigThreshold(double minEigThreshold) { m_minEigThreshold = minEigThreshold; }

/*!
  Set the size of the averaging block used to track the features.

  \warning The input is a signed integer to be compatible with OpenCV.
  However, it must be a positive integer.

  \param blockSize : Size of an average block for computing a derivative
  covariation matrix over each pixel neighborhood. Default value is set to 3.
*/
void vpKltOpencv::setBlockSize(int blockSize) { m_blockSize = blockSize; }

/*!
  Set the maximal pyramid level. If the level is zero, then no pyramid is
  computed for the optical flow.

  \param pyrMaxLevel : 0-based maximal pyramid level number; if set to 0,
  pyramids are not used (single level), if set to 1, two levels are used, and
  so on. Default value is set to 3.
*/
void vpKltOpencv::setPyramidLevels(int pyrMaxLevel) { m_pyrMaxLevel = pyrMaxLevel; }

/*!
  Set the points that will be used as initial guess during the next call to
  track(). A typical usage of this function is to predict the position of the
  features before the next call to track().

  \param guess_pts : Vector of points that should be tracked. The size of this
  vector should be the same as the one returned by getFeatures(). If this is
  not the case, an exception is returned. Note also that the id of the points
  is not modified.

  \sa initTracking()
*/
void vpKltOpencv::setInitialGuess(const std::vector<cv::Point2f> &guess_pts)
{
  if (guess_pts.size() != m_points[1].size()) {
    throw(vpException(vpException::badValue,
                      "Cannot set initial guess: size feature vector [%d] "
                      "and guess vector [%d] doesn't match",
                      m_points[1].size(), guess_pts.size()));
  }

  m_points[1] = guess_pts;
  m_initial_guess = true;
}

/*!
  Set the points that will be used as initial guess during the next call to
  track(). A typical usage of this function is to predict the position of the
  features before the next call to track().

  \param init_pts : Initial points (could be obtained from getPrevFeatures()
  or getFeatures()). \param guess_pts : Prediction of the new position of the
  initial points. The size of this vector must be the same as the size of the
  vector of initial points. \param fid : Identifiers of the initial points.

  \sa getPrevFeatures(),getPrevFeaturesId
  \sa getFeatures(), getFeaturesId
  \sa initTracking()
*/
void vpKltOpencv::setInitialGuess(const std::vector<cv::Point2f> &init_pts, const std::vector<cv::Point2f> &guess_pts,
                                  const std::vector<long> &fid)
{
  if (guess_pts.size() != init_pts.size()) {
    throw(vpException(vpException::badValue,
                      "Cannot set initial guess: size init vector [%d] and "
                      "guess vector [%d] doesn't match",
                      init_pts.size(), guess_pts.size()));
  }

  m_points[0] = init_pts;
  m_points[1] = guess_pts;
  m_points_id = fid;
  m_initial_guess = true;
}

/*!
  Set the points that will be used as initialization during the next call to
  track().

  \param I : Input image.
  \param pts : Vector of points that should be tracked.

*/
void vpKltOpencv::initTracking(const cv::Mat &I, const std::vector<cv::Point2f> &pts)
{
  m_initial_guess = false;
  m_points[1] = pts;
  m_next_points_id = 0;
  m_points_id.clear();
  for (size_t i = 0; i < m_points[1].size(); i++) {
    m_points_id.push_back(m_next_points_id++);
  }

  I.copyTo(m_gray);
}

void vpKltOpencv::initTracking(const cv::Mat &I, const std::vector<cv::Point2f> &pts, const std::vector<long> &ids)
{
  m_initial_guess = false;
  m_points[1] = pts;
  m_points_id.clear();

  if (ids.size() != pts.size()) {
    m_next_points_id = 0;
    for (size_t i = 0; i < m_points[1].size(); i++)
      m_points_id.push_back(m_next_points_id++);
  } else {
    long max = 0;
    for (size_t i = 0; i < m_points[1].size(); i++) {
      m_points_id.push_back(ids[i]);
      if (ids[i] > max)
        max = ids[i];
    }
    m_next_points_id = max + 1;
  }

  I.copyTo(m_gray);
}

/*!

  Add a keypoint at the end of the feature list. The id of the feature is set
  to ensure that it is unique. \param x,y : Coordinates of the feature in the
  image.

*/
void vpKltOpencv::addFeature(const float &x, const float &y)
{
  cv::Point2f f(x, y);
  m_points[1].push_back(f);
  m_points_id.push_back(m_next_points_id++);
}

/*!

  Add a keypoint at the end of the feature list.

 \warning This function doesn't ensure that the id of the feature is unique.
  You should rather use addFeature(const float &, const float &) or
 addFeature(const cv::Point2f &).

  \param id : Feature id. Should be unique
  \param x,y : Coordinates of the feature in the image.

*/
void vpKltOpencv::addFeature(const long &id, const float &x, const float &y)
{
  cv::Point2f f(x, y);
  m_points[1].push_back(f);
  m_points_id.push_back(id);
  if (id >= m_next_points_id)
    m_next_points_id = id + 1;
}

/*!

  Add a keypoint at the end of the feature list. The id of the feature is set
  to ensure that it is unique. \param f : Coordinates of the feature in the
  image.

*/
void vpKltOpencv::addFeature(const cv::Point2f &f)
{
  m_points[1].push_back(f);
  m_points_id.push_back(m_next_points_id++);
}

/*!
   Remove the feature with the given index as parameter.
   \param index : Index of the feature to remove.
 */
void vpKltOpencv::suppressFeature(const int &index)
{
  if ((size_t)index >= m_points[1].size()) {
    throw(vpException(vpException::badValue, "Feature [%d] doesn't exist", index));
  }

  m_points[1].erase(m_points[1].begin() + index);
  m_points_id.erase(m_points_id.begin() + index);
}

#else

// Work around to avoid visp_klt library empty when OpenCV is not installed or used
class VISP_EXPORT dummy_vpKltOpencv
{
public:
  dummy_vpKltOpencv(){};
};

#if !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_klt.a(vpKltOpenCV.cpp.o) has no
// symbols
void dummy_vpKltOpenCV_fct(){};
#endif

#endif
