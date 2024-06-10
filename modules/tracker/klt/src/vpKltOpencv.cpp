/*
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
 */

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

BEGIN_VISP_NAMESPACE
vpKltOpencv::vpKltOpencv()
  : m_gray(), m_prevGray(), m_points_id(), m_maxCount(500), m_termcrit(), m_winSize(10), m_qualityLevel(0.01),
  m_minDistance(15), m_minEigThreshold(1e-4), m_harris_k(0.04), m_blockSize(3), m_useHarrisDetector(1),
  m_pyrMaxLevel(3), m_next_points_id(0), m_initial_guess(false)
{
  m_termcrit = cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
}

vpKltOpencv::vpKltOpencv(const vpKltOpencv &copy)
  : m_gray(), m_prevGray(), m_points_id(), m_maxCount(500), m_termcrit(), m_winSize(10), m_qualityLevel(0.01),
  m_minDistance(15), m_minEigThreshold(1e-4), m_harris_k(0.04), m_blockSize(3), m_useHarrisDetector(1),
  m_pyrMaxLevel(3), m_next_points_id(0), m_initial_guess(false)
{
  *this = copy;
}

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

vpKltOpencv::~vpKltOpencv() { }

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
  }
  else {
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

void vpKltOpencv::getFeature(const int &index, long &id, float &x, float &y) const
{
  if ((size_t)index >= m_points[1].size()) {
    throw(vpException(vpException::badValue, "Feature [%d] doesn't exist", index));
  }

  x = m_points[1][(size_t)index].x;
  y = m_points[1][(size_t)index].y;
  id = m_points_id[(size_t)index];
}

void vpKltOpencv::display(const vpImage<unsigned char> &I, const vpColor &color, unsigned int thickness)
{
  vpKltOpencv::display(I, m_points[1], m_points_id, color, thickness);
}

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
  }
  else {
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

void vpKltOpencv::addFeature(const float &x, const float &y)
{
  cv::Point2f f(x, y);
  m_points[1].push_back(f);
  m_points_id.push_back(m_next_points_id++);
}

void vpKltOpencv::addFeature(const long &id, const float &x, const float &y)
{
  cv::Point2f f(x, y);
  m_points[1].push_back(f);
  m_points_id.push_back(id);
  if (id >= m_next_points_id)
    m_next_points_id = id + 1;
}

void vpKltOpencv::addFeature(const cv::Point2f &f)
{
  m_points[1].push_back(f);
  m_points_id.push_back(m_next_points_id++);
}

void vpKltOpencv::suppressFeature(const int &index)
{
  if ((size_t)index >= m_points[1].size()) {
    throw(vpException(vpException::badValue, "Feature [%d] doesn't exist", index));
  }

  m_points[1].erase(m_points[1].begin() + index);
  m_points_id.erase(m_points_id.begin() + index);
}
END_VISP_NAMESPACE
#else

// Work around to avoid visp_klt library empty when OpenCV is not installed or used
class VISP_EXPORT dummy_vpKltOpencv
{
public:
  dummy_vpKltOpencv() { };
};

#if !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_klt.a(vpKltOpenCV.cpp.o) has no symbols
void dummy_vpKltOpenCV_fct() { };
#endif

#endif
