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
  \file vpKltOpencv.h

  \brief Wrapper for the KLT (Kanade-Lucas-Tomasi) feature tracker
  implemented with opencv.
*/

#ifndef _vpKltOpencv_h_
#define _vpKltOpencv_h_

#include <visp3/core/vpColor.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

/*!
  \class vpKltOpencv

  \ingroup module_klt

  \brief Wrapper for the KLT (Kanade-Lucas-Tomasi) feature tracker
  implemented in OpenCV. Thus to enable this class OpenCV should be installed.
  Installation instructions are provided here
  https://visp.inria.fr/3rd_opencv.

  The following example available in tutorial-klt-tracker.cpp shows how to use
  the main functions of the class.

  \include tutorial-klt-tracker.cpp

  A line by line explanation is provided in \ref tutorial-tracking-keypoint.
*/
class VISP_EXPORT vpKltOpencv
{
public:
  vpKltOpencv();
  vpKltOpencv(const vpKltOpencv &copy);
  virtual ~vpKltOpencv();

  void addFeature(const float &x, const float &y);
  void addFeature(const long &id, const float &x, const float &y);
  void addFeature(const cv::Point2f &f);

  void display(const vpImage<unsigned char> &I, const vpColor &color = vpColor::red, unsigned int thickness = 1);
  static void display(const vpImage<unsigned char> &I, const std::vector<cv::Point2f> &features,
                      const vpColor &color = vpColor::green, unsigned int thickness = 1);
  static void display(const vpImage<vpRGBa> &I, const std::vector<cv::Point2f> &features,
                      const vpColor &color = vpColor::green, unsigned int thickness = 1);
  static void display(const vpImage<unsigned char> &I, const std::vector<cv::Point2f> &features,
                      const std::vector<long> &featuresid, const vpColor &color = vpColor::green,
                      unsigned int thickness = 1);
  static void display(const vpImage<vpRGBa> &I, const std::vector<cv::Point2f> &features,
                      const std::vector<long> &featuresid, const vpColor &color = vpColor::green,
                      unsigned int thickness = 1);

  //! Get the size of the averaging block used to track the features.
  int getBlockSize() const { return m_blockSize; }
  void getFeature(const int &index, long &id, float &x, float &y) const;
  //! Get the list of current features.
  std::vector<cv::Point2f> getFeatures() const { return m_points[1]; }
  // CvPoint2D32f* getFeatures() const {return features;}
  //! Get the unique id of each feature.
  std::vector<long> getFeaturesId() const { return m_points_id; }
  // long* getFeaturesId() const {return featuresid;}
  //! Get the free parameter of the Harris detector.
  double getHarrisFreeParameter() const { return m_harris_k; }
  //! Get the list of lost feature
  // bool *getListOfLostFeature() const { return lostDuringTrack; }
  //! Get the maximum number of features to track in the image.
  int getMaxFeatures() const { return m_maxCount; }
  //! Get the minimal Euclidean distance between detected corners during
  //! initialization.
  double getMinDistance() const { return m_minDistance; }
  //! Get the number of current features
  int getNbFeatures() const { return (int)m_points[1].size(); }
  //! Get the number of previous features.
  int getNbPrevFeatures() const { return (int)m_points[0].size(); }
  // void getPrevFeature(int index, int &id, float &x, float &y) const;
  //! Get the list of previous features
  std::vector<cv::Point2f> getPrevFeatures() const { return m_points[0]; }
  // CvPoint2D32f* getPrevFeatures() const {return prev_features;}
  //! Get the list of features id
  // long* getPrevFeaturesId() const {return prev_featuresid;}
  //! Get the maximal pyramid level.
  int getPyramidLevels() const { return m_pyrMaxLevel; }
  //! Get the parameter characterizing the minimal accepted quality of image
  //! corners.
  double getQuality() const { return m_qualityLevel; }
  //! Get the window size used to refine the corner locations.
  int getWindowSize() const { return m_winSize; }

  void initTracking(const cv::Mat &I, const cv::Mat &mask = cv::Mat());
  void initTracking(const cv::Mat &I, const std::vector<cv::Point2f> &pts);
  void initTracking(const cv::Mat &I, const std::vector<cv::Point2f> &pts, const std::vector<long> &ids);

  vpKltOpencv &operator=(const vpKltOpencv &copy);
  void track(const cv::Mat &I);
  void setBlockSize(int blockSize);
  void setHarrisFreeParameter(double harris_k);
  void setInitialGuess(const std::vector<cv::Point2f> &guess_pts);
  void setInitialGuess(const std::vector<cv::Point2f> &init_pts, const std::vector<cv::Point2f> &guess_pts,
                       const std::vector<long> &fid);
  void setMaxFeatures(int maxCount);
  void setMinDistance(double minDistance);
  void setMinEigThreshold(double minEigThreshold);
  void setPyramidLevels(int pyrMaxLevel);
  void setQuality(double qualityLevel);
  //! Does nothing. Just here for compat with previous releases that use
  //! OpenCV C api to do the tracking.
  void setTrackerId(int tid) { (void)tid; }
  void setUseHarris(int useHarrisDetector);
  void setWindowSize(int winSize);
  void suppressFeature(const int &index);

protected:
  cv::Mat m_gray, m_prevGray;
  std::vector<cv::Point2f> m_points[2]; //!< Previous [0] and current [1] keypoint location
  std::vector<long> m_points_id;        //!< Keypoint id
  int m_maxCount;
  cv::TermCriteria m_termcrit;
  int m_winSize;
  double m_qualityLevel;
  double m_minDistance;
  double m_minEigThreshold;
  double m_harris_k;
  int m_blockSize;
  int m_useHarrisDetector;
  int m_pyrMaxLevel;
  long m_next_points_id;
  bool m_initial_guess;
};

#endif
#endif
