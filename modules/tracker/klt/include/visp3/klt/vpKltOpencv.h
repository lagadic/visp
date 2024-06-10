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

BEGIN_VISP_NAMESPACE
/*!
 * \class vpKltOpencv
 *
 * \ingroup module_klt
 *
 * \brief Wrapper for the KLT (Kanade-Lucas-Tomasi) feature tracker
 * implemented in OpenCV. Thus to enable this class OpenCV should be installed.
 * Installation instructions are provided here
 * https://visp.inria.fr/3rd_opencv.
 *
 * The following example available in tutorial-klt-tracker.cpp shows how to use
 * the main functions of the class.
 *
 * \include tutorial-klt-tracker.cpp
 *
 * A line by line explanation is provided in \ref tutorial-tracking-keypoint.
*/
class VISP_EXPORT vpKltOpencv
{
public:
  /*!
   * Default constructor.
   */
  vpKltOpencv();
  /*!
   * Copy constructor.
   */
  vpKltOpencv(const vpKltOpencv &copy);
  /*!
   * Destructor.
   */
  virtual ~vpKltOpencv();

  /*!
   * Add a keypoint at the end of the feature list. The id of the feature is set
   * to ensure that it is unique. \param x,y : Coordinates of the feature in the
   * image.
   */
  void addFeature(const float &x, const float &y);

  /*!
   * Add a keypoint at the end of the feature list.
   *
   * \warning This function doesn't ensure that the id of the feature is unique.
   * You should rather use addFeature(const float &, const float &) or
   * addFeature(const cv::Point2f &).
   *
   * \param id : Feature id. Should be unique
   * \param x,y : Coordinates of the feature in the image.
   */
  void addFeature(const long &id, const float &x, const float &y);

  /*!
   * Add a keypoint at the end of the feature list. The id of the feature is set
   * to ensure that it is unique.
   * \param f : Coordinates of the feature in the image.
   */
  void addFeature(const cv::Point2f &f);

  /*!
   * Display features position and id.
   *
   * \param I : Image used as background. Display should be initialized on it.
   * \param color : Color used to display the features.
   * \param thickness : Thickness of the drawings.
   */
  void display(const vpImage<unsigned char> &I, const vpColor &color = vpColor::red, unsigned int thickness = 1);
  /*!
   * Display features list.
   *
   * \param I : The image used as background.
   * \param features : Vector of features.
   * \param color : Color used to display the points.
   * \param thickness : Thickness of the points.
   */
  static void display(const vpImage<unsigned char> &I, const std::vector<cv::Point2f> &features,
                      const vpColor &color = vpColor::green, unsigned int thickness = 1);
  /*!
   * Display features list.
   *
   * \param I : The image used as background.
   * \param features : Vector of features.
   * \param color : Color used to display the points.
   * \param thickness : Thickness of the points.
   */
  static void display(const vpImage<vpRGBa> &I, const std::vector<cv::Point2f> &features,
                      const vpColor &color = vpColor::green, unsigned int thickness = 1);
  /*!
   * Display features list with ids.
   *
   * \param I : The image used as background.
   * \param features : Vector of features.
   * \param featuresid : Vector of ids corresponding to the features.
   * \param color : Color used to display the points.
   * \param thickness : Thickness of the points
   */
  static void display(const vpImage<unsigned char> &I, const std::vector<cv::Point2f> &features,
                        const std::vector<long> &featuresid, const vpColor &color = vpColor::green,
                        unsigned int thickness = 1);
  /*!
   * Display features list with ids.
   *
   * \param I : The image used as background.
   * \param features : Vector of features.
   * \param featuresid : Vector of ids corresponding to the features.
   * \param color : Color used to display the points.
   * \param thickness : Thickness of the points
   */
  static void display(const vpImage<vpRGBa> &I, const std::vector<cv::Point2f> &features,
                      const std::vector<long> &featuresid, const vpColor &color = vpColor::green,
                      unsigned int thickness = 1);

  //! Get the size of the averaging block used to track the features.
  int getBlockSize() const { return m_blockSize; }
  /*!
   * Get the 'index'th feature image coordinates.  Beware that
   * getFeature(i,...) may not represent the same feature before and
   * after a tracking iteration (if a feature is lost, features are
   * shifted in the array).
   *
   * \param index : Index of feature.
   * \param id : id of the feature.
   * \param x : x coordinate.
   * \param y : y coordinate.
   */
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

  /*!
   * Initialise the tracking by extracting KLT keypoints on the provided image.
   *
   * \param I : Grey level image used as input. This image should have only 1 channel.
   * \param mask : Image mask used to restrict the keypoint detection
   * area. If mask is nullptr, all the image will be considered.
   *
   * \exception vpTrackingException::initializationError : If the image I is not
   * initialized, or if the image or the mask have bad coding format.
   */
  void initTracking(const cv::Mat &I, const cv::Mat &mask = cv::Mat());

  /*!
   * Set the points that will be used as initialization during the next call to
   * track().
   *
   * \param I : Input image.
   * \param pts : Vector of points that should be tracked.
   */
  void initTracking(const cv::Mat &I, const std::vector<cv::Point2f> &pts);

  /*!
   * Set the points that will be used as initialization during the next call to
   * track().
   *
   * \param I : Input image.
   * \param pts : Vector of points that should be tracked.
   * \param ids : Corresponding point ids.
   */
  void initTracking(const cv::Mat &I, const std::vector<cv::Point2f> &pts, const std::vector<long> &ids);

  /*!
   * Copy operator.
   */
  vpKltOpencv &operator=(const vpKltOpencv &copy);

  /*!
   * Track KLT keypoints using the iterative Lucas-Kanade method with pyramids.
   *
   * \param I : Input image.
   */
  void track(const cv::Mat &I);

  /*!
   * Set the size of the averaging block used to track the features.
   *
   * \warning The input is a signed integer to be compatible with OpenCV.
   * However, it must be a positive integer.
   *
   * \param blockSize : Size of an average block for computing a derivative
   * covariation matrix over each pixel neighborhood. Default value is set to 3.
   */
  void setBlockSize(int blockSize) { m_blockSize = blockSize; }

  /*!
   * Set the free parameter of the Harris detector.
   *
   * \param harris_k : Free parameter of the Harris detector. Default value is
   * set to 0.04.
   */
  void setHarrisFreeParameter(double harris_k) { m_harris_k = harris_k; }

  /*!
   * Set the points that will be used as initial guess during the next call to
   * track(). A typical usage of this function is to predict the position of the
   * features before the next call to track().
   *
   * \param guess_pts : Vector of points that should be tracked. The size of this
   * vector should be the same as the one returned by getFeatures(). If this is
   * not the case, an exception is returned. Note also that the id of the points
   * is not modified.
   *
   * \sa initTracking()
   */
  void setInitialGuess(const std::vector<cv::Point2f> &guess_pts);

  /*!
   * Set the points that will be used as initial guess during the next call to
   * track(). A typical usage of this function is to predict the position of the
   * features before the next call to track().
   *
   * \param init_pts : Initial points (could be obtained from getPrevFeatures()
   * or getFeatures()).
   * \param guess_pts : Prediction of the new position of the
   * initial points. The size of this vector must be the same as the size of the
   * vector of initial points.
   * \param fid : Identifiers of the initial points.
   *
   * \sa getPrevFeatures(),getPrevFeaturesId
   * \sa getFeatures(), getFeaturesId
   * \sa initTracking()
   */
  void setInitialGuess(const std::vector<cv::Point2f> &init_pts, const std::vector<cv::Point2f> &guess_pts,
                       const std::vector<long> &fid);
  /*!
   * Set the maximum number of features to track in the image.
   *
   * \param maxCount : Maximum number of features to detect and track. Default
   * value is set to 500.
   */
  void setMaxFeatures(int maxCount) { m_maxCount = maxCount; }

  /*!
   * Set the minimal Euclidean distance between detected corners during
   * initialization.
   *
   * \param minDistance : Minimal possible Euclidean distance between the
   * detected corners. Default value is set to 15.
   */
  void setMinDistance(double minDistance) { m_minDistance = minDistance; }

  /*!
   * Set the minimal eigen value threshold used to reject a point during the
   * tracking.
   *
   * \param minEigThreshold : Minimal eigen value threshold. Default
   * value is set to 1e-4.
   */
  void setMinEigThreshold(double minEigThreshold) { m_minEigThreshold = minEigThreshold; }

  /*!
   * Set the maximal pyramid level. If the level is zero, then no pyramid is
   * computed for the optical flow.
   *
   * \param pyrMaxLevel : 0-based maximal pyramid level number; if set to 0,
   * pyramids are not used (single level), if set to 1, two levels are used, and
   * so on. Default value is set to 3.
   */
  void setPyramidLevels(int pyrMaxLevel) { m_pyrMaxLevel = pyrMaxLevel; }

  /*!
   * Set the parameter characterizing the minimal accepted quality of image
   * corners.
   *
   * \param qualityLevel : Quality level parameter. Default value is set to 0.01.
   * The parameter value is multiplied by the best corner quality measure, which
   * is the minimal eigenvalue or the Harris function response. The corners with
   * the quality measure less than the product are rejected. For example, if the
   * best corner has the quality measure = 1500, and the qualityLevel=0.01, then
   * all the corners with the quality measure less than 15 are rejected.
   */
  void setQuality(double qualityLevel) { m_qualityLevel = qualityLevel; }

  //! Does nothing. Just here for compat with previous releases that use
  //! OpenCV C api to do the tracking.
  void setTrackerId(int tid) { (void)tid; }

  /*!
   * Set the parameter indicating whether to use a Harris detector or
   * the minimal eigenvalue of gradient matrices for corner detection.
   * \param useHarrisDetector : If 1 (default value), use the Harris detector. If
   * 0 use the eigenvalue.
   */
  void setUseHarris(int useHarrisDetector) { m_useHarrisDetector = useHarrisDetector; }

  /*!
   * Set the window size used to refine the corner locations.
   *
   * \param winSize : Half of the side length of the search window. Default value
   * is set to 10. For example, if \e winSize=5 , then a 5*2+1 \f$\times\f$ 5*2+1
   * = 11 \f$\times\f$ 11 search window is used.
   */
  void setWindowSize(int winSize) { m_winSize = winSize; }

  /*!
   * Remove the feature with the given index as parameter.
   *
   * \param index : Index of the feature to remove.
   */
  void suppressFeature(const int &index);

protected:
  cv::Mat m_gray; //!< Gray image
  cv::Mat m_prevGray; //!< Previous gray image
  std::vector<cv::Point2f> m_points[2]; //!< Previous [0] and current [1] keypoint location
  std::vector<long> m_points_id;        //!< Keypoint id
  int m_maxCount; //!< Max number of keypoints
  cv::TermCriteria m_termcrit; //!< Term criteria
  int m_winSize; //!< Window criteria
  double m_qualityLevel; //!< Quality level
  double m_minDistance; //!< Mins distance between keypoints
  double m_minEigThreshold; //!< Min eigen threshold
  double m_harris_k; //!< Harris parameter
  int m_blockSize; //!< Block size
  int m_useHarrisDetector; //!< true to use Harris detector
  int m_pyrMaxLevel; //!< Pyramid max level
  long m_next_points_id; //!< Id for the newt keypoint
  bool m_initial_guess; //!< true when initial guess is provided
};
END_VISP_NAMESPACE
#endif
#endif
