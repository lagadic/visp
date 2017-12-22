/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Wrapper for the KLT (Kanade-Lucas-Tomasi) feature tracker implemented
 * with opencv.
 *
 * Authors:
 * Fabien Servant
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpKltOpencv.h

  \brief Wrapper for the KLT (Kanade-Lucas-Tomasi) feature tracker
  implemented with opencv.
*/

#ifndef vpKltOpencv_h
#define vpKltOpencv_h

#include <visp3/core/vpColor.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>

#if (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020408))

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
  void setBlockSize(const int blockSize);
  void setHarrisFreeParameter(double harris_k);
  void setInitialGuess(const std::vector<cv::Point2f> &guess_pts);
  void setInitialGuess(const std::vector<cv::Point2f> &init_pts, const std::vector<cv::Point2f> &guess_pts,
                       const std::vector<long> &fid);
  void setMaxFeatures(const int maxCount);
  void setMinDistance(double minDistance);
  void setMinEigThreshold(double minEigThreshold);
  void setPyramidLevels(const int pyrMaxLevel);
  void setQuality(double qualityLevel);
  //! Does nothing. Just here for compat with previous releases that use
  //! OpenCV C api to do the tracking.
  void setTrackerId(int tid) { (void)tid; }
  void setUseHarris(const int useHarrisDetector);
  void setWindowSize(const int winSize);
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

#elif defined(VISP_HAVE_OPENCV)
#ifdef _CH_
#pragma package < opencv >
#endif

#if (VISP_HAVE_OPENCV_VERSION >= 0x020101) // Require opencv >= 2.1.1
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/video/tracking.hpp>
#else
#ifndef _EiC
#include <ctype.h>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#endif
#endif

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpTrackingException.h>

typedef int (*funccheck)(int, double, double);
typedef void (*funcinfo)(int, int, int, double, double);
typedef void (*funcevent)(int);

/*!
  \class vpKltOpencv

  \ingroup module_klt

  \brief Wrapper for the KLT (Kanade-Lucas-Tomasi) feature tracker
  implemented in OpenCV.

  The following example available in tutorial-klt-tracker.cpp shows how to use
  the main functions of the class.

  \include tutorial-klt-tracker.cpp

  A line by line explanation is provided in \ref tutorial-tracking-keypoint.
*/
class VISP_EXPORT vpKltOpencv
{
private:
  int initialized; // Is the tracker ready ?

  int maxFeatures;         // Maximum number of features to track (Default 50)
  int globalcountFeatures; // Keep over time for ID

  int win_size;                 // Size of search window for tracker (default 10)
  double quality;               // Multiplier for the maxmin eigenvalue; specifies minimal
                                // accepted quality of image corners (default 0.01)
  double min_distance;          // Limit, specifying minimum possible distance between
                                // returned corners; Euclidian distance is used.
                                // (default 10)
  double harris_free_parameter; // Harris detector free parameter. (default 0.04)
  int block_size;               // Size of the averaging block used by the corner detector
                                // (default 3)
  int use_harris;               // 0 use a simple Minimum EigenValue Detector, != 0  use
                                // Harris (default 1)
  int pyramid_level;            // Number of level for the tracker's gaussian pyramid
                                // data (default 3)
  int _tid;                     // tracker id for multiple trackers

  IplImage *image;        // Image buffer
  IplImage *prev_image;   // Image buffer for the previous iteration
  IplImage *pyramid;      // Gaussian pyramid data
  IplImage *prev_pyramid; // Gaussian pyramid data for the previous iteration
  IplImage *swap_temp;    // Internal

  int countFeatures;     // Currently tracked features
  int countPrevFeatures; // Previously tracked features

  CvPoint2D32f *features;      // List of features
  CvPoint2D32f *prev_features; // List of features for previous iteration

  long *featuresid;      // Array of ids for features
  long *prev_featuresid; // Array of ids for previous features

  int flags; // Flags for tracking (internal)

  bool initial_guess; // Bool to precise if the next call to track() uses an
                      // initial guess

  bool *lostDuringTrack; // Result of the tracker for every feature : 1 =
                         // lost, 0 = still present
  char *status;          // Result of the tracker for every features : 0 = lost, 1 =
                         // found

  // EVENT FUNCTION POINTERS
  funcevent OnInitialize;
  funcinfo OnFeatureLost;
  funcinfo OnNewFeature;
  funcinfo OnMeasureFeature;
  funccheck IsFeatureValid;

private:
  // Internal
  void clean();
  void cleanAll();
  void reset();

public:
  vpKltOpencv();
  vpKltOpencv(const vpKltOpencv &copy);
  virtual ~vpKltOpencv();

  void addFeature(const int &id, const float &x, const float &y);

  // Draw the tracked features on the given image
  void display(const vpImage<unsigned char> &I, vpColor color = vpColor::red, unsigned int thickness = 1);

  //! Get the block size
  int getBlockSize() const { return block_size; }
  void getFeature(int index, long &id, float &x, float &y) const;
  //! Get the list of features
  CvPoint2D32f *getFeatures() const { return features; }
  //! Get the list of features id
  long *getFeaturesId() const { return featuresid; }
  //! Get Harris free parameter
  double getHarrisFreeParameter() const { return harris_free_parameter; }
  //! Get the list of lost feature
  bool *getListOfLostFeature() const { return lostDuringTrack; }
  //! Get Max number of features
  int getMaxFeatures() const { return maxFeatures; }
  //! Get Min Distance
  double getMinDistance() const { return min_distance; }
  //! Get the current number of features
  int getNbFeatures() const { return countFeatures; }
  //! Get the previous number of features
  int getNbPrevFeatures() const { return countPrevFeatures; }
  void getPrevFeature(int index, int &id, float &x, float &y) const;
  //! Get the list of features
  CvPoint2D32f *getPrevFeatures() const { return prev_features; }
  //! Get the list of features id
  long *getPrevFeaturesId() const { return prev_featuresid; }
  //! Get the number of pyramid levels
  int getPyramidLevels() const { return pyramid_level; }
  //! Get the quality of the tracker
  double getQuality() const { return quality; }
  //! Get Max number of features
  int getWindowSize() const { return win_size; }

  // Detect corners in the image. Initialize the tracker
  void initTracking(const IplImage *I, const IplImage *mask = NULL);
  void initTracking(const IplImage *I, CvPoint2D32f *pts, int size);
  void initTracking(const IplImage *I, CvPoint2D32f *pts, long *fid, int size);
  vpKltOpencv &operator=(const vpKltOpencv &copy);
  // Track !
  void track(const IplImage *I);

  // Seters
  /*!
    Set the size of the averaging block used to track the features.

    \warning The input is a signed integer to be compatible with OpenCV.
    However, it must be a positive integer.

    \param input : The new size of the block.
  */
  void setBlockSize(const int input)
  {
    initialized = 0;
    block_size = input;
  }
  /*!
    Set the Harris parameter (The \e k value).

    \warning The tracker must be re-initialised using the method
    initTracking().

    \param input : The new Harris parameter.
  */
  void setHarrisFreeParameter(double input)
  {
    initialized = 0;
    harris_free_parameter = input;
  }
  void setInitialGuess(CvPoint2D32f **guess_pts);
  void setInitialGuess(CvPoint2D32f **init_pts, CvPoint2D32f **guess_pts, long *fid, int size);
  /*!
    Is a feature valid (e.g. : test if not too close to borders) ->
    event(id_tracker, x, y)
    */
  void setIsFeatureValid(funccheck input) { IsFeatureValid = input; }

  /* Should be used only before initTracking */
  void setMaxFeatures(const int input);
  /*!
    Set the minimal distance between two points during the initialisation.

    \warning The tracker must be re-initialised using the method
    initTracking().

    \param input : The new minimal distance between two points.
  */
  void setMinDistance(double input)
  {
    initialized = 0;
    min_distance = input;
  }

  // Functors

  // Event when tracker is initialized -> event(id_tracker)
  void setOnInitialize(funcevent input) { OnInitialize = input; }
  // Event when a feature is lost -> event(id_tracker, index, uid, x, y)
  void setOnFeatureLost(funcinfo input) { OnFeatureLost = input; }
  // Event when a new feature is found -> event(id_tracker, index, uid, x, y)
  void setOnNewFeature(funcinfo input) { OnNewFeature = input; }
  // Event when a feature is found while tracking -> event(id_tracker, index,
  // uid, x, y)
  void setOnMeasureFeature(funcinfo input) { OnMeasureFeature = input; }
  /*!
    Set the maximal pyramid level. If the level is zero, then no pyramid is
    computed for the optical flow.

    \warning The tracker must be re-initialised using the method
    initTracking().

    \param input : The new maximal pyramid level.
  */
  void setPyramidLevels(const int input)
  {
    initialized = 0;
    pyramid_level = input;
  }
  void setQuality(double input)
  {
    initialized = 0;
    quality = input;
  }
  void setTrackerId(int tid) { _tid = tid; }
  /*!
    Set the window size for the sub-pixel computation.

    \warning The tracker must be re-initialised using the method
    initTracking().

    \param input : The new number of maximum features.
  */
  void setUseHarris(const int input)
  {
    initialized = 0;
    use_harris = input;
  }
  void setWindowSize(const int input)
  {
    initialized = 0;
    win_size = input;
  }

  void suppressFeature(int index);

  // Static Functions
public:
  static void display(const vpImage<unsigned char> &I, const CvPoint2D32f *features_list, const int &nbFeatures,
                      vpColor color = vpColor::green, unsigned int thickness = 1);
  static void display(const vpImage<vpRGBa> &I, const CvPoint2D32f *features_list, const int &nbFeatures,
                      vpColor color = vpColor::green, unsigned int thickness = 1);

  static void display(const vpImage<unsigned char> &I, const CvPoint2D32f *features_list, const long *featuresid_list,
                      const int &nbFeatures, vpColor color = vpColor::green, unsigned int thickness = 1);
  static void display(const vpImage<vpRGBa> &I, const CvPoint2D32f *features_list, const long *featuresid_list,
                      const int &nbFeatures, vpColor color = vpColor::green, unsigned int thickness = 1);
};

#endif
#endif
