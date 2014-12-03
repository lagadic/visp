/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
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
 * Description:
 * Key point functionalities.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/
#ifndef __vpKeyPoint_h__
#define __vpKeyPoint_h__

#include <algorithm>    // std::transform
#include <vector>       // std::vector
#include <stdlib.h>     // srand, rand
#include <time.h>       // time
#include <fstream>      // std::ofstream
#include <numeric>      // std::accumulate

#include <visp/vpConfig.h>
#include <visp/vpBasicKeyPoint.h>
#include <visp/vpImageConvert.h>
#include <visp/vpPoint.h>
#include <visp/vpDisplay.h>
#include <visp/vpPlane.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpIoTools.h>
#include <visp/vpPose.h>
#include <visp/vpImageIo.h>
#include <visp/vpXmlConfigParserKeyPoint.h>

// Require at least opencv >= 2.1.1 with the syntax used
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#if defined (VISP_HAVE_OPENCV_NONFREE)
#  include <opencv2/nonfree/nonfree.hpp>
#endif

#ifdef VISP_HAVE_XML2
#  include <libxml/xmlwriter.h>
#endif


class VISP_EXPORT vpKeyPoint : public vpBasicKeyPoint {

public:

  /*!
     Define different methods to filter false matching.
     Constant factor distance threshold : keep matching points with
     a distance < 2 * min_dist
     Standard deviation distance threshold : keep matching points
     with a distance < min_dist + std
     Ratio distance threshold : keep matching points if the distance
     ratio between their two nearest neighbors is < ratio_threshold
     Standard deviation and ratio distance threshold : keep matching points if
     at least one of the conditions are true
   */
  typedef enum {
    constantFactorDistanceThreshold,   //!< TODO
    stdDistanceThreshold,
    ratioDistanceThreshold,
    stdAndRatioDistanceThreshold,
    noFilterMatching
  } vpFilterMatchingType;

  vpKeyPoint(const std::string &detectorName="SIFT", const std::string &extractorName="SIFT",
             const std::string &matcherName="BruteForce", const vpFilterMatchingType &filterType=stdDistanceThreshold);
  vpKeyPoint(const std::vector<std::string> &detectorNames, const std::vector<std::string> &extractorNames,
             const std::string &matcherName="BruteForce", const vpFilterMatchingType &filterType=stdDistanceThreshold);

  unsigned int buildReference(const vpImage<unsigned char> &I);
  unsigned int buildReference(const vpImage<unsigned char> &I,
                              const vpImagePoint &iP, const unsigned int height, const unsigned int width);
  unsigned int buildReference(const vpImage<unsigned char> &I, const vpRect& rectangle);

  void buildReference(const vpImage<unsigned char> &I, std::vector<cv::KeyPoint> &trainKeyPoint,
                      std::vector<cv::Point3f> &points3f, bool append=false);

  static void convertToOpenCVType(const std::vector<vpImagePoint> &from, std::vector<cv::Point2f> &to);
  static void convertToOpenCVType(const std::vector<vpPoint> &from, std::vector<cv::Point3f> &to, const bool cameraFrame=false);

  static void convertToVpType(const std::vector<cv::KeyPoint> &from, std::vector<vpImagePoint> &to);
  static void convertToVpType(const std::vector<cv::Point2f> &from, std::vector<vpImagePoint> &to);
  static void convertToVpType(const std::vector<cv::Point3f> &from, std::vector<vpPoint> &to);
  static void convertToVpType(const std::vector<cv::DMatch> &from, std::vector<unsigned int> &to);

  void createImageMatching(vpImage<unsigned char> &IRef, vpImage<unsigned char> &ICurrent, vpImage<unsigned char> &IMatching);
  void createImageMatching(vpImage<unsigned char> &ICurrent, vpImage<unsigned char> &IMatching);

  void detect(const vpImage<unsigned char> &I, std::vector<cv::KeyPoint> &keyPoints, double &elapsedTime,
              const vpRect& rectangle=vpRect());

  void display(const vpImage<unsigned char> &IRef, const vpImage<unsigned char> &ICurrent, unsigned int size=3);
  void display(const vpImage<unsigned char> &ICurrent, unsigned int size=3, const vpColor &color=vpColor::green);

  void displayMatching(const vpImage<unsigned char> &IRef, vpImage<unsigned char> &IMatching,
                       unsigned int crossSize, unsigned int lineThickness=1,
                       const vpColor &color=vpColor::green);
  void displayMatching(const vpImage<unsigned char> &ICurrent, vpImage<unsigned char> &IMatching,
                       const std::vector<vpImagePoint> &ransacInliers=std::vector<vpImagePoint>(), unsigned int crossSize=3,
                       unsigned int lineThickness=1);

  void extract(const vpImage<unsigned char> &I, std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors, double &elapsedTime);

  inline double getDetectionTime() {
    return m_detectionTime;
  };

  inline double getExtractionTime() {
    return m_extractionTime;
  };

  inline double getMatchingTime() {
    return m_matchingTime;
  };

  inline std::vector<cv::DMatch> getMatches() {
    return m_filteredMatches;
  }

  inline std::vector<std::pair<cv::KeyPoint, cv::KeyPoint> > getMatchQueryToTrainKeyPoints() {
    return m_matchQueryToTrainKeyPoints;
  }

  inline unsigned int getNbImages() {
    return static_cast<unsigned int>(m_mapOfImages.size());
  }

  bool getPose(const std::vector<cv::Point2f> &imagePoints, const std::vector<cv::Point3f> &objectPoints,
               const vpCameraParameters &cam, vpHomogeneousMatrix &cMo, std::vector<int> &inlierIndex, double &elapsedTime);

  bool getPose(const std::vector<vpPoint> &objectVpPoints, vpHomogeneousMatrix &cMo,
               std::vector<vpPoint> &inliers, double &elapsedTime);

  inline double getPoseTime() {
    return m_poseTime;
  };

  void getObjectPoints(std::vector<cv::Point3f> &objectPoints);
  void getQueryDescriptors(cv::Mat &descriptors);
  void getQueryDescriptors(std::vector<std::vector<float> > &descriptors);

  void getQueryKeyPoints(std::vector<cv::KeyPoint> &keyPoints);
  void getQueryKeyPoints(std::vector<vpImagePoint> &keyPoints);

  inline void getRansacInliers(std::vector<vpImagePoint> &inliers) {
    inliers = m_ransacInliers;
  }

  inline void getRansacOutliers(std::vector<vpImagePoint> &outliers) {
    outliers = m_ransacOutliers;
  }

  void getTrainDescriptors(cv::Mat &descriptors);
  void getTrainDescriptors(std::vector<std::vector<float> > &descriptors);

  void getTrainKeyPoints(std::vector<cv::KeyPoint> &keyPoints);
  void getTrainKeyPoints(std::vector<vpImagePoint> &keyPoints);

  void initMatcher(const std::string &matcherName);

  void insertImageMatching(const vpImage<unsigned char> &IRef, const vpImage<unsigned char> &ICurrent,
                           vpImage<unsigned char> &IMatching);
  void insertImageMatching(const vpImage<unsigned char> &ICurrent, vpImage<unsigned char> &IMatching);

#ifdef VISP_HAVE_XML2
  void loadConfigFile(const std::string &configFile);
#endif

  void loadLearningData(const std::string &filename, const bool binaryMode=false, const bool append=false);

  void match(const cv::Mat &trainDescriptors, const cv::Mat &queryDescriptors,
             std::vector<cv::DMatch> &matches, double &elapsedTime);

  unsigned int matchPoint(const vpImage<unsigned char> &I);
  unsigned int matchPoint(const vpImage<unsigned char> &I, const vpImagePoint &iP,
                          const unsigned int height, const unsigned int width);
  unsigned int matchPoint(const vpImage<unsigned char> &I, const vpRect& rectangle);

  bool matchPoint(const vpImage<unsigned char> &I, const vpCameraParameters &cam, vpHomogeneousMatrix &cMo,
                  double &error, double &elapsedTime);

  void saveLearningData(const std::string &filename, const bool binaryMode=false, const bool saveTrainingImages=true);

  template<typename T1, typename T2, typename T3> inline void setDetectorParameter(const T1 detectorName,
                                                                                   const T2 parameterName, const T3 value) {
    m_detectors[detectorName]->set(parameterName, value);
  }

  template<typename T1, typename T2, typename T3> inline void setExtractorParameter(const T1 extractorName,
                                                                                    const T2 parameterName, const T3 value) {
    m_extractors[extractorName]->set(parameterName, value);
  }

  inline void setFilterMatchingType(const vpFilterMatchingType &filterType) {
    m_filterType = filterType;
  }

  inline void setMatchingFactorThreshold(const double factor) {
    m_matchingFactorThreshold = factor;
  }

  inline void setMatchingRatioThreshold(const double threshold) {
    m_matchingRatioThreshold = threshold;
  }

  inline void setRansacConsensusPercentage(const double percentage) {
    m_ransacConsensusPercentage = percentage;
  }

  inline void setRansacIteration(const int nbIter) {
    m_nbRansacIterations = nbIter;
  }

  inline void setRansacReprojectionError(const double reprojectionError) {
    m_ransacReprojectionError = reprojectionError;
  }

  inline void setRansacMinInlierCount(const int minCount) {
    m_nbRansacMinInlierCount = minCount;
  }

  inline void setRansacThreshold(const double threshold) {
    m_ransacThreshold = threshold;
  }

  inline void setUseRansacVVS(const bool ransacVVS) {
    m_useRansacVVS = ransacVVS;
  }

  inline void setUseRansacConsensusPercentage(const bool usePercentage) {
    m_useConsensusPercentage = usePercentage;
  }

private:

  std::vector<std::string> m_detectorNames;
  std::vector<std::string> m_extractorNames;
  std::string m_matcherName;
  vpFilterMatchingType m_filterType;
  bool m_useKnn;

  std::vector<cv::KeyPoint> m_trainKeyPoints;
  cv::Mat m_trainDescriptors;
  std::vector<cv::Point3f> m_trainPoints;
  std::vector<vpPoint> m_trainVpPoints;

  std::vector<cv::KeyPoint> m_queryKeyPoints;
  cv::Mat m_queryDescriptors;

  std::vector<cv::DMatch> m_matches;
  std::vector<std::vector<cv::DMatch> > m_knnMatches;

  std::vector<cv::KeyPoint> m_queryFilteredKeyPoints;
  std::vector<cv::Point3f> m_objectFilteredPoints;
  std::vector<cv::DMatch> m_filteredMatches;

  std::vector<std::pair<cv::KeyPoint, cv::Point3f> > m_matchRansacKeyPointsToPoints;
  std::vector<std::pair<cv::KeyPoint, cv::KeyPoint> > m_matchQueryToTrainKeyPoints;
  std::vector<std::pair<cv::KeyPoint, cv::KeyPoint> > m_matchRansacQueryToTrainKeyPoints;
  std::vector<vpImagePoint> m_ransacInliers;
  std::vector<vpImagePoint> m_ransacOutliers;

  std::map<std::string, cv::Ptr<cv::FeatureDetector> > m_detectors;
  std::map<std::string, cv::Ptr<cv::DescriptorExtractor> > m_extractors;
  cv::Ptr<cv::DescriptorMatcher> m_matcher;

  std::map<int, vpImage<unsigned char> > m_mapOfImages;
  std::map<int, int> m_mapOfImageId;

  int m_currentImageId;

  bool m_useRansacVVS;
  bool m_useConsensusPercentage;
  double m_matchingFactorThreshold;
  double m_matchingRatioThreshold;
  int m_nbRansacIterations;
  double m_ransacReprojectionError; //In pixel for OpenCV method
  int m_nbRansacMinInlierCount;
  double m_ransacThreshold; //In meter for vpPoseRansac method
  double m_ransacConsensusPercentage;

  double m_detectionTime;
  double m_extractionTime;
  double m_matchingTime;
  double m_poseTime;

  double computePoseEstimationError(const std::vector<std::pair<cv::KeyPoint, cv::Point3f> > &matchKeyPoints,
                                    const vpCameraParameters &cam, const vpHomogeneousMatrix &cMo_est);

  void filterMatches();

  void init();
  void initDetector(const std::string &detectorNames);
  void initDetectors(const std::vector<std::string> &detectorNames);

  void initExtractor(const std::string &extractorName);
  void initExtractors(const std::vector<std::string> &extractorNames);
};

#endif
#endif
