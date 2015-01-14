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
#include <float.h>      // DBL_MAX
#include <map>          // std::map

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
#include <visp/vpPolygon.h>
#include <visp/vpXmlConfigParserKeyPoint.h>

// Require at least OpenCV >= 2.1.1
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#if defined(VISP_HAVE_OPENCV_XFEATURES2D) // OpenCV >= 3.0.0
#  include <opencv2/xfeatures2d.hpp>
#elif defined(VISP_HAVE_OPENCV_NONFREE) && (VISP_HAVE_OPENCV_VERSION >= 0x020400) && (VISP_HAVE_OPENCV_VERSION < 0x030000)
#  include <opencv2/nonfree/nonfree.hpp>
#endif

#ifdef VISP_HAVE_XML2
#  include <libxml/xmlwriter.h>
#endif

/*!
  \class vpKeyPoint

  \brief Class that allows keypoints detection (and descriptors extraction) and matching thanks to OpenCV library.
  This class permits to use different types of detectors, extractors and matchers easily.
  So, the classical SIFT and SURF keypoints could be used, as well as ORB, FAST, (etc.) keypoints,
  depending of the version of OpenCV you use.

  \note Due to some patents, SIFT and SURF are packaged in an external module called nonfree module
  in OpenCV version before 3.0.0 and in xfeatures2d from 3.0.0. You have to check you have the
  corresponding module to use SIFT and SURF.

  The goal of this class is to provide a tool to match reference keypoints from a
  reference image (or train keypoints in OpenCV terminology) and detected keypoints from a current image (or query
  keypoints in OpenCV terminology).

  If you supply the corresponding 3D coordinates corresponding to the 2D coordinates of the reference keypoints,
  you can also estimate the pose of the object by matching a set of detected keypoints in the current image with
  the reference keypoints.


  If you use this class, the first thing you have to do is to build
  the reference keypoints by detecting keypoints in a reference image which contains the
  object to detect. Then you match keypoints detected in a current image with those detected in a reference image
  by calling matchPoint() methods.
  You can access to the lists of matched points thanks to the
  methods getMatchedPointsInReferenceImage() and
  getMatchedPointsInCurrentImage(). These two methods return a list of
  matched points. The nth element of the first list is matched with
  the nth element of the second list.
  To provide easy compatibility with OpenCV terminology, getTrainKeyPoints() give you access to the list
  of keypoints detected in train images (or reference images) and getQueryKeyPoints() give you access to the list
  of keypoints detected in a query image (or current image).
  The method getMatches() give you access to a list of cv::DMatch with the correspondence between the index of the
  train keypoints and the index of the query keypoints.

  The following small example shows how to use the class to do the matching between current and reference keypoints.

  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpKeyPoint.h>

int main()
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
  vpImage<unsigned char> Ireference;
  vpImage<unsigned char> Icurrent;
  vpKeyPoint surf;

  // First grab the reference image Ireference
  // Add your code to load the reference image in Ireference

  // Build the reference SURF points.
  surf.buildReference(Ireference);

  // Then grab another image which represents the current image Icurrent

  // Match points between the reference points and the SURF points computed in the current image.
  surf.matchPoint(Icurrent);

  // Add your code to display image
  // Display the matched points
  surf.display(Ireference, Icurrent);

  return (0);
#endif
}
  \endcode

  It is also possible to build the reference keypoints in a region of interest (ROI) of an image
  and find keypoints to match in only a part of the current image. The small following example shows how to do this:

  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpKeyPoint.h>

int main()
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
  vpImage<unsigned char> Ireference;
  vpImage<unsigned char> Icurrent;
  vpKeyPoint surf;

  //First grab the reference image Ireference
  // Add your code to load the reference image in Ireference

  //Select a part of the image by clicking on top-left and bottom-right corners which define a ROI
  vpImagePoint corners[2];
  for (int i=0 ; i < 2 ; i++)
  {
    vpDisplay::getClick(Ireference, corners[i]);
  }

  //Build the reference SURF keypoints.
  int nbrRef;
  unsigned int height, width;
  height = (unsigned int) (corners[1].get_i() - corners[0].get_i());
  width = (unsigned int) (corners[1].get_j() - corners[0].get_j());
  nbrRef = surf.buildReference(Ireference, corners[0], height, width);

  //Then grab another image which represents the current image Icurrent

  //Select a part of the image by clicking on two points which define a rectangle
  for (int i=0 ; i < 2 ; i++)
  {
    vpDisplay::getClick(Icurrent, corners[i]);
  }

  //Match points between the reference keypoints and the SURF keypoints computed in the current image.
  int nbrMatched;
  height = (unsigned int)(corners[1].get_i() - corners[0].get_i());
  width = (unsigned int)(corners[1].get_j() - corners[0].get_j());
  nbrMatched = surf.matchPoint(Icurrent, corners[0], height, width);

  //Display the matched points
  surf.display(Ireference, Icurrent);

  return(0);
#endif
}
  \endcode

  This class is also described in \ref tutorial-matching.
*/
class VISP_EXPORT vpKeyPoint : public vpBasicKeyPoint {

public:

  /*! Predefined filtering method identifier. */
  typedef enum {
    constantFactorDistanceThreshold,  /*!< Keep all the points below a constant factor threshold. */
    stdDistanceThreshold,             /*!< Keep all the points below a minimal distance + the standard deviation. */
    ratioDistanceThreshold,           /*!< Keep all the points enough discriminated. */
    stdAndRatioDistanceThreshold,     /*!< Keep all the points which fall with the two conditions. */
    noFilterMatching                  /*!< No filtering. */
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

  static void compute3D(const cv::KeyPoint &candidate, const std::vector<vpPoint> &roi,
      const vpCameraParameters &cam, const vpHomogeneousMatrix &cMo, cv::Point3f &point);

  static void compute3D(const vpImagePoint &candidate, const std::vector<vpPoint> &roi,
      const vpCameraParameters &cam, const vpHomogeneousMatrix &cMo, vpPoint &point);

  static void compute3DForPointsInPolygons(const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
      std::vector<cv::KeyPoint> &candidate, std::vector<vpPolygon> &polygons, std::vector<std::vector<vpPoint> > &roisPt,
      std::vector<cv::Point3f> &points);

  static void compute3DForPointsInPolygons(const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
      std::vector<vpImagePoint> &candidate, std::vector<vpPolygon> &polygons, std::vector<std::vector<vpPoint> > &roisPt,
      std::vector<vpPoint> &points);

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

  /*!
    Get the elapsed time to compute the keypoint detection.

    \return The elapsed time.
  */
  inline double getDetectionTime() {
    return m_detectionTime;
  }

  /*!
    Get the elapsed time to compute the keypoint extraction.

    \return The elapsed time.
  */
  inline double getExtractionTime() {
    return m_extractionTime;
  }

  /*!
    Get the elapsed time to compute the matching.

    \return The elapsed time.
  */
  inline double getMatchingTime() {
    return m_matchingTime;
  }

  /*!
    Get the list of matches (correspondences between the indexes of the detected keypoints and the train keypoints).

    \return The list of matches.
  */
  inline std::vector<cv::DMatch> getMatches() {
    return m_filteredMatches;
  }

  /*!
    Get the list of pairs with the correspondence between the matched query and train keypoints.

    \return The list of pairs with the correspondence between the matched query and train keypoints.
  */
  inline std::vector<std::pair<cv::KeyPoint, cv::KeyPoint> > getMatchQueryToTrainKeyPoints() {
    return m_matchQueryToTrainKeyPoints;
  }

  /*!
    Get the number of train images.

    \return The number of train images.
  */
  inline unsigned int getNbImages() {
    return static_cast<unsigned int>(m_mapOfImages.size());
  }

  void getObjectPoints(std::vector<cv::Point3f> &objectPoints);

  bool getPose(const std::vector<cv::Point2f> &imagePoints, const std::vector<cv::Point3f> &objectPoints,
               const vpCameraParameters &cam, vpHomogeneousMatrix &cMo, std::vector<int> &inlierIndex, double &elapsedTime);

  bool getPose(const std::vector<vpPoint> &objectVpPoints, vpHomogeneousMatrix &cMo,
               std::vector<vpPoint> &inliers, double &elapsedTime, bool (*func)(vpHomogeneousMatrix *)=NULL);

  /*!
    Get the elapsed time to compute the pose.

    \return The elapsed time.
  */
  inline double getPoseTime() {
    return m_poseTime;
  }

  void getQueryDescriptors(cv::Mat &descriptors);
  void getQueryDescriptors(std::vector<std::vector<float> > &descriptors);

  void getQueryKeyPoints(std::vector<cv::KeyPoint> &keyPoints);
  void getQueryKeyPoints(std::vector<vpImagePoint> &keyPoints);

  /*!
    Get the list of Ransac inliers.

    \param inliers: List of Ransac inliers
    \return The list of Ransac inliers.
  */
  inline void getRansacInliers(std::vector<vpImagePoint> &inliers) {
    inliers = m_ransacInliers;
  }

  /*!
    Get the list of Ransac outliers.

    \param outliers: List of Ransac outliers
    \return The list of Ransac outliers.
  */
  inline void getRansacOutliers(std::vector<vpImagePoint> &outliers) {
    outliers = m_ransacOutliers;
  }

  void getTrainDescriptors(cv::Mat &descriptors);
  void getTrainDescriptors(std::vector<std::vector<float> > &descriptors);

  void getTrainKeyPoints(std::vector<cv::KeyPoint> &keyPoints);
  void getTrainKeyPoints(std::vector<vpImagePoint> &keyPoints);

  void getTrainPoints(std::vector<cv::Point3f> &points);
  void getTrainPoints(std::vector<vpPoint> &points);

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
                  double &error, double &elapsedTime, bool (*func)(vpHomogeneousMatrix *)=NULL);

  void saveLearningData(const std::string &filename, const bool binaryMode=false, const bool saveTrainingImages=true);

  /*!
     Set and initialize a detector denominated by his name \p detectorName.

     \param detectorName : Name of the detector.
   */
  inline void setDetector(const std::string &detectorName) {
    m_detectorNames.clear();
    m_detectorNames.push_back(detectorName);
    m_detectors.clear();
    initDetector(detectorName);
  }

  /*!
    Template function to set to a \p parameterName a value for a specific detector named by his \p detectorName.

    \param detectorName : Name of the detector
    \param parameterName : Name of the parameter
    \param value : Value to set
  */
  template<typename T1, typename T2, typename T3> inline void setDetectorParameter(const T1 detectorName,
                                                                                   const T2 parameterName, const T3 value) {
    if(m_detectors.find(detectorName) != m_detectors.end()) {
      m_detectors[detectorName]->set(parameterName, value);
    }
  }

  /*!
     Set and initialize a list of detectors denominated by their names \p detectorNames.

     \param detectorNames : List of detector names.
   */
  inline void setDetectors(const std::vector<std::string> &detectorNames) {
    m_detectorNames.clear();
    m_detectors.clear();
    m_detectorNames = detectorNames;
    initDetectors(m_detectorNames);
  }

  /*!
     Set and initialize an extractor denominated by his name \p extractorName.

     \param extractorName : Name of the extractor.
   */
  inline void setExtractor(const std::string &extractorName) {
    m_extractorNames.clear();
    m_extractorNames.push_back(extractorName);
    m_extractors.clear();
    initExtractor(extractorName);
  }

  /*!
    Template function to set to a \p parameterName a value for a specific extractor named by his \p extractorName.

    \param extractorName : Name of the extractor
    \param parameterName : Name of the parameter
    \param value : Value to set
  */
  template<typename T1, typename T2, typename T3> inline void setExtractorParameter(const T1 extractorName,
                                                                                    const T2 parameterName, const T3 value) {
    if(m_extractors.find(extractorName) != m_extractors.end()) {
      m_extractors[extractorName]->set(parameterName, value);
    }
  }

  /*!
     Set and initialize a list of extractors denominated by their names \p extractorNames.

     \param extractorNames : List of extractor names.
   */
  inline void setExtractors(const std::vector<std::string> &extractorNames) {
    m_extractorNames.clear();
    m_extractorNames = extractorNames;
    m_extractors.clear();
    initExtractors(m_extractorNames);
  }

  /*!
     Set and initialize a matcher denominated by his name \p matcherName.

     \param matcherName : Name of the matcher.
   */
  inline void setMatcher(const std::string &matcherName) {
    m_matcherName = matcherName;
    initMatcher(m_matcherName);
  }

  /*!
    Set the filtering method to eliminate false matching.

    \param filterType : Type of the filtering method
  */
  inline void setFilterMatchingType(const vpFilterMatchingType &filterType) {
    m_filterType = filterType;
  }

  /*!
    Set the factor value for the filtering method: constantFactorDistanceThreshold.

    \param factor : Factor value
  */
  inline void setMatchingFactorThreshold(const double factor) {
    m_matchingFactorThreshold = factor;
  }

  /*!
    Set the ratio value for the filtering method: ratioDistanceThreshold.

    \param ratio : Ratio value (]0 ; 1])
  */
  inline void setMatchingRatioThreshold(const double ratio) {
    m_matchingRatioThreshold = ratio;
  }

  /*!
    Set the percentage value for defining the cardinality of the consensus group.

    \param percentage : Percentage value (]0 ; 100])
  */
  inline void setRansacConsensusPercentage(const double percentage) {
    m_ransacConsensusPercentage = percentage;
  }

  /*!
    Set the maximum number of iterations for the Ransac pose estimation method.

    \param nbIter : Maximum number of iterations for the Ransac
  */
  inline void setRansacIteration(const int nbIter) {
    m_nbRansacIterations = nbIter;
  }

  /*!
    Set the maximum reprojection error (in pixel) to determine if a point is an inlier or not.

    \param reprojectionError : Maximum reprojection error in pixel (used by OpenCV function)
  */
  inline void setRansacReprojectionError(const double reprojectionError) {
    m_ransacReprojectionError = reprojectionError;
  }

  /*!
    Set the minimum number of inlier for the Ransac pose estimation method.

    \param minCount : Minimum number of inlier for the consensus
  */
  inline void setRansacMinInlierCount(const int minCount) {
    m_nbRansacMinInlierCount = minCount;
  }

  /*!
    Set the maximum error (in meter) to determine if a point is an inlier or not.

    \param threshold : Maximum error in meter for ViSP function
  */
  inline void setRansacThreshold(const double threshold) {
    m_ransacThreshold = threshold;
  }

  /*!
    Set the flag to choose between the OpenCV or ViSP Ransac pose estimation function.

    \param ransacVVS : True to use ViSP function, otherwise use OpenCV function
  */
  inline void setUseRansacVVS(const bool ransacVVS) {
    m_useRansacVVS = ransacVVS;
  }

  /*!
    Set the flag to choose between a percentage value of inliers for the cardinality of the consensus group
    or a minimum number.

    \param usePercentage : True to a percentage ratio of inliers, otherwise use a specified number of inliers
  */
  inline void setUseRansacConsensusPercentage(const bool usePercentage) {
    m_useConsensusPercentage = usePercentage;
  }

private:
  //! Current id associated to the training image used for the learning.
  int m_currentImageId;
  //! Elapsed time to detect keypoints.
  double m_detectionTime;
  //! List of detector names.
  std::vector<std::string> m_detectorNames;
  //! Map of smart reference-counting pointers (similar to shared_ptr in Boost) detectors,
  // with a key based upon the detector name.
  std::map<std::string, cv::Ptr<cv::FeatureDetector> > m_detectors;
  //! Elapsed time to extract descriptors for the detected keypoints.
  double m_extractionTime;
  //! List of extractor name.
  std::vector<std::string> m_extractorNames;
  //! Map of smart reference-counting pointers (similar to shared_ptr in Boost) extractors,
  // with a key based upon the extractor name.
  std::map<std::string, cv::Ptr<cv::DescriptorExtractor> > m_extractors;
  //! List of filtered matches between the detected and the trained keypoints.
  std::vector<cv::DMatch> m_filteredMatches;
  //! Chosen method of filtering to eliminate false matching.
  vpFilterMatchingType m_filterType;
  //! List of k-nearest neighbors for each detected keypoints (if the method chosen is based upon on knn).
  std::vector<std::vector<cv::DMatch> > m_knnMatches;
  //! Map of image id to know to which training image is related a training keypoints.
  std::map<int, int> m_mapOfImageId;
  //! Map of images to have access to the image buffer according to his image id.
  std::map<int, vpImage<unsigned char> > m_mapOfImages;
  //! Smart reference-counting pointer (similar to shared_ptr in Boost) of descriptor matcher (e.g. BruteForce or FlannBased).
  cv::Ptr<cv::DescriptorMatcher> m_matcher;
  //! Name of the matcher.
  std::string m_matcherName;
  //! List of matches between the detected and the trained keypoints.
  std::vector<cv::DMatch> m_matches;
  //! Factor value for the filtering method: constantFactorDistanceThreshold.
  double m_matchingFactorThreshold;
  //! Ratio value for the filtering method: ratioDistanceThreshold.
  double m_matchingRatioThreshold;
  //! Elapsed time to do the matching.
  double m_matchingTime;
  //! List of pairs between the matched query and train keypoints.
  std::vector<std::pair<cv::KeyPoint, cv::KeyPoint> > m_matchQueryToTrainKeyPoints;
  //! List of pairs between the keypoint and the 3D point after the Ransac.
  std::vector<std::pair<cv::KeyPoint, cv::Point3f> > m_matchRansacKeyPointsToPoints;
  //! List of pairs between the matched query and train keypoints after the Ransac.
  std::vector<std::pair<cv::KeyPoint, cv::KeyPoint> > m_matchRansacQueryToTrainKeyPoints;
  //! Maximum number of iterations for the Ransac method.
  int m_nbRansacIterations;
  //! Minimum number of inliers for the Ransac method.
  int m_nbRansacMinInlierCount;
  //! List of 3D points (in the object frame) filtered after the matching to compute the pose.
  std::vector<cv::Point3f> m_objectFilteredPoints;
  //! Elapsed time to compute the pose.
  double m_poseTime;
  /*! Matrix of descriptors (each row contains the descriptors values for each keypoints
      detected in the current image). */
  cv::Mat m_queryDescriptors;
  //! List of detected keypoints filtered after the matching.
  std::vector<cv::KeyPoint> m_queryFilteredKeyPoints;
  //! List of keypoints detected in the current image.
  std::vector<cv::KeyPoint> m_queryKeyPoints;
  //! Percentage value to determine the number of inliers for the Ransac method.
  double m_ransacConsensusPercentage;
  //! List of inliers.
  std::vector<vpImagePoint> m_ransacInliers;
  //! List of outliers.
  std::vector<vpImagePoint> m_ransacOutliers;
  //! Maximum reprojection error (in pixel for the OpenCV method) to decide if a point is an inlier or not.
  double m_ransacReprojectionError;
  //! Maximum error (in meter for the ViSP method) to decide if a point is an inlier or not.
  double m_ransacThreshold;
  //! Matrix of descriptors (each row contains the descriptors values for each keypoints
  //detected in the train images).
  cv::Mat m_trainDescriptors;
  //! List of keypoints detected in the train images.
  std::vector<cv::KeyPoint> m_trainKeyPoints;
  //! List of 3D points (in the object frame) corresponding to the train keypoints.
  std::vector<cv::Point3f> m_trainPoints;
  //! List of 3D points in vpPoint format (in the object frame) corresponding to the train keypoints.
  std::vector<vpPoint> m_trainVpPoints;
  //! Flag set if a percentage value is used to determine the number of inliers for the Ransac method.
  bool m_useConsensusPercentage;
  //! Flag set if a knn matching method must be used.
  bool m_useKnn;
  //! Flag set if a Ransac VVS pose estimation must be used.
  bool m_useRansacVVS;


  double computePoseEstimationError(const std::vector<std::pair<cv::KeyPoint, cv::Point3f> > &matchKeyPoints,
                                    const vpCameraParameters &cam, const vpHomogeneousMatrix &cMo_est);

  void filterMatches();

  void init();
  void initDetector(const std::string &detectorNames);
  void initDetectors(const std::vector<std::string> &detectorNames);

  void initExtractor(const std::string &extractorName);
  void initExtractors(const std::vector<std::string> &extractorNames);

//TODO: Try to implement a pyramidal feature detection
//#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
//  void pyramidFeatureDetection(cv::Ptr<cv::FeatureDetector> &detector, const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask,
//      const int maxLevel=2);
//  void runByPixelsMask(std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask);
//#endif
};

#endif
#endif
