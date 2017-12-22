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
 * Key point functionalities.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/
#ifndef __vpKeyPoint_h__
#define __vpKeyPoint_h__

#include <algorithm> // std::transform
#include <float.h>   // DBL_MAX
#include <fstream>   // std::ofstream
#include <limits>
#include <map>      // std::map
#include <numeric>  // std::accumulate
#include <stdlib.h> // srand, rand
#include <time.h>   // time
#include <vector>   // std::vector

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPlane.h>
#include <visp3/core/vpPoint.h>
#include <visp3/vision/vpBasicKeyPoint.h>
#include <visp3/vision/vpPose.h>
#ifdef VISP_HAVE_MODULE_IO
#include <visp3/io/vpImageIo.h>
#endif
#include <visp3/core/vpConvert.h>
#include <visp3/core/vpCylinder.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPolygon.h>
#include <visp3/vision/vpXmlConfigParserKeyPoint.h>

// Require at least OpenCV >= 2.1.1
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#if defined(VISP_HAVE_OPENCV_XFEATURES2D) // OpenCV >= 3.0.0
#include <opencv2/xfeatures2d.hpp>
#elif defined(VISP_HAVE_OPENCV_NONFREE) && (VISP_HAVE_OPENCV_VERSION >= 0x020400) &&                                   \
    (VISP_HAVE_OPENCV_VERSION < 0x030000)
#include <opencv2/nonfree/nonfree.hpp>
#endif

#ifdef VISP_HAVE_XML2
#include <libxml/xmlwriter.h>
#endif

/*!
  \class vpKeyPoint
  \ingroup group_vision_keypoints

  \brief Class that allows keypoints detection (and descriptors extraction)
and matching thanks to OpenCV library. Thus to enable this class OpenCV should
be installed. Installation instructions are provided here
https://visp.inria.fr/3rd_opencv.

  This class permits to use different types of detectors, extractors and
matchers easily. So, the classical SIFT and SURF keypoints could be used, as
well as ORB, FAST, (etc.) keypoints, depending of the version of OpenCV you
use.

  \note Due to some patents, SIFT and SURF are packaged in an external module
called nonfree module in OpenCV version before 3.0.0 and in xfeatures2d
from 3.0.0. You have to check you have the corresponding module to use SIFT
and SURF.

  The goal of this class is to provide a tool to match reference keypoints
from a reference image (or train keypoints in OpenCV terminology) and detected
keypoints from a current image (or query keypoints in OpenCV terminology).

  If you supply the corresponding 3D coordinates corresponding to the 2D
coordinates of the reference keypoints, you can also estimate the pose of the
object by matching a set of detected keypoints in the current image with the
reference keypoints.

  If you use this class, the first thing you have to do is to build
  the reference keypoints by detecting keypoints in a reference image which
contains the object to detect. Then you match keypoints detected in a current
image with those detected in a reference image by calling matchPoint()
methods. You can access to the lists of matched points thanks to the methods
getMatchedPointsInReferenceImage() and getMatchedPointsInCurrentImage(). These
two methods return a list of matched points. The nth element of the first list
is matched with the nth element of the second list. To provide easy
compatibility with OpenCV terminology, getTrainKeyPoints() give you access to
the list of keypoints detected in train images (or reference images) and
getQueryKeyPoints() give you access to the list of keypoints detected in a
query image (or current image). The method getMatches() give you access to a
list of cv::DMatch with the correspondence between the index of the train
keypoints and the index of the query keypoints.

  The following small example shows how to use the class to do the matching
between current and reference keypoints.

  \code
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/vision/vpKeyPoint.h>

int main()
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020300)
  vpImage<unsigned char> Irefrence;
  vpImage<unsigned char> Icurrent;

  vpKeyPoint::vpFilterMatchingType filterType = vpKeyPoint::ratioDistanceThreshold;
  vpKeyPoint keypoint("ORB", "ORB", "BruteForce-Hamming", filterType);

  // First grab the reference image Irefrence
  // Add your code to load the reference image in Ireference

  // Build the reference ORB points.
  keypoint.buildReference(Irefrence);

  // Then grab another image which represents the current image Icurrent

  // Match points between the reference points and the ORB points computed in the current image.
  keypoint.matchPoint(Icurrent);

  // Display the matched points
  keypoint.display(Irefrence, Icurrent);
#endif

  return (0);
}
  \endcode

  It is also possible to build the reference keypoints in a region of interest
(ROI) of an image and find keypoints to match in only a part of the current
image. The small following example shows how to do this:

  \code
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/vision/vpKeyPoint.h>

int main()
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020300)
  vpImage<unsigned char> Ireference;
  vpImage<unsigned char> Icurrent;

  vpKeyPoint::vpFilterMatchingType filterType = vpKeyPoint::ratioDistanceThreshold;
  vpKeyPoint keypoint("ORB", "ORB", "BruteForce-Hamming", filterType);

  //First grab the reference image Irefrence
  //Add your code to load the reference image in Ireference

  //Select a part of the image by clincking on two points which define a rectangle
  vpImagePoint corners[2];
  for (int i=0 ; i < 2 ; i++) {
    vpDisplay::getClick(Ireference, corners[i]);
  }

  //Build the reference ORB points.
  int nbrRef;
  unsigned int height, width;
  height = (unsigned int)(corners[1].get_i() - corners[0].get_i());
  width = (unsigned int)(corners[1].get_j() - corners[0].get_j());
  nbrRef = keypoint.buildReference(Ireference, corners[0], height, width);

  //Then grab another image which represents the current image Icurrent

  //Select a part of the image by clincking on two points which define a rectangle
  for (int i=0 ; i < 2 ; i++) {
    vpDisplay::getClick(Icurrent, corners[i]);
  }

  //Match points between the reference points and the ORB points computed in the current image.
  int nbrMatched;
  height = (unsigned int)(corners[1].get_i() - corners[0].get_i());
  width = (unsigned int)(corners[1].get_j() - corners[0].get_j());
  nbrMatched = keypoint.matchPoint(Icurrent, corners[0], height, width);

  //Display the matched points
  keypoint.display(Ireference, Icurrent);
#endif

  return(0);
}
  \endcode

  This class is also described in \ref tutorial-matching.
*/
class VISP_EXPORT vpKeyPoint : public vpBasicKeyPoint
{

public:
  /*! Predefined filtering method identifier. */
  enum vpFilterMatchingType {
    constantFactorDistanceThreshold, /*!< Keep all the points below a constant
                                        factor threshold. */
    stdDistanceThreshold,            /*!< Keep all the points below a minimal distance +
                                        the standard deviation. */
    ratioDistanceThreshold,          /*!< Keep all the points enough discriminated (the
                                        ratio distance between the two best matches is
                                        below the threshold). */
    stdAndRatioDistanceThreshold,    /*!< Keep all the points which fall with the
                                        two conditions above. */
    noFilterMatching                 /*!< No filtering. */
  };

  /*! Predefined detection method identifier. */
  enum vpDetectionMethodType {
    detectionThreshold, /*!< The object is present if the average of the
                           descriptor distances is below the threshold. */
    detectionScore      /*!< Same condition than the previous but with a formula
                           taking into account the number of matches, the object is
                           present if the score is above the threshold. */
  };

  /*! Predefined constant for training image format. */
  typedef enum {
    jpgImageFormat, /*!< Save training images in JPG format. */
    pngImageFormat, /*!< Save training images in PNG format. */
    ppmImageFormat, /*!< Save training images in PPM format. */
    pgmImageFormat  /*!< Save training images in PGM format. */
  } vpImageFormatType;

  /*! Predefined constant for feature detection type. */
  enum vpFeatureDetectorType {
#if (VISP_HAVE_OPENCV_VERSION >= 0x020403)
    DETECTOR_FAST,
    DETECTOR_MSER,
    DETECTOR_ORB,
    DETECTOR_BRISK,
    DETECTOR_GFTT,
    DETECTOR_SimpleBlob,
#if (VISP_HAVE_OPENCV_VERSION < 0x030000) || (defined(VISP_HAVE_OPENCV_XFEATURES2D))
    DETECTOR_STAR,
#endif
#if defined(VISP_HAVE_OPENCV_NONFREE) || defined(VISP_HAVE_OPENCV_XFEATURES2D)
    DETECTOR_SIFT,
    DETECTOR_SURF,
#endif
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
    DETECTOR_KAZE,
    DETECTOR_AKAZE,
    DETECTOR_AGAST,
#endif
#if (VISP_HAVE_OPENCV_VERSION >= 0x030100) && defined(VISP_HAVE_OPENCV_XFEATURES2D)
    DETECTOR_MSD,
#endif
#endif
    DETECTOR_TYPE_SIZE
  };

  /*! Predefined constant for descriptor extraction type. */
  enum vpFeatureDescriptorType {
#if (VISP_HAVE_OPENCV_VERSION >= 0x020403)
    DESCRIPTOR_ORB,
    DESCRIPTOR_BRISK,
#if (VISP_HAVE_OPENCV_VERSION < 0x030000) || (defined(VISP_HAVE_OPENCV_XFEATURES2D))
    DESCRIPTOR_FREAK,
    DESCRIPTOR_BRIEF,
#endif
#if defined(VISP_HAVE_OPENCV_NONFREE) || defined(VISP_HAVE_OPENCV_XFEATURES2D)
    DESCRIPTOR_SIFT,
    DESCRIPTOR_SURF,
#endif
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
    DESCRIPTOR_KAZE,
    DESCRIPTOR_AKAZE,
#if defined(VISP_HAVE_OPENCV_XFEATURES2D)
    DESCRIPTOR_DAISY,
    DESCRIPTOR_LATCH,
#endif
#endif
#if (VISP_HAVE_OPENCV_VERSION >= 0x030200) && defined(VISP_HAVE_OPENCV_XFEATURES2D)
    DESCRIPTOR_VGG,
    DESCRIPTOR_BoostDesc,
#endif
#endif
    DESCRIPTOR_TYPE_SIZE
  };

  vpKeyPoint(const vpFeatureDetectorType &detectorType, const vpFeatureDescriptorType &descriptorType,
             const std::string &matcherName, const vpFilterMatchingType &filterType = ratioDistanceThreshold);
  vpKeyPoint(const std::string &detectorName = "ORB", const std::string &extractorName = "ORB",
             const std::string &matcherName = "BruteForce-Hamming",
             const vpFilterMatchingType &filterType = ratioDistanceThreshold);
  vpKeyPoint(const std::vector<std::string> &detectorNames, const std::vector<std::string> &extractorNames,
             const std::string &matcherName = "BruteForce",
             const vpFilterMatchingType &filterType = ratioDistanceThreshold);

  unsigned int buildReference(const vpImage<unsigned char> &I);
  unsigned int buildReference(const vpImage<unsigned char> &I, const vpImagePoint &iP, const unsigned int height,
                              const unsigned int width);
  unsigned int buildReference(const vpImage<unsigned char> &I, const vpRect &rectangle);

  void buildReference(const vpImage<unsigned char> &I, std::vector<cv::KeyPoint> &trainKeyPoint,
                      std::vector<cv::Point3f> &points3f, const bool append = false, const int class_id = -1);
  void buildReference(const vpImage<unsigned char> &I, const std::vector<cv::KeyPoint> &trainKeyPoint,
                      const cv::Mat &trainDescriptors, const std::vector<cv::Point3f> &points3f,
                      const bool append = false, const int class_id = -1);

  static void compute3D(const cv::KeyPoint &candidate, const std::vector<vpPoint> &roi, const vpCameraParameters &cam,
                        const vpHomogeneousMatrix &cMo, cv::Point3f &point);

  static void compute3D(const vpImagePoint &candidate, const std::vector<vpPoint> &roi, const vpCameraParameters &cam,
                        const vpHomogeneousMatrix &cMo, vpPoint &point);

  static void compute3DForPointsInPolygons(const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                                           std::vector<cv::KeyPoint> &candidates,
                                           const std::vector<vpPolygon> &polygons,
                                           const std::vector<std::vector<vpPoint> > &roisPt,
                                           std::vector<cv::Point3f> &points, cv::Mat *descriptors = NULL);

  static void compute3DForPointsInPolygons(const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                                           std::vector<vpImagePoint> &candidates,
                                           const std::vector<vpPolygon> &polygons,
                                           const std::vector<std::vector<vpPoint> > &roisPt,
                                           std::vector<vpPoint> &points, cv::Mat *descriptors = NULL);

  static void
  compute3DForPointsOnCylinders(const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                                std::vector<cv::KeyPoint> &candidates, const std::vector<vpCylinder> &cylinders,
                                const std::vector<std::vector<std::vector<vpImagePoint> > > &vectorOfCylinderRois,
                                std::vector<cv::Point3f> &points, cv::Mat *descriptors = NULL);

  static void
  compute3DForPointsOnCylinders(const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                                std::vector<vpImagePoint> &candidates, const std::vector<vpCylinder> &cylinders,
                                const std::vector<std::vector<std::vector<vpImagePoint> > > &vectorOfCylinderRois,
                                std::vector<vpPoint> &points, cv::Mat *descriptors = NULL);

  bool computePose(const std::vector<cv::Point2f> &imagePoints, const std::vector<cv::Point3f> &objectPoints,
                   const vpCameraParameters &cam, vpHomogeneousMatrix &cMo, std::vector<int> &inlierIndex,
                   double &elapsedTime, bool (*func)(vpHomogeneousMatrix *) = NULL);

  bool computePose(const std::vector<vpPoint> &objectVpPoints, vpHomogeneousMatrix &cMo, std::vector<vpPoint> &inliers,
                   double &elapsedTime, bool (*func)(vpHomogeneousMatrix *) = NULL);

  bool computePose(const std::vector<vpPoint> &objectVpPoints, vpHomogeneousMatrix &cMo, std::vector<vpPoint> &inliers,
                   std::vector<unsigned int> &inlierIndex, double &elapsedTime,
                   bool (*func)(vpHomogeneousMatrix *) = NULL);

  void createImageMatching(vpImage<unsigned char> &IRef, vpImage<unsigned char> &ICurrent,
                           vpImage<unsigned char> &IMatching);
  void createImageMatching(vpImage<unsigned char> &ICurrent, vpImage<unsigned char> &IMatching);

  void detect(const vpImage<unsigned char> &I, std::vector<cv::KeyPoint> &keyPoints,
              const vpRect &rectangle = vpRect());
  void detect(const cv::Mat &matImg, std::vector<cv::KeyPoint> &keyPoints, const cv::Mat &mask = cv::Mat());
  void detect(const vpImage<unsigned char> &I, std::vector<cv::KeyPoint> &keyPoints, double &elapsedTime,
              const vpRect &rectangle = vpRect());
  void detect(const cv::Mat &matImg, std::vector<cv::KeyPoint> &keyPoints, double &elapsedTime,
              const cv::Mat &mask = cv::Mat());

  void detectExtractAffine(const vpImage<unsigned char> &I, std::vector<std::vector<cv::KeyPoint> > &listOfKeypoints,
                           std::vector<cv::Mat> &listOfDescriptors,
                           std::vector<vpImage<unsigned char> > *listOfAffineI = NULL);

  void display(const vpImage<unsigned char> &IRef, const vpImage<unsigned char> &ICurrent, unsigned int size = 3);
  void display(const vpImage<unsigned char> &ICurrent, unsigned int size = 3, const vpColor &color = vpColor::green);

  void displayMatching(const vpImage<unsigned char> &IRef, vpImage<unsigned char> &IMatching, unsigned int crossSize,
                       unsigned int lineThickness = 1, const vpColor &color = vpColor::green);
  void displayMatching(const vpImage<unsigned char> &ICurrent, vpImage<unsigned char> &IMatching,
                       const std::vector<vpImagePoint> &ransacInliers = std::vector<vpImagePoint>(),
                       unsigned int crossSize = 3, unsigned int lineThickness = 1);

  void extract(const vpImage<unsigned char> &I, std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors,
               std::vector<cv::Point3f> *trainPoints = NULL);
  void extract(const cv::Mat &matImg, std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors,
               std::vector<cv::Point3f> *trainPoints = NULL);
  void extract(const vpImage<unsigned char> &I, std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors,
               double &elapsedTime, std::vector<cv::Point3f> *trainPoints = NULL);
  void extract(const cv::Mat &matImg, std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors, double &elapsedTime,
               std::vector<cv::Point3f> *trainPoints = NULL);

  /*!
    Get the covariance matrix when estimating the pose using the Virtual
    Visual Servoing approach.

    \warning The compute covariance flag has to be true if you want to compute
    the covariance matrix.

    \sa setCovarianceComputation
  */
  inline vpMatrix getCovarianceMatrix() const
  {
    if (!m_computeCovariance) {
      std::cout << "Warning : The covariance matrix has not been computed. "
                   "See setCovarianceComputation() to do it."
                << std::endl;
      return vpMatrix();
    }

    if (m_computeCovariance && !m_useRansacVVS) {
      std::cout << "Warning : The covariance matrix can only be computed "
                   "with a Virtual Visual Servoing approach."
                << std::endl
                << "Use setUseRansacVVS(true) to choose to use a pose "
                   "estimation method based on a Virtual Visual Servoing "
                   "approach."
                << std::endl;
      return vpMatrix();
    }

    return m_covarianceMatrix;
  }

  /*!
    Get the elapsed time to compute the keypoint detection.

    \return The elapsed time.
  */
  inline double getDetectionTime() const { return m_detectionTime; }

  /*!
    Get the detector pointer.
    \param type : Type of the detector.

    \return The detector or NULL if the type passed in parameter does not
    exist.
  */
  inline cv::Ptr<cv::FeatureDetector> getDetector(const vpFeatureDetectorType &type) const
  {
    std::map<vpFeatureDetectorType, std::string>::const_iterator it_name = m_mapOfDetectorNames.find(type);
    if (it_name == m_mapOfDetectorNames.end()) {
      std::cerr << "Internal problem with the feature type and the "
                   "corresponding name!"
                << std::endl;
    }

    std::map<std::string, cv::Ptr<cv::FeatureDetector> >::const_iterator findDetector =
        m_detectors.find(it_name->second);
    if (findDetector != m_detectors.end()) {
      return findDetector->second;
    }

    std::cerr << "Cannot find: " << it_name->second << std::endl;
    return cv::Ptr<cv::FeatureDetector>();
  }

  /*!
    Get the detector pointer.
    \param name : Name of the detector.

    \return The detector or NULL if the name passed in parameter does not
    exist.
  */
  inline cv::Ptr<cv::FeatureDetector> getDetector(const std::string &name) const
  {
    std::map<std::string, cv::Ptr<cv::FeatureDetector> >::const_iterator findDetector = m_detectors.find(name);
    if (findDetector != m_detectors.end()) {
      return findDetector->second;
    }

    std::cerr << "Cannot find: " << name << std::endl;
    return cv::Ptr<cv::FeatureDetector>();
  }

  /*!
    Get the feature detector name associated to the type.
  */
  inline std::map<vpFeatureDetectorType, std::string> getDetectorNames() const { return m_mapOfDetectorNames; }

  /*!
    Get the elapsed time to compute the keypoint extraction.

    \return The elapsed time.
  */
  inline double getExtractionTime() const { return m_extractionTime; }

  /*!
    Get the extractor pointer.
    \param type : Type of the descriptor extractor.

    \return The descriptor extractor or NULL if the name passed in parameter
    does not exist.
  */
  inline cv::Ptr<cv::DescriptorExtractor> getExtractor(const vpFeatureDescriptorType &type) const
  {
    std::map<vpFeatureDescriptorType, std::string>::const_iterator it_name = m_mapOfDescriptorNames.find(type);
    if (it_name == m_mapOfDescriptorNames.end()) {
      std::cerr << "Internal problem with the feature type and the "
                   "corresponding name!"
                << std::endl;
    }

    std::map<std::string, cv::Ptr<cv::DescriptorExtractor> >::const_iterator findExtractor =
        m_extractors.find(it_name->second);
    if (findExtractor != m_extractors.end()) {
      return findExtractor->second;
    }

    std::cerr << "Cannot find: " << it_name->second << std::endl;
    return cv::Ptr<cv::DescriptorExtractor>();
  }

  /*!
    Get the extractor pointer.
    \param name : Name of the descriptor extractor.

    \return The descriptor extractor or NULL if the name passed in parameter
    does not exist.
  */
  inline cv::Ptr<cv::DescriptorExtractor> getExtractor(const std::string &name) const
  {
    std::map<std::string, cv::Ptr<cv::DescriptorExtractor> >::const_iterator findExtractor = m_extractors.find(name);
    if (findExtractor != m_extractors.end()) {
      return findExtractor->second;
    }

    std::cerr << "Cannot find: " << name << std::endl;
    return cv::Ptr<cv::DescriptorExtractor>();
  }

  /*!
    Get the feature descriptor extractor name associated to the type.
  */
  inline std::map<vpFeatureDescriptorType, std::string> getExtractorNames() const { return m_mapOfDescriptorNames; }

  /*!
    Get the image format to use when saving training images.

    \return The image format.
  */
  inline vpImageFormatType getImageFormat() const { return m_imageFormat; }

  /*!
    Get the elapsed time to compute the matching.

    \return The elapsed time.
  */
  inline double getMatchingTime() const { return m_matchingTime; }

  /*!
    Get the matcher pointer.

    \return The matcher pointer.
  */
  inline cv::Ptr<cv::DescriptorMatcher> getMatcher() const { return m_matcher; }

  /*!
    Get the list of matches (correspondences between the indexes of the
    detected keypoints and the train keypoints).

    \return The list of matches.
  */
  inline std::vector<cv::DMatch> getMatches() const { return m_filteredMatches; }

  /*!
    Get the list of pairs with the correspondence between the matched query
    and train keypoints.

    \return The list of pairs with the correspondence between the matched
    query and train keypoints.
  */
  inline std::vector<std::pair<cv::KeyPoint, cv::KeyPoint> > getMatchQueryToTrainKeyPoints() const
  {
    std::vector<std::pair<cv::KeyPoint, cv::KeyPoint> > matchQueryToTrainKeyPoints(m_filteredMatches.size());
    for (size_t i = 0; i < m_filteredMatches.size(); i++) {
      matchQueryToTrainKeyPoints.push_back(
          std::pair<cv::KeyPoint, cv::KeyPoint>(m_queryFilteredKeyPoints[(size_t)m_filteredMatches[i].queryIdx],
                                                m_trainKeyPoints[(size_t)m_filteredMatches[i].trainIdx]));
    }
    return matchQueryToTrainKeyPoints;
  }

  /*!
    Get the number of train images.

    \return The number of train images.
  */
  inline unsigned int getNbImages() const { return static_cast<unsigned int>(m_mapOfImages.size()); }

  void getObjectPoints(std::vector<cv::Point3f> &objectPoints) const;
  void getObjectPoints(std::vector<vpPoint> &objectPoints) const;

  /*!
    Get the elapsed time to compute the pose.

    \return The elapsed time.
  */
  inline double getPoseTime() const { return m_poseTime; }

  /*!
     Get the descriptors matrix for the query keypoints.

     \return Matrix with descriptors values at each row for each query
     keypoints.
   */
  inline cv::Mat getQueryDescriptors() const { return m_queryDescriptors; }

  void getQueryKeyPoints(std::vector<cv::KeyPoint> &keyPoints) const;
  void getQueryKeyPoints(std::vector<vpImagePoint> &keyPoints) const;

  /*!
    Get the list of Ransac inliers.

    \return The list of Ransac inliers.
  */
  inline std::vector<vpImagePoint> getRansacInliers() const { return m_ransacInliers; }

  /*!
    Get the list of Ransac outliers.

    \return The list of Ransac outliers.
  */
  inline std::vector<vpImagePoint> getRansacOutliers() const { return m_ransacOutliers; }

  /*!
     Get the train descriptors matrix.

     \return : Matrix with descriptors values at each row for each train
     keypoints (or reference keypoints).
   */
  inline cv::Mat getTrainDescriptors() const { return m_trainDescriptors; }

  void getTrainKeyPoints(std::vector<cv::KeyPoint> &keyPoints) const;
  void getTrainKeyPoints(std::vector<vpImagePoint> &keyPoints) const;

  void getTrainPoints(std::vector<cv::Point3f> &points) const;
  void getTrainPoints(std::vector<vpPoint> &points) const;

  void initMatcher(const std::string &matcherName);

  void insertImageMatching(const vpImage<unsigned char> &IRef, const vpImage<unsigned char> &ICurrent,
                           vpImage<unsigned char> &IMatching);
  void insertImageMatching(const vpImage<unsigned char> &ICurrent, vpImage<unsigned char> &IMatching);

#ifdef VISP_HAVE_XML2
  void loadConfigFile(const std::string &configFile);
#endif

  void loadLearningData(const std::string &filename, const bool binaryMode = false, const bool append = false);

  void match(const cv::Mat &trainDescriptors, const cv::Mat &queryDescriptors, std::vector<cv::DMatch> &matches,
             double &elapsedTime);

  unsigned int matchPoint(const vpImage<unsigned char> &I);
  unsigned int matchPoint(const vpImage<unsigned char> &I, const vpImagePoint &iP, const unsigned int height,
                          const unsigned int width);
  unsigned int matchPoint(const vpImage<unsigned char> &I, const vpRect &rectangle);

  bool matchPoint(const vpImage<unsigned char> &I, const vpCameraParameters &cam, vpHomogeneousMatrix &cMo,
                  bool (*func)(vpHomogeneousMatrix *) = NULL, const vpRect &rectangle = vpRect());
  bool matchPoint(const vpImage<unsigned char> &I, const vpCameraParameters &cam, vpHomogeneousMatrix &cMo,
                  double &error, double &elapsedTime, bool (*func)(vpHomogeneousMatrix *) = NULL,
                  const vpRect &rectangle = vpRect());

  bool matchPointAndDetect(const vpImage<unsigned char> &I, vpRect &boundingBox, vpImagePoint &centerOfGravity,
                           const bool isPlanarObject = true, std::vector<vpImagePoint> *imPts1 = NULL,
                           std::vector<vpImagePoint> *imPts2 = NULL, double *meanDescriptorDistance = NULL,
                           double *detectionScore = NULL, const vpRect &rectangle = vpRect());

  bool matchPointAndDetect(const vpImage<unsigned char> &I, const vpCameraParameters &cam, vpHomogeneousMatrix &cMo,
                           double &error, double &elapsedTime, vpRect &boundingBox, vpImagePoint &centerOfGravity,
                           bool (*func)(vpHomogeneousMatrix *) = NULL, const vpRect &rectangle = vpRect());

  void reset();

  void saveLearningData(const std::string &filename, const bool binaryMode = false,
                        const bool saveTrainingImages = true);

  /*!
    Set if the covariance matrix has to be computed in the Virtual Visual
    Servoing approach.

    \param flag : True if the covariance has to be computed, false otherwise.
  */
  inline void setCovarianceComputation(const bool &flag)
  {
    m_computeCovariance = flag;
    if (!m_useRansacVVS) {
      std::cout << "Warning : The covariance matrix can only be computed "
                   "with a Virtual Visual Servoing approach."
                << std::endl
                << "Use setUseRansacVVS(true) to choose to use a pose "
                   "estimation method based on a Virtual "
                   "Visual Servoing approach."
                << std::endl;
    }
  }

  /*!
     Set the method to decide if the object is present or not.

     \param method : Detection method (detectionThreshold or detectionScore).
   */
  inline void setDetectionMethod(const vpDetectionMethodType &method) { m_detectionMethod = method; }

  /*!
     Set and initialize a detector.

     \param detectorType : Type of the detector.
   */
  inline void setDetector(const vpFeatureDetectorType &detectorType)
  {
    m_detectorNames.clear();
    m_detectorNames.push_back(m_mapOfDetectorNames[detectorType]);
    m_detectors.clear();
    initDetector(m_mapOfDetectorNames[detectorType]);
  }

  /*!
     Set and initialize a detector denominated by his name \p detectorName.

     \param detectorName : Name of the detector.
   */
  inline void setDetector(const std::string &detectorName)
  {
    m_detectorNames.clear();
    m_detectorNames.push_back(detectorName);
    m_detectors.clear();
    initDetector(detectorName);
  }

#if (VISP_HAVE_OPENCV_VERSION >= 0x020400 && VISP_HAVE_OPENCV_VERSION < 0x030000)
  /*!
    Template function to set to a \p parameterName a value for a specific
    detector named by his \p detectorName.

    \param detectorName : Name of the detector
    \param parameterName : Name of the parameter
    \param value : Value to set
  */
  template <typename T1, typename T2, typename T3>
  inline void setDetectorParameter(const T1 detectorName, const T2 parameterName, const T3 value)
  {
    if (m_detectors.find(detectorName) != m_detectors.end()) {
      m_detectors[detectorName]->set(parameterName, value);
    }
  }
#endif

  /*!
     Set and initialize a list of detectors denominated by their names \p
     detectorNames.

     \param detectorNames : List of detector names.
   */
  inline void setDetectors(const std::vector<std::string> &detectorNames)
  {
    m_detectorNames.clear();
    m_detectors.clear();
    m_detectorNames = detectorNames;
    initDetectors(m_detectorNames);
  }

  /*!
     Set and initialize a descriptor extractor.

     \param extractorType : Type of the descriptor extractor.
   */
  inline void setExtractor(const vpFeatureDescriptorType &extractorType)
  {
    m_extractorNames.clear();
    m_extractorNames.push_back(m_mapOfDescriptorNames[extractorType]);
    m_extractors.clear();
    initExtractor(m_mapOfDescriptorNames[extractorType]);
  }

  /*!
     Set and initialize a descriptor extractor denominated by his name \p
     extractorName.

     \param extractorName : Name of the extractor.
   */
  inline void setExtractor(const std::string &extractorName)
  {
    m_extractorNames.clear();
    m_extractorNames.push_back(extractorName);
    m_extractors.clear();
    initExtractor(extractorName);
  }

#if (VISP_HAVE_OPENCV_VERSION >= 0x020400 && VISP_HAVE_OPENCV_VERSION < 0x030000)
  /*!
    Template function to set to a \p parameterName a value for a specific
    extractor named by his \p extractorName.

    \param extractorName : Name of the extractor
    \param parameterName : Name of the parameter
    \param value : Value to set
  */
  template <typename T1, typename T2, typename T3>
  inline void setExtractorParameter(const T1 extractorName, const T2 parameterName, const T3 value)
  {
    if (m_extractors.find(extractorName) != m_extractors.end()) {
      m_extractors[extractorName]->set(parameterName, value);
    }
  }
#endif

  /*!
     Set and initialize a list of extractors denominated by their names \p
     extractorNames.

     \param extractorNames : List of extractor names.
   */
  inline void setExtractors(const std::vector<std::string> &extractorNames)
  {
    m_extractorNames.clear();
    m_extractorNames = extractorNames;
    m_extractors.clear();
    initExtractors(m_extractorNames);
  }

  /*!
    Set the image format to use when saving training images.

    \param imageFormat : The image format.
  */
  inline void setImageFormat(const vpImageFormatType &imageFormat) { m_imageFormat = imageFormat; }

  /*!
     Set and initialize a matcher denominated by his name \p matcherName.
     The different matchers are:
       - BruteForce (it uses L2 distance)
       - BruteForce-L1
       - BruteForce-Hamming
       - BruteForce-Hamming(2)
       - FlannBased

     L1 and L2 norms are preferable choices for SIFT and SURF descriptors,
     NORM_HAMMING should be used with ORB, BRISK and BRIEF, NORM_HAMMING2
     should be used with ORB when WTA_K==3 or 4.

     \param matcherName : Name of the matcher.
   */
  inline void setMatcher(const std::string &matcherName)
  {
    m_matcherName = matcherName;
    initMatcher(m_matcherName);
  }

  /*!
    Set the filtering method to eliminate false matching.
    The different methods are:
      - constantFactorDistanceThreshold (keep matches whose the descriptor
    distance is below dist_min * factor)
      - stdDistanceThreshold (keep matches whose the descriptor distance is
    below dist_min + standard_deviation)
      - ratioDistanceThreshold (keep matches enough discriminated: the ratio
    distance between the 2 best matches is below the threshold)
      - stdAndRatioDistanceThreshold (keep matches that agree with at least
    one of the two conditions)
      - noFilterMatching

    \param filterType : Type of the filtering method
  */
  inline void setFilterMatchingType(const vpFilterMatchingType &filterType)
  {
    m_filterType = filterType;

    // Use k-nearest neighbors (knn) to retrieve the two best matches for a
    // keypoint  So this is useful only for ratioDistanceThreshold method
    if (filterType == ratioDistanceThreshold || filterType == stdAndRatioDistanceThreshold) {
      m_useKnn = true;

#if (VISP_HAVE_OPENCV_VERSION >= 0x020400 && VISP_HAVE_OPENCV_VERSION < 0x030000)
      if (m_matcher != NULL && m_matcherName == "BruteForce") {
        // if a matcher is already initialized, disable the crossCheck
        // because it will not work with knnMatch
        m_matcher->set("crossCheck", false);
      }
#endif
    } else {
      m_useKnn = false;

#if (VISP_HAVE_OPENCV_VERSION >= 0x020400 && VISP_HAVE_OPENCV_VERSION < 0x030000)
      if (m_matcher != NULL && m_matcherName == "BruteForce") {
        // if a matcher is already initialized, set the crossCheck mode if
        // necessary
        m_matcher->set("crossCheck", m_useBruteForceCrossCheck);
      }
#endif
    }
  }

  /*!
    Set the factor value for the filtering method:
    constantFactorDistanceThreshold.

    \param factor : Factor value
  */
  inline void setMatchingFactorThreshold(const double factor)
  {
    if (factor > 0.0) {
      m_matchingFactorThreshold = factor;
    } else {
      throw vpException(vpException::badValue, "The factor must be positive.");
    }
  }

  /*!
    Set the ratio value for the filtering method: ratioDistanceThreshold.

    \param ratio : Ratio value (]0 ; 1])
  */
  inline void setMatchingRatioThreshold(const double ratio)
  {
    if (ratio > 0.0 && (ratio < 1.0 || std::fabs(ratio - 1.0) < std::numeric_limits<double>::epsilon())) {
      m_matchingRatioThreshold = ratio;
    } else {
      throw vpException(vpException::badValue, "The ratio must be in the interval ]0 ; 1].");
    }
  }

  /*!
    Set the percentage value for defining the cardinality of the consensus
    group.

    \param percentage : Percentage value (]0 ; 100])
  */
  inline void setRansacConsensusPercentage(const double percentage)
  {
    if (percentage > 0.0 &&
        (percentage < 100.0 || std::fabs(percentage - 100.0) < std::numeric_limits<double>::epsilon())) {
      m_ransacConsensusPercentage = percentage;
    } else {
      throw vpException(vpException::badValue, "The percentage must be in the interval ]0 ; 100].");
    }
  }

  /*!
    Set the maximum number of iterations for the Ransac pose estimation
    method.

    \param nbIter : Maximum number of iterations for the Ransac
  */
  inline void setRansacIteration(const int nbIter)
  {
    if (nbIter > 0) {
      m_nbRansacIterations = nbIter;
    } else {
      throw vpException(vpException::badValue, "The number of iterations must be greater than zero.");
    }
  }

  /*!
    Set the maximum reprojection error (in pixel) to determine if a point is
    an inlier or not.

    \param reprojectionError : Maximum reprojection error in pixel (used by
    OpenCV function)
  */
  inline void setRansacReprojectionError(const double reprojectionError)
  {
    if (reprojectionError > 0.0) {
      m_ransacReprojectionError = reprojectionError;
    } else {
      throw vpException(vpException::badValue, "The Ransac reprojection "
                                               "threshold must be positive "
                                               "as we deal with distance.");
    }
  }

  /*!
    Set the minimum number of inlier for the Ransac pose estimation method.

    \param minCount : Minimum number of inlier for the consensus
  */
  inline void setRansacMinInlierCount(const int minCount)
  {
    if (minCount > 0) {
      m_nbRansacMinInlierCount = minCount;
    } else {
      throw vpException(vpException::badValue, "The minimum number of inliers must be greater than zero.");
    }
  }

  /*!
    Set the maximum error (in meter) to determine if a point is an inlier or
    not.

    \param threshold : Maximum error in meter for ViSP function
  */
  inline void setRansacThreshold(const double threshold)
  {
    if (threshold > 0.0) {
      m_ransacThreshold = threshold;
    } else {
      throw vpException(vpException::badValue, "The Ransac threshold must be positive as we deal with distance.");
    }
  }

  /*!
    Set if multiple affine transformations must be used to detect and extract
    keypoints.

    \param useAffine : True to use multiple affine transformations, false
    otherwise
  */
  inline void setUseAffineDetection(const bool useAffine) { m_useAffineDetection = useAffine; }

#if (VISP_HAVE_OPENCV_VERSION >= 0x020400 && VISP_HAVE_OPENCV_VERSION < 0x030000)
  /*!
    Set if cross check method must be used to eliminate some false matches
    with a brute-force matching method.

    \param useCrossCheck : True to use cross check, false otherwise
  */
  inline void setUseBruteForceCrossCheck(const bool useCrossCheck)
  {
    // Only available with BruteForce and with k=1 (i.e not used with a
    // ratioDistanceThreshold method)
    if (m_matcher != NULL && !m_useKnn && m_matcherName == "BruteForce") {
      m_matcher->set("crossCheck", useCrossCheck);
    } else if (m_matcher != NULL && m_useKnn && m_matcherName == "BruteForce") {
      std::cout << "Warning, you try to set the crossCheck parameter with a "
                   "BruteForce matcher but knn is enabled";
      std::cout << " (the filtering method uses a ratio constraint)" << std::endl;
    }
  }
#endif

  /*!
    Set if we want to match the train keypoints to the query keypoints.

    \param useMatchTrainToQuery : True to match the train keypoints to the
    query keypoints
   */
  inline void setUseMatchTrainToQuery(const bool useMatchTrainToQuery)
  {
    m_useMatchTrainToQuery = useMatchTrainToQuery;
  }

  /*!
    Set the flag to choose between a percentage value of inliers for the
    cardinality of the consensus group or a minimum number.

    \param usePercentage : True to a percentage ratio of inliers, otherwise
    use a specified number of inliers
  */
  inline void setUseRansacConsensusPercentage(const bool usePercentage) { m_useConsensusPercentage = usePercentage; }

  /*!
    Set the flag to choose between the OpenCV or ViSP Ransac pose estimation
    function.

    \param ransacVVS : True to use ViSP function, otherwise use OpenCV
    function
  */
  inline void setUseRansacVVS(const bool ransacVVS) { m_useRansacVVS = ransacVVS; }

  /*!
    Set the flag to filter matches where multiple query keypoints are matched
    to the same train keypoints.

    \param singleMatchFilter : True to use the single match filter.
   */
  inline void setUseSingleMatchFilter(const bool singleMatchFilter) { m_useSingleMatchFilter = singleMatchFilter; }

private:
  //! If true, compute covariance matrix if the user select the pose
  //! estimation method using ViSP
  bool m_computeCovariance;
  //! Covariance matrix
  vpMatrix m_covarianceMatrix;
  //! Current id associated to the training image used for the learning.
  int m_currentImageId;
  //! Method (based on descriptor distances) to decide if the object is
  //! present or not.
  vpDetectionMethodType m_detectionMethod;
  //! Detection score to decide if the object is present or not.
  double m_detectionScore;
  //! Detection threshold based on average of descriptor distances to decide
  //! if the object is present or not.
  double m_detectionThreshold;
  //! Elapsed time to detect keypoints.
  double m_detectionTime;
  //! List of detector names.
  std::vector<std::string> m_detectorNames;
  //! Map of smart reference-counting pointers (similar to shared_ptr in
  //! Boost) detectors,
  // with a key based upon the detector name.
  std::map<std::string, cv::Ptr<cv::FeatureDetector> > m_detectors;
  //! Elapsed time to extract descriptors for the detected keypoints.
  double m_extractionTime;
  //! List of extractor name.
  std::vector<std::string> m_extractorNames;
  //! Map of smart reference-counting pointers (similar to shared_ptr in
  //! Boost) extractors,
  // with a key based upon the extractor name.
  std::map<std::string, cv::Ptr<cv::DescriptorExtractor> > m_extractors;
  //! List of filtered matches between the detected and the trained keypoints.
  std::vector<cv::DMatch> m_filteredMatches;
  //! Chosen method of filtering to eliminate false matching.
  vpFilterMatchingType m_filterType;
  //! Image format to use when saving the training images
  vpImageFormatType m_imageFormat;
  //! List of k-nearest neighbors for each detected keypoints (if the method
  //! chosen is based upon on knn).
  std::vector<std::vector<cv::DMatch> > m_knnMatches;
  //! Map descriptor enum type to string.
  std::map<vpFeatureDescriptorType, std::string> m_mapOfDescriptorNames;
  //! Map detector enum type to string.
  std::map<vpFeatureDetectorType, std::string> m_mapOfDetectorNames;
  //! Map of image id to know to which training image is related a training
  //! keypoints.
  std::map<int, int> m_mapOfImageId;
  //! Map of images to have access to the image buffer according to his image
  //! id.
  std::map<int, vpImage<unsigned char> > m_mapOfImages;
  //! Smart reference-counting pointer (similar to shared_ptr in Boost) of
  //! descriptor matcher (e.g. BruteForce or FlannBased).
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
  //! List of pairs between the keypoint and the 3D point after the Ransac.
  std::vector<std::pair<cv::KeyPoint, cv::Point3f> > m_matchRansacKeyPointsToPoints;
  //! Maximum number of iterations for the Ransac method.
  int m_nbRansacIterations;
  //! Minimum number of inliers for the Ransac method.
  int m_nbRansacMinInlierCount;
  //! List of 3D points (in the object frame) filtered after the matching to
  //! compute the pose.
  std::vector<cv::Point3f> m_objectFilteredPoints;
  //! Elapsed time to compute the pose.
  double m_poseTime;
  /*! Matrix of descriptors (each row contains the descriptors values for each
     keypoints detected in the current image). */
  cv::Mat m_queryDescriptors;
  //! List of detected keypoints filtered after the matching.
  std::vector<cv::KeyPoint> m_queryFilteredKeyPoints;
  //! List of keypoints detected in the current image.
  std::vector<cv::KeyPoint> m_queryKeyPoints;
  //! Percentage value to determine the number of inliers for the Ransac
  //! method.
  double m_ransacConsensusPercentage;
  //! List of inliers.
  std::vector<vpImagePoint> m_ransacInliers;
  //! List of outliers.
  std::vector<vpImagePoint> m_ransacOutliers;
  //! Maximum reprojection error (in pixel for the OpenCV method) to decide if
  //! a point is an inlier or not.
  double m_ransacReprojectionError;
  //! Maximum error (in meter for the ViSP method) to decide if a point is an
  //! inlier or not.
  double m_ransacThreshold;
  //! Matrix of descriptors (each row contains the descriptors values for each
  //! keypoints
  // detected in the train images).
  cv::Mat m_trainDescriptors;
  //! List of keypoints detected in the train images.
  std::vector<cv::KeyPoint> m_trainKeyPoints;
  //! List of 3D points (in the object frame) corresponding to the train
  //! keypoints.
  std::vector<cv::Point3f> m_trainPoints;
  //! List of 3D points in vpPoint format (in the object frame) corresponding
  //! to the train keypoints.
  std::vector<vpPoint> m_trainVpPoints;
  //! If true, use multiple affine transformations to cober the 6 affine
  //! parameters
  bool m_useAffineDetection;
#if (VISP_HAVE_OPENCV_VERSION >= 0x020400 && VISP_HAVE_OPENCV_VERSION < 0x030000)
  //! If true, some false matches will be eliminate by keeping only pairs
  //! (i,j) such that for i-th query descriptor the j-th descriptor in the
  //! matcher’s collection is the nearest and vice versa.
  bool m_useBruteForceCrossCheck;
#endif
  //! Flag set if a percentage value is used to determine the number of
  //! inliers for the Ransac method.
  bool m_useConsensusPercentage;
  //! Flag set if a knn matching method must be used.
  bool m_useKnn;
  //! Flag set if we want to match the train keypoints to the query keypoints,
  //! useful when there is only one train image because it reduces the number
  //! of possible false matches (by default it is the inverse because normally
  //! there are multiple train images of different views of the object)
  bool m_useMatchTrainToQuery;
  //! Flag set if a Ransac VVS pose estimation must be used.
  bool m_useRansacVVS;
  //! If true, keep only pairs of keypoints where each train keypoint is
  //! matched to a single query keypoint
  bool m_useSingleMatchFilter;

  void affineSkew(double tilt, double phi, cv::Mat &img, cv::Mat &mask, cv::Mat &Ai);

  double computePoseEstimationError(const std::vector<std::pair<cv::KeyPoint, cv::Point3f> > &matchKeyPoints,
                                    const vpCameraParameters &cam, const vpHomogeneousMatrix &cMo_est);

  void filterMatches();

  void init();
  void initDetector(const std::string &detectorNames);
  void initDetectors(const std::vector<std::string> &detectorNames);

  void initExtractor(const std::string &extractorName);
  void initExtractors(const std::vector<std::string> &extractorNames);

  void initFeatureNames();

  inline size_t myKeypointHash(const cv::KeyPoint &kp)
  {
    size_t _Val = 2166136261U, scale = 16777619U;
    Cv32suf u;
    u.f = kp.pt.x;
    _Val = (scale * _Val) ^ u.u;
    u.f = kp.pt.y;
    _Val = (scale * _Val) ^ u.u;
    u.f = kp.size;
    _Val = (scale * _Val) ^ u.u;
    // As the keypoint angle can be computed for certain type of keypoint only
    // when extracting  the corresponding descriptor, the angle field is not
    // taking into account for the hash
    //    u.f = kp.angle; _Val = (scale * _Val) ^ u.u;
    u.f = kp.response;
    _Val = (scale * _Val) ^ u.u;
    _Val = (scale * _Val) ^ ((size_t)kp.octave);
    _Val = (scale * _Val) ^ ((size_t)kp.class_id);
    return _Val;
  }

#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
  /*
   * Adapts a detector to detect points over multiple levels of a Gaussian
   * pyramid. Useful for detectors that are not inherently scaled.
   * From OpenCV 2.4.11 source code.
   */
  class PyramidAdaptedFeatureDetector : public cv::FeatureDetector
  {
  public:
    // maxLevel - The 0-based index of the last pyramid layer
    PyramidAdaptedFeatureDetector(const cv::Ptr<cv::FeatureDetector> &detector, int maxLevel = 2);

    // TODO implement read/write
    virtual bool empty() const;

  protected:
    virtual void detect(cv::InputArray image, CV_OUT std::vector<cv::KeyPoint> &keypoints,
                        cv::InputArray mask = cv::noArray());
    virtual void detectImpl(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints,
                            const cv::Mat &mask = cv::Mat()) const;

    cv::Ptr<cv::FeatureDetector> detector;
    int maxLevel;
  };

  /*
   * A class filters a vector of keypoints.
   * Because now it is difficult to provide a convenient interface for all
   * usage scenarios of the keypoints filter class, it has only several needed
   * by now static methods.
   */
  class KeyPointsFilter
  {
  public:
    KeyPointsFilter() {}

    /*
     * Remove keypoints within borderPixels of an image edge.
     */
    static void runByImageBorder(std::vector<cv::KeyPoint> &keypoints, cv::Size imageSize, int borderSize);
    /*
     * Remove keypoints of sizes out of range.
     */
    static void runByKeypointSize(std::vector<cv::KeyPoint> &keypoints, float minSize, float maxSize = FLT_MAX);
    /*
     * Remove keypoints from some image by mask for pixels of this image.
     */
    static void runByPixelsMask(std::vector<cv::KeyPoint> &keypoints, const cv::Mat &mask);
    /*
     * Remove duplicated keypoints.
     */
    static void removeDuplicated(std::vector<cv::KeyPoint> &keypoints);

    /*
     * Retain the specified number of the best keypoints (according to the
     * response)
     */
    static void retainBest(std::vector<cv::KeyPoint> &keypoints, int npoints);
  };

#endif
};

#endif
#endif
