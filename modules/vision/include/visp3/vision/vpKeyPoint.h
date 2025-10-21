/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Key point functionalities.
 */
#ifndef VP_KEYPOINT_H
#define VP_KEYPOINT_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_OPENCV) && \
    (((VISP_HAVE_OPENCV_VERSION < 0x050000) && defined(HAVE_OPENCV_CALIB3D) && defined(HAVE_OPENCV_FEATURES2D)) || \
     ((VISP_HAVE_OPENCV_VERSION >= 0x050000) && defined(HAVE_OPENCV_3D) && defined(HAVE_OPENCV_FEATURES)))

#include <algorithm> // std::transform
#include <float.h>   // DBL_MAX
#include <fstream>   // std::ofstream
#include <limits>
#include <map>      // std::map
#include <numeric>  // std::accumulate
#include <stdlib.h> // srand, rand
#include <time.h>   // time
#include <vector>   // std::vector

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

#include <opencv2/core/core.hpp>

#if defined(HAVE_OPENCV_FEATURES2D)
#include <opencv2/features2d/features2d.hpp>
#endif

#if defined(HAVE_OPENCV_XFEATURES2D)
#include <opencv2/xfeatures2d.hpp>
#endif

#if defined(HAVE_OPENCV_IMGPROC)
#include <opencv2/imgproc/imgproc.hpp>
#endif

#if defined(HAVE_OPENCV_NONFREE)
#include <opencv2/nonfree/nonfree.hpp>
#endif

BEGIN_VISP_NAMESPACE
/*!
 * \class vpKeyPoint
 * \ingroup group_vision_keypoints group_detection_keypoint group_detection_mbt_object
 *
 * \brief Class that allows keypoints 2D features detection (and descriptors extraction)
 * and matching thanks to OpenCV library. Thus to enable this class OpenCV should
 * be installed. Installation instructions are provided here
 * https://visp.inria.fr/3rd_opencv.
 *
 * This class permits to use different types of detectors, extractors and
 * matchers easily. So, the classical SIFT and SURF keypoints could be used, as
 * well as ORB, FAST, (etc.) keypoints, depending of the version of OpenCV you
 * use.
 *
 * \note Due to some patents, SIFT and SURF are packaged in an external module
 * called nonfree module in OpenCV version before 3.0.0 and in xfeatures2d
 * from 3.0.0. You have to check you have the corresponding module to use SIFT
 * and SURF.
 *
 * Depending on OpenCV version, the table below shows which OpenCV module
 * is required to be able to use a given 2D features detectors.
 *
 * 2D features detectors | OpenCV < 5.0 | OpenCV >= 5.0
 * :-------------------: | :----------: | :-----------:
 * AGAST                 | features2d   | xfeatures2d
 * AKAZE                 | features2d   | xfeatures2d
 * BRISK                 | features2d   | xfeatures2d
 * GFTTDetector          | features2d   | features
 * FAST                  | features2d   | features
 * KAZE                  | features2d   | xfeatures2d
 * MSDDetector           | xfeatures2d  | xfeatures2d
 * MSER                  | features2d   | features
 * ORB                   | features2d   | features
 * SIFT                  | xfeatures2d  | features
 * SimpleBlobDetector    | features2d   | features
 * STAR                  | xfeatures2d  | xfeatures2d
 * SURF                  | xfeatures2d  | xfeatures2d
 *
 * Depending on OpenCV version, the table below shows which OpenCV module
 * is required to be able to use a given 2D features descriptor.
 *
 * 2D features descriptors | OpenCV < 5.0 | OpenCV >= 5.0
 * :---------------------: | :----------: | :-----------:
 * AKAZE                   | features2d   | xfeatures2d
 * BRIEF                   | xfeatures2d  | xfeatures2d
 * BRISK                   | features2d   | xfeatures2d
 * BoostDesc               | xfeatures2d  | xfeatures2d
 * DAISY                   | xfeatures2d  | xfeatures2d
 * FREAK                   | xfeatures2d  | xfeatures2d
 * KAZE                    | features2d   | xfeatures2d
 * LATCH                   | xfeatures2d  | xfeatures2d
 * ORB                     | features2d   | features
 * SIFT                    | xfeatures2d  | features
 * SURF                    | xfeatures2d  | xfeatures2d
 * VGG                     | xfeatures2d  | xfeatures2d
 *
 * The goal of this class is to provide a tool to match reference keypoints
 * from a reference image (or train keypoints in OpenCV terminology) and detected
 * keypoints from a current image (or query keypoints in OpenCV terminology).
 *
 * If you supply the corresponding 3D coordinates corresponding to the 2D
 * coordinates of the reference keypoints, you can also estimate the pose of the
 * object by matching a set of detected keypoints in the current image with the
 * reference keypoints.
 *
 * If you use this class, the first thing you have to do is to build
 * the reference keypoints by detecting keypoints in a reference image which
 * contains the object to detect. Then you match keypoints detected in a current
 * image with those detected in a reference image by calling matchPoint()
 * methods. You can access to the lists of matched points thanks to the methods
 * getMatchedPointsInReferenceImage() and getMatchedPointsInCurrentImage(). These
 * two methods return a list of matched points. The nth element of the first list
 * is matched with the nth element of the second list. To provide easy
 * compatibility with OpenCV terminology, getTrainKeyPoints() give you access to
 * the list of keypoints detected in train images (or reference images) and
 * getQueryKeyPoints() give you access to the list of keypoints detected in a
 * query image (or current image). The method getMatches() give you access to a
 * list of cv::DMatch with the correspondence between the index of the train
 * keypoints and the index of the query keypoints.
 *
 * The following small example shows how to use the class to do the matching
 * between current and reference keypoints.
 *
 * \code
 * #include <visp3/core/vpImage.h>
 * #include <visp3/vision/vpKeyPoint.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 * #if (VISP_HAVE_OPENCV_VERSION >= 0x020300)
 *   vpImage<unsigned char> Irefrence;
 *   vpImage<unsigned char> Icurrent;
 *
 *   vpKeyPoint::vpFilterMatchingType filterType = vpKeyPoint::ratioDistanceThreshold;
 *   vpKeyPoint keypoint("ORB", "ORB", "BruteForce-Hamming", filterType);
 *
 *   // First grab the reference image Irefrence
 *   // Add your code to load the reference image in Ireference
 *
 *   // Build the reference ORB points.
 *   keypoint.buildReference(Irefrence);
 *
 *   // Then grab another image which represents the current image Icurrent
 *
 *   // Match points between the reference points and the ORB points computed in the current image.
 *   keypoint.matchPoint(Icurrent);
 *
 *   // Display the matched points
 *   keypoint.display(Irefrence, Icurrent);
 * #endif
 *
 *   return (0);
 * }
 * \endcode
 *
 * It is also possible to build the reference keypoints in a region of interest
 * (ROI) of an image and find keypoints to match in only a part of the current
 * image. The small following example shows how to do this:
 *
 * \code
 * #include <visp3/core/vpDisplay.h>
 * #include <visp3/core/vpImage.h>
 * #include <visp3/vision/vpKeyPoint.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 * #if (VISP_HAVE_OPENCV_VERSION >= 0x020300)
 *   vpImage<unsigned char> Ireference;
 *   vpImage<unsigned char> Icurrent;
 *
 *   vpKeyPoint::vpFilterMatchingType filterType = vpKeyPoint::ratioDistanceThreshold;
 *   vpKeyPoint keypoint("ORB", "ORB", "BruteForce-Hamming", filterType);
 *
 *   //First grab the reference image Irefrence
 *   //Add your code to load the reference image in Ireference
 *
 *   //Select a part of the image by clicking on two points which define a rectangle
 *   vpImagePoint corners[2];
 *   for (int i=0 ; i < 2 ; i++) {
 *     vpDisplay::getClick(Ireference, corners[i]);
 *   }
 *
 *   //Build the reference ORB points.
 *   int nbrRef;
 *   unsigned int height, width;
 *   height = static_cast<unsigned int>(corners[1].get_i() - corners[0].get_i());
 *   width = static_cast<unsigned int>(corners[1].get_j() - corners[0].get_j());
 *   nbrRef = keypoint.buildReference(Ireference, corners[0], height, width);
 *
 *   //Then grab another image which represents the current image Icurrent
 *
 *   //Select a part of the image by clicking on two points which define a rectangle
 *   for (int i=0 ; i < 2 ; i++) {
 *     vpDisplay::getClick(Icurrent, corners[i]);
 *   }
 *
 *   //Match points between the reference points and the ORB points computed in the current image.
 *   int nbrMatched;
 *   height = static_cast<unsigned int>(corners[1].get_i() - corners[0].get_i());
 *   width = static_cast<unsigned int>(corners[1].get_j() - corners[0].get_j());
 *   nbrMatched = keypoint.matchPoint(Icurrent, corners[0], height, width);
 *
 *   //Display the matched points
 *   keypoint.display(Ireference, Icurrent);
 * #endif
 *
 *   return(0);
 * }
 * \endcode
 *
 * <h2 id="header-details" class="groupheader">Tutorials & Examples</h2>
 *
 * <b>Tutorials</b><br>
 * <span style="margin-left:2em"> If you are interested in keypoints detection and matching, you may have a look at:</span><br>
 *
 * - \ref tutorial-matching
 * - \ref tutorial-homography-ransac
*/
class VISP_EXPORT vpKeyPoint : public vpBasicKeyPoint
{
public:
  /*! Predefined filtering method identifier. */
  enum vpFilterMatchingType
  {
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
  enum vpDetectionMethodType
  {
    detectionThreshold, /*!< The object is present if the average of the
                           descriptor distances is below the threshold. */
    detectionScore      /*!< Same condition than the previous but with a formula
                           taking into account the number of matches, the object is
                           present if the score is above the threshold. */
  };

  /*! Predefined constant for training image format. */
  typedef enum
  {
    jpgImageFormat, /*!< Save training images in JPG format. */
    pngImageFormat, /*!< Save training images in PNG format. */
    ppmImageFormat, /*!< Save training images in PPM format. */
    pgmImageFormat  /*!< Save training images in PGM format. */
  } vpImageFormatType;

  /*! Predefined constant for feature detection type. */
  enum vpFeatureDetectorType
  {
#if (VISP_HAVE_OPENCV_VERSION >= 0x050000)
#  if defined(HAVE_OPENCV_FEATURES)
    DETECTOR_FAST,       //!< FAST detector
    DETECTOR_GFTT,       //!< GFTT detector
    DETECTOR_MSER,       //!< MSER detector
    DETECTOR_ORB,        //!< ORB detector
    DETECTOR_SIFT,       //!< SIFT detector
    DETECTOR_SimpleBlob, //!< SimpleBlob detector
#  endif
#  if defined(HAVE_OPENCV_XFEATURES2D)
    DETECTOR_AGAST,      //!< AGAST detector
    DETECTOR_AKAZE,      //!< AKAZE detector
    DETECTOR_BRISK,      //!< BRISK detector
    DETECTOR_KAZE,       //!< KAZE detector
    DETECTOR_MSD,        //!< MSD detector
    DETECTOR_STAR,       //!< STAR detector
#  endif
#  if defined(OPENCV_ENABLE_NONFREE) && defined(HAVE_OPENCV_XFEATURES2D)
    DETECTOR_SURF,       //!< SURF detector
#  endif
#else // OpenCV < 5.0.0
#  if defined(HAVE_OPENCV_FEATURES2D)
    DETECTOR_BRISK,      //!< BRISK detector
    DETECTOR_FAST,       //!< FAST detector
    DETECTOR_GFTT,       //!< GFTT detector
    DETECTOR_MSER,       //!< MSER detector
    DETECTOR_ORB,        //!< ORB detector
    DETECTOR_SimpleBlob, //!< SimpleBlob detector
#    if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
    DETECTOR_AGAST,      //!< AGAST detector
    DETECTOR_AKAZE,      //!< AKAZE detector
    DETECTOR_KAZE,       //!< KAZE detector
#    endif
#  endif
#  if (VISP_HAVE_OPENCV_VERSION >= 0x030100) && defined(VISP_HAVE_OPENCV_XFEATURES2D)
    DETECTOR_MSD,        //!< MSD detector
#  endif
#  if ((VISP_HAVE_OPENCV_VERSION >= 0x030411 && CV_MAJOR_VERSION < 4) || (VISP_HAVE_OPENCV_VERSION >= 0x040400)) && defined(HAVE_OPENCV_FEATURES2D)
    DETECTOR_SIFT,       //!< SIFT detector
#  endif
#if (VISP_HAVE_OPENCV_VERSION < 0x030000) || (defined(VISP_HAVE_OPENCV_XFEATURES2D))
    DETECTOR_STAR,       //!< STAR detector
#  endif
#  if defined(OPENCV_ENABLE_NONFREE) && defined(HAVE_OPENCV_XFEATURES2D)
    DETECTOR_SURF,       //!< SURF detector
#  endif
#endif

    DETECTOR_TYPE_SIZE   //!< Number of detectors available
  };

  /*! Predefined constant for descriptor extraction type. */
  enum vpFeatureDescriptorType
  {
#if (VISP_HAVE_OPENCV_VERSION >= 0x050000)
#  if defined(HAVE_OPENCV_FEATURES)
    DESCRIPTOR_ORB,       //!< ORB descriptor
    DESCRIPTOR_SIFT,      //!< SIFT descriptor
#  endif
#  if defined(HAVE_OPENCV_XFEATURES2D)
    DESCRIPTOR_AKAZE,     //!< AKAZE descriptor
    DESCRIPTOR_BRISK,     //!< BRISK descriptor
    DESCRIPTOR_BoostDesc, //!< BoostDesc descriptor
    DESCRIPTOR_BRIEF,     //!< BRIEF descriptor
    DESCRIPTOR_DAISY,     //!< DAISY descriptor
    DESCRIPTOR_FREAK,     //!< FREAK descriptor
    DESCRIPTOR_KAZE,      //!< KAZE descriptor
    DESCRIPTOR_LATCH,     //!< LATCH descriptor
    DESCRIPTOR_VGG,       //!< VGG descriptor
#  endif
#  if defined(OPENCV_ENABLE_NONFREE) && defined(HAVE_OPENCV_XFEATURES2D)
    DESCRIPTOR_SURF,      //!< SURF descriptor
#  endif
#else // opencv < 5.0.0
#  if defined(HAVE_OPENCV_FEATURES2D)
    DESCRIPTOR_BRISK,     //!< BRISK descriptor
    DESCRIPTOR_ORB,       //!< ORB descriptor
#    if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
    DESCRIPTOR_AKAZE,     //!< AKAZE descriptor
    DESCRIPTOR_KAZE,      //!< KAZE descriptor
#    endif
#  endif
#  if defined(HAVE_OPENCV_XFEATURES2D)
    DESCRIPTOR_BRIEF,     //!< BRIEF descriptor
    DESCRIPTOR_DAISY,     //!< DAISY descriptor
    DESCRIPTOR_FREAK,     //!< FREAK descriptor
    DESCRIPTOR_LATCH,     //!< LATCH descriptor
#  endif
#  if ((VISP_HAVE_OPENCV_VERSION >= 0x030411 && CV_MAJOR_VERSION < 4) || (VISP_HAVE_OPENCV_VERSION >= 0x040400)) && defined(HAVE_OPENCV_FEATURES2D)
    DESCRIPTOR_SIFT,      //!< SIFT descriptor
#  endif
#  if defined(OPENCV_ENABLE_NONFREE) && defined(HAVE_OPENCV_XFEATURES2D)
    DESCRIPTOR_SURF,      //!< SURF descriptor
#  endif
#if (VISP_HAVE_OPENCV_VERSION >= 0x030200) && defined(VISP_HAVE_OPENCV_XFEATURES2D)
    DESCRIPTOR_BoostDesc, //!< BoostDesc descriptor, only with OpenCV >= 3.2.0
    DESCRIPTOR_VGG,       //!< VGG descriptor, only with OpenCV >= 3.2.0
#  endif
#endif

    DESCRIPTOR_TYPE_SIZE  //!< Number of descriptors available
  };

  /*!
   * Constructor to initialize the specified detector, descriptor, matcher and
   * filtering method.
   *
   * \param detectorType : Type of feature detector.
   * \param descriptorType : Type of the descriptor extractor.
   * \param matcherName : Name of the matcher.
   * \param filterType : Filtering matching method chosen.
   */
  vpKeyPoint(const vpFeatureDetectorType &detectorType, const vpFeatureDescriptorType &descriptorType,
             const std::string &matcherName, const vpFilterMatchingType &filterType = ratioDistanceThreshold);

  /*!
   * Constructor to initialize the specified detector, descriptor, matcher and
   * filtering method.
   *
   * \param detectorName : Name of the detector.
   * \param extractorName : Name of the extractor.
   * \param matcherName : Name of the matcher.
   * \param filterType : Filtering matching method chosen.
   */
  vpKeyPoint(const std::string &detectorName = "ORB", const std::string &extractorName = "ORB",
             const std::string &matcherName = "BruteForce-Hamming",
             const vpFilterMatchingType &filterType = ratioDistanceThreshold);

  /*!
   * Constructor to initialize specified detector, extractor, matcher and
   * filtering method.
   *
   * \param detectorNames : List of name detector for allowing multiple detectors.
   * \param extractorNames : List of name extractor for allowing multiple extractors.
   * \param matcherName : Name of the matcher.
   * \param filterType : Filtering matching method chosen.
   */
  vpKeyPoint(const std::vector<std::string> &detectorNames, const std::vector<std::string> &extractorNames,
             const std::string &matcherName = "BruteForce",
             const vpFilterMatchingType &filterType = ratioDistanceThreshold);

  /*!
   * Build the reference keypoints list.
   *
   * \param I : Input reference image.
   * \return The number of detected keypoints in the image \p I.
   */
  unsigned int buildReference(const vpImage<unsigned char> &I) VP_OVERRIDE;

  /*!
   * Build the reference keypoints list in a region of interest in the image.
   *
   * \param I : Input reference image.
   * \param iP : Position of the top-left corner of the region of interest.
   * \param height : Height of the region of interest.
   * \param width : Width of the region of interest.
   * \return The number of detected keypoints in the current image I.
   */
  unsigned int buildReference(const vpImage<unsigned char> &I, const vpImagePoint &iP, unsigned int height,
                              unsigned int width) VP_OVERRIDE;

  /*!
   * Build the reference keypoints list in a region of interest in the image.
   *
   * \param I : Input image.
   * \param rectangle : Rectangle of the region of interest.
   * \return The number of detected keypoints in the current image I.
   */
  unsigned int buildReference(const vpImage<unsigned char> &I, const vpRect &rectangle) VP_OVERRIDE;

  /*!
   * Build the reference keypoints list and compute the 3D position
   * corresponding of the keypoints locations.
   *
   * \param I : Input image.
   * \param trainKeyPoints : List of the train keypoints.
   * \param points3f : Output list of the 3D position corresponding of the keypoints locations.
   * \param append : If true, append the supply train keypoints with those already present.
   * \param class_id : The class id to be set to the input cv::KeyPoint if != -1.
   * \return The number of detected keypoints in the current image I.
   */
  unsigned int buildReference(const vpImage<unsigned char> &I, std::vector<cv::KeyPoint> &trainKeyPoints,
                              std::vector<cv::Point3f> &points3f, bool append = false, int class_id = -1);

  /*!
   * Build the reference keypoints list and compute the 3D position
   * corresponding of the keypoints locations.
   *
   * \param I : Input image.
   * \param trainKeyPoints : List of the train keypoints.
   * \param points3f : List of the 3D position corresponding of the keypoints locations.
   * \param trainDescriptors : List of the train descriptors.
   * \param append : If true, append the supply train keypoints with those already present.
   * \param class_id : The class id to be set to the input cv::KeyPoint if != -1.
   *
   * \return The number of keypoints in the current image I.
   */
  unsigned int buildReference(const vpImage<unsigned char> &I, const std::vector<cv::KeyPoint> &trainKeyPoints,
                              const cv::Mat &trainDescriptors, const std::vector<cv::Point3f> &points3f,
                              bool append = false, int class_id = -1);

  /*!
   * Build the reference keypoints list.
   *
   * \param I_color : Input reference image.
   * \return The number of detected keypoints in the image \p I.
   */
  unsigned int buildReference(const vpImage<vpRGBa> &I_color);

  /*!
   * Build the reference keypoints list in a region of interest in the image.
   *
   * \param I_color : Input reference image.
   * \param iP : Position of the top-left corner of the region of interest.
   * \param height : Height of the region of interest.
   * \param width : Width of the region of interest.
   * \return The number of detected keypoints in the current image I.
   */
  unsigned int buildReference(const vpImage<vpRGBa> &I_color, const vpImagePoint &iP, unsigned int height,
                              unsigned int width);

  /*!
   * Build the reference keypoints list in a region of interest in the image.
   *
   * \param I_color : Input image.
   * \param rectangle : Rectangle of the region of interest.
   * \return The number of detected keypoints in the current image I.
   */
  unsigned int buildReference(const vpImage<vpRGBa> &I_color, const vpRect &rectangle);

  /*!
   * Build the reference keypoints list and compute the 3D position
   * corresponding of the keypoints locations.
   *
   * \param I_color : Input image.
   * \param trainKeyPoints : List of the train keypoints.
   * \param points3f : Output list of the 3D position corresponding of the keypoints locations.
   * \param append : If true, append the supply train keypoints with those already present.
   * \param class_id : The class id to be set to the input cv::KeyPoint if != -1.
   * \return The number of detected keypoints in the current image I.
   */
  unsigned int buildReference(const vpImage<vpRGBa> &I_color, std::vector<cv::KeyPoint> &trainKeyPoints,
                              std::vector<cv::Point3f> &points3f, bool append = false, int class_id = -1);

  /*!
   * Build the reference keypoints list and compute the 3D position
   * corresponding of the keypoints locations.
   *
   * \param I_color : Input image.
   * \param trainKeyPoints : List of the train keypoints.
   * \param points3f : List of the 3D position corresponding of the keypoints locations.
   * \param trainDescriptors : List of the train descriptors.
   * \param append : If true, append the supply train keypoints with those already present.
   * \param class_id : The class id to be set to the input cv::KeyPoint if != -1.
   * \return The number of detected keypoints in the current image I.
   */
  unsigned int buildReference(const vpImage<vpRGBa> &I_color, const std::vector<cv::KeyPoint> &trainKeyPoints,
                              const cv::Mat &trainDescriptors, const std::vector<cv::Point3f> &points3f,
                              bool append = false, int class_id = -1);

  /*!
   * Compute the 3D coordinate in the world/object frame given the 2D image
   * coordinate and under the assumption that the point is located on a plane
   * whose the plane equation is known in the camera frame.
   * The Z-coordinate is retrieved according to the proportional relationship
   * between the plane equation expressed in the normalized camera frame
   * (derived from the image coordinate) and the same plane equation expressed
   * in the camera frame.
   *
   * \param candidate : Keypoint we want to compute the 3D coordinate.
   * \param roi : List of 3D points in the camera frame representing a planar face.
   * \param cam : Camera parameters.
   * \param cMo : Homogeneous matrix between the world and the camera frames.
   * \param point : 3D coordinate in the world/object frame computed.
   */
  static void compute3D(const cv::KeyPoint &candidate, const std::vector<vpPoint> &roi, const vpCameraParameters &cam,
                        const vpHomogeneousMatrix &cMo, cv::Point3f &point);

  /*!
   * Compute the 3D coordinate in the world/object frame given the 2D image
   * coordinate and under the assumption that the point is located on a plane
   * whose the plane equation is known in the camera frame.
   * The Z-coordinate is retrieved according to the proportional relationship
   * between the plane equation expressed in the normalized camera frame
   * (derived from the image coordinate) and the same plane equation expressed
   * in the camera frame.
   *
   * \param candidate : vpImagePoint we want to compute the 3D coordinate.
   * \param roi : List of 3D points in the camera frame representing a planar face.
   * \param cam : Camera parameters.
   * \param cMo : Homogeneous matrix between the world and the camera frames.
   * \param point : 3D coordinate in the world/object frame computed.
   */
  static void compute3D(const vpImagePoint &candidate, const std::vector<vpPoint> &roi, const vpCameraParameters &cam,
                        const vpHomogeneousMatrix &cMo, vpPoint &point);

  /*!
   * Keep only keypoints located on faces and compute for those keypoints the 3D
   * coordinate in the world/object frame given the 2D image coordinate and
   * under the assumption that the point is located on a plane.
   *
   * \param cMo : Homogeneous matrix between the world and the camera frames.
   * \param cam : Camera parameters.
   * \param candidates : In input, list of keypoints detected in the whole
   * image, in output, list of keypoints only located on planes.
   * \param polygons : List of 2D polygons representing the projection of the faces in
   * the image plane.
   * \param roisPt : List of faces, with the 3D coordinates known in the camera frame.
   * \param points : Output list of computed 3D coordinates (in
   * the world/object frame) of keypoints located only on faces.
   * \param descriptors : Optional parameter, pointer to the descriptors to filter.
   */
  static void compute3DForPointsInPolygons(const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                                           std::vector<cv::KeyPoint> &candidates,
                                           const std::vector<vpPolygon> &polygons,
                                           const std::vector<std::vector<vpPoint> > &roisPt,
                                           std::vector<cv::Point3f> &points, cv::Mat *descriptors = nullptr);

  /*!
   * Keep only keypoints located on faces and compute for those keypoints the 3D
   * coordinate in the world/object frame given the 2D image coordinate and
   * under the assumption that the point is located on a plane.
   *
   * \param cMo : Homogeneous matrix between the world and the camera frames.
   * \param cam : Camera parameters.
   * \param candidates : In input, list of vpImagePoint located in the whole
   * image, in output, list of vpImagePoint only located on planes.
   * \param polygons : List of 2D polygons representing the projection of the faces in
   * the image plane.
   * \param roisPt : List of faces, with the 3D coordinates known in the camera frame.
   * \param points : Output list of computed 3D coordinates (in the world/object frame)
   * of vpImagePoint located only on faces.
   * \param descriptors : Optional parameter, pointer to the descriptors to filter.
   */
  static void compute3DForPointsInPolygons(const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                                           std::vector<vpImagePoint> &candidates,
                                           const std::vector<vpPolygon> &polygons,
                                           const std::vector<std::vector<vpPoint> > &roisPt,
                                           std::vector<vpPoint> &points, cv::Mat *descriptors = nullptr);

  /*!
   * Keep only keypoints located on cylinders and compute the 3D coordinates in
   * the world/object frame given the 2D image coordinates.
   *
   * \param cMo : Homogeneous matrix between the world and the camera frames.
   * \param cam : Camera parameters.
   * \param candidates : In input, list of keypoints detected in the whole
   * image, in output, list of keypoints only located on cylinders.
   * \param cylinders : List of vpCylinder corresponding of the cylinder objects in the
   * scene, projected in the camera frame.
   * \param vectorOfCylinderRois : For each cylinder, the corresponding list of bounding box.
   * \param points : Output list of computed 3D coordinates in the world/object frame for each
   * keypoint located on a cylinder.
   * \param descriptors : Optional parameter, pointer to the descriptors to filter.
   */
  static void
    compute3DForPointsOnCylinders(const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                                  std::vector<cv::KeyPoint> &candidates, const std::vector<vpCylinder> &cylinders,
                                  const std::vector<std::vector<std::vector<vpImagePoint> > > &vectorOfCylinderRois,
                                  std::vector<cv::Point3f> &points, cv::Mat *descriptors = nullptr);

  /*!
   * Keep only vpImagePoint located on cylinders and compute the 3D coordinates
   * in the world/object frame given the 2D image coordinates.
   *
   * \param cMo : Homogeneous matrix between the world and the camera frames.
   * \param cam : Camera parameters.
   * \param candidates : In input, list of vpImagePoint located in the image, in
   * output, list of vpImagePoint only located on cylinders.
   * \param cylinders : List of vpCylinder corresponding of the cylinder objects in the scene,
   * projected in the camera frame.
   * \param vectorOfCylinderRois : For each cylinder, the corresponding list of bounding box.
   * \param points : Output list of computed 3D coordinates in the world/object frame for each
   * vpImagePoint located on a cylinder.
   * \param descriptors : Optional parameter, pointer to the descriptors to filter.
   */
  static void
    compute3DForPointsOnCylinders(const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                                  std::vector<vpImagePoint> &candidates, const std::vector<vpCylinder> &cylinders,
                                  const std::vector<std::vector<std::vector<vpImagePoint> > > &vectorOfCylinderRois,
                                  std::vector<vpPoint> &points, cv::Mat *descriptors = nullptr);

  /*!
   * Compute the pose using the correspondence between 2D points and 3D points
   * using OpenCV function with RANSAC method.
   *
   * \param imagePoints : List of 2D points corresponding to the location of the detected keypoints.
   * \param  objectPoints : List of the 3D points in the object frame matched.
   * \param cam : Camera parameters.
   * \param cMo : Homogeneous matrix between the object frame and the camera frame.
   * \param inlierIndex : List of indexes of inliers.
   * \param elapsedTime : Elapsed time.
   * \param func : Function pointer to filter the final pose returned by OpenCV pose estimation method.
   * \return True if the pose has been computed, false otherwise (not enough points, or size list mismatch).
   */
  bool computePose(const std::vector<cv::Point2f> &imagePoints, const std::vector<cv::Point3f> &objectPoints,
                   const vpCameraParameters &cam, vpHomogeneousMatrix &cMo, std::vector<int> &inlierIndex,
                   double &elapsedTime, bool (*func)(const vpHomogeneousMatrix &) = nullptr);

  /*!
   * Compute the pose using the correspondence between 2D points and 3D points
   * using ViSP function with RANSAC method.
   *
   * \param objectVpPoints : List of vpPoint with coordinates expressed in the object and in the camera frame.
   * \param cMo : Homogeneous matrix between the object frame and the camera frame.
   * \param inliers : List of inliers.
   * \param elapsedTime : Elapsed time.
   * \param func : Function pointer to filter the pose in Ransac pose estimation, if we want
   * to eliminate the poses which do not respect some criterion
   * \return True if the pose has been computed, false otherwise (not enough points, or size list mismatch).
   */
  bool computePose(const std::vector<vpPoint> &objectVpPoints, vpHomogeneousMatrix &cMo, std::vector<vpPoint> &inliers,
                   double &elapsedTime, bool (*func)(const vpHomogeneousMatrix &) = nullptr);

  /*!
   * Compute the pose using the correspondence between 2D points and 3D points
   * using ViSP function with RANSAC method.
   *
   * \param objectVpPoints : List of vpPoint with coordinates expressed in the object and in the camera frame.
   * \param cMo : Homogeneous matrix between the object frame and the camera frame.
   * \param inliers : List of inlier points.
   * \param inlierIndex : List of inlier index.
   * \param elapsedTime : Elapsed time.
   * \return True if the pose has been computed, false otherwise (not enough points, or size list mismatch).
   * \param func : Function pointer to filter  the pose in Ransac pose estimation, if we want to eliminate the poses which
   * do not respect some criterion
   */
  bool computePose(const std::vector<vpPoint> &objectVpPoints, vpHomogeneousMatrix &cMo, std::vector<vpPoint> &inliers,
                   std::vector<unsigned int> &inlierIndex, double &elapsedTime,
                   bool (*func)(const vpHomogeneousMatrix &) = nullptr);

  /*!
   * Initialize the size of the matching image (case with a matching side by
   * side between IRef and ICurrent).
   *
   * \param IRef : Reference image.
   * \param ICurrent : Current image.
   * \param IMatching : Image matching.
   */
  void createImageMatching(vpImage<unsigned char> &IRef, vpImage<unsigned char> &ICurrent,
                           vpImage<unsigned char> &IMatching);

  /*!
   * Initialize the size of the matching image with appropriate size according
   * to the number of training images. Used to display the matching of keypoints
   * detected in the current image with those detected in multiple training
   * images.
   *
   * \param ICurrent : Current image.
   * \param IMatching : Image initialized with appropriate size.
   */
  void createImageMatching(vpImage<unsigned char> &ICurrent, vpImage<unsigned char> &IMatching);

  /*!
   * Initialize the size of the matching image (case with a matching side by
   * side between IRef and ICurrent).
   *
   * \param IRef : Reference image.
   * \param ICurrent : Current image.
   * \param IMatching : Image matching.
   */
  void createImageMatching(vpImage<unsigned char> &IRef, vpImage<vpRGBa> &ICurrent, vpImage<vpRGBa> &IMatching);

  /*!
   * Initialize the size of the matching image with appropriate size according
   * to the number of training images. Used to display the matching of keypoints
   * detected in the current image with those detected in multiple training
   * images.
   *
   * \param ICurrent : Current image.
   * \param IMatching : Image initialized with appropriate size.
   */
  void createImageMatching(vpImage<vpRGBa> &ICurrent, vpImage<vpRGBa> &IMatching);

  /*!
   * Detect keypoints in the image.
   *
   * \param I : Input image.
   * \param keyPoints : Output list of the detected keypoints.
   * \param rectangle : Optional rectangle of the region of interest.
   */
  void detect(const vpImage<unsigned char> &I, std::vector<cv::KeyPoint> &keyPoints,
              const vpRect &rectangle = vpRect());

  /*!
   * Detect keypoints in the image.
   *
   * \param I_color : Input image.
   * \param keyPoints : Output list of the detected keypoints.
   * \param rectangle : Optional rectangle of the region of interest.
   */
  void detect(const vpImage<vpRGBa> &I_color, std::vector<cv::KeyPoint> &keyPoints, const vpRect &rectangle = vpRect());

  /*!
   * Detect keypoints in the image.
   *
   * \param matImg : Input image.
   * \param keyPoints : Output list of the detected keypoints.
   * \param mask : Optional 8-bit integer mask to detect only where mask[i][j] != 0.
   */
  void detect(const cv::Mat &matImg, std::vector<cv::KeyPoint> &keyPoints, const cv::Mat &mask = cv::Mat());

  /*!
   * Detect keypoints in the image.
   *
   * \param I : Input image.
   * \param keyPoints : Output list of the detected keypoints.
   * \param elapsedTime : Elapsed time.
   * \param rectangle : Optional rectangle of the region of interest.
   */
  void detect(const vpImage<unsigned char> &I, std::vector<cv::KeyPoint> &keyPoints, double &elapsedTime,
              const vpRect &rectangle = vpRect());

  /*!
   * Detect keypoints in the image.
   *
   * \param I_color : Input image.
   * \param keyPoints : Output list of the detected keypoints.
   * \param elapsedTime : Elapsed time.
   * \param rectangle : Optional rectangle of the region of interest.
   */
  void detect(const vpImage<vpRGBa> &I_color, std::vector<cv::KeyPoint> &keyPoints, double &elapsedTime,
              const vpRect &rectangle = vpRect());

  /*!
   * Detect keypoints in the image.
   *
   * \param matImg : Input image.
   * \param keyPoints : Output list of the detected keypoints.
   * \param elapsedTime : Elapsed time.
   * \param mask : Optional 8-bit integer mask to detect only where mask[i][j] != 0.
   */
  void detect(const cv::Mat &matImg, std::vector<cv::KeyPoint> &keyPoints, double &elapsedTime,
       const cv::Mat &mask = cv::Mat());

  /*!
   * Apply a set of affine transformations to the image, detect keypoints and
   * reproject them into initial image coordinates.
   * See http://www.ipol.im/pub/algo/my_affine_sift/ for the details.
   * See https://github.com/Itseez/opencv/blob/master/samples/python2/asift.py
   * for the Python implementation by Itseez and Matt Sheckells for the current
   * implementation in C++.
   * \param I : Input image.
   * \param listOfKeypoints : List of detected keypoints in the multiple images after
   * affine transformations.
   * \param listOfDescriptors : Corresponding list of descriptors.
   * \param listOfAffineI : Optional parameter, list of images after affine
   * transformations.
   */
  void detectExtractAffine(const vpImage<unsigned char> &I, std::vector<std::vector<cv::KeyPoint> > &listOfKeypoints,
                           std::vector<cv::Mat> &listOfDescriptors,
                           std::vector<vpImage<unsigned char> > *listOfAffineI = nullptr);

  /*!
   * Display the reference and the detected keypoints in the images.
   *
   * \param IRef : Input reference image.
   * \param ICurrent : Input current image.
   * \param size : Size of the displayed cross.
   */
  void display(const vpImage<unsigned char> &IRef, const vpImage<unsigned char> &ICurrent, unsigned int size = 3) VP_OVERRIDE;

  /*!
   * Display the reference keypoints.
   *
   * \param ICurrent : Input current image.
   * \param size : Size of the displayed crosses.
   * \param color : Color of the crosses.
   */
  void display(const vpImage<unsigned char> &ICurrent, unsigned int size = 3, const vpColor &color = vpColor::green) VP_OVERRIDE;

  /*!
   * Display the reference and the detected keypoints in the images.
   *
   * \param IRef : Input reference image.
   * \param ICurrent : Input current image.
   * \param size : Size of the displayed cross.
   */
  void display(const vpImage<vpRGBa> &IRef, const vpImage<vpRGBa> &ICurrent, unsigned int size = 3);

  /*!
   * Display the reference keypoints.
   *
   * \param ICurrent : Input current image.
   * \param size : Size of the displayed crosses.
   * \param color : Color of the crosses.
   */
  void display(const vpImage<vpRGBa> &ICurrent, unsigned int size = 3, const vpColor &color = vpColor::green);

  /*!
   * Display the matching lines between the detected keypoints with those
   * detected in one training image.
   *
   * \param IRef : Reference image, used to have the x-offset.
   * \param IMatching : Resulting image matching.
   * \param crossSize : Size of the displayed crosses.
   * \param lineThickness : Thickness of the displayed lines.
   * \param color : Color to use, if none, we pick randomly a color for each pair
   * of matching.
   */
  void displayMatching(const vpImage<unsigned char> &IRef, vpImage<unsigned char> &IMatching, unsigned int crossSize,
                       unsigned int lineThickness = 1, const vpColor &color = vpColor::green);

  /*!
   * Display matching between keypoints detected in the current image and with
   * those detected in the multiple training images. Display also RANSAC inliers
   * if the list is supplied.
   *
   * \param ICurrent : Current image.
   * \param IMatching : Resulting matching image.
   * \param ransacInliers : List of Ransac inliers or empty list if not available.
   * \param crossSize : Size of the displayed crosses.
   * \param lineThickness : Thickness of the displayed line.
   */
  void displayMatching(const vpImage<unsigned char> &ICurrent, vpImage<unsigned char> &IMatching,
                       const std::vector<vpImagePoint> &ransacInliers = std::vector<vpImagePoint>(),
                       unsigned int crossSize = 3, unsigned int lineThickness = 1);

  /*!
   * Display the matching lines between the detected keypoints with those
   * detected in one training image.
   *
   * \param IRef : Reference image, used to have the x-offset.
   * \param IMatching : Resulting image matching.
   * \param crossSize : Size of the displayed crosses.
   * \param lineThickness : Thickness of the displayed lines.
   * \param color : Color to use, if none, we pick randomly a color for each pair
   * of matching.
   */
  void displayMatching(const vpImage<unsigned char> &IRef, vpImage<vpRGBa> &IMatching, unsigned int crossSize,
                       unsigned int lineThickness = 1, const vpColor &color = vpColor::green);

  /*!
   * Display the matching lines between the detected keypoints with those
   * detected in one training image.
   *
   * \param IRef : Reference image, used to have the x-offset.
   * \param IMatching : Resulting image matching.
   * \param crossSize : Size of the displayed crosses.
   * \param lineThickness : Thickness of the displayed lines.
   * \param color : Color to use, if none, we pick randomly a color for each pair
   * of matching.
   */
  void displayMatching(const vpImage<vpRGBa> &IRef, vpImage<vpRGBa> &IMatching, unsigned int crossSize,
                       unsigned int lineThickness = 1, const vpColor &color = vpColor::green);

  /*!
   * Display matching between keypoints detected in the current image and with
   * those detected in the multiple training images. Display also RANSAC inliers
   * if the list is supplied.
   *
   * \param ICurrent : Current image.
   * \param IMatching : Resulting matching image.
   * \param ransacInliers : List of Ransac inliers or empty list if not available.
   * \param crossSize : Size of the displayed crosses.
   * \param lineThickness : Thickness of the displayed line.
   */
  void displayMatching(const vpImage<vpRGBa> &ICurrent, vpImage<vpRGBa> &IMatching,
                       const std::vector<vpImagePoint> &ransacInliers = std::vector<vpImagePoint>(),
                       unsigned int crossSize = 3, unsigned int lineThickness = 1);

  /*!
   * Extract the descriptors for each keypoints of the list.
   *
   * \param I : Input image.
   * \param keyPoints : List of keypoints we want to extract their descriptors.
   * \param descriptors : Descriptors matrix with at each row the descriptors
   * values for each keypoint.
   * \param trainPoints : Pointer to the list of 3D train points, when a keypoint
   * cannot be extracted, we need to remove the corresponding 3D point.
   */
  void extract(const vpImage<unsigned char> &I, std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors,
               std::vector<cv::Point3f> *trainPoints = nullptr);

  /*!
   * Extract the descriptors for each keypoints of the list.
   *
   * \param I_color : Input image.
   * \param keyPoints : List of keypoints we want to extract their descriptors.
   * \param descriptors : Descriptors matrix with at each row the descriptors
   * values for each keypoint.
   * \param trainPoints : Pointer to the list of 3D train points, when a keypoint
   * cannot be extracted, we need to remove the corresponding 3D point.
   */
  void extract(const vpImage<vpRGBa> &I_color, std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors,
               std::vector<cv::Point3f> *trainPoints = nullptr);

  /*!
   * Extract the descriptors for each keypoints of the list.
   *
   * \param matImg : Input image.
   * \param keyPoints : List of keypoints we want to extract their descriptors.
   * \param descriptors : Descriptors matrix with at each row the descriptors
   * values for each keypoint.
   * \param trainPoints : Pointer to the list of 3D train points, when a keypoint cannot
   * be extracted, we need to remove the corresponding 3D point.
   */
  void extract(const cv::Mat &matImg, std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors,
               std::vector<cv::Point3f> *trainPoints = nullptr);

  /*!
   * Extract the descriptors for each keypoints of the list.
   *
   * \param I : Input image.
   * \param keyPoints : List of keypoints we want to extract their descriptors.
   * \param descriptors : Descriptors matrix with at each row the descriptors
   * values for each keypoint.
   * \param elapsedTime : Elapsed time.
   * \param trainPoints : Pointer to the list of 3D train points, when a keypoint
   * cannot be extracted, we need to remove the corresponding 3D point.
   */
  void extract(const vpImage<unsigned char> &I, std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors,
               double &elapsedTime, std::vector<cv::Point3f> *trainPoints = nullptr);

  /*!
   * Extract the descriptors for each keypoints of the list.
   *
   * \param I_color : Input image.
   * \param keyPoints : List of keypoints we want to extract their descriptors.
   * \param descriptors : Descriptors matrix with at each row the descriptors
   * values for each keypoint.
   * \param elapsedTime : Elapsed time.
   * \param trainPoints : Pointer to the list of 3D train points, when a keypoint
   * cannot be extracted, we need to remove the corresponding 3D point.
   */
  void extract(const vpImage<vpRGBa> &I_color, std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors,
               double &elapsedTime, std::vector<cv::Point3f> *trainPoints = nullptr);

  /*!
   * Extract the descriptors for each keypoints of the list.
   *
   * \param matImg : Input image.
   * \param keyPoints : List of keypoints we want to extract their descriptors.
   * \param descriptors : Descriptors matrix with at each row the descriptors
   * values for each keypoint.
   * \param elapsedTime : Elapsed time.
   * \param trainPoints : Pointer to the list of 3D train points, when a keypoint
   * cannot be extracted, we need to remove the corresponding 3D point.
   */
  void extract(const cv::Mat &matImg, std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors, double &elapsedTime,
               std::vector<cv::Point3f> *trainPoints = nullptr);

  /*!
   * Get the covariance matrix when estimating the pose using the Virtual
   * Visual Servoing approach.
   *
   * \warning The compute covariance flag has to be true if you want to compute
   * the covariance matrix.
   *
   * \sa setCovarianceComputation
   */
  inline vpMatrix getCovarianceMatrix() const
  {
    if (!m_computeCovariance) {
      std::cout << "Warning : The covariance matrix has not been computed. "
        << "See setCovarianceComputation() to do it."
        << std::endl;
      return vpMatrix();
    }

    if (m_computeCovariance && !m_useRansacVVS) {
      std::cout << "Warning : The covariance matrix can only be computed "
        << "with a Virtual Visual Servoing approach." << std::endl
        << "Use setUseRansacVVS(true) to choose to use a pose "
        << "estimation method based on a Virtual Visual Servoing approach." << std::endl;
      return vpMatrix();
    }

    return m_covarianceMatrix;
  }

  /*!
   * Get the elapsed time to compute the keypoint detection.
   *
   * \return The elapsed time.
   */
  inline double getDetectionTime() const { return m_detectionTime; }

  /*!
   * Get the detector pointer.
   * \param type : Type of the detector.
   *
   * \return The detector or nullptr if the type passed in parameter does not
   * exist.
   */
  inline cv::Ptr<cv::FeatureDetector> getDetector(const vpFeatureDetectorType &type) const
  {
    std::map<vpFeatureDetectorType, std::string>::const_iterator it_name = m_mapOfDetectorNames.find(type);
    if (it_name == m_mapOfDetectorNames.end()) {
      std::cerr << "Internal problem with the feature type and the corresponding name!" << std::endl;
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
   * Get the detector pointer.
   * \param name : Name of the detector.
   *
   * \return The detector or nullptr if the name passed in parameter does not
   * exist.
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
   * Get the feature detector name associated to the type.
   */
  inline std::map<vpFeatureDetectorType, std::string> getDetectorNames() const { return m_mapOfDetectorNames; }

  /*!
   * Get the elapsed time to compute the keypoint extraction.
   *
   * \return The elapsed time.
   */
  inline double getExtractionTime() const { return m_extractionTime; }

  /*!
   * Get the extractor pointer.
   * \param type : Type of the descriptor extractor.
   *
   * \return The descriptor extractor or nullptr if the name passed in parameter
   * does not exist.
   */
  inline cv::Ptr<cv::DescriptorExtractor> getExtractor(const vpFeatureDescriptorType &type) const
  {
    std::map<vpFeatureDescriptorType, std::string>::const_iterator it_name = m_mapOfDescriptorNames.find(type);
    if (it_name == m_mapOfDescriptorNames.end()) {
      std::cerr << "Internal problem with the feature type and the corresponding name!" << std::endl;
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
   * Get the extractor pointer.
   * \param name : Name of the descriptor extractor.
   *
   * \return The descriptor extractor or nullptr if the name passed in parameter
   * does not exist.
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
   * Get the feature descriptor extractor name associated to the type.
   */
  inline std::map<vpFeatureDescriptorType, std::string> getExtractorNames() const { return m_mapOfDescriptorNames; }

  /*!
   * Get the image format to use when saving training images.
   *
   * \return The image format.
   */
  inline vpImageFormatType getImageFormat() const { return m_imageFormat; }

  /*!
   * Get the elapsed time to compute the matching.
   *
   * \return The elapsed time.
   */
  inline double getMatchingTime() const { return m_matchingTime; }

  /*!
   * Get the matcher pointer.
   *
   * \return The matcher pointer.
   */
  inline cv::Ptr<cv::DescriptorMatcher> getMatcher() const { return m_matcher; }

  /*!
   * Get the list of matches (correspondences between the indexes of the
   * detected keypoints and the train keypoints).
   *
   * \return The list of matches.
   */
  inline std::vector<cv::DMatch> getMatches() const { return m_filteredMatches; }

  /*!
   * Get the list of pairs with the correspondence between the matched query
   * and train keypoints.
   *
   * \return The list of pairs with the correspondence between the matched
   * query and train keypoints.
   */
  inline std::vector<std::pair<cv::KeyPoint, cv::KeyPoint> > getMatchQueryToTrainKeyPoints() const
  {
    std::vector<std::pair<cv::KeyPoint, cv::KeyPoint> > matchQueryToTrainKeyPoints(m_filteredMatches.size());
    for (size_t i = 0; i < m_filteredMatches.size(); i++) {
      matchQueryToTrainKeyPoints.push_back(
          std::pair<cv::KeyPoint, cv::KeyPoint>(m_queryFilteredKeyPoints[static_cast<size_t>(m_filteredMatches[i].queryIdx)],
                                                m_trainKeyPoints[static_cast<size_t>(m_filteredMatches[i].trainIdx)]));
    }
    return matchQueryToTrainKeyPoints;
  }

  /*!
   * Get the number of train images.
   *
   * \return The number of train images.
   */
  inline unsigned int getNbImages() const { return static_cast<unsigned int>(m_mapOfImages.size()); }

  /*!
   * Get the 3D coordinates of the object points matched (the corresponding 3D
   * coordinates in the object frame of the keypoints detected in the current
   * image after the matching).
   *
   * \param objectPoints : List of 3D coordinates in the object frame.
   */
  void getObjectPoints(std::vector<cv::Point3f> &objectPoints) const;

  /*!
   * Get the 3D coordinates of the object points matched (the corresponding 3D
   * coordinates in the object frame of the keypoints detected in the current
   * image after the matching).
   *
   * \param objectPoints : List of 3D coordinates in the object frame.
   */
  void getObjectPoints(std::vector<vpPoint> &objectPoints) const;

  /*!
   * Get the elapsed time to compute the pose.
   *
   * \return The elapsed time.
   */
  inline double getPoseTime() const { return m_poseTime; }

  /*!
   *  Get the descriptors matrix for the query keypoints.
   *
   *  \return Matrix with descriptors values at each row for each query
   *  keypoints.
   */
  inline cv::Mat getQueryDescriptors() const { return m_queryDescriptors; }

  /*!
   * Get the query keypoints list in OpenCV type.
   *
   * \param matches : If false return the list of all query keypoints extracted in the current image.
   * If true, return only the query keypoints list that have matches.
   * \param keyPoints : List of query keypoints (or keypoints detected in the
   * current image).
   */
  void getQueryKeyPoints(std::vector<cv::KeyPoint> &keyPoints, bool matches = true) const;

  /*!
   * Get the query keypoints list in ViSP type.
   *
   * \param keyPoints : List of query keypoints (or keypoints detected in the
   * current image).
   * \param matches : If false return the list of all query keypoints extracted in the current image.
   * If true, return only the query keypoints list that have matches.
   */
  void getQueryKeyPoints(std::vector<vpImagePoint> &keyPoints, bool matches = true) const;

  /*!
   * Get the list of Ransac inliers.
   *
   * \return The list of Ransac inliers.
   */
  inline std::vector<vpImagePoint> getRansacInliers() const { return m_ransacInliers; }

  /*!
   * Get the list of Ransac outliers.
   *
   * \return The list of Ransac outliers.
   */
  inline std::vector<vpImagePoint> getRansacOutliers() const { return m_ransacOutliers; }

  /*!
   *  Get the train descriptors matrix.
   *
   *  \return : Matrix with descriptors values at each row for each train
   *  keypoints (or reference keypoints).
   */
  inline cv::Mat getTrainDescriptors() const { return m_trainDescriptors; }

  /*!
   * Get the train keypoints list in OpenCV type.
   *
   * \param keyPoints : List of train keypoints (or reference keypoints).
   */
  void getTrainKeyPoints(std::vector<cv::KeyPoint> &keyPoints) const;

  /*!
   * Get the train keypoints list in ViSP type.
   *
   * \param keyPoints : List of train keypoints (or reference keypoints).
   */
  void getTrainKeyPoints(std::vector<vpImagePoint> &keyPoints) const;

  /*!
   * Get the train points (the 3D coordinates in the object frame) list in
   * OpenCV type.
   *
   * \param points : List of train points (or reference points).
   */
  void getTrainPoints(std::vector<cv::Point3f> &points) const;

  /*!
   * Get the train points (the 3D coordinates in the object frame) list in ViSP
   * type.
   *
   * \param points : List of train points (or reference points).
   */
  void getTrainPoints(std::vector<vpPoint> &points) const;

  /*!
   * Initialize a matcher based on its name.
   *
   * \param matcherName : Name of the matcher (e.g BruteForce, FlannBased).
   */
  void initMatcher(const std::string &matcherName);

  /*!
   * Insert a reference image and a current image side-by-side.
   *
   * \param IRef : Reference image.
   * \param ICurrent : Current image.
   * \param IMatching : Matching image for displaying all the matching between
   * the query keypoints and those detected in the training images.
   */
  void insertImageMatching(const vpImage<unsigned char> &IRef, const vpImage<unsigned char> &ICurrent,
                           vpImage<unsigned char> &IMatching);

  /*!
   * Insert the different training images in the matching image.
   *
   * \param ICurrent : Current image.
   * \param IMatching : Matching image for displaying all the matching between
   * the query keypoints and those detected in the training images
   */
  void insertImageMatching(const vpImage<unsigned char> &ICurrent, vpImage<unsigned char> &IMatching);

  /*!
   * Insert a reference image and a current image side-by-side.
   *
   * \param IRef : Reference image.
   * \param ICurrent : Current image.
   * \param IMatching : Matching image for displaying all the matching between
   * the query keypoints and those detected in the training images.
   */
  void insertImageMatching(const vpImage<vpRGBa> &IRef, const vpImage<vpRGBa> &ICurrent, vpImage<vpRGBa> &IMatching);

  /*!
   * Insert the different training images in the matching image.
   *
   * \param ICurrent : Current image.
   * \param IMatching : Matching image for displaying all the matching between
   * the query keypoints and those detected in the training images
   */
  void insertImageMatching(const vpImage<vpRGBa> &ICurrent, vpImage<vpRGBa> &IMatching);

  /*!
   * Load configuration parameters from an XML config file.
   *
   * \param configFile : Path to the XML config file.
   */
  void loadConfigFile(const std::string &configFile);

  /*!
   * Load learning data saved on disk.
   *
   * \param filename : Path of the learning file.
   * \param binaryMode : If true, the learning file is in a binary mode,
   * otherwise it is in XML mode.
   * \param append : If true, concatenate the learning data, otherwise reset the variables.
   */
  void loadLearningData(const std::string &filename, bool binaryMode = false, bool append = false);

  /*!
   * Match keypoints based on distance between their descriptors.
   *
   * \param trainDescriptors : Train descriptors (or reference descriptors).
   * \param queryDescriptors : Query descriptors.
   * \param matches : Output list of matches.
   * \param elapsedTime : Elapsed time.
   */
  void match(const cv::Mat &trainDescriptors, const cv::Mat &queryDescriptors, std::vector<cv::DMatch> &matches,
             double &elapsedTime);

  /*!
   * Match keypoints detected in the image with those built in the reference
   * list.
   *
   * \param I : Input current image.
   * \return The number of matched keypoints.
   */
  unsigned int matchPoint(const vpImage<unsigned char> &I) VP_OVERRIDE;

  /*!
   * Match keypoints detected in a region of interest of the image with those
   * built in the reference list.
   *
   * \param I : Input image.
   * \param iP : Coordinate of the top-left corner of the region of interest.
   * \param height : Height of the region of interest.
   * \param width : Width of the region of interest.
   * \return The number of matched keypoints.
   */
  unsigned int matchPoint(const vpImage<unsigned char> &I, const vpImagePoint &iP, unsigned int height,
                          unsigned int width) VP_OVERRIDE;

  /*!
   * Match keypoints detected in a region of interest of the image with those
   * built in the reference list.
   *
   * \param I : Input image.
   * \param rectangle : Rectangle of the region of interest.
   * \return The number of matched keypoints.
   */
  unsigned int matchPoint(const vpImage<unsigned char> &I, const vpRect &rectangle) VP_OVERRIDE;

  /*!
   * Match query keypoints with those built in the reference list using buildReference().
   *
   * \param queryKeyPoints : List of the query keypoints.
   * \param queryDescriptors : List of the query descriptors.
   *
   * \return The number of matched keypoints.
   */
  unsigned int matchPoint(const std::vector<cv::KeyPoint> &queryKeyPoints, const cv::Mat &queryDescriptors);

  /*!
   * Match keypoints detected in the image with those built in the reference
   * list and compute the pose.
   *
   * \param I : Input image.
   * \param cam : Camera parameters.
   * \param cMo : Homogeneous matrix between the object frame and the camera frame.
   * \param func : Function pointer to filter the pose in Ransac pose
   * estimation, if we want to eliminate the poses which do not respect some criterion.
   * \param rectangle : Rectangle corresponding to the ROI (Region of Interest) to consider.
   * \return True if the matching and the pose estimation are OK, false otherwise.
   */
  bool matchPoint(const vpImage<unsigned char> &I, const vpCameraParameters &cam, vpHomogeneousMatrix &cMo,
                  bool (*func)(const vpHomogeneousMatrix &) = nullptr, const vpRect &rectangle = vpRect());

  /*!
   * Match keypoints detected in the image with those built in the reference
   * list and compute the pose.
   *
   * \param I : Input image.
   * \param cam : Camera parameters.
   * \param cMo : Homogeneous matrix between the object frame and the camera frame.
   * \param error : Reprojection mean square error (in pixel) between the
   * 2D points and the projection of the 3D points with the estimated pose.
   * \param elapsedTime : Time to detect, extract, match and compute the pose.
   * \param func : Function pointer to filter the pose in Ransac pose
   * estimation, if we want to eliminate the poses which do not respect some criterion.
   * \param rectangle : Rectangle corresponding to the ROI (Region of Interest) to consider.
   * \return True if the matching and the pose estimation are OK, false otherwise.
   */
  bool matchPoint(const vpImage<unsigned char> &I, const vpCameraParameters &cam, vpHomogeneousMatrix &cMo,
                  double &error, double &elapsedTime, bool (*func)(const vpHomogeneousMatrix &) = nullptr,
                  const vpRect &rectangle = vpRect());

  /*!
   * Match keypoints detected in the image with those built in the reference
   * list and return the bounding box and the center of gravity.
   *
   * \param I : Input image.
   * \param boundingBox : Bounding box that contains the good matches.
   * \param centerOfGravity : Center of gravity computed from the location of
   * the good matches (could differ of the center of the bounding box).
   * \param isPlanarObject : If the object is planar, the homography matrix is
   * estimated to eliminate outliers, otherwise it is the fundamental matrix
   * which is estimated.
   * \param imPts1 : Pointer to the list of reference keypoints if not null.
   * \param imPts2 : Pointer to the list of current keypoints if not null.
   * \param meanDescriptorDistance : Pointer to the value
   * of the average distance of the descriptors if not null.
   * \param detectionScore : Pointer to the value of the detection score if not null.
   * \param rectangle : Rectangle corresponding to the ROI (Region of Interest)
   * to consider.
   * \return True if the object is present, false otherwise.
   */
  bool matchPointAndDetect(const vpImage<unsigned char> &I, vpRect &boundingBox, vpImagePoint &centerOfGravity,
                           const bool isPlanarObject = true, std::vector<vpImagePoint> *imPts1 = nullptr,
                           std::vector<vpImagePoint> *imPts2 = nullptr, double *meanDescriptorDistance = nullptr,
                           double *detectionScore = nullptr, const vpRect &rectangle = vpRect());

  /*!
   * Match keypoints detected in the image with those built in the reference
   * list, compute the pose and return also the bounding box and the center of
   * gravity.
   *
   * \param I : Input image.
   * \param cam : Camera parameters.
   * \param cMo : Homogeneous matrix between the object frame and the camera frame.
   * \param error : Reprojection mean square error (in pixel) between the
   * 2D points and the projection of the 3D points with the estimated pose.
   * \param elapsedTime : Time to detect, extract, match and compute the pose.
   * \param boundingBox : Bounding box that contains the good matches.
   * \param centerOfGravity : Center of gravity computed from the location of
   * the good matches (could differ of the center of the bounding box).
   * \param func : Function pointer to filter the pose in Ransac pose estimation, if we
   * want to eliminate the poses which do not respect some criterion.
   * \param rectangle : Rectangle corresponding to the ROI (Region of Interest) to consider.
   * \return True if the matching and the pose estimation are OK, false otherwise.
   */
  bool matchPointAndDetect(const vpImage<unsigned char> &I, const vpCameraParameters &cam, vpHomogeneousMatrix &cMo,
                           double &error, double &elapsedTime, vpRect &boundingBox, vpImagePoint &centerOfGravity,
                           bool (*func)(const vpHomogeneousMatrix &) = nullptr, const vpRect &rectangle = vpRect());

  /*!
   * Match keypoints detected in the image with those built in the reference
   * list.
   *
   * \param I_color : Input current image.
   * \return The number of matched keypoints.
   */
  unsigned int matchPoint(const vpImage<vpRGBa> &I_color);

  /*!
   * Match keypoints detected in a region of interest of the image with those
   * built in the reference list.
   *
   * \param I_color : Input image.
   * \param iP : Coordinate of the top-left corner of the region of interest.
   * \param height : Height of the region of interest.
   * \param width : Width of the region of interest.
   * \return The number of matched keypoints.
   */
  unsigned int matchPoint(const vpImage<vpRGBa> &I_color, const vpImagePoint &iP, unsigned int height,
                          unsigned int width);

  /*!
   * Match keypoints detected in a region of interest of the image with those
   * built in the reference list.
   *
   * \param I_color : Input image.
   * \param rectangle : Rectangle of the region of interest.
   * \return The number of matched keypoints.
   */
  unsigned int matchPoint(const vpImage<vpRGBa> &I_color, const vpRect &rectangle);

  /*!
   * Match keypoints detected in the image with those built in the reference
   * list and compute the pose.
   *
   * \param I_color : Input image.
   * \param cam : Camera parameters.
   * \param cMo : Homogeneous matrix between the object frame and the camera frame.
   * \param func : Function pointer to filter the pose in Ransac pose
   * estimation, if we want to eliminate the poses which do not respect some criterion.
   * \param rectangle : Rectangle corresponding to the ROI (Region of Interest) to consider.
   * \return True if the matching and the pose estimation are OK, false otherwise.
   */
  bool matchPoint(const vpImage<vpRGBa> &I_color, const vpCameraParameters &cam, vpHomogeneousMatrix &cMo,
                  bool (*func)(const vpHomogeneousMatrix &) = nullptr, const vpRect &rectangle = vpRect());

  /*!
   * Match keypoints detected in the image with those built in the reference
   * list and compute the pose.
   *
   * \param I_color : Input image.
   * \param cam : Camera parameters.
   * \param cMo : Homogeneous matrix between the object frame and the camera frame.
   * \param error : Reprojection mean square error (in pixel) between the
   * 2D points and the projection of the 3D points with the estimated pose.
   * \param elapsedTime : Time to detect, extract, match and compute the pose.
   * \param func : Function pointer to filter the pose in Ransac pose
   * estimation, if we want to eliminate the poses which do not respect some criterion.
   * \param rectangle : Rectangle corresponding to the ROI (Region of Interest) to consider.
   * \return True if the matching and the pose estimation are OK, false otherwise.
   */
  bool matchPoint(const vpImage<vpRGBa> &I_color, const vpCameraParameters &cam, vpHomogeneousMatrix &cMo,
                  double &error, double &elapsedTime, bool (*func)(const vpHomogeneousMatrix &) = nullptr,
                  const vpRect &rectangle = vpRect());

  /*!
   * Reset the instance as if we would declare another vpKeyPoint variable.
   */
  void reset();

  /*!
   * Save the learning data in a file in XML or binary mode.
   *
   * \param filename : Path of the save file.
   * \param binaryMode : If true, the data are saved in binary mode, otherwise
   * in XML mode.
   * \param saveTrainingImages : If true, save also the training images on disk.
   */
  void saveLearningData(const std::string &filename, bool binaryMode = false, bool saveTrainingImages = true);

  /*!
   * Set if the covariance matrix has to be computed in the Virtual Visual
   * Servoing approach.
   *
   * \param flag : True if the covariance has to be computed, false otherwise.
   */
  inline void setCovarianceComputation(const bool &flag)
  {
    m_computeCovariance = flag;
    if (!m_useRansacVVS) {
      std::cout << "Warning : The covariance matrix can only be computed "
        << "with a Virtual Visual Servoing approach." << std::endl
        << "Use setUseRansacVVS(true) to choose to use a pose "
        << "estimation method based on a Virtual "
        << "Visual Servoing approach." << std::endl;
    }
  }

  /*!
   * Set the method to decide if the object is present or not.
   *
   * \param method : Detection method (detectionThreshold or detectionScore).
   */
  inline void setDetectionMethod(const vpDetectionMethodType &method) { m_detectionMethod = method; }

  /*!
   * Set and initialize a detector.
   *
   * \param detectorType : Type of the detector.
   */
  inline void setDetector(const vpFeatureDetectorType &detectorType)
  {
    m_detectorNames.clear();
    m_detectorNames.push_back(m_mapOfDetectorNames[detectorType]);
    m_detectors.clear();
    initDetector(m_mapOfDetectorNames[detectorType]);
  }

  /*!
   *  Set and initialize a detector denominated by his name \p detectorName.
   *
   *  \param detectorName : Name of the detector.
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
   * Template function to set to a \p parameterName a value for a specific
   * detector named by his \p detectorName.
   *
   * \param detectorName : Name of the detector
   * \param parameterName : Name of the parameter
   * \param value : Value to set
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
   * Set and initialize a list of detectors denominated by their names \p
   * detectorNames.
   *
   *  \param detectorNames : List of detector names.
   */
  inline void setDetectors(const std::vector<std::string> &detectorNames)
  {
    m_detectorNames.clear();
    m_detectors.clear();
    m_detectorNames = detectorNames;
    initDetectors(m_detectorNames);
  }

  /*!
   * Set and initialize a descriptor extractor.
   *
   * \param extractorType : Type of the descriptor extractor.
   */
  inline void setExtractor(const vpFeatureDescriptorType &extractorType)
  {
    m_extractorNames.clear();
    m_extractorNames.push_back(m_mapOfDescriptorNames[extractorType]);
    m_extractors.clear();
    initExtractor(m_mapOfDescriptorNames[extractorType]);
  }

  /*!
   * Set and initialize a descriptor extractor denominated by his name \p
   * extractorName.
   *
   *  \param extractorName : Name of the extractor.
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
   * Template function to set to a \p parameterName a value for a specific
   * extractor named by his \p extractorName.
   *
   * \param extractorName : Name of the extractor
   * \param parameterName : Name of the parameter
   * \param value : Value to set
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
   * Set and initialize a list of extractors denominated by their names \p
   * extractorNames.
   *
   * \param extractorNames : List of extractor names.
   */
  inline void setExtractors(const std::vector<std::string> &extractorNames)
  {
    m_extractorNames.clear();
    m_extractorNames = extractorNames;
    m_extractors.clear();
    initExtractors(m_extractorNames);
  }

  /*!
   * Set the image format to use when saving training images.
   *
   * \param imageFormat : The image format.
   */
  inline void setImageFormat(const vpImageFormatType &imageFormat) { m_imageFormat = imageFormat; }

  /*!
   * Set and initialize a matcher denominated by his name \p matcherName.
   * The different matchers are:
   *   - BruteForce (it uses L2 distance)
   *   - BruteForce-L1
   *   - BruteForce-Hamming
   *   - BruteForce-Hamming(2)
   *   - FlannBased
   *
   * L1 and L2 norms are preferable choices for SIFT and SURF descriptors,
   * NORM_HAMMING should be used with ORB, BRISK and BRIEF, NORM_HAMMING2
   * should be used with ORB when WTA_K==3 or 4.
   *
   * \param matcherName : Name of the matcher.
   */
  inline void setMatcher(const std::string &matcherName)
  {
    m_matcherName = matcherName;
    initMatcher(m_matcherName);
  }

  /*!
   * Set maximum number of keypoints to extract.
   * \warning This functionality is only available for ORB and SIFT extractors.
   * \param maxFeatures : Maximum number of keypoints to extract. Set -1 to use default values.
   */
  void setMaxFeatures(int maxFeatures) { m_maxFeatures = maxFeatures; }

  /*!
   * Set the filtering method to eliminate false matching.
   * The different methods are:
   * - vpKeyPoint::constantFactorDistanceThreshold : Keep matches whose descriptor
   *   distance is below dist_min * factor.
   * - vpKeyPoint::stdDistanceThreshold : Keep matches whose the descriptor distance is
   *   below dist_min + standard_deviation.
   * - vpKeyPoint::ratioDistanceThreshold : Keep matches enough discriminated when the ratio
   *   distance between the 2 best matches is below the threshold.
   * - vpKeyPoint::stdAndRatioDistanceThreshold : Keep matches that agree with at least
   *   one of the two conditions.
   * - vpKeyPoint::noFilterMatching : No filter is applied.
   *
   * \param filterType : Type of the filtering method
   */
  inline void setFilterMatchingType(const vpFilterMatchingType &filterType)
  {
    m_filterType = filterType;

    // Use k-nearest neighbors (knn) to retrieve the two best matches for a
    // keypoint  So this is useful only for ratioDistanceThreshold method
    if (filterType == ratioDistanceThreshold || filterType == stdAndRatioDistanceThreshold) {
      m_useKnn = true;

#if (VISP_HAVE_OPENCV_VERSION >= 0x020400 && VISP_HAVE_OPENCV_VERSION < 0x030000)
      if (m_matcher != nullptr && m_matcherName == "BruteForce") {
        // if a matcher is already initialized, disable the crossCheck
        // because it will not work with knnMatch
        m_matcher->set("crossCheck", false);
      }
#endif
    }
    else {
      m_useKnn = false;

#if (VISP_HAVE_OPENCV_VERSION >= 0x020400 && VISP_HAVE_OPENCV_VERSION < 0x030000)
      if (m_matcher != nullptr && m_matcherName == "BruteForce") {
        // if a matcher is already initialized, set the crossCheck mode if
        // necessary
        m_matcher->set("crossCheck", m_useBruteForceCrossCheck);
      }
#endif
    }
  }

  /*!
   * Set the factor value for the filtering method:
   * constantFactorDistanceThreshold.
   *
   * \param factor : Factor value
   */
  inline void setMatchingFactorThreshold(const double factor)
  {
    if (factor > 0.0) {
      m_matchingFactorThreshold = factor;
    }
    else {
      throw vpException(vpException::badValue, "The factor must be positive.");
    }
  }

  /*!
   * Set the ratio value for the filtering method: ratioDistanceThreshold.
   *
   * \param ratio : Ratio value (]0 ; 1])
   */
  inline void setMatchingRatioThreshold(double ratio)
  {
    if (ratio > 0.0 && (ratio < 1.0 || std::fabs(ratio - 1.0) < std::numeric_limits<double>::epsilon())) {
      m_matchingRatioThreshold = ratio;
    }
    else {
      throw vpException(vpException::badValue, "The ratio must be in the interval ]0 ; 1].");
    }
  }

  /*!
   * Set the percentage value for defining the cardinality of the consensus
   * group.
   *
   * \param percentage : Percentage value (]0 ; 100])
   */
  inline void setRansacConsensusPercentage(double percentage)
  {
    if (percentage > 0.0 &&
        (percentage < 100.0 || std::fabs(percentage - 100.0) < std::numeric_limits<double>::epsilon())) {
      m_ransacConsensusPercentage = percentage;
    }
    else {
      throw vpException(vpException::badValue, "The percentage must be in the interval ]0 ; 100].");
    }
  }

  /*!
   * Set filter flag for RANSAC pose estimation.
   */
  inline void setRansacFilterFlag(const vpPose::RANSAC_FILTER_FLAGS &flag) { m_ransacFilterFlag = flag; }

  /*!
   * Set the maximum number of iterations for the Ransac pose estimation
   * method.
   *
   * \param nbIter : Maximum number of iterations for the Ransac
   */
  inline void setRansacIteration(int nbIter)
  {
    if (nbIter > 0) {
      m_nbRansacIterations = nbIter;
    }
    else {
      throw vpException(vpException::badValue, "The number of iterations must be greater than zero.");
    }
  }

  /*!
   * Use or not the multithreaded version.
   *
   * \note Needs C++11 or higher.
   */
  inline void setRansacParallel(bool parallel) { m_ransacParallel = parallel; }

  /*!
   * Set the number of threads to use if multithreaded RANSAC pose.
   *
   * \param nthreads : Number of threads, if 0 the number of CPU threads will be determined
   * \sa setRansacParallel
   */
  inline void setRansacParallelNbThreads(unsigned int nthreads) { m_ransacParallelNbThreads = nthreads; }

  /*!
   * Set the maximum reprojection error (in pixel) to determine if a point is
   * an inlier or not.
   *
   * \param reprojectionError : Maximum reprojection error in pixel (used by
   * OpenCV function)
   */
  inline void setRansacReprojectionError(double reprojectionError)
  {
    if (reprojectionError > 0.0) {
      m_ransacReprojectionError = reprojectionError;
    }
    else {
      throw vpException(vpException::badValue, "The Ransac reprojection "
                                               "threshold must be positive "
                                               "as we deal with distance.");
    }
  }

  /*!
   * Set the minimum number of inlier for the Ransac pose estimation method.
   *
   * \param minCount : Minimum number of inlier for the consensus
   */
  inline void setRansacMinInlierCount(int minCount)
  {
    if (minCount > 0) {
      m_nbRansacMinInlierCount = minCount;
    }
    else {
      throw vpException(vpException::badValue, "The minimum number of inliers must be greater than zero.");
    }
  }

  /*!
   * Set the maximum error (in meter) to determine if a point is an inlier or
   * not.
   *
   * \param threshold : Maximum error in meter for ViSP function
   */
  inline void setRansacThreshold(double threshold)
  {
    if (threshold > 0.0) {
      m_ransacThreshold = threshold;
    }
    else {
      throw vpException(vpException::badValue, "The Ransac threshold must be positive as we deal with distance.");
    }
  }

  /*!
   * Set if multiple affine transformations must be used to detect and extract
   * keypoints.
   *
   * \param useAffine : True to use multiple affine transformations, false
   * otherwise
   */
  inline void setUseAffineDetection(bool useAffine) { m_useAffineDetection = useAffine; }

#if (VISP_HAVE_OPENCV_VERSION >= 0x020400 && VISP_HAVE_OPENCV_VERSION < 0x030000)
  /*!
   * Set if cross check method must be used to eliminate some false matches
   * with a brute-force matching method.
   *
   * \param useCrossCheck : True to use cross check, false otherwise
   */
  inline void setUseBruteForceCrossCheck(bool useCrossCheck)
  {
    // Only available with BruteForce and with k=1 (i.e not used with a
    // ratioDistanceThreshold method)
    if (m_matcher != nullptr && !m_useKnn && m_matcherName == "BruteForce") {
      m_matcher->set("crossCheck", useCrossCheck);
    }
    else if (m_matcher != nullptr && m_useKnn && m_matcherName == "BruteForce") {
      std::cout << "Warning, you try to set the crossCheck parameter with a "
        << "BruteForce matcher but knn is enabled"
        << " (the filtering method uses a ratio constraint)" << std::endl;
    }
  }
#endif

  /*!
   * Set if we want to match the train keypoints to the query keypoints.
   *
   * \param useMatchTrainToQuery : True to match the train keypoints to the
   * query keypoints
   */
  inline void setUseMatchTrainToQuery(bool useMatchTrainToQuery) { m_useMatchTrainToQuery = useMatchTrainToQuery; }

  /*!
   * Set the flag to choose between a percentage value of inliers for the
   * cardinality of the consensus group or a minimum number.
   *
   * \param usePercentage : True to a percentage ratio of inliers, otherwise
   * use a specified number of inliers
   */
  inline void setUseRansacConsensusPercentage(bool usePercentage) { m_useConsensusPercentage = usePercentage; }

  /*!
   * Set the flag to choose between the OpenCV or ViSP Ransac pose estimation
   * function.
   *
   * \param ransacVVS : True to use ViSP function, otherwise use OpenCV
   * function
   */
  inline void setUseRansacVVS(bool ransacVVS) { m_useRansacVVS = ransacVVS; }

  /*!
   * Set the flag to filter matches where multiple query keypoints are matched
   * to the same train keypoints.
   *
   * \param singleMatchFilter : True to use the single match filter.
   */
  inline void setUseSingleMatchFilter(bool singleMatchFilter) { m_useSingleMatchFilter = singleMatchFilter; }

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
  //! Filtering flag for RANSAC and degenerate configuration check
  vpPose::RANSAC_FILTER_FLAGS m_ransacFilterFlag;
  //! List of inliers.
  std::vector<vpImagePoint> m_ransacInliers;
  //! List of outliers.
  std::vector<vpImagePoint> m_ransacOutliers;
  //! If true, use parallel RANSAC
  bool m_ransacParallel;
  //! Number of threads (if 0, try to determine the number of CPU threads)
  unsigned int m_ransacParallelNbThreads;
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
  //! Grayscale image buffer, used when passing color images
  vpImage<unsigned char> m_I;
  //! Max number of features to extract, -1 to use default values
  int m_maxFeatures;

  /*!
   * Apply an affine and skew transformation to an image.
   * \param tilt : Tilt value in the direction of x.
   * \param phi : Rotation value.
   * \param img : Modified image after the transformation.
   * \param mask : Mask containing the location of the image pixels after the transformation.
   * \param Ai : Inverse affine matrix
   */
  void affineSkew(double tilt, double phi, cv::Mat &img, cv::Mat &mask, cv::Mat &Ai);

  /*!
   * Compute the pose estimation error, the mean square error (in pixel) between
   * the location of the detected keypoints and the location of the projection
   * of the 3D model with the estimated pose.
   *
   * \param matchKeyPoints : List of pairs between the detected keypoints and
   * the corresponding 3D points.
   * \param cam : Camera parameters.
   * \param cMo_est : Estimated pose of the object.
   *
   * \return The mean square error (in pixel) between the location of the
   * detected keypoints and the location of the projection of the 3D model with
   * the estimated pose.
   */
  double computePoseEstimationError(const std::vector<std::pair<cv::KeyPoint, cv::Point3f> > &matchKeyPoints,
                                    const vpCameraParameters &cam, const vpHomogeneousMatrix &cMo_est);

  /*!
   * Filter the matches using the desired filtering method.
   */
  void filterMatches();

  /*!
   * Initialize method for RANSAC parameters and for detectors, extractors and
   * matcher, and for others parameters.
   */
  void init() VP_OVERRIDE;

  /*!
   * Initialize a keypoint detector based on its name.
   *
   * \param[in] detectorNames : Name of the detector (e.g FAST, SIFT, SURF, etc.).
   */
  void initDetector(const std::string &detectorNames);

  /*!
   * Initialize a list of keypoints detectors if we want to concatenate multiple
   * detectors.
   *
   * \param[in] detectorNames : List of detector names.
   */
  void initDetectors(const std::vector<std::string> &detectorNames);

  /*!
   * Initialize a descriptor extractor based on its name.
   *
   * \param[in] extractorName : Name of the extractor (e.g SIFT, SURF, ORB, etc.).
   */
  void initExtractor(const std::string &extractorName);

  /*!
   * Initialize a list of descriptor extractors if we want to concatenate
   * multiple extractors.
   *
   * \param[in] extractorNames : List of extractor names.
   */
  void initExtractors(const std::vector<std::string> &extractorNames);

  /*!
   * Initialize map of available detectors and descriptors.
   */
  void initFeatureNames();

  inline size_t myKeypointHash(const cv::KeyPoint &kp)
  {
    size_t _val = 2166136261U, scale = 16777619U;
    Cv32suf u;
    u.f = kp.pt.x;
    _val = (scale * _val) ^ u.u;
    u.f = kp.pt.y;
    _val = (scale * _val) ^ u.u;
    u.f = kp.size;
    _val = (scale * _val) ^ u.u;
    // As the keypoint angle can be computed for certain type of keypoint only
    // when extracting  the corresponding descriptor, the angle field is not
    // taking into account for the hash
    //    u.f = kp.angle; _val = (scale * _val) ^ u.u;
    u.f = kp.response;
    _val = (scale * _val) ^ u.u;
    _val = (scale * _val) ^ (static_cast<size_t>(kp.octave));
    _val = (scale * _val) ^ (static_cast<size_t>(kp.class_id));
    return _val;
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
    virtual bool empty() const VP_OVERRIDE;

  protected:
    virtual void detect(cv::InputArray image, CV_OUT std::vector<cv::KeyPoint> &keypoints,
                        cv::InputArray mask = cv::noArray())  VP_OVERRIDE;
    virtual void detectImpl(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints,
                            const cv::Mat &mask = cv::Mat()) const;

    cv::Ptr<cv::FeatureDetector> m_detector;
    int m_maxLevel;
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
    KeyPointsFilter() { }

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
END_VISP_NAMESPACE
#endif
#endif
