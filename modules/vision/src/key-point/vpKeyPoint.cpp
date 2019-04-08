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
 * Key point functionalities.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

#include <iomanip>
#include <limits>

#include <visp3/core/vpIoTools.h>
#include <visp3/vision/vpKeyPoint.h>

#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)

#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
#include <opencv2/calib3d/calib3d.hpp>
#endif

namespace
{
// Specific Type transformation functions
inline cv::DMatch knnToDMatch(const std::vector<cv::DMatch> &knnMatches)
{
  if (knnMatches.size() > 0) {
    return knnMatches[0];
  }

  return cv::DMatch();
}

inline vpImagePoint matchRansacToVpImage(const std::pair<cv::KeyPoint, cv::Point3f> &pair)
{
  return vpImagePoint(pair.first.pt.y, pair.first.pt.x);
}

}

/*!
  Constructor to initialize the specified detector, descriptor, matcher and
  filtering method.

  \param detectorType : Type of feature detector.
  \param descriptorType : Type of the descriptor extractor.
  \param matcherName : Name of the matcher.
  \param filterType : Filtering matching method chosen.
 */
vpKeyPoint::vpKeyPoint(const vpFeatureDetectorType &detectorType, const vpFeatureDescriptorType &descriptorType,
                       const std::string &matcherName, const vpFilterMatchingType &filterType)
  : m_computeCovariance(false), m_covarianceMatrix(), m_currentImageId(0), m_detectionMethod(detectionScore),
    m_detectionScore(0.15), m_detectionThreshold(100.0), m_detectionTime(0.), m_detectorNames(), m_detectors(),
    m_extractionTime(0.), m_extractorNames(), m_extractors(), m_filteredMatches(), m_filterType(filterType),
    m_imageFormat(jpgImageFormat), m_knnMatches(), m_mapOfImageId(), m_mapOfImages(), m_matcher(),
    m_matcherName(matcherName), m_matches(), m_matchingFactorThreshold(2.0), m_matchingRatioThreshold(0.85),
    m_matchingTime(0.), m_matchRansacKeyPointsToPoints(), m_nbRansacIterations(200), m_nbRansacMinInlierCount(100),
    m_objectFilteredPoints(), m_poseTime(0.), m_queryDescriptors(), m_queryFilteredKeyPoints(), m_queryKeyPoints(),
    m_ransacConsensusPercentage(20.0), m_ransacFilterFlag(vpPose::NO_FILTER), m_ransacInliers(), m_ransacOutliers(),
    m_ransacParallel(false), m_ransacParallelNbThreads(0), m_ransacReprojectionError(6.0),
    m_ransacThreshold(0.01), m_trainDescriptors(), m_trainKeyPoints(), m_trainPoints(), m_trainVpPoints(),
    m_useAffineDetection(false),
#if (VISP_HAVE_OPENCV_VERSION >= 0x020400 && VISP_HAVE_OPENCV_VERSION < 0x030000)
    m_useBruteForceCrossCheck(true),
#endif
    m_useConsensusPercentage(false), m_useKnn(false), m_useMatchTrainToQuery(false), m_useRansacVVS(true),
    m_useSingleMatchFilter(true), m_I()
{
  initFeatureNames();

  m_detectorNames.push_back(m_mapOfDetectorNames[detectorType]);
  m_extractorNames.push_back(m_mapOfDescriptorNames[descriptorType]);

  init();
}

/*!
  Constructor to initialize the specified detector, descriptor, matcher and
  filtering method.

  \param detectorName : Name of the detector.
  \param extractorName : Name of the extractor.
  \param matcherName : Name of the matcher.
  \param filterType : Filtering matching method chosen.
 */
vpKeyPoint::vpKeyPoint(const std::string &detectorName, const std::string &extractorName,
                       const std::string &matcherName, const vpFilterMatchingType &filterType)
  : m_computeCovariance(false), m_covarianceMatrix(), m_currentImageId(0), m_detectionMethod(detectionScore),
    m_detectionScore(0.15), m_detectionThreshold(100.0), m_detectionTime(0.), m_detectorNames(), m_detectors(),
    m_extractionTime(0.), m_extractorNames(), m_extractors(), m_filteredMatches(), m_filterType(filterType),
    m_imageFormat(jpgImageFormat), m_knnMatches(), m_mapOfImageId(), m_mapOfImages(), m_matcher(),
    m_matcherName(matcherName), m_matches(), m_matchingFactorThreshold(2.0), m_matchingRatioThreshold(0.85),
    m_matchingTime(0.), m_matchRansacKeyPointsToPoints(), m_nbRansacIterations(200), m_nbRansacMinInlierCount(100),
    m_objectFilteredPoints(), m_poseTime(0.), m_queryDescriptors(), m_queryFilteredKeyPoints(), m_queryKeyPoints(),
    m_ransacConsensusPercentage(20.0), m_ransacFilterFlag(vpPose::NO_FILTER), m_ransacInliers(), m_ransacOutliers(),
    m_ransacParallel(false), m_ransacParallelNbThreads(0), m_ransacReprojectionError(6.0),
    m_ransacThreshold(0.01), m_trainDescriptors(), m_trainKeyPoints(), m_trainPoints(), m_trainVpPoints(),
    m_useAffineDetection(false),
#if (VISP_HAVE_OPENCV_VERSION >= 0x020400 && VISP_HAVE_OPENCV_VERSION < 0x030000)
    m_useBruteForceCrossCheck(true),
#endif
    m_useConsensusPercentage(false), m_useKnn(false), m_useMatchTrainToQuery(false), m_useRansacVVS(true),
    m_useSingleMatchFilter(true), m_I()
{
  initFeatureNames();

  m_detectorNames.push_back(detectorName);
  m_extractorNames.push_back(extractorName);

  init();
}

/*!
  Constructor to initialize specified detector, extractor, matcher and
  filtering method.

  \param detectorNames : List of name detector for allowing multiple
  detectors. \param extractorNames : List of name extractor for allowing
  multiple extractors. \param matcherName : Name of the matcher. \param
  filterType : Filtering matching method chosen.
 */
vpKeyPoint::vpKeyPoint(const std::vector<std::string> &detectorNames, const std::vector<std::string> &extractorNames,
                       const std::string &matcherName, const vpFilterMatchingType &filterType)
  : m_computeCovariance(false), m_covarianceMatrix(), m_currentImageId(0), m_detectionMethod(detectionScore),
    m_detectionScore(0.15), m_detectionThreshold(100.0), m_detectionTime(0.), m_detectorNames(detectorNames),
    m_detectors(), m_extractionTime(0.), m_extractorNames(extractorNames), m_extractors(), m_filteredMatches(),
    m_filterType(filterType), m_imageFormat(jpgImageFormat), m_knnMatches(), m_mapOfImageId(), m_mapOfImages(),
    m_matcher(), m_matcherName(matcherName), m_matches(), m_matchingFactorThreshold(2.0),
    m_matchingRatioThreshold(0.85), m_matchingTime(0.), m_matchRansacKeyPointsToPoints(), m_nbRansacIterations(200),
    m_nbRansacMinInlierCount(100), m_objectFilteredPoints(), m_poseTime(0.), m_queryDescriptors(),
    m_queryFilteredKeyPoints(), m_queryKeyPoints(), m_ransacConsensusPercentage(20.0), m_ransacFilterFlag(vpPose::NO_FILTER), m_ransacInliers(),
    m_ransacOutliers(), m_ransacParallel(false), m_ransacParallelNbThreads(0), m_ransacReprojectionError(6.0), m_ransacThreshold(0.01),
    m_trainDescriptors(), m_trainKeyPoints(), m_trainPoints(), m_trainVpPoints(), m_useAffineDetection(false),
#if (VISP_HAVE_OPENCV_VERSION >= 0x020400 && VISP_HAVE_OPENCV_VERSION < 0x030000)
    m_useBruteForceCrossCheck(true),
#endif
    m_useConsensusPercentage(false), m_useKnn(false), m_useMatchTrainToQuery(false), m_useRansacVVS(true),
    m_useSingleMatchFilter(true), m_I()
{
  initFeatureNames();
  init();
}

/*!
   Apply an affine and skew transformation to an image.
   \param tilt : Tilt value in the direction of x
   \param phi : Rotation value
   \param img : Modified image after the transformation
   \param mask : Mask containing the location of the image pixels after the
   transformation \param Ai : Inverse affine matrix
 */
void vpKeyPoint::affineSkew(double tilt, double phi, cv::Mat &img, cv::Mat &mask, cv::Mat &Ai)
{
  int h = img.rows;
  int w = img.cols;

  mask = cv::Mat(h, w, CV_8UC1, cv::Scalar(255));

  cv::Mat A = cv::Mat::eye(2, 3, CV_32F);

  // if (phi != 0.0) {
  if (std::fabs(phi) > std::numeric_limits<double>::epsilon()) {
    phi *= M_PI / 180.;
    double s = sin(phi);
    double c = cos(phi);

    A = (cv::Mat_<float>(2, 2) << c, -s, s, c);

    cv::Mat corners = (cv::Mat_<float>(4, 2) << 0, 0, w, 0, w, h, 0, h);
    cv::Mat tcorners = corners * A.t();
    cv::Mat tcorners_x, tcorners_y;
    tcorners.col(0).copyTo(tcorners_x);
    tcorners.col(1).copyTo(tcorners_y);
    std::vector<cv::Mat> channels;
    channels.push_back(tcorners_x);
    channels.push_back(tcorners_y);
    cv::merge(channels, tcorners);

    cv::Rect rect = cv::boundingRect(tcorners);
    A = (cv::Mat_<float>(2, 3) << c, -s, -rect.x, s, c, -rect.y);

    cv::warpAffine(img, img, A, cv::Size(rect.width, rect.height), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
  }
  // if (tilt != 1.0) {
  if (std::fabs(tilt - 1.0) > std::numeric_limits<double>::epsilon()) {
    double s = 0.8 * sqrt(tilt * tilt - 1);
    cv::GaussianBlur(img, img, cv::Size(0, 0), s, 0.01);
    cv::resize(img, img, cv::Size(0, 0), 1.0 / tilt, 1.0, cv::INTER_NEAREST);
    A.row(0) = A.row(0) / tilt;
  }
  // if (tilt != 1.0 || phi != 0.0) {
  if (std::fabs(tilt - 1.0) > std::numeric_limits<double>::epsilon() ||
      std::fabs(phi) > std::numeric_limits<double>::epsilon()) {
    h = img.rows;
    w = img.cols;
    cv::warpAffine(mask, mask, A, cv::Size(w, h), cv::INTER_NEAREST);
  }
  cv::invertAffineTransform(A, Ai);
}

/*!
   Build the reference keypoints list.

   \param I : Input reference image.
   \return The number of detected keypoints in the image \p I.
 */
unsigned int vpKeyPoint::buildReference(const vpImage<unsigned char> &I) { return buildReference(I, vpRect()); }

/*!
   Build the reference keypoints list.

   \param I_color : Input reference image.
   \return The number of detected keypoints in the image \p I.
 */
unsigned int vpKeyPoint::buildReference(const vpImage<vpRGBa> &I_color) { return buildReference(I_color, vpRect()); }

/*!
   Build the reference keypoints list in a region of interest in the image.

   \param I : Input reference image
   \param iP : Position of the top-left corner of the region of interest.
   \param height : Height of the region of interest.
   \param width : Width of the region of interest.
   \return The number of detected keypoints in the current image I.
 */
unsigned int vpKeyPoint::buildReference(const vpImage<unsigned char> &I, const vpImagePoint &iP,
                                        const unsigned int height, const unsigned int width)
{
  return buildReference(I, vpRect(iP, width, height));
}

/*!
   Build the reference keypoints list in a region of interest in the image.

   \param I_color : Input reference image
   \param iP : Position of the top-left corner of the region of interest.
   \param height : Height of the region of interest.
   \param width : Width of the region of interest.
   \return The number of detected keypoints in the current image I.
 */
unsigned int vpKeyPoint::buildReference(const vpImage<vpRGBa> &I_color, const vpImagePoint &iP,
                                        const unsigned int height, const unsigned int width)
{
  return buildReference(I_color, vpRect(iP, width, height));
}

/*!
   Build the reference keypoints list in a region of interest in the image.

   \param I : Input image.
   \param rectangle : Rectangle of the region of interest.
   \return The number of detected keypoints in the current image I.
 */
unsigned int vpKeyPoint::buildReference(const vpImage<unsigned char> &I, const vpRect &rectangle)
{
  // Reset variables used when dealing with 3D models
  // So as no 3D point list is passed, we dont need this variables
  m_trainPoints.clear();
  m_mapOfImageId.clear();
  m_mapOfImages.clear();
  m_currentImageId = 1;

  if (m_useAffineDetection) {
    std::vector<std::vector<cv::KeyPoint> > listOfTrainKeyPoints;
    std::vector<cv::Mat> listOfTrainDescriptors;

    // Detect keypoints and extract descriptors on multiple images
    detectExtractAffine(I, listOfTrainKeyPoints, listOfTrainDescriptors);

    // Flatten the different train lists
    m_trainKeyPoints.clear();
    for (std::vector<std::vector<cv::KeyPoint> >::const_iterator it = listOfTrainKeyPoints.begin();
         it != listOfTrainKeyPoints.end(); ++it) {
      m_trainKeyPoints.insert(m_trainKeyPoints.end(), it->begin(), it->end());
    }

    bool first = true;
    for (std::vector<cv::Mat>::const_iterator it = listOfTrainDescriptors.begin(); it != listOfTrainDescriptors.end();
         ++it) {
      if (first) {
        first = false;
        it->copyTo(m_trainDescriptors);
      } else {
        m_trainDescriptors.push_back(*it);
      }
    }
  } else {
    detect(I, m_trainKeyPoints, m_detectionTime, rectangle);
    extract(I, m_trainKeyPoints, m_trainDescriptors, m_extractionTime);
  }

  // Save the correspondence keypoint class_id with the training image_id in a
  // map  Used to display the matching with all the training images
  for (std::vector<cv::KeyPoint>::const_iterator it = m_trainKeyPoints.begin(); it != m_trainKeyPoints.end(); ++it) {
    m_mapOfImageId[it->class_id] = m_currentImageId;
  }

  // Save the image in a map at a specific image_id
  m_mapOfImages[m_currentImageId] = I;

  // Convert OpenCV type to ViSP type for compatibility
  vpConvert::convertFromOpenCV(m_trainKeyPoints, referenceImagePointsList);

  _reference_computed = true;

  // Add train descriptors in matcher object
  m_matcher->clear();
  m_matcher->add(std::vector<cv::Mat>(1, m_trainDescriptors));

  return static_cast<unsigned int>(m_trainKeyPoints.size());
}

/*!
   Build the reference keypoints list in a region of interest in the image.

   \param I_color : Input image.
   \param rectangle : Rectangle of the region of interest.
   \return The number of detected keypoints in the current image I.
 */
unsigned int vpKeyPoint::buildReference(const vpImage<vpRGBa> &I_color, const vpRect &rectangle)
{
  vpImageConvert::convert(I_color, m_I);
  return (buildReference(m_I, rectangle));
}

/*!
   Build the reference keypoints list and compute the 3D position
   corresponding of the keypoints locations.

   \param I : Input image
   \param trainKeyPoints : List of the train keypoints.
   \param points3f : Output list of the 3D position corresponding of the
   keypoints locations. \param append : If true, append the supply train
   keypoints with those already present. \param class_id : The class id to be
   set to the input cv::KeyPoint if != -1.
 */
void vpKeyPoint::buildReference(const vpImage<unsigned char> &I, std::vector<cv::KeyPoint> &trainKeyPoints,
                                std::vector<cv::Point3f> &points3f, const bool append, const int class_id)
{
  cv::Mat trainDescriptors;
  // Copy the input list of keypoints
  std::vector<cv::KeyPoint> trainKeyPoints_tmp = trainKeyPoints;

  extract(I, trainKeyPoints, trainDescriptors, m_extractionTime, &points3f);

  if (trainKeyPoints.size() != trainKeyPoints_tmp.size()) {
    // Keypoints have been removed
    // Store the hash of a keypoint as the key and the index of the keypoint
    // as the value
    std::map<size_t, size_t> mapOfKeypointHashes;
    size_t cpt = 0;
    for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints_tmp.begin(); it != trainKeyPoints_tmp.end();
         ++it, cpt++) {
      mapOfKeypointHashes[myKeypointHash(*it)] = cpt;
    }

    std::vector<cv::Point3f> trainPoints_tmp;
    for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
      if (mapOfKeypointHashes.find(myKeypointHash(*it)) != mapOfKeypointHashes.end()) {
        trainPoints_tmp.push_back(points3f[mapOfKeypointHashes[myKeypointHash(*it)]]);
      }
    }

    // Copy trainPoints_tmp to points3f
    points3f = trainPoints_tmp;
  }

  buildReference(I, trainKeyPoints, trainDescriptors, points3f, append, class_id);
}

/*!
   Build the reference keypoints list and compute the 3D position
   corresponding of the keypoints locations.

   \param I_color : Input image
   \param trainKeyPoints : List of the train keypoints.
   \param points3f : Output list of the 3D position corresponding of the
   keypoints locations. \param append : If true, append the supply train
   keypoints with those already present. \param class_id : The class id to be
   set to the input cv::KeyPoint if != -1.
 */
void vpKeyPoint::buildReference(const vpImage<vpRGBa> &I_color, std::vector<cv::KeyPoint> &trainKeyPoints,
                                std::vector<cv::Point3f> &points3f, const bool append, const int class_id)
{
  cv::Mat trainDescriptors;
  // Copy the input list of keypoints
  std::vector<cv::KeyPoint> trainKeyPoints_tmp = trainKeyPoints;

  extract(I_color, trainKeyPoints, trainDescriptors, m_extractionTime, &points3f);

  if (trainKeyPoints.size() != trainKeyPoints_tmp.size()) {
    // Keypoints have been removed
    // Store the hash of a keypoint as the key and the index of the keypoint
    // as the value
    std::map<size_t, size_t> mapOfKeypointHashes;
    size_t cpt = 0;
    for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints_tmp.begin(); it != trainKeyPoints_tmp.end();
         ++it, cpt++) {
      mapOfKeypointHashes[myKeypointHash(*it)] = cpt;
    }

    std::vector<cv::Point3f> trainPoints_tmp;
    for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
      if (mapOfKeypointHashes.find(myKeypointHash(*it)) != mapOfKeypointHashes.end()) {
        trainPoints_tmp.push_back(points3f[mapOfKeypointHashes[myKeypointHash(*it)]]);
      }
    }

    // Copy trainPoints_tmp to points3f
    points3f = trainPoints_tmp;
  }

  buildReference(I_color, trainKeyPoints, trainDescriptors, points3f, append, class_id);
}

/*!
   Build the reference keypoints list and compute the 3D position
   corresponding of the keypoints locations.

   \param I : Input image
   \param trainKeyPoints : List of the train keypoints.
   \param points3f : List of the 3D position corresponding of the keypoints
   locations. \param trainDescriptors : List of the train descriptors. \param
   append : If true, append the supply train keypoints with those already
   present. \param class_id : The class id to be set to the input cv::KeyPoint
   if != -1.
 */
void vpKeyPoint::buildReference(const vpImage<unsigned char> &I, const std::vector<cv::KeyPoint> &trainKeyPoints,
                                const cv::Mat &trainDescriptors, const std::vector<cv::Point3f> &points3f,
                                const bool append, const int class_id)
{
  if (!append) {
    m_currentImageId = 0;
    m_mapOfImageId.clear();
    m_mapOfImages.clear();
    this->m_trainKeyPoints.clear();
    this->m_trainPoints.clear();
  }

  m_currentImageId++;

  std::vector<cv::KeyPoint> trainKeyPoints_tmp = trainKeyPoints;
  // Set class_id if != -1
  if (class_id != -1) {
    for (std::vector<cv::KeyPoint>::iterator it = trainKeyPoints_tmp.begin(); it != trainKeyPoints_tmp.end(); ++it) {
      it->class_id = class_id;
    }
  }

  // Save the correspondence keypoint class_id with the training image_id in a
  // map  Used to display the matching with all the training images
  for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints_tmp.begin(); it != trainKeyPoints_tmp.end();
       ++it) {
    m_mapOfImageId[it->class_id] = m_currentImageId;
  }

  // Save the image in a map at a specific image_id
  m_mapOfImages[m_currentImageId] = I;

  // Append reference lists
  this->m_trainKeyPoints.insert(this->m_trainKeyPoints.end(), trainKeyPoints_tmp.begin(), trainKeyPoints_tmp.end());
  if (!append) {
    trainDescriptors.copyTo(this->m_trainDescriptors);
  } else {
    this->m_trainDescriptors.push_back(trainDescriptors);
  }
  this->m_trainPoints.insert(this->m_trainPoints.end(), points3f.begin(), points3f.end());

  // Convert OpenCV type to ViSP type for compatibility
  vpConvert::convertFromOpenCV(this->m_trainKeyPoints, referenceImagePointsList);
  vpConvert::convertFromOpenCV(this->m_trainPoints, m_trainVpPoints);

  // Add train descriptors in matcher object
  m_matcher->clear();
  m_matcher->add(std::vector<cv::Mat>(1, m_trainDescriptors));

  _reference_computed = true;
}

/*!
   Build the reference keypoints list and compute the 3D position
   corresponding of the keypoints locations.

   \param I_color : Input image
   \param trainKeyPoints : List of the train keypoints.
   \param points3f : List of the 3D position corresponding of the keypoints
   locations. \param trainDescriptors : List of the train descriptors. \param
   append : If true, append the supply train keypoints with those already
   present. \param class_id : The class id to be set to the input cv::KeyPoint
   if != -1.
 */
void vpKeyPoint::buildReference(const vpImage<vpRGBa> &I_color, const std::vector<cv::KeyPoint> &trainKeyPoints,
                                const cv::Mat &trainDescriptors, const std::vector<cv::Point3f> &points3f,
                                const bool append, const int class_id)
{
  vpImageConvert::convert(I_color, m_I);
  buildReference(m_I, trainKeyPoints, trainDescriptors, points3f, append, class_id);
}

/*!
   Compute the 3D coordinate in the world/object frame given the 2D image
   coordinate and under the assumption that the point is located on a plane
   whose the plane equation is known in the camera frame.
   The Z-coordinate is retrieved according to the proportional relationship
   between the plane equation expressed in the normalized camera frame
   (derived from the image coordinate) and the same plane equation expressed
   in the camera frame.

   \param candidate : Keypoint we want to compute the 3D coordinate.
   \param roi : List of 3D points in the camera frame representing a planar
   face. \param cam : Camera parameters. \param cMo : Homogeneous matrix
   between the world and the camera frames. \param point : 3D coordinate in
   the world/object frame computed.
 */
void vpKeyPoint::compute3D(const cv::KeyPoint &candidate, const std::vector<vpPoint> &roi,
                           const vpCameraParameters &cam, const vpHomogeneousMatrix &cMo, cv::Point3f &point)
{
  /* compute plane equation */
  std::vector<vpPoint>::const_iterator it_roi = roi.begin();
  vpPoint pts[3];
  pts[0] = *it_roi;
  ++it_roi;
  pts[1] = *it_roi;
  ++it_roi;
  pts[2] = *it_roi;
  vpPlane Po(pts[0], pts[1], pts[2]);
  double xc = 0.0, yc = 0.0;
  vpPixelMeterConversion::convertPoint(cam, candidate.pt.x, candidate.pt.y, xc, yc);
  double Z = -Po.getD() / (Po.getA() * xc + Po.getB() * yc + Po.getC());
  double X = xc * Z;
  double Y = yc * Z;
  vpColVector point_cam(4);
  point_cam[0] = X;
  point_cam[1] = Y;
  point_cam[2] = Z;
  point_cam[3] = 1;
  vpColVector point_obj(4);
  point_obj = cMo.inverse() * point_cam;
  point = cv::Point3f((float)point_obj[0], (float)point_obj[1], (float)point_obj[2]);
}

/*!
   Compute the 3D coordinate in the world/object frame given the 2D image
   coordinate and under the assumption that the point is located on a plane
   whose the plane equation is known in the camera frame.
   The Z-coordinate is retrieved according to the proportional relationship
   between the plane equation expressed in the normalized camera frame
   (derived from the image coordinate) and the same plane equation expressed
   in the camera frame.

   \param candidate : vpImagePoint we want to compute the 3D coordinate.
   \param roi : List of 3D points in the camera frame representing a planar
   face. \param cam : Camera parameters. \param cMo : Homogeneous matrix
   between the world and the camera frames. \param point : 3D coordinate in
   the world/object frame computed.
 */
void vpKeyPoint::compute3D(const vpImagePoint &candidate, const std::vector<vpPoint> &roi,
                           const vpCameraParameters &cam, const vpHomogeneousMatrix &cMo, vpPoint &point)
{
  /* compute plane equation */
  std::vector<vpPoint>::const_iterator it_roi = roi.begin();
  vpPoint pts[3];
  pts[0] = *it_roi;
  ++it_roi;
  pts[1] = *it_roi;
  ++it_roi;
  pts[2] = *it_roi;
  vpPlane Po(pts[0], pts[1], pts[2]);
  double xc = 0.0, yc = 0.0;
  vpPixelMeterConversion::convertPoint(cam, candidate, xc, yc);
  double Z = -Po.getD() / (Po.getA() * xc + Po.getB() * yc + Po.getC());
  double X = xc * Z;
  double Y = yc * Z;
  vpColVector point_cam(4);
  point_cam[0] = X;
  point_cam[1] = Y;
  point_cam[2] = Z;
  point_cam[3] = 1;
  vpColVector point_obj(4);
  point_obj = cMo.inverse() * point_cam;
  point.setWorldCoordinates(point_obj);
}

/*!
   Keep only keypoints located on faces and compute for those keypoints the 3D
   coordinate in the world/object frame given the 2D image coordinate and
   under the assumption that the point is located on a plane.

   \param cMo : Homogeneous matrix between the world and the camera frames.
   \param cam : Camera parameters.
   \param candidates : In input, list of keypoints detected in the whole
   image, in output, list of keypoints only located on planes. \param polygons
   : List of 2D polygons representing the projection of the faces in the image
   plane. \param  roisPt : List of faces, with the 3D coordinates known in the
   camera frame. \param points : Output list of computed 3D coordinates (in
   the world/object frame) of keypoints located only on faces. \param
   descriptors : Optional parameter, pointer to the descriptors to filter.
 */
void vpKeyPoint::compute3DForPointsInPolygons(const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                                              std::vector<cv::KeyPoint> &candidates,
                                              const std::vector<vpPolygon> &polygons,
                                              const std::vector<std::vector<vpPoint> > &roisPt,
                                              std::vector<cv::Point3f> &points, cv::Mat *descriptors)
{

  std::vector<cv::KeyPoint> candidatesToCheck = candidates;
  candidates.clear();
  points.clear();
  vpImagePoint imPt;
  cv::Point3f pt;
  cv::Mat desc;

  std::vector<std::pair<cv::KeyPoint, size_t> > pairOfCandidatesToCheck(candidatesToCheck.size());
  for (size_t i = 0; i < candidatesToCheck.size(); i++) {
    pairOfCandidatesToCheck[i] = std::pair<cv::KeyPoint, size_t>(candidatesToCheck[i], i);
  }

  size_t cpt1 = 0;
  std::vector<vpPolygon> polygons_tmp = polygons;
  for (std::vector<vpPolygon>::iterator it1 = polygons_tmp.begin(); it1 != polygons_tmp.end(); ++it1, cpt1++) {
    std::vector<std::pair<cv::KeyPoint, size_t> >::iterator it2 = pairOfCandidatesToCheck.begin();

    while (it2 != pairOfCandidatesToCheck.end()) {
      imPt.set_ij(it2->first.pt.y, it2->first.pt.x);
      if (it1->isInside(imPt)) {
        candidates.push_back(it2->first);
        vpKeyPoint::compute3D(it2->first, roisPt[cpt1], cam, cMo, pt);
        points.push_back(pt);

        if (descriptors != NULL) {
          desc.push_back(descriptors->row((int)it2->second));
        }

        // Remove candidate keypoint which is located on the current polygon
        it2 = pairOfCandidatesToCheck.erase(it2);
      } else {
        ++it2;
      }
    }
  }

  if (descriptors != NULL) {
    desc.copyTo(*descriptors);
  }
}

/*!
   Keep only keypoints located on faces and compute for those keypoints the 3D
   coordinate in the world/object frame given the 2D image coordinate and
   under the assumption that the point is located on a plane.

   \param cMo : Homogeneous matrix between the world and the camera frames.
   \param cam : Camera parameters.
   \param candidates : In input, list of vpImagePoint located in the whole
   image, in output, list of vpImagePoint only located on planes. \param
   polygons : List of 2D polygons representing the projection of the faces in
   the image plane. \param  roisPt : List of faces, with the 3D coordinates
   known in the camera frame. \param points : Output list of computed 3D
   coordinates (in the world/object frame) of vpImagePoint located only on
   faces. \param descriptors : Optional parameter, pointer to the descriptors
   to filter
 */
void vpKeyPoint::compute3DForPointsInPolygons(const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                                              std::vector<vpImagePoint> &candidates,
                                              const std::vector<vpPolygon> &polygons,
                                              const std::vector<std::vector<vpPoint> > &roisPt,
                                              std::vector<vpPoint> &points, cv::Mat *descriptors)
{

  std::vector<vpImagePoint> candidatesToCheck = candidates;
  candidates.clear();
  points.clear();
  vpPoint pt;
  cv::Mat desc;

  std::vector<std::pair<vpImagePoint, size_t> > pairOfCandidatesToCheck(candidatesToCheck.size());
  for (size_t i = 0; i < candidatesToCheck.size(); i++) {
    pairOfCandidatesToCheck[i] = std::pair<vpImagePoint, size_t>(candidatesToCheck[i], i);
  }

  size_t cpt1 = 0;
  std::vector<vpPolygon> polygons_tmp = polygons;
  for (std::vector<vpPolygon>::iterator it1 = polygons_tmp.begin(); it1 != polygons_tmp.end(); ++it1, cpt1++) {
    std::vector<std::pair<vpImagePoint, size_t> >::iterator it2 = pairOfCandidatesToCheck.begin();

    while (it2 != pairOfCandidatesToCheck.end()) {
      if (it1->isInside(it2->first)) {
        candidates.push_back(it2->first);
        vpKeyPoint::compute3D(it2->first, roisPt[cpt1], cam, cMo, pt);
        points.push_back(pt);

        if (descriptors != NULL) {
          desc.push_back(descriptors->row((int)it2->second));
        }

        // Remove candidate keypoint which is located on the current polygon
        it2 = pairOfCandidatesToCheck.erase(it2);
      } else {
        ++it2;
      }
    }
  }
}

/*!
   Keep only keypoints located on cylinders and compute the 3D coordinates in
   the world/object frame given the 2D image coordinates.

   \param cMo : Homogeneous matrix between the world and the camera frames.
   \param cam : Camera parameters.
   \param candidates : In input, list of keypoints detected in the whole
   image, in output, list of keypoints only located on cylinders. \param
   cylinders : List of vpCylinder corresponding of the cylinder objects in the
   scene, projected in the camera frame. \param vectorOfCylinderRois : For
   each cylinder, the corresponding list of bounding box. \param points :
   Output list of computed 3D coordinates in the world/object frame for each
   keypoint located on a cylinder. \param descriptors : Optional parameter,
   pointer to the descriptors to filter.
 */
void vpKeyPoint::compute3DForPointsOnCylinders(
    const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam, std::vector<cv::KeyPoint> &candidates,
    const std::vector<vpCylinder> &cylinders,
    const std::vector<std::vector<std::vector<vpImagePoint> > > &vectorOfCylinderRois, std::vector<cv::Point3f> &points,
    cv::Mat *descriptors)
{
  std::vector<cv::KeyPoint> candidatesToCheck = candidates;
  candidates.clear();
  points.clear();
  cv::Mat desc;

  // Keep only keypoints on cylinders
  size_t cpt_keypoint = 0;
  for (std::vector<cv::KeyPoint>::const_iterator it1 = candidatesToCheck.begin(); it1 != candidatesToCheck.end();
       ++it1, cpt_keypoint++) {
    size_t cpt_cylinder = 0;

    // Iterate through the list of vpCylinders
    for (std::vector<std::vector<std::vector<vpImagePoint> > >::const_iterator it2 = vectorOfCylinderRois.begin();
         it2 != vectorOfCylinderRois.end(); ++it2, cpt_cylinder++) {
      // Iterate through the list of the bounding boxes of the current
      // vpCylinder
      for (std::vector<std::vector<vpImagePoint> >::const_iterator it3 = it2->begin(); it3 != it2->end(); ++it3) {
        if (vpPolygon::isInside(*it3, it1->pt.y, it1->pt.x)) {
          candidates.push_back(*it1);

          // Calculate the 3D coordinates for each keypoint located on
          // cylinders
          double xm = 0.0, ym = 0.0;
          vpPixelMeterConversion::convertPoint(cam, it1->pt.x, it1->pt.y, xm, ym);
          double Z = cylinders[cpt_cylinder].computeZ(xm, ym);

          if (!vpMath::isNaN(Z) && Z > std::numeric_limits<double>::epsilon()) {
            vpColVector point_cam(4);
            point_cam[0] = xm * Z;
            point_cam[1] = ym * Z;
            point_cam[2] = Z;
            point_cam[3] = 1;
            vpColVector point_obj(4);
            point_obj = cMo.inverse() * point_cam;
            vpPoint pt;
            pt.setWorldCoordinates(point_obj);
            points.push_back(cv::Point3f((float)pt.get_oX(), (float)pt.get_oY(), (float)pt.get_oZ()));

            if (descriptors != NULL) {
              desc.push_back(descriptors->row((int)cpt_keypoint));
            }

            break;
          }
        }
      }
    }
  }

  if (descriptors != NULL) {
    desc.copyTo(*descriptors);
  }
}

/*!
   Keep only vpImagePoint located on cylinders and compute the 3D coordinates
   in the world/object frame given the 2D image coordinates.

   \param cMo : Homogeneous matrix between the world and the camera frames.
   \param cam : Camera parameters.
   \param candidates : In input, list of vpImagePoint located in the image, in
   output, list of vpImagePoint only located on cylinders. \param cylinders :
   List of vpCylinder corresponding of the cylinder objects in the scene,
   projected in the camera frame. \param vectorOfCylinderRois : For each
   cylinder, the corresponding list of bounding box. \param points : Output
   list of computed 3D coordinates in the world/object frame for each
   vpImagePoint located on a cylinder. \param descriptors : Optional
   parameter, pointer to the descriptors to filter.
 */
void vpKeyPoint::compute3DForPointsOnCylinders(
    const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam, std::vector<vpImagePoint> &candidates,
    const std::vector<vpCylinder> &cylinders,
    const std::vector<std::vector<std::vector<vpImagePoint> > > &vectorOfCylinderRois, std::vector<vpPoint> &points,
    cv::Mat *descriptors)
{
  std::vector<vpImagePoint> candidatesToCheck = candidates;
  candidates.clear();
  points.clear();
  cv::Mat desc;

  // Keep only keypoints on cylinders
  size_t cpt_keypoint = 0;
  for (std::vector<vpImagePoint>::const_iterator it1 = candidatesToCheck.begin(); it1 != candidatesToCheck.end();
       ++it1, cpt_keypoint++) {
    size_t cpt_cylinder = 0;

    // Iterate through the list of vpCylinders
    for (std::vector<std::vector<std::vector<vpImagePoint> > >::const_iterator it2 = vectorOfCylinderRois.begin();
         it2 != vectorOfCylinderRois.end(); ++it2, cpt_cylinder++) {
      // Iterate through the list of the bounding boxes of the current
      // vpCylinder
      for (std::vector<std::vector<vpImagePoint> >::const_iterator it3 = it2->begin(); it3 != it2->end(); ++it3) {
        if (vpPolygon::isInside(*it3, it1->get_i(), it1->get_j())) {
          candidates.push_back(*it1);

          // Calculate the 3D coordinates for each keypoint located on
          // cylinders
          double xm = 0.0, ym = 0.0;
          vpPixelMeterConversion::convertPoint(cam, it1->get_u(), it1->get_v(), xm, ym);
          double Z = cylinders[cpt_cylinder].computeZ(xm, ym);

          if (!vpMath::isNaN(Z) && Z > std::numeric_limits<double>::epsilon()) {
            vpColVector point_cam(4);
            point_cam[0] = xm * Z;
            point_cam[1] = ym * Z;
            point_cam[2] = Z;
            point_cam[3] = 1;
            vpColVector point_obj(4);
            point_obj = cMo.inverse() * point_cam;
            vpPoint pt;
            pt.setWorldCoordinates(point_obj);
            points.push_back(pt);

            if (descriptors != NULL) {
              desc.push_back(descriptors->row((int)cpt_keypoint));
            }

            break;
          }
        }
      }
    }
  }

  if (descriptors != NULL) {
    desc.copyTo(*descriptors);
  }
}

/*!
   Compute the pose using the correspondence between 2D points and 3D points
   using OpenCV function with RANSAC method.

   \param imagePoints : List of 2D points corresponding to the location of the detected keypoints.
   \param  objectPoints : List of the 3D points in the object frame matched.
   \param cam : Camera parameters.
   \param cMo : Homogeneous matrix between the object frame and the camera frame.
   \param inlierIndex : List of indexes of inliers.
   \param elapsedTime : Elapsed time.
   \param func : Function pointer to filter the final pose returned by OpenCV pose estimation method.
   \return True if the pose has been computed, false otherwise (not enough points, or size list mismatch).
 */
bool vpKeyPoint::computePose(const std::vector<cv::Point2f> &imagePoints, const std::vector<cv::Point3f> &objectPoints,
                             const vpCameraParameters &cam, vpHomogeneousMatrix &cMo, std::vector<int> &inlierIndex,
                             double &elapsedTime, bool (*func)(const vpHomogeneousMatrix &))
{
  double t = vpTime::measureTimeMs();

  if (imagePoints.size() < 4 || objectPoints.size() < 4 || imagePoints.size() != objectPoints.size()) {
    elapsedTime = (vpTime::measureTimeMs() - t);
    std::cerr << "Not enough points to compute the pose (at least 4 points "
                 "are needed)."
              << std::endl;

    return false;
  }

  cv::Mat cameraMatrix =
      (cv::Mat_<double>(3, 3) << cam.get_px(), 0, cam.get_u0(), 0, cam.get_py(), cam.get_v0(), 0, 0, 1);
  cv::Mat rvec, tvec;

  // Bug with OpenCV < 2.4.0 when zero distorsion is provided by an empty
  // array.  http://code.opencv.org/issues/1700 ;
  // http://code.opencv.org/issues/1718  what(): Distortion coefficients must
  // be 1x4, 4x1, 1x5, 5x1, 1x8 or 8x1 floating-point vector in function
  // cvProjectPoints2  Fixed in OpenCV 2.4.0 (r7558)
  //  cv::Mat distCoeffs;
  cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_64F);

  try {
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
    // OpenCV 3.0.0 (2014/12/12)
    cv::solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, m_nbRansacIterations,
                       (float)m_ransacReprojectionError,
                       0.99, // confidence=0.99 (default) – The probability
                             // that the algorithm produces a useful result.
                       inlierIndex, cv::SOLVEPNP_ITERATIVE);
// SOLVEPNP_ITERATIVE (default): Iterative method is based on
// Levenberg-Marquardt optimization.  In this case the function finds such a
// pose that minimizes reprojection error, that is the sum of squared
// distances between the observed projections imagePoints and the projected
// (using projectPoints() ) objectPoints .  SOLVEPNP_P3P: Method is based on
// the paper of X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang “Complete Solution
// Classification  for the Perspective-Three-Point Problem”. In this case the
// function requires exactly four object and image points.  SOLVEPNP_EPNP:
// Method has been introduced by F.Moreno-Noguer, V.Lepetit and P.Fua in the
// paper “EPnP: Efficient  Perspective-n-Point Camera Pose Estimation”.
// SOLVEPNP_DLS: Method is based on the paper of Joel A. Hesch and Stergios I.
// Roumeliotis. “A Direct Least-Squares (DLS)  Method for PnP”.  SOLVEPNP_UPNP
// Method is based on the paper of A.Penate-Sanchez, J.Andrade-Cetto,
// F.Moreno-Noguer. “Exhaustive Linearization for Robust Camera Pose and Focal
// Length Estimation”. In this case the function also  estimates the
// parameters
// f_x and f_y assuming that both have the same value. Then the cameraMatrix
// is updated with the  estimated focal length.
#else
    int nbInlierToReachConsensus = m_nbRansacMinInlierCount;
    if (m_useConsensusPercentage) {
      nbInlierToReachConsensus = (int)(m_ransacConsensusPercentage / 100.0 * (double)m_queryFilteredKeyPoints.size());
    }

    cv::solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, m_nbRansacIterations,
                       (float)m_ransacReprojectionError, nbInlierToReachConsensus, inlierIndex);
#endif
  } catch (cv::Exception &e) {
    std::cerr << e.what() << std::endl;
    elapsedTime = (vpTime::measureTimeMs() - t);
    return false;
  }
  vpTranslationVector translationVec(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
  vpThetaUVector thetaUVector(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
  cMo = vpHomogeneousMatrix(translationVec, thetaUVector);

  if (func != NULL) {
    // Check the final pose returned by solvePnPRansac to discard
    // solutions which do not respect the pose criterion.
    if (!func(cMo)) {
      elapsedTime = (vpTime::measureTimeMs() - t);
      return false;
    }
  }

  elapsedTime = (vpTime::measureTimeMs() - t);
  return true;
}

/*!
   Compute the pose using the correspondence between 2D points and 3D points
   using ViSP function with RANSAC method.

   \param objectVpPoints : List of vpPoint with coordinates expressed in the object and in the camera frame.
   \param cMo : Homogeneous matrix between the object frame and the camera frame.
   \param inliers : List of inliers.
   \param elapsedTime : Elapsed time.
   \param func : Function pointer to filter the pose in Ransac pose estimation, if we want
   to eliminate the poses which do not respect some criterion
   \return True if the pose has been computed, false otherwise (not enough points, or size list mismatch).
 */
bool vpKeyPoint::computePose(const std::vector<vpPoint> &objectVpPoints, vpHomogeneousMatrix &cMo,
                             std::vector<vpPoint> &inliers, double &elapsedTime, bool (*func)(const vpHomogeneousMatrix &))
{
  std::vector<unsigned int> inlierIndex;
  return computePose(objectVpPoints, cMo, inliers, inlierIndex, elapsedTime, func);
}

/*!
   Compute the pose using the correspondence between 2D points and 3D points
   using ViSP function with RANSAC method.

   \param objectVpPoints : List of vpPoint with coordinates expressed in the object and in the camera frame.
   \param cMo : Homogeneous matrix between the object frame and the camera frame.
   \param inliers : List of inlier points.
   \param inlierIndex : List of inlier index.
   \param elapsedTime : Elapsed time.
   \return True if the pose has been computed, false otherwise (not enough points, or size list mismatch).
   \param func : Function pointer to filter  the pose in Ransac pose estimation, if we want to eliminate the poses which
   do not respect some criterion
 */
bool vpKeyPoint::computePose(const std::vector<vpPoint> &objectVpPoints, vpHomogeneousMatrix &cMo,
                             std::vector<vpPoint> &inliers, std::vector<unsigned int> &inlierIndex, double &elapsedTime,
                             bool (*func)(const vpHomogeneousMatrix &))
{
  double t = vpTime::measureTimeMs();

  if (objectVpPoints.size() < 4) {
    elapsedTime = (vpTime::measureTimeMs() - t);
    //    std::cerr << "Not enough points to compute the pose (at least 4
    //    points are needed)." << std::endl;

    return false;
  }

  vpPose pose;

  for (std::vector<vpPoint>::const_iterator it = objectVpPoints.begin(); it != objectVpPoints.end(); ++it) {
    pose.addPoint(*it);
  }

  unsigned int nbInlierToReachConsensus = (unsigned int)m_nbRansacMinInlierCount;
  if (m_useConsensusPercentage) {
    nbInlierToReachConsensus =
        (unsigned int)(m_ransacConsensusPercentage / 100.0 * (double)m_queryFilteredKeyPoints.size());
  }

  pose.setRansacFilterFlag(m_ransacFilterFlag);
  pose.setUseParallelRansac(m_ransacParallel);
  pose.setNbParallelRansacThreads(m_ransacParallelNbThreads);
  pose.setRansacNbInliersToReachConsensus(nbInlierToReachConsensus);
  pose.setRansacThreshold(m_ransacThreshold);
  pose.setRansacMaxTrials(m_nbRansacIterations);

  bool isRansacPoseEstimationOk = false;
  try {
    pose.setCovarianceComputation(m_computeCovariance);
    isRansacPoseEstimationOk = pose.computePose(vpPose::RANSAC, cMo, func);
    inliers = pose.getRansacInliers();
    inlierIndex = pose.getRansacInlierIndex();

    if (m_computeCovariance) {
      m_covarianceMatrix = pose.getCovarianceMatrix();
    }
  } catch (const vpException &e) {
    std::cerr << "e=" << e.what() << std::endl;
    elapsedTime = (vpTime::measureTimeMs() - t);
    return false;
  }

  //  if(func != NULL && isRansacPoseEstimationOk) {
  //    //Check the final pose returned by the Ransac VVS pose estimation as
  //    in rare some cases
  //    //we can converge toward a final cMo that does not respect the pose
  //    criterion even
  //    //if the 4 minimal points picked to respect the pose criterion.
  //    if(!func(&cMo)) {
  //      elapsedTime = (vpTime::measureTimeMs() - t);
  //      return false;
  //    }
  //  }

  elapsedTime = (vpTime::measureTimeMs() - t);
  return isRansacPoseEstimationOk;
}

/*!
   Compute the pose estimation error, the mean square error (in pixel) between
   the location of the detected keypoints and the location of the projection
   of the 3D model with the estimated pose.

   \param matchKeyPoints : List of pairs between the detected keypoints and
   the corresponding 3D points. \param cam : Camera parameters. \param cMo_est
   : Estimated pose of the object.

   \return The mean square error (in pixel) between the location of the
   detected keypoints and the location of the projection of the 3D model with
   the estimated pose.
 */
double vpKeyPoint::computePoseEstimationError(const std::vector<std::pair<cv::KeyPoint, cv::Point3f> > &matchKeyPoints,
                                              const vpCameraParameters &cam, const vpHomogeneousMatrix &cMo_est)
{
  if (matchKeyPoints.size() == 0) {
    // return std::numeric_limits<double>::max(); // create an error under
    // Windows. To fix it we have to add #undef max
    return DBL_MAX;
  }

  std::vector<double> errors(matchKeyPoints.size());
  size_t cpt = 0;
  vpPoint pt;
  for (std::vector<std::pair<cv::KeyPoint, cv::Point3f> >::const_iterator it = matchKeyPoints.begin();
       it != matchKeyPoints.end(); ++it, cpt++) {
    pt.set_oX(it->second.x);
    pt.set_oY(it->second.y);
    pt.set_oZ(it->second.z);
    pt.project(cMo_est);
    double u = 0.0, v = 0.0;
    vpMeterPixelConversion::convertPoint(cam, pt.get_x(), pt.get_y(), u, v);
    errors[cpt] = std::sqrt((u - it->first.pt.x) * (u - it->first.pt.x) + (v - it->first.pt.y) * (v - it->first.pt.y));
  }

  return std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
}

/*!
   Initialize the size of the matching image (case with a matching side by
   side between IRef and ICurrent).

   \param IRef : Reference image.
   \param ICurrent : Current image.
   \param IMatching : Image matching.
 */
void vpKeyPoint::createImageMatching(vpImage<unsigned char> &IRef, vpImage<unsigned char> &ICurrent,
                                     vpImage<unsigned char> &IMatching)
{
  // Image matching side by side
  unsigned int width = IRef.getWidth() + ICurrent.getWidth();
  unsigned int height = ((std::max))(IRef.getHeight(), ICurrent.getHeight());

  IMatching = vpImage<unsigned char>(height, width);
}

/*!
   Initialize the size of the matching image (case with a matching side by
   side between IRef and ICurrent).

   \param IRef : Reference image.
   \param ICurrent : Current image.
   \param IMatching : Image matching.
 */
void vpKeyPoint::createImageMatching(vpImage<unsigned char> &IRef, vpImage<vpRGBa> &ICurrent,
                                     vpImage<vpRGBa> &IMatching)
{
  // Image matching side by side
  unsigned int width = IRef.getWidth() + ICurrent.getWidth();
  unsigned int height = ((std::max))(IRef.getHeight(), ICurrent.getHeight());

  IMatching = vpImage<vpRGBa>(height, width);
}

/*!
   Initialize the size of the matching image with appropriate size according
   to the number of training images. Used to display the matching of keypoints
   detected in the current image with those detected in multiple training
   images.

   \param ICurrent : Current image.
   \param IMatching : Image initialized with appropriate size.
 */
void vpKeyPoint::createImageMatching(vpImage<unsigned char> &ICurrent, vpImage<unsigned char> &IMatching)
{
  // Nb images in the training database + the current image we want to detect
  // the object
  unsigned int nbImg = (unsigned int)(m_mapOfImages.size() + 1);

  if (m_mapOfImages.empty()) {
    std::cerr << "There is no training image loaded !" << std::endl;
    return;
  }

  if (nbImg == 2) {
    // Only one training image, so we display them side by side
    createImageMatching(m_mapOfImages.begin()->second, ICurrent, IMatching);
  } else {
    // Multiple training images, display them as a mosaic image
    //(unsigned int) std::floor(std::sqrt((double) nbImg) + 0.5);
    unsigned int nbImgSqrt = (unsigned int)vpMath::round(std::sqrt((double)nbImg));

    // Number of columns in the mosaic grid
    unsigned int nbWidth = nbImgSqrt;
    // Number of rows in the mosaic grid
    unsigned int nbHeight = nbImgSqrt;

    // Deals with non square mosaic grid and the total number of images
    if (nbImgSqrt * nbImgSqrt < nbImg) {
      nbWidth++;
    }

    unsigned int maxW = ICurrent.getWidth();
    unsigned int maxH = ICurrent.getHeight();
    for (std::map<int, vpImage<unsigned char> >::const_iterator it = m_mapOfImages.begin(); it != m_mapOfImages.end();
         ++it) {
      if (maxW < it->second.getWidth()) {
        maxW = it->second.getWidth();
      }

      if (maxH < it->second.getHeight()) {
        maxH = it->second.getHeight();
      }
    }

    IMatching = vpImage<unsigned char>(maxH * nbHeight, maxW * nbWidth);
  }
}

/*!
   Initialize the size of the matching image with appropriate size according
   to the number of training images. Used to display the matching of keypoints
   detected in the current image with those detected in multiple training
   images.

   \param ICurrent : Current image.
   \param IMatching : Image initialized with appropriate size.
 */
void vpKeyPoint::createImageMatching(vpImage<vpRGBa> &ICurrent, vpImage<vpRGBa> &IMatching)
{
  // Nb images in the training database + the current image we want to detect
  // the object
  unsigned int nbImg = (unsigned int)(m_mapOfImages.size() + 1);

  if (m_mapOfImages.empty()) {
    std::cerr << "There is no training image loaded !" << std::endl;
    return;
  }

  if (nbImg == 2) {
    // Only one training image, so we display them side by side
    createImageMatching(m_mapOfImages.begin()->second, ICurrent, IMatching);
  } else {
    // Multiple training images, display them as a mosaic image
    //(unsigned int) std::floor(std::sqrt((double) nbImg) + 0.5);
    unsigned int nbImgSqrt = (unsigned int)vpMath::round(std::sqrt((double)nbImg));

    // Number of columns in the mosaic grid
    unsigned int nbWidth = nbImgSqrt;
    // Number of rows in the mosaic grid
    unsigned int nbHeight = nbImgSqrt;

    // Deals with non square mosaic grid and the total number of images
    if (nbImgSqrt * nbImgSqrt < nbImg) {
      nbWidth++;
    }

    unsigned int maxW = ICurrent.getWidth();
    unsigned int maxH = ICurrent.getHeight();
    for (std::map<int, vpImage<unsigned char> >::const_iterator it = m_mapOfImages.begin(); it != m_mapOfImages.end();
         ++it) {
      if (maxW < it->second.getWidth()) {
        maxW = it->second.getWidth();
      }

      if (maxH < it->second.getHeight()) {
        maxH = it->second.getHeight();
      }
    }

    IMatching = vpImage<vpRGBa>(maxH * nbHeight, maxW * nbWidth);
  }
}

/*!
   Detect keypoints in the image.

   \param I : Input image.
   \param keyPoints : Output list of the detected keypoints.
   \param rectangle : Optional rectangle of the region of interest.
 */
void vpKeyPoint::detect(const vpImage<unsigned char> &I, std::vector<cv::KeyPoint> &keyPoints, const vpRect &rectangle)
{
  double elapsedTime;
  detect(I, keyPoints, elapsedTime, rectangle);
}

/*!
   Detect keypoints in the image.

   \param I_color : Input image.
   \param keyPoints : Output list of the detected keypoints.
   \param rectangle : Optional rectangle of the region of interest.
 */
void vpKeyPoint::detect(const vpImage<vpRGBa> &I_color, std::vector<cv::KeyPoint> &keyPoints, const vpRect &rectangle)
{
  double elapsedTime;
  detect(I_color, keyPoints, elapsedTime, rectangle);
}

/*!
   Detect keypoints in the image.

   \param matImg : Input image.
   \param keyPoints : Output list of the detected keypoints.
   \param mask : Optional 8-bit integer mask to detect only where mask[i][j]
   != 0.
 */
void vpKeyPoint::detect(const cv::Mat &matImg, std::vector<cv::KeyPoint> &keyPoints, const cv::Mat &mask)
{
  double elapsedTime;
  detect(matImg, keyPoints, elapsedTime, mask);
}

/*!
   Detect keypoints in the image.

   \param I : Input image.
   \param keyPoints : Output list of the detected keypoints.
   \param elapsedTime : Elapsed time.
   \param rectangle : Optional rectangle of the region of interest.
 */
void vpKeyPoint::detect(const vpImage<unsigned char> &I, std::vector<cv::KeyPoint> &keyPoints, double &elapsedTime,
                        const vpRect &rectangle)
{
  cv::Mat matImg;
  vpImageConvert::convert(I, matImg, false);
  cv::Mat mask = cv::Mat::zeros(matImg.rows, matImg.cols, CV_8U);

  if (rectangle.getWidth() > 0 && rectangle.getHeight() > 0) {
    cv::Point leftTop((int)rectangle.getLeft(), (int)rectangle.getTop()),
        rightBottom((int)rectangle.getRight(), (int)rectangle.getBottom());
    cv::rectangle(mask, leftTop, rightBottom, cv::Scalar(255), CV_FILLED);
  } else {
    mask = cv::Mat::ones(matImg.rows, matImg.cols, CV_8U) * 255;
  }

  detect(matImg, keyPoints, elapsedTime, mask);
}

/*!
   Detect keypoints in the image.

   \param I_color : Input image.
   \param keyPoints : Output list of the detected keypoints.
   \param elapsedTime : Elapsed time.
   \param rectangle : Optional rectangle of the region of interest.
 */
void vpKeyPoint::detect(const vpImage<vpRGBa> &I_color, std::vector<cv::KeyPoint> &keyPoints, double &elapsedTime,
                        const vpRect &rectangle)
{
  cv::Mat matImg;
  vpImageConvert::convert(I_color, matImg);
  cv::Mat mask = cv::Mat::zeros(matImg.rows, matImg.cols, CV_8U);

  if (rectangle.getWidth() > 0 && rectangle.getHeight() > 0) {
    cv::Point leftTop((int)rectangle.getLeft(), (int)rectangle.getTop()),
        rightBottom((int)rectangle.getRight(), (int)rectangle.getBottom());
    cv::rectangle(mask, leftTop, rightBottom, cv::Scalar(255), CV_FILLED);
  } else {
    mask = cv::Mat::ones(matImg.rows, matImg.cols, CV_8U) * 255;
  }

  detect(matImg, keyPoints, elapsedTime, mask);
}

/*!
   Detect keypoints in the image.

   \param matImg : Input image.
   \param keyPoints : Output list of the detected keypoints.
   \param elapsedTime : Elapsed time.
   \param mask : Optional 8-bit integer mask to detect only where mask[i][j]
   != 0.
 */
void vpKeyPoint::detect(const cv::Mat &matImg, std::vector<cv::KeyPoint> &keyPoints, double &elapsedTime,
                        const cv::Mat &mask)
{
  double t = vpTime::measureTimeMs();
  keyPoints.clear();

  for (std::map<std::string, cv::Ptr<cv::FeatureDetector> >::const_iterator it = m_detectors.begin();
       it != m_detectors.end(); ++it) {
    std::vector<cv::KeyPoint> kp;
    it->second->detect(matImg, kp, mask);
    keyPoints.insert(keyPoints.end(), kp.begin(), kp.end());
  }

  elapsedTime = vpTime::measureTimeMs() - t;
}

/*!
   Display the reference and the detected keypoints in the images.

   \param IRef : Input reference image.
   \param ICurrent : Input current image.
   \param size : Size of the displayed cross.
 */
void vpKeyPoint::display(const vpImage<unsigned char> &IRef, const vpImage<unsigned char> &ICurrent, unsigned int size)
{
  std::vector<vpImagePoint> vpQueryImageKeyPoints;
  getQueryKeyPoints(vpQueryImageKeyPoints);
  std::vector<vpImagePoint> vpTrainImageKeyPoints;
  getTrainKeyPoints(vpTrainImageKeyPoints);

  for (std::vector<cv::DMatch>::const_iterator it = m_filteredMatches.begin(); it != m_filteredMatches.end(); ++it) {
    vpDisplay::displayCross(IRef, vpTrainImageKeyPoints[(size_t)(it->trainIdx)], size, vpColor::red);
    vpDisplay::displayCross(ICurrent, vpQueryImageKeyPoints[(size_t)(it->queryIdx)], size, vpColor::green);
  }
}

/*!
   Display the reference and the detected keypoints in the images.

   \param IRef : Input reference image.
   \param ICurrent : Input current image.
   \param size : Size of the displayed cross.
 */
void vpKeyPoint::display(const vpImage<vpRGBa> &IRef, const vpImage<vpRGBa> &ICurrent, unsigned int size)
{
  std::vector<vpImagePoint> vpQueryImageKeyPoints;
  getQueryKeyPoints(vpQueryImageKeyPoints);
  std::vector<vpImagePoint> vpTrainImageKeyPoints;
  getTrainKeyPoints(vpTrainImageKeyPoints);

  for (std::vector<cv::DMatch>::const_iterator it = m_filteredMatches.begin(); it != m_filteredMatches.end(); ++it) {
    vpDisplay::displayCross(IRef, vpTrainImageKeyPoints[(size_t)(it->trainIdx)], size, vpColor::red);
    vpDisplay::displayCross(ICurrent, vpQueryImageKeyPoints[(size_t)(it->queryIdx)], size, vpColor::green);
  }
}

/*!
   Display the reference keypoints.

   \param ICurrent : Input current image.
   \param size : Size of the displayed crosses.
   \param color : Color of the crosses.
 */
void vpKeyPoint::display(const vpImage<unsigned char> &ICurrent, unsigned int size, const vpColor &color)
{
  std::vector<vpImagePoint> vpQueryImageKeyPoints;
  getQueryKeyPoints(vpQueryImageKeyPoints);

  for (std::vector<cv::DMatch>::const_iterator it = m_filteredMatches.begin(); it != m_filteredMatches.end(); ++it) {
    vpDisplay::displayCross(ICurrent, vpQueryImageKeyPoints[(size_t)(it->queryIdx)], size, color);
  }
}

/*!
   Display the reference keypoints.

   \param ICurrent : Input current image.
   \param size : Size of the displayed crosses.
   \param color : Color of the crosses.
 */
void vpKeyPoint::display(const vpImage<vpRGBa> &ICurrent, unsigned int size, const vpColor &color)
{
  std::vector<vpImagePoint> vpQueryImageKeyPoints;
  getQueryKeyPoints(vpQueryImageKeyPoints);

  for (std::vector<cv::DMatch>::const_iterator it = m_filteredMatches.begin(); it != m_filteredMatches.end(); ++it) {
    vpDisplay::displayCross(ICurrent, vpQueryImageKeyPoints[(size_t)(it->queryIdx)], size, color);
  }
}

/*!
  Display the matching lines between the detected keypoints with those
  detected in one training image.

  \param IRef : Reference image, used to have the x-offset.
  \param IMatching : Resulting image matching.
  \param crossSize : Size of the displayed crosses.
  \param lineThickness : Thickness of the displayed lines.
  \param color : Color to use, if none, we pick randomly a color for each pair
  of matching.
 */
void vpKeyPoint::displayMatching(const vpImage<unsigned char> &IRef, vpImage<unsigned char> &IMatching,
                                 unsigned int crossSize, unsigned int lineThickness, const vpColor &color)
{
  bool randomColor = (color == vpColor::none);
  srand((unsigned int)time(NULL));
  vpColor currentColor = color;

  std::vector<vpImagePoint> queryImageKeyPoints;
  getQueryKeyPoints(queryImageKeyPoints);
  std::vector<vpImagePoint> trainImageKeyPoints;
  getTrainKeyPoints(trainImageKeyPoints);

  vpImagePoint leftPt, rightPt;
  for (std::vector<cv::DMatch>::const_iterator it = m_filteredMatches.begin(); it != m_filteredMatches.end(); ++it) {
    if (randomColor) {
      currentColor = vpColor((rand() % 256), (rand() % 256), (rand() % 256));
    }

    leftPt = trainImageKeyPoints[(size_t)(it->trainIdx)];
    rightPt = vpImagePoint(queryImageKeyPoints[(size_t)(it->queryIdx)].get_i(),
                           queryImageKeyPoints[(size_t)it->queryIdx].get_j() + IRef.getWidth());
    vpDisplay::displayCross(IMatching, leftPt, crossSize, currentColor);
    vpDisplay::displayCross(IMatching, rightPt, crossSize, currentColor);
    vpDisplay::displayLine(IMatching, leftPt, rightPt, currentColor, lineThickness);
  }
}

/*!
  Display the matching lines between the detected keypoints with those
  detected in one training image.

  \param IRef : Reference image, used to have the x-offset.
  \param IMatching : Resulting image matching.
  \param crossSize : Size of the displayed crosses.
  \param lineThickness : Thickness of the displayed lines.
  \param color : Color to use, if none, we pick randomly a color for each pair
  of matching.
 */
void vpKeyPoint::displayMatching(const vpImage<unsigned char> &IRef, vpImage<vpRGBa> &IMatching,
                                 unsigned int crossSize, unsigned int lineThickness, const vpColor &color)
{
  bool randomColor = (color == vpColor::none);
  srand((unsigned int)time(NULL));
  vpColor currentColor = color;

  std::vector<vpImagePoint> queryImageKeyPoints;
  getQueryKeyPoints(queryImageKeyPoints);
  std::vector<vpImagePoint> trainImageKeyPoints;
  getTrainKeyPoints(trainImageKeyPoints);

  vpImagePoint leftPt, rightPt;
  for (std::vector<cv::DMatch>::const_iterator it = m_filteredMatches.begin(); it != m_filteredMatches.end(); ++it) {
    if (randomColor) {
      currentColor = vpColor((rand() % 256), (rand() % 256), (rand() % 256));
    }

    leftPt = trainImageKeyPoints[(size_t)(it->trainIdx)];
    rightPt = vpImagePoint(queryImageKeyPoints[(size_t)(it->queryIdx)].get_i(),
                           queryImageKeyPoints[(size_t)it->queryIdx].get_j() + IRef.getWidth());
    vpDisplay::displayCross(IMatching, leftPt, crossSize, currentColor);
    vpDisplay::displayCross(IMatching, rightPt, crossSize, currentColor);
    vpDisplay::displayLine(IMatching, leftPt, rightPt, currentColor, lineThickness);
  }
}

/*!
  Display the matching lines between the detected keypoints with those
  detected in one training image.

  \param IRef : Reference image, used to have the x-offset.
  \param IMatching : Resulting image matching.
  \param crossSize : Size of the displayed crosses.
  \param lineThickness : Thickness of the displayed lines.
  \param color : Color to use, if none, we pick randomly a color for each pair
  of matching.
 */
void vpKeyPoint::displayMatching(const vpImage<vpRGBa> &IRef, vpImage<vpRGBa> &IMatching,
                                 unsigned int crossSize, unsigned int lineThickness, const vpColor &color)
{
  bool randomColor = (color == vpColor::none);
  srand((unsigned int)time(NULL));
  vpColor currentColor = color;

  std::vector<vpImagePoint> queryImageKeyPoints;
  getQueryKeyPoints(queryImageKeyPoints);
  std::vector<vpImagePoint> trainImageKeyPoints;
  getTrainKeyPoints(trainImageKeyPoints);

  vpImagePoint leftPt, rightPt;
  for (std::vector<cv::DMatch>::const_iterator it = m_filteredMatches.begin(); it != m_filteredMatches.end(); ++it) {
    if (randomColor) {
      currentColor = vpColor((rand() % 256), (rand() % 256), (rand() % 256));
    }

    leftPt = trainImageKeyPoints[(size_t)(it->trainIdx)];
    rightPt = vpImagePoint(queryImageKeyPoints[(size_t)(it->queryIdx)].get_i(),
                           queryImageKeyPoints[(size_t)it->queryIdx].get_j() + IRef.getWidth());
    vpDisplay::displayCross(IMatching, leftPt, crossSize, currentColor);
    vpDisplay::displayCross(IMatching, rightPt, crossSize, currentColor);
    vpDisplay::displayLine(IMatching, leftPt, rightPt, currentColor, lineThickness);
  }
}

/*!
   Display matching between keypoints detected in the current image and with
   those detected in the multiple training images. Display also RANSAC inliers
   if the list is supplied.

   \param ICurrent : Current image.
   \param IMatching : Resulting matching image.
   \param ransacInliers : List of Ransac inliers or empty list if not available.
   \param crossSize : Size of the displayed crosses.
   \param lineThickness : Thickness of the displayed line.
 */
void vpKeyPoint::displayMatching(const vpImage<unsigned char> &ICurrent, vpImage<unsigned char> &IMatching,
                                 const std::vector<vpImagePoint> &ransacInliers, unsigned int crossSize,
                                 unsigned int lineThickness)
{
  if (m_mapOfImages.empty() || m_mapOfImageId.empty()) {
    // No training images so return
    std::cerr << "There is no training image loaded !" << std::endl;
    return;
  }

  // Nb images in the training database + the current image we want to detect
  // the object
  int nbImg = (int)(m_mapOfImages.size() + 1);

  if (nbImg == 2) {
    // Only one training image, so we display the matching result side-by-side
    displayMatching(m_mapOfImages.begin()->second, IMatching, crossSize);
  } else {
    // Multiple training images, display them as a mosaic image
    int nbImgSqrt = vpMath::round(std::sqrt((double)nbImg)); //(int) std::floor(std::sqrt((double) nbImg) + 0.5);
    int nbWidth = nbImgSqrt;
    int nbHeight = nbImgSqrt;

    if (nbImgSqrt * nbImgSqrt < nbImg) {
      nbWidth++;
    }

    std::map<int, int> mapOfImageIdIndex;
    int cpt = 0;
    unsigned int maxW = ICurrent.getWidth(), maxH = ICurrent.getHeight();
    for (std::map<int, vpImage<unsigned char> >::const_iterator it = m_mapOfImages.begin(); it != m_mapOfImages.end();
         ++it, cpt++) {
      mapOfImageIdIndex[it->first] = cpt;

      if (maxW < it->second.getWidth()) {
        maxW = it->second.getWidth();
      }

      if (maxH < it->second.getHeight()) {
        maxH = it->second.getHeight();
      }
    }

    // Indexes of the current image in the grid computed to put preferably the
    // image in the center of the mosaic grid
    int medianI = nbHeight / 2;
    int medianJ = nbWidth / 2;
    int medianIndex = medianI * nbWidth + medianJ;
    for (std::vector<cv::KeyPoint>::const_iterator it = m_trainKeyPoints.begin(); it != m_trainKeyPoints.end(); ++it) {
      vpImagePoint topLeftCorner;
      int current_class_id_index = 0;
      if (mapOfImageIdIndex[m_mapOfImageId[it->class_id]] < medianIndex) {
        current_class_id_index = mapOfImageIdIndex[m_mapOfImageId[it->class_id]];
      } else {
        // Shift of one unity the index of the training images which are after
        // the current image
        current_class_id_index = mapOfImageIdIndex[m_mapOfImageId[it->class_id]] + 1;
      }

      int indexI = current_class_id_index / nbWidth;
      int indexJ = current_class_id_index - (indexI * nbWidth);
      topLeftCorner.set_ij((int)maxH * indexI, (int)maxW * indexJ);

      // Display cross for keypoints in the learning database
      vpDisplay::displayCross(IMatching, (int)(it->pt.y + topLeftCorner.get_i()),
                              (int)(it->pt.x + topLeftCorner.get_j()), crossSize, vpColor::red);
    }

    vpImagePoint topLeftCorner((int)maxH * medianI, (int)maxW * medianJ);
    for (std::vector<cv::KeyPoint>::const_iterator it = m_queryKeyPoints.begin(); it != m_queryKeyPoints.end(); ++it) {
      // Display cross for keypoints detected in the current image
      vpDisplay::displayCross(IMatching, (int)(it->pt.y + topLeftCorner.get_i()),
                              (int)(it->pt.x + topLeftCorner.get_j()), crossSize, vpColor::red);
    }
    for (std::vector<vpImagePoint>::const_iterator it = ransacInliers.begin(); it != ransacInliers.end(); ++it) {
      // Display green circle for RANSAC inliers
      vpDisplay::displayCircle(IMatching, (int)(it->get_v() + topLeftCorner.get_i()),
                               (int)(it->get_u() + topLeftCorner.get_j()), 4, vpColor::green);
    }
    for (std::vector<vpImagePoint>::const_iterator it = m_ransacOutliers.begin(); it != m_ransacOutliers.end(); ++it) {
      // Display red circle for RANSAC outliers
      vpDisplay::displayCircle(IMatching, (int)(it->get_i() + topLeftCorner.get_i()),
                               (int)(it->get_j() + topLeftCorner.get_j()), 4, vpColor::red);
    }

    for (std::vector<cv::DMatch>::const_iterator it = m_filteredMatches.begin(); it != m_filteredMatches.end(); ++it) {
      int current_class_id = 0;
      if (mapOfImageIdIndex[m_mapOfImageId[m_trainKeyPoints[(size_t)it->trainIdx].class_id]] < medianIndex) {
        current_class_id = mapOfImageIdIndex[m_mapOfImageId[m_trainKeyPoints[(size_t)it->trainIdx].class_id]];
      } else {
        // Shift of one unity the index of the training images which are after
        // the current image
        current_class_id = mapOfImageIdIndex[m_mapOfImageId[m_trainKeyPoints[(size_t)it->trainIdx].class_id]] + 1;
      }

      int indexI = current_class_id / nbWidth;
      int indexJ = current_class_id - (indexI * nbWidth);

      vpImagePoint end((int)maxH * indexI + m_trainKeyPoints[(size_t)it->trainIdx].pt.y,
                       (int)maxW * indexJ + m_trainKeyPoints[(size_t)it->trainIdx].pt.x);
      vpImagePoint start((int)maxH * medianI + m_queryFilteredKeyPoints[(size_t)it->queryIdx].pt.y,
                         (int)maxW * medianJ + m_queryFilteredKeyPoints[(size_t)it->queryIdx].pt.x);

      // Draw line for matching keypoints detected in the current image and
      // those detected  in the training images
      vpDisplay::displayLine(IMatching, start, end, vpColor::green, lineThickness);
    }
  }
}

/*!
   Display matching between keypoints detected in the current image and with
   those detected in the multiple training images. Display also RANSAC inliers
   if the list is supplied.

   \param ICurrent : Current image.
   \param IMatching : Resulting matching image.
   \param ransacInliers : List of Ransac inliers or empty list if not available.
   \param crossSize : Size of the displayed crosses.
   \param lineThickness : Thickness of the displayed line.
 */
void vpKeyPoint::displayMatching(const vpImage<vpRGBa> &ICurrent, vpImage<vpRGBa> &IMatching,
                                 const std::vector<vpImagePoint> &ransacInliers, unsigned int crossSize,
                                 unsigned int lineThickness)
{
  if (m_mapOfImages.empty() || m_mapOfImageId.empty()) {
    // No training images so return
    std::cerr << "There is no training image loaded !" << std::endl;
    return;
  }

  // Nb images in the training database + the current image we want to detect
  // the object
  int nbImg = (int)(m_mapOfImages.size() + 1);

  if (nbImg == 2) {
    // Only one training image, so we display the matching result side-by-side
    displayMatching(m_mapOfImages.begin()->second, IMatching, crossSize);
  } else {
    // Multiple training images, display them as a mosaic image
    int nbImgSqrt = vpMath::round(std::sqrt((double)nbImg)); //(int) std::floor(std::sqrt((double) nbImg) + 0.5);
    int nbWidth = nbImgSqrt;
    int nbHeight = nbImgSqrt;

    if (nbImgSqrt * nbImgSqrt < nbImg) {
      nbWidth++;
    }

    std::map<int, int> mapOfImageIdIndex;
    int cpt = 0;
    unsigned int maxW = ICurrent.getWidth(), maxH = ICurrent.getHeight();
    for (std::map<int, vpImage<unsigned char> >::const_iterator it = m_mapOfImages.begin(); it != m_mapOfImages.end();
         ++it, cpt++) {
      mapOfImageIdIndex[it->first] = cpt;

      if (maxW < it->second.getWidth()) {
        maxW = it->second.getWidth();
      }

      if (maxH < it->second.getHeight()) {
        maxH = it->second.getHeight();
      }
    }

    // Indexes of the current image in the grid computed to put preferably the
    // image in the center of the mosaic grid
    int medianI = nbHeight / 2;
    int medianJ = nbWidth / 2;
    int medianIndex = medianI * nbWidth + medianJ;
    for (std::vector<cv::KeyPoint>::const_iterator it = m_trainKeyPoints.begin(); it != m_trainKeyPoints.end(); ++it) {
      vpImagePoint topLeftCorner;
      int current_class_id_index = 0;
      if (mapOfImageIdIndex[m_mapOfImageId[it->class_id]] < medianIndex) {
        current_class_id_index = mapOfImageIdIndex[m_mapOfImageId[it->class_id]];
      } else {
        // Shift of one unity the index of the training images which are after
        // the current image
        current_class_id_index = mapOfImageIdIndex[m_mapOfImageId[it->class_id]] + 1;
      }

      int indexI = current_class_id_index / nbWidth;
      int indexJ = current_class_id_index - (indexI * nbWidth);
      topLeftCorner.set_ij((int)maxH * indexI, (int)maxW * indexJ);

      // Display cross for keypoints in the learning database
      vpDisplay::displayCross(IMatching, (int)(it->pt.y + topLeftCorner.get_i()),
                              (int)(it->pt.x + topLeftCorner.get_j()), crossSize, vpColor::red);
    }

    vpImagePoint topLeftCorner((int)maxH * medianI, (int)maxW * medianJ);
    for (std::vector<cv::KeyPoint>::const_iterator it = m_queryKeyPoints.begin(); it != m_queryKeyPoints.end(); ++it) {
      // Display cross for keypoints detected in the current image
      vpDisplay::displayCross(IMatching, (int)(it->pt.y + topLeftCorner.get_i()),
                              (int)(it->pt.x + topLeftCorner.get_j()), crossSize, vpColor::red);
    }
    for (std::vector<vpImagePoint>::const_iterator it = ransacInliers.begin(); it != ransacInliers.end(); ++it) {
      // Display green circle for RANSAC inliers
      vpDisplay::displayCircle(IMatching, (int)(it->get_v() + topLeftCorner.get_i()),
                               (int)(it->get_u() + topLeftCorner.get_j()), 4, vpColor::green);
    }
    for (std::vector<vpImagePoint>::const_iterator it = m_ransacOutliers.begin(); it != m_ransacOutliers.end(); ++it) {
      // Display red circle for RANSAC outliers
      vpDisplay::displayCircle(IMatching, (int)(it->get_i() + topLeftCorner.get_i()),
                               (int)(it->get_j() + topLeftCorner.get_j()), 4, vpColor::red);
    }

    for (std::vector<cv::DMatch>::const_iterator it = m_filteredMatches.begin(); it != m_filteredMatches.end(); ++it) {
      int current_class_id = 0;
      if (mapOfImageIdIndex[m_mapOfImageId[m_trainKeyPoints[(size_t)it->trainIdx].class_id]] < medianIndex) {
        current_class_id = mapOfImageIdIndex[m_mapOfImageId[m_trainKeyPoints[(size_t)it->trainIdx].class_id]];
      } else {
        // Shift of one unity the index of the training images which are after
        // the current image
        current_class_id = mapOfImageIdIndex[m_mapOfImageId[m_trainKeyPoints[(size_t)it->trainIdx].class_id]] + 1;
      }

      int indexI = current_class_id / nbWidth;
      int indexJ = current_class_id - (indexI * nbWidth);

      vpImagePoint end((int)maxH * indexI + m_trainKeyPoints[(size_t)it->trainIdx].pt.y,
                       (int)maxW * indexJ + m_trainKeyPoints[(size_t)it->trainIdx].pt.x);
      vpImagePoint start((int)maxH * medianI + m_queryFilteredKeyPoints[(size_t)it->queryIdx].pt.y,
                         (int)maxW * medianJ + m_queryFilteredKeyPoints[(size_t)it->queryIdx].pt.x);

      // Draw line for matching keypoints detected in the current image and
      // those detected  in the training images
      vpDisplay::displayLine(IMatching, start, end, vpColor::green, lineThickness);
    }
  }
}

/*!
   Extract the descriptors for each keypoints of the list.

   \param I : Input image.
   \param keyPoints : List of keypoints we want to extract their descriptors.
   \param descriptors : Descriptors matrix with at each row the descriptors
   values for each keypoint. \param trainPoints : Pointer to the list of 3D
   train points, when a keypoint cannot be extracted, we need to remove the
   corresponding 3D point.
 */
void vpKeyPoint::extract(const vpImage<unsigned char> &I, std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors,
                         std::vector<cv::Point3f> *trainPoints)
{
  double elapsedTime;
  extract(I, keyPoints, descriptors, elapsedTime, trainPoints);
}

/*!
   Extract the descriptors for each keypoints of the list.

   \param I_color : Input image.
   \param keyPoints : List of keypoints we want to extract their descriptors.
   \param descriptors : Descriptors matrix with at each row the descriptors
   values for each keypoint. \param trainPoints : Pointer to the list of 3D
   train points, when a keypoint cannot be extracted, we need to remove the
   corresponding 3D point.
 */
void vpKeyPoint::extract(const vpImage<vpRGBa> &I_color, std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors,
                         std::vector<cv::Point3f> *trainPoints)
{
  double elapsedTime;
  extract(I_color, keyPoints, descriptors, elapsedTime, trainPoints);
}

/*!
   Extract the descriptors for each keypoints of the list.

   \param matImg : Input image.
   \param keyPoints : List of keypoints we want to extract their descriptors.
   \param descriptors : Descriptors matrix with at each row the descriptors
   values for each keypoint. \param trainPoints : Pointer to the list of 3D
   train points, when a keypoint cannot be extracted, we need to remove the
   corresponding 3D point.
 */
void vpKeyPoint::extract(const cv::Mat &matImg, std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors,
                         std::vector<cv::Point3f> *trainPoints)
{
  double elapsedTime;
  extract(matImg, keyPoints, descriptors, elapsedTime, trainPoints);
}

/*!
   Extract the descriptors for each keypoints of the list.

   \param I : Input image.
   \param keyPoints : List of keypoints we want to extract their descriptors.
   \param descriptors : Descriptors matrix with at each row the descriptors
   values for each keypoint. \param elapsedTime : Elapsed time. \param
   trainPoints : Pointer to the list of 3D train points, when a keypoint
   cannot be extracted, we need to remove the corresponding 3D point.
 */
void vpKeyPoint::extract(const vpImage<unsigned char> &I, std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors,
                         double &elapsedTime, std::vector<cv::Point3f> *trainPoints)
{
  cv::Mat matImg;
  vpImageConvert::convert(I, matImg, false);
  extract(matImg, keyPoints, descriptors, elapsedTime, trainPoints);
}

/*!
   Extract the descriptors for each keypoints of the list.

   \param I_color : Input image.
   \param keyPoints : List of keypoints we want to extract their descriptors.
   \param descriptors : Descriptors matrix with at each row the descriptors
   values for each keypoint. \param elapsedTime : Elapsed time. \param
   trainPoints : Pointer to the list of 3D train points, when a keypoint
   cannot be extracted, we need to remove the corresponding 3D point.
 */
void vpKeyPoint::extract(const vpImage<vpRGBa> &I_color, std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors,
                         double &elapsedTime, std::vector<cv::Point3f> *trainPoints)
{
  cv::Mat matImg;
  vpImageConvert::convert(I_color, matImg);
  extract(matImg, keyPoints, descriptors, elapsedTime, trainPoints);
}

/*!
   Extract the descriptors for each keypoints of the list.

   \param matImg : Input image.
   \param keyPoints : List of keypoints we want to extract their descriptors.
   \param descriptors : Descriptors matrix with at each row the descriptors
   values for each keypoint. \param elapsedTime : Elapsed time. \param
   trainPoints : Pointer to the list of 3D train points, when a keypoint
   cannot be extracted, we need to remove the corresponding 3D point.
 */
void vpKeyPoint::extract(const cv::Mat &matImg, std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors,
                         double &elapsedTime, std::vector<cv::Point3f> *trainPoints)
{
  double t = vpTime::measureTimeMs();
  bool first = true;

  for (std::map<std::string, cv::Ptr<cv::DescriptorExtractor> >::const_iterator itd = m_extractors.begin();
       itd != m_extractors.end(); ++itd) {
    if (first) {
      first = false;
      // Check if we have 3D object points information
      if (trainPoints != NULL && !trainPoints->empty()) {
        // Copy the input list of keypoints, keypoints that cannot be computed
        // are removed in the function compute
        std::vector<cv::KeyPoint> keyPoints_tmp = keyPoints;

        // Extract descriptors for the given list of keypoints
        itd->second->compute(matImg, keyPoints, descriptors);

        if (keyPoints.size() != keyPoints_tmp.size()) {
          // Keypoints have been removed
          // Store the hash of a keypoint as the key and the index of the
          // keypoint as the value
          std::map<size_t, size_t> mapOfKeypointHashes;
          size_t cpt = 0;
          for (std::vector<cv::KeyPoint>::const_iterator it = keyPoints_tmp.begin(); it != keyPoints_tmp.end();
               ++it, cpt++) {
            mapOfKeypointHashes[myKeypointHash(*it)] = cpt;
          }

          std::vector<cv::Point3f> trainPoints_tmp;
          for (std::vector<cv::KeyPoint>::const_iterator it = keyPoints.begin(); it != keyPoints.end(); ++it) {
            if (mapOfKeypointHashes.find(myKeypointHash(*it)) != mapOfKeypointHashes.end()) {
              trainPoints_tmp.push_back((*trainPoints)[mapOfKeypointHashes[myKeypointHash(*it)]]);
            }
          }

          // Copy trainPoints_tmp to m_trainPoints
          *trainPoints = trainPoints_tmp;
        }
      } else {
        // Extract descriptors for the given list of keypoints
        itd->second->compute(matImg, keyPoints, descriptors);
      }
    } else {
      // Copy the input list of keypoints, keypoints that cannot be computed
      // are removed in the function compute
      std::vector<cv::KeyPoint> keyPoints_tmp = keyPoints;

      cv::Mat desc;
      // Extract descriptors for the given list of keypoints
      itd->second->compute(matImg, keyPoints, desc);

      if (keyPoints.size() != keyPoints_tmp.size()) {
        // Keypoints have been removed
        // Store the hash of a keypoint as the key and the index of the
        // keypoint as the value
        std::map<size_t, size_t> mapOfKeypointHashes;
        size_t cpt = 0;
        for (std::vector<cv::KeyPoint>::const_iterator it = keyPoints_tmp.begin(); it != keyPoints_tmp.end();
             ++it, cpt++) {
          mapOfKeypointHashes[myKeypointHash(*it)] = cpt;
        }

        std::vector<cv::Point3f> trainPoints_tmp;
        cv::Mat descriptors_tmp;
        for (std::vector<cv::KeyPoint>::const_iterator it = keyPoints.begin(); it != keyPoints.end(); ++it) {
          if (mapOfKeypointHashes.find(myKeypointHash(*it)) != mapOfKeypointHashes.end()) {
            if (trainPoints != NULL && !trainPoints->empty()) {
              trainPoints_tmp.push_back((*trainPoints)[mapOfKeypointHashes[myKeypointHash(*it)]]);
            }

            if (!descriptors.empty()) {
              descriptors_tmp.push_back(descriptors.row((int)mapOfKeypointHashes[myKeypointHash(*it)]));
            }
          }
        }

        if (trainPoints != NULL) {
          // Copy trainPoints_tmp to m_trainPoints
          *trainPoints = trainPoints_tmp;
        }
        // Copy descriptors_tmp to descriptors
        descriptors_tmp.copyTo(descriptors);
      }

      // Merge descriptors horizontally
      if (descriptors.empty()) {
        desc.copyTo(descriptors);
      } else {
        cv::hconcat(descriptors, desc, descriptors);
      }
    }
  }

  if (keyPoints.size() != (size_t)descriptors.rows) {
    std::cerr << "keyPoints.size() != (size_t) descriptors.rows" << std::endl;
  }
  elapsedTime = vpTime::measureTimeMs() - t;
}

/*!
   Filter the matches using the desired filtering method.
 */
void vpKeyPoint::filterMatches()
{
  std::vector<cv::KeyPoint> queryKpts;
  std::vector<cv::Point3f> trainPts;
  std::vector<cv::DMatch> m;

  if (m_useKnn) {
    // double max_dist = 0;
    // double min_dist = std::numeric_limits<double>::max(); // create an
    // error under Windows. To fix it we have to add #undef max
    double min_dist = DBL_MAX;
    double mean = 0.0;
    std::vector<double> distance_vec(m_knnMatches.size());

    if (m_filterType == stdAndRatioDistanceThreshold) {
      for (size_t i = 0; i < m_knnMatches.size(); i++) {
        double dist = m_knnMatches[i][0].distance;
        mean += dist;
        distance_vec[i] = dist;

        if (dist < min_dist) {
          min_dist = dist;
        }
        // if (dist > max_dist) {
        //  max_dist = dist;
        //}
      }
      mean /= m_queryDescriptors.rows;
    }

    double sq_sum = std::inner_product(distance_vec.begin(), distance_vec.end(), distance_vec.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / distance_vec.size() - mean * mean);
    double threshold = min_dist + stdev;

    for (size_t i = 0; i < m_knnMatches.size(); i++) {
      if (m_knnMatches[i].size() >= 2) {
        // Calculate ratio of the descriptor distance between the two nearest
        // neighbors of the keypoint
        float ratio = m_knnMatches[i][0].distance / m_knnMatches[i][1].distance;
        //        float ratio = std::sqrt((vecMatches[i][0].distance *
        //        vecMatches[i][0].distance)
        //            / (vecMatches[i][1].distance *
        //            vecMatches[i][1].distance));
        double dist = m_knnMatches[i][0].distance;

        if (ratio < m_matchingRatioThreshold || (m_filterType == stdAndRatioDistanceThreshold && dist < threshold)) {
          m.push_back(cv::DMatch((int)queryKpts.size(), m_knnMatches[i][0].trainIdx, m_knnMatches[i][0].distance));

          if (!m_trainPoints.empty()) {
            trainPts.push_back(m_trainPoints[(size_t)m_knnMatches[i][0].trainIdx]);
          }
          queryKpts.push_back(m_queryKeyPoints[(size_t)m_knnMatches[i][0].queryIdx]);
        }
      }
    }
  } else {
    // double max_dist = 0;
    // create an error under Windows. To fix it we have to add #undef max
    // double min_dist = std::numeric_limits<double>::max();
    double min_dist = DBL_MAX;
    double mean = 0.0;
    std::vector<double> distance_vec(m_matches.size());
    for (size_t i = 0; i < m_matches.size(); i++) {
      double dist = m_matches[i].distance;
      mean += dist;
      distance_vec[i] = dist;

      if (dist < min_dist) {
        min_dist = dist;
      }
      // if (dist > max_dist) {
      //  max_dist = dist;
      // }
    }
    mean /= m_queryDescriptors.rows;

    double sq_sum = std::inner_product(distance_vec.begin(), distance_vec.end(), distance_vec.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / distance_vec.size() - mean * mean);

    // Define a threshold where we keep all keypoints whose the descriptor
    // distance falls below a factor of the  minimum descriptor distance (for
    // all the query keypoints)  or below the minimum descriptor distance +
    // the standard deviation (calculated on all the query descriptor
    // distances)
    double threshold =
        m_filterType == constantFactorDistanceThreshold ? m_matchingFactorThreshold * min_dist : min_dist + stdev;

    for (size_t i = 0; i < m_matches.size(); i++) {
      if (m_matches[i].distance <= threshold) {
        m.push_back(cv::DMatch((int)queryKpts.size(), m_matches[i].trainIdx, m_matches[i].distance));

        if (!m_trainPoints.empty()) {
          trainPts.push_back(m_trainPoints[(size_t)m_matches[i].trainIdx]);
        }
        queryKpts.push_back(m_queryKeyPoints[(size_t)m_matches[i].queryIdx]);
      }
    }
  }

  if (m_useSingleMatchFilter) {
    // Eliminate matches where multiple query keypoints are matched to the
    // same train keypoint
    std::vector<cv::DMatch> mTmp;
    std::vector<cv::Point3f> trainPtsTmp;
    std::vector<cv::KeyPoint> queryKptsTmp;

    std::map<int, int> mapOfTrainIdx;
    // Count the number of query points matched to the same train point
    for (std::vector<cv::DMatch>::const_iterator it = m.begin(); it != m.end(); ++it) {
      mapOfTrainIdx[it->trainIdx]++;
    }

    // Keep matches with only one correspondence
    for (std::vector<cv::DMatch>::const_iterator it = m.begin(); it != m.end(); ++it) {
      if (mapOfTrainIdx[it->trainIdx] == 1) {
        mTmp.push_back(cv::DMatch((int)queryKptsTmp.size(), it->trainIdx, it->distance));

        if (!m_trainPoints.empty()) {
          trainPtsTmp.push_back(m_trainPoints[(size_t)it->trainIdx]);
        }
        queryKptsTmp.push_back(queryKpts[(size_t)it->queryIdx]);
      }
    }

    m_filteredMatches = mTmp;
    m_objectFilteredPoints = trainPtsTmp;
    m_queryFilteredKeyPoints = queryKptsTmp;
  } else {
    m_filteredMatches = m;
    m_objectFilteredPoints = trainPts;
    m_queryFilteredKeyPoints = queryKpts;
  }
}

/*!
   Get the 3D coordinates of the object points matched (the corresponding 3D
   coordinates in the object frame of the keypoints detected in the current
   image after the matching).

   \param objectPoints : List of 3D coordinates in the object frame.
 */
void vpKeyPoint::getObjectPoints(std::vector<cv::Point3f> &objectPoints) const
{
  objectPoints = m_objectFilteredPoints;
}

/*!
   Get the 3D coordinates of the object points matched (the corresponding 3D
   coordinates in the object frame of the keypoints detected in the current
   image after the matching).

   \param objectPoints : List of 3D coordinates in the object frame.
 */
void vpKeyPoint::getObjectPoints(std::vector<vpPoint> &objectPoints) const
{
  vpConvert::convertFromOpenCV(m_objectFilteredPoints, objectPoints);
}

/*!
   Get the query keypoints list in OpenCV type.

   \param keyPoints : List of query keypoints (or keypoints detected in the
   current image).
 */
void vpKeyPoint::getQueryKeyPoints(std::vector<cv::KeyPoint> &keyPoints) const { keyPoints = m_queryFilteredKeyPoints; }

/*!
   Get the query keypoints list in ViSP type.

   \param keyPoints : List of query keypoints (or keypoints detected in the
   current image).
 */
void vpKeyPoint::getQueryKeyPoints(std::vector<vpImagePoint> &keyPoints) const { keyPoints = currentImagePointsList; }

/*!
   Get the train keypoints list in OpenCV type.

   \param keyPoints : List of train keypoints (or reference keypoints).
 */
void vpKeyPoint::getTrainKeyPoints(std::vector<cv::KeyPoint> &keyPoints) const { keyPoints = m_trainKeyPoints; }

/*!
   Get the train keypoints list in ViSP type.

   \param keyPoints : List of train keypoints (or reference keypoints).
 */
void vpKeyPoint::getTrainKeyPoints(std::vector<vpImagePoint> &keyPoints) const { keyPoints = referenceImagePointsList; }

/*!
   Get the train points (the 3D coordinates in the object frame) list in
   OpenCV type.

   \param points : List of train points (or reference points).
 */
void vpKeyPoint::getTrainPoints(std::vector<cv::Point3f> &points) const { points = m_trainPoints; }

/*!
   Get the train points (the 3D coordinates in the object frame) list in ViSP
   type.

   \param points : List of train points (or reference points).
 */
void vpKeyPoint::getTrainPoints(std::vector<vpPoint> &points) const { points = m_trainVpPoints; }

/*!
   Initialize method for RANSAC parameters and for detectors, extractors and
   matcher, and for others parameters.
 */
void vpKeyPoint::init()
{
// Require 2.4.0 <= opencv < 3.0.0
#if defined(VISP_HAVE_OPENCV_NONFREE) && (VISP_HAVE_OPENCV_VERSION >= 0x020400) && (VISP_HAVE_OPENCV_VERSION < 0x030000)
  // The following line must be called in order to use SIFT or SURF
  if (!cv::initModule_nonfree()) {
    std::cerr << "Cannot init module non free, SIFT or SURF cannot be used." << std::endl;
  }
#endif

  // Use k-nearest neighbors (knn) to retrieve the two best matches for a
  // keypoint  So this is useful only for ratioDistanceThreshold method
  if (m_filterType == ratioDistanceThreshold || m_filterType == stdAndRatioDistanceThreshold) {
    m_useKnn = true;
  }

  initDetectors(m_detectorNames);
  initExtractors(m_extractorNames);
  initMatcher(m_matcherName);
}

/*!
   Initialize a keypoint detector based on its name.

   \param detectorName : Name of the detector (e.g FAST, SIFT, SURF, etc.).
 */
void vpKeyPoint::initDetector(const std::string &detectorName)
{
#if (VISP_HAVE_OPENCV_VERSION < 0x030000)
  m_detectors[detectorName] = cv::FeatureDetector::create(detectorName);

  if (m_detectors[detectorName] == NULL) {
    std::stringstream ss_msg;
    ss_msg << "Fail to initialize the detector: " << detectorName
           << " or it is not available in OpenCV version: " << std::hex << VISP_HAVE_OPENCV_VERSION << ".";
    throw vpException(vpException::fatalError, ss_msg.str());
  }
#else
  std::string detectorNameTmp = detectorName;
  std::string pyramid = "Pyramid";
  std::size_t pos = detectorName.find(pyramid);
  bool usePyramid = false;
  if (pos != std::string::npos) {
    detectorNameTmp = detectorName.substr(pos + pyramid.size());
    usePyramid = true;
  }

  if (detectorNameTmp == "SIFT") {
#ifdef VISP_HAVE_OPENCV_XFEATURES2D
    cv::Ptr<cv::FeatureDetector> siftDetector = cv::xfeatures2d::SIFT::create();
    if (!usePyramid) {
      m_detectors[detectorNameTmp] = siftDetector;
    } else {
      std::cerr << "You should not use SIFT with Pyramid feature detection!" << std::endl;
      m_detectors[detectorName] = cv::makePtr<PyramidAdaptedFeatureDetector>(siftDetector);
    }
#else
    std::stringstream ss_msg;
    ss_msg << "Fail to initialize the detector: SIFT. OpenCV version  " << std::hex << VISP_HAVE_OPENCV_VERSION
           << " was not build with xFeatures2d module.";
    throw vpException(vpException::fatalError, ss_msg.str());
#endif
  } else if (detectorNameTmp == "SURF") {
#ifdef VISP_HAVE_OPENCV_XFEATURES2D
    cv::Ptr<cv::FeatureDetector> surfDetector = cv::xfeatures2d::SURF::create();
    if (!usePyramid) {
      m_detectors[detectorNameTmp] = surfDetector;
    } else {
      std::cerr << "You should not use SURF with Pyramid feature detection!" << std::endl;
      m_detectors[detectorName] = cv::makePtr<PyramidAdaptedFeatureDetector>(surfDetector);
    }
#else
    std::stringstream ss_msg;
    ss_msg << "Fail to initialize the detector: SURF. OpenCV version  " << std::hex << VISP_HAVE_OPENCV_VERSION
           << " was not build with xFeatures2d module.";
    throw vpException(vpException::fatalError, ss_msg.str());
#endif
  } else if (detectorNameTmp == "FAST") {
    cv::Ptr<cv::FeatureDetector> fastDetector = cv::FastFeatureDetector::create();
    if (!usePyramid) {
      m_detectors[detectorNameTmp] = fastDetector;
    } else {
      m_detectors[detectorName] = cv::makePtr<PyramidAdaptedFeatureDetector>(fastDetector);
    }
  } else if (detectorNameTmp == "MSER") {
    cv::Ptr<cv::FeatureDetector> fastDetector = cv::MSER::create();
    if (!usePyramid) {
      m_detectors[detectorNameTmp] = fastDetector;
    } else {
      m_detectors[detectorName] = cv::makePtr<PyramidAdaptedFeatureDetector>(fastDetector);
    }
  } else if (detectorNameTmp == "ORB") {
    cv::Ptr<cv::FeatureDetector> orbDetector = cv::ORB::create();
    if (!usePyramid) {
      m_detectors[detectorNameTmp] = orbDetector;
    } else {
      std::cerr << "You should not use ORB with Pyramid feature detection!" << std::endl;
      m_detectors[detectorName] = cv::makePtr<PyramidAdaptedFeatureDetector>(orbDetector);
    }
  } else if (detectorNameTmp == "BRISK") {
    cv::Ptr<cv::FeatureDetector> briskDetector = cv::BRISK::create();
    if (!usePyramid) {
      m_detectors[detectorNameTmp] = briskDetector;
    } else {
      std::cerr << "You should not use BRISK with Pyramid feature detection!" << std::endl;
      m_detectors[detectorName] = cv::makePtr<PyramidAdaptedFeatureDetector>(briskDetector);
    }
  } else if (detectorNameTmp == "KAZE") {
    cv::Ptr<cv::FeatureDetector> kazeDetector = cv::KAZE::create();
    if (!usePyramid) {
      m_detectors[detectorNameTmp] = kazeDetector;
    } else {
      std::cerr << "You should not use KAZE with Pyramid feature detection!" << std::endl;
      m_detectors[detectorName] = cv::makePtr<PyramidAdaptedFeatureDetector>(kazeDetector);
    }
  } else if (detectorNameTmp == "AKAZE") {
    cv::Ptr<cv::FeatureDetector> akazeDetector = cv::AKAZE::create();
    if (!usePyramid) {
      m_detectors[detectorNameTmp] = akazeDetector;
    } else {
      std::cerr << "You should not use AKAZE with Pyramid feature detection!" << std::endl;
      m_detectors[detectorName] = cv::makePtr<PyramidAdaptedFeatureDetector>(akazeDetector);
    }
  } else if (detectorNameTmp == "GFTT") {
    cv::Ptr<cv::FeatureDetector> gfttDetector = cv::GFTTDetector::create();
    if (!usePyramid) {
      m_detectors[detectorNameTmp] = gfttDetector;
    } else {
      m_detectors[detectorName] = cv::makePtr<PyramidAdaptedFeatureDetector>(gfttDetector);
    }
  } else if (detectorNameTmp == "SimpleBlob") {
    cv::Ptr<cv::FeatureDetector> simpleBlobDetector = cv::SimpleBlobDetector::create();
    if (!usePyramid) {
      m_detectors[detectorNameTmp] = simpleBlobDetector;
    } else {
      m_detectors[detectorName] = cv::makePtr<PyramidAdaptedFeatureDetector>(simpleBlobDetector);
    }
  } else if (detectorNameTmp == "STAR") {
#ifdef VISP_HAVE_OPENCV_XFEATURES2D
    cv::Ptr<cv::FeatureDetector> starDetector = cv::xfeatures2d::StarDetector::create();
    if (!usePyramid) {
      m_detectors[detectorNameTmp] = starDetector;
    } else {
      m_detectors[detectorName] = cv::makePtr<PyramidAdaptedFeatureDetector>(starDetector);
    }
#else
    std::stringstream ss_msg;
    ss_msg << "Fail to initialize the detector: STAR. OpenCV version  " << std::hex << VISP_HAVE_OPENCV_VERSION
           << " was not build with xFeatures2d module.";
    throw vpException(vpException::fatalError, ss_msg.str());
#endif
  } else if (detectorNameTmp == "AGAST") {
    cv::Ptr<cv::FeatureDetector> agastDetector = cv::AgastFeatureDetector::create();
    if (!usePyramid) {
      m_detectors[detectorNameTmp] = agastDetector;
    } else {
      m_detectors[detectorName] = cv::makePtr<PyramidAdaptedFeatureDetector>(agastDetector);
    }
  } else if (detectorNameTmp == "MSD") {
#if (VISP_HAVE_OPENCV_VERSION >= 0x030100)
#if defined(VISP_HAVE_OPENCV_XFEATURES2D)
    cv::Ptr<cv::FeatureDetector> msdDetector = cv::xfeatures2d::MSDDetector::create();
    if (!usePyramid) {
      m_detectors[detectorNameTmp] = msdDetector;
    } else {
      std::cerr << "You should not use MSD with Pyramid feature detection!" << std::endl;
      m_detectors[detectorName] = cv::makePtr<PyramidAdaptedFeatureDetector>(msdDetector);
    }
#else
    std::stringstream ss_msg;
    ss_msg << "Fail to initialize the detector: MSD. OpenCV version " << std::hex << VISP_HAVE_OPENCV_VERSION
           << " was not build with xFeatures2d module.";
    throw vpException(vpException::fatalError, ss_msg.str());
#endif
#else
    std::stringstream ss_msg;
    ss_msg << "Feature " << detectorName << " is not available in OpenCV version: " << std::hex
           << VISP_HAVE_OPENCV_VERSION << " (require >= OpenCV 3.1).";
#endif
  } else {
    std::cerr << "The detector:" << detectorNameTmp << " is not available." << std::endl;
  }

  bool detectorInitialized = false;
  if (!usePyramid) {
    //if not null and to avoid warning C4800: forcing value to bool 'true' or 'false' (performance warning)
    detectorInitialized = !m_detectors[detectorNameTmp].empty();
  } else {
    //if not null and to avoid warning C4800: forcing value to bool 'true' or 'false' (performance warning)
    detectorInitialized = !m_detectors[detectorName].empty();
  }

  if (!detectorInitialized) {
    std::stringstream ss_msg;
    ss_msg << "Fail to initialize the detector: " << detectorNameTmp
           << " or it is not available in OpenCV version: " << std::hex << VISP_HAVE_OPENCV_VERSION << ".";
    throw vpException(vpException::fatalError, ss_msg.str());
  }

#endif
}

/*!
   Initialize a list of keypoints detectors if we want to concatenate multiple
   detectors.

   \param detectorNames : List of detector names.
 */
void vpKeyPoint::initDetectors(const std::vector<std::string> &detectorNames)
{
  for (std::vector<std::string>::const_iterator it = detectorNames.begin(); it != detectorNames.end(); ++it) {
    initDetector(*it);
  }
}

/*!
   Initialize a descriptor extractor based on its name.

   \param extractorName : Name of the extractor (e.g SIFT, SURF, ORB, etc.).
 */
void vpKeyPoint::initExtractor(const std::string &extractorName)
{
#if (VISP_HAVE_OPENCV_VERSION < 0x030000)
  m_extractors[extractorName] = cv::DescriptorExtractor::create(extractorName);
#else
  if (extractorName == "SIFT") {
#ifdef VISP_HAVE_OPENCV_XFEATURES2D
    m_extractors[extractorName] = cv::xfeatures2d::SIFT::create();
#else
    std::stringstream ss_msg;
    ss_msg << "Fail to initialize the extractor: SIFT. OpenCV version  " << std::hex << VISP_HAVE_OPENCV_VERSION
           << " was not build with xFeatures2d module.";
    throw vpException(vpException::fatalError, ss_msg.str());
#endif
  } else if (extractorName == "SURF") {
#ifdef VISP_HAVE_OPENCV_XFEATURES2D
    // Use extended set of SURF descriptors (128 instead of 64)
    m_extractors[extractorName] = cv::xfeatures2d::SURF::create(100, 4, 3, true);
#else
    std::stringstream ss_msg;
    ss_msg << "Fail to initialize the extractor: SURF. OpenCV version  " << std::hex << VISP_HAVE_OPENCV_VERSION
           << " was not build with xFeatures2d module.";
    throw vpException(vpException::fatalError, ss_msg.str());
#endif
  } else if (extractorName == "ORB") {
    m_extractors[extractorName] = cv::ORB::create();
  } else if (extractorName == "BRISK") {
    m_extractors[extractorName] = cv::BRISK::create();
  } else if (extractorName == "FREAK") {
#ifdef VISP_HAVE_OPENCV_XFEATURES2D
    m_extractors[extractorName] = cv::xfeatures2d::FREAK::create();
#else
    std::stringstream ss_msg;
    ss_msg << "Fail to initialize the extractor: " << extractorName << ". OpenCV version " << std::hex
           << VISP_HAVE_OPENCV_VERSION << " was not build with xFeatures2d module.";
    throw vpException(vpException::fatalError, ss_msg.str());
#endif
  } else if (extractorName == "BRIEF") {
#ifdef VISP_HAVE_OPENCV_XFEATURES2D
    m_extractors[extractorName] = cv::xfeatures2d::BriefDescriptorExtractor::create();
#else
    std::stringstream ss_msg;
    ss_msg << "Fail to initialize the extractor: " << extractorName << ". OpenCV version " << std::hex
           << VISP_HAVE_OPENCV_VERSION << " was not build with xFeatures2d module.";
    throw vpException(vpException::fatalError, ss_msg.str());
#endif
  } else if (extractorName == "KAZE") {
    m_extractors[extractorName] = cv::KAZE::create();
  } else if (extractorName == "AKAZE") {
    m_extractors[extractorName] = cv::AKAZE::create();
  } else if (extractorName == "DAISY") {
#ifdef VISP_HAVE_OPENCV_XFEATURES2D
    m_extractors[extractorName] = cv::xfeatures2d::DAISY::create();
#else
    std::stringstream ss_msg;
    ss_msg << "Fail to initialize the extractor: " << extractorName << ". OpenCV version " << std::hex
           << VISP_HAVE_OPENCV_VERSION << " was not build with xFeatures2d module.";
    throw vpException(vpException::fatalError, ss_msg.str());
#endif
  } else if (extractorName == "LATCH") {
#ifdef VISP_HAVE_OPENCV_XFEATURES2D
    m_extractors[extractorName] = cv::xfeatures2d::LATCH::create();
#else
    std::stringstream ss_msg;
    ss_msg << "Fail to initialize the extractor: " << extractorName << ". OpenCV version " << std::hex
           << VISP_HAVE_OPENCV_VERSION << " was not build with xFeatures2d module.";
    throw vpException(vpException::fatalError, ss_msg.str());
#endif
  } else if (extractorName == "LUCID") {
#ifdef VISP_HAVE_OPENCV_XFEATURES2D
    //    m_extractors[extractorName] = cv::xfeatures2d::LUCID::create(1, 2);
    // Not possible currently, need a color image
    throw vpException(vpException::badValue, "Not possible currently as it needs a color image.");
#else
    std::stringstream ss_msg;
    ss_msg << "Fail to initialize the extractor: " << extractorName << ". OpenCV version " << std::hex
           << VISP_HAVE_OPENCV_VERSION << " was not build with xFeatures2d module.";
    throw vpException(vpException::fatalError, ss_msg.str());
#endif
  } else if (extractorName == "VGG") {
#if (VISP_HAVE_OPENCV_VERSION >= 0x030200)
#if defined(VISP_HAVE_OPENCV_XFEATURES2D)
    m_extractors[extractorName] = cv::xfeatures2d::VGG::create();
#else
    std::stringstream ss_msg;
    ss_msg << "Fail to initialize the extractor: " << extractorName << ". OpenCV version " << std::hex
           << VISP_HAVE_OPENCV_VERSION << " was not build with xFeatures2d module.";
    throw vpException(vpException::fatalError, ss_msg.str());
#endif
#else
    std::stringstream ss_msg;
    ss_msg << "Fail to initialize the extractor: " << extractorName << ". OpenCV version " << std::hex
           << VISP_HAVE_OPENCV_VERSION << " but requires at least OpenCV 3.2.";
    throw vpException(vpException::fatalError, ss_msg.str());
#endif
  } else if (extractorName == "BoostDesc") {
#if (VISP_HAVE_OPENCV_VERSION >= 0x030200)
#if defined(VISP_HAVE_OPENCV_XFEATURES2D)
    m_extractors[extractorName] = cv::xfeatures2d::BoostDesc::create();
#else
    std::stringstream ss_msg;
    ss_msg << "Fail to initialize the extractor: " << extractorName << ". OpenCV version " << std::hex
           << VISP_HAVE_OPENCV_VERSION << " was not build with xFeatures2d module.";
    throw vpException(vpException::fatalError, ss_msg.str());
#endif
#else
    std::stringstream ss_msg;
    ss_msg << "Fail to initialize the extractor: " << extractorName << ". OpenCV version " << std::hex
           << VISP_HAVE_OPENCV_VERSION << " but requires at least OpenCV 3.2.";
    throw vpException(vpException::fatalError, ss_msg.str());
#endif
  } else {
    std::cerr << "The extractor:" << extractorName << " is not available." << std::endl;
  }
#endif

  if (!m_extractors[extractorName]) { //if null
    std::stringstream ss_msg;
    ss_msg << "Fail to initialize the extractor: " << extractorName
           << " or it is not available in OpenCV version: " << std::hex << VISP_HAVE_OPENCV_VERSION << ".";
    throw vpException(vpException::fatalError, ss_msg.str());
  }

#if (VISP_HAVE_OPENCV_VERSION >= 0x020400 && VISP_HAVE_OPENCV_VERSION < 0x030000)
  if (extractorName == "SURF") {
    // Use extended set of SURF descriptors (128 instead of 64)
    m_extractors[extractorName]->set("extended", 1);
  }
#endif
}

/*!
   Initialize a list of descriptor extractors if we want to concatenate
   multiple extractors.

   \param extractorNames : List of extractor names.
 */
void vpKeyPoint::initExtractors(const std::vector<std::string> &extractorNames)
{
  for (std::vector<std::string>::const_iterator it = extractorNames.begin(); it != extractorNames.end(); ++it) {
    initExtractor(*it);
  }

  int descriptorType = CV_32F;
  bool firstIteration = true;
  for (std::map<std::string, cv::Ptr<cv::DescriptorExtractor> >::const_iterator it = m_extractors.begin();
       it != m_extractors.end(); ++it) {
    if (firstIteration) {
      firstIteration = false;
      descriptorType = it->second->descriptorType();
    } else {
      if (descriptorType != it->second->descriptorType()) {
        throw vpException(vpException::fatalError, "All the descriptors must have the same type !");
      }
    }
  }
}

void vpKeyPoint::initFeatureNames()
{
// Create map enum to string
#if (VISP_HAVE_OPENCV_VERSION >= 0x020403)
  m_mapOfDetectorNames[DETECTOR_FAST] = "FAST";
  m_mapOfDetectorNames[DETECTOR_MSER] = "MSER";
  m_mapOfDetectorNames[DETECTOR_ORB] = "ORB";
  m_mapOfDetectorNames[DETECTOR_BRISK] = "BRISK";
  m_mapOfDetectorNames[DETECTOR_GFTT] = "GFTT";
  m_mapOfDetectorNames[DETECTOR_SimpleBlob] = "SimpleBlob";
#if (VISP_HAVE_OPENCV_VERSION < 0x030000) || (defined(VISP_HAVE_OPENCV_XFEATURES2D))
  m_mapOfDetectorNames[DETECTOR_STAR] = "STAR";
#endif
#if defined(VISP_HAVE_OPENCV_NONFREE) || defined(VISP_HAVE_OPENCV_XFEATURES2D)
  m_mapOfDetectorNames[DETECTOR_SIFT] = "SIFT";
  m_mapOfDetectorNames[DETECTOR_SURF] = "SURF";
#endif
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
  m_mapOfDetectorNames[DETECTOR_KAZE] = "KAZE";
  m_mapOfDetectorNames[DETECTOR_AKAZE] = "AKAZE";
  m_mapOfDetectorNames[DETECTOR_AGAST] = "AGAST";
#endif
#if (VISP_HAVE_OPENCV_VERSION >= 0x030100) && defined(VISP_HAVE_OPENCV_XFEATURES2D)
  m_mapOfDetectorNames[DETECTOR_MSD] = "MSD";
#endif
#endif

#if (VISP_HAVE_OPENCV_VERSION >= 0x020403)
  m_mapOfDescriptorNames[DESCRIPTOR_ORB] = "ORB";
  m_mapOfDescriptorNames[DESCRIPTOR_BRISK] = "BRISK";
#if (VISP_HAVE_OPENCV_VERSION < 0x030000) || (defined(VISP_HAVE_OPENCV_XFEATURES2D))
  m_mapOfDescriptorNames[DESCRIPTOR_FREAK] = "FREAK";
  m_mapOfDescriptorNames[DESCRIPTOR_BRIEF] = "BRIEF";
#endif
#if defined(VISP_HAVE_OPENCV_NONFREE) || defined(VISP_HAVE_OPENCV_XFEATURES2D)
  m_mapOfDescriptorNames[DESCRIPTOR_SIFT] = "SIFT";
  m_mapOfDescriptorNames[DESCRIPTOR_SURF] = "SURF";
#endif
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
  m_mapOfDescriptorNames[DESCRIPTOR_KAZE] = "KAZE";
  m_mapOfDescriptorNames[DESCRIPTOR_AKAZE] = "AKAZE";
#if defined(VISP_HAVE_OPENCV_XFEATURES2D)
  m_mapOfDescriptorNames[DESCRIPTOR_DAISY] = "DAISY";
  m_mapOfDescriptorNames[DESCRIPTOR_LATCH] = "LATCH";
#endif
#endif
#if (VISP_HAVE_OPENCV_VERSION >= 0x030200) && defined(VISP_HAVE_OPENCV_XFEATURES2D)
  m_mapOfDescriptorNames[DESCRIPTOR_VGG] = "VGG";
  m_mapOfDescriptorNames[DESCRIPTOR_BoostDesc] = "BoostDesc";
#endif
#endif
}

/*!
   Initialize a matcher based on its name.

   \param matcherName : Name of the matcher (e.g BruteForce, FlannBased).
 */
void vpKeyPoint::initMatcher(const std::string &matcherName)
{
  int descriptorType = CV_32F;
  bool firstIteration = true;
  for (std::map<std::string, cv::Ptr<cv::DescriptorExtractor> >::const_iterator it = m_extractors.begin();
       it != m_extractors.end(); ++it) {
    if (firstIteration) {
      firstIteration = false;
      descriptorType = it->second->descriptorType();
    } else {
      if (descriptorType != it->second->descriptorType()) {
        throw vpException(vpException::fatalError, "All the descriptors must have the same type !");
      }
    }
  }

  if (matcherName == "FlannBased") {
    if (m_extractors.empty()) {
      std::cout << "Warning: No extractor initialized, by default use "
                   "floating values (CV_32F) "
                   "for descriptor type !"
                << std::endl;
    }

    if (descriptorType == CV_8U) {
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
      m_matcher = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
#else
      m_matcher = new cv::FlannBasedMatcher(new cv::flann::LshIndexParams(12, 20, 2));
#endif
    } else {
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
      m_matcher = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::KDTreeIndexParams>());
#else
      m_matcher = new cv::FlannBasedMatcher(new cv::flann::KDTreeIndexParams());
#endif
    }
  } else {
    m_matcher = cv::DescriptorMatcher::create(matcherName);
  }

#if (VISP_HAVE_OPENCV_VERSION >= 0x020400 && VISP_HAVE_OPENCV_VERSION < 0x030000)
  if (m_matcher != NULL && !m_useKnn && matcherName == "BruteForce") {
    m_matcher->set("crossCheck", m_useBruteForceCrossCheck);
  }
#endif

  if (!m_matcher) { //if null
    std::stringstream ss_msg;
    ss_msg << "Fail to initialize the matcher: " << matcherName
           << " or it is not available in OpenCV version: " << std::hex << VISP_HAVE_OPENCV_VERSION << ".";
    throw vpException(vpException::fatalError, ss_msg.str());
  }
}

/*!
   Insert a reference image and a current image side-by-side.

   \param IRef : Reference image.
   \param ICurrent : Current image.
   \param IMatching : Matching image for displaying all the matching between
   the query keypoints and those detected in the training images.
 */
void vpKeyPoint::insertImageMatching(const vpImage<unsigned char> &IRef, const vpImage<unsigned char> &ICurrent,
                                     vpImage<unsigned char> &IMatching)
{
  vpImagePoint topLeftCorner(0, 0);
  IMatching.insert(IRef, topLeftCorner);
  topLeftCorner = vpImagePoint(0, IRef.getWidth());
  IMatching.insert(ICurrent, topLeftCorner);
}

/*!
   Insert a reference image and a current image side-by-side.

   \param IRef : Reference image.
   \param ICurrent : Current image.
   \param IMatching : Matching image for displaying all the matching between
   the query keypoints and those detected in the training images.
 */
void vpKeyPoint::insertImageMatching(const vpImage<vpRGBa> &IRef, const vpImage<vpRGBa> &ICurrent,
                                     vpImage<vpRGBa> &IMatching)
{
  vpImagePoint topLeftCorner(0, 0);
  IMatching.insert(IRef, topLeftCorner);
  topLeftCorner = vpImagePoint(0, IRef.getWidth());
  IMatching.insert(ICurrent, topLeftCorner);
}

/*!
   Insert the different training images in the matching image.

   \param ICurrent : Current image.
   \param IMatching : Matching image for displaying all the matching between
   the query keypoints and those detected in the training images
 */
void vpKeyPoint::insertImageMatching(const vpImage<unsigned char> &ICurrent, vpImage<unsigned char> &IMatching)
{
  // Nb images in the training database + the current image we want to detect
  // the object
  int nbImg = (int)(m_mapOfImages.size() + 1);

  if (m_mapOfImages.empty()) {
    std::cerr << "There is no training image loaded !" << std::endl;
    return;
  }

  if (nbImg == 2) {
    // Only one training image, so we display them side by side
    insertImageMatching(m_mapOfImages.begin()->second, ICurrent, IMatching);
  } else {
    // Multiple training images, display them as a mosaic image
    int nbImgSqrt = vpMath::round(std::sqrt((double)nbImg)); //(int) std::floor(std::sqrt((double) nbImg) + 0.5);
    int nbWidth = nbImgSqrt;
    int nbHeight = nbImgSqrt;

    if (nbImgSqrt * nbImgSqrt < nbImg) {
      nbWidth++;
    }

    unsigned int maxW = ICurrent.getWidth(), maxH = ICurrent.getHeight();
    for (std::map<int, vpImage<unsigned char> >::const_iterator it = m_mapOfImages.begin(); it != m_mapOfImages.end();
         ++it) {
      if (maxW < it->second.getWidth()) {
        maxW = it->second.getWidth();
      }

      if (maxH < it->second.getHeight()) {
        maxH = it->second.getHeight();
      }
    }

    // Indexes of the current image in the grid made in order to the image is
    // in the center square in the mosaic grid
    int medianI = nbHeight / 2;
    int medianJ = nbWidth / 2;
    int medianIndex = medianI * nbWidth + medianJ;

    int cpt = 0;
    for (std::map<int, vpImage<unsigned char> >::const_iterator it = m_mapOfImages.begin(); it != m_mapOfImages.end();
         ++it, cpt++) {
      int local_cpt = cpt;
      if (cpt >= medianIndex) {
        // Shift of one unity the index of the training images which are after
        // the current image
        local_cpt++;
      }
      int indexI = local_cpt / nbWidth;
      int indexJ = local_cpt - (indexI * nbWidth);
      vpImagePoint topLeftCorner((int)maxH * indexI, (int)maxW * indexJ);

      IMatching.insert(it->second, topLeftCorner);
    }

    vpImagePoint topLeftCorner((int)maxH * medianI, (int)maxW * medianJ);
    IMatching.insert(ICurrent, topLeftCorner);
  }
}

/*!
   Insert the different training images in the matching image.

   \param ICurrent : Current image.
   \param IMatching : Matching image for displaying all the matching between
   the query keypoints and those detected in the training images
 */
void vpKeyPoint::insertImageMatching(const vpImage<vpRGBa> &ICurrent, vpImage<vpRGBa> &IMatching)
{
  // Nb images in the training database + the current image we want to detect
  // the object
  int nbImg = (int)(m_mapOfImages.size() + 1);

  if (m_mapOfImages.empty()) {
    std::cerr << "There is no training image loaded !" << std::endl;
    return;
  }

  if (nbImg == 2) {
    // Only one training image, so we display them side by side
    vpImage<vpRGBa> IRef;
    vpImageConvert::convert(m_mapOfImages.begin()->second, IRef);
    insertImageMatching(IRef, ICurrent, IMatching);
  } else {
    // Multiple training images, display them as a mosaic image
    int nbImgSqrt = vpMath::round(std::sqrt((double)nbImg)); //(int) std::floor(std::sqrt((double) nbImg) + 0.5);
    int nbWidth = nbImgSqrt;
    int nbHeight = nbImgSqrt;

    if (nbImgSqrt * nbImgSqrt < nbImg) {
      nbWidth++;
    }

    unsigned int maxW = ICurrent.getWidth(), maxH = ICurrent.getHeight();
    for (std::map<int, vpImage<unsigned char> >::const_iterator it = m_mapOfImages.begin(); it != m_mapOfImages.end();
         ++it) {
      if (maxW < it->second.getWidth()) {
        maxW = it->second.getWidth();
      }

      if (maxH < it->second.getHeight()) {
        maxH = it->second.getHeight();
      }
    }

    // Indexes of the current image in the grid made in order to the image is
    // in the center square in the mosaic grid
    int medianI = nbHeight / 2;
    int medianJ = nbWidth / 2;
    int medianIndex = medianI * nbWidth + medianJ;

    int cpt = 0;
    for (std::map<int, vpImage<unsigned char> >::const_iterator it = m_mapOfImages.begin(); it != m_mapOfImages.end();
         ++it, cpt++) {
      int local_cpt = cpt;
      if (cpt >= medianIndex) {
        // Shift of one unity the index of the training images which are after
        // the current image
        local_cpt++;
      }
      int indexI = local_cpt / nbWidth;
      int indexJ = local_cpt - (indexI * nbWidth);
      vpImagePoint topLeftCorner((int)maxH * indexI, (int)maxW * indexJ);

      vpImage<vpRGBa> IRef;
      vpImageConvert::convert(it->second, IRef);
      IMatching.insert(IRef, topLeftCorner);
    }

    vpImagePoint topLeftCorner((int)maxH * medianI, (int)maxW * medianJ);
    IMatching.insert(ICurrent, topLeftCorner);
  }
}

#ifdef VISP_HAVE_XML2
/*!
   Load configuration parameters from an XML config file.

   \param configFile : Path to the XML config file.
 */
void vpKeyPoint::loadConfigFile(const std::string &configFile)
{
  vpXmlConfigParserKeyPoint xmlp;

  try {
    // Reset detector and extractor
    m_detectorNames.clear();
    m_extractorNames.clear();
    m_detectors.clear();
    m_extractors.clear();

    std::cout << " *********** Parsing XML for configuration for vpKeyPoint "
                 "************ "
              << std::endl;
    xmlp.parse(configFile);

    m_detectorNames.push_back(xmlp.getDetectorName());
    m_extractorNames.push_back(xmlp.getExtractorName());
    m_matcherName = xmlp.getMatcherName();

    switch (xmlp.getMatchingMethod()) {
    case vpXmlConfigParserKeyPoint::constantFactorDistanceThreshold:
      m_filterType = constantFactorDistanceThreshold;
      break;

    case vpXmlConfigParserKeyPoint::stdDistanceThreshold:
      m_filterType = stdDistanceThreshold;
      break;

    case vpXmlConfigParserKeyPoint::ratioDistanceThreshold:
      m_filterType = ratioDistanceThreshold;
      break;

    case vpXmlConfigParserKeyPoint::stdAndRatioDistanceThreshold:
      m_filterType = stdAndRatioDistanceThreshold;
      break;

    case vpXmlConfigParserKeyPoint::noFilterMatching:
      m_filterType = noFilterMatching;
      break;

    default:
      break;
    }

    m_matchingFactorThreshold = xmlp.getMatchingFactorThreshold();
    m_matchingRatioThreshold = xmlp.getMatchingRatioThreshold();

    m_useRansacVVS = xmlp.getUseRansacVVSPoseEstimation();
    m_useConsensusPercentage = xmlp.getUseRansacConsensusPercentage();
    m_nbRansacIterations = xmlp.getNbRansacIterations();
    m_ransacReprojectionError = xmlp.getRansacReprojectionError();
    m_nbRansacMinInlierCount = xmlp.getNbRansacMinInlierCount();
    m_ransacThreshold = xmlp.getRansacThreshold();
    m_ransacConsensusPercentage = xmlp.getRansacConsensusPercentage();

    if (m_filterType == ratioDistanceThreshold || m_filterType == stdAndRatioDistanceThreshold) {
      m_useKnn = true;
    } else {
      m_useKnn = false;
    }

    init();
  } catch (...) {
    throw vpException(vpException::ioError, "Can't open XML file \"%s\"\n ", configFile.c_str());
  }
}
#endif

/*!
   Load learning data saved on disk.

   \param filename : Path of the learning file.
   \param binaryMode : If true, the learning file is in a binary mode,
   otherwise it is in XML mode. \param append : If true, concatenate the
   learning data, otherwise reset the variables.
 */
void vpKeyPoint::loadLearningData(const std::string &filename, const bool binaryMode, const bool append)
{
  int startClassId = 0;
  int startImageId = 0;
  if (!append) {
    m_trainKeyPoints.clear();
    m_trainPoints.clear();
    m_mapOfImageId.clear();
    m_mapOfImages.clear();
  } else {
    // In append case, find the max index of keypoint class Id
    for (std::map<int, int>::const_iterator it = m_mapOfImageId.begin(); it != m_mapOfImageId.end(); ++it) {
      if (startClassId < it->first) {
        startClassId = it->first;
      }
    }

    // In append case, find the max index of images Id
    for (std::map<int, vpImage<unsigned char> >::const_iterator it = m_mapOfImages.begin(); it != m_mapOfImages.end();
         ++it) {
      if (startImageId < it->first) {
        startImageId = it->first;
      }
    }
  }

  // Get parent directory
  std::string parent = vpIoTools::getParent(filename);
  if (!parent.empty()) {
    parent += "/";
  }

  if (binaryMode) {
    std::ifstream file(filename.c_str(), std::ifstream::binary);
    if (!file.is_open()) {
      throw vpException(vpException::ioError, "Cannot open the file.");
    }

    // Read info about training images
    int nbImgs = 0;
    vpIoTools::readBinaryValueLE(file, nbImgs);

#if !defined(VISP_HAVE_MODULE_IO)
    if (nbImgs > 0) {
      std::cout << "Warning: The learning file contains image data that will "
                   "not be loaded as visp_io module "
                   "is not available !"
                << std::endl;
    }
#endif

    for (int i = 0; i < nbImgs; i++) {
      // Read image_id
      int id = 0;
      vpIoTools::readBinaryValueLE(file, id);

      int length = 0;
      vpIoTools::readBinaryValueLE(file, length);
      // Will contain the path to the training images
      char *path = new char[length + 1]; // char path[length + 1];

      for (int cpt = 0; cpt < length; cpt++) {
        char c;
        file.read((char *)(&c), sizeof(c));
        path[cpt] = c;
      }
      path[length] = '\0';

      vpImage<unsigned char> I;
#ifdef VISP_HAVE_MODULE_IO
      if (vpIoTools::isAbsolutePathname(std::string(path))) {
        vpImageIo::read(I, path);
      } else {
        vpImageIo::read(I, parent + path);
      }

      // Add the image previously loaded only if VISP_HAVE_MODULE_IO
      m_mapOfImages[id + startImageId] = I;
#endif

      // Delete path
      delete[] path;
    }

    // Read if 3D point information are saved or not
    int have3DInfoInt = 0;
    vpIoTools::readBinaryValueLE(file, have3DInfoInt);
    bool have3DInfo = have3DInfoInt != 0;

    // Read the number of descriptors
    int nRows = 0;
    vpIoTools::readBinaryValueLE(file, nRows);

    // Read the size of the descriptor
    int nCols = 0;
    vpIoTools::readBinaryValueLE(file, nCols);

    // Read the type of the descriptor
    int descriptorType = 5; // CV_32F
    vpIoTools::readBinaryValueLE(file, descriptorType);

    cv::Mat trainDescriptorsTmp = cv::Mat(nRows, nCols, descriptorType);
    for (int i = 0; i < nRows; i++) {
      // Read information about keyPoint
      float u, v, size, angle, response;
      int octave, class_id, image_id;
      vpIoTools::readBinaryValueLE(file, u);
      vpIoTools::readBinaryValueLE(file, v);
      vpIoTools::readBinaryValueLE(file, size);
      vpIoTools::readBinaryValueLE(file, angle);
      vpIoTools::readBinaryValueLE(file, response);
      vpIoTools::readBinaryValueLE(file, octave);
      vpIoTools::readBinaryValueLE(file, class_id);
      vpIoTools::readBinaryValueLE(file, image_id);
      cv::KeyPoint keyPoint(cv::Point2f(u, v), size, angle, response, octave, (class_id + startClassId));
      m_trainKeyPoints.push_back(keyPoint);

      if (image_id != -1) {
#ifdef VISP_HAVE_MODULE_IO
        // No training images if image_id == -1
        m_mapOfImageId[m_trainKeyPoints.back().class_id] = image_id + startImageId;
#endif
      }

      if (have3DInfo) {
        // Read oX, oY, oZ
        float oX, oY, oZ;
        vpIoTools::readBinaryValueLE(file, oX);
        vpIoTools::readBinaryValueLE(file, oY);
        vpIoTools::readBinaryValueLE(file, oZ);
        m_trainPoints.push_back(cv::Point3f(oX, oY, oZ));
      }

      for (int j = 0; j < nCols; j++) {
        // Read the value of the descriptor
        switch (descriptorType) {
        case CV_8U: {
          unsigned char value;
          file.read((char *)(&value), sizeof(value));
          trainDescriptorsTmp.at<unsigned char>(i, j) = value;
        } break;

        case CV_8S: {
          char value;
          file.read((char *)(&value), sizeof(value));
          trainDescriptorsTmp.at<char>(i, j) = value;
        } break;

        case CV_16U: {
          unsigned short int value;
          vpIoTools::readBinaryValueLE(file, value);
          trainDescriptorsTmp.at<unsigned short int>(i, j) = value;
        } break;

        case CV_16S: {
          short int value;
          vpIoTools::readBinaryValueLE(file, value);
          trainDescriptorsTmp.at<short int>(i, j) = value;
        } break;

        case CV_32S: {
          int value;
          vpIoTools::readBinaryValueLE(file, value);
          trainDescriptorsTmp.at<int>(i, j) = value;
        } break;

        case CV_32F: {
          float value;
          vpIoTools::readBinaryValueLE(file, value);
          trainDescriptorsTmp.at<float>(i, j) = value;
        } break;

        case CV_64F: {
          double value;
          vpIoTools::readBinaryValueLE(file, value);
          trainDescriptorsTmp.at<double>(i, j) = value;
        } break;

        default: {
          float value;
          vpIoTools::readBinaryValueLE(file, value);
          trainDescriptorsTmp.at<float>(i, j) = value;
        } break;
        }
      }
    }

    if (!append || m_trainDescriptors.empty()) {
      trainDescriptorsTmp.copyTo(m_trainDescriptors);
    } else {
      cv::vconcat(m_trainDescriptors, trainDescriptorsTmp, m_trainDescriptors);
    }

    file.close();
  } else {
#ifdef VISP_HAVE_XML2
    xmlDocPtr doc = NULL;
    xmlNodePtr root_element = NULL;

    /*
     * this initialize the library and check potential ABI mismatches
     * between the version it was compiled for and the actual shared
     * library used.
     */
    LIBXML_TEST_VERSION

    /*parse the file and get the DOM */
    doc = xmlReadFile(filename.c_str(), NULL, 0);

    if (doc == NULL) {
      throw vpException(vpException::ioError, "Error with file " + filename);
    }

    root_element = xmlDocGetRootElement(doc);

    xmlNodePtr first_level_node = NULL;
    char *pEnd = NULL;

    int descriptorType = CV_32F; // float
    int nRows = 0, nCols = 0;
    int i = 0, j = 0;

    cv::Mat trainDescriptorsTmp;

    for (first_level_node = root_element->children; first_level_node; first_level_node = first_level_node->next) {

      std::string name((char *)first_level_node->name);
      if (first_level_node->type == XML_ELEMENT_NODE && name == "TrainingImageInfo") {
        xmlNodePtr image_info_node = NULL;

        for (image_info_node = first_level_node->children; image_info_node; image_info_node = image_info_node->next) {
          name = std::string((char *)image_info_node->name);

          if (name == "trainImg") {
            // Read image_id
            xmlChar *image_id_property = xmlGetProp(image_info_node, BAD_CAST "image_id");
            int id = 0;
            if (image_id_property) {
              id = std::atoi((char *)image_id_property);
            }
            xmlFree(image_id_property);

            vpImage<unsigned char> I;
#ifdef VISP_HAVE_MODULE_IO
            std::string path((char *)image_info_node->children->content);
            // Read path to the training images
            if (vpIoTools::isAbsolutePathname(std::string(path))) {
              vpImageIo::read(I, path);
            } else {
              vpImageIo::read(I, parent + path);
            }

            // Add the image previously loaded only if VISP_HAVE_MODULE_IO
            m_mapOfImages[id + startImageId] = I;
#endif
          }
        }
      } else if (first_level_node->type == XML_ELEMENT_NODE && name == "DescriptorsInfo") {
        xmlNodePtr descriptors_info_node = NULL;
        for (descriptors_info_node = first_level_node->children; descriptors_info_node;
             descriptors_info_node = descriptors_info_node->next) {
          if (descriptors_info_node->type == XML_ELEMENT_NODE) {
            name = std::string((char *)descriptors_info_node->name);

            if (name == "nrows") {
              nRows = std::atoi((char *)descriptors_info_node->children->content);
            } else if (name == "ncols") {
              nCols = std::atoi((char *)descriptors_info_node->children->content);
            } else if (name == "type") {
              descriptorType = std::atoi((char *)descriptors_info_node->children->content);
            }
          }
        }

        trainDescriptorsTmp = cv::Mat(nRows, nCols, descriptorType);
      } else if (first_level_node->type == XML_ELEMENT_NODE && name == "DescriptorInfo") {
        xmlNodePtr point_node = NULL;
        double u = 0.0, v = 0.0, size = 0.0, angle = 0.0, response = 0.0;
        int octave = 0, class_id = 0, image_id = 0;
        double oX = 0.0, oY = 0.0, oZ = 0.0;

        std::stringstream ss;

        for (point_node = first_level_node->children; point_node; point_node = point_node->next) {
          if (point_node->type == XML_ELEMENT_NODE) {
            name = std::string((char *)point_node->name);

            // Read information about keypoints
            if (name == "u") {
              u = std::strtod((char *)point_node->children->content, &pEnd);
            } else if (name == "v") {
              v = std::strtod((char *)point_node->children->content, &pEnd);
            } else if (name == "size") {
              size = std::strtod((char *)point_node->children->content, &pEnd);
            } else if (name == "angle") {
              angle = std::strtod((char *)point_node->children->content, &pEnd);
            } else if (name == "response") {
              response = std::strtod((char *)point_node->children->content, &pEnd);
            } else if (name == "octave") {
              octave = std::atoi((char *)point_node->children->content);
            } else if (name == "class_id") {
              class_id = std::atoi((char *)point_node->children->content);
              cv::KeyPoint keyPoint(cv::Point2f((float)u, (float)v), (float)size, (float)angle, (float)response, octave,
                                    (class_id + startClassId));
              m_trainKeyPoints.push_back(keyPoint);
            } else if (name == "image_id") {
              image_id = std::atoi((char *)point_node->children->content);
              if (image_id != -1) {
#ifdef VISP_HAVE_MODULE_IO
                // No training images if image_id == -1
                m_mapOfImageId[m_trainKeyPoints.back().class_id] = image_id + startImageId;
#endif
              }
            } else if (name == "oX") {
              oX = std::atof((char *)point_node->children->content);
            } else if (name == "oY") {
              oY = std::atof((char *)point_node->children->content);
            } else if (name == "oZ") {
              oZ = std::atof((char *)point_node->children->content);
              m_trainPoints.push_back(cv::Point3f((float)oX, (float)oY, (float)oZ));
            } else if (name == "desc") {
              xmlNodePtr descriptor_value_node = NULL;
              j = 0;

              for (descriptor_value_node = point_node->children; descriptor_value_node;
                   descriptor_value_node = descriptor_value_node->next) {

                if (descriptor_value_node->type == XML_ELEMENT_NODE) {
                  // Read descriptors values
                  std::string parseStr((char *)descriptor_value_node->children->content);
                  ss.clear();
                  ss.str(parseStr);

                  if (!ss.fail()) {
                    switch (descriptorType) {
                    case CV_8U: {
                      // Parse the numeric value [0 ; 255] to an int
                      int parseValue;
                      ss >> parseValue;
                      trainDescriptorsTmp.at<unsigned char>(i, j) = (unsigned char)parseValue;
                    } break;

                    case CV_8S:
                      // Parse the numeric value [-128 ; 127] to an int
                      int parseValue;
                      ss >> parseValue;
                      trainDescriptorsTmp.at<char>(i, j) = (char)parseValue;
                      break;

                    case CV_16U:
                      ss >> trainDescriptorsTmp.at<unsigned short int>(i, j);
                      break;

                    case CV_16S:
                      ss >> trainDescriptorsTmp.at<short int>(i, j);
                      break;

                    case CV_32S:
                      ss >> trainDescriptorsTmp.at<int>(i, j);
                      break;

                    case CV_32F:
                      ss >> trainDescriptorsTmp.at<float>(i, j);
                      break;

                    case CV_64F:
                      ss >> trainDescriptorsTmp.at<double>(i, j);
                      break;

                    default:
                      ss >> trainDescriptorsTmp.at<float>(i, j);
                      break;
                    }
                  } else {
                    std::cerr << "Error when converting:" << ss.str() << std::endl;
                  }

                  j++;
                }
              }
            }
          }
        }
        i++;
      }
    }

    if (!append || m_trainDescriptors.empty()) {
      trainDescriptorsTmp.copyTo(m_trainDescriptors);
    } else {
      cv::vconcat(m_trainDescriptors, trainDescriptorsTmp, m_trainDescriptors);
    }

    /*free the document */
    xmlFreeDoc(doc);

    /*
     *Free the global variables that may
     *have been allocated by the parser.
     */
    xmlCleanupParser();
#else
    std::cout << "Error: libxml2 is required !" << std::endl;
#endif
  }

  // Convert OpenCV type to ViSP type for compatibility
  vpConvert::convertFromOpenCV(m_trainKeyPoints, referenceImagePointsList);
  vpConvert::convertFromOpenCV(this->m_trainPoints, m_trainVpPoints);

  // Add train descriptors in matcher object
  m_matcher->clear();
  m_matcher->add(std::vector<cv::Mat>(1, m_trainDescriptors));

  // Set _reference_computed to true as we load a learning file
  _reference_computed = true;

  // Set m_currentImageId
  m_currentImageId = (int)m_mapOfImages.size();
}

/*!
   Match keypoints based on distance between their descriptors.

   \param trainDescriptors : Train descriptors (or reference descriptors).
   \param queryDescriptors : Query descriptors.
   \param matches : Output list of matches.
   \param elapsedTime : Elapsed time.
 */
void vpKeyPoint::match(const cv::Mat &trainDescriptors, const cv::Mat &queryDescriptors,
                       std::vector<cv::DMatch> &matches, double &elapsedTime)
{
  double t = vpTime::measureTimeMs();

  if (m_useKnn) {
    m_knnMatches.clear();

    if (m_useMatchTrainToQuery) {
      std::vector<std::vector<cv::DMatch> > knnMatchesTmp;

      // Match train descriptors to query descriptors
      cv::Ptr<cv::DescriptorMatcher> matcherTmp = m_matcher->clone(true);
      matcherTmp->knnMatch(trainDescriptors, queryDescriptors, knnMatchesTmp, 2);

      for (std::vector<std::vector<cv::DMatch> >::const_iterator it1 = knnMatchesTmp.begin();
           it1 != knnMatchesTmp.end(); ++it1) {
        std::vector<cv::DMatch> tmp;
        for (std::vector<cv::DMatch>::const_iterator it2 = it1->begin(); it2 != it1->end(); ++it2) {
          tmp.push_back(cv::DMatch(it2->trainIdx, it2->queryIdx, it2->distance));
        }
        m_knnMatches.push_back(tmp);
      }

      matches.resize(m_knnMatches.size());
      std::transform(m_knnMatches.begin(), m_knnMatches.end(), matches.begin(), knnToDMatch);
    } else {
      // Match query descriptors to train descriptors
      m_matcher->knnMatch(queryDescriptors, m_knnMatches, 2);
      matches.resize(m_knnMatches.size());
      std::transform(m_knnMatches.begin(), m_knnMatches.end(), matches.begin(), knnToDMatch);
    }
  } else {
    matches.clear();

    if (m_useMatchTrainToQuery) {
      std::vector<cv::DMatch> matchesTmp;
      // Match train descriptors to query descriptors
      cv::Ptr<cv::DescriptorMatcher> matcherTmp = m_matcher->clone(true);
      matcherTmp->match(trainDescriptors, queryDescriptors, matchesTmp);

      for (std::vector<cv::DMatch>::const_iterator it = matchesTmp.begin(); it != matchesTmp.end(); ++it) {
        matches.push_back(cv::DMatch(it->trainIdx, it->queryIdx, it->distance));
      }
    } else {
      // Match query descriptors to train descriptors
      m_matcher->match(queryDescriptors, matches);
    }
  }
  elapsedTime = vpTime::measureTimeMs() - t;
}

/*!
   Match keypoints detected in the image with those built in the reference
   list.

   \param I : Input current image.
   \return The number of matched keypoints.
 */
unsigned int vpKeyPoint::matchPoint(const vpImage<unsigned char> &I) { return matchPoint(I, vpRect()); }

/*!
   Match keypoints detected in the image with those built in the reference
   list.

   \param I_color : Input current image.
   \return The number of matched keypoints.
 */
unsigned int vpKeyPoint::matchPoint(const vpImage<vpRGBa> &I_color) { return matchPoint(I_color, vpRect()); }

/*!
   Match keypoints detected in a region of interest of the image with those
   built in the reference list.

   \param I : Input image
   \param iP : Coordinate of the top-left corner of the region of interest
   \param height : Height of the region of interest
   \param width : Width of the region of interest
   \return The number of matched keypoints
 */
unsigned int vpKeyPoint::matchPoint(const vpImage<unsigned char> &I, const vpImagePoint &iP, const unsigned int height,
                                    const unsigned int width)
{
  return matchPoint(I, vpRect(iP, width, height));
}

/*!
   Match keypoints detected in a region of interest of the image with those
   built in the reference list.

   \param I_color : Input image
   \param iP : Coordinate of the top-left corner of the region of interest
   \param height : Height of the region of interest
   \param width : Width of the region of interest
   \return The number of matched keypoints
 */
unsigned int vpKeyPoint::matchPoint(const vpImage<vpRGBa> &I_color, const vpImagePoint &iP, const unsigned int height,
                                    const unsigned int width)
{
  return matchPoint(I_color, vpRect(iP, width, height));
}

/*!
   Match keypoints detected in a region of interest of the image with those
   built in the reference list.

   \param I : Input image
   \param rectangle : Rectangle of the region of interest
   \return The number of matched keypoints
 */
unsigned int vpKeyPoint::matchPoint(const vpImage<unsigned char> &I, const vpRect &rectangle)
{
  if (m_trainDescriptors.empty()) {
    std::cerr << "Reference is empty." << std::endl;
    if (!_reference_computed) {
      std::cerr << "Reference is not computed." << std::endl;
    }
    std::cerr << "Matching is not possible." << std::endl;

    return 0;
  }

  if (m_useAffineDetection) {
    std::vector<std::vector<cv::KeyPoint> > listOfQueryKeyPoints;
    std::vector<cv::Mat> listOfQueryDescriptors;

    // Detect keypoints and extract descriptors on multiple images
    detectExtractAffine(I, listOfQueryKeyPoints, listOfQueryDescriptors);

    // Flatten the different train lists
    m_queryKeyPoints.clear();
    for (std::vector<std::vector<cv::KeyPoint> >::const_iterator it = listOfQueryKeyPoints.begin();
         it != listOfQueryKeyPoints.end(); ++it) {
      m_queryKeyPoints.insert(m_queryKeyPoints.end(), it->begin(), it->end());
    }

    bool first = true;
    for (std::vector<cv::Mat>::const_iterator it = listOfQueryDescriptors.begin(); it != listOfQueryDescriptors.end();
         ++it) {
      if (first) {
        first = false;
        it->copyTo(m_queryDescriptors);
      } else {
        m_queryDescriptors.push_back(*it);
      }
    }
  } else {
    detect(I, m_queryKeyPoints, m_detectionTime, rectangle);
    extract(I, m_queryKeyPoints, m_queryDescriptors, m_extractionTime);
  }

  match(m_trainDescriptors, m_queryDescriptors, m_matches, m_matchingTime);

  if (m_filterType != noFilterMatching) {
    m_queryFilteredKeyPoints.clear();
    m_objectFilteredPoints.clear();
    m_filteredMatches.clear();

    filterMatches();
  } else {
    if (m_useMatchTrainToQuery) {
      // Add only query keypoints matched with a train keypoints
      m_queryFilteredKeyPoints.clear();
      m_filteredMatches.clear();
      for (std::vector<cv::DMatch>::const_iterator it = m_matches.begin(); it != m_matches.end(); ++it) {
        m_filteredMatches.push_back(cv::DMatch((int)m_queryFilteredKeyPoints.size(), it->trainIdx, it->distance));
        m_queryFilteredKeyPoints.push_back(m_queryKeyPoints[(size_t)it->queryIdx]);
      }
    } else {
      m_queryFilteredKeyPoints = m_queryKeyPoints;
      m_filteredMatches = m_matches;
    }

    if (!m_trainPoints.empty()) {
      m_objectFilteredPoints.clear();
      // Add 3D object points such as the same index in
      // m_queryFilteredKeyPoints and in m_objectFilteredPoints
      // matches to the same train object
      for (std::vector<cv::DMatch>::const_iterator it = m_matches.begin(); it != m_matches.end(); ++it) {
        // m_matches is normally ordered following the queryDescriptor index
        m_objectFilteredPoints.push_back(m_trainPoints[(size_t)it->trainIdx]);
      }
    }
  }

  // Convert OpenCV type to ViSP type for compatibility
  vpConvert::convertFromOpenCV(m_queryFilteredKeyPoints, currentImagePointsList);
  vpConvert::convertFromOpenCV(m_filteredMatches, matchedReferencePoints);

  return static_cast<unsigned int>(m_filteredMatches.size());
}

/*!
   Match keypoints detected in a region of interest of the image with those
   built in the reference list.

   \param I_color : Input image
   \param rectangle : Rectangle of the region of interest
   \return The number of matched keypoints
 */
unsigned int vpKeyPoint::matchPoint(const vpImage<vpRGBa> &I_color, const vpRect &rectangle)
{
  vpImageConvert::convert(I_color, m_I);
  return matchPoint(m_I, rectangle);
}

/*!
   Match keypoints detected in the image with those built in the reference
   list and compute the pose.

   \param I : Input image
   \param cam : Camera parameters
   \param cMo : Homogeneous matrix between the object frame and the camera frame
   \param func : Function pointer to filter the pose in Ransac pose
   estimation, if we want to eliminate the poses which do not respect some criterion
   \param rectangle : Rectangle corresponding to the ROI (Region of Interest) to consider
   \return True if the matching and the pose estimation are OK, false otherwise
 */
bool vpKeyPoint::matchPoint(const vpImage<unsigned char> &I, const vpCameraParameters &cam, vpHomogeneousMatrix &cMo,
                            bool (*func)(const vpHomogeneousMatrix &), const vpRect &rectangle)
{
  double error, elapsedTime;
  return matchPoint(I, cam, cMo, error, elapsedTime, func, rectangle);
}

/*!
   Match keypoints detected in the image with those built in the reference
   list and compute the pose.

   \param I_color : Input image
   \param cam : Camera parameters
   \param cMo : Homogeneous matrix between the object frame and the camera frame
   \param func : Function pointer to filter the pose in Ransac pose
   estimation, if we want to eliminate the poses which do not respect some criterion
   \param rectangle : Rectangle corresponding to the ROI (Region of Interest) to consider
   \return True if the matching and the pose estimation are OK, false otherwise
 */
bool vpKeyPoint::matchPoint(const vpImage<vpRGBa> &I_color, const vpCameraParameters &cam, vpHomogeneousMatrix &cMo,
                            bool (*func)(const vpHomogeneousMatrix &), const vpRect &rectangle)
{
  double error, elapsedTime;
  return matchPoint(I_color, cam, cMo, error, elapsedTime, func, rectangle);
}

/*!
   Match keypoints detected in the image with those built in the reference
   list and compute the pose.

   \param I : Input image
   \param cam : Camera parameters
   \param cMo : Homogeneous matrix between the object frame and the camera frame
   \param error : Reprojection mean square error (in pixel) between the
   2D points and the projection of the 3D points with the estimated pose
   \param elapsedTime : Time to detect, extract, match and compute the pose
   \param func : Function pointer to filter the pose in Ransac pose
   estimation, if we want to eliminate the poses which do not respect some criterion
   \param rectangle : Rectangle corresponding to the ROI (Region of Interest) to consider
   \return True if the matching and the pose estimation are OK, false otherwise
 */
bool vpKeyPoint::matchPoint(const vpImage<unsigned char> &I, const vpCameraParameters &cam, vpHomogeneousMatrix &cMo,
                            double &error, double &elapsedTime, bool (*func)(const vpHomogeneousMatrix &),
                            const vpRect &rectangle)
{
  // Check if we have training descriptors
  if (m_trainDescriptors.empty()) {
    std::cerr << "Reference is empty." << std::endl;
    if (!_reference_computed) {
      std::cerr << "Reference is not computed." << std::endl;
    }
    std::cerr << "Matching is not possible." << std::endl;

    return false;
  }

  if (m_useAffineDetection) {
    std::vector<std::vector<cv::KeyPoint> > listOfQueryKeyPoints;
    std::vector<cv::Mat> listOfQueryDescriptors;

    // Detect keypoints and extract descriptors on multiple images
    detectExtractAffine(I, listOfQueryKeyPoints, listOfQueryDescriptors);

    // Flatten the different train lists
    m_queryKeyPoints.clear();
    for (std::vector<std::vector<cv::KeyPoint> >::const_iterator it = listOfQueryKeyPoints.begin();
         it != listOfQueryKeyPoints.end(); ++it) {
      m_queryKeyPoints.insert(m_queryKeyPoints.end(), it->begin(), it->end());
    }

    bool first = true;
    for (std::vector<cv::Mat>::const_iterator it = listOfQueryDescriptors.begin(); it != listOfQueryDescriptors.end();
         ++it) {
      if (first) {
        first = false;
        it->copyTo(m_queryDescriptors);
      } else {
        m_queryDescriptors.push_back(*it);
      }
    }
  } else {
    detect(I, m_queryKeyPoints, m_detectionTime, rectangle);
    extract(I, m_queryKeyPoints, m_queryDescriptors, m_extractionTime);
  }

  match(m_trainDescriptors, m_queryDescriptors, m_matches, m_matchingTime);

  elapsedTime = m_detectionTime + m_extractionTime + m_matchingTime;

  if (m_filterType != noFilterMatching) {
    m_queryFilteredKeyPoints.clear();
    m_objectFilteredPoints.clear();
    m_filteredMatches.clear();

    filterMatches();
  } else {
    if (m_useMatchTrainToQuery) {
      // Add only query keypoints matched with a train keypoints
      m_queryFilteredKeyPoints.clear();
      m_filteredMatches.clear();
      for (std::vector<cv::DMatch>::const_iterator it = m_matches.begin(); it != m_matches.end(); ++it) {
        m_filteredMatches.push_back(cv::DMatch((int)m_queryFilteredKeyPoints.size(), it->trainIdx, it->distance));
        m_queryFilteredKeyPoints.push_back(m_queryKeyPoints[(size_t)it->queryIdx]);
      }
    } else {
      m_queryFilteredKeyPoints = m_queryKeyPoints;
      m_filteredMatches = m_matches;
    }

    if (!m_trainPoints.empty()) {
      m_objectFilteredPoints.clear();
      // Add 3D object points such as the same index in
      // m_queryFilteredKeyPoints and in m_objectFilteredPoints
      // matches to the same train object
      for (std::vector<cv::DMatch>::const_iterator it = m_matches.begin(); it != m_matches.end(); ++it) {
        // m_matches is normally ordered following the queryDescriptor index
        m_objectFilteredPoints.push_back(m_trainPoints[(size_t)it->trainIdx]);
      }
    }
  }

  // Convert OpenCV type to ViSP type for compatibility
  vpConvert::convertFromOpenCV(m_queryFilteredKeyPoints, currentImagePointsList);
  vpConvert::convertFromOpenCV(m_filteredMatches, matchedReferencePoints);

  // error = std::numeric_limits<double>::max(); // create an error under
  // Windows. To fix it we have to add #undef max
  error = DBL_MAX;
  m_ransacInliers.clear();
  m_ransacOutliers.clear();

  if (m_useRansacVVS) {
    std::vector<vpPoint> objectVpPoints(m_objectFilteredPoints.size());
    size_t cpt = 0;
    // Create a list of vpPoint with 2D coordinates (current keypoint
    // location) + 3D coordinates (world/object coordinates)
    for (std::vector<cv::Point3f>::const_iterator it = m_objectFilteredPoints.begin();
         it != m_objectFilteredPoints.end(); ++it, cpt++) {
      vpPoint pt;
      pt.setWorldCoordinates(it->x, it->y, it->z);

      vpImagePoint imP(m_queryFilteredKeyPoints[cpt].pt.y, m_queryFilteredKeyPoints[cpt].pt.x);

      double x = 0.0, y = 0.0;
      vpPixelMeterConversion::convertPoint(cam, imP, x, y);
      pt.set_x(x);
      pt.set_y(y);

      objectVpPoints[cpt] = pt;
    }

    std::vector<vpPoint> inliers;
    std::vector<unsigned int> inlierIndex;

    bool res = computePose(objectVpPoints, cMo, inliers, inlierIndex, m_poseTime, func);

    std::map<unsigned int, bool> mapOfInlierIndex;
    m_matchRansacKeyPointsToPoints.clear();

    for (std::vector<unsigned int>::const_iterator it = inlierIndex.begin(); it != inlierIndex.end(); ++it) {
      m_matchRansacKeyPointsToPoints.push_back(std::pair<cv::KeyPoint, cv::Point3f>(
          m_queryFilteredKeyPoints[(size_t)(*it)], m_objectFilteredPoints[(size_t)(*it)]));
      mapOfInlierIndex[*it] = true;
    }

    for (size_t i = 0; i < m_queryFilteredKeyPoints.size(); i++) {
      if (mapOfInlierIndex.find((unsigned int)i) == mapOfInlierIndex.end()) {
        m_ransacOutliers.push_back(vpImagePoint(m_queryFilteredKeyPoints[i].pt.y, m_queryFilteredKeyPoints[i].pt.x));
      }
    }

    error = computePoseEstimationError(m_matchRansacKeyPointsToPoints, cam, cMo);

    m_ransacInliers.resize(m_matchRansacKeyPointsToPoints.size());
    std::transform(m_matchRansacKeyPointsToPoints.begin(), m_matchRansacKeyPointsToPoints.end(),
                   m_ransacInliers.begin(), matchRansacToVpImage);

    elapsedTime += m_poseTime;

    return res;
  } else {
    std::vector<cv::Point2f> imageFilteredPoints;
    cv::KeyPoint::convert(m_queryFilteredKeyPoints, imageFilteredPoints);
    std::vector<int> inlierIndex;
    bool res = computePose(imageFilteredPoints, m_objectFilteredPoints, cam, cMo, inlierIndex, m_poseTime);

    std::map<int, bool> mapOfInlierIndex;
    m_matchRansacKeyPointsToPoints.clear();

    for (std::vector<int>::const_iterator it = inlierIndex.begin(); it != inlierIndex.end(); ++it) {
      m_matchRansacKeyPointsToPoints.push_back(std::pair<cv::KeyPoint, cv::Point3f>(
          m_queryFilteredKeyPoints[(size_t)(*it)], m_objectFilteredPoints[(size_t)(*it)]));
      mapOfInlierIndex[*it] = true;
    }

    for (size_t i = 0; i < m_queryFilteredKeyPoints.size(); i++) {
      if (mapOfInlierIndex.find((int)i) == mapOfInlierIndex.end()) {
        m_ransacOutliers.push_back(vpImagePoint(m_queryFilteredKeyPoints[i].pt.y, m_queryFilteredKeyPoints[i].pt.x));
      }
    }

    error = computePoseEstimationError(m_matchRansacKeyPointsToPoints, cam, cMo);

    m_ransacInliers.resize(m_matchRansacKeyPointsToPoints.size());
    std::transform(m_matchRansacKeyPointsToPoints.begin(), m_matchRansacKeyPointsToPoints.end(),
                   m_ransacInliers.begin(), matchRansacToVpImage);

    elapsedTime += m_poseTime;

    return res;
  }
}

/*!
   Match keypoints detected in the image with those built in the reference
   list and compute the pose.

   \param I_color : Input image
   \param cam : Camera parameters
   \param cMo : Homogeneous matrix between the object frame and the camera frame
   \param error : Reprojection mean square error (in pixel) between the
   2D points and the projection of the 3D points with the estimated pose
   \param elapsedTime : Time to detect, extract, match and compute the pose
   \param func : Function pointer to filter the pose in Ransac pose
   estimation, if we want to eliminate the poses which do not respect some criterion
   \param rectangle : Rectangle corresponding to the ROI (Region of Interest) to consider
   \return True if the matching and the pose estimation are OK, false otherwise
 */
bool vpKeyPoint::matchPoint(const vpImage<vpRGBa> &I_color, const vpCameraParameters &cam, vpHomogeneousMatrix &cMo,
                            double &error, double &elapsedTime, bool (*func)(const vpHomogeneousMatrix &),
                            const vpRect &rectangle)
{
  vpImageConvert::convert(I_color, m_I);
  return (matchPoint(m_I, cam, cMo, error, elapsedTime, func, rectangle));
}


/*!
   Match keypoints detected in the image with those built in the reference
   list and return the bounding box and the center of gravity.

   \param I : Input image
   \param boundingBox : Bounding box that contains the good matches
   \param centerOfGravity : Center of gravity computed from the location of
   the good matches (could differ of the center of the bounding box) \param
   isPlanarObject : If the object is planar, the homography matrix is
   estimated to eliminate outliers, otherwise it is the fundamental matrix
   which is estimated \param imPts1 : Pointer to the list of reference
   keypoints if not null \param imPts2 : Pointer to the list of current
   keypoints if not null \param meanDescriptorDistance : Pointer to the value
   of the average distance of the descriptors if not null \param
   detection_score : Pointer to the value of the detection score if not null
   \param rectangle : Rectangle corresponding to the ROI (Region of Interest)
   to consider \return True if the object is present, false otherwise
 */
bool vpKeyPoint::matchPointAndDetect(const vpImage<unsigned char> &I, vpRect &boundingBox,
                                     vpImagePoint &centerOfGravity, const bool isPlanarObject,
                                     std::vector<vpImagePoint> *imPts1, std::vector<vpImagePoint> *imPts2,
                                     double *meanDescriptorDistance, double *detection_score, const vpRect &rectangle)
{
  if (imPts1 != NULL && imPts2 != NULL) {
    imPts1->clear();
    imPts2->clear();
  }

  matchPoint(I, rectangle);

  double meanDescriptorDistanceTmp = 0.0;
  for (std::vector<cv::DMatch>::const_iterator it = m_filteredMatches.begin(); it != m_filteredMatches.end(); ++it) {
    meanDescriptorDistanceTmp += (double)it->distance;
  }

  meanDescriptorDistanceTmp /= (double)m_filteredMatches.size();
  double score = (double)m_filteredMatches.size() / meanDescriptorDistanceTmp;

  if (meanDescriptorDistance != NULL) {
    *meanDescriptorDistance = meanDescriptorDistanceTmp;
  }
  if (detection_score != NULL) {
    *detection_score = score;
  }

  if (m_filteredMatches.size() >= 4) {
    // Training / Reference 2D points
    std::vector<cv::Point2f> points1(m_filteredMatches.size());
    // Query / Current 2D points
    std::vector<cv::Point2f> points2(m_filteredMatches.size());

    for (size_t i = 0; i < m_filteredMatches.size(); i++) {
      points1[i] = cv::Point2f(m_trainKeyPoints[(size_t)m_filteredMatches[i].trainIdx].pt);
      points2[i] = cv::Point2f(m_queryFilteredKeyPoints[(size_t)m_filteredMatches[i].queryIdx].pt);
    }

    std::vector<vpImagePoint> inliers;
    if (isPlanarObject) {
#if (VISP_HAVE_OPENCV_VERSION < 0x030000)
      cv::Mat homographyMatrix = cv::findHomography(points1, points2, CV_RANSAC);
#else
      cv::Mat homographyMatrix = cv::findHomography(points1, points2, cv::RANSAC);
#endif

      for (size_t i = 0; i < m_filteredMatches.size(); i++) {
        // Compute reprojection error
        cv::Mat realPoint = cv::Mat(3, 1, CV_64F);
        realPoint.at<double>(0, 0) = points1[i].x;
        realPoint.at<double>(1, 0) = points1[i].y;
        realPoint.at<double>(2, 0) = 1.f;

        cv::Mat reprojectedPoint = homographyMatrix * realPoint;
        double err_x = (reprojectedPoint.at<double>(0, 0) / reprojectedPoint.at<double>(2, 0)) - points2[i].x;
        double err_y = (reprojectedPoint.at<double>(1, 0) / reprojectedPoint.at<double>(2, 0)) - points2[i].y;
        double reprojectionError = std::sqrt(err_x * err_x + err_y * err_y);

        if (reprojectionError < 6.0) {
          inliers.push_back(vpImagePoint((double)points2[i].y, (double)points2[i].x));
          if (imPts1 != NULL) {
            imPts1->push_back(vpImagePoint((double)points1[i].y, (double)points1[i].x));
          }

          if (imPts2 != NULL) {
            imPts2->push_back(vpImagePoint((double)points2[i].y, (double)points2[i].x));
          }
        }
      }
    } else if (m_filteredMatches.size() >= 8) {
      cv::Mat fundamentalInliers;
      cv::Mat fundamentalMatrix = cv::findFundamentalMat(points1, points2, cv::FM_RANSAC, 3, 0.99, fundamentalInliers);

      for (size_t i = 0; i < (size_t)fundamentalInliers.rows; i++) {
        if (fundamentalInliers.at<uchar>((int)i, 0)) {
          inliers.push_back(vpImagePoint((double)points2[i].y, (double)points2[i].x));

          if (imPts1 != NULL) {
            imPts1->push_back(vpImagePoint((double)points1[i].y, (double)points1[i].x));
          }

          if (imPts2 != NULL) {
            imPts2->push_back(vpImagePoint((double)points2[i].y, (double)points2[i].x));
          }
        }
      }
    }

    if (!inliers.empty()) {
      // Build a polygon with the list of inlier keypoints detected in the
      // current image to get the bounding box
      vpPolygon polygon(inliers);
      boundingBox = polygon.getBoundingBox();

      // Compute the center of gravity
      double meanU = 0.0, meanV = 0.0;
      for (std::vector<vpImagePoint>::const_iterator it = inliers.begin(); it != inliers.end(); ++it) {
        meanU += it->get_u();
        meanV += it->get_v();
      }

      meanU /= (double)inliers.size();
      meanV /= (double)inliers.size();

      centerOfGravity.set_u(meanU);
      centerOfGravity.set_v(meanV);
    }
  } else {
    // Too few matches
    return false;
  }

  if (m_detectionMethod == detectionThreshold) {
    return meanDescriptorDistanceTmp < m_detectionThreshold;
  } else {
    return score > m_detectionScore;
  }
}

/*!
   Match keypoints detected in the image with those built in the reference
   list, compute the pose and return also the bounding box and the center of
   gravity.

   \param I : Input image
   \param cam : Camera parameters
   \param cMo : Homogeneous matrix between the object frame and the camera frame
   \param error : Reprojection mean square error (in pixel) between the
   2D points and the projection of the 3D points with the estimated pose
   \param elapsedTime : Time to detect, extract, match and compute the pose
   \param boundingBox : Bounding box that contains the good matches
   \param centerOfGravity : Center of gravity computed from the location of
   the good matches (could differ of the center of the bounding box)
   \param func : Function pointer to filter the pose in Ransac pose estimation, if we
   want to eliminate the poses which do not respect some criterion
   \param rectangle : Rectangle corresponding to the ROI (Region of Interest) to consider
   \return True if the matching and the pose estimation are OK, false otherwise.
 */
bool vpKeyPoint::matchPointAndDetect(const vpImage<unsigned char> &I, const vpCameraParameters &cam,
                                     vpHomogeneousMatrix &cMo, double &error, double &elapsedTime, vpRect &boundingBox,
                                     vpImagePoint &centerOfGravity, bool (*func)(const vpHomogeneousMatrix &),
                                     const vpRect &rectangle)
{
  bool isMatchOk = matchPoint(I, cam, cMo, error, elapsedTime, func, rectangle);
  if (isMatchOk) {
    // Use the pose estimated to project the model points in the image
    vpPoint pt;
    vpImagePoint imPt;
    std::vector<vpImagePoint> modelImagePoints(m_trainVpPoints.size());
    size_t cpt = 0;
    for (std::vector<vpPoint>::const_iterator it = m_trainVpPoints.begin(); it != m_trainVpPoints.end(); ++it, cpt++) {
      pt = *it;
      pt.project(cMo);
      vpMeterPixelConversion::convertPoint(cam, pt.get_x(), pt.get_y(), imPt);
      modelImagePoints[cpt] = imPt;
    }

    // Build a polygon with the list of model image points to get the bounding
    // box
    vpPolygon polygon(modelImagePoints);
    boundingBox = polygon.getBoundingBox();

    // Compute the center of gravity of the current inlier keypoints
    double meanU = 0.0, meanV = 0.0;
    for (std::vector<vpImagePoint>::const_iterator it = m_ransacInliers.begin(); it != m_ransacInliers.end(); ++it) {
      meanU += it->get_u();
      meanV += it->get_v();
    }

    meanU /= (double)m_ransacInliers.size();
    meanV /= (double)m_ransacInliers.size();

    centerOfGravity.set_u(meanU);
    centerOfGravity.set_v(meanV);
  }

  return isMatchOk;
}

/*!
    Apply a set of affine transormations to the image, detect keypoints and
    reproject them into initial image coordinates.
    See http://www.ipol.im/pub/algo/my_affine_sift/ for the details.
    See https://github.com/Itseez/opencv/blob/master/samples/python2/asift.py
   for the Python implementation by Itseez and Matt Sheckells for the current
   implementation in C++. \param I : Input image \param listOfKeypoints : List
   of detected keypoints in the multiple images after affine transformations
    \param listOfDescriptors : Corresponding list of descriptors
    \param listOfAffineI : Optional parameter, list of images after affine
   transformations
 */
void vpKeyPoint::detectExtractAffine(const vpImage<unsigned char> &I,
                                     std::vector<std::vector<cv::KeyPoint> > &listOfKeypoints,
                                     std::vector<cv::Mat> &listOfDescriptors,
                                     std::vector<vpImage<unsigned char> > *listOfAffineI)
{
#if 0
  cv::Mat img;
  vpImageConvert::convert(I, img);
  listOfKeypoints.clear();
  listOfDescriptors.clear();

  for (int tl = 1; tl < 6; tl++) {
    double t = pow(2, 0.5 * tl);
    for (int phi = 0; phi < 180; phi += (int)(72.0 / t)) {
      std::vector<cv::KeyPoint> keypoints;
      cv::Mat descriptors;

      cv::Mat timg, mask, Ai;
      img.copyTo(timg);

      affineSkew(t, phi, timg, mask, Ai);


      if(listOfAffineI != NULL) {
        cv::Mat img_disp;
        bitwise_and(mask, timg, img_disp);
        vpImage<unsigned char> tI;
        vpImageConvert::convert(img_disp, tI);
        listOfAffineI->push_back(tI);
      }
#if 0
      cv::Mat img_disp;
      cv::bitwise_and(mask, timg, img_disp);
      cv::namedWindow( "Skew", cv::WINDOW_AUTOSIZE ); // Create a window for display.
      cv::imshow( "Skew", img_disp );
      cv::waitKey(0);
#endif

      for(std::map<std::string, cv::Ptr<cv::FeatureDetector> >::const_iterator it = m_detectors.begin();
          it != m_detectors.end(); ++it) {
        std::vector<cv::KeyPoint> kp;
        it->second->detect(timg, kp, mask);
        keypoints.insert(keypoints.end(), kp.begin(), kp.end());
      }

      double elapsedTime;
      extract(timg, keypoints, descriptors, elapsedTime);

      for(unsigned int i = 0; i < keypoints.size(); i++) {
        cv::Point3f kpt(keypoints[i].pt.x, keypoints[i].pt.y, 1.f);
        cv::Mat kpt_t = Ai * cv::Mat(kpt);
        keypoints[i].pt.x = kpt_t.at<float>(0, 0);
        keypoints[i].pt.y = kpt_t.at<float>(1, 0);
      }

      listOfKeypoints.push_back(keypoints);
      listOfDescriptors.push_back(descriptors);
    }
  }

#else
  cv::Mat img;
  vpImageConvert::convert(I, img);

  // Create a vector for storing the affine skew parameters
  std::vector<std::pair<double, int> > listOfAffineParams;
  for (int tl = 1; tl < 6; tl++) {
    double t = pow(2, 0.5 * tl);
    for (int phi = 0; phi < 180; phi += (int)(72.0 / t)) {
      listOfAffineParams.push_back(std::pair<double, int>(t, phi));
    }
  }

  listOfKeypoints.resize(listOfAffineParams.size());
  listOfDescriptors.resize(listOfAffineParams.size());

  if (listOfAffineI != NULL) {
    listOfAffineI->resize(listOfAffineParams.size());
  }

#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
  for (int cpt = 0; cpt < static_cast<int>(listOfAffineParams.size()); cpt++) {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    cv::Mat timg, mask, Ai;
    img.copyTo(timg);

    affineSkew(listOfAffineParams[(size_t)cpt].first, listOfAffineParams[(size_t)cpt].second, timg, mask, Ai);

    if (listOfAffineI != NULL) {
      cv::Mat img_disp;
      bitwise_and(mask, timg, img_disp);
      vpImage<unsigned char> tI;
      vpImageConvert::convert(img_disp, tI);
      (*listOfAffineI)[(size_t)cpt] = tI;
    }

#if 0
    cv::Mat img_disp;
    cv::bitwise_and(mask, timg, img_disp);
    cv::namedWindow( "Skew", cv::WINDOW_AUTOSIZE ); // Create a window for display.
    cv::imshow( "Skew", img_disp );
    cv::waitKey(0);
#endif

    for (std::map<std::string, cv::Ptr<cv::FeatureDetector> >::const_iterator it = m_detectors.begin();
         it != m_detectors.end(); ++it) {
      std::vector<cv::KeyPoint> kp;
      it->second->detect(timg, kp, mask);
      keypoints.insert(keypoints.end(), kp.begin(), kp.end());
    }

    double elapsedTime;
    extract(timg, keypoints, descriptors, elapsedTime);

    for (size_t i = 0; i < keypoints.size(); i++) {
      cv::Point3f kpt(keypoints[i].pt.x, keypoints[i].pt.y, 1.f);
      cv::Mat kpt_t = Ai * cv::Mat(kpt);
      keypoints[i].pt.x = kpt_t.at<float>(0, 0);
      keypoints[i].pt.y = kpt_t.at<float>(1, 0);
    }

    listOfKeypoints[(size_t)cpt] = keypoints;
    listOfDescriptors[(size_t)cpt] = descriptors;
  }
#endif
}

/*!
   Reset the instance as if we would declare another vpKeyPoint variable.
 */
void vpKeyPoint::reset()
{
  // vpBasicKeyPoint class
  referenceImagePointsList.clear();
  currentImagePointsList.clear();
  matchedReferencePoints.clear();
  _reference_computed = false;

  m_computeCovariance = false;
  m_covarianceMatrix = vpMatrix();
  m_currentImageId = 0;
  m_detectionMethod = detectionScore;
  m_detectionScore = 0.15;
  m_detectionThreshold = 100.0;
  m_detectionTime = 0.0;
  m_detectorNames.clear();
  m_detectors.clear();
  m_extractionTime = 0.0;
  m_extractorNames.clear();
  m_extractors.clear();
  m_filteredMatches.clear();
  m_filterType = ratioDistanceThreshold;
  m_imageFormat = jpgImageFormat;
  m_knnMatches.clear();
  m_mapOfImageId.clear();
  m_mapOfImages.clear();
  m_matcher = cv::Ptr<cv::DescriptorMatcher>();
  m_matcherName = "BruteForce-Hamming";
  m_matches.clear();
  m_matchingFactorThreshold = 2.0;
  m_matchingRatioThreshold = 0.85;
  m_matchingTime = 0.0;
  m_matchRansacKeyPointsToPoints.clear();
  m_nbRansacIterations = 200;
  m_nbRansacMinInlierCount = 100;
  m_objectFilteredPoints.clear();
  m_poseTime = 0.0;
  m_queryDescriptors = cv::Mat();
  m_queryFilteredKeyPoints.clear();
  m_queryKeyPoints.clear();
  m_ransacConsensusPercentage = 20.0;
  m_ransacFilterFlag = vpPose::NO_FILTER;
  m_ransacInliers.clear();
  m_ransacOutliers.clear();
  m_ransacParallel = true;
  m_ransacParallelNbThreads = 0;
  m_ransacReprojectionError = 6.0;
  m_ransacThreshold = 0.01;
  m_trainDescriptors = cv::Mat();
  m_trainKeyPoints.clear();
  m_trainPoints.clear();
  m_trainVpPoints.clear();
  m_useAffineDetection = false;
#if (VISP_HAVE_OPENCV_VERSION >= 0x020400 && VISP_HAVE_OPENCV_VERSION < 0x030000)
  m_useBruteForceCrossCheck = true;
#endif
  m_useConsensusPercentage = false;
  m_useKnn = true; // as m_filterType == ratioDistanceThreshold
  m_useMatchTrainToQuery = false;
  m_useRansacVVS = true;
  m_useSingleMatchFilter = true;

  m_detectorNames.push_back("ORB");
  m_extractorNames.push_back("ORB");

  init();
}

/*!
   Save the learning data in a file in XML or binary mode.

   \param filename : Path of the save file
   \param binaryMode : If true, the data are saved in binary mode, otherwise
   in XML mode \param saveTrainingImages : If true, save also the training
   images on disk
 */
void vpKeyPoint::saveLearningData(const std::string &filename, bool binaryMode, const bool saveTrainingImages)
{
  std::string parent = vpIoTools::getParent(filename);
  if (!parent.empty()) {
    vpIoTools::makeDirectory(parent);
  }

  std::map<int, std::string> mapOfImgPath;
  if (saveTrainingImages) {
#ifdef VISP_HAVE_MODULE_IO
    // Save the training image files in the same directory
    unsigned int cpt = 0;

    for (std::map<int, vpImage<unsigned char> >::const_iterator it = m_mapOfImages.begin(); it != m_mapOfImages.end();
         ++it, cpt++) {
      if (cpt > 999) {
        throw vpException(vpException::fatalError, "The number of training images to save is too big !");
      }

      std::stringstream ss;
      ss << "train_image_" << std::setfill('0') << std::setw(3) << cpt;

      switch (m_imageFormat) {
      case jpgImageFormat:
        ss << ".jpg";
        break;

      case pngImageFormat:
        ss << ".png";
        break;

      case ppmImageFormat:
        ss << ".ppm";
        break;

      case pgmImageFormat:
        ss << ".pgm";
        break;

      default:
        ss << ".png";
        break;
      }

      std::string imgFilename = ss.str();
      mapOfImgPath[it->first] = imgFilename;
      vpImageIo::write(it->second, parent + (!parent.empty() ? "/" : "") + imgFilename);
    }
#else
    std::cout << "Warning: in vpKeyPoint::saveLearningData() training images "
                 "are not saved because "
                 "visp_io module is not available !"
              << std::endl;
#endif
  }

  bool have3DInfo = m_trainPoints.size() > 0;
  if (have3DInfo && m_trainPoints.size() != m_trainKeyPoints.size()) {
    throw vpException(vpException::fatalError, "List of keypoints and list of 3D points have different size !");
  }

  if (binaryMode) {
    // Save the learning data into little endian binary file.
    std::ofstream file(filename.c_str(), std::ofstream::binary);
    if (!file.is_open()) {
      throw vpException(vpException::ioError, "Cannot create the file.");
    }

    // Write info about training images
    int nbImgs = (int)mapOfImgPath.size();
    vpIoTools::writeBinaryValueLE(file, nbImgs);

#ifdef VISP_HAVE_MODULE_IO
    for (std::map<int, std::string>::const_iterator it = mapOfImgPath.begin(); it != mapOfImgPath.end(); ++it) {
      // Write image_id
      int id = it->first;
      vpIoTools::writeBinaryValueLE(file, id);

      // Write image path
      std::string path = it->second;
      int length = (int)path.length();
      vpIoTools::writeBinaryValueLE(file, length);

      for (int cpt = 0; cpt < length; cpt++) {
        file.write((char *)(&path[(size_t)cpt]), sizeof(path[(size_t)cpt]));
      }
    }
#endif

    // Write if we have 3D point information
    int have3DInfoInt = have3DInfo ? 1 : 0;
    vpIoTools::writeBinaryValueLE(file, have3DInfoInt);

    int nRows = m_trainDescriptors.rows, nCols = m_trainDescriptors.cols;
    int descriptorType = m_trainDescriptors.type();

    // Write the number of descriptors
    vpIoTools::writeBinaryValueLE(file, nRows);

    // Write the size of the descriptor
    vpIoTools::writeBinaryValueLE(file, nCols);

    // Write the type of the descriptor
    vpIoTools::writeBinaryValueLE(file, descriptorType);

    for (int i = 0; i < nRows; i++) {
      unsigned int i_ = (unsigned int)i;
      // Write u
      float u = m_trainKeyPoints[i_].pt.x;
      vpIoTools::writeBinaryValueLE(file, u);

      // Write v
      float v = m_trainKeyPoints[i_].pt.y;
      vpIoTools::writeBinaryValueLE(file, v);

      // Write size
      float size = m_trainKeyPoints[i_].size;
      vpIoTools::writeBinaryValueLE(file, size);

      // Write angle
      float angle = m_trainKeyPoints[i_].angle;
      vpIoTools::writeBinaryValueLE(file, angle);

      // Write response
      float response = m_trainKeyPoints[i_].response;
      vpIoTools::writeBinaryValueLE(file, response);

      // Write octave
      int octave = m_trainKeyPoints[i_].octave;
      vpIoTools::writeBinaryValueLE(file, octave);

      // Write class_id
      int class_id = m_trainKeyPoints[i_].class_id;
      vpIoTools::writeBinaryValueLE(file, class_id);

// Write image_id
#ifdef VISP_HAVE_MODULE_IO
      std::map<int, int>::const_iterator it_findImgId = m_mapOfImageId.find(m_trainKeyPoints[i_].class_id);
      int image_id = (saveTrainingImages && it_findImgId != m_mapOfImageId.end()) ? it_findImgId->second : -1;
      vpIoTools::writeBinaryValueLE(file, image_id);
#else
      int image_id = -1;
      //      file.write((char *)(&image_id), sizeof(image_id));
      vpIoTools::writeBinaryValueLE(file, image_id);
#endif

      if (have3DInfo) {
        float oX = m_trainPoints[i_].x, oY = m_trainPoints[i_].y, oZ = m_trainPoints[i_].z;
        // Write oX
        vpIoTools::writeBinaryValueLE(file, oX);

        // Write oY
        vpIoTools::writeBinaryValueLE(file, oY);

        // Write oZ
        vpIoTools::writeBinaryValueLE(file, oZ);
      }

      for (int j = 0; j < nCols; j++) {
        // Write the descriptor value
        switch (descriptorType) {
        case CV_8U:
          file.write((char *)(&m_trainDescriptors.at<unsigned char>(i, j)),
                     sizeof(m_trainDescriptors.at<unsigned char>(i, j)));
          break;

        case CV_8S:
          file.write((char *)(&m_trainDescriptors.at<char>(i, j)), sizeof(m_trainDescriptors.at<char>(i, j)));
          break;

        case CV_16U:
          vpIoTools::writeBinaryValueLE(file, m_trainDescriptors.at<unsigned short int>(i, j));
          break;

        case CV_16S:
          vpIoTools::writeBinaryValueLE(file, m_trainDescriptors.at<short int>(i, j));
          break;

        case CV_32S:
          vpIoTools::writeBinaryValueLE(file, m_trainDescriptors.at<int>(i, j));
          break;

        case CV_32F:
          vpIoTools::writeBinaryValueLE(file, m_trainDescriptors.at<float>(i, j));
          break;

        case CV_64F:
          vpIoTools::writeBinaryValueLE(file, m_trainDescriptors.at<double>(i, j));
          break;

        default:
          throw vpException(vpException::fatalError, "Problem with the data type of descriptors !");
          break;
        }
      }
    }

    file.close();
  } else {
#ifdef VISP_HAVE_XML2
    xmlDocPtr doc = NULL;
    xmlNodePtr root_node = NULL, image_node = NULL, image_info_node = NULL, descriptors_info_node = NULL,
               descriptor_node = NULL, desc_node = NULL;

    /*
     * this initialize the library and check potential ABI mismatches
     * between the version it was compiled for and the actual shared
     * library used.
     */
    LIBXML_TEST_VERSION

    doc = xmlNewDoc(BAD_CAST "1.0");
    if (doc == NULL) {
      throw vpException(vpException::ioError, "Error with file " + filename);
    }

    root_node = xmlNewNode(NULL, BAD_CAST "LearningData");
    xmlDocSetRootElement(doc, root_node);

    std::stringstream ss;

    // Write the training images info
    image_node = xmlNewChild(root_node, NULL, BAD_CAST "TrainingImageInfo", NULL);

#ifdef VISP_HAVE_MODULE_IO
    for (std::map<int, std::string>::const_iterator it = mapOfImgPath.begin(); it != mapOfImgPath.end(); ++it) {
      image_info_node = xmlNewChild(image_node, NULL, BAD_CAST "trainImg", BAD_CAST it->second.c_str());
      ss.str("");
      ss << it->first;
      xmlNewProp(image_info_node, BAD_CAST "image_id", BAD_CAST ss.str().c_str());
    }
#endif

    // Write information about descriptors
    descriptors_info_node = xmlNewChild(root_node, NULL, BAD_CAST "DescriptorsInfo", NULL);

    int nRows = m_trainDescriptors.rows, nCols = m_trainDescriptors.cols;
    int descriptorType = m_trainDescriptors.type();

    // Write the number of rows
    ss.str("");
    ss << nRows;
    xmlNewChild(descriptors_info_node, NULL, BAD_CAST "nrows", BAD_CAST ss.str().c_str());

    // Write the number of cols
    ss.str("");
    ss << nCols;
    xmlNewChild(descriptors_info_node, NULL, BAD_CAST "ncols", BAD_CAST ss.str().c_str());

    // Write the descriptors type
    ss.str("");
    ss << descriptorType;
    xmlNewChild(descriptors_info_node, NULL, BAD_CAST "type", BAD_CAST ss.str().c_str());

    for (int i = 0; i < nRows; i++) {
      unsigned int i_ = (unsigned int)i;
      descriptor_node = xmlNewChild(root_node, NULL, BAD_CAST "DescriptorInfo", NULL);

      ss.str("");
      // max_digits10 == 9 for float
      ss << std::fixed << std::setprecision(9) << m_trainKeyPoints[i_].pt.x;
      xmlNewChild(descriptor_node, NULL, BAD_CAST "u", BAD_CAST ss.str().c_str());

      ss.str("");
      // max_digits10 == 9 for float
      ss << std::fixed << std::setprecision(9) << m_trainKeyPoints[i_].pt.y;
      xmlNewChild(descriptor_node, NULL, BAD_CAST "v", BAD_CAST ss.str().c_str());

      ss.str("");
      // max_digits10 == 9 for float
      ss << std::fixed << std::setprecision(9) << m_trainKeyPoints[i_].size;
      xmlNewChild(descriptor_node, NULL, BAD_CAST "size", BAD_CAST ss.str().c_str());

      ss.str("");
      // max_digits10 == 9 for float
      ss << std::fixed << std::setprecision(9) << m_trainKeyPoints[i_].angle;
      xmlNewChild(descriptor_node, NULL, BAD_CAST "angle", BAD_CAST ss.str().c_str());

      ss.str("");
      // max_digits10 == 9 for float
      ss << std::fixed << std::setprecision(9) << m_trainKeyPoints[i_].response;
      xmlNewChild(descriptor_node, NULL, BAD_CAST "response", BAD_CAST ss.str().c_str());

      ss.str("");
      ss << m_trainKeyPoints[i_].octave;
      xmlNewChild(descriptor_node, NULL, BAD_CAST "octave", BAD_CAST ss.str().c_str());

      ss.str("");
      ss << m_trainKeyPoints[i_].class_id;
      xmlNewChild(descriptor_node, NULL, BAD_CAST "class_id", BAD_CAST ss.str().c_str());

      ss.str("");
#ifdef VISP_HAVE_MODULE_IO
      std::map<int, int>::const_iterator it_findImgId = m_mapOfImageId.find(m_trainKeyPoints[i_].class_id);
      ss << ((saveTrainingImages && it_findImgId != m_mapOfImageId.end()) ? it_findImgId->second : -1);
      xmlNewChild(descriptor_node, NULL, BAD_CAST "image_id", BAD_CAST ss.str().c_str());
#else
      ss << -1;
      xmlNewChild(descriptor_node, NULL, BAD_CAST "image_id", BAD_CAST ss.str().c_str());
#endif

      if (have3DInfo) {
        ss.str("");
        // max_digits10 == 9 for float
        ss << std::fixed << std::setprecision(9) << m_trainPoints[i_].x;
        xmlNewChild(descriptor_node, NULL, BAD_CAST "oX", BAD_CAST ss.str().c_str());

        ss.str("");
        // max_digits10 == 9 for float
        ss << std::fixed << std::setprecision(9) << m_trainPoints[i_].y;
        xmlNewChild(descriptor_node, NULL, BAD_CAST "oY", BAD_CAST ss.str().c_str());

        ss.str("");
        // max_digits10 == 9 for float
        ss << std::fixed << std::setprecision(9) << m_trainPoints[i_].z;
        xmlNewChild(descriptor_node, NULL, BAD_CAST "oZ", BAD_CAST ss.str().c_str());
      }

      desc_node = xmlNewChild(descriptor_node, NULL, BAD_CAST "desc", NULL);

      for (int j = 0; j < nCols; j++) {
        ss.str("");

        switch (descriptorType) {
        case CV_8U: {
          // Promote an unsigned char to an int
          // val_tmp holds the numeric value that will be written
          // We save the value in numeric form otherwise libxml2 will not be
          // able to parse  A better solution could be possible
          int val_tmp = m_trainDescriptors.at<unsigned char>(i, j);
          ss << val_tmp;
        } break;

        case CV_8S: {
          // Promote a char to an int
          // val_tmp holds the numeric value that will be written
          // We save the value in numeric form otherwise libxml2 will not be
          // able to parse  A better solution could be possible
          int val_tmp = m_trainDescriptors.at<char>(i, j);
          ss << val_tmp;
        } break;

        case CV_16U:
          ss << m_trainDescriptors.at<unsigned short int>(i, j);
          break;

        case CV_16S:
          ss << m_trainDescriptors.at<short int>(i, j);
          break;

        case CV_32S:
          ss << m_trainDescriptors.at<int>(i, j);
          break;

        case CV_32F:
          // max_digits10 == 9 for float
          ss << std::fixed << std::setprecision(9) << m_trainDescriptors.at<float>(i, j);
          break;

        case CV_64F:
          // max_digits10 == 17 for double
          ss << std::fixed << std::setprecision(17) << m_trainDescriptors.at<double>(i, j);
          break;

        default:
          throw vpException(vpException::fatalError, "Problem with the data type of descriptors !");
          break;
        }
        xmlNewChild(desc_node, NULL, BAD_CAST "val", BAD_CAST ss.str().c_str());
      }
    }

    xmlSaveFormatFileEnc(filename.c_str(), doc, "UTF-8", 1);

    /*free the document */
    xmlFreeDoc(doc);

    /*
     *Free the global variables that may
     *have been allocated by the parser.
     */
    xmlCleanupParser();
#else
    std::cerr << "Error: libxml2 is required !" << std::endl;
#endif
  }
}

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x030000)
// From OpenCV 2.4.11 source code.
struct KeypointResponseGreaterThanThreshold {
  KeypointResponseGreaterThanThreshold(float _value) : value(_value) {}
  inline bool operator()(const cv::KeyPoint &kpt) const { return kpt.response >= value; }
  float value;
};

struct KeypointResponseGreater {
  inline bool operator()(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2) const { return kp1.response > kp2.response; }
};

// takes keypoints and culls them by the response
void vpKeyPoint::KeyPointsFilter::retainBest(std::vector<cv::KeyPoint> &keypoints, int n_points)
{
  // this is only necessary if the keypoints size is greater than the number
  // of desired points.
  if (n_points >= 0 && keypoints.size() > (size_t)n_points) {
    if (n_points == 0) {
      keypoints.clear();
      return;
    }
    // first use nth element to partition the keypoints into the best and
    // worst.
    std::nth_element(keypoints.begin(), keypoints.begin() + n_points, keypoints.end(), KeypointResponseGreater());
    // this is the boundary response, and in the case of FAST may be ambiguous
    float ambiguous_response = keypoints[(size_t)(n_points - 1)].response;
    // use std::partition to grab all of the keypoints with the boundary
    // response.
    std::vector<cv::KeyPoint>::const_iterator new_end = std::partition(
        keypoints.begin() + n_points, keypoints.end(), KeypointResponseGreaterThanThreshold(ambiguous_response));
    // resize the keypoints, given this new end point. nth_element and
    // partition reordered the points inplace
    keypoints.resize((size_t)(new_end - keypoints.begin()));
  }
}

struct RoiPredicate {
  RoiPredicate(const cv::Rect &_r) : r(_r) {}

  bool operator()(const cv::KeyPoint &keyPt) const { return !r.contains(keyPt.pt); }

  cv::Rect r;
};

void vpKeyPoint::KeyPointsFilter::runByImageBorder(std::vector<cv::KeyPoint> &keypoints, cv::Size imageSize,
                                                   int borderSize)
{
  if (borderSize > 0) {
    if (imageSize.height <= borderSize * 2 || imageSize.width <= borderSize * 2)
      keypoints.clear();
    else
      keypoints.erase(std::remove_if(keypoints.begin(), keypoints.end(),
                                     RoiPredicate(cv::Rect(
                                         cv::Point(borderSize, borderSize),
                                         cv::Point(imageSize.width - borderSize, imageSize.height - borderSize)))),
                      keypoints.end());
  }
}

struct SizePredicate {
  SizePredicate(float _minSize, float _maxSize) : minSize(_minSize), maxSize(_maxSize) {}

  bool operator()(const cv::KeyPoint &keyPt) const
  {
    float size = keyPt.size;
    return (size < minSize) || (size > maxSize);
  }

  float minSize, maxSize;
};

void vpKeyPoint::KeyPointsFilter::runByKeypointSize(std::vector<cv::KeyPoint> &keypoints, float minSize, float maxSize)
{
  CV_Assert(minSize >= 0);
  CV_Assert(maxSize >= 0);
  CV_Assert(minSize <= maxSize);

  keypoints.erase(std::remove_if(keypoints.begin(), keypoints.end(), SizePredicate(minSize, maxSize)), keypoints.end());
}

class MaskPredicate
{
public:
  MaskPredicate(const cv::Mat &_mask) : mask(_mask) {}
  bool operator()(const cv::KeyPoint &key_pt) const
  {
    return mask.at<uchar>((int)(key_pt.pt.y + 0.5f), (int)(key_pt.pt.x + 0.5f)) == 0;
  }

private:
  const cv::Mat mask;
  MaskPredicate &operator=(const MaskPredicate &);
};

void vpKeyPoint::KeyPointsFilter::runByPixelsMask(std::vector<cv::KeyPoint> &keypoints, const cv::Mat &mask)
{
  if (mask.empty())
    return;

  keypoints.erase(std::remove_if(keypoints.begin(), keypoints.end(), MaskPredicate(mask)), keypoints.end());
}

struct KeyPoint_LessThan {
  KeyPoint_LessThan(const std::vector<cv::KeyPoint> &_kp) : kp(&_kp) {}
  bool operator()(/*int i, int j*/ size_t i, size_t j) const
  {
    const cv::KeyPoint &kp1 = (*kp)[/*(size_t)*/ i];
    const cv::KeyPoint &kp2 = (*kp)[/*(size_t)*/ j];
    if (!vpMath::equal(kp1.pt.x, kp2.pt.x,
                       std::numeric_limits<float>::epsilon())) { // if (kp1.pt.x !=
                                                                 // kp2.pt.x) {
      return kp1.pt.x < kp2.pt.x;
    }

    if (!vpMath::equal(kp1.pt.y, kp2.pt.y,
                       std::numeric_limits<float>::epsilon())) { // if (kp1.pt.y !=
                                                                 // kp2.pt.y) {
      return kp1.pt.y < kp2.pt.y;
    }

    if (!vpMath::equal(kp1.size, kp2.size,
                       std::numeric_limits<float>::epsilon())) { // if (kp1.size !=
                                                                 // kp2.size) {
      return kp1.size > kp2.size;
    }

    if (!vpMath::equal(kp1.angle, kp2.angle,
                       std::numeric_limits<float>::epsilon())) { // if (kp1.angle !=
                                                                 // kp2.angle) {
      return kp1.angle < kp2.angle;
    }

    if (!vpMath::equal(kp1.response, kp2.response,
                       std::numeric_limits<float>::epsilon())) { // if (kp1.response !=
                                                                 // kp2.response) {
      return kp1.response > kp2.response;
    }

    if (kp1.octave != kp2.octave) {
      return kp1.octave > kp2.octave;
    }

    if (kp1.class_id != kp2.class_id) {
      return kp1.class_id > kp2.class_id;
    }

    return i < j;
  }
  const std::vector<cv::KeyPoint> *kp;
};

void vpKeyPoint::KeyPointsFilter::removeDuplicated(std::vector<cv::KeyPoint> &keypoints)
{
  size_t i, j, n = keypoints.size();
  std::vector<size_t> kpidx(n);
  std::vector<uchar> mask(n, (uchar)1);

  for (i = 0; i < n; i++) {
    kpidx[i] = i;
  }
  std::sort(kpidx.begin(), kpidx.end(), KeyPoint_LessThan(keypoints));
  for (i = 1, j = 0; i < n; i++) {
    cv::KeyPoint &kp1 = keypoints[kpidx[i]];
    cv::KeyPoint &kp2 = keypoints[kpidx[j]];
    //    if (kp1.pt.x != kp2.pt.x || kp1.pt.y != kp2.pt.y || kp1.size !=
    //    kp2.size || kp1.angle != kp2.angle) {
    if (!vpMath::equal(kp1.pt.x, kp2.pt.x, std::numeric_limits<float>::epsilon()) ||
        !vpMath::equal(kp1.pt.y, kp2.pt.y, std::numeric_limits<float>::epsilon()) ||
        !vpMath::equal(kp1.size, kp2.size, std::numeric_limits<float>::epsilon()) ||
        !vpMath::equal(kp1.angle, kp2.angle, std::numeric_limits<float>::epsilon())) {
      j = i;
    } else {
      mask[kpidx[i]] = 0;
    }
  }

  for (i = j = 0; i < n; i++) {
    if (mask[i]) {
      if (i != j) {
        keypoints[j] = keypoints[i];
      }
      j++;
    }
  }
  keypoints.resize(j);
}

/*
 *  PyramidAdaptedFeatureDetector
 */
vpKeyPoint::PyramidAdaptedFeatureDetector::PyramidAdaptedFeatureDetector(const cv::Ptr<cv::FeatureDetector> &_detector,
                                                                         int _maxLevel)
  : detector(_detector), maxLevel(_maxLevel)
{
}

bool vpKeyPoint::PyramidAdaptedFeatureDetector::empty() const
{
  return detector.empty() || (cv::FeatureDetector *)detector->empty();
}

void vpKeyPoint::PyramidAdaptedFeatureDetector::detect(cv::InputArray image,
                                                       CV_OUT std::vector<cv::KeyPoint> &keypoints, cv::InputArray mask)
{
  detectImpl(image.getMat(), keypoints, mask.getMat());
}

void vpKeyPoint::PyramidAdaptedFeatureDetector::detectImpl(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints,
                                                           const cv::Mat &mask) const
{
  cv::Mat src = image;
  cv::Mat src_mask = mask;

  cv::Mat dilated_mask;
  if (!mask.empty()) {
    cv::dilate(mask, dilated_mask, cv::Mat());
    cv::Mat mask255(mask.size(), CV_8UC1, cv::Scalar(0));
    mask255.setTo(cv::Scalar(255), dilated_mask != 0);
    dilated_mask = mask255;
  }

  for (int l = 0, multiplier = 1; l <= maxLevel; ++l, multiplier *= 2) {
    // Detect on current level of the pyramid
    std::vector<cv::KeyPoint> new_pts;
    detector->detect(src, new_pts, src_mask);
    std::vector<cv::KeyPoint>::iterator it = new_pts.begin(), end = new_pts.end();
    for (; it != end; ++it) {
      it->pt.x *= multiplier;
      it->pt.y *= multiplier;
      it->size *= multiplier;
      it->octave = l;
    }
    keypoints.insert(keypoints.end(), new_pts.begin(), new_pts.end());

    // Downsample
    if (l < maxLevel) {
      cv::Mat dst;
      pyrDown(src, dst);
      src = dst;

      if (!mask.empty())
        resize(dilated_mask, src_mask, src.size(), 0, 0, CV_INTER_AREA);
    }
  }

  if (!mask.empty())
    vpKeyPoint::KeyPointsFilter::runByPixelsMask(keypoints, mask);
}
#endif

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_vision.a(vpKeyPoint.cpp.o) has no
// symbols
void dummy_vpKeyPoint(){};
#endif
