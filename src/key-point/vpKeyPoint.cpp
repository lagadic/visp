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

#include <visp/vpKeyPoint.h>

#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)

#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
#  include <opencv2/calib3d/calib3d.hpp>
#endif

//Specific Type transformation functions
// TODO: Create a vpBridge for all the conversions in src/tools/type-convertors
/*!
   Convert a cv::KeyPoint to a vpImagePoint.

   \param keypoint : Keypoint to convert.
   \return The result vpImagePoint.
 */
inline vpImagePoint keyPointToVpImagePoint(const cv::KeyPoint &keypoint) {
  return vpImagePoint(keypoint.pt.y, keypoint.pt.x);
}

///*!
//   Convert a cv::Point2f to a vpImagePoint.
//
//   \param point : Point2f to convert in ViSP type.
//   \return The result vpImagePoint.
// */
inline vpImagePoint point2fToVpImagePoint(const cv::Point2f &point) {
  return vpImagePoint(point.y, point.x);
}

///*!
//   Convert a list of cv::DMatch to a cv::DMatch (extract the first cv::DMatch, the nearest neighbor).
//
//   \param knnMatches : List of cv::DMatch.
//   \return The nearest neighbor.
// */
inline cv::DMatch knnToDMatch(const std::vector<cv::DMatch> &knnMatches) {
  if(knnMatches.size() > 0) {
    return knnMatches[0];
  }

  return cv::DMatch();
}

///*!
//   Convert a cv::Point3f to a vpPoint.
//
//   \param point3f : Point to convert in ViSP type.
//   \return The result vpPoint.
// */
inline vpPoint point3fToVpPoint(const cv::Point3f &point3f) {
  vpPoint pt;
  pt.set_oX(point3f.x);
  pt.set_oY(point3f.y);
  pt.set_oZ(point3f.z);
  return pt;
}

///*!
//   Convert a cv::DMatch to an index (extract the train index).
//
//   \param match : Point to convert in ViSP type.
//   \return The train index.
// */
inline unsigned int dMatchToTrainIndex(const cv::DMatch &match) {
  return match.trainIdx;
}

///*!
//   Convert a cv::DMatch to an index (extract the train index).
//
//   \param match : Point to convert in ViSP type.
//   \return The train index.
// */
inline vpImagePoint matchRansacToVpImage(const std::pair<cv::KeyPoint, cv::Point3f> &pair) {
  return vpImagePoint(pair.first.pt.y, pair.first.pt.x);
}

inline cv::Point2f vpImagePointToPoint2f(const vpImagePoint &point) {
  return cv::Point2f((float) point.get_u(), (float) point.get_v());
}

inline cv::Point3f vpCamPointToPoint3f(const vpPoint &point) {
  return cv::Point3f((float) point.get_X(), (float) point.get_Y(), (float) point.get_Z());
}

inline cv::Point3f vpObjectPointToPoint3f(const vpPoint &point) {
  return cv::Point3f((float) point.get_oX(), (float) point.get_oY(), (float) point.get_oZ());
}

/*!
  Constructor to initialize specified detector, extractor, matcher and filtering method.

  \param detectorName : Name of the detector.
  \param extractorName : Name of the extractor.
  \param matcherName : Name of the matcher.
  \param filterType : Filtering matching method chosen.
 */
vpKeyPoint::vpKeyPoint(const std::string &detectorName, const std::string &extractorName,
                       const std::string &matcherName, const vpFilterMatchingType &filterType) {
  m_filterType = filterType;

  //Use k-nearest neighbors (knn) to retrieve the two best matches for a keypoint
  //So this is useful only for ratioDistanceThreshold method
  if(filterType == ratioDistanceThreshold || filterType == stdAndRatioDistanceThreshold) {
    m_useKnn = true;
  } else {
    m_useKnn = false;
  }

  m_detectorNames.push_back(detectorName);
  m_extractorNames.push_back(extractorName);
  m_matcherName = matcherName;

  init();
}

/*!
  Constructor to initialize specified detector, extractor, matcher and filtering method.

  \param detectorNames : List of name detector for allowing multiple detectors.
  \param extractorNames : List of name extractor for allowing multiple extractors.
  \param matcherName : Name of the matcher.
  \param filterType : Filtering matching method chosen.
 */
vpKeyPoint::vpKeyPoint(const std::vector<std::string> &detectorNames, const std::vector<std::string> &extractorNames,
                       const std::string &matcherName, const vpFilterMatchingType &filterType) {
  m_filterType = filterType;

  //Use k-nearest neighbors (knn) to retrieve the two best matches for a keypoint
  //So this is useful only for ratioDistanceThreshold method
  if(filterType == ratioDistanceThreshold || filterType == stdAndRatioDistanceThreshold) {
    m_useKnn = true;
  } else {
    m_useKnn = false;
  }

  m_detectorNames = detectorNames;
  m_extractorNames = extractorNames;
  m_matcherName = matcherName;

  init();
}

/*!
   Build the reference keypoints list.

   \param I : Input reference image.
   \return The number of detected keypoints in the image \p I.
 */
unsigned int vpKeyPoint::buildReference(const vpImage<unsigned char> &I) {
  //Reset variables used when dealing with 3D models
  //So as no 3D point list is passed, we dont need this variables
  m_trainPoints.clear();
  m_mapOfImageId.clear();
  m_mapOfImages.clear();
  m_currentImageId = 1;

  detect(I, m_trainKeyPoints, m_detectionTime);
  extract(I, m_trainKeyPoints, m_trainDescriptors, m_extractionTime);

  //Save the correspondence keypoint class_id with the training image_id in a map
  //Used to display the matching with all the training images
  for(std::vector<cv::KeyPoint>::const_iterator it = m_trainKeyPoints.begin(); it != m_trainKeyPoints.end(); ++it) {
    m_mapOfImageId[it->class_id] = m_currentImageId;
  }

  //Save the image in a map at a specific image_id
  m_mapOfImages[m_currentImageId] = I;

  //Convert OpenCV type to ViSP type for compatibility
  convertToVpType(m_trainKeyPoints, referenceImagePointsList);

  _reference_computed = true;

  return static_cast<unsigned int>(m_trainKeyPoints.size());
}

/*!
   Build the reference keypoints list in a region of interest in the image.

   \param I : Input reference image
   \param iP : Position of the top-left corner of the region of interest.
   \param height : Height of the region of interest.
   \param width : Width of the region of interest.
   \return The number of detected keypoints in the current image I.
 */
unsigned int vpKeyPoint::buildReference(const vpImage<unsigned char> &I,
                                        const vpImagePoint &iP,
                                        const unsigned int height, const unsigned int width) {

  return buildReference(I, vpRect(iP, width, height));
}

/*!
   Build the reference keypoints list in a region of interest in the image.

   \param I : Input image.
   \param rectangle : Rectangle of the region of interest.
   \return The number of detected keypoints in the current image I.
 */
unsigned int vpKeyPoint::buildReference(const vpImage<unsigned char> &I,
                                        const vpRect &rectangle) {
  //Reset variables used when dealing with 3D models
  //So as no 3D point list is passed, we dont need this variables
  m_trainPoints.clear();
  m_mapOfImageId.clear();
  m_mapOfImages.clear();
  m_currentImageId = 1;

  detect(I, m_trainKeyPoints, m_detectionTime, rectangle);
  extract(I, m_trainKeyPoints, m_trainDescriptors, m_extractionTime);

  //Save the correspondence keypoint class_id with the training image_id in a map
  //Used to display the matching with all the training images
  for(std::vector<cv::KeyPoint>::const_iterator it = m_trainKeyPoints.begin(); it != m_trainKeyPoints.end(); ++it) {
    m_mapOfImageId[it->class_id] = m_currentImageId;
  }

  //Save the image in a map at a specific image_id
  m_mapOfImages[m_currentImageId] = I;

  //Convert OpenCV type to ViSP type for compatibility
  convertToVpType(m_trainKeyPoints, referenceImagePointsList);

  _reference_computed = true;

  return static_cast<unsigned int>(m_trainKeyPoints.size());
}

/*!
   Build the reference keypoints list and compute the 3D position corresponding of the keypoints locations.

   \param I : Input image
   \param trainKeyPoints : List of the train keypoints.
   \param points3f : Output list of the 3D position corresponding of the keypoints locations.
   \param append : If true, append the supply train keypoints with those already present.
 */
void vpKeyPoint::buildReference(const vpImage<unsigned char> &I, std::vector<cv::KeyPoint> &trainKeyPoints,
                                std::vector<cv::Point3f> &points3f, bool append) {
  cv::Mat trainDescriptors;
  extract(I, trainKeyPoints, trainDescriptors, m_extractionTime);

  if(!append) {
    m_currentImageId = 0;
    m_mapOfImageId.clear();
    m_mapOfImages.clear();
  }

  m_currentImageId++;

  //Save the correspondence keypoint class_id with the training image_id in a map
  //Used to display the matching with all the training images
  for(std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
    m_mapOfImageId[it->class_id] = m_currentImageId;
  }

  //Save the image in a map at a specific image_id
  m_mapOfImages[m_currentImageId] = I;

  //Append reference lists
  this->m_trainKeyPoints.insert(this->m_trainKeyPoints.end(), trainKeyPoints.begin(), trainKeyPoints.end());

  if(this->m_trainDescriptors.empty()) {
    trainDescriptors.copyTo(this->m_trainDescriptors);
  } else {
    cv::vconcat(this->m_trainDescriptors, trainDescriptors, this->m_trainDescriptors);
  }
  this->m_trainPoints.insert(this->m_trainPoints.end(), points3f.begin(), points3f.end());


  //Convert OpenCV type to ViSP type for compatibility
  convertToVpType(this->m_trainKeyPoints, referenceImagePointsList);
  convertToVpType(this->m_trainPoints, m_trainVpPoints);

  _reference_computed = true;
}

/*!
   Compute the pose estimation error, the mean square error (in pixel) between the location of the detected keypoints
   and the location of the projection of the 3D model with the estimated pose.

   \param matchKeyPoints : List of pairs between the detected keypoints and the corresponding 3D points.
   \param cam : Camera parameters.
   \param cMo_est : Estimated pose of the object.

   \return The mean square error (in pixel) between the location of the detected keypoints
   and the location of the projection of the 3D model with the estimated pose.
 */
double vpKeyPoint::computePoseEstimationError(const std::vector<std::pair<cv::KeyPoint, cv::Point3f> > &matchKeyPoints,
                                              const vpCameraParameters &cam, const vpHomogeneousMatrix &cMo_est) {
  if(matchKeyPoints.size() == 0) {
    //return std::numeric_limits<double>::max(); // create an error under Windows. To fix it we have to add #undef max
    return DBL_MAX;
  }

  std::vector<double> errors(matchKeyPoints.size());
  size_t cpt = 0;
  vpPoint pt;
  for(std::vector<std::pair<cv::KeyPoint, cv::Point3f> >::const_iterator it = matchKeyPoints.begin();
      it != matchKeyPoints.end(); ++it, cpt++) {
    pt.set_oX(it->second.x);
    pt.set_oY(it->second.y);
    pt.set_oZ(it->second.z);
    pt.project(cMo_est);
    double u = 0.0, v = 0.0;
    vpMeterPixelConversion::convertPoint(cam, pt.get_x(), pt.get_y(), u, v);
    errors[cpt] = std::sqrt((u-it->first.pt.x)*(u-it->first.pt.x) + (v-it->first.pt.y)*(v-it->first.pt.y));
  }

  return std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
}

/*!
  Convert a list of vpImagePoint to a list of cv::Point2f.

  \param from : List of vpImagePoint to convert.
  \param to : List of cv::Point2f converted.
 */
void vpKeyPoint::convertToOpenCVType(const std::vector<vpImagePoint> &from, std::vector<cv::Point2f> &to) {
  to.resize(from.size());
  std::transform(from.begin(), from.end(), to.begin(), vpImagePointToPoint2f);
}

/*!
  Convert a list of vpPoint to a list of cv::Point3. As a vpPoint contains coordinates in the camera frame
  and also in the object frame, which of this two frames to convert must be specified.

  \param from : List of vpPoint to convert.
  \param to : List of cv::Point3f converted.
  \param cameraFrame : If true, the coordinates in the camera frame must be converted, otherwise it will be
  those in the object frame.
 */
void vpKeyPoint::convertToOpenCVType(const std::vector<vpPoint> &from, std::vector<cv::Point3f> &to, const bool cameraFrame) {
  to.resize(from.size());
  if(cameraFrame) {
    std::transform(from.begin(), from.end(), to.begin(), vpCamPointToPoint3f);
  } else {
    std::transform(from.begin(), from.end(), to.begin(), vpObjectPointToPoint3f);
  }
}

/*!
  Convert a list of cv::KeyPoint to a list of vpImagePoint. Only the 2D coordinate is converted.

  \param from : List of cv::KeyPoint to convert.
  \param to : List of vpImagePoint converted.
 */
void vpKeyPoint::convertToVpType(const std::vector<cv::KeyPoint> &from, std::vector<vpImagePoint> &to) {
  to.resize(from.size());
  std::transform(from.begin(), from.end(), to.begin(), keyPointToVpImagePoint);
}

/*!
  Convert a list of cv::Point2f to a list of vpImagePoint.

  \param from : List of cv::Point2f to convert.
  \param to : List of vpImagePoint converted.
 */
void vpKeyPoint::convertToVpType(const std::vector<cv::Point2f> &from, std::vector<vpImagePoint> &to) {
  to.resize(from.size());
  std::transform(from.begin(), from.end(), to.begin(), point2fToVpImagePoint);
}

/*!
  Convert a list of cv::Point3f to a list of vpPoint.

  \param from : List of cv::Point3f to convert.
  \param to : List of vpPoint converted.
 */
void vpKeyPoint::convertToVpType(const std::vector<cv::Point3f> &from, std::vector<vpPoint> &to) {
  to.resize(from.size());
  std::transform(from.begin(), from.end(), to.begin(), point3fToVpPoint);
}

/*!
  Convert a list of cv::DMatch to a list of indexes. At each entry of the result list of indexes contains the
  index of the corresponding point in the reference list (i.e index_train = to[0] is the corresponding index
  of the keypoint in currentImagePointsList[0]).

  \warning The list of query indexes in DMatch must be ordered in a way that from[i].queryIdx == i.

  \param from : List of cv::DMatch to convert.
  \param to : List of indexes converted.
 */
void vpKeyPoint::convertToVpType(const std::vector<cv::DMatch> &from, std::vector<unsigned int> &to) {
  to.resize(from.size());
  std::transform(from.begin(), from.end(), to.begin(), dMatchToTrainIndex);
}

/*!
   Initialize the size of the matching image (case with a matching side by side between IRef and ICurrent).

   \param IRef : Reference image.
   \param ICurrent : Current image.
   \param IMatching : Image matching.
 */
void vpKeyPoint::createImageMatching(vpImage<unsigned char> &IRef, vpImage<unsigned char> &ICurrent,
                                     vpImage<unsigned char> &IMatching) {
  //Image matching side by side
  unsigned int width = IRef.getWidth() + ICurrent.getWidth();
  unsigned int height = (std::max)(IRef.getHeight(), ICurrent.getHeight());

  IMatching = vpImage<unsigned char>(height, width);
}

/*!
   Initialize the size of the matching image with appropriate size according to the number of training images.
   Used to display the matching of keypoints detected in the current image with those detected in multiple
   training images.

   \param ICurrent : Current image.
   \param IMatching : Image initialized with appropriate size.
 */
void vpKeyPoint::createImageMatching(vpImage<unsigned char> &ICurrent, vpImage<unsigned char> &IMatching) {
  //Nb images in the training database + the current image we want to detect the object
  int nbImg = (int) (m_mapOfImages.size() + 1);

  if(m_mapOfImages.empty()) {
    return;
  }

  if(nbImg == 2) {
    //Only one training image, so we display them side by side
    createImageMatching(m_mapOfImages.begin()->second, ICurrent, IMatching);
  } else {
    //Multiple training images, display them as a mosaic image
    //round(std::sqrt((double) nbImg)); VC++ compiler does not have round so the next line is used
    //Different implementations of round exist, here round to the closest integer but will not work for negative numbers
    int nbImgSqrt = (int) std::floor(std::sqrt((double) nbImg) + 0.5);

    //Number of columns in the mosaic grid
    int nbWidth = nbImgSqrt;
    //Number of rows in the mosaic grid
    int nbHeight = nbImgSqrt;

    //Deals with non square mosaic grid and the total number of images
    if(nbImgSqrt * nbImgSqrt < nbImg) {
      nbWidth++;
    }

    unsigned int maxW = ICurrent.getWidth();
    unsigned int maxH = ICurrent.getHeight();
    for(std::map<int, vpImage<unsigned char> >::const_iterator it = m_mapOfImages.begin(); it != m_mapOfImages.end(); ++it) {
      if(maxW < it->second.getWidth()) {
        maxW = it->second.getWidth();
      }

      if(maxH < it->second.getHeight()) {
        maxH = it->second.getHeight();
      }
    }

    IMatching = vpImage<unsigned char>(maxH * nbHeight, maxW * nbWidth);
  }
}

/*!
   Detect keypoints in the image.

   \param I : Input image.
   \param keyPoints : Output list of the detected keypoints.
   \param elapsedTime : Elapsed time.
   \param rectangle : Optional rectangle of the region of interest.
 */
void vpKeyPoint::detect(const vpImage<unsigned char> &I, std::vector<cv::KeyPoint> &keyPoints, double &elapsedTime,
                        const vpRect &rectangle){
  double t = vpTime::measureTimeMs();
  keyPoints.clear();
  cv::Mat matImg;
  vpImageConvert::convert(I, matImg, false);
  if(rectangle.getWidth() > 0 && rectangle.getHeight() > 0) {
    matImg = cv::Mat(matImg, cv::Rect((int) rectangle.getLeft(), (int) rectangle.getTop(),
                                      (int) rectangle.getWidth(), (int) rectangle.getHeight()));
  }

  for(std::map<std::string, cv::Ptr<cv::FeatureDetector> >::const_iterator it = m_detectors.begin(); it != m_detectors.end(); ++it) {
    std::vector<cv::KeyPoint> kp;
    it->second->detect(matImg, kp);
    keyPoints.insert(keyPoints.end(), kp.begin(), kp.end());
  }

  //If a ROI is supplied, correct the 2D coordinates
  if(rectangle.getWidth() > 0 && rectangle.getHeight() > 0) {
    for(std::vector<cv::KeyPoint>::iterator it = keyPoints.begin(); it != keyPoints.end(); ++it) {
      it->pt.x += (float) rectangle.getLeft();
      it->pt.y += (float) rectangle.getTop();
    }
  }

  elapsedTime = vpTime::measureTimeMs() - t;
}

/*!
   Display the reference and the detected keypoints in the images.

   \param IRef : Input reference image.
   \param ICurrent : Input current image.
   \param size : Size of the displayed cross.
 */
void vpKeyPoint::display(const vpImage<unsigned char> &IRef,
                         const vpImage<unsigned char> &ICurrent, unsigned int size) {
  std::vector<vpImagePoint> vpQueryImageKeyPoints;
  getQueryKeyPoints(vpQueryImageKeyPoints);
  std::vector<vpImagePoint> vpTrainImageKeyPoints;
  getTrainKeyPoints(vpTrainImageKeyPoints);

  for(std::vector<cv::DMatch>::const_iterator it = m_filteredMatches.begin(); it != m_filteredMatches.end(); ++it) {
    vpDisplay::displayCross(IRef, vpTrainImageKeyPoints[it->trainIdx], size, vpColor::red);
    vpDisplay::displayCross(ICurrent, vpQueryImageKeyPoints[it->queryIdx], size, vpColor::green);
  }
}

/*!
   Display the reference keypoints.

   \param ICurrent : Input current image.
   \param size : Size of the displayed crosses.
   \param color : Color of the crosses.
 */
void vpKeyPoint::display(const vpImage<unsigned char> &ICurrent, unsigned int size, const vpColor &color) {
  std::vector<vpImagePoint> vpQueryImageKeyPoints;
  getQueryKeyPoints(vpQueryImageKeyPoints);

  for(std::vector<cv::DMatch>::const_iterator it = m_filteredMatches.begin(); it != m_filteredMatches.end(); ++it) {
    vpDisplay::displayCross (ICurrent, vpQueryImageKeyPoints[it->queryIdx], size, color);
  }
}

/*!
  Display the matching lines between the detected keypoints with those detected in one training image.

  \param IRef : Reference image, used to have the x-offset.
  \param IMatching : Resulting image matching.
  \param crossSize : Size of the displayed crosses.
  \param lineThickness : Thickness of the displayed lines.
  \param color : Color to use, if none, we pick randomly a color for each pair of matching.
 */
void vpKeyPoint::displayMatching(const vpImage<unsigned char> &IRef, vpImage<unsigned char> &IMatching,
                                 unsigned int crossSize, unsigned int lineThickness, const vpColor &color) {
  bool randomColor = (color == vpColor::none);
  srand((unsigned int) time(NULL));
  vpColor currentColor = color;

  std::vector<vpImagePoint> queryImageKeyPoints;
  getQueryKeyPoints(queryImageKeyPoints);
  std::vector<vpImagePoint> trainImageKeyPoints;
  getTrainKeyPoints(trainImageKeyPoints);

  vpImagePoint leftPt, rightPt;
  for(std::vector<cv::DMatch>::const_iterator it = m_filteredMatches.begin(); it != m_filteredMatches.end(); ++it) {
    if(randomColor) {
      currentColor = vpColor((rand() % 256), (rand() % 256), (rand() % 256));
    }

    leftPt = trainImageKeyPoints[it->trainIdx];
    rightPt = vpImagePoint(queryImageKeyPoints[it->queryIdx].get_i(),
        queryImageKeyPoints[it->queryIdx].get_j() + IRef.getWidth());
    vpDisplay::displayCross(IMatching, leftPt, crossSize, currentColor);
    vpDisplay::displayCross(IMatching, rightPt, crossSize, currentColor);
    vpDisplay::displayLine(IMatching, leftPt, rightPt, currentColor, lineThickness);
  }
}

/*!
   Display matching between keypoints detected in the current image and with those detected in the multiple training
   images. Display also RANSAC inliers if the list is supplied.

   \param ICurrent : Current image.
   \param IMatching : Resulting matching image.
   \param ransacInliers : List of Ransac inliers or empty list if not available.
   \param crossSize : Size of the displayed crosses.
   \param lineThickness : Thickness of the displayed line.
 */
void vpKeyPoint::displayMatching(const vpImage<unsigned char> &ICurrent, vpImage<unsigned char> &IMatching,
                                 const std::vector<vpImagePoint> &ransacInliers, unsigned int crossSize, unsigned int lineThickness) {
  if(m_mapOfImages.empty()) {
    //No training images so return
    return;
  }

  //Nb images in the training database + the current image we want to detect the object
  int nbImg = (int) (m_mapOfImages.size() + 1);

  if(nbImg == 2) {
    //Only one training image, so we display the matching result side-by-side
    displayMatching(m_mapOfImages.begin()->second, IMatching, crossSize);
  } else {
    //Multiple training images, display them as a mosaic image
    //round(std::sqrt((double) nbImg)); VC++ compiler does not have round so the next line is used
    //Different implementations of round exist, here round to the closest integer but will not work for negative numbers
    int nbImgSqrt = (int) std::floor(std::sqrt((double) nbImg) + 0.5);
    int nbWidth = nbImgSqrt;
    int nbHeight = nbImgSqrt;

    if(nbImgSqrt * nbImgSqrt < nbImg) {
      nbWidth++;
    }

    std::map<int, int> mapOfImageIdIndex;
    int cpt = 0;
    unsigned int maxW = ICurrent.getWidth(), maxH = ICurrent.getHeight();
    for(std::map<int, vpImage<unsigned char> >::const_iterator it = m_mapOfImages.begin(); it != m_mapOfImages.end(); ++it, cpt++) {
      mapOfImageIdIndex[it->first] = cpt;

      if(maxW < it->second.getWidth()) {
        maxW = it->second.getWidth();
      }

      if(maxH < it->second.getHeight()) {
        maxH = it->second.getHeight();
      }
    }

    //Indexes of the current image in the grid made in order to the image is in the center square in the mosaic grid
    int medianI = nbHeight / 2;
    int medianJ = nbWidth / 2;
    int medianIndex = medianI * nbWidth + medianJ;
    for(std::vector<cv::KeyPoint>::const_iterator it = m_trainKeyPoints.begin(); it != m_trainKeyPoints.end(); ++it) {
      vpImagePoint topLeftCorner;
      int current_class_id_index = 0;
      if(mapOfImageIdIndex[m_mapOfImageId[it->class_id]] < medianIndex) {
        current_class_id_index = mapOfImageIdIndex[m_mapOfImageId[it->class_id]];
      } else {
        //Shift of one unity the index of the training images which are after the current image
        current_class_id_index = mapOfImageIdIndex[m_mapOfImageId[it->class_id]] + 1;
      }

      int indexI = current_class_id_index / nbWidth;
      int indexJ = current_class_id_index - (indexI * nbWidth);
      topLeftCorner.set_ij(maxH*indexI, maxW*indexJ);

      //Display cross for keypoints in the learning database
      vpDisplay::displayCross(IMatching, (int) (it->pt.y + topLeftCorner.get_i()), (int) (it->pt.x + topLeftCorner.get_j()),
                              crossSize, vpColor::red);
    }

    vpImagePoint topLeftCorner(maxH*medianI, maxW*medianJ);
    for(std::vector<cv::KeyPoint>::const_iterator it = m_queryKeyPoints.begin(); it != m_queryKeyPoints.end(); ++it) {
      //Display cross for keypoints detected in the current image
      vpDisplay::displayCross(IMatching, (int) (it->pt.y + topLeftCorner.get_i()), (int) (it->pt.x + topLeftCorner.get_j()),
                              crossSize, vpColor::red);
    }
    for(std::vector<vpImagePoint>::const_iterator it = ransacInliers.begin();
        it != ransacInliers.end(); ++it) {
      //Display green circle for RANSAC inliers
      vpDisplay::displayCircle(IMatching, (int) (it->get_v() + topLeftCorner.get_i()), (int) (it->get_u() +
                                                                                              topLeftCorner.get_j()), 4, vpColor::green);
    }
    for(std::vector<vpImagePoint>::const_iterator it = m_ransacOutliers.begin(); it != m_ransacOutliers.end(); ++it) {
      //Display red circle for RANSAC outliers
      vpDisplay::displayCircle(IMatching, (int) (it->get_i() + topLeftCorner.get_i()), (int) (it->get_j() +
                                                                                              topLeftCorner.get_j()), 4, vpColor::red);
    }

    for(std::vector<std::pair<cv::KeyPoint, cv::KeyPoint> >::const_iterator it = m_matchQueryToTrainKeyPoints.begin();
        it != m_matchQueryToTrainKeyPoints.end(); ++it) {
      int current_class_id = 0;
      if(mapOfImageIdIndex[m_mapOfImageId[it->second.class_id]] < medianIndex) {
        current_class_id = mapOfImageIdIndex[m_mapOfImageId[it->second.class_id]];
      } else {
        //Shift of one unity the index of the training images which are after the current image
        current_class_id = mapOfImageIdIndex[m_mapOfImageId[it->second.class_id]] + 1;
      }

      int indexI = current_class_id / nbWidth;
      int indexJ = current_class_id - (indexI * nbWidth);

      vpImagePoint end(maxH*indexI + it->second.pt.y, maxW*indexJ + it->second.pt.x);
      vpImagePoint start(maxH*medianI + it->first.pt.y, maxW*medianJ + it->first.pt.x);

      //Draw line for matching keypoints detected in the current image and those detected
      //in the training images
      vpDisplay::displayLine(IMatching, start, end, vpColor::green, lineThickness);
    }
  }
}

/*!
   Extract the descriptors for each keypoints of the list.

   \param I : Input image.
   \param keyPoints : List of keypoints we want to extract their descriptors.
   \param descriptors : Descriptors matrix with at each row the descriptors values for each keypoint.
   \param elapsedTime : Elapsed time.
 */
void vpKeyPoint::extract(const vpImage<unsigned char> &I, std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors,
                         double &elapsedTime) {
  double t = vpTime::measureTimeMs();
  cv::Mat matImg;
  vpImageConvert::convert(I, matImg, false);
  bool first = true;
  for(std::map<std::string, cv::Ptr<cv::DescriptorExtractor> >::const_iterator it = m_extractors.begin();
      it != m_extractors.end(); ++it) {
    if(first) {
      first = false;
      it->second->compute(matImg, keyPoints, descriptors);
    } else {
      cv::Mat desc;
      it->second->compute(matImg, keyPoints, desc);
      if(descriptors.empty()) {
        desc.copyTo(descriptors);
      } else {
        cv::hconcat(descriptors, desc, descriptors);
      }
    }
  }
  elapsedTime = vpTime::measureTimeMs() - t;
}

/*!
   Filter the matches using the desired filtering method.
 */
void vpKeyPoint::filterMatches() {
  m_matchQueryToTrainKeyPoints.clear();

  std::vector<cv::KeyPoint> queryKpts;
  std::vector<cv::Point3f> trainPts;
  std::vector<cv::DMatch> m;

  if(m_useKnn) {
    double max_dist = 0;
    //double min_dist = std::numeric_limits<double>::max(); // create an error under Windows. To fix it we have to add #undef max
    double min_dist = DBL_MAX;
    double mean = 0.0;
    std::vector<double> distance_vec(m_queryDescriptors.rows);

    if(m_filterType == stdAndRatioDistanceThreshold) {
      for(size_t i = 0; i < m_knnMatches.size(); i++) {
        double dist = m_knnMatches[i][0].distance;
        mean += dist;
        distance_vec[i] = dist;

        if (dist < min_dist) {
          min_dist = dist;
        }
        if (dist > max_dist) {
          max_dist = dist;
        }
      }
      mean /= m_queryDescriptors.rows;
    }

    double sq_sum = std::inner_product(distance_vec.begin(), distance_vec.end(), distance_vec.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / distance_vec.size() - mean * mean);
    double threshold = min_dist + stdev;

    for(size_t i = 0; i < m_knnMatches.size(); i++) {
      if(m_knnMatches[i].size() >= 2) {
        //Calculate ratio of the descriptor distance between the two nearest neighbors of the keypoint
        float ratio = m_knnMatches[i][0].distance / m_knnMatches[i][1].distance;
//        float ratio = std::sqrt((vecMatches[i][0].distance * vecMatches[i][0].distance)
//            / (vecMatches[i][1].distance * vecMatches[i][1].distance));
        double dist = m_knnMatches[i][0].distance;
        if(ratio < m_matchingRatioThreshold || (m_filterType == stdAndRatioDistanceThreshold && dist < threshold)) {
          m.push_back(cv::DMatch((int) queryKpts.size(), m_knnMatches[i][0].trainIdx, m_knnMatches[i][0].distance));

          if(!m_trainPoints.empty()) {
            trainPts.push_back(m_trainPoints[m_knnMatches[i][0].trainIdx]);
          }
          queryKpts.push_back(m_queryKeyPoints[m_knnMatches[i][0].queryIdx]);

          //Add the pair with the correspondence between the detected keypoints and the one matched in the train keypoints list
          m_matchQueryToTrainKeyPoints.push_back(std::pair<cv::KeyPoint, cv::KeyPoint>(
                                                m_queryKeyPoints[m_knnMatches[i][0].queryIdx], m_trainKeyPoints[m_knnMatches[i][0].trainIdx]));
        }
      }
    }
  } else {
    double max_dist = 0;
    //double min_dist = std::numeric_limits<double>::max(); // create an error under Windows. To fix it we have to add #undef max
    double min_dist = DBL_MAX;
    double mean = 0.0;
    std::vector<double> distance_vec(m_queryDescriptors.rows);
    for (int i = 0; i < m_queryDescriptors.rows; i++) {
      double dist = m_matches[i].distance;
      mean += dist;
      distance_vec[i] = dist;

      if (dist < min_dist) {
        min_dist = dist;
      }
      if (dist > max_dist) {
        max_dist = dist;
      }
    }
    mean /= m_queryDescriptors.rows;

    double sq_sum = std::inner_product(distance_vec.begin(), distance_vec.end(), distance_vec.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / distance_vec.size() - mean * mean);

    //  std::cout << "stdev=" << stdev << std::endl;
    //    std::cout << "min_dist=" << min_dist << " ; max_dist=" << max_dist << std::endl;

    //Define a threshold where we keep all keypoints whose the descriptor distance falls below a factor of the
    //minimum descriptor distance (for all the query keypoints)
    //or below the minimum descriptor distance + the standard deviation (calculated on all the query descriptor distances)
    double threshold = m_filterType == constantFactorDistanceThreshold ? m_matchingFactorThreshold * min_dist : min_dist + stdev;

    for (size_t i = 0; i < m_matches.size(); i++) {
      if(m_matches[i].distance <= threshold) {
        m.push_back(cv::DMatch((int) queryKpts.size(), m_matches[i].trainIdx, m_matches[i].distance));

        if(!m_trainPoints.empty()) {
          trainPts.push_back(m_trainPoints[m_matches[i].trainIdx]);
        }
        queryKpts.push_back(m_queryKeyPoints[m_matches[i].queryIdx]);

        //Add the pair with the correspondence between the detected keypoints and the one matched in the train keypoints list
        m_matchQueryToTrainKeyPoints.push_back(std::pair<cv::KeyPoint, cv::KeyPoint>(
                                              m_queryKeyPoints[m_matches[i].queryIdx], m_trainKeyPoints[m_matches[i].trainIdx]));
      }
    }
  }

  m_filteredMatches = m;
  m_objectFilteredPoints = trainPts;
  m_queryFilteredKeyPoints = queryKpts;
}

/*!
   Get the 3D coordinates of the object points matched (the corresponding 3D coordinates in the object frame
   of the keypoints detected in the current image after the matching).

   \param objectPoints : List of 3D coordinates in the object frame.
 */
void vpKeyPoint::getObjectPoints(std::vector<cv::Point3f> &objectPoints) {
  objectPoints = m_objectFilteredPoints;
}

/*!
   Compute the pose using the correspondence between 2D points and 3D points using OpenCV function with RANSAC method.

   \param imagePoints : List of 2D points corresponding to the location of the detected keypoints.
   \param  objectPoints : List of the 3D points in the object frame matched.
   \param cam : Camera parameters.
   \param cMo : Homogeneous matrix between the object frame and the camera frame.
   \param inlierIndex : List of indexes of inliers.
   \param elapsedTime : Elapsed time.
   \return True if the pose has been computed, false otherwise (not enough points, or size list mismatch).
 */
bool vpKeyPoint::getPose(const std::vector<cv::Point2f> &imagePoints, const std::vector<cv::Point3f> &objectPoints,
                         const vpCameraParameters &cam, vpHomogeneousMatrix &cMo, std::vector<int> &inlierIndex, double &elapsedTime) {
  double t = vpTime::measureTimeMs();

  if(imagePoints.size() < 4 || objectPoints.size() < 4 || imagePoints.size() != objectPoints.size()) {
    elapsedTime = (vpTime::measureTimeMs() - t);
    return false;
  }

  cv::Mat cameraMatrix =
      (cv::Mat_<double>(3, 3) << cam.get_px(), 0, cam.get_u0(), 0, cam.get_py(), cam.get_v0(), 0, 0, 1);
  cv::Mat rvec, tvec;
  cv::Mat distCoeffs;

  cv::solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs,
                     rvec, tvec, false, m_nbRansacIterations, (float) m_ransacReprojectionError,
                     m_nbRansacMinInlierCount, inlierIndex);
  vpTranslationVector translationVec(tvec.at<double>(0),
                                     tvec.at<double>(1), tvec.at<double>(2));
  vpThetaUVector thetaUVector(rvec.at<double>(0), rvec.at<double>(1),
                              rvec.at<double>(2));
  cMo = vpHomogeneousMatrix(translationVec, thetaUVector);

  elapsedTime = (vpTime::measureTimeMs() - t);

  return true;
}

/*!
   Compute the pose using the correspondence between 2D points and 3D points using ViSP function with RANSAC method.

   \param objectVpPoints : List of vpPoint with coordinates expressed in the object and in the camera frame.
   \param cMo : Homogeneous matrix between the object frame and the camera frame.
   \param inliers : List of inliers.
   \param elapsedTime : Elapsed time.
   \return True if the pose has been computed, false otherwise (not enough points, or size list mismatch).
 */
bool vpKeyPoint::getPose(const std::vector<vpPoint> &objectVpPoints, vpHomogeneousMatrix &cMo,
                         std::vector<vpPoint> &inliers, double &elapsedTime) {
  double t = vpTime::measureTimeMs();

  if(objectVpPoints.size() < 4) {
    elapsedTime = (vpTime::measureTimeMs() - t);
    return false;
  }

  vpPose pose;

  for(std::vector<vpPoint>::const_iterator it = objectVpPoints.begin(); it != objectVpPoints.end(); ++it) {
    pose.addPoint(*it);
  }

  if(m_useConsensusPercentage) {
    unsigned int nbInlierToReachConsensus = (unsigned int) (m_ransacConsensusPercentage
                                                            * (double) m_queryFilteredKeyPoints.size() / 100.0);
    pose.setRansacNbInliersToReachConsensus(nbInlierToReachConsensus);
  } else {
    pose.setRansacNbInliersToReachConsensus(m_nbRansacMinInlierCount);
  }

  pose.setRansacThreshold(m_ransacThreshold);
  pose.setRansacMaxTrials(m_nbRansacIterations);

  try {
    pose.computePose(vpPose::RANSAC, cMo);
    inliers = pose.getRansacInliers();
  } catch(vpException &e) {
    std::cerr << "e=" << e.what() << std::endl;
    elapsedTime = (vpTime::measureTimeMs() - t);
    return false;
  }

  elapsedTime = (vpTime::measureTimeMs() - t);
  return true;
}

/*!
   Get the descriptors matrix for the query keypoints.

   \param descriptors : Matrix with descriptors values at each row for each query keypoints.
 */
void vpKeyPoint::getQueryDescriptors(cv::Mat &descriptors) {
  descriptors = m_queryDescriptors;
}

/*!
   Get the descriptors list for the query keypoints.

   \param descriptors : List of descriptors values.
 */
void vpKeyPoint::getQueryDescriptors(std::vector<std::vector<float> > &descriptors) {
  int nRows = m_queryDescriptors.rows;
  int nCols = m_queryDescriptors.cols;
  descriptors.resize(nRows);

  for(int i = 0; i < nRows; i++) {
    std::vector<float> desc(nCols);
    for(int j = 0; j < nCols; j++) {
      desc[j] = m_queryDescriptors.at<float>(i, j);
    }
    descriptors[i] = desc;
  }
}

/*!
   Get the query keypoints list in OpenCV type.

   \param keyPoints : List of query keypoints (or keypoints detected in the current image).
 */
void vpKeyPoint::getQueryKeyPoints(std::vector<cv::KeyPoint> &keyPoints) {
  keyPoints = m_queryFilteredKeyPoints;
}

/*!
   Get the query keypoints list in ViSP type.

   \param keyPoints : List of query keypoints (or keypoints detected in the current image).
 */
void vpKeyPoint::getQueryKeyPoints(std::vector<vpImagePoint> &keyPoints) {
  keyPoints = currentImagePointsList;
}

/*!
   Get the train descriptors matrix.

   \param descriptors : Matrix with descriptors values at each row for each train keypoints (or reference keypoints).
 */
void vpKeyPoint::getTrainDescriptors(cv::Mat &descriptors) {
  descriptors = m_trainDescriptors;
}

/*!
   Get the train descriptors list.

   \param descriptors : List of train descriptors values (or reference descriptors values).
 */
void vpKeyPoint::getTrainDescriptors(std::vector<std::vector<float> > &descriptors) {
  int nRows = m_trainDescriptors.rows;
  int nCols = m_trainDescriptors.cols;
  descriptors.resize(nRows);

  for(int i = 0; i < nRows; i++) {
    std::vector<float> desc(nCols);
    for(int j = 0; j < nCols; j++) {
      desc[j] = m_trainDescriptors.at<float>(i, j);
    }
    descriptors[i] = desc;
  }
}

/*!
   Get the train keypoints list in OpenCV type.

   \param keyPoints : List of train keypoints (or reference keypoints).
 */
void vpKeyPoint::getTrainKeyPoints(std::vector<cv::KeyPoint> &keyPoints) {
  keyPoints = m_trainKeyPoints;
}

/*!
   Get the train keypoints list in ViSP type.

   \param keyPoints : List of train keypoints (or reference keypoints).
 */
void vpKeyPoint::getTrainKeyPoints(std::vector<vpImagePoint> &keyPoints) {
  keyPoints = referenceImagePointsList;
}

/*!
   Get the train points (the 3D coordinates in the object frame) list in OpenCV type.

   \param points : List of train points (or reference points).
 */
void vpKeyPoint::getTrainPoints(std::vector<cv::Point3f> &points) {
  points = m_trainPoints;
}

/*!
   Get the train points (the 3D coordinates in the object frame) list in ViSP type.

   \param points : List of train points (or reference points).
 */
void vpKeyPoint::getTrainPoints(std::vector<vpPoint> &points) {
  points = m_trainVpPoints;
}

/*!
   Initialize method for RANSAC parameters and for detectors, extractors and matcher, and for others parameters.
 */
void vpKeyPoint::init() {
  m_useRansacVVS = false;
  m_useConsensusPercentage = false;
  m_matchingFactorThreshold = 2.0;
  m_matchingRatioThreshold = 0.85;
  m_nbRansacIterations = 200;
  m_ransacReprojectionError = 6.0; //In pixel, used in OpenCV function
  m_nbRansacMinInlierCount = 100;
  m_ransacThreshold = 0.001; //In meter, used in ViSP function
  m_ransacConsensusPercentage = 20.0;

  m_detectionTime = -1.0;
  m_extractionTime = -1.0;
  m_matchingTime = -1.0;
  m_poseTime = -1.0;

  m_currentImageId = 0;

#if defined(VISP_HAVE_OPENCV_NONFREE) && (VISP_HAVE_OPENCV_VERSION >= 0x020400) // Require opencv >= 2.4.0
  //The following line must be called in order to use SIFT or SURF
  if (!cv::initModule_nonfree()) {
    std::cerr << "Cannot init module non free, SIFT or SURF cannot be used."
        << std::endl;
  }
#endif

  initDetectors(m_detectorNames);
  initExtractors(m_extractorNames);
  initMatcher(m_matcherName);
}

/*!
   Initialize a keypoint detector based on its name.

   \param detectorName : Name of the detector (e.g FAST, SIFT, SURF, etc.).
 */
void vpKeyPoint::initDetector(const std::string &detectorName) {
  // TODO: Fix compat with OpenCV 3.0.0
  //  std::string pyramid = "Pyramid";
  //  std::size_t pos = detectorName.find(pyramid);
  //  if(pos != std::string::npos) {
  //    std::string sub = detectorName.substr(pos + pyramid.size());
  //    detectors[detectorName] = cv::Ptr<cv::FeatureDetector>(
  //        new cv::PyramidAdaptedFeatureDetector(cv::FeatureDetector::create<cv::FeatureDetector>(sub), 2));
  //  } else {
  //    detectors[detectorName] = cv::FeatureDetector::create<cv::FeatureDetector>(detectorName);
  //  }
#if (VISP_HAVE_OPENCV_VERSION < 0x030000)
  m_detectors[detectorName] = cv::FeatureDetector::create(detectorName);
#else
  m_detectors[detectorName] = cv::FeatureDetector::create<cv::FeatureDetector>(detectorName);
#endif
}

/*!
   Initialize a list of keypoints detectors if we want to concatenate multiple detectors.

   \param detectorNames : List of detector names.
 */
void vpKeyPoint::initDetectors(const std::vector<std::string> &detectorNames) {
  for(std::vector<std::string>::const_iterator it = detectorNames.begin(); it != detectorNames.end(); ++it) {
    initDetector(*it);
  }
}

/*!
   Initialize a descriptor extractor based on its name.

   \param extractorName : Name of the extractor (e.g SIFT, SURF, ORB, etc.).
 */
void vpKeyPoint::initExtractor(const std::string &extractorName) {
#if (VISP_HAVE_OPENCV_VERSION < 0x030000)
  m_extractors[extractorName] = cv::DescriptorExtractor::create(extractorName);
#else
  m_extractors[extractorName] = cv::DescriptorExtractor::create<cv::DescriptorExtractor>(extractorName);
#endif
}

/*!
   Initialize a list of descriptor extractors if we want to concatenate multiple extractors.

   \param extractorNames : List of extractor names.
 */
void vpKeyPoint::initExtractors(const std::vector<std::string> &extractorNames) {
  for(std::vector<std::string>::const_iterator it = extractorNames.begin(); it != extractorNames.end(); ++it) {
    initExtractor(*it);
  }
}

/*!
   Initialize a matcher based on its name.

   \param matcherName : Name of the matcher (e.g BruteForce, FlannBased).
 */
void vpKeyPoint::initMatcher(const std::string &matcherName) {
  m_matcher = cv::DescriptorMatcher::create(matcherName);
}

/*!
   Insert a reference image and a current image side-by-side.

   \param IRef : Reference image.
   \param ICurrent : Current image.
   \param IMatching : Matching image for displaying all the matching between the query keypoints and those
   detected in the training images.
 */
void vpKeyPoint::insertImageMatching(const vpImage<unsigned char> &IRef, const vpImage<unsigned char> &ICurrent,
                                     vpImage<unsigned char> &IMatching) {
  vpImagePoint topLeftCorner(0, 0);
  IMatching.insert(IRef, topLeftCorner);
  topLeftCorner = vpImagePoint(0, IRef.getWidth());
  IMatching.insert(ICurrent, topLeftCorner);
}

/*!
   Insert the different training images in the matching image.

   \param ICurrent : Current image.
   \param IMatching : Matching image for displaying all the matching between the query keypoints and those
   detected in the training images
 */
void vpKeyPoint::insertImageMatching(const vpImage<unsigned char> &ICurrent, vpImage<unsigned char> &IMatching) {
  //Nb images in the training database + the current image we want to detect the object
  int nbImg = (int) (m_mapOfImages.size() + 1);

  if(nbImg == 2) {
    //Only one training image, so we display them side by side
    insertImageMatching(m_mapOfImages.begin()->second, ICurrent, IMatching);
  } else {
    //Multiple training images, display them as a mosaic image
    //round(std::sqrt((double) nbImg)); VC++ compiler does not have round so the next line is used
    //Different implementations of round exist, here round to the closest integer but will not work for negative numbers
    int nbImgSqrt = (int) std::floor(std::sqrt((double) nbImg) + 0.5);
    int nbWidth = nbImgSqrt;
    int nbHeight = nbImgSqrt;

    if(nbImgSqrt * nbImgSqrt < nbImg) {
      nbWidth++;
    }

    unsigned int maxW = ICurrent.getWidth(), maxH = ICurrent.getHeight();
    for(std::map<int, vpImage<unsigned char> >::const_iterator it = m_mapOfImages.begin(); it != m_mapOfImages.end(); ++it) {
      if(maxW < it->second.getWidth()) {
        maxW = it->second.getWidth();
      }

      if(maxH < it->second.getHeight()) {
        maxH = it->second.getHeight();
      }
    }

    //Indexes of the current image in the grid made in order to the image is in the center square in the mosaic grid
    int medianI = nbHeight / 2;
    int medianJ = nbWidth / 2;
    int medianIndex = medianI * nbWidth + medianJ;

    int cpt = 0;
    for(std::map<int, vpImage<unsigned char> >::const_iterator it = m_mapOfImages.begin(); it != m_mapOfImages.end(); ++it, cpt++) {
      int local_cpt = cpt;
      if(cpt >= medianIndex) {
        //Shift of one unity the index of the training images which are after the current image
        local_cpt++;
      }
      int indexI = local_cpt / nbWidth;
      int indexJ = local_cpt - (indexI * nbWidth);
      vpImagePoint topLeftCorner(maxH*indexI, maxW*indexJ);

      IMatching.insert(it->second, topLeftCorner);
    }

    vpImagePoint topLeftCorner(maxH*medianI, maxW*medianJ);
    IMatching.insert(ICurrent, topLeftCorner);
  }
}

#ifdef VISP_HAVE_XML2
/*!
   Load configuration parameters from an XML config file.

   \param configFile : Path to the XML config file.
 */
void vpKeyPoint::loadConfigFile(const std::string &configFile) {
  vpXmlConfigParserKeyPoint xmlp;

  try {
    //Reset detector and extractor
    m_detectorNames.clear();
    m_extractorNames.clear();
    m_detectors.clear();
    m_extractors.clear();

    std::cout << " *********** Parsing XML for configuration for vpKeyPoint ************ " << std::endl;
    xmlp.parse(configFile);

    m_detectorNames.push_back(xmlp.getDetectorName());
    m_extractorNames.push_back(xmlp.getExtractorName());
    m_matcherName = xmlp.getMatcherName();

    switch(xmlp.getMatchingMethod()) {
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

    if(m_filterType == ratioDistanceThreshold || m_filterType == stdAndRatioDistanceThreshold) {
      m_useKnn = true;
    } else {
      m_useKnn = false;
    }

    init();
  }
  catch(...) {
    throw vpException(vpException::ioError, "Can't open XML file \"%s\"\n ", configFile.c_str());
  }
}
#endif

/*!
   Load learning data saved on disk.

   \param filename : Path of the learning file.
   \param binaryMode : If true, the learning file is in a binary mode, otherwise it is in XML mode.
   \param append : If true, concatenate the learning data, otherwise reset the variables.
 */
void vpKeyPoint::loadLearningData(const std::string &filename, const bool binaryMode, const bool append) {
  if(!append) {
    m_trainKeyPoints.clear();
    m_trainPoints.clear();
    m_mapOfImageId.clear();
    m_mapOfImages.clear();
  }

  //Get parent directory
  std::string parent = vpIoTools::getParent(filename) + "/";

  if(binaryMode) {
    std::ifstream file(filename.c_str(), std::ifstream::binary);
    if(!file.is_open()){
      throw vpException(vpException::ioError, "Cannot open the file.");
    }

    //Read info about training images
    int nbImgs = 0;
    file.read((char *)(&nbImgs), sizeof(nbImgs));

    for(int i = 0; i < nbImgs; i++) {
      //Read image_id
      int id = 0;
      file.read((char *)(&id), sizeof(id));

      int length = 0;
      file.read((char *)(&length), sizeof(length));
      //Will contain the path to the training images
      char* path = new char[length + 1];//char path[length + 1];

      for(int cpt = 0; cpt < length; cpt++) {
        char c;
        file.read((char *)(&c), sizeof(c));
        path[cpt] = c;
      }
      path[length] = '\0';

      vpImage<unsigned char> I;
      if(vpIoTools::isAbsolutePathname(std::string(path))) {
        vpImageIo::read(I, path);
      } else {
        vpImageIo::read(I, parent + path);
      }

      m_mapOfImages[id] = I;
    }

    //Read if 3D point information are saved or not
    int have3DInfoInt = 0;
    file.read((char *)(&have3DInfoInt), sizeof(have3DInfoInt));
    bool have3DInfo = have3DInfoInt != 0;

    //Read the number of descriptors
    int nRows = 0;
    file.read((char *)(&nRows), sizeof(nRows));

    //Read the size of the descriptor
    int nCols = 0;
    file.read((char *)(&nCols), sizeof(nCols));

    std::vector<std::vector<double> > descriptorValue;
    for(int i = 0; i < nRows; i++) {
      std::vector<double> rowValue;

      //Read information about keyPoint
      float u, v, size, angle, response;
      int octave, class_id, image_id;
      file.read((char *)(&u), sizeof(u));
      file.read((char *)(&v), sizeof(v));
      file.read((char *)(&size), sizeof(size));
      file.read((char *)(&angle), sizeof(angle));
      file.read((char *)(&response), sizeof(response));
      file.read((char *)(&octave), sizeof(octave));
      file.read((char *)(&class_id), sizeof(class_id));
      file.read((char *)(&image_id), sizeof(image_id));
      cv::KeyPoint keyPoint(cv::Point2f(u, v), size, angle, response, octave, class_id);
      m_trainKeyPoints.push_back(keyPoint);

      if(image_id != -1) {
        //No training images if image_id == -1
        m_mapOfImageId[class_id] = image_id;
      }

      if(have3DInfo) {
        //Read oX, oY, oZ
        float oX, oY, oZ;
        file.read((char *)(&oX), sizeof(oX));
        file.read((char *)(&oY), sizeof(oY));
        file.read((char *)(&oZ), sizeof(oZ));
        m_trainPoints.push_back(cv::Point3f(oX, oY, oZ));
      }

      for(int j = 0; j < nCols; j++) {
        //Read the value of the descriptor
        float value;
        file.read((char *)(&value), sizeof(value));
        rowValue.push_back(value);
      }

      descriptorValue.push_back(rowValue);
    }

    m_trainDescriptors = cv::Mat((int) descriptorValue.size(),
                               (int) descriptorValue[0].size(), CV_32F);
    int i = 0;
    for (std::vector<std::vector<double> >::const_iterator it1 =
         descriptorValue.begin(); it1 != descriptorValue.end(); ++it1, i++) {
      int j = 0;
      for (std::vector<double>::const_iterator it2 = it1->begin();
           it2 != it1->end(); ++it2, j++) {
        m_trainDescriptors.at<float>(i, j) = (float) *it2;
      }
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

    std::vector<std::vector<double> > descriptor_value;
    xmlNodePtr first_level_node = NULL;
    char *pEnd = NULL;

    for (first_level_node = root_element->children; first_level_node;
         first_level_node = first_level_node->next) {

      std::string name((char *) first_level_node->name);
      if (first_level_node->type == XML_ELEMENT_NODE && name == "TrainingImageInfo") {
        xmlNodePtr image_info_node = NULL;

        for (image_info_node = first_level_node->children; image_info_node; image_info_node =
             image_info_node->next) {
          name = std::string ((char *) image_info_node->name);

          if(name == "trainImg") {
            //Read image_id
            int id = std::atoi((char *) xmlGetProp(image_info_node, BAD_CAST "image_id"));

            vpImage<unsigned char> I;
            std::string path((char *) image_info_node->children->content);
            //Read path to the training images
            if(vpIoTools::isAbsolutePathname(std::string(path))) {
              vpImageIo::read(I, path);
            } else {
              vpImageIo::read(I, parent + path);
            }

            m_mapOfImages[id] = I;
          }
        }
      } else if (first_level_node->type == XML_ELEMENT_NODE && name == "DescriptorInfo") {
        xmlNodePtr point_node = NULL;
        double u = 0.0, v = 0.0, size = 0.0, angle = 0.0, response = 0.0;
        int octave = 0, class_id = 0, image_id = 0;
        double oX = 0.0, oY = 0.0, oZ = 0.0;
        std::vector<double> row_value;

        for (point_node = first_level_node->children; point_node; point_node =
             point_node->next) {
          if (point_node->type == XML_ELEMENT_NODE) {
            name = std::string ((char *) point_node->name);

            //Read information about keypoints
            if(name == "u") {
              u = std::strtod((char *) point_node->children->content, &pEnd);
            } else if(name == "v") {
              v = std::strtod((char *) point_node->children->content, &pEnd);
            } else if(name == "size") {
              size = std::strtod((char *) point_node->children->content, &pEnd);
            } else if(name == "angle") {
              angle = std::strtod((char *) point_node->children->content, &pEnd);
            } else if(name == "response") {
              response = std::strtod((char *) point_node->children->content, &pEnd);
            } else if(name == "octave") {
              octave = std::atoi((char *) point_node->children->content);
            } else if(name == "class_id") {
              class_id = std::atoi((char *) point_node->children->content);
              cv::KeyPoint keyPoint(cv::Point2f((float) u, (float) v), (float) size,
                                    (float) angle, (float) response, octave, class_id);
              m_trainKeyPoints.push_back(keyPoint);
            } else if(name == "image_id") {
              image_id = std::atoi((char *) point_node->children->content);
              if(image_id != -1) {
                //No training images if image_id == -1
                m_mapOfImageId[m_trainKeyPoints.back().class_id] = image_id;
              }
            } else if (name == "oX") {
              oX = std::atof((char *) point_node->children->content);
            } else if (name == "oY") {
              oY = std::atof((char *) point_node->children->content);
            } else if (name == "oZ") {
              oZ = std::atof((char *) point_node->children->content);
              m_trainPoints.push_back(cv::Point3f((float) oX, (float) oY, (float) oZ));
            } else if (name == "desc") {
              xmlNodePtr descriptor_value_node = NULL;
              for (descriptor_value_node = point_node->children;
                   descriptor_value_node; descriptor_value_node =
                   descriptor_value_node->next) {

                if (descriptor_value_node->type == XML_ELEMENT_NODE) {
                  //Read descriptors values
                  row_value.push_back(
                        std::atof(
                          (char *) descriptor_value_node->children->content));
                }
              }
            }
          }
        }

        descriptor_value.push_back(row_value);
      }
    }

    m_trainDescriptors = cv::Mat((int) descriptor_value.size(),
                               (int) descriptor_value[0].size(), CV_32F);
    int i = 0;
    for (std::vector<std::vector<double> >::const_iterator it1 =
         descriptor_value.begin(); it1 != descriptor_value.end(); ++it1, i++) {
      int j = 0;
      for (std::vector<double>::const_iterator it2 = it1->begin();
           it2 != it1->end(); ++it2, j++) {
        m_trainDescriptors.at<float>(i, j) = (float) *it2;
      }
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

  //Convert OpenCV type to ViSP type for compatibility
  convertToVpType(m_trainKeyPoints, referenceImagePointsList);
  convertToVpType(this->m_trainPoints, m_trainVpPoints);
}

/*!
   Match keypoints based on distance between their descriptors.

   \param trainDescriptors : Train descriptors (or reference descriptors).
   \param queryDescriptors : Query descriptors.
   \param matches : Output list of matches.
   \param elapsedTime : Elapsed time.
 */
void vpKeyPoint::match(const cv::Mat &trainDescriptors, const cv::Mat &queryDescriptors,
                       std::vector<cv::DMatch> &matches, double &elapsedTime) {
  double t = vpTime::measureTimeMs();

  if(m_useKnn) {
    m_matcher->knnMatch(queryDescriptors, trainDescriptors, m_knnMatches, 2);
    matches.resize(m_knnMatches.size());
    std::transform(m_knnMatches.begin(), m_knnMatches.end(), matches.begin(), knnToDMatch);
  } else {
    m_matcher->match(queryDescriptors, trainDescriptors, matches);
  }
  elapsedTime = vpTime::measureTimeMs() - t;
}

/*!
   Match keypoints detected in the image with those built in the reference list.

   \param I : Input current image.
   \return The number of matched keypoints.
 */
unsigned int vpKeyPoint::matchPoint(const vpImage<unsigned char> &I) {
  detect(I, m_queryKeyPoints, m_detectionTime);
  extract(I, m_queryKeyPoints, m_queryDescriptors, m_extractionTime);
  match(m_trainDescriptors, m_queryDescriptors, m_matches, m_matchingTime);

  if(m_filterType != noFilterMatching) {
    m_queryFilteredKeyPoints.clear();
    m_filteredMatches.clear();

    filterMatches();
  } else {
    m_queryFilteredKeyPoints = m_queryKeyPoints;
    m_objectFilteredPoints = m_trainPoints;
    m_filteredMatches = m_matches;

    for (size_t i = 0; i < m_matches.size(); i++) {
      //Add the correspondence between query and train keypoints
      m_matchQueryToTrainKeyPoints.push_back(std::pair<cv::KeyPoint, cv::KeyPoint>(
                                            m_queryKeyPoints[m_matches[i].queryIdx], m_trainKeyPoints[m_matches[i].trainIdx]));
    }
  }

  //Convert OpenCV type to ViSP type for compatibility
  convertToVpType(m_queryFilteredKeyPoints, currentImagePointsList);
  convertToVpType(m_filteredMatches, matchedReferencePoints);

  return static_cast<unsigned int>(m_filteredMatches.size());
}

/*!
   Match keypoints detected in a region of interest of the image with those
   built in the reference list.

   \param I : Input image
   \param iP : Coordinate of the top-left corner of the region of interest
   \param height : Height of the region of interest
   \param width : Width of the region of interest
   \return The number of matched keypoints
 */
unsigned int vpKeyPoint::matchPoint(const vpImage<unsigned char> &I,
                                    const vpImagePoint &iP,
                                    const unsigned int height, const unsigned int width) {
  return matchPoint(I, vpRect(iP, width, height));
}

/*!
   Match keypoints detected in a region of interest of the image with those
   built in the reference list.

   \param I : Input image
   \param rectangle : Rectangle of the region of interest
   \return The number of matched keypoints
 */
unsigned int vpKeyPoint::matchPoint(const vpImage<unsigned char> &I,
                                    const vpRect& rectangle) {
  detect(I, m_queryKeyPoints, m_detectionTime, rectangle);
  extract(I, m_queryKeyPoints, m_queryDescriptors, m_extractionTime);
  match(m_trainDescriptors, m_queryDescriptors, m_matches, m_matchingTime);

  if(m_filterType != noFilterMatching) {
    m_queryFilteredKeyPoints.clear();
    m_objectFilteredPoints = m_trainPoints;
    m_filteredMatches.clear();

    filterMatches();
  } else {
    m_queryFilteredKeyPoints = m_queryKeyPoints;
    m_filteredMatches = m_matches;

    for (size_t i = 0; i < m_matches.size(); i++) {
      m_matchQueryToTrainKeyPoints.push_back(std::pair<cv::KeyPoint, cv::KeyPoint>(
                                            m_queryKeyPoints[m_matches[i].queryIdx], m_trainKeyPoints[m_matches[i].trainIdx]));
    }
  }

  //Convert OpenCV type to ViSP type for compatibility
  convertToVpType(m_queryFilteredKeyPoints, currentImagePointsList);
  convertToVpType(m_filteredMatches, matchedReferencePoints);

  return static_cast<unsigned int>(m_filteredMatches.size());
}

/*!
   Match keypoints detected in the image with those built in the reference list and compute the pose.

   \param I : Input image
   \param cam : Camera parameters
   \param cMo : Homogeneous matrix between the object frame and the camera frame
   \param error : Reprojection mean square error (in pixel) between the 2D points and the projection of the 3D points with
   the estimated pose
   \param elapsedTime : Time to detect, extract, match and compute the pose
 */
bool vpKeyPoint::matchPoint(const vpImage<unsigned char> &I, const vpCameraParameters &cam, vpHomogeneousMatrix &cMo,
                            double &error, double &elapsedTime) {
  detect(I, m_queryKeyPoints, m_detectionTime);
  extract(I, m_queryKeyPoints, m_queryDescriptors, m_extractionTime);
  match(m_trainDescriptors, m_queryDescriptors, m_matches, m_matchingTime);

  elapsedTime = m_detectionTime + m_extractionTime + m_matchingTime;

  if(m_filterType != noFilterMatching) {
    m_queryFilteredKeyPoints.clear();
    m_objectFilteredPoints.clear();
    m_filteredMatches.clear();

    filterMatches();
  } else {
    m_queryFilteredKeyPoints = m_queryKeyPoints;
    m_objectFilteredPoints = m_trainPoints;
    m_filteredMatches = m_matches;
  }

  //Convert OpenCV type to ViSP type for compatibility
  convertToVpType(m_queryFilteredKeyPoints, currentImagePointsList);
  convertToVpType(m_filteredMatches, matchedReferencePoints);

  //error = std::numeric_limits<double>::max(); // create an error under Windows. To fix it we have to add #undef max
  error = DBL_MAX;
  m_ransacInliers.clear();
  m_ransacOutliers.clear();

  if(m_useRansacVVS) {
    std::vector<vpPoint> objectVpPoints(m_objectFilteredPoints.size());
    size_t cpt = 0;
    for(std::vector<cv::Point3f>::const_iterator it = m_objectFilteredPoints.begin(); it != m_objectFilteredPoints.end();
        ++it, cpt++) {
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
    bool res = getPose(objectVpPoints, cMo, inliers, m_poseTime);
    m_ransacInliers.resize(inliers.size());
    for(size_t i = 0; i < m_ransacInliers.size(); i++) {
      vpMeterPixelConversion::convertPoint(cam, inliers[i].get_x(), inliers[i].get_y(), m_ransacInliers[i]);
    }

    elapsedTime += m_poseTime;

    return res;
  } else {
    std::vector<cv::Point2f> imageFilteredPoints;
    cv::KeyPoint::convert(m_queryFilteredKeyPoints, imageFilteredPoints);
    std::vector<int> inlierIndex;
    bool res = getPose(imageFilteredPoints, m_objectFilteredPoints, cam, cMo, inlierIndex, m_poseTime);

    std::map<int, bool> mapOfInlierIndex;
    m_matchRansacKeyPointsToPoints.clear();
    m_matchRansacQueryToTrainKeyPoints.clear();
    for (std::vector<int>::const_iterator it = inlierIndex.begin(); it != inlierIndex.end(); ++it) {
      m_matchRansacKeyPointsToPoints.push_back(std::pair<cv::KeyPoint, cv::Point3f>(m_queryFilteredKeyPoints[*it],
                                              m_objectFilteredPoints[*it]));
      m_matchRansacQueryToTrainKeyPoints.push_back(std::pair<cv::KeyPoint, cv::KeyPoint>(m_queryFilteredKeyPoints[*it],
                                                  m_trainKeyPoints[m_matches[*it].trainIdx]));
      mapOfInlierIndex[*it] = true;
    }

    for(size_t i = 0; i < m_queryFilteredKeyPoints.size(); i++) {
      if(mapOfInlierIndex.find((int) i) == mapOfInlierIndex.end()) {
        m_ransacOutliers.push_back(vpImagePoint(m_queryFilteredKeyPoints[i].pt.y, m_queryFilteredKeyPoints[i].pt.x));
      }
    }

    error = computePoseEstimationError(m_matchRansacKeyPointsToPoints, cam, cMo);

    m_ransacInliers.resize(m_matchRansacKeyPointsToPoints.size());
    std::transform(m_matchRansacKeyPointsToPoints.begin(), m_matchRansacKeyPointsToPoints.end(), m_ransacInliers.begin(),
                   matchRansacToVpImage);

    elapsedTime += m_poseTime;

    return res;
  }
}

/*!
   Save the learning data in a file in XML or binary mode.

   \param filename : Path of the save file
   \param binaryMode : If true, the data are saved in binary mode, otherwise in XML mode
   \param saveTrainingImages : If true, save also the training images on disk
 */
void vpKeyPoint::saveLearningData(const std::string &filename, bool binaryMode, const bool saveTrainingImages) {
  std::string parent = vpIoTools::getParent(filename);
  if(!parent.empty()) {
    vpIoTools::makeDirectory(parent);
  }

  std::map<int, std::string> mapOfImgPath;
  if(saveTrainingImages) {
    //Save the training image files in the same directory
    int cpt = 0;

    for(std::map<int, vpImage<unsigned char> >::const_iterator it = m_mapOfImages.begin(); it != m_mapOfImages.end(); ++it, cpt++) {
      char buffer[4];
      sprintf(buffer, "%03d", cpt);
      std::stringstream ss;
      ss << "train_image_" << buffer << ".jpg";
      std::string filename = ss.str();
      mapOfImgPath[it->first] = filename;
      vpImageIo::write(it->second, parent + "/" + filename);
    }
  }

  bool have3DInfo = m_trainPoints.size() > 0;
  if(have3DInfo && m_trainPoints.size() != m_trainKeyPoints.size()) {
    throw vpException(vpException::fatalError, "List of keypoints and list of 3D points have different size !");
  }

  if(binaryMode) {
    std::ofstream file(filename.c_str(), std::ofstream::binary);
    if(!file.is_open()) {
      throw vpException(vpException::ioError, "Cannot create the file.");
    }

    //Write info about training images
    int nbImgs = (int) mapOfImgPath.size();
    file.write((char *)(&nbImgs), sizeof(nbImgs));

    for(std::map<int, std::string>::const_iterator it = mapOfImgPath.begin(); it != mapOfImgPath.end(); ++it) {
      //Write image_id
      int id = it->first;
      file.write((char *)(&id), sizeof(id));

      //Write image path
      std::string path = it->second;
      int length = (int) path.length();
      file.write((char *)(&length), sizeof(length));

      for(int cpt = 0; cpt < length; cpt++) {
        file.write((char *) (&path[cpt]), sizeof(path[cpt]));
      }
    }

    //Write if we have 3D point information
    int have3DInfoInt = have3DInfo ? 1 : 0;
    file.write((char *)(&have3DInfoInt), sizeof(have3DInfoInt));


    int nRows = m_trainDescriptors.rows,
        nCols = m_trainDescriptors.cols;
    //Write the number of descriptors
    file.write((char *)(&nRows), sizeof(nRows));

    //Write the size of the descriptor
    file.write((char *)(&nCols), sizeof(nCols));

    for (int i = 0; i < nRows; i++) {
      //Write u
      float u = m_trainKeyPoints[i].pt.x;
      file.write((char *)(&u), sizeof(u));

      //Write v
      float v = m_trainKeyPoints[i].pt.y;
      file.write((char *)(&v), sizeof(v));

      //Write size
      float size = m_trainKeyPoints[i].size;
      file.write((char *)(&size), sizeof(size));

      //Write angle
      float angle = m_trainKeyPoints[i].angle;
      file.write((char *)(&angle), sizeof(angle));

      //Write response
      float response = m_trainKeyPoints[i].response;
      file.write((char *)(&response), sizeof(response));

      //Write octave
      int octave = m_trainKeyPoints[i].octave;
      file.write((char *)(&octave), sizeof(octave));

      //Write class_id
      int class_id = m_trainKeyPoints[i].class_id;
      file.write((char *)(&class_id), sizeof(class_id));

      //Write image_id
      int image_id = (saveTrainingImages && m_mapOfImageId.size() > 0) ? m_mapOfImageId[m_trainKeyPoints[i].class_id] : -1;
      file.write((char *)(&image_id), sizeof(image_id));

      if(have3DInfo) {
        float oX = m_trainPoints[i].x, oY = m_trainPoints[i].y, oZ = m_trainPoints[i].z;
        //Write oX
        file.write((char *)(&oX), sizeof(oX));

        //Write oY
        file.write((char *)(&oY), sizeof(oY));

        //Write oZ
        file.write((char *)(&oZ), sizeof(oZ));
      }

      for (int j = 0; j < nCols; j++) {
        //Write the descriptor value
        file.write((char *)(&m_trainDescriptors.at<float>(i, j)), sizeof(m_trainDescriptors.at<float>(i, j)));
      }
    }


    file.close();
  } else {
#ifdef VISP_HAVE_XML2
    xmlDocPtr doc = NULL;
    xmlNodePtr root_node = NULL, image_node = NULL, image_info_node = NULL, descriptor_node = NULL,
        point_node = NULL, desc_node = NULL, desc_val_node = NULL;

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

    //Write the training images info
    image_node = xmlNewChild(root_node, NULL, BAD_CAST "TrainingImageInfo", NULL);

    for(std::map<int, std::string>::const_iterator it = mapOfImgPath.begin(); it != mapOfImgPath.end(); ++it) {
      image_info_node = xmlNewChild(image_node, NULL, BAD_CAST "trainImg",
                                    BAD_CAST it->second.c_str());
      ss.str("");
      ss << it->first;
      xmlNewProp(image_info_node, BAD_CAST "image_id", BAD_CAST ss.str().c_str());
    }

    int nRows = m_trainDescriptors.rows,
        nCols = m_trainDescriptors.cols;
    for (int i = 0; i < nRows; i++) {
      descriptor_node = xmlNewChild(root_node, NULL, BAD_CAST "DescriptorInfo",
                                    NULL);

      ss.str("");
      ss << m_trainKeyPoints[i].pt.x;
      point_node = xmlNewChild(descriptor_node, NULL, BAD_CAST "u",
                               BAD_CAST ss.str().c_str());

      ss.str("");
      ss << m_trainKeyPoints[i].pt.y;
      point_node = xmlNewChild(descriptor_node, NULL, BAD_CAST "v",
                               BAD_CAST ss.str().c_str());

      ss.str("");
      ss << m_trainKeyPoints[i].size;
      point_node = xmlNewChild(descriptor_node, NULL, BAD_CAST "size",
                               BAD_CAST ss.str().c_str());

      ss.str("");
      ss << m_trainKeyPoints[i].angle;
      point_node = xmlNewChild(descriptor_node, NULL, BAD_CAST "angle",
                               BAD_CAST ss.str().c_str());

      ss.str("");
      ss << m_trainKeyPoints[i].response;
      point_node = xmlNewChild(descriptor_node, NULL, BAD_CAST "response",
                               BAD_CAST ss.str().c_str());

      ss.str("");
      ss << m_trainKeyPoints[i].octave;
      point_node = xmlNewChild(descriptor_node, NULL, BAD_CAST "octave",
                               BAD_CAST ss.str().c_str());

      ss.str("");
      ss << m_trainKeyPoints[i].class_id;
      point_node = xmlNewChild(descriptor_node, NULL, BAD_CAST "class_id",
                               BAD_CAST ss.str().c_str());

      ss.str("");
      ss << ((saveTrainingImages && m_mapOfImageId.size() > 0) ? m_mapOfImageId[m_trainKeyPoints[i].class_id] : -1);
      point_node = xmlNewChild(descriptor_node, NULL, BAD_CAST "image_id",
                               BAD_CAST ss.str().c_str());

      if (have3DInfo) {
        ss.str("");
        ss << m_trainPoints[i].x;
        point_node = xmlNewChild(descriptor_node, NULL, BAD_CAST "oX",
                                 BAD_CAST ss.str().c_str());

        ss.str("");
        ss << m_trainPoints[i].y;
        point_node = xmlNewChild(descriptor_node, NULL, BAD_CAST "oY",
                                 BAD_CAST ss.str().c_str());

        ss.str("");
        ss << m_trainPoints[i].z;
        point_node = xmlNewChild(descriptor_node, NULL, BAD_CAST "oZ",
                                 BAD_CAST ss.str().c_str());
      }

      desc_node = xmlNewChild(descriptor_node, NULL, BAD_CAST "desc", NULL);

      for (int j = 0; j < nCols; j++) {
        ss.str("");
        ss << m_trainDescriptors.at<float>(i, j);
        desc_val_node = xmlNewChild(desc_node, NULL, BAD_CAST "val",
                                    BAD_CAST ss.str().c_str());
      }
    }

    (void)point_node;    // TODO: Fix to avoid compiler warning
    (void)desc_val_node; // TODO: Fix to avoid compiler warning
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

#endif
