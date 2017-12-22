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
 * Planar surface detection tool.
 *
 * Authors:
 * Romain Tallonneau
 *
 *****************************************************************************/

#ifndef VPPLANAROBJECTDETECTOR_H_
#define VPPLANAROBJECTDETECTOR_H_

#include <visp3/core/vpConfig.h>

#if (VISP_HAVE_OPENCV_VERSION >= 0x020000) &&                                                                          \
    (VISP_HAVE_OPENCV_VERSION < 0x030000) // Require opencv >= 2.0.0 and < 3.0.0

#if (VISP_HAVE_OPENCV_VERSION >= 0x020101) // Require opencv >= 2.1.1
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#elif (VISP_HAVE_OPENCV_VERSION >= 0x020000) // Require opencv >= 2.0.0
#include <cv.h>
#include <cvaux.hpp>
#endif

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpRect.h>
#include <visp3/vision/vpFernClassifier.h>
#include <visp3/vision/vpHomography.h>

/*!
  \class vpPlanarObjectDetector
  \ingroup group_vision_keypoints

  \brief Class used to detect a planar surface.

  \deprecated This class is deprecated with OpenCV 3.0.0 or more recent.

  This class allows to learn and recognise a surface in an image based on the
  Fern Classifier or any other point of interest matching class.

  It uses the class vpFernClassifier to extract points of interest in a
  reference image. These points are recorded and a classifier is trained to
  recognises them.

  In this class the points detected are assumed to belong to a planar surface.
  Therefore an homography can be computed between the reference image and the
  current image if the object is detected in the image.

  A robust method (RANSAC) is used to remove outliers in the matching process.

  The following example shows how to use the class.

  \code
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/vision/vpPlanarObjectDetector.h>

#if VISP_HAVE_OPENCV_VERSION >= 0x020000 // Surf Fern classifier only
available since 2.1.0 int main()
{
  vpImage<unsigned char> Ireference;
  vpImage<unsigned char> Icurrent;
  vpPlanarObjectDetector planar;

  //First grab the reference image Ireference

  //Select a part of the image by clincking on two points which define a rectangle
  vpImagePoint corners[2];
  for (int i=0 ; i < 2 ; i++) {
    vpDisplay::getClick(Ireference, corners[i]);
  }

  //Build the reference points (and train the classifier).
  int nbrRef;
  unsigned int height, width;
  height = (unsigned int)(corners[1].get_i() - corners[0].get_i());
  width = (unsigned int)(corners[1].get_j() - corners[0].get_j());
  nbrRef = planar.buildReference(Ireference, corners[0], height, width);

  //Then grab another image which represents the current image Icurrent

  //Match points between the reference points and the current points computed in the current image.
  bool isDetected; height = (unsigned int)(corners[1].get_i() - corners[0].get_i());
  width = (unsigned int)(corners[1].get_j() - corners[0].get_j());
  isDetected = planar.matchPoint(Icurrent, corners[0], height, width);

  //Display the matched points
  if(isDetected){
    planar.display(Ireference, Icurrent);
    vpHomography homography;
    planar.getHomography(homography);
  }
  else{
    std::cerr << "planar surface not detected in the current image" << std::endl;
  }

  return(0);
}
#else
int main() {}
#endif
  \endcode

*/
class VISP_EXPORT vpPlanarObjectDetector
{
protected:
  //! Fern Classifier used to match the points between a reference image and
  //! the current image.
  vpFernClassifier fern;

  //! Computed homography in the ViSP format.
  vpHomography homography;
  //! Computed homography in the OpenCV format.
  cv::Mat H;

  //! The estimated new coordinates of the corners (reprojected using the
  //! homography).
  std::vector<cv::Point2f> dst_corners;

  //! Flag to indicate wether the last computed homography is correct or not.
  bool isCorrect;

  //! The corners in the reference image
  std::vector<cv::Point2f> ref_corners;

  //! The ROI for the reference image.
  cv::Rect modelROI;

  //! Vector of the image point in the current image that match after the
  //! deletion of the outliers with the RANSAC.
  std::vector<vpImagePoint> currentImagePoints;
  //! Vector of the image point in the reference image that match after the
  //! deletion of the outliers with the RANSAC.
  std::vector<vpImagePoint> refImagePoints;

  //! Minimal number of point to after the ransac needed to suppose that the
  //! homography has been correctly computed.
  unsigned int minNbMatching;

public:
  // constructors and destructors
  vpPlanarObjectDetector();
  vpPlanarObjectDetector(const std::string &dataFile, const std::string &objectName);
  virtual ~vpPlanarObjectDetector();

  // main functions
  // creation of reference
  unsigned int buildReference(const vpImage<unsigned char> &I);
  unsigned int buildReference(const vpImage<unsigned char> &I, const vpImagePoint &iP, unsigned int height,
                              unsigned int width);
  unsigned int buildReference(const vpImage<unsigned char> &I, const vpRect &rectangle);

  // matching
  bool matchPoint(const vpImage<unsigned char> &I);
  bool matchPoint(const vpImage<unsigned char> &I, const vpImagePoint &iP, const unsigned int height,
                  const unsigned int width);
  bool matchPoint(const vpImage<unsigned char> &I, const vpRect &rectangle);
  // database management
  void recordDetector(const std::string &objectName, const std::string &dataFile);
  void load(const std::string &dataFilename, const std::string &objName);

  // display
  void display(vpImage<unsigned char> &I, bool displayKpts = false);
  void display(vpImage<unsigned char> &Iref, vpImage<unsigned char> &Icurrent, bool displayKpts = false);

  /*!
    Return the positions of the detected corners.

    \return a vector of vpImagePoint containing the position of the corners of
    the planar surface in the current image.
  */
  std::vector<vpImagePoint> getDetectedCorners() const;

  /*!
    Return a reference to the classifier.

    \return The fern classifier.
  */
  vpFernClassifier &getFernClassifier() { return this->fern; }

  /*!
    Return the computed homography between the reference image and the current
    image.

    \param _H : The computed homography.
  */
  inline void getHomography(vpHomography &_H) const { _H = this->homography; }

  /*!
    Return the number of reference points

    \return Number of reference points.
  */
  inline unsigned int getNbRefPoints() { return (unsigned int)currentImagePoints.size(); }

  /*!
    Get the i-th reference point.

    \throw vpException if _i is out if bound.

    \param _i : index of the point to get
    \param _imPoint : image point returned by the
  */
  void getReferencePoint(const unsigned int _i, vpImagePoint &_imPoint);

  /*!
    Get the nth couple of reference point and current point which have been
   matched. These points are copied in the vpImagePoint instances given in
   argument.

   \param _index : The index of the desired couple of reference point and
   current point . The index must be between 0 and the number of matched
   points - 1. \param _referencePoint : The coordinates of the desired
   reference point are copied here. \param _currentPoint : The coordinates of
   the desired current point are copied here.
  */
  void getMatchedPoints(const unsigned int _index, vpImagePoint &_referencePoint, vpImagePoint &_currentPoint);

  /*!
    Set the threshold for the minimal number of point to validate the
    homography. Default value is 10.

    \param _min : the new threshold.
  */
  void setMinNbPointValidation(const unsigned int _min) { this->minNbMatching = _min; }

  /*!
    Get the threshold for the minimal number of point to validate the
    homography. Default value is 10.

    \return : the current threshold.
  */
  unsigned int getMinNbPointValidation() const { return this->minNbMatching; }

protected:
  virtual void init();
  void computeRoi(vpImagePoint *ip, const unsigned int nbpt);
  void initialiseRefCorners(const cv::Rect &_modelROI);
};

#endif

#endif /* VPPLANAROBJECTDETECTOR_H_ */
