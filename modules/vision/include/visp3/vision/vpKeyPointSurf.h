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
 * Key point Surf.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

#ifndef vpKeyPointSurf_H
#define vpKeyPointSurf_H

/*!
  \file vpKeyPointSurf.h

  \brief Class that implements the SURF key points and technics thanks
  to the OpenCV library.
*/

#include <visp3/vision/vpBasicKeyPoint.h>

#include <list>
#include <vector>

#if defined(VISP_HAVE_OPENCV_NONFREE) && (VISP_HAVE_OPENCV_VERSION < 0x030000)

#if (VISP_HAVE_OPENCV_VERSION >= 0x020400) // Require opencv >= 1.1.0 < 3.0.0
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/legacy/compat.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#elif (VISP_HAVE_OPENCV_VERSION >= 0x020101) // Require opencv >= 2.1.1
#include <opencv2/features2d/features2d.hpp>
#elif (VISP_HAVE_OPENCV_VERSION >= 0x010100) // Require opencv >= 1.1.0
#include <cv.h>
#include <cxcore.h>
#endif

/*!
  \class vpKeyPointSurf
  \ingroup group_vision_keypoints

  \brief Class that implements the SURF key points and technics thanks
  to OpenCV library.

  \deprecated This class is deprecated with OpenCV 3.0.0 or more recent.
  You should rather use vpKeyPoint class that is more generic.

  The goal of this class is to provide a tool to match points from a
  model and points belonging to an image in which the model appears.
  The coordinates of the different reference points and matched points
  are given in pixel thanks to the class vpImagePoint. In this
  documentation we do not explain the SURF technics. So if you want to
  learn more about it you can refer to the following article :
  Herbert Bay, Tinne Tuytelaars and Luc Van Gool "SURF: Speeded Up
  Robust Features", Proceedings of the 9th European Conference on
  Computer Vision, Springer LNCS volume 3951, part 1, pp 404--417,
  2006.

  If you use this class the first things you have to do is to create
  the reference thanks to a reference image which contains the
  interesting object to detect. Then you have to grab other images
  containing the object. After calling the specific method to match
  points you can access to the lists of matched points thanks to the
  methods getMatchedPointsInReferenceImage() and
  getMatchedPointsInCurrentImage(). These two methods return a list of
  matched points. The nth element of the first list is matched with
  the nth element of the second list. The following small example show
  how to use the class.

  \code
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/vision/vpKeyPointSurf.h>

int main()
{
#if defined (VISP_HAVE_OPENCV_NONFREE) && (VISP_HAVE_OPENCV_VERSION < 0x030000)
  vpImage<unsigned char> Ireference; vpImage<unsigned char> Icurrent;
  vpKeyPointSurf surf;

  // First grab the reference image Ireference

  // Build the reference SURF points.
  surf.buildReference(Ireference);

  // Then grab another image which represents the current image Icurrent

  // Match points between the reference points and the SURF points computed in the current image.
  surf.matchPoint(Icurrent);

  // Display the matched points
  surf.display(Ireference, Icurrent);

  return (0);
#endif
}
  \endcode

  It is also possible to create the reference thanks to only a part of the
  reference image (not the whole image) and find points to match in only a
  part of the current image. The small following example shows how to this

  \code
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/vision/vpKeyPointSurf.h>

int main()
{
#if defined (VISP_HAVE_OPENCV_NONFREE) && (VISP_HAVE_OPENCV_VERSION < 0x030000)
  vpImage<unsigned char> Ireference; vpImage<unsigned char> Icurrent;
  vpKeyPointSurf surf;

  //First grab the reference image Ireference

  //Select a part of the image by clincking on two points which define a rectangle
  vpImagePoint corners[2]; for (int i=0 ; i < 2 ; i++) {
    vpDisplay::getClick(Ireference, corners[i]);
  }

  //Build the reference SURF points.
  int nbrRef;
  unsigned int height, width;
  height = (unsigned int)(corners[1].get_i() - corners[0].get_i());
  width = (unsigned int)(corners[1].get_j() - corners[0].get_j());
  nbrRef = surf.buildReference(Ireference, corners[0], height, width);

  //Then grab another image which represents the current image Icurrent

  //Select a part of the image by clincking on two points which define a rectangle
  for (int i=0 ; i < 2 ; i++) {
    vpDisplay::getClick(Icurrent, corners[i]);
  }

  //Match points between the reference points and the SURF points computed in the current image.
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

class VISP_EXPORT vpKeyPointSurf : public vpBasicKeyPoint
{
public:
  /*!
    This enumerate enables to set the detail level of the
    descriptors.
  */
  typedef enum {
    basicDescriptor,   /*<! basicDescriptor means that the descriptors are
                         composed by 64 elements floating-point vector. */
    extendedDescriptor /*<! Means that the descriptors are composed by
                         128 elements floating-point vector. */
  } vpDescriptorType;

public:
  vpKeyPointSurf();

  virtual ~vpKeyPointSurf();

  unsigned int buildReference(const vpImage<unsigned char> &I);
  unsigned int buildReference(const vpImage<unsigned char> &I, const vpImagePoint &iP, const unsigned int height,
                              const unsigned int width);
  unsigned int buildReference(const vpImage<unsigned char> &I, const vpRect &rectangle);
  unsigned int matchPoint(const vpImage<unsigned char> &I);
  unsigned int matchPoint(const vpImage<unsigned char> &I, const vpImagePoint &iP, const unsigned int height,
                          const unsigned int width);
  unsigned int matchPoint(const vpImage<unsigned char> &I, const vpRect &rectangle);
  void display(const vpImage<unsigned char> &Iref, const vpImage<unsigned char> &Icurrent, unsigned int size = 3);
  void display(const vpImage<unsigned char> &Icurrent, unsigned int size = 3, const vpColor &color = vpColor::green);
  std::list<int *> *matchPoint(std::list<float *> descriptorList, std::list<int> laplacianList);
  float *getDescriptorReferencePoint(const int index);
  int getLaplacianReferencePoint(const int index);
  void getDescriptorParamReferencePoint(const int index, int &size, float &dir);
  /*!

    Sets the value of the hessian threhold.  Note that during the
    computation of the hessian for each potential points, only the
    points which have a hessian value higher than the threshold are
    keeped.  Fore more details about the threshold see the article
    Herbert Bay, Tinne Tuytelaars and Luc Van Gool "SURF: Speeded Up
    Robust Features", Proceedings of the 9th European Conference on
    Computer Vision, Springer LNCS volume 3951, part 1, pp 404--417,
    2006.

    \param hessian_threshold : Desired hessian threshold value.
  */
  void setHessianThreshold(double hessian_threshold)
  {
    this->hessianThreshold = hessian_threshold;
    params = cvSURFParams(this->hessianThreshold, this->descriptorType);
  };

  /*!

    Sets the type of descriptors to use.

    \param descriptor_type : Type of descriptor to use.
  */
  void setDescriptorType(vpDescriptorType descriptor_type)
  {
    this->descriptorType = descriptor_type;
    params = cvSURFParams(this->hessianThreshold, this->descriptorType);
  };

  /*!
    Gets the value of the hessian threhold.

    \return the hessian threshold value.
  */
  double getHessianThreshold() { return hessianThreshold; }

  /*!
    Gets the type of descriptor used.

    \return the type of descriptor used.
  */
  vpDescriptorType getDescriptorType() { return descriptorType; }

private:
  void init();

private:
  // OpenCV Parameters
  CvMemStorage *storage;
  CvSURFParams params;
  CvMemStorage *storage_cur;

  CvSeq *image_keypoints;
  CvSeq *image_descriptors;

  CvSeq *ref_keypoints;
  CvSeq *ref_descriptors;

  /*!
    only features with keypoint.hessian larger than that are extracted.
    Good default value is ~300-500 (can depend on the average
    local contrast and sharpness of the image).
    User can further filter out some features based on their hessian values
    and other characteristics.
  */
  double hessianThreshold;
  vpDescriptorType descriptorType;
};

#endif

#endif
