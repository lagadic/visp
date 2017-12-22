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
 * Key point used in matching algorithm.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

#ifndef vpBasicKeyPoint_H
#define vpBasicKeyPoint_H

/*!
  \file vpBasicKeyPoint.h
  \brief class that defines what is a Keypoint
*/

#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpRect.h>

#include <vector>

/*!
  \class vpBasicKeyPoint
  \ingroup group_vision_keypoints

  \brief class that defines what is a Keypoint. This class provides
  all the basic elements to implement classes which aims to match
  points from an image to another.
*/
class VISP_EXPORT vpBasicKeyPoint
{
public:
  vpBasicKeyPoint();

  virtual ~vpBasicKeyPoint()
  {
    matchedReferencePoints.resize(0);
    currentImagePointsList.resize(0);
    referenceImagePointsList.resize(0);
  };

  virtual unsigned int buildReference(const vpImage<unsigned char> &I) = 0;

  virtual unsigned int buildReference(const vpImage<unsigned char> &I, const vpImagePoint &iP,
                                      const unsigned int height, const unsigned int width) = 0;

  virtual unsigned int buildReference(const vpImage<unsigned char> &I, const vpRect &rectangle) = 0;

  virtual unsigned int matchPoint(const vpImage<unsigned char> &I) = 0;

  virtual unsigned int matchPoint(const vpImage<unsigned char> &I, const vpImagePoint &iP, const unsigned int height,
                                  const unsigned int width) = 0;

  virtual unsigned int matchPoint(const vpImage<unsigned char> &I, const vpRect &rectangle) = 0;

  virtual void display(const vpImage<unsigned char> &Iref, const vpImage<unsigned char> &Icurrent,
                       unsigned int size = 3) = 0;

  virtual void display(const vpImage<unsigned char> &Icurrent, unsigned int size = 3,
                       const vpColor &color = vpColor::green) = 0;

  /*!
   Indicate wether the reference has been built or not.

   \return True if the reference of the current instance has been built.
  */
  bool referenceBuilt() const { return _reference_computed; }

  /*!
    Get the pointer to the complete list of reference points. The pointer is
    const. Thus the points can not be modified

    \return The pointer to the complete list of reference points.
  */
  inline const vpImagePoint *getAllPointsInReferenceImage() { return &referenceImagePointsList[0]; };

  /*!
    Get the nth reference point. This point is copied in the vpImagePoint
   instance given in argument.

   \param index : The index of the desired reference point. The index must be
   between 0 and the number of reference points - 1. \param referencePoint :
   The coordinates of the desired reference point are copied there.
  */
  inline void getReferencePoint(const unsigned int index, vpImagePoint &referencePoint)
  {
    if (index >= referenceImagePointsList.size()) {
      vpTRACE("Index of the reference point out of range");
      throw(vpException(vpException::fatalError, "Index of the reference point out of range"));
    }

    referencePoint.set_ij(referenceImagePointsList[index].get_i(), referenceImagePointsList[index].get_j());
  }

  /*!
    Get the nth couple of reference point and current point which have been
   matched. These points are copied in the vpImagePoint instances given in
   argument.

   \param index : The index of the desired couple of reference point and
   current point . The index must be between 0 and the number of matched
   points - 1. \param referencePoint : The coordinates of the desired
   reference point are copied here. \param currentPoint : The coordinates of
   the desired current point are copied here.
  */
  inline void getMatchedPoints(const unsigned int index, vpImagePoint &referencePoint, vpImagePoint &currentPoint)
  {
    if (index >= matchedReferencePoints.size()) {
      vpTRACE("Index of the matched points out of range");
      throw(vpException(vpException::fatalError, "Index of the matched points out of range"));
    }
    referencePoint.set_ij(referenceImagePointsList[matchedReferencePoints[index]].get_i(),
                          referenceImagePointsList[matchedReferencePoints[index]].get_j());
    currentPoint.set_ij(currentImagePointsList[index].get_i(), currentImagePointsList[index].get_j());
  };

  /*!
    Get the nth matched reference point index in the complete list of
   reference point.

    In the code below referencePoint1 and referencePoint2 correspond to the
   same matched reference point.

   \code
   vpKeyPointSurf surf;

   //Here the code to compute the reference points and the current points.

   vpImagePoint referencePoint1;
   vpImagePoint currentPoint;
   surf.getMatchedPoints(1, referencePoint1, currentPoint);  //Get the first matched points

   vpImagePoint referencePoint2;
   const vpImagePoint* referencePointsList = surf.getAllPointsInReferenceImage();
   // Get the first matched reference point index in the complete reference point list
   int index = surf.getIndexInAllReferencePointList(1);
   // Get the first matched reference point
   referencePoint2 = referencePointsList[index];
   \endcode
  */
  inline unsigned int getIndexInAllReferencePointList(const unsigned int indexInMatchedPointList)
  {
    if (indexInMatchedPointList >= matchedReferencePoints.size()) {
      vpTRACE("Index of the matched reference point out of range");
      throw(vpException(vpException::fatalError, "Index of the matched reference point out of range"));
    }
    return matchedReferencePoints[indexInMatchedPointList];
  }

  /*!
   Get the number of reference points.

   \return the number of reference points.
  */
  inline unsigned int getReferencePointNumber() const { return (unsigned int)referenceImagePointsList.size(); };

  /*!
   Get the number of matched points.

   \return the number of matched points.
  */
  inline unsigned int getMatchedPointNumber() const { return (unsigned int)matchedReferencePoints.size(); };

  /*!
    Return the vector of reference image point.

    \warning Should not be modified.

    \return Vector of reference image point.
  */
  const std::vector<vpImagePoint> &getReferenceImagePointsList() const { return referenceImagePointsList; }

  /*!
    Return the vector of current image point.

    \warning Should not be modified.

    \return Vector of the current image point.
  */
  const std::vector<vpImagePoint> &getCurrentImagePointsList() const { return currentImagePointsList; }

  /*!
    Return the index of the matched associated to the current image point i.
    The ith element of the vector is the index of the reference image point
    matching with the current image point.

    \warning Should not be modified.

    \return The vector of matching index.
  */
  const std::vector<unsigned int> &getMatchedReferencePoints() const { return matchedReferencePoints; }

private:
  virtual void init() = 0;

protected:
  /*!
    List of the points which define the reference.
  */
  std::vector<vpImagePoint> referenceImagePointsList;

  /*!
    List of the points which belong to the current image and have
    been matched with points belonging to the reference.
  */
  std::vector<vpImagePoint> currentImagePointsList;

  /*!
    Array containing the index in the array "referenceImagePointsList" of the
    reference points which have been matched.

    The first element of the "currentImagePointsList" array is matched with
    the nth element of the "referenceImagePointsList" array. The value of n is
    stored in the first element of the "matchedReferencePoints" array.
  */
  std::vector<unsigned int> matchedReferencePoints;

  //! flag to indicate if the reference has been built.
  bool _reference_computed;
};

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
