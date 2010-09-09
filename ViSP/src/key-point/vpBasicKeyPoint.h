/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.GPL at the root directory of this source
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


#include <visp/vpConfig.h>
#include <visp/vpList.h>
#include <visp/vpImagePoint.h>
#include <visp/vpImage.h>
#include <visp/vpRect.h>


/*!
  \class vpBasicKeyPoint

  \brief class that defines what is a Keypoint. This class provides
  all the basic elements to implement classes which aims to match
  points from an image to another.
*/


class VISP_EXPORT vpBasicKeyPoint
{
  public:
    vpBasicKeyPoint();

   virtual ~vpBasicKeyPoint() { 
     if (referenceImagePointsList != NULL) delete[] referenceImagePointsList;
     if (currentImagePointsList != NULL) delete[] currentImagePointsList;
     if (matchedReferencePoints != NULL) delete[] matchedReferencePoints;
   };

   virtual int buildReference(const vpImage<unsigned char> &I) =0;

   virtual int buildReference(const vpImage<unsigned char> &I, 
			      vpImagePoint &iP, 
			      unsigned int height, unsigned int width) =0;

   virtual int buildReference(const vpImage<unsigned char> &I, 
			      const vpRect rectangle) =0;

   virtual int matchPoint(const vpImage<unsigned char> &I) =0;

   virtual int matchPoint(const vpImage<unsigned char> &I, 
			  vpImagePoint &iP, 
			  unsigned int height, unsigned int width) =0;

   virtual int matchPoint(const vpImage<unsigned char> &I, 
			  const vpRect rectangle) =0;

   virtual void display(vpImage<unsigned char> &Iref, 
			vpImage<unsigned char> &Icurrent) =0;

   virtual void display(vpImage<unsigned char> &Icurrent) =0;

   /*!
     Get the pointer to the complete list of reference points. The pointer is const. Thus the points can not be modified

     \return The pointer to the complete list of reference points.
   */
   inline const vpImagePoint* getAllPointsInReferenceImage() {
     return referenceImagePointsList;
   } ;

   /*!
     Get the nth reference point. This point is copied in the vpImagePoint instance given in argument.

    \param index : The index of the desired reference point. The index must be between 0 and the number of reference points - 1.
    \param referencePoint : The coordinates of the desired reference point are copied there.
   */
   inline void getReferencePoint (const int index, vpImagePoint &referencePoint )
   {
     if (index >= nbReferencePoints || index < 0)
     {
       vpTRACE("Index of the reference point out of range");
       throw(vpException(vpException::fatalError,"Index of the refrence point out of range"));
     }

     referencePoint.set_ij((referenceImagePointsList+index)->get_i(),(referenceImagePointsList+index)->get_j());
   }

   /*!
     Get the nth couple of reference point and current point which have been matched. These points are copied in the vpImagePoint instances given in argument.

    \param index : The index of the desired couple of reference point and current point . The index must be between 0 and the number of matched points - 1.
    \param referencePoint : The coordinates of the desired reference point are copied here.
    \param currentPoint : The coordinates of the desired current point are copied here.
   */
   inline void getMatchedPoints(const int index, vpImagePoint &referencePoint, vpImagePoint &currentPoint) {
     if (index >= nbMatchedPoints || index < 0)
     {
       vpTRACE("Index of the matched points out of range");
       throw(vpException(vpException::fatalError,"Index of the matched points out of range"));
     }
     referencePoint.set_ij(referenceImagePointsList[matchedReferencePoints[index]].get_i(),referenceImagePointsList[matchedReferencePoints[index]].get_j());
     currentPoint.set_ij(currentImagePointsList[index].get_i(), currentImagePointsList[index].get_j());
   };

   /*!
     Get the nth matched reference point index in the complete list of reference point.

     In the code below referencePoint1 and referencePoint2 correspond to the same matched reference point.

    \code
    vpKeyPointSurf surf;

    //Here the code to compute the reference points and the current points.

    vpImagePoint referencePoint1;
    vpImagePoint currentPoint;
    surf.getMatchedPoints(1, referencePoint1, currentPoint);  //Get the first matched points

    vpImagePoint referencePoint2;
    const vpImagePoint* referencePointsList = surf.getAllPointsInReferenceImage();
    int index = surf.getIndexInAllReferencePointList(1);  //Get the first matched reference point index in the complete reference point list 
    referencePoint2 = referencePointsList[index]; //Get the first matched reference point
    \endcode
   */
   inline int getIndexInAllReferencePointList ( const int indexInMatchedPointList ) {
     if (indexInMatchedPointList >= nbMatchedPoints || indexInMatchedPointList < 0)
     {
       vpTRACE("Index of the matched reference point out of range");
       throw(vpException(vpException::fatalError,"Index of the matched reference point out of range"));
     }
     return matchedReferencePoints[indexInMatchedPointList];
   }

   /*!
     Get the number of reference points.

     \return the number of reference points.
   */
   inline int getReferencePointNumber() {return nbReferencePoints;};

   /*!
     Get the number of matched points.

     \return the number of matched points.
   */
   inline int getMatchedPointNumber() {return nbMatchedPoints;};

  private:
    virtual void init()=0;

  protected:
    /*!
      List of the points which define the refrence.
    */
    vpImagePoint* referenceImagePointsList;

    /*!
      List of the points which belong to the current image and have
      been matched with points belonging to the reference.
    */
    vpImagePoint* currentImagePointsList;

    /*!
      Number of refrence points which are computed thanks to the buildReference method.
    */
    int nbReferencePoints;

    /*!
      Number of matched points which are computed thanks to the matchPoint method.
    */
    int nbMatchedPoints;

    /*!
      Array containing the index in the array "referenceImagePointsList" of the reference points which have been matched.

      The first element of the "currentImagePointsList" array is matched with the nth element of the "referenceImagePointsList" array.
      The value of n is stored in the first element of the "matchedReferencePoints" array.
    */
    int* matchedReferencePoints;
};

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

