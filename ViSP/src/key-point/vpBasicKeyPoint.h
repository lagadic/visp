/****************************************************************************
 *
 * $Id: vpBasicFeature.h,v 1.10 2008-02-26 10:32:10 asaunier Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
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
  \brief class that defines what is a Keypoint. This class provides all the basic elements to implement classes which aims to match points from an image to another.
*/


class VISP_EXPORT vpBasicKeyPoint
{
  public:
    vpBasicKeyPoint();

   virtual ~vpBasicKeyPoint() { if (referenceImagePointsList != NULL) delete[] referenceImagePointsList;
                                if (currentImagePointsList != NULL) delete[] currentImagePointsList;
                                matchedPointsCurrentImageList.kill();
                                matchedPointsReferenceImageList.kill();};

   virtual int buildReference(const vpImage<unsigned char> &I) =0;

   virtual int buildReference(const vpImage<unsigned char> &I, vpImagePoint &iP, unsigned int height, unsigned int width) =0;

   virtual int buildReference(const vpImage<unsigned char> &I, const vpRect rectangle) =0;

   virtual int matchPoint(const vpImage<unsigned char> &I) =0;

   virtual int matchPoint(const vpImage<unsigned char> &I, vpImagePoint &iP, unsigned int height, unsigned int width) =0;

   virtual int matchPoint(const vpImage<unsigned char> &I, const vpRect rectangle) =0;

   virtual void display(vpImage<unsigned char> &Iref, vpImage<unsigned char> &Icurrent) =0;

   virtual void display(vpImage<unsigned char> &Icurrent) =0;

   inline vpImagePoint* getAllPointsInReferenceImage() {return referenceImagePointsList;} ;

   inline vpList<vpImagePoint*>* getMatchedPointsInReferenceImage() {return &matchedPointsReferenceImageList;} ;

   inline vpList<vpImagePoint*>* getMatchedPointsInCurrentImage() {return &matchedPointsCurrentImageList;} ;

  private:
    virtual void init()=0;

  protected:
    /*!
      List of the points which define the refrence.
    */
    vpImagePoint* referenceImagePointsList;

    /*!
      List of the points which belong to the current image and have been matched with points belonging to the reference.
    */
    vpImagePoint* currentImagePointsList;

    /*!
      List of pointers that locates the matched points belonging to the reference.
    */
    vpList<vpImagePoint*> matchedPointsCurrentImageList;

    /*!
      List of pointers that locates the matched points belonging to the current image.
    */
    vpList<vpImagePoint*> matchedPointsReferenceImageList;
};

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

