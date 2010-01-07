/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
 * Generic tracker.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/



#ifndef vpTracker_H
#define vpTracker_H

/*!
  \file vpTracker.h
  \brief Class that defines what is a generic tracker.
*/

#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpHomogeneousMatrix.h>


/*!
  \class vpTracker
  \brief Class that defines what is a feature generic tracker.

  A tracker is able to track features with parameters expressed in:
  - in the camera frame \e cP. These parameters are located in the public
    attribute vpTracker::cP.
  - in the image plane \e p. These parameters are located in the public
    attribute vpTracker::p. They correspond to normalized coordinates 
    of the feature expressed in meters.

*/
class VISP_EXPORT vpTracker
{


public:
  /*!
    Feature coordinates expressed in the image plane \e p. They correspond
    to 2D normalized coordinates expressed in meters.
  */
  vpColVector p ;
  /*!
    Feature coordinates expressed in the camera frame \e cP. 
  */
  vpColVector cP ;

  /*!
    Flag used to indicate if the feature parameters \e cP expressed
    in the camera frame are available.
  */
  bool cPAvailable ;

public:
  //! Default initialization.
  void init() ;
  //! Default constructor.
  vpTracker() ;

  //! Destructor.
  virtual ~vpTracker() { ; }
} ;


#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
