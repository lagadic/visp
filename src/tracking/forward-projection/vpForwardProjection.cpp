/****************************************************************************
 *
 * $Id: vpForwardProjection.cpp,v 1.7 2008-07-18 10:20:46 marchand Exp $
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
 * Forward projection.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#include <visp/vpForwardProjection.h>
#include <visp/vpDebug.h>


/*!
  \file vpForwardProjection.cpp
  \brief   class that defines what is a point
*/





/*!
  Print to stdout the feature parameters in:
  - the object frame
  - the camera frame
  - the image plane.
*/
void
vpForwardProjection::print() const
{
  std::cout << "oP : " << oP.t() ;
  std::cout << "cP : " << cP.t() ;
  std::cout << "p : " << p.t() ;
}

/*!

  Compute the feature parameters in the image plane (vpTracker::p)
  from the parameters in the camera frame (vpTracker::cP).

  \warning Be careful to update vpTracker::p and vpTracker::cP public
  attributes before the call of this method.
*/
void
vpForwardProjection::project()
{
  projection(cP, p) ;
}

/*! 
  
  Compute the feature parameters in the camera frame (vpTracker::cP)
  and than compute the projection of these parameters in the image
  plane (vpTracker::p).

  \warning The feature parameters in the object frame
  (vpForwardProjection:oP) need to be set prior the use of this
  method. To initialize these parameters see setWorldCoordinates().
 
  \param cMo : The homogeneous matrix corresponding to the pose
  between the camera frame and the object frame.

*/
void
vpForwardProjection::project(const vpHomogeneousMatrix &cMo)
{
  try{
    changeFrame(cMo) ;
    projection() ;
  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
}


/*! 
  
  Track the feature parameters in the camera frame (vpTracker::cP)
  and than compute the projection of these parameters in the image
  plane (vpTracker::p).

  This method is similar to project(const vpHomogeneousMatrix &).

  \warning The feature parameters in the object frame
  (vpForwardProjection:oP) need to be set prior the use of this
  method. To initialize these parameters see setWorldCoordinates().
 
  \param cMo : The homogeneous matrix corresponding to the pose
  between the camera frame and the object frame.

*/
void
vpForwardProjection::track(const vpHomogeneousMatrix &cMo)
{
  try{
    project(cMo) ;
  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
