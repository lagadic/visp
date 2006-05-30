/****************************************************************************
 *
 * $Id: vpTracker.h,v 1.2 2006-05-30 08:40:47 fspindle Exp $
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
  \brief  class that defines what is a generic tracker
*/

#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpHomogeneousMatrix.h>


/*!
  \class vpTracker
  \brief  class that defines what is a generic tracker
*/
class VISP_EXPORT vpTracker
{


public:
  //! 2D feature coordinates
  vpColVector p ;
  //! feature coordinates  expressed in
  //! camera frame
  vpColVector cP ;

  //!
  bool cPAvailable ;


public:
  //! basic construction
  void init() ;
  //! constructor
  vpTracker() ;

  //! destructor
  virtual ~vpTracker() { ; }

public:

  //  virtual void track(vpHomogeneousMatrix &cMo) ;
  //  virtual void track(vpImage<unsigned char>, vpCameraParameters &c) ;
  //  virtual void pixelToMeter(vpCameraParameters &c) ;

} ;


#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
