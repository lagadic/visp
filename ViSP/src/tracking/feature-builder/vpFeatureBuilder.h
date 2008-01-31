/****************************************************************************
 *
 * $Id: vpFeatureBuilder.h,v 1.8 2008-01-31 14:53:55 asaunier Exp $
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
 * Conversion between tracker and visual feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpFeatureBuilder.h
  \brief  class  that defines conversion between tracker and visual feature
*/
#include <visp/vpConfig.h>

// tracker
#include <visp/vpDot.h>
#include <visp/vpDot2.h>
#include <visp/vpMeLine.h>


// forward projection tracker
#include <visp/vpPoint.h>
#include <visp/vpLine.h>
#include <visp/vpSphere.h>
#include <visp/vpCircle.h>
#include <visp/vpCylinder.h>

// visual feature
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureLine.h>
#include <visp/vpFeatureEllipse.h>
#include <visp/vpFeaturePoint3D.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpFeatureTranslation.h>
#include <visp/vpFeatureVanishingPoint.h>

//pixel / meter conversion
#include <visp/vpCameraParameters.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>

#ifndef vpFeatureBuilder_H
#define vpFeatureBuilder_H

/*!
  \class vpFeatureBuilder
  \brief  class  that defines conversion between tracker and visual feature
*/
class VISP_EXPORT vpFeatureBuilder
{
public:
  // create vpFeaturePoint feature
  static void create(vpFeaturePoint &s, const vpCameraParameters &cam,
		      const vpDot &t) ;
  static void create(vpFeaturePoint &s, const vpCameraParameters &cam,
		      const vpDot2 &t) ;
  static void create(vpFeaturePoint &s, const vpPoint &t) ;
  static void create(vpFeaturePoint &s,
		     const vpCameraParameters &goodCam,
		     const vpCameraParameters &wrongCam,
		     const vpPoint &t) ;

  // create vpFeaturePoint3D feature
  static void create(vpFeaturePoint3D &s, const vpPoint &t ) ;

  // create vpFeatureLine feature
  static void create(vpFeatureLine &s, const vpLine &t ) ;
  static void create(vpFeatureLine &s, const vpCylinder &t, const int line) ;

  static  void create(vpFeatureLine &s,
		      const vpCameraParameters &cam,
		      const vpMeLine &t) ;

  //! create vpFeatureEllipse feature
  static void create(vpFeatureEllipse &s, const vpCircle &t) ;
  static void create(vpFeatureEllipse &s, const vpSphere &t) ;
  static void create(vpFeatureEllipse &s,
		     const vpCameraParameters &cam,
		     const vpDot &t ) ;
  static void create(vpFeatureEllipse &s,
		     const vpCameraParameters &cam,
		     const vpDot2 &t ) ;


  /*!
      create vpFeatureVanishingPoint feature from the 2D coordinates of a point
      in the image plane
  */
  static void create(vpFeatureVanishingPoint &s, const vpPoint &t);
  /*!
    create vpFeatureVanishingPoint feature from 2 FeatureLine, ie lines in
    the image plane (error if the 2 lines are parallel)
  */
  static void create(vpFeatureVanishingPoint &s, const vpFeatureLine &L1, const vpFeatureLine &L2 );
  /*!
    create vpFeatureVanishingPoint feature from 2 Lines, (error if the 2
    lines are parallel in the image plane)
  */
  static void create(vpFeatureVanishingPoint &s, const vpLine &L1, const vpLine &L2 );



} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
