/****************************************************************************
 *
 * $Id$
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
 *
 * Description:
 * Conversion between tracker and visual feature.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpFeatureBuilder_H
#define vpFeatureBuilder_H

/*!
  \file vpFeatureBuilder.h
  \brief  class  that defines conversion between tracker and visual feature
*/
#include <visp3/core/vpConfig.h>

// tracker
#include <visp3/core/vpDot.h>
#include <visp3/core/vpDot2.h>
#include <visp3/core/vpMeLine.h>
#include <visp3/core/vpMeEllipse.h>


// forward projection tracker
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpLine.h>
#include <visp3/core/vpSphere.h>
#include <visp3/core/vpCircle.h>
#include <visp3/core/vpCylinder.h>

// visual feature
#include <visp3/core/vpFeaturePoint.h>
#include <visp3/core/vpFeaturePointPolar.h>
#include <visp3/core/vpFeatureLine.h>
#include <visp3/core/vpFeatureEllipse.h>
#include <visp3/core/vpFeaturePoint3D.h>
#include <visp3/core/vpFeatureThetaU.h>
#include <visp3/core/vpFeatureTranslation.h>
#include <visp3/core/vpFeatureVanishingPoint.h>
#include <visp3/core/vpFeatureSegment.h>

// others
#include <visp3/core/vpImagePoint.h>

//pixel / meter conversion
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpMeterPixelConversion.h>


/*!
  \class vpFeatureBuilder

  \ingroup group_core_visual_features
  \brief Class that defines conversion between trackers and visual features.
*/
class VISP_EXPORT vpFeatureBuilder
{
public:
  // create vpFeaturePoint feature
  static void create(vpFeaturePoint &s, const vpCameraParameters &cam,
                     const vpDot &d) ;
  static void create(vpFeaturePoint &s, const vpCameraParameters &cam,
                     const vpDot2 &d) ;
  static void create(vpFeaturePoint &s, const vpCameraParameters &cam,
                     const vpImagePoint &t) ;
  static void create(vpFeaturePoint &s, const vpPoint &p) ;
  static void create(vpFeaturePoint &s,
                     const vpCameraParameters &goodCam,
                     const vpCameraParameters &wrongCam,
                     const vpPoint &p) ;

  static void create(vpFeatureSegment &s, const vpCameraParameters &cam, const vpDot &d1, const vpDot &d2 ) ;
  static void create(vpFeatureSegment &s, const vpCameraParameters &cam, const vpDot2 &d1, const vpDot2 &d2) ;
  static void create(vpFeatureSegment &s, const vpCameraParameters &cam,
                     const vpImagePoint &ip1, const vpImagePoint &ip2);
  static void create(vpFeatureSegment &s, vpPoint& P1, vpPoint& P2);

  // create vpFeaturePointPolar feature
  static void create(vpFeaturePointPolar &s, 
		     const vpCameraParameters &cam,
		     const vpDot &dot) ;
  static void create(vpFeaturePointPolar &s, 
		     const vpCameraParameters &cam,
		     const vpDot2 &dot) ;
  static void create(vpFeaturePointPolar &s, 
		     const vpCameraParameters &cam,
		     const vpImagePoint &iP) ;
  static void create(vpFeaturePointPolar &s, const vpPoint &p) ;
  static void create(vpFeaturePointPolar &s,
		     const vpCameraParameters &goodCam,
		     const vpCameraParameters &wrongCam,
		     const vpPoint &p) ;

  // create vpFeaturePoint3D feature
  static void create(vpFeaturePoint3D &s, const vpPoint &p ) ;

  // create vpFeatureLine feature
  static void create(vpFeatureLine &s, const vpLine &l ) ;
  static void create(vpFeatureLine &s, const vpCylinder &c, const int line) ;

  static  void create(vpFeatureLine &s,
                      const vpCameraParameters &cam,
                      const vpMeLine &mel) ;

  //! create vpFeatureEllipse feature
  static void create(vpFeatureEllipse &s, const vpCircle &c) ;
  static void create(vpFeatureEllipse &s, const vpSphere &sphere) ;
  static void create(vpFeatureEllipse &s,
		     const vpCameraParameters &cam,
		     const vpDot &d ) ;
  static void create(vpFeatureEllipse &s,
		     const vpCameraParameters &cam,
		     const vpDot2 &d ) ;
  static void create(vpFeatureEllipse &s,
		     const vpCameraParameters &cam,
		     const vpMeEllipse &d ) ;


  /*!
      create vpFeatureVanishingPoint feature from the 2D coordinates of a point
      in the image plane
  */
  static void create(vpFeatureVanishingPoint &s, const vpPoint &p);
  /*!
    create vpFeatureVanishingPoint feature from 2 FeatureLine, ie lines in
    the image plane (error if the 2 lines are parallel)
  */
  static void create(vpFeatureVanishingPoint &s, const vpFeatureLine &l1, const vpFeatureLine &l2 );
  /*!
    create vpFeatureVanishingPoint feature from 2 Lines, (error if the 2
    lines are parallel in the image plane)
  */
  static void create(vpFeatureVanishingPoint &s, const vpLine &l1, const vpLine &l2 );



} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
