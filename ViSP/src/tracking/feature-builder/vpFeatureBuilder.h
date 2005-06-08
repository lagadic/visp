
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpFeatureBuilder.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpFeatureBuilder.h,v 1.1.1.1 2005-06-08 07:08:11 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines conversion between tracker and visual feature
 *
 * ++++++++++++
 */

/*!
  \file vpFeatureBuilder.h
  \brief  class  that defines conversion between tracker and visual feature
*/
// tracker
#include<visp/vpDot.h>
#include<visp/vpMeLine.h>


// forward projection tracker
#include<visp/vpPoint.h>
#include<visp/vpLine.h>
#include<visp/vpSphere.h>
#include<visp/vpCircle.h>
#include<visp/vpCylinder.h>

// visual feature
#include<visp/vpFeaturePoint.h>
#include<visp/vpFeatureLine.h>
#include<visp/vpFeatureEllipse.h>
#include<visp/vpFeaturePoint3D.h>
#include<visp/vpFeatureThetaU.h>
#include<visp/vpFeatureTranslation.h>


//pixel / meter conversion
#include<visp/vpCameraParameters.h>
#include<visp/vpPixelMeterConversion.h>
#include<visp/vpMeterPixelConversion.h>

#ifndef vpFeatureBuilder_H
#define vpFeatureBuilder_H

/*!
  \class vpFeatureBuilder
  \brief  class  that defines conversion between tracker and visual feature
*/
class vpFeatureBuilder
{
public:
  // create vpFeaturePoint feature
  static void create(vpFeaturePoint &s, const vpCameraParameters &cam,
		      const vpDot &t) ;
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
} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
