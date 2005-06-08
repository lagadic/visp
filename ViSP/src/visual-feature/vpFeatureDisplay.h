
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpFeatureDisplay.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpFeatureDisplay.h,v 1.1.1.1 2005-06-08 07:08:10 fspindle Exp $
 *
 * Description
 * ============
 *     interface with the image for feature display
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef vpFeatureDisplay_H
#define vpFeatureDisplay_H

/*!
  \file vpFeatureDisplay.h
  \brief interface with the image for feature display
*/


// Meter/pixel conversion
#include <visp/vpCameraParameters.h>

//Color / image / display
#include <visp/vpColor.h>
#include <visp/vpImage.h>

/*!
  \class vpFeatureDisplay
  \brief interface with the image for feature display
*/
class vpFeatureDisplay
{

public:
  static void displayPoint(double x,double y,
			   const vpCameraParameters &cam,
			   vpImage<unsigned char> &I,
			   int color = vpColor::green) ;
  static void displayLine(double rho,double theta,
			  const vpCameraParameters &cam,
			  vpImage<unsigned char> &I,
			  int color = vpColor::green) ;
  static void displayCylinder(double rho1,double theta1,
			      double rho2,double theta2,
			      const vpCameraParameters &cam,
			      vpImage<unsigned char> &I,
			      int color = vpColor::green) ;
  static void displayEllipse(double x,double y,
			     double mu20, double mu11, double m02,
			     const vpCameraParameters &cam,
			     vpImage<unsigned char> &I,
			     int color = vpColor::green) ;


} ;



#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
