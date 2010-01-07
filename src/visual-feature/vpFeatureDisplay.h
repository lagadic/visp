/****************************************************************************
 *
 * $Id: vpFeatureDisplay.h,v 1.9 2008-09-26 15:21:02 fspindle Exp $
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
 * Interface with the image for feature display.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/


#ifndef vpFeatureDisplay_H
#define vpFeatureDisplay_H

/*!
  \file vpFeatureDisplay.h
  \brief interface with the image for feature display
*/

#include <visp/vpConfig.h>

// Meter/pixel conversion
#include <visp/vpCameraParameters.h>

//Color / image / display
#include <visp/vpColor.h>
#include <visp/vpImage.h>
#include <visp/vpRGBa.h>

/*!
  \class vpFeatureDisplay

  \ingroup VsFeatureDisplay
  \brief Interface with the image for feature display.
*/
class VISP_EXPORT vpFeatureDisplay
{

public:
  static void displayPoint(double x,double y,
			   const vpCameraParameters &cam,
			   vpImage<unsigned char> &I,
			   vpColor color = vpColor::green) ;
  static void displayLine(double rho,double theta,
			  const vpCameraParameters &cam,
			  vpImage<unsigned char> &I,
			  vpColor color = vpColor::green) ;
  static void displayCylinder(double rho1,double theta1,
			      double rho2,double theta2,
			      const vpCameraParameters &cam,
			      vpImage<unsigned char> &I,
			      vpColor color = vpColor::green) ;
  static void displayEllipse(double x,double y,
			     double mu20, double mu11, double m02,
			     const vpCameraParameters &cam,
			     vpImage<unsigned char> &I,
			     vpColor color = vpColor::green) ;

  static void displayPoint(double x,double y,
			   const vpCameraParameters &cam,
			   vpImage<vpRGBa> &I,
			   vpColor color = vpColor::green) ;
  static void displayLine(double rho,double theta,
			  const vpCameraParameters &cam,
			  vpImage<vpRGBa> &I,
			  vpColor color = vpColor::green) ;
  static void displayCylinder(double rho1,double theta1,
			      double rho2,double theta2,
			      const vpCameraParameters &cam,
			      vpImage<vpRGBa> &I,
			      vpColor color = vpColor::green) ;
  static void displayEllipse(double x,double y,
			     double mu20, double mu11, double m02,
			     const vpCameraParameters &cam,
			     vpImage<vpRGBa> &I,
			     vpColor color = vpColor::green) ;


};



#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
