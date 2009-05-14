/****************************************************************************
 *
 * $Id: vpFeatureDisplay.cpp,v 1.15 2008-02-26 10:32:10 asaunier Exp $
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
 * Interface with the image for feature display.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp/vpFeatureDisplay.h>

// Meter/pixel conversion
#include <visp/vpMeterPixelConversion.h>

// display
#include <visp/vpDisplay.h>

// Debug trace
#include <visp/vpDebug.h>

// math
#include <visp/vpMath.h>

#include <visp/vpImagePoint.h>



/*!
  \param x,y : Point coordinates in meters.
  \param cam : Camera intrinsic parameters.
  \param I : Image.

  \param color : Color to use to display the feature
*/
void vpFeatureDisplay::displayPoint(double x,double y,
				    const vpCameraParameters &cam,
				    vpImage<unsigned char> &I,
				    vpColor::vpColorType color)
{
  try{
    vpImagePoint ip; // pixel coordinates in float
    vpMeterPixelConversion::convertPoint(cam, x, y, ip) ;

    vpDisplay::displayCross(I, ip, 5, color) ;
  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }

}
/*!
  \param rho, theta : Line parameters.
  \param cam : Camera intrinsic parameters.
  \param I : Image.
  \param color : Color to use to display the feature
*/
void vpFeatureDisplay::displayLine(double rho,double theta,
				   const vpCameraParameters &cam,
				   vpImage<unsigned char> &I,
				   vpColor::vpColorType color )
{


  try{
    //    x cos(theta) + y sin(theta) - rho = 0

    double rhop,thetap ;
    vpMeterPixelConversion::convertLine(cam,rho,theta,rhop,thetap) ;

    //    u cos(thetap) + v sin(thetap) - rhop = 0

    double co = cos(thetap) ;
    double si = sin(thetap) ;
    double c = -rhop ;

    // vpTRACE("rhop %f %f ",rhop, atan2(si,co)) ;
    // double u1,v1,u2,v2 ;

    double a = si ;
    double b = co ;
    vpImagePoint ip1, ip2;

    if (fabs(a) < fabs(b)) {
        ip1.set_ij(0, (-c)/b);
        double h = I.getHeight() - 1;
        ip2.set_ij(h, (-c - a*h)/b);
	vpDisplay::displayLine(I, ip1, ip2, color);
    }
    else {
        ip1.set_ij((-c)/a, 0);
        double w = I.getWidth()-1;
        ip2.set_ij((-c - b*w)/a, w);
	vpDisplay::displayLine(I, ip1, ip2, color);
    }
  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
}
/*!
  \param rho1, theta1 : Cylinder limb1 parameters.
  \param rho2, theta2 : Cylinder limb2 parameters.
  \param cam : Camera intrinsic parameters.
  \param I : Image.

  \param color : Color to use to display the feature
*/
void vpFeatureDisplay::displayCylinder(double rho1,double theta1,
				       double rho2,double theta2,
				       const vpCameraParameters &cam,
				       vpImage<unsigned char> &I,
				       vpColor::vpColorType color)
{
  try
  {
    displayLine(rho1, theta1, cam, I, color) ;
    displayLine(rho2, theta2, cam, I, color) ;
  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
}
/*!
  \param x, y, mu20, mu11, mu02 : Ellipse parameters.
  \param cam : Camera intrinsic parameters.
  \param I : Image.
  \param color : Color to use to display the feature
*/
void vpFeatureDisplay::displayEllipse(double x,double y,
				      double mu20, double mu11, double mu02,
				      const vpCameraParameters &cam,
				      vpImage<unsigned char> &I,
				      vpColor::vpColorType color)
{


  try{
    {
      int number_of_points = 45 ;
      const double incr = 2 * M_PI/(double)number_of_points ; // angle increment
      int i = 0 ;

      //	 std::cout << s.t() ;
      double s = sqrt(vpMath::sqr(mu20-mu02)+4*mu11*mu11) ;
      double e ;

      if (fabs(mu11)<1e-6) e =0 ;
      else e =  (mu02-mu20+s)/(2*mu11) ;
      double a =sqrt( (mu02+mu20+s)/2.0) ;
      double b =sqrt( (mu02+mu20-s)/2.0) ;

      //    vpTRACE("%f %f %f", a,b,e) ;

      double e1  = atan(e) ;

      double k = 0.0 ;

      double ce = cos(e1) ;
      double se = sin(e1) ;

      double x2  = 0;
      double y2 =0;
      vpImagePoint ip1, ip2;

      for( i = 0; i < number_of_points+2 ; i++)
      {
	double    x1 = a *cos(k) ; // equation of an ellipse
	double    y1 = b *sin(k) ; // equation of an ellipse
	double    x11 = x + ce *x1 - se *y1 ;
	double    y11 = y + se *x1 + ce *y1 ;

	vpMeterPixelConversion::convertPoint(cam, x11, y11, ip1);

	if (i > 1) {
	  ip2.set_u( x2 );
	  ip2.set_v( y2 );

	  vpDisplay::displayLine(I, ip1, ip2, color) ;
	}

	ip2 = ip1;
	y2 = y1 ;
	k += incr ;
      } // end for loop
    }
    //    vpDisplay::getClick(I) ;
  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
}

/*!
  \param x, y : Point coordinates in meters.
  \param cam : Camera intrinsic parameters.
  \param I : Image.

  \param color : Color to use to display the feature
*/
void vpFeatureDisplay::displayPoint(double x,double y,
				    const vpCameraParameters &cam,
				    vpImage<vpRGBa> &I,
				    vpColor::vpColorType color)
{
  try{
    vpImagePoint ip; // pixel coordinates in float
    vpMeterPixelConversion::convertPoint(cam, x, y, ip) ;

    vpDisplay::displayCross(I, ip, 5, color) ;
  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }

}

/*!
  \param rho, theta : Line parameters.
  \param cam : Camera intrinsic parameters.
  \param I : Image.
  \param color : Color to use to display the feature
*/
void vpFeatureDisplay::displayLine(double rho,double theta,
				   const vpCameraParameters &cam,
				   vpImage<vpRGBa> &I,
				   vpColor::vpColorType color )
{


  try{
    //    x cos(theta) + y sin(theta) - rho = 0

    double rhop,thetap ;
    vpMeterPixelConversion::convertLine(cam,rho,theta,rhop,thetap) ;

    //    u cos(thetap) + v sin(thetap) - rhop = 0

    double co = cos(thetap) ;
    double si = sin(thetap) ;
    double c = -rhop ;

    // vpTRACE("rhop %f %f ",rhop, atan2(si,co)) ;
    // double u1,v1,u2,v2 ;

    double a = si ;
    double b = co ;
    vpImagePoint ip1, ip2;

    if (fabs(a) < fabs(b)) {
        ip1.set_ij(0, (-c)/b);
        double h = I.getHeight() - 1;
        ip2.set_ij(h, (-c - a*h)/b);
	vpDisplay::displayLine(I, ip1, ip2, color);
    }
    else {
        ip1.set_ij((-c)/a, 0);
        double w = I.getWidth()-1;
        ip2.set_ij((-c - b*w)/a, w);
	vpDisplay::displayLine(I, ip1, ip2, color);
    }
  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
}
/*!
  \param rho1, theta1 : Cylinder limb1 parameters.
  \param rho2, theta2 : Cylinder limb2 parameters.
  \param cam : Camera intrinsic parameters.
  \param I : Image.
  \param color : Color to use to display the feature
*/
void vpFeatureDisplay::displayCylinder(double rho1, double theta1,
				       double rho2, double theta2,
				       const vpCameraParameters &cam,
				       vpImage<vpRGBa> &I,
				       vpColor::vpColorType color)
{
  try
  {
    displayLine(rho1, theta1, cam, I, color) ;
    displayLine(rho2, theta2, cam, I, color) ;
  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
}

/*!
  \param x, y, mu20, mu11, mu02 : Ellipse parameters.
  \param cam : Camera intrinsic parameters.
  \param I : Image.
  \param color : Color to use to display the feature
*/
void vpFeatureDisplay::displayEllipse(double x, double y,
				      double mu20, double mu11, double mu02,
				      const vpCameraParameters &cam,
				      vpImage<vpRGBa> &I,
				      vpColor::vpColorType color)
{


  try{
    {
      int number_of_points = 45 ;
      const double incr = 2 * M_PI/(double)number_of_points ; // angle increment
      int i = 0 ;

      //	 std::cout << s.t() ;
      double s = sqrt(vpMath::sqr(mu20-mu02)+4*mu11*mu11) ;
      double e ;

      if (fabs(mu11)<1e-6) e =0 ;
      else e =  (mu02-mu20+s)/(2*mu11) ;
      double a =sqrt( (mu02+mu20+s)/2.0) ;
      double b =sqrt( (mu02+mu20-s)/2.0) ;

      //    vpTRACE("%f %f %f", a,b,e) ;

      double e1  = atan(e) ;

      double k = 0.0 ;

      double ce = cos(e1) ;
      double se = sin(e1) ;
      double x2  = 0;
      double y2 =0;
      vpImagePoint ip1, ip2;

      for( i = 0; i < number_of_points+2 ; i++)
      {
	double    x1 = a *cos(k) ; // equation of an ellipse
	double    y1 = b *sin(k) ; // equation of an ellipse
	double    x11 = x + ce *x1 - se *y1 ;
	double    y11 = y + se *x1 + ce *y1 ;

	vpMeterPixelConversion::convertPoint(cam, x11, y11, ip1);

	if (i > 1) {
	  ip2.set_u( x2 );
	  ip2.set_v( y2 );

	  vpDisplay::displayLine(I, ip1, ip2, color) ;
	}

	ip2 = ip1;
	k += incr ;
      } // end for loop
    }
    //    vpDisplay::getClick(I) ;
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
