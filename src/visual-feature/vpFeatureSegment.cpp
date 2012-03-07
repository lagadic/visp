/****************************************************************************
 *
 * $Id: vpFeatureThetaU.cpp 3530 2012-01-03 10:52:12Z fspindle $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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
 * Segment visual feature.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/


#include <visp/vpBasicFeature.h>
#include <visp/vpFeatureSegment.h>
#include <visp/vpImagePoint.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpMath.h>
#include <visp/vpDisplay.h>
#include <cmath>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpFeatureException.h>

// Debug trace
#include <visp/vpDebug.h>


/*!
  \file vpFeatureSegment.cpp
  \brief class that defines the vpFeatureSegment visual feature
*/
/*

attributes and members directly related to the vpBasicFeature needs
other functionalities are useful but not mandatory

*/

/*! 

  Initialise the memory space requested for 3D \f$ \theta u \f$ visual
  feature.
*/
void
vpFeatureSegment::init()
{
  //feature dimension
  dim_s = 4 ;
  nbParameters = 4;

  // memory allocation
  s.resize(dim_s) ;
  if (flags == NULL)
    flags = new bool[nbParameters];
  for (unsigned int i = 0; i < nbParameters; i++) flags[i] = false;

  

}

/*! 
  Default constructor that build a visual feature and initialize it to zero.

  \param r [in] : The rotation representation of the \f$ \theta u\f$
  visual feature.

*/
vpFeatureSegment::vpFeatureSegment(vpPoint& P1,vpPoint& P2):
  vpBasicFeature()  
{ 
  init();
  buildFrom(P1,P2);
}




/*!

  Compute and return the interaction matrix 
*/
vpMatrix
vpFeatureSegment::interaction(const unsigned int select)
{

  vpMatrix L ;
  L.resize(0,6) ;

  if (deallocate == vpBasicFeature::user)  
  {
    for (unsigned int i = 0; i < nbParameters; i++)
    {
      if (flags[i] == false)
      {
        switch(i){
        case 0:
          vpTRACE("Warning !!!  The interaction matrix is computed but Xc was not set yet");
        break;
        case 1:
          vpTRACE("Warning !!!  The interaction matrix is computed but Yc was not set yet");
        break;
        case 2:
          vpTRACE("Warning !!!  The interaction matrix is computed but l was not set yet");
        break;
		case 3:
          vpTRACE("Warning !!!  The interaction matrix is computed but alpha was not set yet");
        break;
        default:
          vpTRACE("Problem during the reading of the variable flags");
        }
      }
    }
  }

  //This version is a simplification
  double lambda1 = (z1-z2)/(z1*z2);
  double lambda2 = (z1+z2)/(2*z1*z2);
  

  if (vpFeatureSegment::selectXc() & select ){
      vpMatrix LXc(1,6);
      LXc[0][0] = -lambda2 ;
      LXc[0][1] = 0. ;
      LXc[0][2] = lambda2*Xc - lambda1*l*cos_a/4.;
      LXc[0][3] = Xc*Yc + l*l*cos_a*sin_a/4. ;
      LXc[0][4] = -(1+Xc*Xc+l*l*cos_a*cos_a/4.) ;
      LXc[0][5] = Yc ;
    L = vpMatrix::stackMatrices(L,LXc) ;
  }

  if (vpFeatureSegment::selectYc() & select ){
    vpMatrix LYc(1,6);
      LYc[0][0] = 0. ;
      LYc[0][1] = -lambda2 ;
      LYc[0][2] = lambda2*Yc - lambda1*l*sin_a/4.;
      LYc[0][3] = 1+Yc*Yc+l*l*sin_a*sin_a/4. ;
      LYc[0][4] = -Xc*Yc-l*l*cos_a*sin_a/4. ;
      LYc[0][5] = -Xc ;
    L = vpMatrix::stackMatrices(L,LYc) ;
  }

  if (vpFeatureSegment::selectL() & select ){
    vpMatrix Ll(1,6);
      Ll[0][0] = lambda1*cos_a ;
      Ll[0][1] = lambda1*sin_a ;
      Ll[0][2] = lambda2*l-lambda1*(Xc*cos_a+Yc*sin_a);
      Ll[0][3] = l*(Xc*cos_a*sin_a + Yc*(1+sin_a*sin_a)) ;
      Ll[0][4] = -l*(Xc*(1+cos_a*cos_a)+Yc*cos_a*sin_a) ;
      Ll[0][5] = 0 ;
    L = vpMatrix::stackMatrices(L,Ll) ;
  }

  if (vpFeatureSegment::selectAlpha() & select ){
    vpMatrix Lalpha(1,6);
      Lalpha[0][0] = -lambda1*sin_a/l ;
      Lalpha[0][1] = lambda1*cos_a ;
      Lalpha[0][2] = lambda1*(Xc*sin_a-Yc*cos_a)/l;
      Lalpha[0][3] = -Xc*sin_a*sin_a+Yc*cos_a*sin_a;
      Lalpha[0][4] = Xc*cos_a*sin_a - Yc*cos_a*cos_a ;
      Lalpha[0][5] = -1 ;
    L = vpMatrix::stackMatrices(L,Lalpha) ;
  }

  return L ;
}

/*!
   Compute the error \f$ (s-s^*)\f$ between the current and the desired
  visual features from a subset of the possible features (\f$ X_c \f$,\f$ Y_c \f$,\f$ l \f$,\f$ \alpha \f$).
  
*/
vpColVector
vpFeatureSegment::error(const vpBasicFeature &s_star,  const unsigned int select)
{
  vpColVector e(0) ;

  if (vpFeatureSegment::selectXc() & select ){
	  vpColVector eXc(1) ;
	  eXc[0] = Xc;
      e = vpMatrix::stackMatrices(e,eXc) ;	  
  }

  if (vpFeatureSegment::selectYc() & select ){
      vpColVector eYc(1) ;
	  eYc[0] = Yc;
      e = vpMatrix::stackMatrices(e,eYc) ;
  }

  if (vpFeatureSegment::selectL() & select ){
      vpColVector eL(1) ;
	  eL[0] = l;
      e = vpMatrix::stackMatrices(e,eL) ;
  }

  if (vpFeatureSegment::selectAlpha() & select ){
      vpColVector eAlpha(1) ;
	  eAlpha[0] = alpha;
      e = vpMatrix::stackMatrices(e,eAlpha) ;
  }
  return e ;
}

/*!
  Print to stdout the values of the current visual feature \f$ s \f$.

  \param select : Selection of a subset of the possible segement features (\f$ X_c \f$,\f$ Y_c \f$,\f$ l \f$,\f$ \alpha \f$).
*/
void
vpFeatureSegment::print(const unsigned int select) const
{
  std::cout <<"vpFeatureSegment: (";
  if (vpFeatureSegment::selectXc() & select ) {
    std::cout << "Xc = " << s[0] << ";";
  }
  if (vpFeatureSegment::selectYc() & select ) {
    std::cout << "Yc = " << s[1] << ";";
  }
  if (vpFeatureSegment::selectL() & select ) {
    std::cout << "l = " << s[2] << ";";
  }
  if (vpFeatureSegment::selectAlpha() & select ) {
    std::cout << "alpha = " << vpMath::deg(s[3]) << " deg";
  }
  std::cout << ")" << std::endl;
}

/*!  
  Create an object with the same type.

  \code
  vpBasicFeature *s_star;
  vpFeatureSegment s;
  s_star = s.duplicate(); // s_star is now a vpFeatureSegment
  \endcode

*/
vpFeatureSegment *vpFeatureSegment::duplicate() const
{
  vpFeatureSegment *feature;
  
  feature = new vpFeatureSegment(*this) ;    
  return feature ;
}

/*!

  Displays a segment representing the feature on a greyscale image.
  The two limiting points are displayed in cyan and yellow.

  \param cam : Camera parameters.
  \param I : Image.
  \param color : Color to use for the display
  \param thickness : Thickness of the feature representation.

*/
void
vpFeatureSegment::display(const vpCameraParameters &cam ,
       vpImage<unsigned char> & I ,
       vpColor  color ,
       unsigned int  thickness ) const
{
  double X1 = Xc - (l/2.)*cos_a;
  double X2 = Xc + (l/2.)*cos_a;

  double Y1 = Yc - (l/2.)*sin_a;
  double Y2 = Yc + (l/2.)*sin_a;
  vpImagePoint ip1,ip2;

  vpMeterPixelConversion::convertPoint(cam,X1,Y1,ip1);
  vpMeterPixelConversion::convertPoint(cam,X2,Y2,ip2);
  vpDisplay::displayLine(I, ip1, ip2, color, thickness);
  vpDisplay::displayCircle(I,ip1,5,vpColor::cyan,true);
  vpDisplay::displayCircle(I,ip2,5,vpColor::yellow,true);
}

/*!

  Displays a segment representing the feature on a RGBa image.
  The two limiting points are displayed in cyan and yellow.

  \param cam : Camera parameters.
  \param I : Image.
  \param color : Color to use for the display
  \param thickness : Thickness of the feature representation.
*/
void
vpFeatureSegment::display(const vpCameraParameters & cam ,
      vpImage<vpRGBa> & I ,
      vpColor color ,
      unsigned int thickness ) const
{
  double X1 = Xc - (l/2.)*cos_a;
  double X2 = Xc + (l/2.)*cos_a;

  double Y1 = Yc - (l/2.)*sin_a;
  double Y2 = Yc + (l/2.)*sin_a;
  vpImagePoint ip1,ip2;

  vpMeterPixelConversion::convertPoint(cam,X1,Y1,ip1);
  vpMeterPixelConversion::convertPoint(cam,X2,Y2,ip2);
  vpDisplay::displayLine(I, ip1, ip2, color, thickness);
  vpDisplay::displayCircle(I,ip1,5,vpColor::cyan,true);
  vpDisplay::displayCircle(I,ip2,5,vpColor::yellow,true);
} 

/*!  
  
  Build a segment visual feature from two image points.

  \param P1(\f$X_1,Y_1\f$), P2 : Two image points defining the segment. These points must contain coordinates (x and y) projected on the camera plane.
  
  The parameters \f$ X_c \f$,\f$ Y_c \f$,\f$ l \f$,\f$ \alpha \f$ are
  computed from two two points using the following formulae:
  \f$ X_c = \frac{X_1 + X_2}{2} \f$
  \f$ Y_c = \frac{Y_1 + Y_2}{2} \f$
  \f$ l = \sqrt{{X_1 - X_2}^2 + {Y_1 - Y_2}^2} \f$
  \f$ \alpha = arctan(\frac{Y_1 - Y_2}{X_1 - X_2}) \f$
*/
void vpFeatureSegment::buildFrom(vpPoint& P1,vpPoint& P2){
  Xc = ((P1.get_x()+P2.get_x())/2.);
  Yc = ((P1.get_y()+P2.get_y())/2.);
  z1 = (P1.get_Z());
  z2 = (P2.get_Z());
  l = (sqrt( (P1.get_x()-P2.get_x())*(P1.get_x()-P2.get_x()) + (P1.get_y()-P2.get_y())*(P1.get_y()-P2.get_y())) );
  alpha = atan2(P1.get_y()-P2.get_y(),P1.get_x()-P2.get_x());// + M_PI;
  cos_a = cos(alpha);
  sin_a = sin(alpha);

  s[0] = Xc;
  s[1] = Yc;
  s[2] = l;
  s[3] = alpha;

  flags[0] = flags[1] = flags[2] = flags[3 ] = true;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
