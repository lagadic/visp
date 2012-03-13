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


/*! 

  Initialise the memory space requested for segment visual
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
  Default constructor that builds a segment visual feature.

  \param P1 : 3D point representing one extremity of the segment
  \param P2 : 3D point representing another extremity of the segment

*/
vpFeatureSegment::vpFeatureSegment(vpPoint& P1,vpPoint& P2):
  vpBasicFeature()  
{ 
  init();
  buildFrom(P1,P2);
}




/*!
  Compute and return the interaction matrix \f$ L \f$ associated to a
  subset of the possible features (\f$ x_c \f$,\f$ y_c \f$,\f$ l \f$,\f$ \alpha \f$).

  The interaction matrix is of the following form:
  \f[
  L = \left[
      \begin{array}{c}
        L_{x_c} \\
        L_{y_c} \\
        L_{l} \\
        L_{\alpha}
      \end{array}
    \right] =
    \left[
      \begin{array}{cccccc}
        -\lambda_2 & 0 & \lambda_2 x_c - \lambda_1 l \frac{cos \alpha}{4} &
        x_c y_c + l^2 \frac{cos \alpha sin \alpha}{4} &
        -(1 + {x_{c}}^{2} + l^2 \frac{cos^2\alpha}{4}) &
        y_c \\
        0 & -\lambda_2 & \lambda_2 y_c - \lambda_1 l \frac{sin \alpha}{4} &
        1 + {y_{c}}^{2} + l^2 \frac{sin^2 \alpha}{4} &
        -x_c y_c-l^2 \frac{cos \alpha sin \alpha}{4} &
        -x_c \\
        \lambda_1 cos \alpha & \lambda_1 sin \alpha &
        \lambda_2 l - \lambda_1 (x_c cos \alpha + y_c sin \alpha) &
        l (x_c cos \alpha sin \alpha + y_c (1 + sin^2 \alpha)) &
        -l (x_c (1 + cos^2 \alpha)+y_c cos \alpha sin \alpha) &
        0 \\
        -\lambda_1  \frac{sin \alpha}{l} & \lambda_1 \frac{cos \alpha}{l} &
        \lambda_1 \frac{x_c sin \alpha - y_c cos \alpha}{l} &
        -x_c sin^2 \alpha + y_c cos \alpha sin \alpha &
        x_c cos \alpha sin \alpha - y_c cos^2 \alpha &
        -1
      \end{array}
     \right]
  \f]

  with \f$ \lambda_1 = \frac{Z_1 - Z_2}{Z_1 Z_2}\f$ and \f$ \lambda_2 = \frac{Z_1 + Z_2}{2 Z_1 Z_2}\f$
  where \f$Z_i\f$ are the depths of the points.


  \param select : Selection of a subset of the possible segment features.
  - To compute the interaction matrix for all the four 
    subset features \f$ x_c \f$,\f$ y_c \f$,\f$ l \f$,\f$ \alpha \f$ use vpBasicFeature::FEATURE_ALL. In
    that case the dimension of the interaction matrix is \f$ [4 \times 6] \f$
  - To compute the interaction matrix for only one of the subset
    (\f$ x_c \f$,\f$ y_c \f$,\f$ l \f$,\f$ \alpha \f$) use one of the corresponding functions:
    selectXc(),selectYc(),selectL(),selectAlpha(). In that case the returned
    interaction matrix is of dimension \f$ [1 \times 6] \f$ .

  \return The interaction matrix computed from the segment features.

  The code below shows how to compute the interaction matrix associated to 
  the visual feature \f$s\f$ =  (\f$ x_c \f$,\f$ y_c \f$,\f$ l \f$,\f$ \alpha \f$).
  \code
  vpPoint p1, p2;  

  p1.setWorldCoordinates(.1, .1, 0.);
  p2.setWorldCoordinates(.3, .2, 0.);
  
  vpFeatureSegment seg(p2, p1);
  
  vpMatrix L = seg.interaction()
  \endcode

  The interaction matrix could also be build by:
  \code
  vpMatrix L = seg.interaction( vpBasicFeature::FEATURE_ALL );
  \endcode

  In both cases, L is a 4 by 6 matrix.

  It is also possible to build the interaction matrix associated to
  one of the possible features. The code below shows how to consider
  only the \f$\alpha\f$ component.

  \code
  vpMatrix L_alpha = s.interaction( vpFeatureSegment::selectAlpha() );
  \endcode

  In that case, L_alpha is a 1 by 6 matrix.
*/
vpMatrix
vpFeatureSegment::interaction( const unsigned int select )
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
      Lalpha[0][1] = lambda1*cos_a/l ;
      Lalpha[0][2] = lambda1*(Xc*sin_a-Yc*cos_a)/l;
      Lalpha[0][3] = -Xc*sin_a*sin_a+Yc*cos_a*sin_a;
      Lalpha[0][4] = Xc*cos_a*sin_a - Yc*cos_a*cos_a ;
      Lalpha[0][5] = -1 ;
    L = vpMatrix::stackMatrices(L,Lalpha) ;
  }

  return L ;
}

/*!
   \brief Compute the error between the current and the desired visual features from a subset of the possible features (\f$ x_c \f$,\f$ y_c \f$,\f$ l \f$,\f$ \alpha \f$).

  For the angular component \f$\alpha\f$, we define the error as
  \f$\alpha \ominus \alpha^*\f$, where \f$\ominus\f$ is modulo \f$2\pi\f$
  substraction.

  \param s_star : Desired 2D segment feature.

  \param select : The error can be computed for a selection of a
  subset of the possible segment features.
  - To compute the error for all the three coordinates use
    vpBasicFeature::FEATURE_ALL. In that case the error vector is a 3 
    dimension column vector.
  - To compute the error for only one subfeature (\f$ x_c \f$,\f$ y_c \f$,\f$ l \f$,\f$ \alpha \f$) use one of the
    corresponding functions: selectXc(),selectYc(),selectL(),selectAlpha(). 

  \return The error between the current and the desired
  visual feature.

*/
vpColVector
vpFeatureSegment::error( const vpBasicFeature &s_star,  const unsigned int select )
{ 
  vpColVector e(0) ;

  if (vpFeatureSegment::selectXc() & select ){
    vpColVector eXc(1) ;
    eXc[0] = Xc-s_star[0];
    e = vpMatrix::stackMatrices(e,eXc) ;	  
  }

  if (vpFeatureSegment::selectYc() & select ){
    vpColVector eYc(1) ;
    eYc[0] = Yc - s_star[1];
    e = vpMatrix::stackMatrices(e,eYc) ;
  }

  if (vpFeatureSegment::selectL() & select ){
    vpColVector eL(1) ;
    eL[0] = l - s_star[2];
    e = vpMatrix::stackMatrices(e,eL) ;
  }

  if (vpFeatureSegment::selectAlpha() & select ){
    vpColVector eAlpha(1) ;
    eAlpha[0] = alpha - s_star[3];
    while (eAlpha[0] < -M_PI) eAlpha[0] += 2*M_PI ;
    while (eAlpha[0] > M_PI) eAlpha[0] -= 2*M_PI ;
    e = vpMatrix::stackMatrices(e,eAlpha) ;
  }
  return e ;
}

/*!
  Print to stdout the values of the current visual feature \f$ s \f$.

  \param select : Selection of a subset of the possible segement features (\f$ x_c \f$,\f$ y_c \f$,\f$ l \f$,\f$ \alpha \f$).

  \code
  vpPoint p1,p2;
  
  p1.setWorldCoordinates(.1,.1,0.);
  p2.setWorldCoordinates(.3,.2,0.);

  p1.setWorldCoordinates(.1,.1,0.);
  p2.setWorldCoordinates(.3,.2,0.);
  
  p1.project(cMo);
  p2.project(cMo);

  p1.project(cdMo);
  p2.project(cdMo);
  
  vpFeatureSegment seg(p2,p1);  
  
  seg.print();
  \endcode

  produces the following output:

  \code
  vpFeatureSegment: (Xc = -0.255634;Yc = -0.13311;l = 0.105005;alpha = 92.1305 deg)
  \endcode
*/
void
vpFeatureSegment::print( const unsigned int select ) const
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

  \param P1, P2 : Two image points defining the segment. These points must contain coordinates (x and y) projected on the camera plane.
  
  The parameters \f$ x_c \f$,\f$ y_c \f$,\f$ l \f$,\f$ \alpha \f$ are
  computed from two two points using the following formulae:
  \f$ x_c = \frac{X_1 + X_2}{2} \f$
  \f$ y_c = \frac{Y_1 + Y_2}{2} \f$
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
